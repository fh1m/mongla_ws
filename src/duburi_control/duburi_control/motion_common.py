#!/usr/bin/env python3
"""Shared infrastructure for the per-axis motion modules.

`motion_forward.py` and `motion_lateral.py` both use the same thrust
envelope, the same brake-kick / settle, and the same heading-drift
logging. Centralising it here keeps the per-axis files focused on
*which channel they write* and nothing else.

Heading-lock awareness
----------------------
When a ``lock_heading`` is active, the lock thread streams Ch4
rate-overrides continuously. Any RC override that touches Ch4 from a
translation command -- even at the neutral 1500 us value -- would
race against that stream because both writers share the same RC
slot. The ``Writers`` family below exposes two flavours so the
translation modules can stay agnostic of which writer they use:

  release_yaw=False  ->  pixhawk.send_rc_override(...)
                         Ch4 gets 1500 explicitly, ArduSub's internal
                         heading-hold latches at the current heading.
                         Default: every motion command without an
                         active heading lock.

  release_yaw=True   ->  pixhawk.send_rc_translation(...)
                         Ch4 stays released (65535) so the HeadingLock
                         thread is the sole author of Ch4. Used while
                         a heading lock is active.

The Duburi facade picks the flavour based on ``_lock_active()`` and
hands a fully-built ``Writers`` to the per-axis module. The per-axis
module never has to know about lock state.
"""

import time
from dataclasses import dataclass
from typing import Callable

from .pixhawk import Pixhawk


# ---- Shared constants (sourced from motion_rates) ---------------------
# Re-exported under historical names so callers can keep
# ``from .motion_common import THRUST_RATE_HZ`` etc. without a sweep.
from .motion_rates import THRUST_HZ as THRUST_RATE_HZ
from .motion_rates import LOG_THROTTLE_S as LOG_THROTTLE

EASE_SECONDS     = 0.4    # ease-in / ease-out duration (seconds) each side

# Brake / settle (constant variant only — eased uses the ease-out as brake)
REVERSE_KICK_PCT = 25
REVERSE_KICK_SEC = 0.20
SETTLE_SEC       = 1.2


@dataclass
class Writers:
    """Bundle of three RC write functions, lock-state aware.

    `forward(pwm)`, `lateral(pwm)`, `neutral()` -- callers don't care
    whether the underlying pymavlink call is `send_rc_override` or
    `send_rc_translation`.
    """
    forward: Callable[[int], None]
    lateral: Callable[[int], None]
    neutral: Callable[[], None]


def make_writers(pixhawk, release_yaw=False):
    """Build a `Writers` matching the current heading-lock state."""
    if release_yaw:
        return Writers(
            forward=lambda pwm: pixhawk.send_rc_translation(forward=pwm),
            lateral=lambda pwm: pixhawk.send_rc_translation(lateral=pwm),
            neutral=lambda: pixhawk.send_rc_translation(),
        )
    return Writers(
        forward=lambda pwm: pixhawk.send_rc_override(forward=pwm),
        lateral=lambda pwm: pixhawk.send_rc_override(lateral=pwm),
        neutral=pixhawk.send_neutral,
    )


def read_heading(pixhawk, yaw_source):
    """Latest yaw in degrees [0, 360) or None when no fresh sample.

    THE single switchpoint between AHRS (default) and an external
    source (BNO085, future Gazebo). Re-imported by ``motion_yaw`` and
    ``heading_lock`` so every code path that reads heading goes
    through one function -- changing the freshness contract is a
    one-place edit.
    """
    if yaw_source is not None:
        return yaw_source.read_yaw()
    attitude = pixhawk.get_attitude()
    return None if attitude is None else attitude['yaw']


def thrust_loop(pixhawk, axis_writer, duration, gain, log,
                throttle_curve, axis_label, yaw_source=None):
    """Drive ONE channel for `duration` seconds at `gain * curve(t)`.

    `axis_writer(pwm: int)` writes the active axis (Ch5 or Ch6). The
    caller binds it to the right channel + lock-state via
    `Writers.forward` / `Writers.lateral`.

    Heading drift against the entry heading is logged so the operator
    can spot a misbehaving heading-hold (or, when heading-lock is
    active, verify the lock is doing its job).
    """
    locked_heading = read_heading(pixhawk, yaw_source) or 0.0
    worst_drift    = 0.0
    started_at     = time.time()

    while True:
        elapsed = time.time() - started_at
        if elapsed >= duration:
            break

        scale = throttle_curve(elapsed)
        pwm   = Pixhawk.percent_to_pwm(gain * scale)
        axis_writer(pwm)

        heading = read_heading(pixhawk, yaw_source)
        if heading is not None:
            drift       = Pixhawk.heading_error(locked_heading, heading)
            worst_drift = max(worst_drift, abs(drift))
        else:
            drift = 0.0

        depth = pixhawk.get_attitude()
        depth_str = f'{depth["depth"]:+.2f}m' if depth else 'N/A'
        log.info(
            f'[{axis_label:<5}] t={elapsed:.1f}s  drift={drift:+.1f}  '
            f'depth={depth_str}',
            throttle_duration_sec=LOG_THROTTLE)

        time.sleep(1.0 / THRUST_RATE_HZ)

    log.info(f'[{axis_label:<5}] done  worst_drift={worst_drift:.1f}')


def brake_kick_then_settle(axis_writer, writers, brake_pct, log, axis_label,
                           extra_settle=0.0):
    """Constant-variant brake. Reverse-kick the active axis, then settle.

    Constant-gain commands exit at full velocity, so a short reverse
    kick is needed to bleed the momentum before going neutral.
    """
    pwm = Pixhawk.percent_to_pwm(brake_pct)
    log.info(
        f'[{axis_label:<5}] brake -- reverse {abs(brake_pct):.0f}% '
        f'x {REVERSE_KICK_SEC:.2f}s')
    axis_writer(pwm)
    time.sleep(REVERSE_KICK_SEC)

    final_settle(writers, log, extra=extra_settle)


def final_settle(writers, log, extra=0.0):
    """Send neutral (lock-aware) and sleep `SETTLE_SEC + extra`."""
    writers.neutral()
    duration = SETTLE_SEC + extra
    log.info(f'[CMD  ] settle {duration:.1f}s')
    time.sleep(duration)

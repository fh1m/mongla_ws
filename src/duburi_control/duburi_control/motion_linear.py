#!/usr/bin/env python3
"""Linear translation commands.

Two variants with identical signatures:

  drive_constant(pixhawk, direction, duration, gain, log)
      Bang-bang. Constant-gain RC override on Ch5/Ch6 for the full
      duration. Exits with FULL velocity, so a short reverse-kick
      brake is applied before the settle.

  drive_eased(pixhawk, direction, duration, gain, log)
      S-curve. Thrust scale follows `trapezoid_ramp`:
        smootherstep ease-in -> cruise at gain -> smootherstep ease-out.
      The ease-out IS the brake -- residual velocity is small at exit,
      so no reverse kick is applied, only a settle.

Each variant owns its own exit semantics (brake + settle) so the facade
stays a pure dispatcher and the variants compare apples-to-apples at
the command boundary.
"""

import time

from .pixhawk         import Pixhawk
from .motion_profiles import trapezoid_ramp


# ---- Shared constants -------------------------------------------------
LINEAR_RATE_HZ   = 20.0   # RC override publish rate
LOG_THROTTLE     = 0.5    # seconds between [DIR] log lines

# ---- Eased-only tunable -----------------------------------------------
EASE_SECONDS     = 0.4    # ease-in / ease-out duration (seconds) each side

# ---- Brake / settle tunables (CONSTANT variant only) -----------------
REVERSE_KICK_PCT = 25     # gentle -- higher values push the sub backward
REVERSE_KICK_SEC = 0.20
SETTLE_SEC       = 1.2

# Translation gain sign per direction.  (forward-axis, lateral-axis)
DIRECTION_SIGNS = {
    'forward': (+1,  0),
    'back':    (-1,  0),
    'left':    (0, -1),
    'right':   (0, +1),
}

# Which channel/sign to push during the reverse kick.
BRAKE_AXIS = {
    'forward': ('forward', -REVERSE_KICK_PCT),
    'back':    ('forward', +REVERSE_KICK_PCT),
    'left':    ('lateral', +REVERSE_KICK_PCT),
    'right':   ('lateral', -REVERSE_KICK_PCT),
}


# ---------------------------------------------------------------------- #
#  Public variants                                                         #
# ---------------------------------------------------------------------- #
def drive_constant(pixhawk, direction, duration, gain, log):
    """Constant gain + aggressive reverse-kick brake -- original behaviour."""
    thrust_loop(pixhawk, direction, duration, gain, log,
                throttle_curve=lambda _elapsed: 1.0)
    brake_kick_then_settle(pixhawk, direction, log)


def drive_eased(pixhawk, direction, duration, gain, log):
    """Smootherstep-shaped ease-in + cruise + ease-out (ease-out is the brake)."""
    thrust_loop(pixhawk, direction, duration, gain, log,
                throttle_curve=lambda elapsed:
                    trapezoid_ramp(elapsed, duration, EASE_SECONDS))
    brake_settle_only(pixhawk, log)


# ---------------------------------------------------------------------- #
#  Shared loop                                                             #
# ---------------------------------------------------------------------- #
def thrust_loop(pixhawk, direction, duration, gain, log, throttle_curve):
    """Drive the chosen direction for `duration` seconds.

    `throttle_curve(elapsed)` returns a [0, 1] scale applied on top of
    `gain` -- constant-1.0 for bang-bang, smootherstep-trapezoid for
    eased. Heading drift against the entry heading is logged so the
    operator can spot a misbehaving heading-hold.
    """
    forward_sign, lateral_sign = DIRECTION_SIGNS[direction]

    attitude       = pixhawk.get_attitude()
    locked_heading = attitude['yaw'] if attitude else 0.0
    worst_drift    = 0.0
    started_at     = time.time()
    short_label    = direction[:3].upper()

    while True:
        elapsed = time.time() - started_at
        if elapsed >= duration:
            break

        attitude = pixhawk.get_attitude()
        if attitude is not None:
            drift       = Pixhawk.heading_error(locked_heading, attitude['yaw'])
            worst_drift = max(worst_drift, abs(drift))
        else:
            drift = 0.0

        scale       = throttle_curve(elapsed)
        forward_pwm = Pixhawk.percent_to_pwm(forward_sign * gain * scale)
        lateral_pwm = Pixhawk.percent_to_pwm(lateral_sign * gain * scale)
        pixhawk.send_rc_override(forward=forward_pwm, lateral=lateral_pwm)

        depth_str = f'{attitude["depth"]:+.2f}m' if attitude else 'N/A'
        log.info(
            f'[{short_label:<5}] '
            f't={elapsed:.1f}s  drift={drift:+.1f}  depth={depth_str}',
            throttle_duration_sec=LOG_THROTTLE)

        time.sleep(1.0 / LINEAR_RATE_HZ)

    log.info(f'[{short_label:<5}] done  worst_drift={worst_drift:.1f}')


# ---------------------------------------------------------------------- #
#  Exit semantics (brake + settle) -- owned by each variant               #
# ---------------------------------------------------------------------- #
def brake_kick_then_settle(pixhawk, direction, log):
    """Constant-variant brake. Exits with full velocity, so we apply a
    short reverse kick on the primary axis before settling."""
    axis, percent = BRAKE_AXIS[direction]
    pwm           = Pixhawk.percent_to_pwm(percent)
    log.info(
        f'[CMD  ] brake -- reverse {axis} {REVERSE_KICK_PCT}% '
        f'x {REVERSE_KICK_SEC:.2f}s')
    pixhawk.send_rc_override(**{axis: pwm})
    time.sleep(REVERSE_KICK_SEC)

    pixhawk.send_neutral()
    log.info(f'[CMD  ] settle {SETTLE_SEC:.1f}s')
    time.sleep(SETTLE_SEC)


def brake_settle_only(pixhawk, log):
    """Eased-variant brake. The trapezoid ease-out already decelerated
    us, so any reverse kick would over-brake and push the sub backward.
    Just go neutral and let drag + heading-hold do the rest."""
    pixhawk.send_neutral()
    log.info(f'[CMD  ] settle {SETTLE_SEC:.1f}s (ease-out = brake)')
    time.sleep(SETTLE_SEC)

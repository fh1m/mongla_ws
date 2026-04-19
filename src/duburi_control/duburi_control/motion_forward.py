#!/usr/bin/env python3
"""Forward-axis translation (Ch5) and curved arc motion (Ch5 + Ch4).

Three public functions, all using `Writers` from `motion_common`:

  drive_forward_constant(pixhawk, signed_dir, duration, gain, log,
                         writers, yaw_source=None, settle=0.0)
      Bang-bang. Constant-gain RC override on Ch5 for the full
      duration. Exits with full velocity, so a reverse-kick brake is
      applied before the settle.

  drive_forward_eased(pixhawk, signed_dir, duration, gain, log,
                      writers, yaw_source=None, settle=0.0)
      S-curve via `trapezoid_ramp`: smootherstep ease-in -> cruise at
      gain -> smootherstep ease-out. The ease-out IS the brake -- only
      the settle phase runs at exit, no reverse kick.

  arc(pixhawk, signed_dir, duration, gain, yaw_rate_pct, log,
      yaw_source=None, settle=0.0)
      Curved car-style motion. Single 20 Hz loop writes Ch5 (forward
      thrust) AND Ch4 (yaw rate) in the same packet. Ch4 is signed
      yaw stick (+ = right turn, - = left). Heading-lock is
      incompatible by design -- the Duburi facade suspends any active
      lock around `arc` and re-engages at the exit heading.

`signed_dir` is +1 for forward, -1 for back. The per-axis split makes
the call sites unambiguous: `move_forward` -> `drive_forward_*(+1, ...)`,
`move_back` -> `drive_forward_*(-1, ...)`.
"""

import time

from .pixhawk         import Pixhawk
from .motion_profiles import trapezoid_ramp
from .motion_common   import (
    THRUST_RATE_HZ, LOG_THROTTLE, EASE_SECONDS, REVERSE_KICK_PCT,
    thrust_loop, brake_kick_then_settle, final_settle, read_heading,
)


# ---------------------------------------------------------------------- #
#  Forward / back -- bang-bang                                            #
# ---------------------------------------------------------------------- #
def drive_forward_constant(pixhawk, signed_dir, duration, gain, log,
                           writers, yaw_source=None, settle=0.0):
    """Constant gain on Ch5, reverse-kick brake, then settle."""
    label = 'FWD' if signed_dir > 0 else 'BACK'
    axis_writer = writers.forward
    signed_gain = signed_dir * gain

    thrust_loop(pixhawk, axis_writer, duration, signed_gain, log,
                throttle_curve=lambda _t: 1.0,
                axis_label=label, yaw_source=yaw_source)

    brake_kick_then_settle(
        axis_writer, writers,
        brake_pct=-signed_dir * REVERSE_KICK_PCT,
        log=log, axis_label=label, extra_settle=settle)


def drive_forward_eased(pixhawk, signed_dir, duration, gain, log,
                        writers, yaw_source=None, settle=0.0):
    """Smootherstep envelope on Ch5, settle only (ease-out IS the brake)."""
    label = 'FWD' if signed_dir > 0 else 'BACK'
    axis_writer = writers.forward
    signed_gain = signed_dir * gain

    thrust_loop(pixhawk, axis_writer, duration, signed_gain, log,
                throttle_curve=lambda elapsed:
                    trapezoid_ramp(elapsed, duration, EASE_SECONDS),
                axis_label=label, yaw_source=yaw_source)

    log.info(f'[{label:<5}] settle (ease-out = brake)')
    final_settle(writers, log, extra=settle)


# ---------------------------------------------------------------------- #
#  arc -- forward thrust + yaw rate in the same packet                    #
# ---------------------------------------------------------------------- #
def arc(pixhawk, signed_dir, duration, gain, yaw_rate_pct, log,
        yaw_source=None, settle=0.0):
    """Drive Ch5 + Ch4 simultaneously for a curved car-style trajectory.

    `signed_dir` controls forward/back ({+1, -1}); `gain` the magnitude
    of forward thrust [%]. `yaw_rate_pct` is the signed yaw stick
    [-100..+100], positive = right turn.

    Heading-lock is incompatible: `arc` intentionally changes heading.
    The Duburi facade suspends an active lock around the arc and
    re-engages at the exit heading. `arc` itself always uses
    `send_rc_override` so Ch4 is sent explicitly.
    """
    label    = 'ARC'
    fwd_pct  = signed_dir * gain
    yaw_pct  = yaw_rate_pct

    started_at     = time.time()
    locked_heading = read_heading(pixhawk, yaw_source) or 0.0
    last_heading   = locked_heading

    while True:
        elapsed = time.time() - started_at
        if elapsed >= duration:
            break

        fwd_pwm = Pixhawk.percent_to_pwm(fwd_pct)
        yaw_pwm = Pixhawk.percent_to_pwm(yaw_pct)
        pixhawk.send_rc_override(forward=fwd_pwm, yaw=yaw_pwm)

        heading = read_heading(pixhawk, yaw_source)
        if heading is not None:
            last_heading = heading

        depth = pixhawk.get_attitude()
        depth_str = f'{depth["depth"]:+.2f}m' if depth else 'N/A'
        log.info(
            f'[{label:<5}] t={elapsed:.1f}s  fwd={fwd_pct:+.0f}%  '
            f'yaw={yaw_pct:+.0f}%  hdg={last_heading:.1f}  depth={depth_str}',
            throttle_duration_sec=LOG_THROTTLE)

        time.sleep(1.0 / THRUST_RATE_HZ)

    swept = Pixhawk.heading_error(last_heading, locked_heading)
    log.info(
        f'[{label:<5}] done  start={locked_heading:.1f}  '
        f'end={last_heading:.1f}  swept={swept:+.1f}')

    # Always neutral after arc -- Ch5 + Ch4 both released-then-held at 1500.
    pixhawk.send_neutral()
    log.info(f'[{label:<5}] settle {settle:.1f}s + brake')
    time.sleep(max(0.6, settle))

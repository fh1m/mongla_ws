#!/usr/bin/env python3
"""Lateral-axis (strafe) translation -- Ch6 only.

Two public functions, mirror images of `motion_forward`:

  drive_lateral_constant(pixhawk, signed_dir, duration, gain, log,
                         writers, yaw_source=None, settle=0.0)
      Bang-bang. Constant-gain RC override on Ch6 for the full
      duration. Reverse-kick brake then settle.

  drive_lateral_eased(pixhawk, signed_dir, duration, gain, log,
                      writers, yaw_source=None, settle=0.0)
      Smootherstep envelope, settle only.

`signed_dir` is +1 for right strafe, -1 for left. `move_left` and
`move_right` on `Duburi` are the only public callers.
"""

import time

from .pixhawk        import Pixhawk
from .motion_easing  import trapezoid_ramp
from .motion_writers import (
    EASE_SECONDS, LOG_THROTTLE, REVERSE_KICK_PCT,
    thrust_loop, brake_kick_then_settle, final_settle,
)

_DVL_POLL_HZ   = 20
_DVL_TIMEOUT_K = 10.0


def drive_lateral_constant(pixhawk, signed_dir, duration, gain, log,
                           writers, yaw_source=None, settle=0.0):
    """Constant gain on Ch6, reverse-kick brake, then settle."""
    label = 'RIGHT' if signed_dir > 0 else 'LEFT'
    axis_writer = writers.lateral
    signed_gain = signed_dir * gain

    thrust_loop(pixhawk, axis_writer, duration, signed_gain, log,
                throttle_curve=lambda _t: 1.0,
                axis_label=label, yaw_source=yaw_source)

    brake_kick_then_settle(
        axis_writer, writers,
        brake_pct=-signed_dir * REVERSE_KICK_PCT,
        log=log, axis_label=label, extra_settle=settle)


def drive_lateral_eased(pixhawk, signed_dir, duration, gain, log,
                        writers, yaw_source=None, settle=0.0):
    """Smootherstep envelope on Ch6, settle only (ease-out IS the brake)."""
    label = 'RIGHT' if signed_dir > 0 else 'LEFT'
    axis_writer = writers.lateral
    signed_gain = signed_dir * gain

    thrust_loop(pixhawk, axis_writer, duration, signed_gain, log,
                throttle_curve=lambda elapsed:
                    trapezoid_ramp(elapsed, duration, EASE_SECONDS),
                axis_label=label, yaw_source=yaw_source)

    log.info(f'[{label:<5}] settle (ease-out = brake)')
    final_settle(writers, log, extra=settle)


# ---------------------------------------------------------------------- #
#  DVL closed-loop lateral distance                                       #
# ---------------------------------------------------------------------- #

def drive_lateral_dist(pixhawk, signed_dir, distance_m, gain, tolerance,
                       log, writers, yaw_source=None, settle=0.0):
    """Strafe a fixed distance using DVL position feedback.

    signed_dir: +1 = right, -1 = left
    distance_m: absolute distance in metres (always positive)
    gain:       thrust percentage (0-100)
    tolerance:  stop when |error| <= tolerance metres (typical: 0.1)
    """
    label      = 'RT_D' if signed_dir > 0 else 'LT_D'
    target_m   = abs(distance_m)
    signed_gain = signed_dir * gain

    has_dvl = (yaw_source is not None
               and hasattr(yaw_source, 'get_position')
               and hasattr(yaw_source, 'reset_position'))

    if not has_dvl:
        log.info(f'[{label}] no DVL position source -- open-loop fallback '
                 f'(rough ~{target_m:.1f}m estimate)')
        rough_s = max(1.0, target_m / 0.2)
        drive_lateral_constant(pixhawk, signed_dir, rough_s, gain, log,
                               writers, yaw_source=yaw_source, settle=settle)
        return

    yaw_source.reset_position()  # type: ignore[union-attr]
    pwm      = Pixhawk.percent_to_pwm(signed_gain)
    deadline = time.monotonic() + target_m / 0.05 + _DVL_TIMEOUT_K
    interval = 1.0 / _DVL_POLL_HZ

    log.info(f'[{label}] DVL dist {target_m:.2f}m  gain={gain:.0f}%  '
             f'tol={tolerance:.3f}m')

    while time.monotonic() < deadline:
        _, y_m  = yaw_source.get_position()  # type: ignore[union-attr]
        error   = target_m - abs(y_m)

        if abs(error) <= tolerance:
            log.info(f'[{label}] reached  y={y_m:.3f}m  err={error:+.3f}m')
            break

        writers.lateral(pwm)
        log.info(f'[{label}] y={y_m:.3f}m  err={error:+.3f}m',
                 throttle_duration_sec=LOG_THROTTLE)
        time.sleep(interval)
    else:
        log.info(f'[{label}] timeout  target={target_m:.2f}m')

    writers.neutral()
    if settle > 0.0:
        time.sleep(settle)

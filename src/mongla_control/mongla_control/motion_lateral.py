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
`move_right` on `Mongla` are the only public callers.
"""

import time

from .pixhawk        import Pixhawk
from .motion_easing  import trapezoid_ramp
from .motion_writers import (
    EASE_SECONDS, LOG_THROTTLE, REVERSE_KICK_PCT,
    thrust_loop, brake_kick_then_settle, final_settle,
    _interruptible_sleep,
)
from .errors        import MovementError, MovementTimeout

_DVL_POLL_HZ   = 20
_DVL_TIMEOUT_K = 10.0
# See drive_forward_dist for why these exist.
_OVERSHOOT_K = 1.5
_OVERSHOOT_PAD = 0.5
# WALL-CLOCK, while the DVL integrates distance in SIM time. Gazebo runs
# well below real time (RTF ~0.65 measured, lower under load), so a wall
# second buys well under a second of travel -- a 6 s window false-tripped
# move_back_dist and move_lateral_dist, whose thrust is weaker than
# forward. Generous on purpose: the OVERSHOOT guard is what actually
# bounds a runaway, this one only catches a DVL that is dead on arrival.
_STALL_S = 15.0
_STALL_M = 0.05


def drive_lateral_constant(pixhawk, signed_dir, duration, gain, log,
                           writers, yaw_source=None, settle=0.0,
                           abort_fn=None):
    """Constant gain on Ch6, reverse-kick brake, then settle."""
    label = 'RIGHT' if signed_dir > 0 else 'LEFT'
    axis_writer = writers.lateral
    signed_gain = signed_dir * gain

    thrust_loop(pixhawk, axis_writer, duration, signed_gain, log,
                throttle_curve=lambda _t: 1.0,
                axis_label=label, yaw_source=yaw_source, abort_fn=abort_fn)

    brake_kick_then_settle(
        axis_writer, writers,
        brake_pct=-signed_dir * REVERSE_KICK_PCT,
        log=log, axis_label=label, extra_settle=settle, abort_fn=abort_fn)


def drive_lateral_eased(pixhawk, signed_dir, duration, gain, log,
                        writers, yaw_source=None, settle=0.0,
                        abort_fn=None):
    """Smootherstep envelope on Ch6, settle only (ease-out IS the brake)."""
    label = 'RIGHT' if signed_dir > 0 else 'LEFT'
    axis_writer = writers.lateral
    signed_gain = signed_dir * gain

    thrust_loop(pixhawk, axis_writer, duration, signed_gain, log,
                throttle_curve=lambda elapsed:
                    trapezoid_ramp(elapsed, duration, EASE_SECONDS),
                axis_label=label, yaw_source=yaw_source, abort_fn=abort_fn)

    log.info(f'[{label:<5}] settle (ease-out = brake)')
    final_settle(writers, log, extra=settle, abort_fn=abort_fn)


# ---------------------------------------------------------------------- #
#  DVL closed-loop lateral distance                                       #
# ---------------------------------------------------------------------- #

def drive_lateral_dist(pixhawk, signed_dir, distance_m, gain, tolerance,
                       log, writers, yaw_source=None, settle=0.0,
                       abort_fn=None):
    """Strafe a fixed distance using DVL position feedback.

    RAISES without a DVL position source -- see drive_forward_dist for why the
    old open-loop fallback was removed. Returns the measured |y| travelled.

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
        src = getattr(yaw_source, 'name', type(yaw_source).__name__)
        raise MovementError(
            f'{label}: no DVL position source (yaw_source={src!r} provides no '
            f'get_position/reset_position), so {target_m:.2f} m cannot be '
            f'measured. Use yaw_source=dvl or bno085_dvl on the vehicle, or '
            f'sim_dvl in Gazebo. For an explicitly timed move use move_left/right.')

    yaw_source.reset_position()  # type: ignore[union-attr]
    pwm      = Pixhawk.percent_to_pwm(signed_gain)
    deadline = time.monotonic() + target_m / 0.05 + _DVL_TIMEOUT_K
    interval = 1.0 / _DVL_POLL_HZ

    log.info(f'[{label}] DVL dist {target_m:.2f}m  gain={gain:.0f}%  '
             f'tol={tolerance:.3f}m')

    # Same runaway guards as drive_forward_dist: the deadline bounds time, not
    # distance, so a DVL reading zero means full thrust for the whole budget.
    overshoot_limit = target_m * _OVERSHOOT_K + _OVERSHOOT_PAD
    stall_deadline = time.monotonic() + _STALL_S

    while time.monotonic() < deadline:
        if abort_fn and abort_fn():
            break
        _, y_m  = yaw_source.get_position()  # type: ignore[union-attr]
        error   = target_m - abs(y_m)

        if abs(y_m) > overshoot_limit:
            writers.neutral()
            raise MovementError(
                f'{label}: overshoot guard -- DVL measured {abs(y_m):.2f}m for a '
                f'{target_m:.2f}m command. Stopping rather than driving on.')
        if time.monotonic() > stall_deadline and abs(y_m) < _STALL_M:
            writers.neutral()
            raise MovementError(
                f'{label}: no progress -- DVL still reads {abs(y_m):.3f}m after '
                f'{_STALL_S:.0f}s at {gain:.0f}% thrust. Refusing to keep driving blind.')

        if abs(error) <= tolerance:
            log.info(f'[{label}] reached  y={y_m:.3f}m  err={error:+.3f}m')
            break

        writers.lateral(pwm)
        log.info(f'[{label}] y={y_m:.3f}m  err={error:+.3f}m',
                 throttle_duration_sec=LOG_THROTTLE)
        time.sleep(interval)
    else:
        # Frozen-but-connected DVL: same silent-success as the missing-DVL case.
        writers.neutral()
        _, y_m = yaw_source.get_position()  # type: ignore[union-attr]
        raise MovementTimeout(
            f'{label}: timeout after {target_m / 0.05 + _DVL_TIMEOUT_K:.0f}s -- '
            f'target={target_m:.2f}m, DVL measured only {abs(y_m):.3f}m. '
            f'Check the DVL has bottom lock.')

    # Brake before the final read -- see drive_forward_dist.
    brake_kick_then_settle(
        writers.lateral, writers, -signed_dir * REVERSE_KICK_PCT, log, label,
        extra_settle=settle, abort_fn=abort_fn)
    _, y_m = yaw_source.get_position()  # type: ignore[union-attr]
    return abs(y_m)

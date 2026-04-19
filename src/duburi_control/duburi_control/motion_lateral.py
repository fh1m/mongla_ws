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

from .motion_profiles import trapezoid_ramp
from .motion_common   import (
    EASE_SECONDS, REVERSE_KICK_PCT,
    thrust_loop, brake_kick_then_settle, final_settle,
)


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

#!/usr/bin/env python3
"""Forward-axis translation (Ch5) and curved arc motion (Ch5 + Ch4).

Three public functions, all using `Writers` from `motion_writers`:

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

  arc(pixhawk, signed_dir, duration, gain, target_yaw, log,
      yaw_source=None, settle=0.0)
      Curved motion to an ABSOLUTE heading. A single YAW_RATE_HZ loop writes
      Ch5 (forward thrust) AND Ch4 (a _YawPID closing on `target_yaw`) in the
      same packet, for the full `duration` -- the hull curves onto the target
      heading then drives straight. Turn direction is auto-computed from the
      shortest-path error. Heading-lock is incompatible by design -- the Duburi
      facade suspends any active lock around `arc` and re-engages at the exit
      heading (the ACTUAL measured heading, not necessarily `target_yaw`).

`signed_dir` is +1 for forward, -1 for back. The per-axis split makes
the call sites unambiguous: `move_forward` -> `drive_forward_*(+1, ...)`,
`move_back` -> `drive_forward_*(-1, ...)`.
"""

import time

from .pixhawk         import Pixhawk
from .motion_easing  import trapezoid_ramp
from .motion_writers import (
    LOG_THROTTLE, EASE_SECONDS, REVERSE_KICK_PCT,
    thrust_loop, brake_kick_then_settle, final_settle, read_heading,
    _interruptible_sleep,
)
# arc closes Ch4 on an ABSOLUTE heading -- reuse the proven turn controller
# (same _YawPID + rate as motion_yaw so its I/D terms stay in calibration).
from .motion_yaw      import _YawPID, YAW_RATE_HZ

_DVL_POLL_HZ   = 20     # position polling rate for distance moves
_DVL_TIMEOUT_K = 10.0   # generous extra timeout: metres / 0.05 + this


# ---------------------------------------------------------------------- #
#  Forward / back -- bang-bang                                            #
# ---------------------------------------------------------------------- #
def drive_forward_constant(pixhawk, signed_dir, duration, gain, log,
                           writers, yaw_source=None, settle=0.0,
                           abort_fn=None):
    """Constant gain on Ch5, reverse-kick brake, then settle."""
    label = 'FWD' if signed_dir > 0 else 'BACK'
    axis_writer = writers.forward
    signed_gain = signed_dir * gain

    thrust_loop(pixhawk, axis_writer, duration, signed_gain, log,
                throttle_curve=lambda _t: 1.0,
                axis_label=label, yaw_source=yaw_source, abort_fn=abort_fn)

    brake_kick_then_settle(
        axis_writer, writers,
        brake_pct=-signed_dir * REVERSE_KICK_PCT,
        log=log, axis_label=label, extra_settle=settle, abort_fn=abort_fn)


def drive_forward_eased(pixhawk, signed_dir, duration, gain, log,
                        writers, yaw_source=None, settle=0.0,
                        abort_fn=None):
    """Smootherstep envelope on Ch5, settle only (ease-out IS the brake)."""
    label = 'FWD' if signed_dir > 0 else 'BACK'
    axis_writer = writers.forward
    signed_gain = signed_dir * gain

    thrust_loop(pixhawk, axis_writer, duration, signed_gain, log,
                throttle_curve=lambda elapsed:
                    trapezoid_ramp(elapsed, duration, EASE_SECONDS),
                axis_label=label, yaw_source=yaw_source, abort_fn=abort_fn)

    log.info(f'[{label:<5}] settle (ease-out = brake)')
    final_settle(writers, log, extra=settle, abort_fn=abort_fn)


# ---------------------------------------------------------------------- #
#  arc -- forward thrust + yaw rate in the same packet                    #
# ---------------------------------------------------------------------- #
def arc(pixhawk, signed_dir, duration, gain, target_yaw, log,
        yaw_source=None, settle=0.0, abort_fn=None):
    """Drive forward while turning to (and holding) an ABSOLUTE heading.

    Ch5 forward at `gain`% (`signed_dir` = +1/-1 for fwd/back) for the full
    `duration`, while a `_YawPID` closes Ch4 to bring the heading to
    `target_yaw` (absolute degrees) and hold it -- the trajectory curves onto
    the heading then straightens. `duration` sets how long / far the hull
    travels; `target_yaw` the heading it ends on. The turn direction is
    AUTO-COMPUTED from the shortest-path heading error -- there is no yaw-rate
    stick to set (that is the whole point of the target-heading arc).

    Heading-lock is incompatible (arc changes heading): the Duburi facade
    suspends an active lock around this and, on exit, retargets the lock to the
    ACTUAL measured heading (not `target_yaw`, in case the arc didn't fully
    reach it within `duration`). The loop runs at `YAW_RATE_HZ` so `_YawPID`'s
    I/D terms stay in the same calibration as a stationary turn. Ch5 + Ch4 go
    in one `send_rc_override` packet.
    """
    label    = 'ARC'
    fwd_pct  = signed_dir * gain
    pid      = _YawPID()

    started_at     = time.monotonic()
    start_heading  = read_heading(pixhawk, yaw_source) or 0.0
    last_heading   = start_heading
    fwd_pwm        = Pixhawk.percent_to_pwm(fwd_pct)   # constant forward thrust

    try:
        while True:
            elapsed = time.monotonic() - started_at
            if elapsed >= duration:
                break
            if abort_fn and abort_fn():
                break

            heading = read_heading(pixhawk, yaw_source)
            if heading is not None:
                last_heading = heading
            # Shortest-path error to the ABSOLUTE target; _YawPID maps it to a
            # signed Ch4 % (0 inside YAW_TOL_DEG -> Ch4 neutral -> drives straight
            # once the heading is held). Same controller as a stationary turn.
            error   = Pixhawk.heading_error(target_yaw, last_heading)
            yaw_pct = pid.update(error)
            yaw_pwm = Pixhawk.percent_to_pwm(yaw_pct)
            pixhawk.send_rc_override(forward=fwd_pwm, yaw=yaw_pwm)

            depth = pixhawk.get_attitude()
            depth_str = f'{depth["depth"]:+.2f}m' if depth else 'N/A'
            log.info(
                f'[{label:<5}] t={elapsed:.1f}s  fwd={fwd_pct:+.0f}%  '
                f'->{target_yaw:.0f}deg  hdg={last_heading:.1f}  '
                f'err={error:+.1f}  yaw={yaw_pct:+.0f}%  depth={depth_str}',
                throttle_duration_sec=LOG_THROTTLE)

            time.sleep(1.0 / YAW_RATE_HZ)
    finally:
        pixhawk.send_neutral()

    swept = Pixhawk.heading_error(last_heading, start_heading)
    log.info(
        f'[{label:<5}] done  start={start_heading:.1f}  target={target_yaw:.1f}  '
        f'end={last_heading:.1f}  swept={swept:+.1f}')

    # The `finally` above already left Ch5+Ch4 neutral; just settle. Interruptible
    # so a cancel/safety-verb during the settle window returns promptly instead of
    # waiting out the full sleep (the hull is already at neutral meanwhile).
    log.info(f'[{label:<5}] settle {settle:.1f}s + brake')
    _interruptible_sleep(max(0.6, settle), abort_fn)


# ---------------------------------------------------------------------- #
#  DVL closed-loop forward distance                                       #
# ---------------------------------------------------------------------- #

def drive_forward_dist(pixhawk, signed_dir, distance_m, gain, tolerance,
                       log, writers, yaw_source=None, settle=0.0,
                       abort_fn=None):
    """Drive forward (or back) a fixed distance using DVL position feedback.

    Requires `yaw_source` to implement `get_position()` and
    `reset_position()` (i.e. NucleusDVLSource). Falls back to an
    open-loop timed estimate when DVL position is not available.

    signed_dir: +1 = forward, -1 = back
    distance_m: absolute distance in metres (always positive; direction from signed_dir)
    gain:       thrust percentage (0-100)
    tolerance:  stop when |error| <= tolerance metres (typical: 0.1)
    """
    label      = 'FWD_D' if signed_dir > 0 else 'BACK_D'
    target_m   = abs(distance_m)
    signed_gain = signed_dir * gain

    has_dvl = (yaw_source is not None
               and hasattr(yaw_source, 'get_position')
               and hasattr(yaw_source, 'reset_position'))

    if not has_dvl:
        # Open-loop TIME estimate at a HARDCODED 0.3 m/s -- only valid for the
        # current thruster tune. WARN (not info) so a silently-wrong distance
        # after a retune / DVL dropout is visible on the console.
        log.warning(f'[{label}] no DVL position source -- OPEN-LOOP fallback at '
                    f'~0.3 m/s (distance is a rough time estimate, not measured)')
        rough_s = max(1.0, target_m / 0.3)
        drive_forward_constant(pixhawk, signed_dir, rough_s, gain, log,
                               writers, yaw_source=yaw_source, settle=settle,
                               abort_fn=abort_fn)
        return

    yaw_source.reset_position()
    pwm       = Pixhawk.percent_to_pwm(signed_gain)
    deadline  = time.monotonic() + target_m / 0.05 + _DVL_TIMEOUT_K
    interval  = 1.0 / _DVL_POLL_HZ

    log.info(f'[{label}] DVL dist {target_m:.2f}m  gain={gain:.0f}%  '
             f'tol={tolerance:.3f}m')

    while time.monotonic() < deadline:
        if abort_fn and abort_fn():
            break
        x_m, _ = yaw_source.get_position()
        error   = target_m - abs(x_m)

        if abs(error) <= tolerance:
            log.info(f'[{label}] reached  x={x_m:.3f}m  err={error:+.3f}m')
            break

        writers.forward(pwm)
        log.info(f'[{label}] x={x_m:.3f}m  err={error:+.3f}m',
                 throttle_duration_sec=LOG_THROTTLE)
        time.sleep(interval)
    else:
        log.info(f'[{label}] timeout  target={target_m:.2f}m')

    writers.neutral()
    if settle > 0.0:
        _interruptible_sleep(settle, abort_fn)

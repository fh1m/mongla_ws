#!/usr/bin/env python3
"""Yaw command implementations.

Two variants with identical signatures:

  yaw_snap(pixhawk, start_heading, end_heading, timeout, label, log, yaw_source=None)
      Bang-bang. Send the final target as a step setpoint at 10 Hz and
      let ArduSub's internal 400 Hz attitude stabiliser drive the
      rotation profile. Fast, proven, matches every successful mission
      log to date.

  yaw_glide(pixhawk, start_heading, end_heading, timeout, label, log, yaw_source=None)
      Smoothed. Two phases:
        1. Stream a smootherstep-interpolated setpoint from `start` to
           `end` over `duration` seconds (derived from angle size).
        2. Hold the final target until heading is locked for N frames.
      Slower but no overshoot — the setpoint's own rate decays to zero
      at the target, so the sub cannot accelerate past it.

Both release Ch4 (via `send_rc_translation`) so SET_ATTITUDE_TARGET
keeps authority over yaw. Both use the same tolerance / lock-frame
exit so callers see identical success/timeout semantics. On success:
return `None` silently. On timeout: raise `MovementTimeout` (caught by
the action server, surfaced as `Move.Result.success = False`).

Yaw source selection
--------------------
`yaw_source` is an optional `duburi_sensors.YawSource`. When None
(default), both functions read the current heading from
`pixhawk.get_attitude()['yaw']`. When provided (e.g. BNO085Source),
`read_yaw()` is called instead and the AHRS is bypassed for the error
computation. If the source returns None for a tick (stale sample,
parse error), the previous valid heading is held — never block, never
crash.

Note: ArduSub's *internal* attitude stabiliser still runs off its own
EKF yaw — we only swap the source of the *error* we measure against
the target.
"""

import time

from .errors          import MovementTimeout
from .pixhawk         import Pixhawk
from .motion_profiles import smootherstep


# ---- Shared constants (apply to both variants) -----------------------
YAW_RATE_HZ   = 10.0   # SET_ATTITUDE_TARGET stream rate (ArduSub needs >= 1 Hz)
YAW_TOL_DEG   = 2.0    # ArduSub's attitude stabiliser accuracy
YAW_LOCK_N    = 5      # consecutive frames within tol before success
LOG_THROTTLE  = 0.5    # seconds between [YAW] log lines

# ---- Glide-only tunables ---------------------------------------------
YAW_AVG_DPS   = 30.0   # average deg/s across a glided turn
# 90 deg -> 3.0 s, 45 deg -> 1.5 s, 180 deg -> 6.0 s.
# Peak rate = avg x 1.875 (smootherstep peak derivative).
YAW_MIN_DUR   = 1.5    # lower bound so small turns still get a glide


def read_heading(pixhawk, yaw_source):
    """Latest yaw in degrees [0, 360), or None when no fresh sample is
    available. Single point that swaps AHRS for an external source.
    """
    if yaw_source is not None:
        return yaw_source.read_yaw()
    attitude = pixhawk.get_attitude()
    return None if attitude is None else attitude['yaw']


# ---------------------------------------------------------------------- #
#  yaw_snap — bang-bang: ship the target, let ArduSub draw the curve     #
# ---------------------------------------------------------------------- #
def yaw_snap(pixhawk, start_heading, end_heading,
             timeout, label, log, yaw_source=None):
    turn_degrees = Pixhawk.heading_error(end_heading, start_heading)
    log.info(
        f'[CMD  ] yaw_{label.lower()}  {abs(turn_degrees):.0f} deg  '
        f'cur={start_heading:.1f}  tgt={end_heading:.1f}  (SNAP)')

    pixhawk.send_rc_translation()

    deadline       = time.time() + timeout
    frames_locked  = 0
    peak_error_deg = 0.0
    current        = start_heading

    while time.time() < deadline:
        pixhawk.set_attitude_setpoint(yaw_deg=end_heading)
        pixhawk.send_rc_translation()

        heading = read_heading(pixhawk, yaw_source)
        if heading is None:
            time.sleep(0.1)
            continue
        current        = heading
        error          = Pixhawk.heading_error(end_heading, current)
        peak_error_deg = max(peak_error_deg, abs(error))

        log.info(
            f'[YAW  ] {label}  tgt:{end_heading:.1f}  cur:{current:.1f}  '
            f'err:{error:+.1f}',
            throttle_duration_sec=LOG_THROTTLE)

        if abs(error) <= YAW_TOL_DEG:
            frames_locked += 1
            if frames_locked >= YAW_LOCK_N:
                log.info(
                    f'[YAW  ] OK {label} locked at {current:.1f}  '
                    f'(peak err {peak_error_deg:.1f})')
                return
        else:
            frames_locked = 0

        time.sleep(1.0 / YAW_RATE_HZ)

    error = Pixhawk.heading_error(end_heading, current)
    log.info(
        f'[YAW  ] !! {label} timeout  cur:{current:.1f}  tgt:{end_heading:.1f}  '
        f'err:{error:+.1f}')
    raise MovementTimeout(
        f'yaw_{label.lower()} timeout after {timeout:.1f}s -- '
        f'cur={current:.1f} tgt={end_heading:.1f} err={error:+.1f}')


# ---------------------------------------------------------------------- #
#  yaw_glide — smoothed setpoint sweep, ease-out IS the brake             #
# ---------------------------------------------------------------------- #
def yaw_glide(pixhawk, start_heading, end_heading,
              timeout, label, log, yaw_source=None):
    turn_degrees = Pixhawk.heading_error(end_heading, start_heading)
    duration     = max(YAW_MIN_DUR, abs(turn_degrees) / YAW_AVG_DPS)

    log.info(
        f'[CMD  ] yaw_{label.lower()}  {abs(turn_degrees):.0f} deg  '
        f'cur={start_heading:.1f}  tgt={end_heading:.1f}  '
        f'(GLIDE {duration:.1f}s)')

    pixhawk.send_rc_translation()

    # ---- Phase 1: stream a smootherstep-swept setpoint ----------------
    started_at = time.time()
    current    = start_heading

    while True:
        elapsed = time.time() - started_at
        if elapsed >= duration:
            break

        fraction = elapsed / duration
        swept    = (start_heading + turn_degrees * smootherstep(fraction)) % 360

        pixhawk.set_attitude_setpoint(yaw_deg=swept)
        pixhawk.send_rc_translation()

        heading = read_heading(pixhawk, yaw_source)
        if heading is not None:
            current = heading

        log.info(
            f'[YAW  ] {label}  sweep:{swept:6.1f}  cur:{current:6.1f}  '
            f'({fraction*100:3.0f}%)',
            throttle_duration_sec=LOG_THROTTLE)

        time.sleep(1.0 / YAW_RATE_HZ)

    # ---- Phase 2: lock at the final target ----------------------------
    deadline       = time.time() + timeout
    frames_locked  = 0
    peak_error_deg = 0.0

    while time.time() < deadline:
        pixhawk.set_attitude_setpoint(yaw_deg=end_heading)
        pixhawk.send_rc_translation()

        heading = read_heading(pixhawk, yaw_source)
        if heading is None:
            time.sleep(0.1)
            continue
        current        = heading
        error          = Pixhawk.heading_error(end_heading, current)
        peak_error_deg = max(peak_error_deg, abs(error))

        log.info(
            f'[YAW  ] {label}  lock  cur:{current:.1f}  err:{error:+.1f}',
            throttle_duration_sec=LOG_THROTTLE)

        if abs(error) <= YAW_TOL_DEG:
            frames_locked += 1
            if frames_locked >= YAW_LOCK_N:
                log.info(
                    f'[YAW  ] OK {label} locked at {current:.1f}  '
                    f'(peak err {peak_error_deg:.1f})')
                return
        else:
            frames_locked = 0

        time.sleep(1.0 / YAW_RATE_HZ)

    error = Pixhawk.heading_error(end_heading, current)
    log.info(
        f'[YAW  ] !! {label} timeout  cur:{current:.1f}  tgt:{end_heading:.1f}  '
        f'err:{error:+.1f}')
    raise MovementTimeout(
        f'yaw_{label.lower()} timeout after {timeout:.1f}s -- '
        f'cur={current:.1f} tgt={end_heading:.1f} err={error:+.1f}')

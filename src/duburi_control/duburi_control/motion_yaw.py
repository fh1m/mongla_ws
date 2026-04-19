#!/usr/bin/env python3
"""Yaw command implementations.

Two variants with identical signatures:

  yaw_snap(pixhawk, start_heading, end_heading, timeout, label, log, yaw_source=None)
      Bang-bang. Closed rate loop on Ch4: ``yaw_pct = kp * err`` each
      tick (error measured against ``yaw_source``). When the source
      reads the target, Ch4 -> 1500 and the vehicle stops. Fast.

  yaw_glide(pixhawk, start_heading, end_heading, timeout, label, log, yaw_source=None)
      Smoothed. Same rate loop but the setpoint is swept with
      smootherstep from ``start`` to ``end`` over ``duration`` seconds
      before locking at ``end``. No overshoot because the sweep's own
      rate decays to zero at the target.

Why rate-based and not absolute-attitude
----------------------------------------
ArduSub's absolute-attitude controller closes the physical loop on
ArduSub's *own* compass/AHRS. On the real vehicle that compass is
corrupted by thruster ESCs (the whole reason a BNO085 exists on the
AUV). On Gazebo SITL it is the sim compass, which has no coupling to
the external ``yaw_source`` the operator picked. Both variants below
instead write Ch4 directly via ``send_rc_override(yaw=...)``: ArduSub
treats any Ch4 override != 1500 as a pilot yaw-rate command and stops
using its own compass to close the heading loop. The BNO (or whichever
source is configured) is now the sole feedback for both the rate
command and the termination check. Identical pattern to the one
``motion_vision`` uses for vision-driven yaw.

Both use the same tolerance / lock-frame exit so callers see identical
success/timeout semantics. On success: return ``None`` silently. On
timeout: raise ``MovementTimeout`` (caught by the action server,
surfaced as ``Move.Result.success = False``).

Yaw source selection
--------------------
``yaw_source`` is an optional ``duburi_sensors.YawSource``. When None
(default), both functions read the current heading from
``pixhawk.get_attitude()['yaw']``. When provided (e.g. BNO085Source),
``read_yaw()`` is called and the AHRS is bypassed. If the source
returns None for a tick (stale sample, parse error), we hold the last
valid heading; if it stays silent longer than ``STALE_HOLD_S`` we
park Ch4 at 1500 (safe stop) until the source recovers.
"""

import time

from .errors          import MovementTimeout
from .motion_common   import read_heading
from .pixhawk         import Pixhawk
from .motion_profiles import smootherstep


# ---- Shared constants (apply to both variants) -----------------------
YAW_RATE_HZ   = 10.0   # Ch4 rate-override publish rate
YAW_TOL_DEG   = 2.0    # heading tolerance for "locked"
YAW_LOCK_N    = 5      # consecutive frames within tol before success
LOG_THROTTLE  = 0.5    # seconds between [YAW  ] log lines

# ---- Rate-loop tunables (motion_vision-style proportional control) ---
YAW_KP_PCT_PER_DEG = 1.0     # %Ch4 per degree error (90 deg -> 90%, clamped)
YAW_PCT_MAX        = 55.0    # cap on |yaw_pct| so we never saturate the bus
STALE_HOLD_S       = 0.5     # if yaw_source goes silent longer than this,
                             # park Ch4 at 1500 instead of guessing

# ---- Glide-only tunables ---------------------------------------------
YAW_AVG_DPS   = 30.0   # average deg/s across a glided turn
# 90 deg -> 3.0 s, 45 deg -> 1.5 s, 180 deg -> 6.0 s.
# Peak rate = avg x 1.875 (smootherstep peak derivative).
YAW_MIN_DUR   = 1.5    # lower bound so small turns still get a glide


def _yaw_rate_pct(error_deg: float) -> float:
    """Proportional yaw rate (percent of Ch4 authority) from a heading
    error in degrees. Deadbands inside YAW_TOL_DEG so a "locked" sub
    does not twitch on sensor noise, and clamps to +/-YAW_PCT_MAX so
    large errors can not saturate the bus.
    """
    if abs(error_deg) <= YAW_TOL_DEG:
        return 0.0
    rate = error_deg * YAW_KP_PCT_PER_DEG
    if rate >  YAW_PCT_MAX: return  YAW_PCT_MAX
    if rate < -YAW_PCT_MAX: return -YAW_PCT_MAX
    return rate


def _send_yaw_pct(pixhawk, yaw_pct: float) -> None:
    """Write a single Ch4-rate-override frame. Ch4 = 1500 means 'zero
    pilot yaw rate' (ArduSub holds heading). Any other value is treated
    by ArduSub as a pilot yaw-rate command, bypassing its internal
    compass-driven heading hold -- which is the whole point of this
    rewrite.
    """
    pixhawk.send_rc_override(yaw=Pixhawk.percent_to_pwm(yaw_pct))


# ---------------------------------------------------------------------- #
#  yaw_snap -- Ch4 rate loop, yaw_source drives motion AND termination  #
# ---------------------------------------------------------------------- #
def yaw_snap(pixhawk, start_heading, end_heading,
             timeout, label, log, yaw_source=None):
    turn_degrees = Pixhawk.heading_error(end_heading, start_heading)
    log.info(
        f'[CMD  ] yaw_{label.lower()}  {abs(turn_degrees):.0f} deg  '
        f'cur={start_heading:.1f}  tgt={end_heading:.1f}  (SNAP)')

    deadline       = time.time() + timeout
    frames_locked  = 0
    peak_error_deg = 0.0
    current        = start_heading
    last_good_mono = time.monotonic()

    while time.time() < deadline:
        heading = read_heading(pixhawk, yaw_source)
        now_mono = time.monotonic()

        if heading is None:
            if (now_mono - last_good_mono) > STALE_HOLD_S:
                _send_yaw_pct(pixhawk, 0.0)   # safe stop on dead source
            time.sleep(0.1)
            continue
        current        = heading
        last_good_mono = now_mono

        error          = Pixhawk.heading_error(end_heading, current)
        peak_error_deg = max(peak_error_deg, abs(error))

        yaw_pct = _yaw_rate_pct(error)
        _send_yaw_pct(pixhawk, yaw_pct)

        log.info(
            f'[YAW  ] {label}  tgt:{end_heading:.1f}  cur:{current:.1f}  '
            f'err:{error:+.1f}  pct:{yaw_pct:+5.1f}',
            throttle_duration_sec=LOG_THROTTLE)

        if abs(error) <= YAW_TOL_DEG:
            frames_locked += 1
            if frames_locked >= YAW_LOCK_N:
                _send_yaw_pct(pixhawk, 0.0)
                log.info(
                    f'[YAW  ] OK {label} locked at {current:.1f}  '
                    f'(peak err {peak_error_deg:.1f})')
                return
        else:
            frames_locked = 0

        time.sleep(1.0 / YAW_RATE_HZ)

    _send_yaw_pct(pixhawk, 0.0)
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

    # ---- Phase 1: rate-loop closes on a smootherstep-swept target ----
    started_at     = time.time()
    current        = start_heading
    last_good_mono = time.monotonic()

    while True:
        elapsed = time.time() - started_at
        if elapsed >= duration:
            break

        fraction = elapsed / duration
        swept    = (start_heading + turn_degrees * smootherstep(fraction)) % 360

        heading = read_heading(pixhawk, yaw_source)
        now_mono = time.monotonic()

        if heading is None:
            if (now_mono - last_good_mono) > STALE_HOLD_S:
                _send_yaw_pct(pixhawk, 0.0)
            time.sleep(0.1)
            continue
        current        = heading
        last_good_mono = now_mono

        error   = Pixhawk.heading_error(swept, current)
        yaw_pct = _yaw_rate_pct(error)
        _send_yaw_pct(pixhawk, yaw_pct)

        log.info(
            f'[YAW  ] {label}  sweep:{swept:6.1f}  cur:{current:6.1f}  '
            f'err:{error:+5.1f}  pct:{yaw_pct:+5.1f}  ({fraction*100:3.0f}%)',
            throttle_duration_sec=LOG_THROTTLE)

        time.sleep(1.0 / YAW_RATE_HZ)

    # ---- Phase 2: lock at the final target ----------------------------
    deadline       = time.time() + timeout
    frames_locked  = 0
    peak_error_deg = 0.0

    while time.time() < deadline:
        heading = read_heading(pixhawk, yaw_source)
        now_mono = time.monotonic()

        if heading is None:
            if (now_mono - last_good_mono) > STALE_HOLD_S:
                _send_yaw_pct(pixhawk, 0.0)
            time.sleep(0.1)
            continue
        current        = heading
        last_good_mono = now_mono

        error          = Pixhawk.heading_error(end_heading, current)
        peak_error_deg = max(peak_error_deg, abs(error))

        yaw_pct = _yaw_rate_pct(error)
        _send_yaw_pct(pixhawk, yaw_pct)

        log.info(
            f'[YAW  ] {label}  lock  cur:{current:.1f}  err:{error:+.1f}  '
            f'pct:{yaw_pct:+5.1f}',
            throttle_duration_sec=LOG_THROTTLE)

        if abs(error) <= YAW_TOL_DEG:
            frames_locked += 1
            if frames_locked >= YAW_LOCK_N:
                _send_yaw_pct(pixhawk, 0.0)
                log.info(
                    f'[YAW  ] OK {label} locked at {current:.1f}  '
                    f'(peak err {peak_error_deg:.1f})')
                return
        else:
            frames_locked = 0

        time.sleep(1.0 / YAW_RATE_HZ)

    _send_yaw_pct(pixhawk, 0.0)
    error = Pixhawk.heading_error(end_heading, current)
    log.info(
        f'[YAW  ] !! {label} timeout  cur:{current:.1f}  tgt:{end_heading:.1f}  '
        f'err:{error:+.1f}')
    raise MovementTimeout(
        f'yaw_{label.lower()} timeout after {timeout:.1f}s -- '
        f'cur={current:.1f} tgt={end_heading:.1f} err={error:+.1f}')

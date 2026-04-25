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

import math
import time

from .errors          import MovementTimeout
from .motion_writers  import read_heading
from .pixhawk         import Pixhawk
from .motion_easing  import smootherstep


# ---- Shared constants (apply to both variants) -----------------------
# Rates / throttles come from motion_rates so we change them in one
# place; see motion_rates.py for sizing rationale.
from .motion_rates import YAW_RATE_HZ            # noqa: F401  (re-export)
from .motion_rates import LOG_THROTTLE_S as LOG_THROTTLE

YAW_TOL_DEG   = 1.0    # heading tolerance for "locked" (matches proven 1° floor)
YAW_LOCK_N    = 5      # consecutive frames within tol before success

# ---- Rate-loop tunables (min-speed clamped, matching sample_codebase) -------
# T200 thrusters need ~20 PWM units (5%) above neutral to overcome static
# friction. Pure proportional at small errors falls below this threshold and
# the sub stops turning before reaching the target. The proven formula from
# competition testing uses max(30, ...) in raw PWM units, which in our
# percent_to_pwm system (±400 range) equals max(7.5%, ...).
YAW_SPEED_MIN_PCT  = 7.5    # floor: 30 PWM / 400 range * 100
YAW_SPEED_MAX_PCT  = 22.5   # ceiling: 90 PWM / 400 range * 100
STALE_HOLD_S       = 0.5    # if yaw_source goes silent longer than this,
                             # park Ch4 at 1500 instead of guessing

# ---- PID gains for commanded yaw turns ----------------------------------------
# Kp: scales proportionally -- 1.2 %/deg gives ~11% at 9 deg error (well above
#     T200 dead zone) and ~22% at max range, staying below YAW_SPEED_MAX_PCT.
# Ki: small integral to push through static friction near target (anti-windup
#     cap at YAW_KI_MAX prevents runaway on long overshoots).
# Kd: derivative damping -- reduces overshoot on large turns by braking as the
#     error shrinks. Uses 1/YAW_RATE_HZ as dt, so units are %/deg/sample.
YAW_KP            = 1.2    # proportional gain (%/deg)
YAW_KI            = 0.03   # integral gain (%/deg·sample); 0 to disable
YAW_KD            = 0.5    # derivative gain (%/deg change)
YAW_KI_MAX        = 8.0    # anti-windup: clamp accumulated integral output

# ---- Glide-only tunables ---------------------------------------------
YAW_AVG_DPS   = 30.0   # average deg/s across a glided turn
# 90 deg -> 3.0 s, 45 deg -> 1.5 s, 180 deg -> 6.0 s.
# Peak rate = avg x 1.875 (smootherstep peak derivative).
YAW_MIN_DUR   = 1.5    # lower bound so small turns still get a glide


class _YawPID:
    """Minimal PID state for a single yaw turn.

    Call update(error_deg) each tick; returns the signed %output to
    drive into Ch4. Integral resets automatically when error crosses zero
    (direction reversal) to prevent windup during overshoots.
    """

    def __init__(self):
        self._i_acc   = 0.0
        self._last_e  = 0.0

    def update(self, error_deg: float) -> float:
        if abs(error_deg) <= YAW_TOL_DEG:
            self._i_acc = 0.0
            self._last_e = error_deg
            return 0.0

        # Reset integrator when error sign flips (overshoot crossed target)
        if math.copysign(1, error_deg) != math.copysign(1, self._last_e):
            self._i_acc = 0.0

        d_term = YAW_KD * (error_deg - self._last_e)
        self._i_acc = max(-YAW_KI_MAX, min(YAW_KI_MAX,
                          self._i_acc + YAW_KI * error_deg))

        raw = YAW_KP * error_deg + self._i_acc + d_term
        speed = max(YAW_SPEED_MIN_PCT, min(YAW_SPEED_MAX_PCT, abs(raw)))

        self._last_e = error_deg
        return math.copysign(speed, raw)


def _yaw_rate_pct(error_deg: float) -> float:
    """Simple P-only clamped rate -- used by heading_lock and glide sweep phase.

    Returns signed percent of Ch4 authority. Deadbands at YAW_TOL_DEG.
    Formula matches competition-proven sample_codebase:
    max(30, min(90, |err|/180*200)) in raw PWM, ≡ max(7.5, min(22.5, ...))%.
    """
    if abs(error_deg) <= YAW_TOL_DEG:
        return 0.0
    speed = max(YAW_SPEED_MIN_PCT,
                min(YAW_SPEED_MAX_PCT, abs(error_deg) / 180.0 * 50.0))
    return math.copysign(speed, error_deg)


def _send_yaw_pct(pixhawk, yaw_pct: float) -> None:
    """Write a single Ch4-rate-override frame. Ch4 = 1500 means 'zero
    pilot yaw rate' (ArduSub holds heading). Any other value is treated
    by ArduSub as a pilot yaw-rate command, bypassing its internal
    compass-driven heading hold -- which is the whole point of this
    rewrite.
    """
    pixhawk.send_rc_override(yaw=Pixhawk.percent_to_pwm(yaw_pct))


def _lock_to_target(pixhawk, end_heading, timeout, label, log,
                    yaw_source, current, last_good_mono, pid=None):
    """Hold a heading by Ch4 rate-override until locked or timed out.

    Uses PID control for precise, smooth settling. When `pid` is provided
    (a _YawPID instance from the caller's snap phase), its accumulated
    I/D state carries over so there's no discontinuity at the phase
    boundary in yaw_glide. When None, a fresh PID is created.

    Returns:
        None on success (also writes a ``[YAW  ] OK`` log line).

    Raises:
        MovementTimeout: when ``timeout`` seconds elapse without
                         ``YAW_LOCK_N`` consecutive in-tolerance
                         samples. Channel 4 is parked at 1500 us
                         before the exception so the sub stops turning.
    """
    if pid is None:
        pid = _YawPID()

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

        yaw_pct = pid.update(error)
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
#  yaw_snap -- PID rate loop, yaw_source drives motion AND termination   #
# ---------------------------------------------------------------------- #
def yaw_snap(pixhawk, start_heading, end_heading,
             timeout, label, log, yaw_source=None):
    turn_degrees = Pixhawk.heading_error(end_heading, start_heading)
    log.info(
        f'[CMD  ] yaw_{label.lower()}  {abs(turn_degrees):.0f} deg  '
        f'cur={start_heading:.1f}  tgt={end_heading:.1f}  (PID)')

    _lock_to_target(
        pixhawk, end_heading, timeout, label, log, yaw_source,
        current=start_heading,
        last_good_mono=time.monotonic(),
        pid=_YawPID())


# ---------------------------------------------------------------------- #
#  yaw_glide -- smoothed setpoint sweep, then PID lock                  #
# ---------------------------------------------------------------------- #
def yaw_glide(pixhawk, start_heading, end_heading,
              timeout, label, log, yaw_source=None):
    turn_degrees = Pixhawk.heading_error(end_heading, start_heading)
    duration     = max(YAW_MIN_DUR, abs(turn_degrees) / YAW_AVG_DPS)

    log.info(
        f'[CMD  ] yaw_{label.lower()}  {abs(turn_degrees):.0f} deg  '
        f'cur={start_heading:.1f}  tgt={end_heading:.1f}  '
        f'(GLIDE {duration:.1f}s)')

    # ---- Phase 1: P rate-loop closes on a smootherstep-swept target --
    # Phase 1 uses simple P (not PID) because the swept target is itself
    # moving -- integral would just build up against a moving reference.
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

    # ---- Phase 2: PID lock on final target ----------------------------
    # Fresh PID so I doesn't carry over from the moving-reference phase.
    _lock_to_target(
        pixhawk, end_heading, timeout, label, log, yaw_source,
        current=current,
        last_good_mono=last_good_mono,
        pid=_YawPID())

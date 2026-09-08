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

YAW_TOL_DEG   = 2.0    # heading tolerance for "locked". 2° is the pre-regression
                       # proven value: sub-degree settle on a 20 kg hull at
                       # YAW_RATE_HZ is unrealistic, and SUSTAINED precision after
                       # a turn is heading_lock's job, not the turn's success gate.
                       # (Was briefly 1.0 -> combined with the hard floor below it
                       # made every yaw command wobble forever and TIMEOUT.)
YAW_LOCK_N    = 5      # consecutive frames within tol before success (~0.5 s at
                       # YAW_RATE_HZ=10). If YAW_RATE_HZ is ever raised, scale this
                       # to keep the same ~0.5 s dwell or a 0.1 s blip declares OK.

# ---- Rate-loop tunables (floor TAPERED across an approach band) -------------
# T200 thrusters need ~30 PWM (7.5%) above neutral to overcome static friction,
# so during the bulk of a turn we floor the rate to that -- pure proportional
# falls below it near target and the sub stalls SHORT. BUT a HARD floor at every
# error > tol is its own bug: it pins Ch4 at 7.5% right up to the tol edge, and a
# heavy hull (+ BNO/actuator latency, 100 ms pulses at 10 Hz) overshoots the band,
# flips sign, and limit-cycles forever -> never YAW_LOCK_N in-band -> TIMEOUT.
# Fix: the floor is full only OUTSIDE an approach band, then TAPERS LINEARLY to
# zero at the tol edge (see _yaw_floor). Inside the band the command decays and
# the anti-stall integral (previously masked by max(floor, .)) is unmasked, so
# the hull eases in and a residual stall winds up gently instead of being driven
# through the target. heading_lock keeps its own (continuous-hold) floor -- this
# taper is only for the discrete "turn then declare locked" path.
YAW_SPEED_MIN_PCT     = 7.5    # full floor: 30 PWM / 400 range * 100 (outside band)
YAW_SPEED_MAX_PCT     = 22.5   # ceiling: 90 PWM / 400 range * 100
YAW_APPROACH_BAND_DEG = 6.0    # |error| below this -> floor tapers MIN_PCT -> 0 at tol
STALE_HOLD_S          = 0.5    # if yaw_source goes silent longer than this,
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
YAW_KD            = 0.5    # derivative gain (%/deg change per tick at _YAW_DT_REF)

# The loop period the gains above were POOL-TUNED at. Both the D and the I term
# are scaled by the ratio of the live dt to this, so the numbers are unchanged at
# the shipped rate and rate-INDEPENDENT if it ever moves. (B13)
_YAW_DT_REF = 1.0 / YAW_RATE_HZ
YAW_KI_MAX        = 8.0    # anti-windup: clamp accumulated integral output

# ---- Glide-only tunables ---------------------------------------------
YAW_AVG_DPS   = 30.0   # average deg/s across a glided turn
# 90 deg -> 3.0 s, 45 deg -> 1.5 s, 180 deg -> 6.0 s.
# Peak rate = avg x 1.875 (smootherstep peak derivative).
YAW_MIN_DUR   = 1.5    # lower bound so small turns still get a glide


def _yaw_floor(abs_error_deg: float) -> float:
    """Stiction-breaking speed floor (%), tapered across the approach band.

    Full ``YAW_SPEED_MIN_PCT`` at/above ``YAW_APPROACH_BAND_DEG`` (brisk travel,
    break T200 stiction), then linearly to **0 at the lock tolerance** so the
    command can decay and the hull eases into the band instead of being driven
    through it at a hard floor (the limit-cycle bug). Pure / side-effect-free.
    Only meaningful for ``abs_error_deg > YAW_TOL_DEG`` (inside tol the loop
    commands 0 and never calls this).
    """
    if abs_error_deg >= YAW_APPROACH_BAND_DEG:
        return YAW_SPEED_MIN_PCT
    span = YAW_APPROACH_BAND_DEG - YAW_TOL_DEG
    if span <= 0.0:
        return YAW_SPEED_MIN_PCT
    frac = (abs_error_deg - YAW_TOL_DEG) / span   # 1.0 at band edge -> 0 at tol
    return YAW_SPEED_MIN_PCT * max(0.0, frac)


class _YawPID:
    """Minimal PID state for a single yaw turn.

    Call update(error_deg) each tick; returns the signed %output to
    drive into Ch4. Integral resets automatically when error crosses zero
    (direction reversal) to prevent windup during overshoots.
    """

    def __init__(self):
        self._i_acc   = 0.0
        # None, not 0.0: on the FIRST tick there is no previous error, and
        # pretending it was 0 manufactures a derivative of the full error --
        # 0.5 x 90 = 45% for a 90 deg turn. The clamp masked that; it did not
        # prevent it. The first tick now contributes no D at all. (B13)
        self._last_e  = None

    def update(self, error_deg: float, dt: float | None = None) -> float:
        # B13: `dt` was absent from BOTH the D and the I term, so YAW_KD and
        # YAW_KI were per-TICK, not per-second -- silently coupled to
        # YAW_RATE_HZ. Changing the loop rate retuned the controller without
        # anyone touching a gain.
        #
        # The gains are POOL-TUNED at YAW_RATE_HZ, so they are anchored to that
        # rate rather than rewritten: at dt == _YAW_DT_REF both expressions below
        # reduce EXACTLY to what shipped (verified numerically), while a rate
        # change now preserves behaviour instead of altering it. Same approach as
        # the B09 Kalman-Q fix, for the same reason.
        dt = _YAW_DT_REF if (dt is None or dt <= 0.0) else float(dt)

        if abs(error_deg) <= YAW_TOL_DEG:
            self._i_acc = 0.0
            self._last_e = error_deg
            return 0.0

        # Reset integrator when error sign flips (overshoot crossed target)
        if self._last_e is not None and \
           math.copysign(1, error_deg) != math.copysign(1, self._last_e):
            self._i_acc = 0.0

        if self._last_e is None:
            d_term = 0.0                      # no previous sample -> no derivative
        else:
            d_term = YAW_KD * (error_deg - self._last_e) * (_YAW_DT_REF / dt)
        self._i_acc = max(-YAW_KI_MAX, min(YAW_KI_MAX,
                          self._i_acc + YAW_KI * error_deg * (dt / _YAW_DT_REF)))

        raw = YAW_KP * error_deg + self._i_acc + d_term
        # Cap, then apply the TAPERED floor (full only outside the approach band).
        # Near target the floor relaxes to ~0 so |raw| (P/D + the now-unmasked
        # integral) decides the command -- the hull eases in instead of limit-
        # cycling on a hard 7.5% floor.
        mag   = min(YAW_SPEED_MAX_PCT, abs(raw))
        speed = max(_yaw_floor(abs(error_deg)), mag)

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
                    yaw_source, current, last_good_mono, pid=None,
                    abort_fn=None):
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

    deadline       = time.monotonic() + timeout
    frames_locked  = 0
    peak_error_deg = 0.0

    while time.monotonic() < deadline:
        if abort_fn and abort_fn():
            _send_yaw_pct(pixhawk, 0.0)
            return
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
             timeout, label, log, yaw_source=None, abort_fn=None):
    turn_degrees = Pixhawk.heading_error(end_heading, start_heading)
    log.info(
        f'[CMD  ] yaw_{label.lower()}  {abs(turn_degrees):.0f} deg  '
        f'cur={start_heading:.1f}  tgt={end_heading:.1f}  (PID)')

    _lock_to_target(
        pixhawk, end_heading, timeout, label, log, yaw_source,
        current=start_heading,
        last_good_mono=time.monotonic(),
        pid=_YawPID(), abort_fn=abort_fn)


# ---------------------------------------------------------------------- #
#  yaw_glide -- smoothed setpoint sweep, then PID lock                  #
# ---------------------------------------------------------------------- #
def yaw_glide(pixhawk, start_heading, end_heading,
              timeout, label, log, yaw_source=None, abort_fn=None):
    turn_degrees = Pixhawk.heading_error(end_heading, start_heading)
    duration     = max(YAW_MIN_DUR, abs(turn_degrees) / YAW_AVG_DPS)

    log.info(
        f'[CMD  ] yaw_{label.lower()}  {abs(turn_degrees):.0f} deg  '
        f'cur={start_heading:.1f}  tgt={end_heading:.1f}  '
        f'(GLIDE {duration:.1f}s)')

    # ---- Phase 1: P rate-loop closes on a smootherstep-swept target --
    # Phase 1 uses simple P (not PID) because the swept target is itself
    # moving -- integral would just build up against a moving reference.
    started_at     = time.monotonic()
    current        = start_heading
    last_good_mono = time.monotonic()

    while True:
        elapsed = time.monotonic() - started_at
        if elapsed >= duration:
            break
        if abort_fn and abort_fn():
            _send_yaw_pct(pixhawk, 0.0)
            return

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
        pid=_YawPID(), abort_fn=abort_fn)

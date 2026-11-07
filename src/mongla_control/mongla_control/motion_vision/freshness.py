"""How much a bounding box may still steer, from how old it is.

The loop runs faster than a slow detector, so one box is read on several
ticks. These pure functions turn the age of that box -- and, for a coasted
box, the time since the last real detection -- into a 0..1 authority the
loops multiply their lateral and forward commands by. Yaw is never scaled.
"""

from __future__ import annotations

import math

from ..motion_rates import LOG_THROTTLE_S


# Freshness-decay of the translational command. The loop runs at VISION_LOOP_HZ
# (20 Hz) but the detector may publish far slower (3-4 Hz when inference-bound),
# so the same bbox error is re-used for several ticks. Re-commanding the same P
# output while the hull drives blind for a whole frame period over-drives by
# ~Kp*e*T_frame -- the low-FPS twitchiness. Fix: full command authority the
# instant a frame lands, then decay the LATERAL/FORWARD command toward neutral as
# the sample ages, hard-zero once we are driving blind. Yaw is NOT decayed: it
# is a rate demand the board's heading hold absorbs. At healthy FPS frames
# refresh before VISION_FRESH_FULL_S so the factor stays 1.0 -- zero behaviour
# change; the decay only engages when detections are slow or drop out. ZERO_S is
# below _STALE_LIMIT_S so the command zeroes BEFORE "target lost" declares.
#
# Nothing here is tied to a particular camera. The thresholds are DERIVED at
# runtime from two quantities the loop observes: the pipeline latency and the
# detection interval. A competition can put any sensor in front of us and this
# has to keep meaning the same thing.
#
# Why derivation is necessary rather than tidy -- measured on the vehicle,
# both detectors alone, same instrument:
#
#                      forward        downward
#     detections       77.1 Hz         30.1 Hz
#     sample age med   22.10 ms        46.51 ms
#             max      32.88 ms        56.42 ms
#     interval med     13.44 ms        32.41 ms
#
# `sample.age_s` is time since CAPTURE, so it carries the whole pipeline. A
# threshold must therefore cover the PIPELINE (or the loop never reaches full
# authority at all) plus enough INTERVALS to ride out a missed detection (or
# it decays during normal operation). Neither term is optional and neither is
# a constant across cameras.
#
# At the old fixed 0.10/0.40 the forward path held FULL authority through
# seven consecutive missed detections and drove blind at partial authority for
# 400 ms -- 26 cm at 0.65 m/s. Simply tightening the constant would have put
# the DOWNWARD path permanently below full authority, since its max age
# (56.4 ms) already exceeds a 50 ms threshold.
#
# The floors and the CEILING are safety rails, not tuning:
#   * floors    keep a degenerate observation (one sample, a zero interval)
#               from producing a threshold tighter than the pipeline
#   * ceilings  bound the derivation, and this is the part that matters.
#
# The full ceiling exists because derivation alone is unsafe. Feed this a
# pipeline that is ALWAYS 300 ms late and it concludes "300 ms is normal here"
# and grants full authority -- which is exactly wrong. An observation 300 ms
# old is 20 cm of travel at this hull's 0.65 m/s cruise, whatever the reason
# it is old. So full authority is capped at 100 ms (~6.5 cm) no matter what
# the sensor claims about itself; a genuinely slow camera runs at REDUCED
# authority, which is the safe answer, and `_warn_low_fps` already surfaces it
# to the operator rather than leaving it as a mystery sluggishness.
#
# The zero ceiling bounds blind driving the same way: a 5 Hz sensor would
# otherwise derive a zero-authority age above `lost_grace_s`, and the ladder
# "authority reaches zero BEFORE loss is declared" is what makes a dropout a
# glide instead of a lurch.
#
# Net effect versus the fixed 0.10/0.40 this replaces: never LOOSER than
# before (full <= 0.10 always), and considerably tighter on a fast camera.
VISION_FRESH_FULL_S  = 0.05   # floor for full authority
VISION_FRESH_ZERO_S  = 0.20   # floor for zero authority (driving blind)
VISION_FRESH_FULL_MAX_S = 0.10  # CEILING on full authority -- see below
VISION_FRESH_ZERO_MAX_S = 0.80  # CEILING on blind driving; under lost_grace_s (1.0)
_FRESH_PIPE_K = 1.3    # margin on the observed pipeline latency
_FRESH_FULL_K = 1.5    # ...plus this many detection intervals -> full
_FRESH_ZERO_K = 8.0    # ...plus this many -> zero


def _fresh_bounds(interval_s: float = 0.0, pipe_age_s: float = 0.0):
    """(full, zero) authority thresholds for the sensor actually attached.

    Derived, never configured: `pipe_age_s` is the latency the pipeline is
    observed to have (capture -> the loop seeing it) and `interval_s` the gap
    between detections. Both are measured by the caller.

    Pure and total. With nothing observed yet -- the first frame, or a caller
    that tracks neither -- it returns the floors, which is the behaviour that
    shipped before and is safe for a fast camera.

    Invariants it guarantees for ANY input, including hostile ones:
      * full < zero, so the ramp is never inverted or zero-width
      * full <= VISION_FRESH_FULL_MAX_S, in ABSOLUTE time -- a slow pipeline
        does not make old data safe to steer on
      * zero <= VISION_FRESH_ZERO_MAX_S, so blind driving is bounded on a
        slow sensor and always ends before `lost_grace_s` declares loss
      * both >= their floors, so a degenerate measurement cannot produce a
        threshold tighter than the pipeline itself
    """
    if interval_s <= 0.0 and pipe_age_s <= 0.0:
        return VISION_FRESH_FULL_S, VISION_FRESH_ZERO_S
    interval_s = max(0.0, interval_s)
    # The observed pipeline is trusted only up to the safety cap. Beyond it
    # the pipeline is degraded, and a degraded pipeline must not be allowed to
    # EXTEND the window we are willing to steer in -- that is the derivation
    # arguing itself into more trust the worse things get. Measured effect: a
    # pipeline stuck at 350 ms stretched the zero-authority age to 800 ms and
    # a 350 ms-old sample still commanded 64 % thrust.
    base = _FRESH_PIPE_K * min(max(0.0, pipe_age_s), VISION_FRESH_FULL_MAX_S)
    full = max(VISION_FRESH_FULL_S, base + _FRESH_FULL_K * interval_s)
    # Capped in ABSOLUTE time, not relative to the sensor. A slow pipeline
    # does not make old data safe to steer on.
    full = min(full, VISION_FRESH_FULL_MAX_S)
    zero = max(VISION_FRESH_ZERO_S, base + _FRESH_ZERO_K * interval_s)
    zero = min(zero, VISION_FRESH_ZERO_MAX_S)
    # The ceiling can drag `zero` below `full` on a very slow sensor. Keep a
    # real ramp rather than a step: a step means the command goes from full
    # authority to nothing between two ticks.
    full = min(full, zero * 0.5)
    return full, zero


# A box is where the target was. `sample.age_s` runs from CAPTURE (median
# 22 ms forward, 46 ms downward on the vehicle, and it keeps growing while a
# 20 Hz loop re-reads one frame). The hull keeps yawing in that time, so a
# world-fixed target has moved f*tan(w*age) in the image. At 0.5 rad/s over
# 30 ms that is ~15 px of phantom error the loop chases -- and when the loop is
# the one yawing, the phantom points the way the hull is already going: the
# overshoot-past-the-target the operator reported. Attenuating by age
# (`_freshness`) shrinks a wrong number; this makes it right.
#
# Capped: beyond ~0.25 s a constant-rate extrapolation is a guess, and
# `_freshness` has already removed most of the authority by then.
DEROTATE_MAX_AGE_S = 0.25


def _derotated_ex(ex: float, age_s: float, yaw_rate: float,
                  px_per_rad: float, half_w: float) -> float:
    """`ex` moved to where a world-fixed target is NOW. Identity when unknown.

    Positive yaw rate is a turn to the RIGHT (MAVLink FRD), which moves the
    target LEFT. `px_per_rad` maps a ray angle to pixels in the medium the
    camera is in (fx * n in water).
    """
    if not (age_s > 0.0 and half_w > 0.0 and px_per_rad > 0.0
            and math.isfinite(yaw_rate) and math.isfinite(ex)):
        return ex
    dtheta = yaw_rate * min(age_s, DEROTATE_MAX_AGE_S)
    theta = math.atan(ex * half_w / px_per_rad)
    return px_per_rad * math.tan(theta - dtheta) / half_w


def _freshness(age_s: float, interval_s: float = 0.0,
               pipe_age_s: float = 0.0) -> float:
    """Translational-command authority [0,1] as a function of sample age.

    1.0 while the sample is fresher than the FULL threshold, then linearly to
    0.0 by the ZERO threshold (driving blind -> stop). Both scale with the
    observed detection interval; see `_fresh_bounds`. Pure + side-effect-free
    so it unit-tests without ROS. Caps per-frame over-drive at low FPS while
    leaving healthy FPS untouched (frames refresh before decay engages).

    `interval_s` defaults to 0 so every existing caller and test keeps the
    unscaled behaviour; the loop passes the interval it measures.
    """
    full, zero = _fresh_bounds(interval_s, pipe_age_s)
    if age_s <= full:
        return 1.0
    if age_s >= zero:
        return 0.0
    return (zero - age_s) / (zero - full)


def _coast_authority(coast_age_s: float, coast_s: float) -> float:
    """Translational authority [0,1] for a COASTED (tracker-predicted) box.

    1.0 at the instant the real detection drops, decaying LINEARLY to 0.0 by
    ``coast_s`` (time since the last real detection). Distinct from
    ``_freshness``: the predicted box itself is fresh every tick (it arrives
    each frame), so freshness would NOT decay it -- this curve is the SEPARATE,
    longer window that bounds how long, and how hard, we steer on a guess. Sized
    so authority reaches ~0 before ``lost_grace_s`` fires LOST. Pure; coast_s<=0
    -> 0.0 (coasting disabled). See the timeout ladder in precision-alignment.md.
    """
    if coast_s <= 0.0:
        return 0.0
    if coast_age_s <= 0.0:
        return 1.0
    if coast_age_s >= coast_s:
        return 0.0
    return (coast_s - coast_age_s) / coast_s


def _authority(sample, coast_s: float, interval_s: float = 0.0,
               pipe_age_s: float = 0.0) -> float:
    """Per-tick translational authority for `sample`.

    Live box  -> ``_freshness(age)``       (per-frame staleness at low FPS).
    Coasted   -> ``_coast_authority(age)``  (gap decay; the box is fresh each
                 tick so freshness must NOT also be applied -- that would
                 double-decay and kill the coast inside ~0.4 s).
    """
    if getattr(sample, 'coasted', False):
        return _coast_authority(sample.age_s, coast_s)
    return _freshness(sample.age_s, interval_s, pipe_age_s)


# Below this authority, a LIVE bbox is stale enough that freshness-decay is
# materially cutting the translational command -- the operator should see it as
# "detector too slow", not a mystery stall (the align/move barely translates yet
# never declares LOST, so no fallback fires). Diagnostic only; no behaviour change.
_FRESH_WARN_FLOOR = 0.5


def _warn_low_fps(log, fresh: float, sample) -> None:
    """Throttled heads-up when low detector FPS is eating translational authority.

    Fires only for a LIVE box (a coast has its own decay semantics) whose
    freshness has fallen below ``_FRESH_WARN_FLOOR``. Pure observability: raising
    detector FPS (a compiled ``.hef``, one detector at a time) is the fix; at healthy FPS fresh==1.0 and
    this never fires.
    """
    if log is None or getattr(sample, 'coasted', False) or fresh >= _FRESH_WARN_FLOOR:
        return
    log.info(
        f"[VIS  ] low detector FPS: lat/fwd authority {fresh * 100:.0f}% "
        f"(bbox {sample.age_s * 1000:.0f}ms stale) -- raise FPS (.hef) if "
        f"the approach stalls",
        throttle_duration_sec=LOG_THROTTLE_S)

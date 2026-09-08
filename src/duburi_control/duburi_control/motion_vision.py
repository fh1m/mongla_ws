#!/usr/bin/env python3
"""Vision-driven closed-loop motion -- two loops, pixel-native.

This module is the control engine behind the two mission verbs
``vision_align`` and ``vision_move``. Everything here is P-control on
**pixel error**, with ``gain`` acting as a hard max-speed cap. There are
no frame-age knobs, no on_lost policies, no lock modes -- the mission DSL
owns search/recovery via a Python ``fallback`` function.

Two loops
---------
``align_loop`` -- centre the target on the selected axes (lat / yaw /
    depth), each with an optional pixel offset. Exits when every active
    axis is within ``err_px`` for ``align_stable_frames`` ticks.

``move_loop`` -- drive forward until the target's bounding box fills the
    frame to ``fwd_fill`` (measured by ``mode`` = area/width/height),
    optionally holding a lateral pixel offset (``maintain``). Never
    re-centres yaw/depth; depth is left to ArduSub's depth-hold.

Both return an :class:`Outcome` whose ``code`` the facade copies into
``Move.Result.final_value`` so the DSL can branch without the action
client ever raising on a miss.

Sign rules (forward camera, image y grows downward)
---------------------------------------------------
  ex > 0  -> target RIGHT of centre
  ey > 0  -> target BELOW centre
  yaw : target-right -> yaw RIGHT toward it -> NO negate (same polarity as
        lateral; pool-verified -- the old `-ex` negation drove away from the
        target)
  lat : Ch6 > 1500 = strafe RIGHT, so target-right needs Ch6 > 1500 -> NO negate
  depth: target below -> descend -> depth setpoint more negative

Offsets (pixels, signed): a positive ``lat``/``yaw`` offset keeps the
target that many px to the RIGHT of centre; a positive ``depth`` offset
keeps it that many px BELOW centre. ``0`` = dead centre.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, Optional, Set

from .pixhawk import Pixhawk
from .motion_rates import VISION_LOOP_HZ as LOOP_HZ
from .motion_rates import VISION_LOOP_HZ_SROT
from .motion_rates import DEPTH_SETPOINT_HZ as DEPTH_HZ
from .motion_rates import LOG_THROTTLE_S
from .motion_writers import REVERSE_KICK_SEC, _interruptible_sleep, is_srot


# ---- Outcome codes (copied to Move.Result.final_value) --------------- #
ALIGNED   = 0   # align: centred; move: bbox reached fwd_fill
LOST      = 1   # target gone past lost_grace_s (DSL runs fallback)
TIMEOUT   = 2   # duration elapsed without success
NO_CAMERA = 3   # camera_info never seen -- pipeline not up
ABORTED   = 4   # cooperative abort (goal cancelled)

_CODE_NAME = {ALIGNED: 'OK', LOST: 'LOST', TIMEOUT: 'TIMEOUT',
              NO_CAMERA: 'NO_CAMERA', ABORTED: 'ABORTED'}


# ---- Tunable defaults (P-gains live as ROS params; these are the floor) #
KP_LAT_DEFAULT     = 60.0
KP_YAW_DEFAULT     = 60.0
KP_DEPTH_DEFAULT   = 0.05
KP_FORWARD_DEFAULT = 200.0

# align_loop's optional SETTLE gate (opt-in; off by default). Why align can end
# off-target while move's `maintain` centering looks near-perfect: move never
# exits on lateral (it exits on forward fill), so lateral is corrected for the
# whole approach and is genuinely SETTLED (~0 velocity) when it stops. align
# exits the instant POSITION is in-band for align_stable_frames ticks -- a
# *position* gate, not a *settle* gate -- so a hull strafing THROUGH centre at
# speed can satisfy the band mid-pass, declare ALIGNED, exit, and coast out on
# inertia (and the returned px was measured BEFORE the arrival brake, so it reads
# clean while the hull ends dirty). The settle gate ports move's behaviour: when
# settle_px>0, a tick only counts toward `stable` if the worst error is in-band
# AND barely moving frame-to-frame (|Δworst| <= settle_px) -- i.e. the hull has
# actually stopped on target, not just passed through. Keyed on ERROR VELOCITY
# (Δworst), NOT command magnitude, so a steady current (which holds a non-zero
# command at a perfect lock) does NOT block the gate -- only genuine motion does.
# settle_px=0 (default) -> gate off, exit path byte-identical to before.
SETTLE_PX_DEFAULT = 0.0   # 0 = settle gate off (legacy exit-on-position-in-band)

# align_loop's optional forward range-hold axis (the unified torpedo standoff shot).
# When fwd_fill>0 align ALSO drives forward toward that fill (same _fill law as
# move_loop) so ONE verb does forward-standoff + lat/depth + hold + fire. The term
# is ONE-SIDED: drive forward while too far, command exactly 0 at/past the standoff
# (never reverse -- no reverse-kick, no ramming the board; water drag bleeds a small
# overshoot). FWD_BAND is the fill deadband within which forward counts as "at
# standoff" (in-band -> contributes to the stable/fire gate, so the shot only leaves
# once we are AT the standoff). A fraction of frame, pool-tunable. fwd_fill=0
# (default) -> no forward axis, align behaviour unchanged bit-for-bit.
FWD_BAND = 0.03   # |fwd_fill - fill| within this -> at standoff, forward neutral

# Yaw is THE essential axis for micro-aligning + holding against a small
# target (the torpedo 'hole'). Pure-proportional yaw falls below the T200
# spin-up threshold near centre, so a small residual error commands only a
# few PWM and the thruster never turns -- the hull drifts off and the tight
# hole-lock stalls just outside err_px. Mirror heading_lock: zero inside the
# err_px deadband, else floor the magnitude so corrections actually move the
# hull. Starting value copied from heading_lock.LOCK_SPEED_MIN_PCT; it is a
# hardware spin-up assumption, NOT a measured value -- confirm on pool day
# with the bare Ch4 check (does the floor visibly spin the yaw thrusters?).
VISION_YAW_MIN_PCT = 5.0

# The yaw floor above is a hard minimum command outside the deadband. On Ch4
# (a yaw RATE) that is a relay feeding an integrator -- a limit-cycle oscillator
# that wobbles the hull left/right when aligning a small, distant target. Gate
# it on bbox fill so it only engages CLOSE (large bbox), where it does its real
# job: break T200 stiction for the tight hole-lock. Far away (small bbox) yaw is
# left pure-proportional and decays cleanly into the deadband with no relay.
# Pool-tunable: confirm the re-engage distance on pool day.
VISION_YAW_FLOOR_FILL = 0.25

# The close-in yaw floor above must not be a hard relay either: a hard min on the
# Ch4 rate channel just outside the deadband slams the hull, overshoots, and
# limit-cycles (the reported terminal yaw jitter). Taper the floor to 0 across
# this px band above the deadband so a small residual eases in -- mirrors
# heading_lock._lock_floor / motion_yaw._yaw_floor (the proven fix, applied twice
# already on the discrete-turn + lock paths). Pool-tunable.
VISION_YAW_APPROACH_BAND_PX = 40.0

# Inertial arrival brake (lateral + forward-on-fill-stop). The vision loops drive
# open-loop translation (Ch5/Ch6); on arrival the hull coasts on water inertia in
# its last travel direction, drifting off the planned position so the next mission
# step starts wrong. Mirror the control verbs' reverse-kick brake, but scale it to
# how hard the loop was actually translating just before exit -- a trailing EMA of
# the signed thrust command. brake = reverse kick opposite the EMA, magnitude
# VISION_BRAKE_GAIN * |EMA|, capped. A near-zero gate (VISION_BRAKE_MIN_PCT) skips
# the kick when the loop had already tapered into the deadband (gentle convergence,
# e.g. the torpedo hole-lock) so the shot is never disturbed. All pool-tunable.
VISION_BRAKE_GAIN    = 0.6    # reverse-kick % = this * |EMA command %|
VISION_BRAKE_MIN_PCT = 6.0    # gate: skip the kick below this |EMA| (deadband exit)
VISION_BRAKE_CAP_PCT = 30.0   # never reverse-kick harder than this
_BRAKE_EMA_ALPHA     = 0.3    # per-tick EMA weight (~0.4 s memory at LOOP_HZ)

# --- Range-adaptive gain schedule (Layer 1) -------------------------------- #
# The law is P on NORMALIZED pixel error, so the image shift per unit hull
# motion grows ~1/range (~ bbox fill): a kp critically damped far-field is
# OVER-gained close-in, oscillating the 20 kg hull off a small target. Scale the
# lat/depth kp DOWN as fill grows -- 1.0 below FILL_LO (far, full gain), ramping
# linearly to `floor` at/above FILL_HI (close, gentle). `floor` is the only deck
# knob (vision.range_gain_floor; 1.0 = off); the ramp endpoints stay constants.
# Yaw is EXCLUDED -- terminal yaw is delegated to heading_lock, and near-field
# yaw has its own stiction floor that ADDS authority (opposite intent).
VISION_RANGE_GAIN_FILL_LO = 0.25   # fill below which lat/depth gain is unscaled
VISION_RANGE_GAIN_FILL_HI = 0.60   # fill at/above which gain is fully floored

# --- Lateral anti-windup integral (Layer 2) -------------------------------- #
# Ch6 lateral is OPEN-LOOP thrust with no downstream position hold, so a steady
# current leaves a steady-state offset pure-P cannot null. A small bounded
# I-term (accumulated ONLY while holding, clamped to ±I_LAT_MAX, frozen on
# output saturation, reset on target loss) cancels it. ki rides
# vision.ki_lat (default 0 = OFF until the range damping is confirmed -- an
# integral on an under-damped loop makes it worse). Yaw/depth get NO I-term
# (Ch4 rate + ALT_HOLD already integrate downstream).
VISION_I_LAT_MAX = 15.0   # |lateral integral| clamp, % thrust

# --- Continuity lock (Layer 3) --------------------------------------------- #
# Once a target is acquired, prefer the detection NEAREST the last-accepted
# centre within this normalized gate (not the largest box) so a second hole /
# spurious box can't steal the aim. Enabled per-call via lock_on; the gate width
# is a constant (re-tune in code if a target legitimately moves faster than this
# between ticks). Acquisition (no prior centre) stays largest-area.
VISION_LOCK_GATE_NORM = 0.30   # max normalized centre jump per tick AT 20 Hz

# The DETECTION rate the gate above was tuned at. It is a per-frame jump limit,
# so it is really a VELOCITY -- and leaving it a constant while the rate
# changed silently loosened it: 0.30 of the frame between detections is 6.0
# frame-widths/s at 20 Hz and 29.4 at the Hailo path's 98 Hz, so the same
# number let a target move 5x faster before the lock let go. The gate exists to
# stop a second hole or a spurious box stealing the aim on a close-in shot, and
# a lock that admits anything within 30 % of the frame is barely a lock.
VISION_LOCK_GATE_HZ = 20.0

# Never scale below this. At a high loop rate the dt-scaled gate becomes very
# small, and a gate tighter than the detector's own per-frame centre jitter
# would drop the lock on noise -- the failure mode is the mirror of the one
# above and just as bad. 0.05 of a 640 px frame is 32 px, comfortably above the
# few-px jitter measured on this detector.
VISION_LOCK_GATE_MIN = 0.05


def _lock_gate(dt_s: float) -> float:
    """The continuity-lock gate for a gap of `dt_s` between accepted centres.

    THE CLOCK IS THE DETECTION INTERVAL, NOT THE CONTROL TICK, and getting that
    wrong is a defect I shipped and had to correct. The gate limits how far the
    box may jump between ACCEPTED CENTRES -- and a centre only changes when a
    new detection lands. Scaling by the control period made it 1.7x TIGHTER
    than tuned at 50 Hz control over a 30 Hz camera, which drops the lock on a
    real target instead of holding it: the exact failure the floor below exists
    to prevent, reintroduced by the fix for the opposite one.

    Pure and side-effect-free so it unit-tests without ROS. At the tuned 20 Hz
    interval it returns exactly VISION_LOCK_GATE_NORM.
    """
    if not dt_s or dt_s <= 0.0:
        return VISION_LOCK_GATE_NORM
    scaled = VISION_LOCK_GATE_NORM * (float(dt_s) * VISION_LOCK_GATE_HZ)
    return max(VISION_LOCK_GATE_MIN, min(VISION_LOCK_GATE_NORM, scaled))

# --- Minimum achievable deadband ------------------------------------------- #
# A 20 kg hull on open-loop Ch6 thrust against a bbox that itself jitters a few
# px per frame cannot hold a literal 0 px error -- and `err=0` from the operator
# means "use the default" anyway (the rosidl-0 = unset live-tuning convention in
# commands.fields_for, NOT a literal zero deadband). So clamp the EFFECTIVE
# deadband to this floor: an over-tight POSITIVE err (e.g. err=1) still completes
# instead of perpetually TIMEOUTing on noise it can never satisfy. The align
# entry log prints the effective value so a floor is never silent. Pool-tunable
# to the detector's real per-frame bbox jitter.
MIN_ALIGN_ERR_PX = 5.0

# Freshness-decay of the translational command. The loop runs at VISION_LOOP_HZ
# (20 Hz) but the detector may publish far slower (3-4 Hz when inference-bound),
# so the same bbox error is re-used for several ticks. Re-commanding the same P
# output while the hull drives blind for a whole frame period over-drives by
# ~Kp*e*T_frame -- the low-FPS twitchiness. Fix: full command authority the
# instant a frame lands, then decay the LATERAL/FORWARD command toward neutral as
# the sample ages, hard-zero once we are driving blind. Yaw/depth are NOT decayed
# (Ch4 is a rate ArduSub bleeds; depth is ArduSub's hold). At healthy FPS frames
# refresh before VISION_FRESH_FULL_S so the factor stays 1.0 -- zero behaviour
# change; the decay only engages when detections are slow or drop out. ZERO_S is
# below _STALE_LIMIT_S so the command zeroes BEFORE "target lost" declares.
#
# NOTHING HERE IS TIED TO A PARTICULAR CAMERA. The thresholds are DERIVED at
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
# THE FULL CEILING EXISTS BECAUSE DERIVATION ALONE IS UNSAFE. Feed this a
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

# Distinct-detection gate for the align stable-frame counter. The counter must
# advance on new DETECTIONS, not 20 Hz control-loop ticks: otherwise, at a low
# detector FPS (e.g. 3-4 Hz on a raw .pt model) a single in-band frame re-read
# ``align_stable_frames`` times inside one detection period declares ALIGNED --
# and could fire a torpedo -- on effectively ONE frame. A detection's arrival
# time is ``now - sample.age_s`` (constant across re-reads of the same frame);
# a new frame advances it by more than this epsilon. Sized well under the
# inter-frame gap at competition FPS (33 ms at 30 Hz) yet above the sub-tick
# skew between the loop's ``now`` and bbox_error's internal clock. A detector at
# or above loop rate (age_s ~ 0 every tick) makes every tick a new frame, so
# behaviour is unchanged there (and the age_s=0 test doubles stay valid).
_FRAME_EPS_S = 0.005   # min monotonic gap to count a sample as a new detection

# ...AND the counter must also span real TIME, which counting frames alone
# stopped guaranteeing when perception got fast.
#
# `align_stable_frames = 3` was chosen against a 20 Hz detector, where three
# distinct frames span two inter-frame gaps = 0.10 s. The Hailo path runs
# 55-98 Hz, where the same three frames span 0.02-0.05 s. Three detections
# 20 ms apart are very nearly ONE moment: the hull cannot have settled in it,
# and a detector's centre noise is correlated across it -- so the gate that
# exists to stop a single lucky frame declaring ALIGNED (and ARMING THE FIRE)
# was measuring almost nothing. The frame count alone was FPS-independent in
# name only; it is the dwell that has to be.
#
# Requiring both a frame count and a dwell keeps each doing the job it can:
# frames rule out a re-read, time rules out a burst. At 20 Hz this is satisfied
# on the same tick the third frame lands, so the ArduSub path is unchanged
# except when the detector jitters, where it can cost one extra frame (~10 ms)
# -- the right price for not firing a torpedo on 20 ms of evidence. At 3-4 Hz
# three frames already span 0.75-1.0 s and this never binds.
_ALIGN_STABLE_MIN_S = 0.10

# A detection older than this (seconds) counts as "no target this tick".
# bbox_error() returns None when the class is absent; this only catches a
# detector that has died while the last box is still cached.
_STALE_LIMIT_S = 1.0

# Depth setpoint nudge ceiling per tick at gain=100 (scaled by gain/100).
_MAX_DEPTH_NUDGE = 0.02   # metres / tick

# Pass-through commit: how long to keep driving forward AFTER the target
# leaves the frame, so the hull fully clears the gate. Overridable per
# call via move(..., hold=<s>).
_PASSTHROUGH_COMMIT_S = 2.0

# Surfacing safety floor: never command shallower than 0.2 m.
_MIN_DEPTH_M = -0.2

VALID_AXES = {'lat', 'yaw', 'depth'}
VALID_MODES = {'area', 'width', 'height'}


@dataclass
class Outcome:
    """What the facade needs to build a Move.Result.

    ``code`` is one of the module constants above. ``last_err_px`` is the
    worst per-axis pixel error at exit (align) or the lateral error
    (move). ``fill`` is the bbox fill fraction at exit (move; 0 for
    align).

    ``end_x_px`` / ``end_y_px`` are the SIGNED pixel offset of the target from
    frame CENTRE at the LAST SEEN frame (+x = target right of centre, +y =
    below). They are ``nan`` when the target was never seen during the verb,
    which lets a mission tell "ended off to the left" from "never detected".
    Offset-independent (raw observable) -- distinct from ``last_err_px`` which
    is the residual from the goal (centre+offset).
    """
    code:        int
    reason:      str
    last_err_px: float = 0.0
    fill:        float = 0.0
    elapsed_s:   float = 0.0
    end_x_px:    float = math.nan
    end_y_px:    float = math.nan

    @property
    def ok(self) -> bool:
        return self.code == ALIGNED


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


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
    # THE OBSERVED PIPELINE IS TRUSTED ONLY UP TO THE SAFETY CAP. Beyond it
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
    detector FPS (TensorRT ``.engine``) is the fix; at healthy FPS fresh==1.0 and
    this never fires.
    """
    if log is None or getattr(sample, 'coasted', False) or fresh >= _FRESH_WARN_FLOOR:
        return
    log.info(
        f"[VIS  ] low detector FPS: lat/fwd authority {fresh * 100:.0f}% "
        f"(bbox {sample.age_s * 1000:.0f}ms stale) -- raise FPS (.engine) if "
        f"the approach stalls",
        throttle_duration_sec=LOG_THROTTLE_S)


def _range_gain(fill: float, floor: float,
                lo: float = VISION_RANGE_GAIN_FILL_LO,
                hi: float = VISION_RANGE_GAIN_FILL_HI) -> float:
    """kp multiplier in [floor, 1] that DROPS as bbox fill (closeness) rises.

    1.0 below `lo` fill (far -> full gain), ramping linearly to `floor` at/above
    `hi` fill (close -> gentle), cancelling the ~1/range loop-gain rise of
    normalized-pixel P-control so the hull stays damped from far to close.
    `floor` >= 1.0 (or hi<=lo) is a no-op. Pure; unit-tests without ROS.
    """
    if floor >= 1.0 or hi <= lo or fill <= lo:
        return 1.0
    if fill >= hi:
        return floor
    frac = (fill - lo) / (hi - lo)
    return 1.0 + (floor - 1.0) * frac


def _brake_axis(axis_writer, ema_pct: float, brake_gain: float,
                *, abort_fn=None, log=None, label: str = 'VIS') -> None:
    """Reverse-kick one translation axis to bleed exit momentum.

    ``ema_pct`` is the trailing EMA of the signed thrust the loop was
    commanding on this axis (% thrust, +/-). The kick is OPPOSITE that
    sign, magnitude ``brake_gain * |ema_pct|`` capped at
    ``VISION_BRAKE_CAP_PCT``, held for ``REVERSE_KICK_SEC``. A near-zero
    EMA (loop already tapered into the deadband -- gentle convergence /
    hole-lock) is gated out so the kick never disturbs a steady hull.

    Leaves the axis at the kick PWM; the caller goes neutral next (its
    normal exit path), so a separate settle is not needed here.
    """
    if abs(ema_pct) < VISION_BRAKE_MIN_PCT:
        return   # deadband exit -- no momentum worth braking
    mag = min(brake_gain * abs(ema_pct), VISION_BRAKE_CAP_PCT)
    kick = -math.copysign(mag, ema_pct)
    if log is not None:
        log.info(f'[{label:<5}] brake -- reverse {mag:.0f}% '
                 f'x {REVERSE_KICK_SEC:.2f}s (ema={ema_pct:+.0f}%)')
    # NOTE: on a lat+depth align, writers.lateral routes through
    # send_rc_override(lateral=) which drops Ch3 throttle from the 65535
    # ALT_HOLD release to 1500 for this REVERSE_KICK_SEC. That is intentional and
    # benign -- 1500 = "hold depth" in ALT_HOLD and matches the send_neutral the
    # caller issues immediately after; do NOT "fix" it back to a release.
    axis_writer(Pixhawk.percent_to_pwm(kick))
    _interruptible_sleep(REVERSE_KICK_SEC, abort_fn)


def _fill(sample, mode: str) -> float:
    """Bounding-box fill fraction of the frame.

    'area'   -> geometric mean sqrt(w*h)  (1.0 when bbox fills frame)
    'width'  -> w_frac                     (wide targets: gate bar)
    'height' -> h_frac                     (tall targets: slalom pipe)
    """
    if mode == 'width':
        return sample.w_frac
    if mode == 'height':
        return sample.h_frac
    return (sample.w_frac * sample.h_frac) ** 0.5   # 'area'


def _present(sample) -> bool:
    """True when this tick has a usable detection."""
    return sample is not None and sample.age_s <= _STALE_LIMIT_S


# `_is_srot` now lives in `motion_writers.is_srot` -- `make_writers` has to
# branch on it too (B30), and a second copy is how the two would disagree.
_is_srot = is_srot


def _tick(vision_state, fc) -> None:
    """Wait for the next observation, or the tick timeout -- whichever first.

    THE LOOP USED TO SLEEP A FIXED PERIOD. Against an asynchronous producer
    that costs, on average, half a period of pure waiting for data that has
    already arrived: detections land ~77 Hz (13 ms) and the srot loop ticked
    at 50 Hz (20 ms), so every command was computed from an observation up to
    13 ms staler than the one available -- about a third of the entire
    detection age, and the same defect that a fixed-rate timer caused in
    `camera_node`, one layer down.

    The timeout is the FLOOR, not the rate: it keeps the loop's time-based
    work running -- freshness decay, hold timing, the arrival brake, the
    deadline -- when nothing is being detected at all. So the loop can only
    get faster, never slower, and the lost/searching path behaves exactly as
    it did.

    Falls back to a plain sleep when there is no VisionState (unit tests, the
    non-vision callers), so nothing depends on the wake-up existing.
    """
    period = 1.0 / _loop_hz(fc)
    waiter = getattr(vision_state, 'wait_for_sample', None)
    if waiter is None:
        time.sleep(period)
        return
    waiter(period)


def _loop_hz(fc) -> float:
    """Tick rate for THIS backend. See motion_rates for why they differ."""
    return VISION_LOOP_HZ_SROT if _is_srot(fc) else LOOP_HZ


def _srot_drive(fc, *, fwd_pct: float, lat_pct: float, yaw_pct: float) -> None:
    """Write one MANUAL_CONTROL frame for the srot board.

    WHY THIS IS NOT `PixhawkFC.manual()` FOR BOTH BACKENDS. The HAL's `manual()`
    exists on both, but the ArduSub one maps onto `send_rc_override` with all
    four channels written -- it cannot express the two RELEASE semantics this
    loop depends on:

      * `release_yaw` releases Ch4 (65535) so the background `HeadingLock` owns
        yaw. Writing yaw=1500 instead races the lock's own stream, which is the
        fight the `_drive` docstrings below already warn about.
      * `throttle_ch = 65535` releases Ch3 so ArduSub's ALT_HOLD owns depth.

    So the ArduSub path is left EXACTLY as it was -- it is the configuration
    that placed 8th at RoboSub 2025 -- and srot gets its own branch.

    On srot both releases have a different and simpler answer: there is no host
    heading lock (`lock_heading` is refused on this backend), the board holds
    attitude and heading itself at 500 Hz in STABILIZE, and MANUAL_CONTROL has
    no "release" -- every frame carries all four axes. So `release_yaw` becomes
    "command zero yaw and let the board hold", which is the same intent through
    a better mechanism.

    `up` is always 0. The depth axis needs `set_target_depth`, which SrotFC does
    not implement; `vision_verbs` refuses a depth-axis align on this backend
    rather than letting it silently do nothing.
    """
    fc.manual(fwd=fwd_pct / 100.0, lat=lat_pct / 100.0,
              up=0.0, yaw=yaw_pct / 100.0)


def _read_depth(pixhawk) -> float:
    att = pixhawk.get_attitude()
    return float(att['depth']) if att else 0.0


def _vision_yaw_floor(epx: float, eff_err: float, full_pct: float) -> float:
    """Stiction-breaking yaw floor (%), tapered across the approach band.

    Full ``full_pct`` at/above ``eff_err + VISION_YAW_APPROACH_BAND_PX``, then
    linearly to 0 at the ``eff_err`` deadband edge, so a small residual near the
    lock eases in instead of a hard min-PWM relay limit-cycling the hull on the
    Ch4 rate channel (the close-in yaw wobble). Mirrors heading_lock._lock_floor
    and motion_yaw._yaw_floor. Pure; only meaningful for ``epx > eff_err`` (inside
    the deadband the loop commands 0 and never calls this).
    """
    band = VISION_YAW_APPROACH_BAND_PX
    if band <= 0.0 or epx >= eff_err + band:
        return full_pct
    frac = (epx - eff_err) / band       # 1.0 at band edge -> 0 at deadband edge
    return full_pct * max(0.0, frac)


def _camera_ready(vision_state) -> bool:
    """True once the camera pipeline has published a CameraInfo.

    Uses ``info_seen()`` when available (the real VisionState); falls back
    to a positive image size so lightweight test doubles still work. Until
    this is True the controller has no trustworthy pixel scale, so the
    verbs return NO_CAMERA rather than steer on mis-scaled pixel error.
    """
    seen = getattr(vision_state, 'info_seen', None)
    if callable(seen):
        return bool(seen())
    width, height = vision_state.image_size()
    return width > 0 and height > 0


def _live_classes(vision_state):
    """Class ids the detector is currently publishing (for the no-match hint)."""
    fn = getattr(vision_state, 'list_classes', None)
    return fn() if callable(fn) else []


# ---------------------------------------------------------------------- #
#  align_loop -- centre on lat / yaw / depth                             #
# ---------------------------------------------------------------------- #
def align_loop(*,
               pixhawk: Pixhawk,
               vision_state,
               target_class: str,
               axes: Set[str],
               offsets: Dict[str, float],
               err_px: float,
               duration: float,
               gain: float,
               gain_lat: Optional[float] = None,
               gain_yaw: Optional[float] = None,
               brake: bool = True,
               brake_gain: float = VISION_BRAKE_GAIN,
               hold_s: float = 0.0,
               kp_lat: float = KP_LAT_DEFAULT,
               kp_yaw: float = KP_YAW_DEFAULT,
               kp_depth: float = KP_DEPTH_DEFAULT,
               lost_grace_s: float = 1.0,
               hold_through_loss: bool = False,
               align_stable_frames: int = 3,
               depth_sign: int = +1,
               release_yaw: bool = False,
               lock_on: bool = False,
               ctrl_conf: float = 0.0,
               range_gain_floor: float = 1.0,
               ki_lat: float = 0.0,
               i_lat_max: float = VISION_I_LAT_MAX,
               coast_s: float = 0.0,
               lock_s: float = 0.0,
               fwd_fill: float = 0.0,
               fwd_mode: str = 'area',
               kp_forward: float = KP_FORWARD_DEFAULT,
               settle_px: float = SETTLE_PX_DEFAULT,
               depth_step: float = _MAX_DEPTH_NUDGE,
               downward: bool = False,
               surge_sign: int = +1,
               max_depth_m: float = 0.0,
               depth_ceiling_m: float = 0.0,
               on_locked=None,
               fire_t: float = 0.0,
               fire_pass: bool = False,
               fire_max_tilt_deg: float = 0.0,
               tilt_gate_fn=None,
               report_fn=None,
               writers=None,
               log=None,
               abort_fn=None) -> Outcome:
    """Hold ``target_class`` at the requested pixel offset on each active axis.

    ``axes`` is a subset of {'lat','yaw','depth'}; ``offsets`` carries the
    signed pixel offset for each active axis (0 = centre). Returns an
    Outcome -- never raises on a miss.

    ``hold_s`` > 0 turns the verb into an ACTIVE station-keep: once centred,
    the loop keeps running its lat/yaw/depth corrections for hold_s seconds
    (fighting water inertia) before exiting ALIGNED, instead of exiting on the
    first stable tick. hold_s counts against ``duration`` -- budget
    duration >= approach + hold_s or the verb TIMEOUTs mid-hold.

    ``on_locked`` (if given) is called AT MOST ONCE, on the first STABLY-ALIGNED
    tick at or after ``fire_t`` seconds into the hold window (measured from the
    first stable tick), BEFORE the hold-exit check -- so a payload fire fires
    mid-hold while the loop is still correcting, not on the drifting exit tick.
    It is gated on the target being in-band: if alignment is never held during
    the window, ``on_locked`` is NOT called (a torpedo never fires off-target).
    The caller is expected to make ``on_locked`` non-blocking (it spawns the fire
    on a background thread); the 20 Hz loop must not stall. ``fire_t`` should be
    < hold_s (the verb clamps it upstream). The fire is additionally gated on a
    FRESH detection (``is_new_frame and not sample.coasted``) -- it leaves only on
    the tick a genuinely new live box lands, so it is FPS-robust (fires promptly at
    low detector rate) yet never fires on a frozen detector's stale frame or a
    Kalman-coasted box.

    ``fire_pass`` (opt-in, default off): if ``on_locked`` never fired a strict
    in-band shot, fire it anyway on a NATURAL exit (TIMEOUT / hold-complete) as long
    as the target was seen LIVE within ``lost_grace_s`` -- a guaranteed partial-
    points shot when full alignment was not reached. Never fires when the target was
    never seen or only coasted.

    ``depth_step`` -- per-UPDATE depth-setpoint resolution (m). The depth axis steps
    the ArduSub ALT_HOLD setpoint by AT MOST this each 5 Hz update and FREEZES it
    inside the deadband, so ArduSub settles between steps (no z-wobble). 0.02 slow ..
    0.10 coarse -- the SOLE depth-rate knob (depth has no % speed cap like lat/yaw;
    to drop the depth axis entirely, omit 'depth' from ``axes``).

    ``downward`` -- DOWNWARD-CAMERA FRAME REMAP (bottom-mounted cam, looks at the
    pool floor). The bin task hovers ABOVE the target, so the body axes rotate:
      * ``lat`` (image-X) -> Ch6 lateral strafe  -- SAME as forward.
      * ``depth`` (image-Y) -> Ch5 SURGE fore/aft -- REMAPPED. Two-sided P-on-pixel
        modelled on the lat axis (rgain + freshness-decay + arrival reverse-kick
        brake), NOT the depth setpoint. Ch5 is open-loop timed thrust so it MUST be
        braked or the 20 kg hull coasts past the bin. ``surge_sign`` flips its
        polarity for the physical mount (verify DISARMED with vision_thrust_check --
        a wrong sign is positive feedback that drives the hull AWAY from the bin).
      * ``fwd_fill`` (optional) -> DEPTH DESCENT = the "approach" (get closer to the
        bin for the drop). Driven by the SAME depth_step logic as the forward depth
        axis: PROPORTIONAL to the fill deficit, capped at ``depth_step`` m/update,
        deadband-frozen at the fill target. ONE-SIDED (descends only -> can't
        surface). Bounded shallow by ``depth_ceiling_m`` (surface guard) and deep by
        ``max_depth_m`` (floor). 0 = no vision depth; ArduSub holds ``set_depth`` (the
        minimum-viable bin path is lat + surge + set_depth hold + drop).
      * fire -> dropper (channels 3/4) via ``on_locked`` -- same mechanism as torpedo.
    ``depth_ceiling_m`` (< 0) is the shallowest setpoint allowed on ANY depth motion
    (defaults to _MIN_DEPTH_M); a bin mission sets e.g. -0.4 so alignment can't
    surface the hull. yaw is released to ``heading_lock`` (no vision-yaw on the bin).
    ``move_loop`` is meaningless on a downward camera (surging doesn't grow fill);
    the DSL guards ``move(camera='downward')``.

    ``report_fn`` (if given) is called every PRESENT tick with the signed
    from-centre pixel offset ``(x_off, y_off)`` of the target -- a live-telemetry
    sink for action feedback. It stays rclpy-free (just a callable); the caller
    wires it to a shared slot the manager's feedback pump reads.

    Precision knobs (all default to "no change"):
      * ``lock_on`` -- after the target is acquired, steer to the detection
        NEAREST the last-accepted centre within ``VISION_LOCK_GATE_NORM`` (not the
        largest box), so a second hole / spurious box can't steal the aim. Resets
        to largest-area acquisition after a real loss (LOST exit).
      * ``ctrl_conf`` -- control-side minimum detection score to accept a box.
      * ``range_gain_floor`` -- scales lat/depth kp DOWN as the bbox fills the
        frame (close) to stop the close-in overshoot (1.0 = off).
      * ``ki_lat`` -- lateral integral gain; cancels the steady-current offset of
        the open-loop Ch6 axis, accumulated only during the hold (0 = off).
      * ``settle_px`` -- settle gate (0 = off). When > 0, a tick only counts toward
        the stable-frame exit if the worst error is in-band AND barely moving
        (|Δworst| <= settle_px) -- so align exits SETTLED on target (like move's
        continuously-held lateral) instead of mid-pass through the band. Keyed on
        error velocity, so a steady current does not block it (that is ``ki_lat``'s
        job). Use it when an align must END accurate (terminal locks).

    Forward range-hold axis (``fwd_fill`` > 0; the unified standoff shot):
      align ALSO drives forward toward ``fwd_fill`` (fraction of frame, measured by
      ``fwd_mode``), ONE-SIDED -- drive while the bbox is smaller than the standoff,
      neutral at/past it. Forward joins the in-band stable/fire gate, so a mid-hold
      fire only leaves once lat/depth AND the standoff range are all satisfied. With
      ``fwd_fill`` = 0 (default) there is no forward axis and behaviour is unchanged.
    """
    bad = axes - VALID_AXES
    if bad:
        raise ValueError(f"align_loop: unknown axes {sorted(bad)}")
    if not axes:
        raise ValueError("align_loop: at least one axis required")

    if not _camera_ready(vision_state):
        return Outcome(NO_CAMERA, "camera pipeline not up (no camera_info)",
                       elapsed_s=0.0)
    width, height = vision_state.image_size()
    half_w, half_h = width * 0.5, height * 0.5

    # Per-axis speed caps: each falls back to the global `gain` when unset, so
    # `align(gain=30)` caps every axis at 30% while `align(gain=40, yaw_gain=15)`
    # slows only yaw. (Mirrors the kp_* `or DEFAULT` idiom at the call site.)
    g_lat   = gain if gain_lat   is None else gain_lat
    g_yaw   = gain if gain_yaw   is None else gain_yaw
    g_fwd   = gain                       # forward range-hold capped by the global gain
    use_fwd = float(fwd_fill) > 0.0      # optional forward standoff / downward-descent axis

    # SIGN GUARD: depths are NEGATIVE metres below the surface, so max_depth_m (floor) and
    # depth_ceiling_m (surface guard) must be < 0. A POSITIVE value is a sign error -- it
    # silently reads as OFF (eff_ceiling below falls back to _MIN_DEPTH_M; the descent
    # fail-safe requires max_depth_m < 0), which mis-leads the operator into thinking they
    # set a floor. Warn loudly instead of failing silently; the values still read as OFF.
    if float(max_depth_m) > 0.0 or float(depth_ceiling_m) > 0.0:
        log.warning(
            f"[VIS  ] depth bound is POSITIVE (max_depth_m={float(max_depth_m):+.2f} "
            f"depth_ceiling={float(depth_ceiling_m):+.2f}) -- depths are NEGATIVE metres; "
            f"treating positive as OFF. Pass NEGATIVE metres to bound the descent.")

    # FAIL-SAFE: on a downward camera the fwd_fill axis DESCENDS (one-sided, deeper)
    # toward the fill target. Without a deep floor (`max_depth_m` < 0) an unreachable
    # fill would drive the hull into the pool floor. Require the floor to enable the
    # descent; otherwise drop the fill axis and just hold ArduSub depth. (Forward
    # cameras are unaffected -- there fwd_fill is a Ch5 standoff, not a descent.)
    if downward and use_fwd and float(max_depth_m) >= 0.0:
        log.warning("[VIS  ] downward fill->depth descent needs max_depth_m<0 "
                    "(deep floor) -- ignoring fwd_fill, holding ArduSub depth")
        use_fwd = False

    use_depth   = 'depth' in axes
    # DOWNWARD: the 'depth' axis drives Ch5 SURGE (image-Y), not the depth setpoint;
    # ArduSub's ALT_HOLD owns Ch3 (mission set_depth), and the optional fwd_fill axis
    # descends that setpoint. FORWARD: the 'depth' axis drives the depth setpoint.
    use_surge   = use_depth and downward          # image-Y -> Ch5 fore/aft
    use_vdepth  = use_depth and not downward      # image-Y -> depth setpoint (forward)
    # Stream the depth setpoint whenever a vision axis owns depth (forward 'depth' axis)
    # OR we are on the DOWNWARD camera. THE DOWNWARD BUG: previously this streamed only on
    # a downward *descent* (use_fwd), so a downward SURGE-only align (lat + Ch5 surge, no
    # descent -- the bin task) released Ch3 (65535) while streaming NOTHING. Nothing then
    # asserted depth-hold and the negatively-buoyant hull sank to the floor, ignoring
    # set_depth. Streaming the (constant, when there is no descent deficit) setpoint here
    # holds depth via ArduSub's position controller -- the SAME proven mechanism the
    # forward torpedo-standoff depth axis already uses. Release Ch3 exactly when we stream
    # (so ArduSub's depth PID is the sole Ch3 consumer); otherwise hold neutral 1500.
    stream_depth = use_vdepth or downward
    throttle_ch = 65535 if stream_depth else 1500
    # depth_step is the per-UPDATE setpoint resolution (m): the depth axis moves the
    # ArduSub ALT_HOLD setpoint by AT MOST this each 5 Hz update, so max slew =
    # depth_step * DEPTH_HZ (0.1 m/s at the 0.02 default). It is the operator's depth-
    # rate knob (0.02 fine/slow .. 0.10 coarse) -- a metres cap, NOT a % like lat/yaw
    # (depth has no per-axis % gain). Slower + stepped + deadband-frozen lets ArduSub's
    # ALT_HOLD PID actually settle between steps instead of chasing a setpoint that
    # jitters with the bbox (the z-wobble). At the engine level max_nudge=0 would
    # freeze depth entirely, but the verb coerces an unset (0.0) depth_step to the
    # default -- to skip the depth axis, omit 'depth' from ``axes``.
    max_nudge   = max(float(depth_step), 0.0)
    # Shallowest depth the setpoint may reach (negative m). Guards the DOWNWARD path
    # especially: ratio/alignment math must never drive the hull toward the surface.
    # Defaults to the global _MIN_DEPTH_M when unset (depth_ceiling_m >= 0), so the
    # forward path is byte-unchanged; a mission passes e.g. -0.4 to keep the bin run
    # safely submerged.
    eff_ceiling = float(depth_ceiling_m) if float(depth_ceiling_m) < 0.0 else _MIN_DEPTH_M

    depth_setpoint = _read_depth(pixhawk)

    def _drive(lat_pct: float, yaw_pct: float, fwd_pct: float = 0.0) -> None:
        """Write the translation/yaw RC frame, honouring an active lock.

        When ``release_yaw`` is set the background heading lock owns Ch4,
        so touch only throttle/forward/lateral and leave yaw released --
        writing yaw=1500 here would race the lock's 20 Hz Ch4 stream
        (the same fight ``move_loop`` avoids via ``send_rc_translation``).

        ``fwd_pct`` is the optional forward range-hold command (Ch5); it is 0
        unless the forward standoff axis is active (``fwd_fill`` > 0).
        """
        if _is_srot(pixhawk):
            # release_yaw -> command 0 yaw; the board holds heading at 500 Hz.
            _srot_drive(pixhawk, fwd_pct=fwd_pct, lat_pct=lat_pct,
                        yaw_pct=0.0 if release_yaw else yaw_pct)
        elif release_yaw:
            pixhawk.send_rc_translation(
                throttle=throttle_ch, forward=Pixhawk.percent_to_pwm(fwd_pct),
                lateral=Pixhawk.percent_to_pwm(lat_pct))
        else:
            pixhawk.send_rc_override(
                forward=Pixhawk.percent_to_pwm(fwd_pct),
                lateral=Pixhawk.percent_to_pwm(lat_pct),
                yaw=Pixhawk.percent_to_pwm(yaw_pct),
                throttle=throttle_ch)

    def _pass_fire() -> None:
        """fire_pass fallback: on a NATURAL exit (TIMEOUT / hold-complete) without a
        strict mid-hold fire, actuate the payload anyway so a detected-but-not-fully-
        aligned run still scores. Gated on a LIVE (non-coasted) sighting within
        lost_grace_s -- never fires into empty water or on a Kalman ghost. One-shot
        (guarded by ``fired``). Opt-in via fire_pass; no-op when off."""
        nonlocal fired
        if (not fire_pass or on_locked is None or fired
                or (time.monotonic() - last_live_at) > lost_grace_s):
            return
        fired = True
        log.info("[VIS  ] fire_pass -- target seen; firing at command end "
                 "(alignment not required)")
        try:
            on_locked()
        except Exception as exc:   # noqa: BLE001 -- fire must not kill the loop
            log.error(f"[VIS  ] on_locked (fire_pass) raised {exc!r}")

    stable      = 0
    lost_since: Optional[float] = None
    aligned_at: Optional[float] = None   # monotonic of FIRST stable -> hold-window start
    fired       = False  # on_locked fired once at fire_t into the hold (payload mid-hold)
    _sq_warned  = [False]  # one line per command, not one per tick
    last_log    = 0.0
    last_depth  = 0.0
    depth_ctrl  = 0.0    # depth axis error carried from axis-calc into the 5 Hz step
    depth_epx   = 0.0
    last_err_px = float('inf')
    last_fill   = 0.0    # bbox fill on the forward axis (0 unless use_fwd) -> Outcome.fill
    prev_worst: Optional[float] = None   # settle gate: worst error last NEW frame (for |Δworst|)
    last_frame_at = float('-inf')        # arrival time of the last COUNTED detection frame
    stable_since  = float('inf')         # arrival of the FIRST frame of the current in-band run
    last_live_at  = float('-inf')        # monotonic of the last LIVE (non-coasted) sighting -> fire_pass gate
    end_x_px    = math.nan  # signed from-centre px of target at last seen frame
    end_y_px    = math.nan
    lat_ema     = 0.0   # trailing EMA of the signed lateral command -> brake proxy
    surge_ema   = 0.0   # downward: trailing EMA of the signed Ch5 surge command -> brake proxy
    fill_deficit = 0.0  # downward fill->depth: (target_fill - fill), carried into the 5 Hz step
    lat_i       = 0.0   # lateral integral accumulator (Layer 2; 0 unless ki_lat>0)
    loop_hz     = _loop_hz(pixhawk)          # backend-dependent; see motion_rates
    # Nominal tick. Used to SEED the measured dt below and as its ceiling;
    # the loop no longer sleeps a fixed period (see `_tick`), so treating this
    # as the real interval would misstate both the lock gate and the integral
    # term the moment the loop starts running at detection rate.
    dt          = 1.0 / loop_hz
    _dt_nominal = dt
    _last_pass  = time.monotonic()
    # Observed detection interval, EMA. The freshness thresholds scale with
    # it, because "~1 frame" is 13 ms on the forward camera and 32 ms on the
    # downward one and a fixed constant cannot be both. 0 until two distinct
    # frames have been seen, which yields the floors -- the unscaled
    # behaviour -- rather than a wild guess from one sample.
    det_interval = 0.0
    # Observed pipeline latency: the sample's age at the instant it FIRST
    # appears, i.e. capture -> the loop seeing it. EMA'd, and only sampled on
    # a genuinely new frame -- a re-read's age has grown by however long the
    # loop took and would inflate it without bound.
    pipe_age = 0.0
    # Seeded at the control period -- the shortest gap possible, so the gate
    # starts at its tightest and widens to the real detection interval once one
    # has been observed. Erring tight before acquisition is safe: there is no
    # lock to drop yet.
    gate_norm   = _lock_gate(dt) if lock_on else 0.0
    last_accept_t: Optional[float] = None
    locked_ex: Optional[float] = None        # last-accepted centre -> continuity lock
    locked_ey: Optional[float] = None
    locked_id   = -1     # tracker id of the locked target -> coast follows this id
    saw_target  = False  # True once any frame yields the target -> distinguishes
                         #   "never detected" (wrong model/classes/view) from
                         #   "seen but couldn't converge" at exit.

    # Effective deadband: clamp to a physical floor so an over-tight err can't
    # perpetually TIMEOUT on bbox jitter. err_px here is already post-coercion
    # (commands.fields_for turned a rosidl-0 "unset" into the default/param), so
    # this only ever floors a genuinely tiny positive request -- logged, never
    # silent, so 'aligned (Npx)' is always read against the real deadband.
    eff_err = max(float(err_px), MIN_ALIGN_ERR_PX)
    floored = eff_err > float(err_px)
    fwd_note = f" fwd>={fwd_fill * 100:.0f}%({fwd_mode})" if use_fwd else ""
    log.info(
        f"[VIS  ] align class={target_class!r} axes={sorted(axes)}{fwd_note} "
        f"err={eff_err:.0f}px"
        f"{' (floored from %.0f)' % err_px if floored else ''} "
        f"gain={gain:.0f}% dur={duration:.0f}s hold={hold_s:.0f}s")
    if downward:
        # Loudly announce the rotated frame so the operator never mistakes a
        # downward align's axis meanings for the forward ones (see docstring).
        fill_note = (f", fill->depth descend to {fwd_fill*100:.0f}% "
                     f"@ depth_step={max_nudge:.02f}m (ceil {eff_ceiling:.1f}m, "
                     f"floor {max_depth_m:.1f}m)" if use_fwd else
                     f", depth held by ArduSub (no fill axis; ceil {eff_ceiling:.1f}m)")
        log.info(f"[VIS  ] downward frame: image-X->lat Ch6, image-Y->surge Ch5 "
                 f"(sign {surge_sign:+d}){fill_note}")

    started  = time.monotonic()
    deadline = started + max(duration, 0.0)
    try:
        while True:
            now     = time.monotonic()
            # MEASURED, not assumed. The loop wakes on a new detection now, so
            # the interval is the detection interval when targets are visible
            # and the tick timeout when they are not. `dt` feeds the
            # continuity-lock gate and the lateral integral; holding it at the
            # nominal period would overstate both by up to 1.5x once the loop
            # starts running at 77 Hz instead of 50.
            #
            # Clamped: a debugger pause or a scheduling stall must not inject a
            # huge dt into an integrator. The floor keeps a burst of two
            # detections in the same millisecond from collapsing it to zero.
            dt = min(max(now - _last_pass, 1e-3), _dt_nominal * 4.0)
            _last_pass = now
            elapsed = now - started
            if abort_fn and abort_fn():
                return Outcome(ABORTED, "aborted", last_err_px, last_fill, elapsed,
                               end_x_px, end_y_px)
            if now >= deadline:
                reason = ("not aligned (duration elapsed)" if saw_target else
                          f"target {target_class!r} NEVER detected -- check "
                          f"model/classes/camera view")
                _pass_fire()   # opt-in guaranteed shot if the target was seen live
                return Outcome(TIMEOUT, reason, last_err_px, last_fill, elapsed,
                               end_x_px, end_y_px)

            # Continuity lock: once acquired, prefer the box NEAREST the last
            # centre (within gate_norm) over the largest, so a 2nd hole / spurious
            # box can't steal the aim. near=None (pre-acquire) or gate_norm=0
            # (lock_on off) -> largest-area, unchanged. ctrl_conf gates low-score
            # boxes out of the control target.
            near = (locked_ex, locked_ey) if locked_ex is not None else None
            sample = vision_state.bbox_error(
                target_class, near=near, gate_norm=gate_norm, min_score=ctrl_conf,
                locked_id=locked_id, coast_s=coast_s, lock_s=lock_s)
            if not _present(sample):
                stable = 0
                lat_i = 0.0   # bleed integral windup while blind
                _drive(0.0, 0.0)
                # Keep asserting depth-hold while blind on ANY streaming path (forward
                # depth axis OR any downward align) -- _drive released Ch3 (65535), so
                # without this the hull would sink during a target dropout. Gated on
                # stream_depth (not use_depth) so a downward surge-only/lat-only align
                # keeps holding through the loss too.
                if stream_depth:
                    pixhawk.set_target_depth(depth_setpoint)
                if lost_since is None:
                    lost_since = now
                if (not hold_through_loss) and (now - lost_since) >= lost_grace_s:
                    reason = (f"target {target_class!r} lost" if saw_target else
                              f"target {target_class!r} NEVER detected -- check "
                              f"model/classes/camera view")
                    return Outcome(LOST, reason, last_err_px, last_fill, elapsed,
                                   end_x_px, end_y_px)
                if (now - last_log) >= LOG_THROTTLE_S:
                    live = _live_classes(vision_state)
                    if live:
                        log.debug(
                            f"[VIS  ] align: {target_class!r} not among live "
                            f"detections {live} -- check classes filter / model")
                    else:
                        log.debug(f"[VIS  ] align LOST {now - lost_since:.1f}s "
                                  f"(grace {lost_grace_s:.1f}s)")
                    last_log = now
                _tick(vision_state, pixhawk)
                continue

            saw_target = True
            lost_since = None
            if not sample.coasted:
                last_live_at = now   # last LIVE sighting -> fire_pass recency gate
            # Re-arm the continuity lock on the accepted box (used as `near` next
            # tick) ONLY when lock_on -- otherwise leave near=None so selection
            # stays largest-area. After a real loss the loop exits LOST, so a
            # fresh verb call re-acquires largest -- no stale lock survives.
            if lock_on:
                locked_ex, locked_ey = sample.ex, sample.ey
            # Capture the tracker id of a LIVE box so a later gap coasts the right
            # target (VisionState only fills track_id when coast_s>0; a coasted
            # sample keeps the existing id). No-op when coasting is off.
            if not sample.coasted and sample.track_id >= 0:
                locked_id = sample.track_id
            # Range-adaptive gain: soften lat/depth kp as the bbox fills the frame
            # (close) so the 20 kg hull doesn't overshoot a small target.
            rgain = _range_gain(_fill(sample, 'area'), range_gain_floor)
            # Signed from-centre offset of the target THIS tick: the end-position
            # the verb returns, and the live value report_fn streams to feedback.
            end_x_px = sample.ex * half_w
            end_y_px = sample.ey * half_h
            if report_fn is not None:
                report_fn(end_x_px, end_y_px)
            yaw_pct = lat_pct = p_lat = fwd_pct = 0.0
            in_band = []
            worst   = 0.0

            if 'lat' in axes:
                ctrl = sample.ex - offsets.get('lat', 0.0) / half_w
                epx  = abs(ctrl) * half_w
                worst = max(worst, epx)
                p_lat = ctrl * kp_lat * rgain
                # Lateral integral (Layer 2): Ch6 is open-loop, so only the
                # integral nulls a steady-current offset. Accumulate ONLY during
                # the hold (aligned_at set), conditional on the output not being
                # saturated (anti-windup), clamped. ki_lat=0 -> exactly P.
                if ki_lat > 0.0 and aligned_at is not None \
                        and abs(p_lat + lat_i) < g_lat:
                    lat_i = _clamp(lat_i + ki_lat * ctrl * dt,
                                   -i_lat_max, i_lat_max)
                lat_pct = _clamp(p_lat + lat_i, -g_lat, g_lat)
                in_band.append(epx <= eff_err)

            if 'yaw' in axes:
                ctrl = sample.ex - offsets.get('yaw', 0.0) / half_w
                epx  = abs(ctrl) * half_w
                worst = max(worst, epx)
                # Polarity: un-negated, same as the lateral axis (ex > 0 ->
                # target RIGHT -> yaw RIGHT). Inside the err_px deadband we
                # command 0 (let the hull settle, no shot-jitter when the
                # mission fires in-band). Outside it, pure proportional capped
                # by g_yaw -- and ONLY when the bbox is large (target close) do
                # we floor the magnitude to VISION_YAW_MIN_PCT to break T200
                # stiction for the tight hole-lock. Far away (small bbox) the
                # floor is suppressed: a hard minimum on a rate channel is a
                # relay that limit-cycles the hull, which is the far-field
                # wobble. See VISION_YAW_FLOOR_FILL.
                if epx <= eff_err:
                    yaw_pct = 0.0
                else:
                    mag = min(abs(ctrl * kp_yaw), g_yaw)
                    # Stiction floor only when close (large bbox), and TAPERED across
                    # the approach band above the deadband so it eases to 0 at the
                    # edge instead of a hard min-PWM relay that limit-cycles the hull
                    # on Ch4 (the close-in yaw wobble). Mirrors the heading_lock /
                    # motion_yaw taper -- see _vision_yaw_floor.
                    if _fill(sample, 'area') >= VISION_YAW_FLOOR_FILL:
                        mag = max(mag, _vision_yaw_floor(
                            epx, eff_err, min(VISION_YAW_MIN_PCT, g_yaw)))
                    yaw_pct = math.copysign(mag, ctrl)
                in_band.append(epx <= eff_err)

            if use_surge:
                # DOWNWARD: the 'depth' axis (image-Y) drives Ch5 SURGE fore/aft.
                # Two-sided P modelled on the lat axis (same kp/cap/rgain), so the
                # hull drives forward when the bin is ahead and BACK when behind --
                # the one-sided fwd/fill law can't back up and would never centre.
                # surge_sign flips polarity for the physical mount (verify disarmed).
                ctrl = sample.ey - offsets.get('depth', 0.0) / half_h
                epx  = abs(ctrl) * half_h
                worst = max(worst, epx)
                p_surge = ctrl * kp_lat * rgain
                fwd_pct = _clamp(p_surge, -g_fwd, g_fwd) * surge_sign
                in_band.append(epx <= eff_err)
            elif use_vdepth:
                ctrl = sample.ey - offsets.get('depth', 0.0) / half_h
                epx  = abs(ctrl) * half_h
                worst = max(worst, epx)
                # Carry the depth error into the 5 Hz setpoint step below (do NOT
                # move the setpoint here every 20 Hz tick -- that gives ArduSub a
                # target sliding at up to max_nudge*LOOP_HZ and it never settles).
                depth_ctrl, depth_epx = ctrl, epx
                in_band.append(epx <= eff_err)

            if use_fwd and downward:
                # DOWNWARD: the fill axis drives a DEPTH DESCENT (get closer to the
                # bin for the drop), not Ch5. One-sided: descend while the bbox is
                # smaller than fwd_fill, hold at/past it. Carried into the 5 Hz depth
                # step (deep-floor bounded there). Never surges Ch5 here -- Ch5 is the
                # surge axis above.
                last_fill = _fill(sample, fwd_mode)   # reported in Outcome.fill
                fill_deficit = float(fwd_fill) - last_fill
                in_band.append(fill_deficit <= FWD_BAND)
            elif use_fwd:
                # FORWARD range-hold (the unified standoff shot). ONE-SIDED: drive
                # forward while the bbox is smaller than the standoff fill, command
                # exactly 0 once at/past it (never reverse -> no reverse-kick, no
                # ramming the board; water drag bleeds a small overshoot). The
                # standoff band joins in_band so the mid-hold fire / exit only
                # trips once the range is satisfied too. Same _fill law as move_loop.
                last_fill = _fill(sample, fwd_mode)   # reported in Outcome.fill
                fwd_err = float(fwd_fill) - last_fill
                if fwd_err <= FWD_BAND:
                    fwd_pct = 0.0
                    in_band.append(True)
                else:
                    fwd_pct = _clamp(fwd_err * kp_forward, 0.0, g_fwd)
                    in_band.append(False)

            last_err_px = worst
            # Freshness-decay: pace LATERAL authority to measurement freshness so
            # the loop doesn't blind-drive on a stale bbox between slow frames
            # (yaw/depth excluded -- ArduSub bleeds Ch4, holds depth). At healthy
            # FPS fresh==1.0 so this is a no-op. A COASTED sample decays on the
            # coast curve instead (gap decay), not freshness -- see _authority.
            fresh = _authority(sample, coast_s, det_interval, pipe_age)
            _warn_low_fps(log, fresh, sample)   # F3: surface FPS-starvation, don't stall silently
            lat_pct *= fresh
            fwd_pct *= fresh   # forward shares the freshness/coast decay (never braked)
            _drive(lat_pct, yaw_pct, fwd_pct)
            # Brake EMA tracks the PROPORTIONAL command only (a travel-momentum
            # proxy), NOT the full lat_pct: a hull holding STILL against a steady
            # current carries a nonzero integral (lat_i) but ~0 motion, so
            # including it would make the arrival brake reverse-kick a stationary
            # hull off the spot it was holding. Exclude lat_i here.
            lat_ema += _BRAKE_EMA_ALPHA * (p_lat * fresh - lat_ema)
            if use_surge:
                # Downward Ch5 surge is open-loop timed thrust (like lat/Ch6) -- track
                # its EMA so the arrival brake bleeds the fore/aft coast, or the hull
                # sails past the bin. fwd_pct already carries the signed, fresh-decayed
                # surge command, so the EMA is the true momentum proxy and the brake
                # reverse-kicks opposite it regardless of surge_sign.
                surge_ema += _BRAKE_EMA_ALPHA * (fwd_pct - surge_ema)
            if stream_depth and (now - last_depth) >= 1.0 / DEPTH_HZ:
                # Step the ALT_HOLD setpoint at 5 Hz (not 20) so the depth PID reaches
                # each step before the next: slow, stable, resolution = depth_step.
                if use_vdepth:
                    # FORWARD: image-Y drives the setpoint; FROZEN inside the deadband
                    # (no z-wobble). eff_ceiling keeps it never shallower than the
                    # surface guard; max_depth_m (when set <0) keeps it never DEEPER
                    # than the floor -- the forward axis is two-sided (a target below
                    # centre drives it deeper), so without this a persistent low target
                    # could walk the setpoint into the pool floor. Symmetric with the
                    # downward clamp below; opt-in (max_depth_m>=0 -> forward unchanged).
                    if depth_epx > eff_err:
                        step = _clamp(depth_ctrl * kp_depth * rgain,
                                      -max_nudge, max_nudge) * depth_sign
                        depth_setpoint = min(depth_setpoint - step, eff_ceiling)
                        if max_depth_m < 0.0:
                            depth_setpoint = max(depth_setpoint, max_depth_m)
                else:
                    # DOWNWARD. With a descent (use_fwd) this is the "approach" (get closer
                    # to the bin for the drop), driven by the SAME depth_step logic as the
                    # forward axis -- PROPORTIONAL to the fill deficit, capped at
                    # depth_step/update, and FROZEN inside the fill deadband (FWD_BAND) so it
                    # settles instead of chasing bbox jitter. ONE-SIDED (descends deeper only,
                    # never ascends -> can't surface). Bounded both ways: never shallower than
                    # eff_ceiling (surface guard), never deeper than max_depth_m (floor).
                    # WITHOUT a descent (surge-only / lat-only bin align) fill_deficit stays
                    # 0, so this branch does NOT step -- it re-streams the CONSTANT captured
                    # depth_setpoint below, holding set_depth (the downward depth-hold fix).
                    if fill_deficit > FWD_BAND:
                        step = min(fill_deficit, 1.0) * max_nudge   # decelerates as it nears
                        depth_setpoint = min(depth_setpoint - step, eff_ceiling)
                        if max_depth_m < 0.0:
                            depth_setpoint = max(depth_setpoint, max_depth_m)
                pixhawk.set_target_depth(depth_setpoint)
                last_depth = now

            # Settle gate (opt-in): require the hull to also be barely MOVING, not
            # just in-band, before a tick counts toward the stable-frame exit -- so
            # align ends settled on target (like move's held lateral) instead of
            # mid-pass through the band. Keyed on |Δworst| (error velocity), so a
            # steady current that holds a non-zero command at a perfect lock does
            # NOT block it. settle_px=0 -> settled always True -> legacy behaviour.
            # Distinct-detection gate: advance the stable-frame counter on new
            # DETECTIONS, not loop ticks (see _FRAME_EPS_S). sampled_at is the
            # frame's arrival time -- constant across re-reads of one detection,
            # so a re-read HOLDS the counter (neither advances nor resets) and
            # only a genuinely new frame moves it. This makes align_stable_frames
            # mean "N distinct in-band frames", FPS-independent, so a lucky single
            # frame can't declare ALIGNED (or arm the fire) at low detector FPS.
            sampled_at   = now - sample.age_s
            is_new_frame = sampled_at > last_frame_at + _FRAME_EPS_S
            if is_new_frame:
                if lock_on:
                    # Widen (or tighten) the gate to the interval actually
                    # observed between accepted frames, floored at the control
                    # period so a duplicate timestamp cannot collapse it to 0.
                    if last_accept_t is not None:
                        gate_norm = _lock_gate(
                            max(dt, sampled_at - last_accept_t))
                    last_accept_t = sampled_at
                pipe_age = (sample.age_s if pipe_age <= 0.0
                            else pipe_age + 0.2 * (sample.age_s - pipe_age))
                if last_frame_at > 0.0:
                    gap = sampled_at - last_frame_at
                    # Ignore absurd gaps: a dropout is not a slower camera,
                    # and letting one widen the interval would loosen the
                    # freshness thresholds exactly when the target is lost.
                    if 0.0 < gap <= 0.5:
                        det_interval = (gap if det_interval <= 0.0
                                        else det_interval + 0.2 * (gap - det_interval))
                last_frame_at = sampled_at
                if stable == 0:
                    # Start of a fresh in-band run. Stamped on the FRAME's own
                    # arrival, not on `now`: `now` is a control tick and at
                    # 50 Hz control over a 15 Hz camera the two differ by most
                    # of a frame period, which would credit the dwell with time
                    # the target was not actually in band.
                    stable_since = sampled_at
                # Settle gate compares against the previous NEW frame's error
                # (re-reads are numerically identical and would trivially pass),
                # so |Δworst| is a real cross-frame error velocity.
                settled = (settle_px <= 0.0 or prev_worst is None
                           or abs(worst - prev_worst) <= settle_px)
                prev_worst = worst
                stable = stable + 1 if (all(in_band) and settled) else 0
            # Both gates, for the reasons at _ALIGN_STABLE_MIN_S: enough
            # distinct frames AND enough elapsed time across them.
            if (stable >= align_stable_frames
                    and last_frame_at - stable_since >= _ALIGN_STABLE_MIN_S):
                # First confirmed-centred tick opens the hold window. With
                # hold_s>0 we keep the loop ALIVE and correcting for hold_s --
                # the ACTIVE station-keep the operator needs to hold steady
                # against water inertia while a payload fires (passive neutral
                # would just drift). hold_s<=0 keeps the original exit-on-stable
                # behaviour bit-for-bit.
                if aligned_at is None:
                    aligned_at = now
                    if hold_s > 0.0:
                        log.info(f"[VIS  ] align HELD -- station-keeping "
                                 f"{hold_s:.1f}s ({worst:.0f}px)")
                # Mid-hold fire: BEFORE the exit check so the payload actuates
                # while the loop is still correcting (not on the drifting exit
                # tick). on_locked is non-blocking (spawns a thread) so a slow
                # payload reconnect can't stall the 20 Hz station-keep. fire_t is
                # clamped < hold_s upstream, so this trips while still holding.
                # FRESHNESS GUARD: fire ONLY on the tick a genuinely NEW, non-coasted
                # detection lands (is_new_frame -- the same distinct-frame signal that
                # gates `stable`). This guarantees freshness at ANY detector FPS (a
                # new frame is by definition current), fixing the low-FPS miss where
                # the old `age_s <= 0.10s` window rarely coincided with the aligned
                # tick at 3-4 Hz. It is ALSO strictly safer: a FROZEN detector never
                # produces a new frame, so a stale cached box (or a Kalman-coasted
                # one) can never fire even while `stable` stands held at threshold
                # through a mid-hold freeze. A torpedo leaves on a real live sighting.
                fire_fresh = is_new_frame and not sample.coasted
                # AIM GATE (opt-in; 0.0 = off = byte-identical to before).
                # `fire_max_tilt_deg` is how far the TARGET'S FACE may be
                # tilted from square to our shot axis and still allow a shot --
                # not a vehicle attitude, and not a pixel error.
                # A round leaves along the hull's axis, so a centred box is not
                # enough -- fired 30 deg off-normal it misses an opening it was
                # perfectly centred on. `tilt_gate_fn` must account for the planar
                # FLIP AMBIGUITY (both branches inside tolerance), and it
                # answers False when there is no pose at all: for a firing gate
                # the fail-safe direction is DO NOT FIRE.
                #
                # Deliberately does NOT set `fired`: a refusal here is "not
                # yet", so a hull that squares up later in the hold still gets
                # its shot.
                square_ok = True
                if fire_max_tilt_deg > 0.0 and fire_fresh and not fired:
                    try:
                        square_ok = bool(tilt_gate_fn and tilt_gate_fn(fire_max_tilt_deg))
                    except Exception as exc:   # noqa: BLE001
                        log.error(f"[VIS  ] tilt_gate_fn raised {exc!r} -- refusing")
                        square_ok = False
                    if not square_ok and not _sq_warned[0]:
                        _sq_warned[0] = True
                        log.info(f"[VIS  ] fire HELD: target not square within "
                                 f"{fire_max_tilt_deg:.0f} deg (or no pose)")
                if on_locked is not None and not fired and fire_fresh and \
                        square_ok and \
                        (now - aligned_at) >= fire_t:
                    fired = True
                    try:
                        on_locked()
                    except Exception as exc:   # noqa: BLE001 -- fire must not kill the loop
                        log.error(f"[VIS  ] on_locked (fire) raised {exc!r}")
                if hold_s <= 0.0 or (now - aligned_at) >= hold_s:
                    # Arrival / hold complete: bleed lateral inertia so the hull
                    # stops square and the next mission step starts from the
                    # planned position. Gated on the EMA, so a gently-converged
                    # lock (hole-lock) exits with ~0 EMA and is NOT kicked.
                    # Yaw/depth never brake.
                    if brake and 'lat' in axes:
                        _brake_axis(writers.lateral, lat_ema, brake_gain,
                                    abort_fn=abort_fn, log=log, label='VBRK')
                    # Downward: also bleed the Ch5 SURGE coast (fore/aft) so the hull
                    # stops square over the bin instead of sailing past. Ch5 open-loop
                    # timed thrust coasts exactly like Ch6 -- same self-gating brake.
                    if brake and use_surge:
                        _brake_axis(writers.forward, surge_ema, brake_gain,
                                    abort_fn=abort_fn, log=log, label='VBRK')
                    writers.neutral()
                    # State the deadband next to the residual so 'aligned
                    # (Npx)' is never misread as "should have been 0" -- N is
                    # within the eff_err deadband by construction.
                    # If a mid-hold strict fire never landed (e.g. hold completed but
                    # freshness never coincided), fire_pass still actuates here on a
                    # live+recent sighting. No-op when off or already fired.
                    _pass_fire()
                    reason = (f"held {hold_s:.1f}s ({worst:.0f}/{eff_err:.0f}px)"
                              if hold_s > 0.0
                              else f"aligned ({worst:.0f}/{eff_err:.0f}px)")
                    return Outcome(ALIGNED, reason, worst, last_fill, elapsed,
                                   end_x_px, end_y_px)
                # else: inside the hold window -- fall through to the loop tail
                # and keep correcting (the per-tick _drive above already ran).
                # ponytail: aligned_at is set ONCE and never reset (unlike
                # move_loop's reached_at ~L660). Align drift is transient jitter
                # the loop corrects; resetting on a drift-out tick would risk
                # never completing the hold under a steady current. The in-band
                # gate on this block still guarantees we exit centred. Do NOT
                # "fix" this to match move.

            if (now - last_log) >= LOG_THROTTLE_S:
                # The detector node owns the always-on operator bearing line, so
                # this per-verb copy is demoted to debug to avoid a duplicate in
                # the mission terminal. Worded as live offset, not a verdict.
                x_off = sample.ex * half_w
                y_off = sample.ey * half_h
                log.debug(
                    f"[ offset lat={x_off:+.0f} depth={y_off:+.0f}px ] "
                    f"'{target_class}' -> err {worst:.0f}/{eff_err:.0f}px")
                last_log = now
            _tick(vision_state, pixhawk)
    finally:
        try:
            writers.neutral()
        except Exception as exc:   # noqa: BLE001
            log.warning(f"[VIS  ] align cleanup neutral raised: {exc!r}")


# ---------------------------------------------------------------------- #
#  move_loop -- drive forward to a bbox fill ratio                       #
# ---------------------------------------------------------------------- #
def move_loop(*,
              pixhawk: Pixhawk,
              vision_state,
              target_class: str,
              fwd_fill: float,
              mode: str = 'area',
              passthrough: bool = False,
              maintain_px: float = 0.0,
              maintain_on: bool = False,
              hold_s: float = 0.0,
              err_px: float = 40.0,
              duration: float = 20.0,
              gain: float = 30.0,
              gain_lat: Optional[float] = None,
              brake: bool = True,
              brake_gain: float = VISION_BRAKE_GAIN,
              kp_forward: float = KP_FORWARD_DEFAULT,
              kp_lat: float = KP_LAT_DEFAULT,
              lost_grace_s: float = 1.0,
              hold_through_loss: bool = False,
              release_yaw: bool = False,
              range_gain_floor: float = 1.0,
              coast_s: float = 0.0,
              lock_s: float = 0.0,
              report_fn=None,
              writers=None,
              log=None,
              abort_fn=None) -> Outcome:
    """Drive forward toward ``target_class`` -- stop at a fill ratio or pass through.

    Two modes:

    * **fill-stop** (default): drive forward, slowing as the bbox fills,
      until it reaches ``fwd_fill`` (measured by ``mode``); ``hold_s``
      station-keeps there before exiting.
    * **pass-through** (``passthrough=True``, selected by the DSL when
      ``move(fwd=None)``): drive forward at ``gain`` while the target is
      visible; once the target has been seen and then leaves the frame,
      keep driving for a commit window (``hold_s`` if given, else
      ``_PASSTHROUGH_COMMIT_S``) and report ALIGNED -- this carries the
      hull *through* the gate. If the target is never seen, falls back to
      the normal loss path (LOST -> mission fallback search).

    ``maintain_on`` holds a lateral pixel offset (``maintain_px``) while
    driving; depth and yaw are never commanded (ArduSub holds depth;
    heading lock or the autopilot holds yaw). Returns an Outcome -- never
    raises on a miss.

    ``report_fn`` (if given) is called every PRESENT tick with the signed
    from-centre pixel offset ``(x_off, y_off)`` -- the live-telemetry sink for
    action feedback (same contract as align_loop).
    """
    if mode not in VALID_MODES:
        raise ValueError(f"move_loop: mode must be one of {sorted(VALID_MODES)}")

    if not _camera_ready(vision_state):
        return Outcome(NO_CAMERA, "camera pipeline not up (no camera_info)",
                       elapsed_s=0.0)
    width, height = vision_state.image_size()
    half_w = width * 0.5
    half_h = height * 0.5

    # Lateral 'maintain' strafe gets its own cap; forward stays capped by `gain`.
    g_lat = gain if gain_lat is None else gain_lat

    def _drive(fwd_pct: float, lat_pct: float) -> None:
        if _is_srot(pixhawk):
            # move never commands yaw on either backend.
            _srot_drive(pixhawk, fwd_pct=fwd_pct, lat_pct=lat_pct, yaw_pct=0.0)
            return
        if release_yaw:
            pixhawk.send_rc_translation(
                forward=Pixhawk.percent_to_pwm(fwd_pct),
                lateral=Pixhawk.percent_to_pwm(lat_pct))
        else:
            pixhawk.send_rc_override(
                forward=Pixhawk.percent_to_pwm(fwd_pct),
                lateral=Pixhawk.percent_to_pwm(lat_pct),
                yaw=1500, throttle=1500)

    seen_once    = False
    lost_since:   Optional[float] = None
    reached_at:   Optional[float] = None
    commit_until: Optional[float] = None
    last_log     = 0.0
    last_fill    = 0.0
    last_lat_err = 0.0
    end_x_px     = math.nan   # signed from-centre px of target at last seen frame
    end_y_px     = math.nan
    fwd_ema      = 0.0   # trailing EMA of forward command -> brake proxy (fill-stop)
    lat_ema      = 0.0   # trailing EMA of the maintain strafe command -> brake proxy
    locked_id    = -1    # tracker id of the target -> coast follows this id

    # Pass-through commit window: hold_s overrides the module default.
    commit_s = hold_s if hold_s > 0.0 else _PASSTHROUGH_COMMIT_S

    log.debug(
        f"[VIS  ] move class={target_class!r} "
        f"{'PASS-THROUGH commit=%.1fs' % commit_s if passthrough else 'fwd_fill=%.0f%%' % (fwd_fill * 100)} "
        f"mode={mode} maintain={'%+.0fpx' % maintain_px if maintain_on else 'off'} "
        f"gain={gain:.0f}% dur={duration:.0f}s")

    started  = time.monotonic()
    deadline = started + max(duration, 0.0)
    # Observed detection interval, EMA -- the freshness thresholds scale with
    # it (see `_fresh_bounds`). `move_loop` has no distinct-frame machinery of
    # its own, so it derives arrivals from the sample's own capture time,
    # which is constant across re-reads of one detection.
    det_interval  = 0.0
    last_seen_at  = 0.0
    pipe_age      = 0.0
    try:
        while True:
            now     = time.monotonic()
            elapsed = now - started
            if abort_fn and abort_fn():
                return Outcome(ABORTED, "aborted", last_lat_err, last_fill, elapsed,
                               end_x_px, end_y_px)
            if now >= deadline:
                if not seen_once:
                    reason = (f"target {target_class!r} NEVER detected -- check "
                              f"model/classes/camera view")
                else:
                    reason = ("passed-through window not closed (duration elapsed)"
                              if passthrough else "fill not reached (duration elapsed)")
                return Outcome(TIMEOUT, reason, last_lat_err, last_fill, elapsed,
                               end_x_px, end_y_px)

            # Pass-through (fwd=None) must DETECT "target gone" to fire its commit
            # window, so it deliberately does NOT coast -- a coasted box would
            # keep present=True and delay the commit by up to coast_s. Fill-stop
            # moves coast normally.
            eff_coast = 0.0 if passthrough else coast_s
            sample  = vision_state.bbox_error(
                target_class, locked_id=locked_id, coast_s=eff_coast,
                lock_s=lock_s)
            present = _present(sample)
            if present and not sample.coasted and sample.track_id >= 0:
                locked_id = sample.track_id   # follow this id when a gap coasts

            # Pass-through trigger: the target was seen and has now left the
            # frame -- keep driving straight for the commit window so the hull
            # clears the gate, then report success.
            if passthrough and seen_once and not present:
                reached_at = None
                if commit_until is None:
                    commit_until = now + commit_s
                    log.debug(f"[VIS  ] move: {target_class!r} cleared frame -- "
                              f"committing {commit_s:.1f}s to pass through")
                if now >= commit_until:
                    writers.neutral()
                    return Outcome(ALIGNED, "passed through", last_lat_err,
                                   last_fill, elapsed, end_x_px, end_y_px)
                _drive(gain, 0.0)   # no detection -> no lateral, just drive on
                _tick(vision_state, pixhawk)
                continue

            if not present:
                reached_at = None
                _drive(0.0, 0.0)
                if lost_since is None:
                    lost_since = now
                if (not hold_through_loss) and (now - lost_since) >= lost_grace_s:
                    reason = (f"target {target_class!r} lost" if seen_once else
                              f"target {target_class!r} NEVER detected -- check "
                              f"model/classes/camera view")
                    return Outcome(LOST, reason, last_lat_err, last_fill, elapsed,
                                   end_x_px, end_y_px)
                if (now - last_log) >= LOG_THROTTLE_S:
                    live = _live_classes(vision_state)
                    if live:
                        log.debug(
                            f"[VIS  ] move: {target_class!r} not among live "
                            f"detections {live} -- check classes filter / model")
                    else:
                        log.debug(f"[VIS  ] move LOST {now - lost_since:.1f}s "
                                  f"(grace {lost_grace_s:.1f}s)")
                    last_log = now
                _tick(vision_state, pixhawk)
                continue

            seen_once    = True
            lost_since   = None
            commit_until = None
            fill = _fill(sample, mode)
            last_fill = fill
            sampled_at = now - sample.age_s
            if sampled_at > last_seen_at + _FRAME_EPS_S or last_seen_at <= 0.0:
                # A NEW frame: its age is the pipeline latency. A re-read's is
                # not -- it has grown by however long this loop took.
                pipe_age = (sample.age_s if pipe_age <= 0.0
                            else pipe_age + 0.2 * (sample.age_s - pipe_age))
            if last_seen_at > 0.0 and sampled_at > last_seen_at + _FRAME_EPS_S:
                gap = sampled_at - last_seen_at
                # A dropout is not a slower camera. Letting one widen the
                # interval would loosen the freshness thresholds exactly when
                # the target has been lost.
                if gap <= 0.5:
                    det_interval = (gap if det_interval <= 0.0
                                    else det_interval + 0.2 * (gap - det_interval))
            if sampled_at > last_seen_at:
                last_seen_at = sampled_at
            fresh = _authority(sample, coast_s, det_interval, pipe_age)  # FPS staleness / coast decay
            _warn_low_fps(log, fresh, sample)     # F3: surface FPS-starvation, don't stall silently

            x_off = sample.ex * half_w         # signed horizontal offset (operator px)
            # End-position (returned) + live feedback sink.
            end_x_px = x_off
            end_y_px = sample.ey * half_h
            if report_fn is not None:
                report_fn(end_x_px, end_y_px)
            lat_pct = 0.0
            if maintain_on:
                ctrl = sample.ex - maintain_px / half_w
                last_lat_err = abs(ctrl) * half_w
                rgain = _range_gain(_fill(sample, mode), range_gain_floor)
                lat_pct = _clamp(ctrl * kp_lat * rgain, -g_lat, g_lat) * fresh
            lat_ema += _BRAKE_EMA_ALPHA * (lat_pct - lat_ema)

            if passthrough:
                # Drive forward at the speed cap until the target leaves frame.
                # Pass-through is defined to COAST through the gate -- never braked,
                # and forward is NOT freshness-decayed (it must clear the gate).
                _drive(gain, lat_pct)
                if (now - last_log) >= LOG_THROTTLE_S:
                    log.info(f"[ move PASS-THROUGH fill={fill * 100:.0f}% "
                             f"lat={x_off:+.0f}px ] ['{target_class}'] -> clear gate")
                    last_log = now
                _tick(vision_state, pixhawk)
                continue

            if fill >= fwd_fill:
                if reached_at is None:
                    reached_at = now
                # Station-keep: zero forward, keep correcting lateral if asked.
                _drive(0.0, lat_pct)
                fwd_ema += _BRAKE_EMA_ALPHA * (0.0 - fwd_ema)   # forward decays while holding
                if hold_s <= 0.0 or (now - reached_at) >= hold_s:
                    # Arrival: bleed the forward (and maintain-strafe) inertia so the
                    # hull halts in front of the target instead of creeping into it.
                    if brake:
                        _brake_axis(writers.forward, fwd_ema, brake_gain,
                                    abort_fn=abort_fn, log=log, label='VBRK')
                        if maintain_on:
                            _brake_axis(writers.lateral, lat_ema, brake_gain,
                                        abort_fn=abort_fn, log=log, label='VBRK')
                    writers.neutral()
                    return Outcome(ALIGNED,
                                   f"reached fill={fill * 100:.0f}%",
                                   last_lat_err, fill, elapsed, end_x_px, end_y_px)
            else:
                reached_at = None
                # Freshness-decay forward so a stale frame doesn't blind-drive the
                # approach past the fill target (yaw/depth untouched).
                fwd_pct = _clamp((fwd_fill - fill) * kp_forward, 0.0, gain) * fresh
                _drive(fwd_pct, lat_pct)
                fwd_ema += _BRAKE_EMA_ALPHA * (fwd_pct - fwd_ema)

            if (now - last_log) >= LOG_THROTTLE_S:
                hold_tag = ("" if reached_at is None
                            else f" hold={now - reached_at:.1f}/{hold_s:.0f}s")
                log.info(
                    f"[ move fill={fill * 100:.0f}% -> {fwd_fill * 100:.0f}% "
                    f"lat={x_off:+.0f}px ] ['{target_class}']{hold_tag}")
                last_log = now
            _tick(vision_state, pixhawk)
    finally:
        try:
            writers.neutral()
        except Exception as exc:   # noqa: BLE001
            log.warning(f"[VIS  ] move cleanup neutral raised: {exc!r}")

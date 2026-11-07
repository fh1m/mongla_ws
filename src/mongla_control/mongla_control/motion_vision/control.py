"""What both vision loops share: outcome codes, tuned constants, pure helpers,
and `_VisionLoop`, the base class that owns the clock, the loss rule and the
neutral-on-exit guarantee. No loop lives here; see `align.py` and `move.py`.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

from ..motion_rates import VISION_LOOP_HZ as LOOP_HZ
from ..motion_rates import LOG_THROTTLE_S
from ..motion_writers import REVERSE_KICK_SEC, _interruptible_sleep
from .. import actuation_model

# ---- Outcome codes (copied to Move.Result.final_value) --------------- #
# A separate code space from `fc.base` (SUCCEEDED, TIMEOUT = 4, ABORTED = 5,
# ...), which grades a board verb. Same names, different numbers: compare an
# `Outcome.code` only against the names below, and a `MoveResult.code` only
# against fc.base's. No module imports both sets today; keep it that way.
ALIGNED   = 0   # align: centred; move: bbox reached fwd_fill
LOST      = 1   # target gone past lost_grace_s (DSL runs fallback)
TIMEOUT   = 2   # duration elapsed without success
NO_CAMERA = 3   # camera_info never seen -- pipeline not up
ABORTED   = 4   # cooperative abort (goal cancelled)
DRIFTED   = 5   # align: centred while driving, OUT of band once stopped and re-measured

_CODE_NAME = {ALIGNED: 'OK', LOST: 'LOST', TIMEOUT: 'TIMEOUT',
              NO_CAMERA: 'NO_CAMERA', ABORTED: 'ABORTED', DRIFTED: 'DRIFTED'}


# ---- Tunable defaults (P-gains live as ROS params; these are the floor) #
KP_LAT_DEFAULT     = 60.0
KP_YAW_DEFAULT     = 60.0
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
# target (the torpedo 'hole'). Pure-proportional yaw falls below what the
# board actuates near centre, so a small residual error commands nothing and
# the hole-lock stalls just outside err_px. So: zero inside the err_px
# deadband, and close in, a floor at the smallest command that moves the hull.
#
# That smallest command is MEASURED, not guessed (the first version copied a
# 5.0 % spin-up assumption from an older autopilot). `attitude_control.cpp:87`
# gates the yaw stick at a hardcoded 0.02, AFTER PILOT_EXPO. Below it the board is not taking a rate command at all -- it is
# Holding heading. Above it the output jumps straight to 17.4 % of full scale,
# because `MOT_SPIN_MIN` lifts every non-zero demand into [0.15, 1].
#
# Measured on `tools/control_bench` -- the board's own attitude_control.cpp and
# mixer.cpp compiled natively -- under the LIVE board's parameters
# (`workbench/data/board_params_20260923.json`, read off the vehicle 2026-09-23):
#
#     stick 2.80 %  ->  yaw torque 0.000000  ->   0.00 % output
#     stick 2.86 %  ->  yaw torque 0.010369  ->  17.42 % output
#     stick 5.00 %  ->  yaw torque 0.018140  ->  19.12 % output   (the old guess)
#     stick 100  %  ->  yaw torque 0.506651  ->  70.67 % output
#
# Two consequences, and the second is the one that changes behaviour.
#  1. The old 5.0 was 1.75x higher than the smallest command that moves the hull.
#  2. A taper cannot taper. A floor ramped from 0 across an approach band (the
#     `_vision_yaw_floor` this file carried until 2026-11-06) produces EXACTLY
#     ZERO output for every value in (0, 2.856) and 17.4-19.1 % above it: a step
#     function wearing a ramp's clothes. Once the snap below existed the ramp
#     changed nothing -- 1 050 cases, identical output -- except at kp_yaw=0,
#     where it commanded yaw the caller had switched off. So it was deleted.
# So the close-in floor SNAPS UP to the smallest value that actuates:
# inside the fill gate a non-zero correction becomes at least 2.8561 % (17.4 % of
# full scale), capped by the caller's g_yaw. Rounding DOWN was tried and is
# wrong -- it stalls the hole-lock just outside err_px, which is the exact
# failure this floor was written for.
#
# And one claim we had to retract. It is NOT true that a sub-gate demand
# disturbs the board's heading hold while a true zero does not. Measured
# 2026-09-23 with a 5 deg heading error: sticks of 0.0, 0.5, 1.0, 2.0 and 2.8 %
# all produce the same hold torque, -0.097075. They land in the same branch of
# `attitude::stabilize`. The reason to act on a sub-gate demand is that it
# produces no thrust, full stop.
SROT_YAW_MIN_PCT = 100.0 * actuation_model.yaw_rate_command_floor()   # 2.8561 %

# The yaw floor above is a hard minimum command outside the deadband. On yaw
# (a yaw RATE) that is a relay feeding an integrator -- a limit-cycle oscillator
# that wobbles the hull left/right when aligning a small, distant target. Gate
# it on bbox fill so it only engages CLOSE (large bbox), where it does its real
# job: break T200 stiction for the tight hole-lock. Far away (small bbox) yaw is
# left pure-proportional and decays cleanly into the deadband with no relay.
# Pool-tunable: confirm the re-engage distance on pool day.
VISION_YAW_FLOOR_FILL = 0.25

# Inertial arrival brake (lateral + forward-on-fill-stop). The vision loops drive
# open-loop translation (surge/lateral); on arrival the hull coasts on water inertia in
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
# The brake's momentum proxy is an EMA with a time constant in SECONDS. It was
# tuned as a weight of 0.3 per tick on the 20 Hz loop, i.e. tau = -0.05/ln(0.7)
# = 0.140 s. Per tick it was rate-dependent: once the loop woke on data (~77 Hz)
# the same weight decayed ~4x faster, and through the arrival dwell (N in-band
# frames + _ALIGN_STABLE_MIN_S at ~0 command) a 30 % strafe fell under
# VISION_BRAKE_MIN_PCT before the brake read it, so the brake never fired.
_BRAKE_EMA_TAU_S     = 0.140


def _ema_step(ema: float, value: float, dt_s: float) -> float:
    """One step of the brake EMA over `dt_s` seconds -- same memory at any rate."""
    alpha = 1.0 - math.exp(-max(dt_s, 0.0) / _BRAKE_EMA_TAU_S)
    return ema + alpha * (value - ema)

# --- Range-adaptive gain schedule (Layer 1) -------------------------------- #
# The law is P on NORMALIZED pixel error, so the image shift per unit hull
# motion grows ~1/range (~ bbox fill): a kp critically damped far-field is
# OVER-gained close-in, oscillating the 20 kg hull off a small target. Scale the
# lat/surge kp DOWN as fill grows -- 1.0 below FILL_LO (far, full gain), ramping
# linearly to `floor` at/above FILL_HI (close, gentle). `floor` is the only deck
# knob (vision.range_gain_floor; 1.0 = off); the ramp endpoints stay constants.
# Yaw is EXCLUDED -- near-field yaw has its own stiction floor that ADDS
# authority (opposite intent).
VISION_RANGE_GAIN_FILL_LO = 0.25   # fill below which lat/surge gain is unscaled
VISION_RANGE_GAIN_FILL_HI = 0.60   # fill at/above which gain is fully floored

# --- Lateral anti-windup integral (Layer 2) -------------------------------- #
# Lateral is OPEN-LOOP thrust with no downstream position hold, so a steady
# current leaves a steady-state offset pure-P cannot null. A small bounded
# I-term (accumulated ONLY while holding, clamped to ±I_LAT_MAX, frozen on
# output saturation, reset on target loss) cancels it. ki rides
# vision.ki_lat (default 0 = OFF until the range damping is confirmed -- an
# integral on an under-damped loop makes it worse). Yaw gets NO I-term: the
# board's heading hold already integrates downstream.
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

    The clock is the detection interval, not the control tick, and getting that
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
# A 20 kg hull on open-loop lateral thrust against a bbox that itself jitters a few
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


# Verify against a re-measured pose, not the controller's own error (plan
# Block 1C; BumblebeeAS's `cluster_goto` does exactly this and gates at 2.5 cm /
# 1 deg). align declared ALIGNED on the last in-band tick BEFORE it braked, and
# reported that pre-brake frame's offset as where it ended. The brake, the
# neutral and the coast that follow were never looked at. So after the hull is
# stopped, align now waits for `align_stable_frames` NEW detections captured
# AFTER the brake, and asks the controller's own question of them again.
#
# The bar is the controller's own band (`eff_err`), on purpose: this checks that
# the controller's belief survives the stop, so it must not move the goalposts.
# Bumblebee's 2.5 cm / 1 deg is theirs; ours is not measured yet -- it needs
# water, and until then no new number is invented here.
#
# 1.5 s is how long to wait for those frames: three frames at a 5 Hz detector is
# 0.6 s, so this is 2.5x margin at the slowest rate the lock ladder is sized
# for. A target not re-seen in that time is LOST -- not ALIGNED.
VERIFY_TIMEOUT_S = 1.5


def _axis_ctrl(e_norm: float, offset_px: float, tool_px: float,
               half: float) -> float:
    """Signed normalised error of one centring axis from its goal.

    ONE copy, used by the control law and by the post-brake re-measurement, so
    the verification cannot quietly ask a different question.
    """
    return e_norm - (offset_px + tool_px) / half


def _centring_error_px(sample, *, axes, offsets, tool_du, tool_dv,
                       half_w, half_h, depth_axis: bool) -> float:
    """Worst centring error (px) over the active lat / yaw / depth axes."""
    worst = 0.0
    for ax in ('lat', 'yaw'):
        if ax in axes:
            worst = max(worst, abs(_axis_ctrl(sample.ex, offsets.get(ax, 0.0),
                                              tool_du, half_w)) * half_w)
    if depth_axis:
        worst = max(worst, abs(_axis_ctrl(sample.ey, offsets.get('depth', 0.0),
                                          tool_dv, half_h)) * half_h)
    return worst

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
# on the same tick the third frame lands, so nothing changes
# except when the detector jitters, where it can cost one extra frame (~10 ms)
# -- the right price for not firing a torpedo on 20 ms of evidence. At 3-4 Hz
# three frames already span 0.75-1.0 s and this never binds.
_ALIGN_STABLE_MIN_S = 0.10

# A detection older than this (seconds) counts as "no target this tick".
# bbox_error() returns None when the class is absent; this only catches a
# detector that has died while the last box is still cached.
_STALE_LIMIT_S = 1.0

# Pass-through commit: how long to keep driving forward AFTER the target
# leaves the frame, so the hull fully clears the gate. Overridable per
# call via move(..., hold=<s>).
_PASSTHROUGH_COMMIT_S = 2.0

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

    Leaves the axis at the kick; the caller goes neutral next (its
    normal exit path), so a separate settle is not needed here.
    """
    if abs(ema_pct) < VISION_BRAKE_MIN_PCT:
        return   # deadband exit -- no momentum worth braking
    mag = min(brake_gain * abs(ema_pct), VISION_BRAKE_CAP_PCT)
    kick = -math.copysign(mag, ema_pct)
    if log is not None:
        log.info(f'[{label:<5}] brake -- reverse {mag:.0f}% '
                 f'x {REVERSE_KICK_SEC:.2f}s (ema={ema_pct:+.0f}%)')
    axis_writer(kick)
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


def _tick(vision_state) -> None:
    """Wait for the next observation, or the tick timeout -- whichever first.

    The loop used to sleep a fixed period. Against an asynchronous producer
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
    period = 1.0 / LOOP_HZ
    waiter = getattr(vision_state, 'wait_for_sample', None)
    if waiter is None:
        time.sleep(period)
        return
    waiter(period)


def _mixer_saturated(lat_pct: float, yaw_pct: float, fwd_pct: float) -> bool:
    """True when the board will scale this demand down before it reaches water.

    Percent-thrust in, normalised axes out -- the mixer works in [-1, 1] and
    this loop in percent, and getting that conversion wrong would make the
    check fire either never or always.

    Best-effort: a failure here must never stop a vision verb, so it degrades
    to "not saturated", which is exactly the behaviour that existed before
    this function and therefore cannot be a regression.
    """
    try:
        from mongla_control.allocation import allocate
        return allocate(yaw=yaw_pct / 100.0, forward=fwd_pct / 100.0,
                        lateral=lat_pct / 100.0).saturated
    except Exception:                       # noqa: BLE001
        return False


def _srot_drive(fc, *, fwd_pct: float, lat_pct: float, yaw_pct: float,
                prioritise: bool = True) -> None:
    """Write one MANUAL_CONTROL frame for the srot board.

    Prioritised before it leaves. The board scales a saturated horizontal
    group UNIFORMLY, so a large lateral correction steals yaw in the same
    proportion -- measured on the mixer at fwd 0.9 / lat 0.9 / yaw 0.3, yaw
    arrives at 0.143. `prioritise` fits the demand host-side, yaw first, so the
    axis the tool points with keeps what it asked for. Below saturation it
    returns the demand unchanged, so an unsaturated frame is identical to
    before. A failure degrades to the raw demand, which is the old behaviour.
    """
    fwd, lat, yaw = fwd_pct / 100.0, lat_pct / 100.0, yaw_pct / 100.0
    if prioritise:                          # vision.mixer_aware; False = raw demand
        try:
            from mongla_control.allocation import prioritise as _fit
            fit = _fit({'forward': fwd, 'lateral': lat, 'yaw': yaw})
            fwd, lat, yaw = fit['forward'], fit['lateral'], fit['yaw']
        except Exception:                   # noqa: BLE001
            pass
    fc.manual(fwd=fwd, lat=lat, up=0.0, yaw=yaw)


def _srot_yaw_expressible(mag_pct: float, cap_pct: float) -> float:
    """Round a NON-ZERO yaw magnitude (%) UP to something the board can produce.

    Why up and not down. Between 0 and `SROT_YAW_MIN_PCT` the board produces
    no thrust at all, so a demand in that range is a correction that does not
    happen. The old code reached it through a taper and stalled just outside
    the deadband -- which is the exact failure the yaw floor was written for.
    Rounding up costs a relay; rounding down costs convergence, and a verb that
    times out having never moved is worse than one that wobbles in.

    The relay is unavoidable here, not a choice we are making. `MOT_SPIN_MIN`
    puts the smallest non-zero output at ~17.4 % of full scale, so there is no
    gentler value to pick. The real fix is a heading-hold SETPOINT we can move,
    which makes small yaw a continuous function of heading error instead of a
    stick demand -- upstream ask N §2. Until that lands this is the best the
    axis can do, and it is 17.4 % rather than the 19.1 % the guessed constant
    was producing.

    An exact zero stays zero. Only a correction the loop actually wants is
    lifted; `mag_pct == 0` means the error is inside the deadband.

    `cap_pct` is the caller's speed cap (`g_yaw`): we never exceed it, so a
    mission that asked for a gentle yaw does not get a lurch because of this.

    NOT a claim about heading hold. Measured 2026-09-23: with a 5 deg heading
    error, sticks of 0.0, 0.5, 1.0, 2.0 and 2.8 % ALL produce the same hold
    torque (-0.097075). A sub-gate demand and a true zero land in the same
    branch of `attitude::stabilize`, so neither disturbs the hold. The reason to
    act here is the stall, not the hold.

    Pure.
    """
    if mag_pct <= 0.0:
        return 0.0
    return min(max(mag_pct, SROT_YAW_MIN_PCT), max(cap_pct, 0.0))


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
def _tool_offset(tool_offset_fn, log) -> Tuple[float, float]:
    """(du, dv) pixels from camera centre to where the TOOL points, or (0, 0).

    Aim the tool, not the camera. The two axes are parallel, so an
    uncorrected offset misses by the offset itself AT EVERY RANGE -- it does
    not shrink as the hull closes in. The correction is `du = fx*x/Z`, LARGE up
    close, so it is resolved once per tick because Z moves.

    (0, 0) when it cannot be computed (no tool named, no calibration, no range):
    the previous behaviour, aiming the camera, which is what every mission has
    always done. The caller warns; refusing would break every align with no pose.
    """
    if tool_offset_fn is None:
        return 0.0, 0.0
    try:
        to = tool_offset_fn()
    except Exception as exc:                                  # noqa: BLE001
        log.error(f'[VIS  ] tool_offset_fn raised {exc!r} -- aiming the camera')
        return 0.0, 0.0
    if to is None:
        return 0.0, 0.0
    return float(to[0]), float(to[1])


def _yaw_command(ctrl: float, epx: float, eff_err: float, kp_yaw: float,
                 g_yaw: float, fill: float) -> float:
    """The yaw demand, %, for one tick.

    Polarity un-negated, same as lateral (ex > 0 -> target RIGHT -> yaw RIGHT).
    Inside the deadband: 0, so the hull settles and an in-band shot does not
    jitter. Outside: proportional, capped at `g_yaw`.

    The stiction snap belongs inside the fill gate, not outside. Close in
    (large box) the magnitude is lifted to the smallest yaw the board produces,
    so the hole-lock does not stall just outside err_px. Far away the rule is
    the opposite: a minimum on a rate channel is a relay that limit-cycles the
    hull, which is the far-field wobble VISION_YAW_FLOOR_FILL fixed.
    """
    if epx <= eff_err:
        return 0.0
    mag = min(abs(ctrl * kp_yaw), g_yaw)
    if fill >= VISION_YAW_FLOOR_FILL:
        mag = _srot_yaw_expressible(mag, g_yaw)
    return math.copysign(mag, ctrl) if mag > 0.0 else 0.0


def _forward_range_hold(sample, *, fwd_fill, fwd_mode, kp_forward, g_fwd,
                        standoff_max_tilt_deg, obliquity_fn, standoff_held, log):
    """The forward standoff axis for one tick.

    Returns (fwd_pct, in_band, fill, standoff_held). ONE-SIDED: drive forward
    while the box is smaller than `fwd_fill`, exactly 0 once at or past it --
    never reverse, so no reverse-kick and no ramming; drag bleeds a small
    overshoot. Same `_fill` law as move_loop.

    An oblique target reads as further away, so the hull closes in.
    `_fill('area')` is sqrt(w*h), Corke & Hutchinson's Z feature (IEEE T-RA
    17(4) 2001), valid while the target normal is within +/-35 deg of the
    optical axis; past that the area "will appear diminished, due to
    perspective, which causes the camera to initially approach the target".
    Approaching is the dangerous direction, so a PRESENT pose that says too
    oblique holds the approach.

    Fail direction is the opposite of the firing gate, deliberately.
    `square_within` treats "no pose" as not-square because for a shot both mean
    don't. Here "no pose" is the ordinary case (it needs lock_node, a
    calibration and a committed width), and treating it as oblique would
    disable forward drive on every mission without one -- a guard that breaks
    what it guards.
    """
    fill = _fill(sample, fwd_mode)
    fwd_err = float(fwd_fill) - fill
    if standoff_max_tilt_deg > 0.0 and fwd_err > FWD_BAND:
        try:
            obl = obliquity_fn() if obliquity_fn else None
        except Exception as exc:                              # noqa: BLE001
            log.error(f'[VIS  ] obliquity_fn raised {exc!r} -- '
                      f'treating as unknown; standoff unguarded')
            obl = None
        if obl is not None and obl > float(standoff_max_tilt_deg):
            if not standoff_held:
                standoff_held = True
                log.warning(
                    f'[VIS  ] standoff HELD: target face {obl:.0f} deg '
                    f'off-normal (> {standoff_max_tilt_deg:.0f}). '
                    f'sqrt(area) under-reads at this obliquity, so '
                    f'closing would drive PAST the standoff. Square '
                    f'up first.')
            fwd_err = 0.0
    if fwd_err <= FWD_BAND:
        return 0.0, True, fill, standoff_held
    return _clamp(fwd_err * kp_forward, 0.0, g_fwd), False, fill, standoff_held


def _verify_after_stop(vision_state, target_class, *, near, gate_norm,
                       ctrl_conf, locked_id, tool_offset_fn, axes, offsets,
                       half_w, half_h, depth_axis, frames_needed, eff_err,
                       worst, fill, started, end_px, reason, abort_fn,
                       log) -> Outcome:
    """Re-measure the target once the hull has stopped, and grade the align.

    The controller's own error says where it BELIEVED the target was while
    driving; this counts `frames_needed` fresh, non-coasted frames captured
    AFTER the stop (no coasting, no lock widening) and grades the worst of them
    against the same band. ALIGNED only if re-seen and still in band; LOST if
    not re-seen within VERIFY_TIMEOUT_S; DRIFTED if seen but out of band.
    """
    end_x_px, end_y_px = end_px
    t_stop = time.monotonic()
    seen, v_worst, last_at = 0, 0.0, t_stop
    while time.monotonic() - t_stop < VERIFY_TIMEOUT_S:
        if abort_fn and abort_fn():
            return Outcome(ABORTED, 'aborted while verifying', worst, fill,
                           time.monotonic() - started, end_x_px, end_y_px)
        _tick(vision_state)
        vs = vision_state.bbox_error(
            target_class, near=near, gate_norm=gate_norm,
            min_score=ctrl_conf, locked_id=locked_id,
            coast_s=0.0, lock_s=0.0)
        if not _present(vs) or vs.coasted:
            continue
        at = time.monotonic() - vs.age_s
        # Captured AFTER the stop, and a frame not already counted.
        if at <= last_at + _FRAME_EPS_S:
            continue
        last_at = at
        seen += 1
        tdu, tdv = _tool_offset(tool_offset_fn, log)
        v_worst = max(v_worst, _centring_error_px(
            vs, axes=axes, offsets=offsets, tool_du=tdu, tool_dv=tdv,
            half_w=half_w, half_h=half_h, depth_axis=depth_axis))
        end_x_px, end_y_px = vs.ex * half_w, vs.ey * half_h
        if seen >= frames_needed:
            break
    elapsed = time.monotonic() - started
    if seen < frames_needed:
        return Outcome(
            LOST, f'{reason}; then NOT re-seen once stopped '
                  f'({seen}/{frames_needed} frames in '
                  f'{VERIFY_TIMEOUT_S:.1f}s) -- unverified',
            worst, fill, elapsed, end_x_px, end_y_px)
    if v_worst > eff_err:
        return Outcome(
            DRIFTED, f'centred while driving ({worst:.0f}px), '
                     f'{v_worst:.0f}px off once stopped -- out of '
                     f'the {eff_err:.0f}px band on {seen} fresh frames',
            v_worst, fill, elapsed, end_x_px, end_y_px)
    return Outcome(ALIGNED, f'{reason}; verified {v_worst:.0f}px '
                            f'on {seen} frames after stopping',
                   v_worst, fill, elapsed, end_x_px, end_y_px)


class _VisionLoop:
    """What `align_loop` and `move_loop` share: the clock, the loss rule, the report.

    One verb call is one instance. ``c`` holds the verb's arguments exactly as
    they were passed, and the attributes are the state the loop carries from
    one tick to the next. Each subclass's ``run`` reads top to bottom as the
    stages of one tick.
    """

    verb = 'vision'

    def __init__(self, c) -> None:
        self.c = c
        self.log = c.log
        width, height = c.vision_state.image_size()
        self.half_w, self.half_h = width * 0.5, height * 0.5
        self.seen = False      # any frame yielded the target -> distinguishes
                               #   "never detected" (wrong model/classes/view)
                               #   from "seen but couldn't converge" at exit.
        self.lost_since: Optional[float] = None
        self.last_log = 0.0
        self.residual_px = 0.0  # the residual the Outcome reports
        self.fill = 0.0        # the bbox fill the Outcome reports
        self.end_x_px = math.nan   # signed from-centre px of target at last seen frame
        self.end_y_px = math.nan
        self.locked_id = -1    # tracker id of the target -> coast follows this id
        # Observed detection interval, EMA. The freshness thresholds scale with
        # it, because "~1 frame" is 13 ms on the forward camera and 32 ms on the
        # downward one and a fixed constant cannot be both. 0 until two distinct
        # frames have been seen, which yields the floors -- the unscaled
        # behaviour -- rather than a wild guess from one sample.
        self.det_interval = 0.0
        # Observed pipeline latency: the sample's age at the instant it FIRST
        # appears, i.e. capture -> the loop seeing it. EMA'd, and only sampled on
        # a genuinely new frame -- a re-read's age has grown by however long the
        # loop took and would inflate it without bound.
        self.pipe_age = 0.0
        self.last_frame_at = float('-inf')   # arrival time of the last COUNTED frame
        self.started = time.monotonic()
        self.deadline = self.started + max(c.duration, 0.0)

    def outcome(self, code: int, reason: str, now: float) -> Outcome:
        return Outcome(code, reason, self.residual_px, self.fill, now - self.started,
                       self.end_x_px, self.end_y_px)

    def never_seen(self) -> str:
        return (f"target {self.c.target_class!r} NEVER detected -- check "
                f"model/classes/camera view")

    def log_due(self, now: float) -> bool:
        """True at most once per LOG_THROTTLE_S."""
        if (now - self.last_log) < LOG_THROTTLE_S:
            return False
        self.last_log = now
        return True

    def lost(self, now: float) -> Optional[Outcome]:
        """No target this tick: start the grace clock, and give up after it.

        The caller has already commanded neutral and reset its own per-axis
        state; this owns only the rule both verbs share.
        """
        c = self.c
        if self.lost_since is None:
            self.lost_since = now
        if (not c.hold_through_loss) and (now - self.lost_since) >= c.lost_grace_s:
            reason = (f"target {c.target_class!r} lost" if self.seen
                      else self.never_seen())
            return self.outcome(LOST, reason, now)
        if self.log_due(now):
            live = _live_classes(c.vision_state)
            if live:
                self.log.debug(
                    f"[VIS  ] {self.verb}: {c.target_class!r} not among live "
                    f"detections {live} -- check classes filter / model")
            else:
                self.log.debug(f"[VIS  ] {self.verb} LOST "
                               f"{now - self.lost_since:.1f}s "
                               f"(grace {c.lost_grace_s:.1f}s)")
        return None

    def note_frame(self, sample, now: float) -> Tuple[float, bool]:
        """``(sampled_at, is_new)`` for this sample; a new frame updates the EMAs.

        ``sampled_at`` is the frame's arrival time -- constant across re-reads
        of one detection -- so only a genuinely new frame (see _FRAME_EPS_S)
        moves the latency, the interval, or anything counted in frames.
        """
        sampled_at = now - sample.age_s
        is_new = sampled_at > self.last_frame_at + _FRAME_EPS_S
        if is_new:
            self.pipe_age = (sample.age_s if self.pipe_age <= 0.0
                             else self.pipe_age + 0.2 * (sample.age_s - self.pipe_age))
            if self.last_frame_at > 0.0:
                gap = sampled_at - self.last_frame_at
                # Ignore absurd gaps: a dropout is not a slower camera, and
                # letting one widen the interval would loosen the freshness
                # thresholds exactly when the target is lost.
                if 0.0 < gap <= 0.5:
                    self.det_interval = (
                        gap if self.det_interval <= 0.0
                        else self.det_interval + 0.2 * (gap - self.det_interval))
            self.last_frame_at = sampled_at
        return sampled_at, is_new

    def report(self, sample) -> None:
        """Remember where the target is THIS tick, and stream it to feedback."""
        self.end_x_px = sample.ex * self.half_w
        self.end_y_px = sample.ey * self.half_h
        if self.c.report_fn is not None:
            self.c.report_fn(self.end_x_px, self.end_y_px)

    def neutral_on_exit(self) -> None:
        try:
            self.c.writers.neutral()
        except Exception as exc:   # noqa: BLE001
            self.log.warning(f"[VIS  ] {self.verb} cleanup neutral raised: {exc!r}")



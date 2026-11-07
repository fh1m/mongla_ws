#!/usr/bin/env python3
"""Vision-driven closed-loop motion -- two loops, pixel-native.

This module is the control engine behind the two mission verbs
``vision_align`` and ``vision_move``. Everything here is P-control on
**pixel error**, with ``gain`` acting as a hard max-speed cap. There are
no frame-age knobs, no on_lost policies, no lock modes -- the mission DSL
owns search/recovery via a Python ``fallback`` function.

Two loops
---------
``align_loop`` -- centre the target on the selected axes (lat / yaw, and
    on the downward camera image-Y as surge), each with an optional pixel
    offset. Exits when every active
    axis is within ``err_px`` for ``align_stable_frames`` ticks.

``move_loop`` -- drive forward until the target's bounding box fills the
    frame to ``fwd_fill`` (measured by ``mode`` = area/width/height),
    optionally holding a lateral pixel offset (``maintain``). Never
    re-centres yaw; the board holds depth.

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
  lat : positive = strafe RIGHT, so target-right needs a positive demand -> NO negate

Offsets (pixels, signed): a positive ``lat``/``yaw`` offset keeps the
target that many px to the RIGHT of centre; a positive ``depth`` offset
keeps it that many px BELOW centre. ``0`` = dead centre.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Dict, NamedTuple, Optional, Set, Tuple

from .motion_rates import VISION_LOOP_HZ as LOOP_HZ
from .motion_rates import LOG_THROTTLE_S
from .motion_writers import REVERSE_KICK_SEC, _interruptible_sleep
from . import actuation_model


# ---- Outcome codes (copied to Move.Result.final_value) --------------- #
# ⚠ A SEPARATE CODE SPACE from `fc.base` (SUCCEEDED, TIMEOUT = 4, ABORTED = 5,
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
# HOLDING HEADING. Above it the output jumps straight to 17.4 % of full scale,
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
# ⚠ Two consequences, and the second is the one that changes behaviour.
#  1. The old 5.0 was 1.75x higher than the smallest command that moves the hull.
#  2. A TAPER CANNOT TAPER. A floor ramped from 0 across an approach band (the
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
# ⚠ AND ONE CLAIM WE HAD TO RETRACT. It is NOT true that a sub-gate demand
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


# ⛔ VERIFY AGAINST A RE-MEASURED POSE, NOT THE CONTROLLER'S OWN ERROR (plan
# Block 1C; BumblebeeAS's `cluster_goto` does exactly this and gates at 2.5 cm /
# 1 deg). align declared ALIGNED on the last in-band tick BEFORE it braked, and
# reported that pre-brake frame's offset as where it ended. The brake, the
# neutral and the coast that follow were never looked at. So after the hull is
# stopped, align now waits for `align_stable_frames` NEW detections captured
# AFTER the brake, and asks the controller's own question of them again.
#
# THE BAR IS THE CONTROLLER'S OWN BAND (`eff_err`), on purpose: this checks that
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


# ⛔ A BOX IS WHERE THE TARGET WAS. `sample.age_s` runs from CAPTURE (median
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

    ⛔ PRIORITISED BEFORE IT LEAVES. The board scales a saturated horizontal
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

    ⛔ WHY UP AND NOT DOWN. Between 0 and `SROT_YAW_MIN_PCT` the board produces
    no thrust at all, so a demand in that range is a correction that does not
    happen. The old code reached it through a taper and stalled just outside
    the deadband -- which is the exact failure the yaw floor was written for.
    Rounding up costs a relay; rounding down costs convergence, and a verb that
    times out having never moved is worse than one that wobbles in.

    ⚠ THE RELAY IS UNAVOIDABLE HERE, not a choice we are making. `MOT_SPIN_MIN`
    puts the smallest non-zero output at ~17.4 % of full scale, so there is no
    gentler value to pick. The real fix is a heading-hold SETPOINT we can move,
    which makes small yaw a continuous function of heading error instead of a
    stick demand -- upstream ask N §2. Until that lands this is the best the
    axis can do, and it is 17.4 % rather than the 19.1 % the guessed constant
    was producing.

    ⛔ AN EXACT ZERO STAYS ZERO. Only a correction the loop actually wants is
    lifted; `mag_pct == 0` means the error is inside the deadband.

    `cap_pct` is the caller's speed cap (`g_yaw`): we never exceed it, so a
    mission that asked for a gentle yaw does not get a lurch because of this.

    ⚠ NOT a claim about heading hold. Measured 2026-09-23: with a 5 deg heading
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

    ⛔ AIM THE TOOL, NOT THE CAMERA. The two axes are parallel, so an
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

    ⛔ THE STICTION SNAP BELONGS INSIDE THE FILL GATE, NOT OUTSIDE. Close in
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

    ⛔ AN OBLIQUE TARGET READS AS FURTHER AWAY, SO THE HULL CLOSES IN.
    `_fill('area')` is sqrt(w*h), Corke & Hutchinson's Z feature (IEEE T-RA
    17(4) 2001), valid while the target normal is within +/-35 deg of the
    optical axis; past that the area "will appear diminished, due to
    perspective, which causes the camera to initially approach the target".
    Approaching is the dangerous direction, so a PRESENT pose that says too
    oblique holds the approach.

    FAIL DIRECTION IS THE OPPOSITE OF THE FIRING GATE, deliberately.
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


def align_loop(*,
               fc,
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
               lost_grace_s: float = 1.0,
               hold_through_loss: bool = False,
               align_stable_frames: int = 3,
               lock_on: bool = False,
               ctrl_conf: float = 0.0,
               range_gain_floor: float = 1.0,
               ki_lat: float = 0.0,
               i_lat_max: float = VISION_I_LAT_MAX,
               coast_s: float = 0.0,
               lock_s: float = 0.0,
               mixer_aware: bool = True,
               fwd_fill: float = 0.0,
               fwd_mode: str = 'area',
               kp_forward: float = KP_FORWARD_DEFAULT,
               settle_px: float = SETTLE_PX_DEFAULT,
               verify: bool = True,
               downward: bool = False,
               surge_sign: int = +1,
               on_locked=None,
               fire_t: float = 0.0,
               fire_pass: bool = False,
               fire_max_tilt_deg: float = 0.0,
               tilt_gate_fn=None,
               standoff_max_tilt_deg: float = 0.0,
               tool_offset_fn=None,
               derotate_fn=None,
               obliquity_fn=None,
               report_fn=None,
               writers=None,
               log=None,
               abort_fn=None) -> Outcome:
    """Hold ``target_class`` at the requested pixel offset on each active axis.

    ``axes`` is a subset of {'lat','yaw','depth'}; ``offsets`` carries the
    signed pixel offset for each active axis (0 = centre). ``depth`` exists
    only on the downward camera, where it is image-Y driven as surge.
    Returns an Outcome -- never raises on a miss.

    ``hold_s`` > 0 turns the verb into an ACTIVE station-keep: once centred,
    the loop keeps running its corrections for hold_s seconds
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

    ``downward`` -- DOWNWARD-CAMERA FRAME REMAP (bottom-mounted cam, looks at the
    pool floor). The bin task hovers ABOVE the target, so the body axes rotate:
      * ``lat`` (image-X) -> lateral strafe  -- SAME as forward.
      * ``depth`` (image-Y) -> SURGE fore/aft -- REMAPPED. Two-sided P-on-pixel
        modelled on the lat axis (rgain + freshness-decay + arrival reverse-kick
        brake), NOT the depth setpoint. Surge is open-loop timed thrust so it MUST be
        braked or the 20 kg hull coasts past the bin. ``surge_sign`` flips its
        polarity for the physical mount (verify DISARMED with vision_thrust_check --
        a wrong sign is positive feedback that drives the hull AWAY from the bin).
      * fire -> dropper via ``on_locked`` -- same mechanism as torpedo.
    The board holds depth (``set_depth``) and heading throughout; yaw is not
    driven on the downward camera.
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
      * ``range_gain_floor`` -- scales lat/surge kp DOWN as the bbox fills the
        frame (close) to stop the close-in overshoot (1.0 = off).
      * ``ki_lat`` -- lateral integral gain; cancels the steady-current offset of
        the open-loop lateral axis, accumulated only during the hold (0 = off).
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
      fire only leaves once lat/yaw AND the standoff range are all satisfied. With
      ``fwd_fill`` = 0 (default) there is no forward axis and behaviour is unchanged.

    How one tick runs: `_Align.run`, one method per stage.
    """
    c = SimpleNamespace(**locals())   # every argument, by name, for _Align
    bad = axes - VALID_AXES
    if bad:
        raise ValueError(f"align_loop: unknown axes {sorted(bad)}")
    if not axes:
        raise ValueError("align_loop: at least one axis required")
    # The board takes no depth setpoint over this wire, so nothing here may
    # pretend to move one. vision_verbs refuses both with the reason first.
    if 'depth' in axes and not downward:
        raise ValueError("align_loop: 'depth' is a downward-camera (surge) axis")
    if downward and float(fwd_fill) > 0.0:
        raise ValueError("align_loop: fwd_fill on the downward camera is a "
                         "depth descent, which the board does not take")

    if not _camera_ready(vision_state):
        return Outcome(NO_CAMERA, "camera pipeline not up (no camera_info)",
                       elapsed_s=0.0)
    return _Align(c).run()


class _Command(NamedTuple):
    """What one align tick asks of the thrusters, before freshness decay."""
    lat_pct: float
    yaw_pct: float
    fwd_pct: float
    p_lat: float          # the proportional part of lat_pct -> the brake proxy
    in_band: list         # one bool per active axis
    worst: float          # worst per-axis error, px


class _Align(_VisionLoop):
    """`align_loop`'s state, and one method per stage of a tick."""

    verb = 'align'

    def __init__(self, c) -> None:
        super().__init__(c)
        # Per-axis speed caps: each falls back to the global `gain` when unset, so
        # `align(gain=30)` caps every axis at 30% while `align(gain=40, yaw_gain=15)`
        # slows only yaw. (Mirrors the kp_* `or DEFAULT` idiom at the call site.)
        self.g_lat = c.gain if c.gain_lat is None else c.gain_lat
        self.g_yaw = c.gain if c.gain_yaw is None else c.gain_yaw
        self.g_fwd = c.gain                      # forward range-hold capped by the global gain
        self.use_fwd = float(c.fwd_fill) > 0.0   # optional forward standoff axis
        self.use_surge = 'depth' in c.axes       # downward image-Y -> surge fore/aft

        self.residual_px = float('inf')
        self.stable = 0
        self.aligned_at: Optional[float] = None   # monotonic of FIRST stable -> hold-window start
        self.fired = False          # on_locked fired once at fire_t into the hold (payload mid-hold)
        self.sq_warned = False      # one line per command, not one per tick
        self.standoff_held = False
        self.prev_worst: Optional[float] = None   # settle gate: worst error last NEW frame
        self.stable_since = float('inf')          # arrival of the FIRST frame of the current in-band run
        self.last_live_at = float('-inf')         # last LIVE (non-coasted) sighting -> fire_pass gate
        self.lat_ema = 0.0     # trailing EMA of the signed lateral command -> brake proxy
        self.surge_ema = 0.0   # downward: trailing EMA of the signed surge command -> brake proxy
        self.lat_i = 0.0       # lateral integral accumulator (Layer 2; 0 unless ki_lat>0)
        # ⛔ THE MIXER'S OWN LIMIT, WHICH THE EXISTING ANTI-WINDUP CANNOT SEE.
        # `abs(p_lat + lat_i) < g_lat` watches the SOFTWARE gain cap. The board
        # scales a whole thruster group down when any motor in it passes full
        # (`mixer.cpp`), and it never reports that -- so with yaw already spending
        # horizontal authority the hull can be clipping at an effective lateral
        # well below `g_lat`, while the integrator happily winds against a wall it
        # has no way to detect. Carried from the previous tick because yaw is
        # computed after lateral in this loop; one tick of lag on a limit that
        # persists for many is immaterial.
        self.was_saturated = False
        # Nominal tick. Used to SEED the measured dt below and as its ceiling;
        # the loop no longer sleeps a fixed period (see `_tick`), so treating this
        # as the real interval would misstate both the lock gate and the integral
        # term the moment the loop starts running at detection rate.
        self.dt_nominal = 1.0 / LOOP_HZ
        self.dt = self.dt_nominal
        self._last_pass = time.monotonic()
        # Seeded at the control period -- the shortest gap possible, so the gate
        # starts at its tightest and widens to the real detection interval once one
        # has been observed. Erring tight before acquisition is safe: there is no
        # lock to drop yet.
        self.gate_norm = _lock_gate(self.dt) if c.lock_on else 0.0
        self.last_accept_t: Optional[float] = None
        self.locked_ex: Optional[float] = None   # last-accepted centre -> continuity lock
        self.locked_ey: Optional[float] = None

        # Effective deadband: clamp to a physical floor so an over-tight err can't
        # perpetually TIMEOUT on bbox jitter. err_px here is already post-coercion
        # (commands.fields_for turned a rosidl-0 "unset" into the default/param), so
        # this only ever floors a genuinely tiny positive request -- logged, never
        # silent, so 'aligned (Npx)' is always read against the real deadband.
        self.eff_err = max(float(c.err_px), MIN_ALIGN_ERR_PX)
        floored = self.eff_err > float(c.err_px)
        fwd_note = (f" fwd>={c.fwd_fill * 100:.0f}%({c.fwd_mode})"
                    if self.use_fwd else "")
        self.log.info(
            f"[VIS  ] align class={c.target_class!r} axes={sorted(c.axes)}{fwd_note} "
            f"err={self.eff_err:.0f}px"
            f"{' (floored from %.0f)' % c.err_px if floored else ''} "
            f"gain={c.gain:.0f}% dur={c.duration:.0f}s hold={c.hold_s:.0f}s")
        if c.downward:
            # Loudly announce the rotated frame so the operator never mistakes a
            # downward align's axis meanings for the forward ones (see docstring).
            self.log.info(f"[VIS  ] downward frame: image-X->lateral, image-Y->surge "
                          f"(sign {c.surge_sign:+d}); depth held by the board")
        # The hold, the deadline and the log throttle count from here.
        self.started = time.monotonic()
        self.deadline = self.started + max(c.duration, 0.0)

    # -- the loop ------------------------------------------------------- #

    def run(self) -> Outcome:
        try:
            while True:
                out = self.step(time.monotonic())
                if out is not None:
                    return out
                _tick(self.c.vision_state)
        finally:
            self.neutral_on_exit()

    def step(self, now: float) -> Optional[Outcome]:
        """One tick: check the clock, sense, command, count, and maybe finish."""
        c = self.c
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
        self.dt = min(max(now - self._last_pass, 1e-3), self.dt_nominal * 4.0)
        self._last_pass = now
        if c.abort_fn and c.abort_fn():
            return self.outcome(ABORTED, "aborted", now)
        if now >= self.deadline:
            reason = ("not aligned (duration elapsed)" if self.seen
                      else self.never_seen())
            self.pass_fire()   # opt-in guaranteed shot if the target was seen live
            return self.outcome(TIMEOUT, reason, now)

        sample = self.sense()
        if not _present(sample):
            self.stable = 0
            self.lat_i = 0.0   # bleed integral windup while blind
            self.drive(0.0, 0.0)
            return self.lost(now)

        self.acquire(sample, now)
        cmd = self.command(sample)
        self.residual_px = cmd.worst
        # Freshness-decay: pace LATERAL authority to measurement freshness so
        # the loop doesn't blind-drive on a stale bbox between slow frames
        # (yaw excluded; the board holds depth). At healthy
        # FPS fresh==1.0 so this is a no-op. A COASTED sample decays on the
        # coast curve instead (gap decay), not freshness -- see _authority.
        fresh = _authority(sample, c.coast_s, self.det_interval, self.pipe_age)
        _warn_low_fps(self.log, fresh, sample)   # F3: surface FPS-starvation, don't stall silently
        lat_pct = cmd.lat_pct * fresh
        fwd_pct = cmd.fwd_pct * fresh   # forward shares the freshness/coast decay (never braked)
        # What the mixer will do to what we just asked for. Computed from
        # OUR demand and the frame's published matrix, so it needs nothing
        # from the board -- which is the only reason it is available at all.
        self.was_saturated = (_mixer_saturated(lat_pct, cmd.yaw_pct, fwd_pct)
                              if c.mixer_aware else False)
        self.drive(lat_pct, cmd.yaw_pct, fwd_pct)
        # Brake EMA tracks the PROPORTIONAL command only (a travel-momentum
        # proxy), NOT the full lat_pct: a hull holding STILL against a steady
        # current carries a nonzero integral (lat_i) but ~0 motion, so
        # including it would make the arrival brake reverse-kick a stationary
        # hull off the spot it was holding. Exclude lat_i here.
        self.lat_ema = _ema_step(self.lat_ema, cmd.p_lat * fresh, self.dt)
        if self.use_surge:
            # Downward surge is open-loop timed thrust (like lateral) -- track
            # its EMA so the arrival brake bleeds the fore/aft coast, or the hull
            # sails past the bin. fwd_pct already carries the signed, fresh-decayed
            # surge command, so the EMA is the true momentum proxy and the brake
            # reverse-kicks opposite it regardless of surge_sign.
            self.surge_ema = _ema_step(self.surge_ema, fwd_pct, self.dt)

        is_new_frame = self.count_frame(sample, now, cmd)
        if self.held_long_enough():
            out = self.hold(sample, now, is_new_frame, cmd.worst)
            if out is not None:
                return out

        if self.log_due(now):
            # The detector node owns the always-on operator bearing line, so
            # this per-verb copy is demoted to debug to avoid a duplicate in
            # the mission terminal. Worded as live offset, not a verdict.
            x_off = sample.ex * self.half_w
            y_off = sample.ey * self.half_h
            self.log.debug(
                f"[ offset lat={x_off:+.0f} depth={y_off:+.0f}px ] "
                f"'{c.target_class}' -> err {cmd.worst:.0f}/{self.eff_err:.0f}px")
        return None

    # -- the stages ----------------------------------------------------- #

    def drive(self, lat_pct: float, yaw_pct: float, fwd_pct: float = 0.0) -> None:
        """One MANUAL_CONTROL frame. ``fwd_pct`` is the forward range-hold
        (or the downward surge); 0 unless that axis is active."""
        _srot_drive(self.c.fc, fwd_pct=fwd_pct, lat_pct=lat_pct, yaw_pct=yaw_pct,
                    prioritise=self.c.mixer_aware)

    def sense(self):
        """The target box this tick.

        Continuity lock: once acquired, prefer the box NEAREST the last
        centre (within gate_norm) over the largest, so a 2nd hole / spurious
        box can't steal the aim. near=None (pre-acquire) or gate_norm=0
        (lock_on off) -> largest-area, unchanged. ctrl_conf gates low-score
        boxes out of the control target.
        """
        c = self.c
        near = (self.locked_ex, self.locked_ey) if self.locked_ex is not None else None
        return c.vision_state.bbox_error(
            c.target_class, near=near, gate_norm=self.gate_norm,
            min_score=c.ctrl_conf, locked_id=self.locked_id, coast_s=c.coast_s,
            lock_s=c.lock_s)

    def acquire(self, sample, now: float) -> None:
        """The target is present: remember which one, and where it is."""
        self.seen = True
        self.lost_since = None
        if not sample.coasted:
            self.last_live_at = now   # last LIVE sighting -> fire_pass recency gate
        # Re-arm the continuity lock on the accepted box (used as `near` next
        # tick) ONLY when lock_on -- otherwise leave near=None so selection
        # stays largest-area. After a real loss the loop exits LOST, so a
        # fresh verb call re-acquires largest -- no stale lock survives.
        if self.c.lock_on:
            self.locked_ex, self.locked_ey = sample.ex, sample.ey
        # Capture the tracker id of a LIVE box so a later gap coasts the right
        # target (VisionState only fills track_id when coast_s>0; a coasted
        # sample keeps the existing id). No-op when coasting is off.
        if not sample.coasted and sample.track_id >= 0:
            self.locked_id = sample.track_id
        # Signed from-centre offset of the target THIS tick: the end-position
        # the verb returns, and the live value report_fn streams to feedback.
        self.report(sample)

    def command(self, sample) -> _Command:
        """The per-axis P(+I) law: what each active axis asks for, and is it in band."""
        c = self.c
        half_w, half_h = self.half_w, self.half_h
        # Range-adaptive gain: soften lat/surge kp as the bbox fills the frame
        # (close) so the 20 kg hull doesn't overshoot a small target.
        rgain = _range_gain(_fill(sample, 'area'), c.range_gain_floor)
        yaw_pct = lat_pct = p_lat = fwd_pct = 0.0
        in_band = []
        worst = 0.0

        # Where the TOOL points, not the camera (see _tool_offset).
        tool_du, tool_dv = _tool_offset(c.tool_offset_fn, self.log)

        # Horizontal bearing NOW, not at capture. Forward camera only: a
        # yaw spins the downward image about its centre rather than
        # shifting it. Never on a coasted box -- the tracker already
        # extrapolated it. Absent rate or focal = the historical behaviour.
        ex_now = sample.ex
        if c.derotate_fn is not None and not c.downward and not sample.coasted:
            try:
                _dr = c.derotate_fn()
            except Exception:                             # noqa: BLE001
                _dr = None
            if _dr is not None:
                ex_now = _derotated_ex(sample.ex, sample.age_s,
                                       float(_dr[0]), float(_dr[1]), half_w)

        if 'lat' in c.axes:
            ctrl = _axis_ctrl(ex_now, c.offsets.get('lat', 0.0), tool_du, half_w)
            epx = abs(ctrl) * half_w
            worst = max(worst, epx)
            p_lat = ctrl * c.kp_lat * rgain
            # Lateral integral (Layer 2): Lateral is open-loop, so only the
            # integral nulls a steady-current offset. Accumulate ONLY during
            # the hold (aligned_at set), conditional on the output not being
            # saturated (anti-windup), clamped. ki_lat=0 -> exactly P.
            if c.ki_lat > 0.0 and self.aligned_at is not None \
                    and abs(p_lat + self.lat_i) < self.g_lat \
                    and not self.was_saturated:
                self.lat_i = _clamp(self.lat_i + c.ki_lat * ctrl * self.dt,
                                    -c.i_lat_max, c.i_lat_max)
            lat_pct = _clamp(p_lat + self.lat_i, -self.g_lat, self.g_lat)
            in_band.append(epx <= self.eff_err)

        if 'yaw' in c.axes:
            ctrl = _axis_ctrl(ex_now, c.offsets.get('yaw', 0.0), tool_du, half_w)
            epx = abs(ctrl) * half_w
            worst = max(worst, epx)
            yaw_pct = _yaw_command(ctrl, epx, self.eff_err, c.kp_yaw, self.g_yaw,
                                   _fill(sample, 'area'))
            in_band.append(epx <= self.eff_err)

        if self.use_surge:
            # DOWNWARD: the 'depth' axis (image-Y) drives SURGE fore/aft.
            # Two-sided P modelled on the lat axis (same kp/cap/rgain), so the
            # hull drives forward when the bin is ahead and BACK when behind --
            # the one-sided fwd/fill law can't back up and would never centre.
            # surge_sign flips polarity for the physical mount (verify disarmed).
            ctrl = _axis_ctrl(sample.ey, c.offsets.get('depth', 0.0), tool_dv, half_h)
            epx = abs(ctrl) * half_h
            worst = max(worst, epx)
            p_surge = ctrl * c.kp_lat * rgain
            fwd_pct = _clamp(p_surge, -self.g_fwd, self.g_fwd) * c.surge_sign
            in_band.append(epx <= self.eff_err)
        if self.use_fwd:
            fwd_pct, fwd_ok, self.fill, self.standoff_held = _forward_range_hold(
                sample, fwd_fill=c.fwd_fill, fwd_mode=c.fwd_mode,
                kp_forward=c.kp_forward, g_fwd=self.g_fwd,
                standoff_max_tilt_deg=c.standoff_max_tilt_deg,
                obliquity_fn=c.obliquity_fn, standoff_held=self.standoff_held,
                log=self.log)
            in_band.append(fwd_ok)
        return _Command(lat_pct, yaw_pct, fwd_pct, p_lat, in_band, worst)

    def count_frame(self, sample, now: float, cmd: _Command) -> bool:
        """Advance the stable-frame counter on a NEW frame; True if this was one.

        Distinct-detection gate: advance the stable-frame counter on new
        DETECTIONS, not loop ticks (see _FRAME_EPS_S). A re-read HOLDS the
        counter (neither advances nor resets) and only a genuinely new frame
        moves it. This makes align_stable_frames mean "N distinct in-band
        frames", FPS-independent, so a lucky single frame can't declare
        ALIGNED (or arm the fire) at low detector FPS.

        Settle gate (opt-in): require the hull to also be barely MOVING, not
        just in-band, before a frame counts toward the stable-frame exit -- so
        align ends settled on target (like move's held lateral) instead of
        mid-pass through the band. Keyed on |Δworst| (error velocity), so a
        steady current that holds a non-zero command at a perfect lock does
        NOT block it. settle_px=0 -> settled always True -> legacy behaviour.
        """
        c = self.c
        sampled_at, is_new_frame = self.note_frame(sample, now)
        if not is_new_frame:
            return False
        if c.lock_on:
            # Widen (or tighten) the gate to the interval actually
            # observed between accepted frames, floored at the control
            # period so a duplicate timestamp cannot collapse it to 0.
            if self.last_accept_t is not None:
                self.gate_norm = _lock_gate(
                    max(self.dt, sampled_at - self.last_accept_t))
            self.last_accept_t = sampled_at
        if self.stable == 0:
            # Start of a fresh in-band run. Stamped on the FRAME's own
            # arrival, not on `now`: `now` is a control tick and at
            # 50 Hz control over a 15 Hz camera the two differ by most
            # of a frame period, which would credit the dwell with time
            # the target was not actually in band.
            self.stable_since = sampled_at
        # Compared against the previous NEW frame's error (re-reads are
        # numerically identical and would trivially pass), so |Δworst| is a
        # real cross-frame error velocity.
        settled = (c.settle_px <= 0.0 or self.prev_worst is None
                   or abs(cmd.worst - self.prev_worst) <= c.settle_px)
        self.prev_worst = cmd.worst
        self.stable = self.stable + 1 if (all(cmd.in_band) and settled) else 0
        return True

    def held_long_enough(self) -> bool:
        """Both gates, for the reasons at _ALIGN_STABLE_MIN_S: enough distinct
        frames AND enough elapsed time across them."""
        return (self.stable >= self.c.align_stable_frames
                and self.last_frame_at - self.stable_since >= _ALIGN_STABLE_MIN_S)

    def hold(self, sample, now: float, is_new_frame: bool,
             worst: float) -> Optional[Outcome]:
        """Centred: open the hold window, fire if due, and finish when it closes.

        First confirmed-centred tick opens the hold window. With hold_s>0 we
        keep the loop ALIVE and correcting for hold_s -- the ACTIVE
        station-keep the operator needs to hold steady against water inertia
        while a payload fires (passive neutral would just drift). hold_s<=0
        keeps the original exit-on-stable behaviour bit-for-bit.

        ponytail: aligned_at is set ONCE and never reset (unlike move_loop's
        reached_at). Align drift is transient jitter the loop corrects;
        resetting on a drift-out tick would risk never completing the hold
        under a steady current. The in-band gate on this stage still
        guarantees we exit centred. Do NOT "fix" this to match move.
        """
        c = self.c
        if self.aligned_at is None:
            self.aligned_at = now
            if c.hold_s > 0.0:
                self.log.info(f"[VIS  ] align HELD -- station-keeping "
                              f"{c.hold_s:.1f}s ({worst:.0f}px)")
        self.fire_if_due(sample, now, is_new_frame)
        if c.hold_s <= 0.0 or (now - self.aligned_at) >= c.hold_s:
            return self.finish(now, worst)
        # Inside the hold window: keep correcting (this tick's drive already ran).
        return None

    def fire_if_due(self, sample, now: float, is_new_frame: bool) -> None:
        """The mid-hold payload shot, BEFORE the exit check.

        Fires while the loop is still correcting (not on the drifting exit
        tick). on_locked is non-blocking (spawns a thread) so a slow payload
        reconnect can't stall the 20 Hz station-keep. fire_t is clamped
        < hold_s upstream, so this trips while still holding.

        FRESHNESS GUARD: fire ONLY on the tick a genuinely NEW, non-coasted
        detection lands (is_new_frame -- the same distinct-frame signal that
        gates `stable`). This guarantees freshness at ANY detector FPS (a new
        frame is by definition current), fixing the low-FPS miss where the old
        `age_s <= 0.10s` window rarely coincided with the aligned tick at
        3-4 Hz. It is ALSO strictly safer: a FROZEN detector never produces a
        new frame, so a stale cached box (or a Kalman-coasted one) can never
        fire even while `stable` stands held at threshold through a mid-hold
        freeze. A torpedo leaves on a real live sighting.
        """
        c = self.c
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
        if c.fire_max_tilt_deg > 0.0 and fire_fresh and not self.fired:
            try:
                square_ok = bool(c.tilt_gate_fn and c.tilt_gate_fn(c.fire_max_tilt_deg))
            except Exception as exc:   # noqa: BLE001
                self.log.error(f"[VIS  ] tilt_gate_fn raised {exc!r} -- refusing")
                square_ok = False
            if not square_ok and not self.sq_warned:
                self.sq_warned = True
                self.log.info(f"[VIS  ] fire HELD: target not square within "
                              f"{c.fire_max_tilt_deg:.0f} deg (or no pose)")
        if c.on_locked is not None and not self.fired and fire_fresh and \
                square_ok and \
                (now - self.aligned_at) >= c.fire_t:
            self.fired = True
            try:
                c.on_locked()
            except Exception as exc:   # noqa: BLE001 -- fire must not kill the loop
                self.log.error(f"[VIS  ] on_locked (fire) raised {exc!r}")

    def pass_fire(self) -> None:
        """fire_pass fallback: on a NATURAL exit (TIMEOUT / hold-complete) without a
        strict mid-hold fire, actuate the payload anyway so a detected-but-not-fully-
        aligned run still scores. Gated on a LIVE (non-coasted) sighting within
        lost_grace_s -- never fires into empty water or on a Kalman ghost. One-shot
        (guarded by ``fired``). Opt-in via fire_pass; no-op when off."""
        c = self.c
        if (not c.fire_pass or c.on_locked is None or self.fired
                or (time.monotonic() - self.last_live_at) > c.lost_grace_s):
            return
        self.fired = True
        self.log.info("[VIS  ] fire_pass -- target seen; firing at command end "
                      "(alignment not required)")
        try:
            c.on_locked()
        except Exception as exc:   # noqa: BLE001 -- fire must not kill the loop
            self.log.error(f"[VIS  ] on_locked (fire_pass) raised {exc!r}")

    def finish(self, now: float, worst: float) -> Outcome:
        """Arrival / hold complete: brake, stop, and grade the result.

        Bleed lateral inertia so the hull stops square and the next mission
        step starts from the planned position. Gated on the EMA, so a
        gently-converged lock (hole-lock) exits with ~0 EMA and is NOT kicked.
        Yaw never brakes.
        """
        c = self.c
        if c.brake and 'lat' in c.axes:
            _brake_axis(c.writers.lateral, self.lat_ema, c.brake_gain,
                        abort_fn=c.abort_fn, log=self.log, label='VBRK')
        # Downward: also bleed the SURGE coast (fore/aft) so the hull
        # stops square over the bin instead of sailing past. Surge is open-loop
        # timed thrust coasts exactly like lateral -- same self-gating brake.
        if c.brake and self.use_surge:
            _brake_axis(c.writers.forward, self.surge_ema, c.brake_gain,
                        abort_fn=c.abort_fn, log=self.log, label='VBRK')
        c.writers.neutral()
        # If a mid-hold strict fire never landed (e.g. hold completed but
        # freshness never coincided), fire_pass still actuates here on a
        # live+recent sighting. No-op when off or already fired.
        self.pass_fire()
        # State the deadband next to the residual so 'aligned (Npx)' is never
        # misread as "should have been 0" -- N is within the eff_err deadband
        # by construction.
        reason = (f"held {c.hold_s:.1f}s ({worst:.0f}/{self.eff_err:.0f}px)"
                  if c.hold_s > 0.0
                  else f"aligned ({worst:.0f}/{self.eff_err:.0f}px)")
        if not c.verify:
            return self.outcome(ALIGNED, reason, now)
        return _verify_after_stop(
            c.vision_state, c.target_class,
            near=((self.locked_ex, self.locked_ey)
                  if self.locked_ex is not None else None),
            gate_norm=self.gate_norm, ctrl_conf=c.ctrl_conf,
            locked_id=self.locked_id, tool_offset_fn=c.tool_offset_fn,
            axes=c.axes, offsets=c.offsets, half_w=self.half_w,
            half_h=self.half_h, depth_axis=self.use_surge,
            frames_needed=c.align_stable_frames, eff_err=self.eff_err,
            worst=worst, fill=self.fill, started=self.started,
            end_px=(self.end_x_px, self.end_y_px), reason=reason,
            abort_fn=c.abort_fn, log=self.log)


# ---------------------------------------------------------------------- #
#  move_loop -- drive forward to a bbox fill ratio                       #
# ---------------------------------------------------------------------- #
def move_loop(*,
              fc,
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
              range_gain_floor: float = 1.0,
              coast_s: float = 0.0,
              lock_s: float = 0.0,
              mixer_aware: bool = True,
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
    driving; depth and yaw are never commanded (the board holds both). Returns an Outcome -- never
    raises on a miss.

    ``report_fn`` (if given) is called every PRESENT tick with the signed
    from-centre pixel offset ``(x_off, y_off)`` -- the live-telemetry sink for
    action feedback (same contract as align_loop).

    How one tick runs: `_Move.run`, one method per stage.
    """
    c = SimpleNamespace(**locals())   # every argument, by name, for _Move
    if mode not in VALID_MODES:
        raise ValueError(f"move_loop: mode must be one of {sorted(VALID_MODES)}")

    if not _camera_ready(vision_state):
        return Outcome(NO_CAMERA, "camera pipeline not up (no camera_info)",
                       elapsed_s=0.0)
    return _Move(c).run()


class _Move(_VisionLoop):
    """`move_loop`'s state, and one method per stage of a tick."""

    verb = 'move'

    def __init__(self, c) -> None:
        super().__init__(c)
        # Lateral 'maintain' strafe gets its own cap; forward stays capped by `gain`.
        self.g_lat = c.gain if c.gain_lat is None else c.gain_lat
        self.reached_at: Optional[float] = None
        self.commit_until: Optional[float] = None
        self.fwd_ema = 0.0   # trailing EMA of forward command -> brake proxy (fill-stop)
        self.lat_ema = 0.0   # trailing EMA of the maintain strafe command -> brake proxy
        # Pass-through commit window: hold_s overrides the module default.
        self.commit_s = c.hold_s if c.hold_s > 0.0 else _PASSTHROUGH_COMMIT_S
        self.log.debug(
            f"[VIS  ] move class={c.target_class!r} "
            f"{'PASS-THROUGH commit=%.1fs' % self.commit_s if c.passthrough else 'fwd_fill=%.0f%%' % (c.fwd_fill * 100)} "
            f"mode={c.mode} maintain={'%+.0fpx' % c.maintain_px if c.maintain_on else 'off'} "
            f"gain={c.gain:.0f}% dur={c.duration:.0f}s")
        self.started = time.monotonic()
        self.deadline = self.started + max(c.duration, 0.0)
        self._last_pass = self.started
        self.dt = 1.0 / LOOP_HZ

    # -- the loop ------------------------------------------------------- #

    def run(self) -> Outcome:
        try:
            while True:
                out = self.step(time.monotonic())
                if out is not None:
                    return out
                _tick(self.c.vision_state)
        finally:
            self.neutral_on_exit()

    def step(self, now: float) -> Optional[Outcome]:
        """One tick: check the clock, sense, then commit, give up, or drive."""
        c = self.c
        # Seconds since the last pass, for the brake EMA. Clamped like
        # align_loop's: a stall must not dump a huge step into it.
        self.dt = min(max(now - self._last_pass, 1e-3), 4.0 / LOOP_HZ)
        self._last_pass = now
        if c.abort_fn and c.abort_fn():
            return self.outcome(ABORTED, "aborted", now)
        if now >= self.deadline:
            if not self.seen:
                reason = self.never_seen()
            else:
                reason = ("passed-through window not closed (duration elapsed)"
                          if c.passthrough else "fill not reached (duration elapsed)")
            return self.outcome(TIMEOUT, reason, now)

        sample = self.sense()
        present = _present(sample)
        if present and not sample.coasted and sample.track_id >= 0:
            self.locked_id = sample.track_id   # follow this id when a gap coasts

        if c.passthrough and self.seen and not present:
            return self.commit(now)
        if not present:
            self.reached_at = None
            self.drive(0.0, 0.0)
            return self.lost(now)

        fill, fresh = self.acquire(sample, now)
        lat_pct = self.maintain(sample, fresh)
        x_off = sample.ex * self.half_w   # signed horizontal offset (operator px)
        if c.passthrough:
            # Drive forward at the speed cap until the target leaves frame.
            # Pass-through is defined to COAST through the gate -- never braked,
            # and forward is NOT freshness-decayed (it must clear the gate).
            self.drive(c.gain, lat_pct)
            if self.log_due(now):
                self.log.info(f"[ move PASS-THROUGH fill={fill * 100:.0f}% "
                              f"lat={x_off:+.0f}px ] ['{c.target_class}'] -> clear gate")
            return None

        out = self.approach(now, fill, fresh, lat_pct)
        if out is not None:
            return out
        if self.log_due(now):
            hold_tag = ("" if self.reached_at is None
                        else f" hold={now - self.reached_at:.1f}/{c.hold_s:.0f}s")
            self.log.info(
                f"[ move fill={fill * 100:.0f}% -> {c.fwd_fill * 100:.0f}% "
                f"lat={x_off:+.0f}px ] ['{c.target_class}']{hold_tag}")
        return None

    # -- the stages ----------------------------------------------------- #

    def drive(self, fwd_pct: float, lat_pct: float) -> None:
        # move never commands yaw: the board holds heading.
        _srot_drive(self.c.fc, fwd_pct=fwd_pct, lat_pct=lat_pct, yaw_pct=0.0,
                    prioritise=self.c.mixer_aware)

    def sense(self):
        """The target box this tick.

        Pass-through (fwd=None) must DETECT "target gone" to fire its commit
        window, so it deliberately does NOT coast -- a coasted box would
        keep present=True and delay the commit by up to coast_s. Fill-stop
        moves coast normally.
        """
        c = self.c
        eff_coast = 0.0 if c.passthrough else c.coast_s
        return c.vision_state.bbox_error(
            c.target_class, locked_id=self.locked_id, coast_s=eff_coast,
            lock_s=c.lock_s)

    def commit(self, now: float) -> Optional[Outcome]:
        """Pass-through trigger: the target was seen and has now left the frame.

        Keep driving straight for the commit window so the hull clears the
        gate, then report success.
        """
        c = self.c
        self.reached_at = None
        if self.commit_until is None:
            self.commit_until = now + self.commit_s
            self.log.debug(f"[VIS  ] move: {c.target_class!r} cleared frame -- "
                           f"committing {self.commit_s:.1f}s to pass through")
        if now >= self.commit_until:
            c.writers.neutral()
            return self.outcome(ALIGNED, "passed through", now)
        self.drive(c.gain, 0.0)   # no detection -> no lateral, just drive on
        return None

    def acquire(self, sample, now: float) -> Tuple[float, float]:
        """The target is present: ``(fill, freshness)`` for this tick."""
        c = self.c
        self.seen = True
        self.lost_since = None
        self.commit_until = None
        fill = _fill(sample, c.mode)
        self.fill = fill
        self.note_frame(sample, now)
        fresh = _authority(sample, c.coast_s, self.det_interval, self.pipe_age)  # FPS staleness / coast decay
        _warn_low_fps(self.log, fresh, sample)     # F3: surface FPS-starvation, don't stall silently
        # End-position (returned) + live feedback sink.
        self.report(sample)
        return fill, fresh

    def maintain(self, sample, fresh: float) -> float:
        """The lateral strafe that holds ``maintain_px``; 0 when maintain is off."""
        c = self.c
        lat_pct = 0.0
        if c.maintain_on:
            ctrl = sample.ex - c.maintain_px / self.half_w
            self.residual_px = abs(ctrl) * self.half_w
            rgain = _range_gain(_fill(sample, c.mode), c.range_gain_floor)
            lat_pct = _clamp(ctrl * c.kp_lat * rgain, -self.g_lat, self.g_lat) * fresh
        self.lat_ema = _ema_step(self.lat_ema, lat_pct, self.dt)
        return lat_pct

    def approach(self, now: float, fill: float, fresh: float,
                 lat_pct: float) -> Optional[Outcome]:
        """Fill-stop: drive toward ``fwd_fill``, then hold and brake on arrival."""
        c = self.c
        if fill < c.fwd_fill:
            self.reached_at = None
            # Freshness-decay forward so a stale frame doesn't blind-drive the
            # approach past the fill target (yaw/depth untouched).
            fwd_pct = _clamp((c.fwd_fill - fill) * c.kp_forward, 0.0, c.gain) * fresh
            self.drive(fwd_pct, lat_pct)
            self.fwd_ema = _ema_step(self.fwd_ema, fwd_pct, self.dt)
            return None
        if self.reached_at is None:
            self.reached_at = now
        # Station-keep: zero forward, keep correcting lateral if asked.
        self.drive(0.0, lat_pct)
        self.fwd_ema = _ema_step(self.fwd_ema, 0.0, self.dt)   # forward decays while holding
        if c.hold_s > 0.0 and (now - self.reached_at) < c.hold_s:
            return None
        # Arrival: bleed the forward (and maintain-strafe) inertia so the
        # hull halts in front of the target instead of creeping into it.
        if c.brake:
            _brake_axis(c.writers.forward, self.fwd_ema, c.brake_gain,
                        abort_fn=c.abort_fn, log=self.log, label='VBRK')
            if c.maintain_on:
                _brake_axis(c.writers.lateral, self.lat_ema, c.brake_gain,
                            abort_fn=c.abort_fn, log=self.log, label='VBRK')
        c.writers.neutral()
        return self.outcome(ALIGNED, f"reached fill={fill * 100:.0f}%", now)

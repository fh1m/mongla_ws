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
from .motion_rates import DEPTH_SETPOINT_HZ as DEPTH_HZ
from .motion_rates import LOG_THROTTLE_S
from .motion_writers import REVERSE_KICK_SEC, _interruptible_sleep


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
VISION_LOCK_GATE_NORM = 0.30   # max normalized centre jump to stay locked

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
VISION_FRESH_FULL_S = 0.10   # full authority while the sample is this fresh (~1 frame)
VISION_FRESH_ZERO_S = 0.40   # linearly decayed to zero by this age (driving blind)

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


def _freshness(age_s: float) -> float:
    """Translational-command authority [0,1] as a function of sample age.

    1.0 while the sample is fresher than VISION_FRESH_FULL_S, then linearly to
    0.0 by VISION_FRESH_ZERO_S (driving blind -> stop). Pure + side-effect-free
    so it unit-tests without ROS. Caps per-frame over-drive at low FPS while
    leaving healthy FPS untouched (frames refresh before decay engages).
    """
    if age_s <= VISION_FRESH_FULL_S:
        return 1.0
    if age_s >= VISION_FRESH_ZERO_S:
        return 0.0
    span = VISION_FRESH_ZERO_S - VISION_FRESH_FULL_S
    return (VISION_FRESH_ZERO_S - age_s) / span


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


def _authority(sample, coast_s: float) -> float:
    """Per-tick translational authority for `sample`.

    Live box  -> ``_freshness(age)``       (per-frame staleness at low FPS).
    Coasted   -> ``_coast_authority(age)``  (gap decay; the box is fresh each
                 tick so freshness must NOT also be applied -- that would
                 double-decay and kill the coast inside ~0.4 s).
    """
    if getattr(sample, 'coasted', False):
        return _coast_authority(sample.age_s, coast_s)
    return _freshness(sample.age_s)


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


def _read_depth(pixhawk) -> float:
    att = pixhawk.get_attitude()
    return float(att['depth']) if att else 0.0


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
               gain_depth: Optional[float] = None,
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
               on_locked=None,
               fire_t: float = 0.0,
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
    < hold_s (the verb clamps it upstream).

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
    g_depth = gain if gain_depth is None else gain_depth

    use_depth   = 'depth' in axes
    throttle_ch = 65535 if use_depth else 1500   # release Ch3 for ALT_HOLD depth PID
    max_nudge   = _MAX_DEPTH_NUDGE * max(g_depth, 0.0) / 100.0

    depth_setpoint = _read_depth(pixhawk)

    def _drive(lat_pct: float, yaw_pct: float) -> None:
        """Write the translation/yaw RC frame, honouring an active lock.

        When ``release_yaw`` is set the background heading lock owns Ch4,
        so touch only throttle/forward/lateral and leave yaw released --
        writing yaw=1500 here would race the lock's 20 Hz Ch4 stream
        (the same fight ``move_loop`` avoids via ``send_rc_translation``).
        """
        if release_yaw:
            pixhawk.send_rc_translation(
                throttle=throttle_ch, forward=1500,
                lateral=Pixhawk.percent_to_pwm(lat_pct))
        else:
            pixhawk.send_rc_override(
                forward=1500,
                lateral=Pixhawk.percent_to_pwm(lat_pct),
                yaw=Pixhawk.percent_to_pwm(yaw_pct),
                throttle=throttle_ch)

    stable      = 0
    lost_since: Optional[float] = None
    aligned_at: Optional[float] = None   # monotonic of FIRST stable -> hold-window start
    fired       = False  # on_locked fired once at fire_t into the hold (payload mid-hold)
    last_log    = 0.0
    last_depth  = 0.0
    last_err_px = float('inf')
    end_x_px    = math.nan  # signed from-centre px of target at last seen frame
    end_y_px    = math.nan
    lat_ema     = 0.0   # trailing EMA of the signed lateral command -> brake proxy
    lat_i       = 0.0   # lateral integral accumulator (Layer 2; 0 unless ki_lat>0)
    dt          = 1.0 / LOOP_HZ              # fixed tick (loop sleeps this each pass)
    gate_norm   = VISION_LOCK_GATE_NORM if lock_on else 0.0
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
    log.info(
        f"[VIS  ] align class={target_class!r} axes={sorted(axes)} "
        f"err={eff_err:.0f}px"
        f"{' (floored from %.0f)' % err_px if floored else ''} "
        f"gain={gain:.0f}% dur={duration:.0f}s hold={hold_s:.0f}s")

    started  = time.monotonic()
    deadline = started + max(duration, 0.0)
    try:
        while True:
            now     = time.monotonic()
            elapsed = now - started
            if abort_fn and abort_fn():
                return Outcome(ABORTED, "aborted", last_err_px, 0.0, elapsed,
                               end_x_px, end_y_px)
            if now >= deadline:
                reason = ("not aligned (duration elapsed)" if saw_target else
                          f"target {target_class!r} NEVER detected -- check "
                          f"model/classes/camera view")
                return Outcome(TIMEOUT, reason, last_err_px, 0.0, elapsed,
                               end_x_px, end_y_px)

            # Continuity lock: once acquired, prefer the box NEAREST the last
            # centre (within gate_norm) over the largest, so a 2nd hole / spurious
            # box can't steal the aim. near=None (pre-acquire) or gate_norm=0
            # (lock_on off) -> largest-area, unchanged. ctrl_conf gates low-score
            # boxes out of the control target.
            near = (locked_ex, locked_ey) if locked_ex is not None else None
            sample = vision_state.bbox_error(
                target_class, near=near, gate_norm=gate_norm, min_score=ctrl_conf,
                locked_id=locked_id, coast_s=coast_s)
            if not _present(sample):
                stable = 0
                lat_i = 0.0   # bleed integral windup while blind
                _drive(0.0, 0.0)
                if use_depth:
                    pixhawk.set_target_depth(depth_setpoint)
                if lost_since is None:
                    lost_since = now
                if (not hold_through_loss) and (now - lost_since) >= lost_grace_s:
                    reason = (f"target {target_class!r} lost" if saw_target else
                              f"target {target_class!r} NEVER detected -- check "
                              f"model/classes/camera view")
                    return Outcome(LOST, reason, last_err_px, 0.0, elapsed,
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
                time.sleep(1.0 / LOOP_HZ)
                continue

            saw_target = True
            lost_since = None
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
            yaw_pct = lat_pct = p_lat = 0.0
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
                    if _fill(sample, 'area') >= VISION_YAW_FLOOR_FILL:
                        mag = max(mag, min(VISION_YAW_MIN_PCT, g_yaw))
                    yaw_pct = math.copysign(mag, ctrl)
                in_band.append(epx <= eff_err)

            if use_depth:
                ctrl = sample.ey - offsets.get('depth', 0.0) / half_h
                epx  = abs(ctrl) * half_h
                worst = max(worst, epx)
                step = _clamp(ctrl * kp_depth * rgain,
                              -max_nudge, max_nudge) * depth_sign
                depth_setpoint = min(depth_setpoint - step, _MIN_DEPTH_M)
                in_band.append(epx <= eff_err)

            last_err_px = worst
            # Freshness-decay: pace LATERAL authority to measurement freshness so
            # the loop doesn't blind-drive on a stale bbox between slow frames
            # (yaw/depth excluded -- ArduSub bleeds Ch4, holds depth). At healthy
            # FPS fresh==1.0 so this is a no-op. A COASTED sample decays on the
            # coast curve instead (gap decay), not freshness -- see _authority.
            fresh = _authority(sample, coast_s)
            lat_pct *= fresh
            _drive(lat_pct, yaw_pct)
            # Brake EMA tracks the PROPORTIONAL command only (a travel-momentum
            # proxy), NOT the full lat_pct: a hull holding STILL against a steady
            # current carries a nonzero integral (lat_i) but ~0 motion, so
            # including it would make the arrival brake reverse-kick a stationary
            # hull off the spot it was holding. Exclude lat_i here.
            lat_ema += _BRAKE_EMA_ALPHA * (p_lat * fresh - lat_ema)
            if use_depth and (now - last_depth) >= 1.0 / DEPTH_HZ:
                pixhawk.set_target_depth(depth_setpoint)
                last_depth = now

            stable = stable + 1 if all(in_band) else 0
            if stable >= align_stable_frames:
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
                if on_locked is not None and not fired and \
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
                    writers.neutral()
                    # State the deadband next to the residual so 'aligned
                    # (Npx)' is never misread as "should have been 0" -- N is
                    # within the eff_err deadband by construction.
                    reason = (f"held {hold_s:.1f}s ({worst:.0f}/{eff_err:.0f}px)"
                              if hold_s > 0.0
                              else f"aligned ({worst:.0f}/{eff_err:.0f}px)")
                    return Outcome(ALIGNED, reason, worst, 0.0, elapsed,
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
            time.sleep(1.0 / LOOP_HZ)
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
                target_class, locked_id=locked_id, coast_s=eff_coast)
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
                time.sleep(1.0 / LOOP_HZ)
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
                time.sleep(1.0 / LOOP_HZ)
                continue

            seen_once    = True
            lost_since   = None
            commit_until = None
            fill = _fill(sample, mode)
            last_fill = fill
            fresh = _authority(sample, coast_s)   # FPS staleness (live) or coast decay

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
                time.sleep(1.0 / LOOP_HZ)
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
            time.sleep(1.0 / LOOP_HZ)
    finally:
        try:
            writers.neutral()
        except Exception as exc:   # noqa: BLE001
            log.warning(f"[VIS  ] move cleanup neutral raised: {exc!r}")

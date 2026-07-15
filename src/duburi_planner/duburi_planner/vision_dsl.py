"""vision_dsl -- duburi.vision.* closed-loop sub-namespace (two verbs).

Exactly two mission-facing verbs, both pixel-native and recover-don't-fail:

    duburi.vision.align(target, *, lat=None, yaw=None, depth=None,
                        err=40, duration=20, gain=30,
                        lat_gain=None, yaw_gain=None, depth_step=None,
                        brake=True, brake_gain=None, hold=None,
                        fire_pass=False, hold_heading=False,
                        fallback=None, camera=None)

    duburi.vision.move(target, *, fwd=95, mode='area', maintain=None,
                       hold=None, err=40, duration=20, gain=30,
                       lat_gain=None, brake=True, brake_gain=None,
                       fallback=None, camera=None)

Axis rule (align): each of ``lat`` / ``yaw`` / ``depth`` is ``None`` =
axis OFF; a number = axis ON, where the number is the **signed pixel
offset from centre** (``0`` = centre, ``+`` = right/below, ``-`` =
left/above). ``lat`` + ``yaw`` are horizontal (strafe / rotate),
``depth`` is vertical. At least one axis is required.

Neither verb ever raises on a miss. The server always returns
``success=True``; the align/move outcome rides in ``Move.Result.final_value``
as an integer code. The DSL turns that into a :class:`VisionResult` and,
on a real target loss, runs the mission-authored ``fallback`` search
function for one cycle before re-entering the vision loop -- all inside
the original ``duration`` budget.

The ``fallback`` is INTERRUPTED the instant the target reappears: the
in-flight control verb is cancelled and every remaining verb in the search
short-circuits, so the loop re-enters the vision verb while the target is
still in frame (a search that ran to completion would carry the just-seen
target back out of view -- the miss this prevents). Keep ``fallback`` bodies
to motion verbs only (no ``fire``/``disarm`` -- a post-trip safety verb would
be short-circuited; the mission's own emergency-disarm path runs OUTSIDE the
fallback and is unaffected).

Model / class switching (``ClassRef`` -> ``set_model`` + ``set_classes``)
is unchanged from the old DSL.
"""
from __future__ import annotations

import inspect
import math
import time as _time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Optional

import rclpy

from .client import MoveFailed, MoveRejected
from .model_context import ClassRef

# Outcome codes are the single source of truth in the control engine.
from duburi_control.motion_vision import (
    ALIGNED, LOST, TIMEOUT, NO_CAMERA, ABORTED,
)

if TYPE_CHECKING:
    from .duburi_dsl import DuburiMission


_CODE_NAME = {
    ALIGNED:   'ALIGNED',
    LOST:      'LOST',
    TIMEOUT:   'TIMEOUT',
    NO_CAMERA: 'NO_CAMERA',
    ABORTED:   'ABORTED',
}


@dataclass
class VisionResult:
    """Outcome of a vision verb -- truthy only when the goal was achieved.

    Branch on more than success: a mission can read WHERE and HOW the verb
    finished and run tested open-loop recovery (the hybrid vision+control
    paradigm), e.g.::

        res = duburi.vision.align('gate', yaw=0, lat=0)
        if res:                       # ALIGNED
            ...                        # proceed
        elif res.saw_target:          # tried, didn't fully centre
            if res.x_px < -30: duburi.move_right(1)
            elif res.x_px > 30: duburi.move_left(1)
        else:                         # never saw the gate
            duburi.search()

    Fields:
      ``ok``          -- True iff aligned (align) / reached fill (move).
      ``reason``/``status`` -- outcome name ('ALIGNED','LOST','TIMEOUT',
                          'NO_CAMERA','ABORTED','FAILED').
      ``code``        -- the raw integer code from the server.
      ``x_px``/``y_px`` -- SIGNED px of the target from frame CENTRE at the last
                          seen frame (+x = right, +y = below). ``nan`` when the
                          target was never seen -> use ``saw_target``.
      ``saw_target``  -- True iff the target was detected at least once.
      ``last_err_px`` -- worst residual px from the goal (align) / lateral (move).
      ``fill``        -- bbox fill fraction at exit [0..1] (move; 0 for align).
      ``elapsed_s``   -- verb duration.
    """
    ok:          bool
    reason:      str
    code:        int = TIMEOUT
    last_err_px: float = 0.0
    fill:        float = 0.0
    x_px:        float = math.nan
    y_px:        float = math.nan
    saw_target:  bool  = False
    elapsed_s:   float = 0.0

    @property
    def status(self) -> str:
        return self.reason

    def __bool__(self) -> bool:           # ``if duburi.vision.align(...):``
        return self.ok

    def __repr__(self) -> str:
        pos = ('(never seen)' if not self.saw_target
               else f'ended ({self.x_px:+.0f},{self.y_px:+.0f})px')
        extra = f' fill={self.fill * 100:.0f}%' if self.fill else ''
        return (f'VisionResult({self.reason} {pos}{extra} '
                f'err={self.last_err_px:.0f}px {self.elapsed_s:.1f}s)')


class _VisionDSL:
    """duburi.vision.* -- the closed-loop sub-namespace (align + move).

    Both verbs block until the goal achieves its outcome, times out, or
    (with a ``fallback``) exhausts its duration budget across search
    cycles. Gains and grace fall back to live ``vision.*`` ROS params
    when left unset, so deck-side tuning needs no mission edit.
    """

    def __init__(self, mission: 'DuburiMission'):
        self._dsl = mission

    # ---- target / camera resolution (unchanged mechanism) ------------- #

    def _resolve_camera(self, camera) -> str:
        return camera if camera else self._dsl.camera

    def _resolve_target(self, target, camera: Optional[str] = None) -> str:
        """string -> use as-is; ClassRef -> switch detector model/class first.

        ``camera`` selects which detector node gets reprogrammed so the
        model/class switch lands on the same node the verb will read from.
        """
        if isinstance(target, ClassRef):
            self._dsl.set_model(target.model_name, camera=camera)
            self._dsl.set_classes(target.class_name, camera=camera)
            return target.class_name
        return target if target else self._dsl.target

    def _send(self, cmd: str, **fields):
        return self._dsl._send(cmd, **fields)

    # ================================================================== #
    #  align -- centre on lat / yaw / depth at signed pixel offsets       #
    # ================================================================== #
    def align(self, target=None, *,
              lat: Optional[float] = None,
              yaw: Optional[float] = None,
              depth: Optional[float] = None,
              err: float = 40.0,
              duration: float = 20.0,
              gain: float = 30.0,
              lat_gain: Optional[float] = None,
              yaw_gain: Optional[float] = None,
              brake: bool = True,
              brake_gain: Optional[float] = None,
              hold: Optional[float] = None,
              fire=None,
              fire_t: Optional[float] = None,
              lock_on: bool = False,
              fwd: Optional[float] = None,
              fwd_mode: str = 'area',
              settle: Optional[float] = None,
              depth_step: Optional[float] = None,
              fire_pass: bool = False,
              hold_heading: bool = False,
              surge_sign: Optional[int] = None,
              max_depth_m: Optional[float] = None,
              depth_ceiling: Optional[float] = None,
              fire_gap: Optional[float] = None,
              fallback: Optional[Callable] = None,
              camera: Optional[str] = None) -> VisionResult:
        """Hold ``target`` at the requested pixel offset on each active axis.

        Pass a number to ``lat`` / ``yaw`` / ``depth`` to activate that
        axis (the number is the signed px offset from centre; ``0`` =
        centre). At least one axis is required. Returns a
        :class:`VisionResult`; never raises on a miss.

        DOWNWARD CAMERA (``camera='downward'``) -- the kwargs REMAP because the
        bottom cam looks straight down (full table: downward-camera.md)::

            forward cam :  lat=Ch6 strafe   depth=Ch3 up/down   fwd=fwd standoff(fill)
            downward cam:  lat=Ch6 strafe   fwd=Ch5 surge px     depth=descent(fill%)

        i.e. on downward ``fwd`` is the fore/aft SURGE pixel offset (image-Y, 0 =
        centre) and ``depth`` is the DESCENT target as a bbox fill %% (measured by
        ``fwd_mode`` = area/width/height). ``lat`` is unchanged. The centring axes
        are then ``lat`` + ``fwd``; ``depth`` is the optional approach. A bin drop
        reads ``align('fire', camera='downward', lat=0, fwd=0, depth=30,
        fwd_mode='height', surge_sign=…, max_depth_m=<neg floor>)``. Physically
        identical to the old ``lat/depth/fwd`` downward form -- ONLY the kwarg names
        swap; the Ch5 output and ``surge_sign`` behaviour are byte-identical, so a
        previously-verified sign stays valid. (A DISARMED ``vision_thrust_check
        --camera downward`` is still the standard pre-armed check -- it just
        confirms you're now driving surge with the ``fwd=`` kwarg, not re-hunting a
        changed sign.)

        ``gain`` is the global max-speed cap (% thrust). ``lat_gain`` /
        ``yaw_gain`` override that cap on the lat/yaw axis; leave them
        unset to inherit ``gain``. (Depth has no % cap -- its rate is set
        by ``depth_step``, metres per update.) This is for slow, stable
        per-axis micro-alignment -- e.g. ``align('hole', yaw=0, lat=0,
        gain=25, yaw_gain=10)`` creeps yaw in while lateral stays brisk
        (a 20 kg hull needs the yaw inertia fought gently to hold a tight
        torpedo-hole lock). Note ``*_gain=0`` / unset means *inherit*, NOT
        *disable* -- to drop an axis, omit it (``lat`` / ``yaw`` /
        ``depth`` = None).

        ``brake`` (on by default) reverse-kicks the lateral axis on arrival
        to bleed water inertia, so the hull stops square and the next
        mission step starts from the planned position. It is self-gating: a
        gently-converged lock (the hole-lock) exits with ~0 momentum and is
        NOT kicked, so a fire-on-align shot is never disturbed. Pass
        ``brake=False`` to coast; ``brake_gain`` scales the kick. Yaw/depth
        never brake.

        ``hold`` (seconds) turns align into an ACTIVE station-keep: once
        centred, the loop keeps running its corrections for ``hold`` s --
        fighting water inertia/current -- before returning, instead of exiting
        the instant it's centred. It holds lat/yaw/depth (and forward range when
        ``fwd`` is set). ``hold`` counts against ``duration``: budget
        ``duration >= approach + hold`` or the verb TIMEOUTs mid-hold. For a
        fire-from-lock pass ``brake=False`` so there's no pre-shot lateral nudge.

        ``fwd`` (% frame fill, optional) adds a forward range-hold axis so align
        ALSO drives the hull forward to that standoff and holds it -- ONE verb
        does forward-standoff + lat/depth centering + station-keep + mid-hold
        fire (the unified torpedo standoff shot). ``fwd_mode`` is the fill metric
        (area/width/height; ``height`` for the torpedo board/hole). The forward
        term is ONE-SIDED (drives forward while too far, neutral at/past the
        standoff -- never reverses), and the fire is gated on reaching the
        standoff too. Leave ``fwd=None`` (default) for the lat/yaw/depth-only
        align (e.g. a coarse board centre)::

            align('hole', lat=0, depth=0, fwd=25, fwd_mode='height',
                  lock_on=True, hold=4, fire=1, fire_t=1.5, brake=False)

        ``fire`` (channel int or list, e.g. ``fire=1`` or ``fire=[1, 2]`` --
        1/2=torpedo, 3/4=dropper) fires the payload WHILE the hold loop is
        still correcting, ``fire_t`` seconds into the hold (0 = at hold start),
        on a background thread so the 20 Hz correction never stalls on the
        payload write. This is the accurate-shot pattern: instead of
        ``align(hold=3)`` THEN ``fire()`` (the gap drifts the hull off the hole
        and the shot misses), the torpedo leaves WHILE glued::

            align('hole', yaw=0, lat=0, depth=0, hold=4, fire=1, fire_t=1,
                  brake=False)

        Requires ``hold > fire_t`` (else fire_t is clamped to 0 -- fire at hold
        start). Multiple channels fire one-by-one. The fire is GATED on
        alignment: it leaves on the first stably-aligned tick at or after
        ``fire_t``; if the hull never holds the lock during the hold window the
        shot is NOT fired (deliberate -- a torpedo never launches off-target).
        So budget enough ``hold`` to actually settle on the target before
        ``fire_t``. CAVEAT: on a CH340 payload reconnect the shot can leave up
        to ~2 s late (threading keeps the loop alive, it can't make the board
        faster) -- use a small ``fire_t`` and generous ``hold`` so a delayed
        shot still lands inside the hold window.

        ``settle`` (px, default None=off) is a per-call SETTLE GATE: align only
        declares aligned once the worst error is in-band AND barely moving
        (``|Δerr| <= settle``) between ticks -- so it ends SETTLED on target (like
        ``move``'s continuously-held lateral) instead of exiting mid-pass through
        the band and coasting off on inertia. Use it on a COARSE align that must
        exit clean for the next step (e.g. the board centre, so ``lock_heading``
        captures a steady heading). **Do NOT use it on a terminal fire-lock**: the
        mid-hold ``fire`` is gated on the same stable-frame counter, so a settle
        threshold below the bbox jitter (~5px) can suppress the shot -- the fire
        lock wants ``lock_on`` + ``hold`` + ``ki_lat``, not ``settle``. Keyed on
        error velocity, so a steady current does not block it (that is
        ``vision.ki_lat``'s job).

        ``lock_on`` (default False) turns on the continuity lock: once the target
        is acquired, the loop steers to the detection NEAREST the last-accepted
        centre (within a gate) instead of the largest box -- so a second hole /
        spurious box can't steal the aim during a close-in fire. Use it on the
        terminal hole/bin lock (``align('hole', lat=0, depth=0, lock_on=True,
        hold=..., fire=1)``); leave it off for far-field acquisition. The
        control-side conf floor (``vision.ctrl_conf``), the close-in gain
        softening (``vision.range_gain_floor``) and the hold integral
        (``vision.ki_lat``) are deck ROS params -- set them with
        ``ros2 param set /duburi_manager vision.<name> <value>`` (they apply on
        the next goal); see ``.claude/context/precision-alignment.md``.
        """
        cam = self._resolve_camera(camera)
        # ── DOWNWARD-CAMERA ARG SWAP (operator-facing kwarg remap) ──────────────
        # The bottom cam looks straight down, so the axes you *think* in rotate.
        # `lat` stays Ch6 strafe (image-X), but the other two SWAP so the kwargs
        # match hovering over a bin/target:
        #     fwd   -> Ch5 SURGE fore/aft  (image-Y pixel offset; 0 = centre)
        #     depth -> DEPTH DESCENT       (bbox fill %, measured by `fwd_mode`)
        # So `fwd` is ALWAYS the fore/aft joystick and `depth` ALWAYS drives the
        # real depth setpoint. This is a PURE kwarg remap: physically identical to
        # the align engine's existing downward frame-rotation (Ch5 surge from the
        # image-Y pixel, fill->depth descent) -- align_loop, the wire fields, and
        # every thruster sign are UNCHANGED. Only which kwarg you type is swapped.
        # Full axis table + why: .claude/context/downward-camera.md + CLAUDE.md.
        if cam in ('downward', 'sim_bottom'):
            depth, fwd = fwd, depth   # fwd->depth-axis (Ch5 surge); depth->fwd_fill (descent)

        active = [(name, val) for name, val in
                  (('lat', lat), ('yaw', yaw), ('depth', depth))
                  if val is not None]
        if not active:
            raise ValueError(
                "vision.align needs at least one centring axis as a KEYWORD with "
                "a number: align('gate', yaw=0, lat=0) centres on yaw+lat "
                "(0 = centre, a number = signed px offset). On camera='downward' "
                "the centring axes are lat (Ch6) + fwd (Ch5 surge); `depth` there "
                "is the fill%->descent add-on, not a centring axis. Bare names "
                "like align('gate', yaw, lat) do not work -- axes are keyword-only.")

        axes_csv = ','.join(name for name, _ in active)
        offsets = {name: float(val) for name, val in active}
        # ORDER MATTERS: program the detector (model/class for a ClassRef) + confirm
        # it's up BEFORE activating the camera, so _activate_camera's settle warms
        # the NEW config. If we resumed first (old order) the settle would warm the
        # OLD model -- wasting it and briefly emitting wrong-class boxes -- and the
        # warm-gate in _orchestrate would then have to wait all over again.
        #   1. loud preflight: abort if this camera's detector node isn't running.
        self._dsl._ensure_detector(self._dsl._detector_node(camera=cam))
        #   2. set model + classes for a ClassRef (applies even while paused).
        tgt = self._resolve_target(target, cam)
        #   3. pause the other detector, resume THIS one (now programmed), settle +
        #      point the HUD at it. Idempotent -- no-op when already live.
        self._dsl._activate_camera(cam)

        # fire: int | list | None -> CSV channels for the goal ('' = no fire).
        fire_csv = ''
        if fire is not None:
            seq = fire if isinstance(fire, (list, tuple)) else [fire]
            fire_csv = ','.join(str(int(c)) for c in seq)

        def _one_shot(remaining: float):
            return self._send(
                'vision_align',
                camera=cam, target_class=tgt, axes=axes_csv,
                offset_lat=offsets.get('lat', 0.0),
                offset_yaw=offsets.get('yaw', 0.0),
                offset_depth=offsets.get('depth', 0.0),
                err_px=float(err), duration=remaining, gain=float(gain),
                gain_lat=float(lat_gain) if lat_gain is not None else 0.0,
                gain_yaw=float(yaw_gain) if yaw_gain is not None else 0.0,
                brake_off=(not brake),
                brake_gain=float(brake_gain) if brake_gain is not None else 0.0,
                hold_s=float(hold) if hold is not None else 0.0,
                hold_through_loss=(fallback is None),
                fire_channels=fire_csv,
                fire_t=float(fire_t) if fire_t is not None else 0.0,
                lock_target=bool(lock_on),
                fwd_fill=float(fwd) if fwd is not None else 0.0,
                mode=str(fwd_mode),
                settle_px=float(settle) if settle is not None else 0.0,
                depth_step=float(depth_step) if depth_step is not None else 0.0,
                fire_pass_enabled=bool(fire_pass),
                hold_heading=bool(hold_heading),
                surge_sign=float(surge_sign) if surge_sign is not None else 0.0,
                max_depth_m=float(max_depth_m) if max_depth_m is not None else 0.0,
                depth_ceiling_m=float(depth_ceiling) if depth_ceiling is not None else 0.0,
                fire_gap=float(fire_gap) if fire_gap is not None else 0.0)

        return self._orchestrate('align', tgt, cam, duration, fallback,
                                 _one_shot)

    # ================================================================== #
    #  move -- drive forward to a bbox fill ratio                         #
    # ================================================================== #
    def move(self, target=None, *,
             fwd: Optional[float] = None,
             mode: str = 'area',
             maintain: Optional[float] = None,
             hold: Optional[float] = None,
             err: float = 40.0,
             duration: float = 20.0,
             gain: float = 30.0,
             lat_gain: Optional[float] = None,
             brake: bool = True,
             brake_gain: Optional[float] = None,
             fallback: Optional[Callable] = None,
             camera: Optional[str] = None) -> VisionResult:
        """Drive forward toward ``target``; stop at a fill ratio or pass through.

        ``fwd`` is the bbox fill % at which to stop. **Leave it unset
        (``fwd=None``, the default) to PASS THROUGH**: the AUV drives
        forward until the target is seen and then leaves the frame
        (passed / lost), plus a short commit overshoot to fully clear it
        -- this is what gets the vehicle *through* a gate. A number (e.g.
        ``fwd=80``) instead stops once the bbox fills that % (use it to
        stand off in front of a target).

        ``mode`` is the fill metric (``area`` default, ``width``,
        ``height`` for slalom). ``maintain`` (px) holds a lateral offset
        while driving (``None`` = pure forward, never touches lat/yaw/
        depth). ``hold`` (s) station-keeps once a fill target is reached
        (ignored in pass-through, where it sets the commit overshoot).
        Never re-centres yaw/depth. ``gain`` caps forward speed;
        ``lat_gain`` overrides the cap on the ``maintain`` strafe (unset
        = inherit ``gain``). ``brake`` (on by default) reverse-kicks the
        forward (and maintain) axis on a **fill-stop** arrival so the hull
        halts in front of the target instead of creeping in; PASS-THROUGH
        (``fwd=None``) never brakes -- it must coast through the gate. Pass
        ``brake=False`` to coast; ``brake_gain`` scales the kick. Returns a
        :class:`VisionResult`; never raises on a miss.
        """
        cam = self._resolve_camera(camera)
        # ORDER MATTERS (see align): program the detector BEFORE resuming it so the
        # settle warms the NEW config. move() is rejected on downward, but a forward
        # move after a downward align still needs to flip the live detector back.
        self._dsl._ensure_detector(self._dsl._detector_node(camera=cam))  # loud preflight
        tgt = self._resolve_target(target, cam)                           # set model/classes
        self._dsl._activate_camera(cam)                                   # resume + settle
        maintain_on = maintain is not None
        # fwd=None -> pass-through: wire a negative sentinel so it survives
        # the manager's `0.0 == unset` rule (which would otherwise restore
        # the 95% spec default). The control loop treats fwd_fill <= 0 as
        # "drive through until the target leaves the frame".
        fwd_fill = -1.0 if fwd is None else float(fwd)

        def _one_shot(remaining: float):
            return self._send(
                'vision_move',
                camera=cam, target_class=tgt,
                fwd_fill=fwd_fill, mode=str(mode),
                maintain_px=float(maintain) if maintain_on else 0.0,
                maintain_on=maintain_on,
                hold_s=float(hold) if hold is not None else 0.0,
                err_px=float(err), duration=remaining, gain=float(gain),
                gain_lat=float(lat_gain) if lat_gain is not None else 0.0,
                brake_off=(not brake),
                brake_gain=float(brake_gain) if brake_gain is not None else 0.0,
                hold_through_loss=(fallback is None))

        return self._orchestrate('move', tgt, cam, duration, fallback,
                                 _one_shot)

    # ================================================================== #
    #  Shared orchestration: bounded verb + client-side fallback          #
    # ================================================================== #
    def _orchestrate(self, verb: str, target: str, camera: str,
                     duration: float, fallback: Optional[Callable],
                     one_shot: Callable) -> VisionResult:
        """Run the bounded server verb; on LOST run fallback, then re-enter.

        The duration budget is owned here: each server call gets only the
        time left, and fallback cycles count against the same deadline so
        a mission step can never overrun its declared ``duration``.
        """
        # Cold-detector guard (only when a fallback is set -- without one the
        # server holds through loss and never searches, so there is nothing
        # dangerous to guard). Right after a camera / model / class switch the
        # detector is briefly cold; if we send the verb immediately its acquire
        # clock (lost_grace_s) can expire on a still-warming detector and drop
        # into an autonomous SEARCH during the switch -- off-track and silent
        # (LOST->fallback is a normal path). Block until the detector is producing
        # frames first. A warm detector (incl. a genuine "target absent" search)
        # returns instantly, so only a cold/just-switched detector pays the wait.
        if fallback is not None:
            if not self._dsl._wait_detector_warm(camera):
                self.log.warning(
                    f"[VIS  ] {verb} {target!r}: detector on {camera!r} still not "
                    f"producing frames after warm-up -- proceeding (may fall back)")

        deadline = _time.monotonic() + max(float(duration), 0.0)

        while True:
            remaining = deadline - _time.monotonic()
            if remaining <= 0.05:
                self.log.warning(
                    f"[VIS  ] {verb} {target!r}: NOT reached (duration "
                    f"elapsed) -- mission continues")
                return VisionResult(False, 'TIMEOUT', TIMEOUT)

            try:
                result = one_shot(remaining)
            except (MoveFailed, MoveRejected) as exc:
                # A vision verb must never abort a mission. Setup errors --
                # bad camera name, ALT_HOLD rejected, disarmed, server abort --
                # surface here as a non-fatal FAILED outcome so the mission
                # moves on to its next step instead of unwinding.
                self.log.error(
                    f"[VIS  ] {verb} {target!r}: server error ({exc}); "
                    f"mission continues")
                return VisionResult(False, 'FAILED', TIMEOUT)
            except Exception as exc:   # noqa: BLE001 -- never let vision kill a mission
                self.log.error(
                    f"[VIS  ] {verb} {target!r}: unexpected error {exc!r}; "
                    f"mission continues")
                return VisionResult(False, 'FAILED', TIMEOUT)
            code   = int(round(getattr(result, 'final_value', TIMEOUT)))
            err_px = float(getattr(result, 'error_value', 0.0))
            x_px   = float(getattr(result, 'end_x_px', math.nan))
            y_px   = float(getattr(result, 'end_y_px', math.nan))
            fill   = float(getattr(result, 'fill_frac', 0.0))
            elapsed = float(getattr(result, 'elapsed_s', 0.0))
            saw    = not math.isnan(x_px)

            def _mk(ok, reason):
                return VisionResult(ok, reason, code, err_px, fill,
                                    x_px, y_px, saw, elapsed)

            # Where/how it ended -- logged on EVERY terminal outcome (success too)
            # so practice notes are automatic.
            pos = (f'ended ({x_px:+.0f},{y_px:+.0f})px' if saw else 'never seen')

            if code == ALIGNED:
                self.log.info(
                    f"[VIS  ] {verb} {target!r}: ALIGNED -- {pos} "
                    f"err={err_px:.0f}px{' fill=%.0f%%' % (fill * 100) if verb == 'move' else ''}")
                return _mk(True, 'ALIGNED')

            if code == ABORTED:
                self.log.warning(f"[VIS  ] {verb} {target!r}: ABORTED -- {pos}")
                return _mk(False, 'ABORTED')

            if code == NO_CAMERA:
                self.log.error(
                    f"[VIS  ] {verb} {target!r}: NO_CAMERA -- pipeline not up "
                    f"(camera={camera!r}); mission continues")
                return _mk(False, 'NO_CAMERA')

            if code == LOST and fallback is not None:
                if _time.monotonic() >= deadline:
                    return _mk(False, 'TIMEOUT')
                self.log.info(
                    f"[VIS  ] {verb} {target!r}: target lost -- running "
                    f"fallback {getattr(fallback, '__name__', 'fn')}()")
                self._run_fallback(fallback, target, camera)
                continue   # re-enter the vision loop with remaining budget

            # LOST without fallback, or TIMEOUT: out of options for this step.
            reason = _CODE_NAME.get(code, str(code))
            self.log.warning(
                f"[VIS  ] {verb} {target!r}: NOT reached ({reason}) -- {pos} "
                f"err={err_px:.0f}px saw={saw} elapsed={elapsed:.1f}s -- "
                f"mission continues")
            return _mk(False, reason)

    def _run_fallback(self, fallback: Callable, target: str,
                      camera: str) -> None:
        """Run one fallback search cycle -- interrupted the instant the target is back.

        ``fallback(duburi)`` runs one short manoeuvre and returns.
        ``fallback(duburi, should_stop)`` may self-poll a longer sweep and
        is asked to bail the moment the target reappears.

        REGARDLESS of the signature, an interrupt is armed on the client for the
        whole cycle: the in-flight control verb is CANCELLED the instant the target
        is re-detected and every remaining verb short-circuits, so the search stops
        immediately and the vision loop re-enters while the target is still in frame
        (the failure this fixes: a long ``move_forward`` inside a 1-arg fallback ran
        to completion and carried the just-seen target back out of view). The 2-arg
        ``should_stop`` path is kept for back-compat and is now largely redundant.
        """
        # Ensure the camera's /detections is connected BEFORE the search so the very
        # first control verb can already see a reacquisition (DDS discovery latency).
        self._dsl._subscribe_detections(camera)
        start = _time.monotonic()
        should_stop = lambda: self._dsl.detected(target, camera=camera)  # noqa: E731
        try:
            # Arm INSIDE the try so the finally-disarm is structurally paired with it
            # (no edit can slip an interrupt-arm past the guaranteed disarm). Trip only
            # on a sighting stamped AFTER the search began (a stale pre-loss sighting
            # can't trip it). Pure cache read; the client's sliced goal-wait spin is
            # what refreshes the cache during a verb.
            self._dsl.client.begin_search_interrupt(
                lambda: self._dsl._seen_since(camera, target, start))  # noqa: E731
            params = inspect.signature(fallback).parameters
            if len(params) >= 2:
                fallback(self._dsl, should_stop)
            else:
                fallback(self._dsl)
        except Exception as exc:   # noqa: BLE001 -- a bad search must not kill the mission
            self.log.error(
                f"[VIS  ] fallback {getattr(fallback, '__name__', 'fn')} "
                f"raised {exc!r}; continuing")
        finally:
            if self._dsl.client.end_search_interrupt():
                self.log.info(
                    f"[VIS  ] fallback interrupted -- {target!r} reacquired; "
                    f"re-entering the vision verb")

    # ---- convenience -------------------------------------------------- #
    @property
    def log(self):
        return self._dsl.log

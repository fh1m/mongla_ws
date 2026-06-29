"""vision_dsl -- duburi.vision.* closed-loop sub-namespace (two verbs).

Exactly two mission-facing verbs, both pixel-native and recover-don't-fail:

    duburi.vision.align(target, *, lat=None, yaw=None, depth=None,
                        err=40, duration=20, gain=30,
                        lat_gain=None, yaw_gain=None, depth_gain=None,
                        brake=True, brake_gain=None, hold=None,
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
              depth_gain: Optional[float] = None,
              brake: bool = True,
              brake_gain: Optional[float] = None,
              hold: Optional[float] = None,
              fire=None,
              fire_t: Optional[float] = None,
              lock_on: bool = False,
              fallback: Optional[Callable] = None,
              camera: Optional[str] = None) -> VisionResult:
        """Hold ``target`` at the requested pixel offset on each active axis.

        Pass a number to ``lat`` / ``yaw`` / ``depth`` to activate that
        axis (the number is the signed px offset from centre; ``0`` =
        centre). At least one axis is required. Returns a
        :class:`VisionResult`; never raises on a miss.

        ``gain`` is the global max-speed cap (% thrust). ``lat_gain`` /
        ``yaw_gain`` / ``depth_gain`` override that cap on one axis;
        leave them unset to inherit ``gain``. This is for slow, stable
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
        centred, the loop keeps running its lat/yaw/depth corrections for
        ``hold`` s -- fighting water inertia/current -- before returning,
        instead of exiting the instant it's centred. It holds lat/yaw/depth
        only -- NOT forward range (the prior ``move`` set the standoff).
        ``hold`` counts against ``duration``: budget ``duration >= approach +
        hold`` or the verb TIMEOUTs mid-hold. For a fire-from-lock pass
        ``brake=False`` so there's no pre-shot lateral nudge.

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
        active = [(name, val) for name, val in
                  (('lat', lat), ('yaw', yaw), ('depth', depth))
                  if val is not None]
        if not active:
            raise ValueError(
                "vision.align needs at least one axis as a KEYWORD with a "
                "number: align('gate', yaw=0, lat=0) centres on yaw+lat "
                "(0 = centre, a number = signed px offset). Bare names like "
                "align('gate', yaw, lat) do not work -- the axes are "
                "keyword-only.")

        axes_csv = ','.join(name for name, _ in active)
        offsets = {name: float(val) for name, val in active}
        cam     = self._resolve_camera(camera)
        # Loud preflight: align needs live detections -- abort if the detector
        # node for this camera isn't running (rather than idle on err=+inf).
        self._dsl._ensure_detector(self._dsl._detector_node(camera=cam))
        tgt     = self._resolve_target(target, cam)

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
                gain_depth=float(depth_gain) if depth_gain is not None else 0.0,
                brake_off=(not brake),
                brake_gain=float(brake_gain) if brake_gain is not None else 0.0,
                hold_s=float(hold) if hold is not None else 0.0,
                hold_through_loss=(fallback is None),
                fire_channels=fire_csv,
                fire_t=float(fire_t) if fire_t is not None else 0.0,
                lock_target=bool(lock_on))

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
        # Loud preflight: move needs live detections -- abort if the detector
        # node for this camera isn't running (rather than idle on err=+inf).
        self._dsl._ensure_detector(self._dsl._detector_node(camera=cam))
        tgt = self._resolve_target(target, cam)
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
    #  anchor -- XFeat geometric "superglue" lock                         #
    # ================================================================== #
    def anchor_snap(self, name=None, *, target=None, conf=0.5, err=40.0,
                    camera=None) -> bool:
        """Capture the current view as the anchor reference (non-blocking).

        ``duburi.vision.anchor_snap()`` -- the anchor_node stores the next
        frame; a later ``anchor_align`` drives the hull back onto it. Pass
        ``name`` to ALSO save it to ``references/<name>.png`` so it survives a
        restart and can be reloaded with ``anchor_align('<name>')`` (reproducible
        runs -- snap every prop once, re-lock them later).

        ``target`` switches to **detection-gated crop snap**: the node waits up
        to 3 s for a detection of that class with score >= ``conf`` within
        ``err`` px of centre, then snaps JUST that bbox crop (keys the lock on
        the target, not the moving background -- ideal for the torpedo hole that
        YOLO loses up close). ``err<=0`` disables the centring gate. If the
        target never appears in 3 s, the whole frame is snapped instead.
        Returns True on success. Requires the anchor node (``anchor:=true``).
        """
        cam = self._resolve_camera(camera)
        try:
            res = self._send('vision_anchor_snap', camera=cam,
                             ref_name=str(name) if name else '',
                             target_class=str(target) if target else '',
                             conf=float(conf), err_px=float(err))
        except (MoveFailed, MoveRejected) as exc:
            self.log.error(f'[ANCH ] snap failed ({exc}); mission continues')
            return False
        return bool(round(getattr(res, 'final_value', 0.0)))

    def anchor_clear(self, camera=None) -> bool:
        """Drop the stored anchor reference so the next snap starts fresh."""
        cam = self._resolve_camera(camera)
        try:
            res = self._send('vision_anchor_clear', camera=cam)
        except (MoveFailed, MoveRejected) as exc:
            self.log.error(f'[ANCH ] clear failed ({exc}); mission continues')
            return False
        return bool(round(getattr(res, 'final_value', 0.0)))

    def anchor_align(self, name=None, *, err: float = 20.0, theta: float = 0.05,
                     duration: float = 30.0, hold=None, fire=None,
                     match=None, gain: float = 30.0,
                     lat_gain=None, yaw_gain=None, depth_gain=None,
                     brake: bool = True, brake_gain=None,
                     camera=None) -> VisionResult:
        """Superglue the hull to a reference (lat / yaw / depth).

        Geometric lock on XFeat+LighterGlue keypoints (no YOLO bbox needed):
        drives lat from the homography tx, yaw from theta, depth from ty until
        the live view re-superimposes within ``err`` px and ``theta`` rad, then
        actively holds for ``hold`` s. ``fire`` (int or list, e.g. ``[1, 2]``)
        fires those payload channels ONCE at first lock -- a torpedo leaves
        mid-hold while the hull is glued. ``match`` = min RANSAC inliers a tick
        must clear to count as locked (None = trust the node's verdict). No
        forward axis (a monocular homography has no metric range -- a prior
        ``move`` sets the standoff).

        ``name`` (positional) loads ``references/<name>.png`` from disk as the
        reference BEFORE locking -- ``anchor_align('hole')`` re-locks a snapshot
        saved earlier with ``anchor_snap('hole')`` (reproducible runs; no live
        snap needed). Omit it to lock on the last in-memory snap. Returns a
        :class:`VisionResult`.
        """
        cam = self._resolve_camera(camera)
        if fire is None:
            channels = ''
        elif isinstance(fire, (list, tuple)):
            channels = ','.join(str(int(c)) for c in fire)
        else:
            channels = str(int(fire))

        def _one_shot(remaining: float):
            return self._send(
                'vision_anchor_align',
                camera=cam, err_px=float(err), theta_thresh=float(theta),
                duration=remaining, gain=float(gain),
                gain_lat=float(lat_gain) if lat_gain is not None else 0.0,
                gain_yaw=float(yaw_gain) if yaw_gain is not None else 0.0,
                gain_depth=float(depth_gain) if depth_gain is not None else 0.0,
                brake_off=(not brake),
                brake_gain=float(brake_gain) if brake_gain is not None else 0.0,
                hold_s=float(hold) if hold is not None else 0.0,
                fire_channels=channels,
                min_inliers=float(match) if match is not None else 0.0,
                ref_name=str(name) if name else '')

        label = f'reference {name!r}' if name else 'reference'
        return self._orchestrate('anchor', label, cam, duration, None,
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
        """Run one fallback search cycle.

        ``fallback(duburi)`` runs one short manoeuvre and returns.
        ``fallback(duburi, should_stop)`` may self-poll a longer sweep and
        is asked to bail the moment the target reappears.
        """
        should_stop = lambda: self._dsl.detected(target, camera=camera)  # noqa: E731
        try:
            params = inspect.signature(fallback).parameters
            if len(params) >= 2:
                fallback(self._dsl, should_stop)
            else:
                fallback(self._dsl)
        except Exception as exc:   # noqa: BLE001 -- a bad search must not kill the mission
            self.log.error(
                f"[VIS  ] fallback {getattr(fallback, '__name__', 'fn')} "
                f"raised {exc!r}; continuing")

    # ---- convenience -------------------------------------------------- #
    @property
    def log(self):
        return self._dsl.log

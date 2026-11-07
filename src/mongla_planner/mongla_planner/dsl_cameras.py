"""Which camera is live: one detector at a time, because the Hailo holds one model.

Mixed into `MonglaMission` (mongla_dsl.py), which owns the client, the
node and the caches these methods read.
"""
from __future__ import annotations

import time as _time

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String


class CameraMixin:
    def camera_usable(self, name: str, *, task: str) -> bool:
        """May ``task`` run on camera ``name``? The degrade choice, CONFIGURED.

        True when the camera's detector is up. When it is not, the answer comes
        from `competition_config.CAMERA_REQUIRED` -- never from a branch written
        into the mission (plan Block 1D):

          required      -> RuntimeError, the same loud abort `_ensure_detector`
                           would give, but said BEFORE any motion is spent
          not required  -> False, after a loud SKIP line naming the task; the
                           caller returns and every later task still scores

        An unknown camera name is treated as required: a typo must not be
        mistaken for "optional and absent".
        """
        cam = str(name).strip().lower()
        if self.camera_available(cam):
            return True
        from mongla_planner.missions.competition_config import CAMERA_REQUIRED
        if CAMERA_REQUIRED.get(cam, True):
            raise RuntimeError(
                f"camera {cam!r} is ABSENT and CAMERA_REQUIRED says the run "
                f"cannot continue without it (task: {task}). Aborting loudly.")
        self.log.warning(f"[DEGRADE] camera {cam!r} absent -- SKIPPING {task}. "
                         f"Every other task in the run still scores "
                         f"(CAMERA_REQUIRED[{cam!r}] = False).")
        return False

    def camera_available(self, name: str | None = None) -> bool:
        """Is ``name``'s detector actually on the graph? Cached per camera.

        The branch a mission needs before it commits to an eye. A camera that
        fails to open no longer kills the vision stack -- the other camera keeps
        running and that camera's detector is simply absent (see
        `detector_dual_node`). That is the right behaviour for the vehicle and
        it hands the mission a decision: a bin task that steers on `downward`
        must SKIP rather than drive blind, and it can only skip if it can ask.

        Without this the mission has two bad options. Call the vision verb and
        `_ensure_detector` aborts the whole run over one dead camera -- losing
        every later task with it. Or do not call it, and the mission cannot
        tell "camera dead" from "target not in view".

        Use it as the fallback selector the task trees are built from::

            if mongla.camera_usable('downward', task='bin drop'):
                mongla.vision.align('fire', camera='downward', lat=0, fwd=0)
                mongla.fire(DROPPER_1_CHANNEL)

        -- `camera_usable` adds the CONFIGURED policy (skip or abort) on top.

        Cheap after the first call per camera: the answer is cached, because a
        detector that is up stays up for the run and one that never came up is
        not going to appear mid-mission.
        """
        cam = str(name or self.camera).strip().lower()
        if cam in self._camera_available:
            return self._camera_available[cam]
        ok = self._detector_present(cam, settle=self._DISCOVERY_SETTLE_S)
        self._camera_available[cam] = ok
        if not ok:
            self.log.warning(
                f'[CAM  ] {cam} detector is NOT on the graph. A task that '
                f'steers on {cam} will find nothing -- branch on '
                f'camera_available({cam!r}) rather than calling a vision verb.')
        return ok

    def use_camera(self, name: str) -> None:
        """Switch the sticky camera for all subsequent vision verbs AND make it the
        single live detector (pause the other, resume this one, point the HUD at it).

        Logs the switch so pool-side operators see the transition. Idempotent on the
        detector switch (no-op when already live). Best-effort: a missing detector
        node warns rather than crashing, so pure-control / single-camera runs are
        unaffected.

        Example::

            mongla.use_camera('downward')            # downward detector live, HUD flips
            mongla.vision.align('fire', lat=0, depth=0, err=30)
            mongla.use_camera('forward')             # back to forward
        """
        self.log.info(f'[MISSION] camera → {name!r}')
        self.camera = name
        # Eager-subscribe so discovery warms before the first detected()/where()
        # on the new camera (otherwise that first query false-negates).
        self._subscribe_detections(name)
        self._activate_camera(name)

    def _activate_camera(self, name: str) -> None:
        """Make ``name`` the ONLY live detector + point the HUD at it. Idempotent.

        Called by ``use_camera`` and automatically by a vision verb whose camera
        differs from the live one, so ``camera='downward'`` in a verb "just works"
        (never two detectors at once on the Hailo). Best-effort on the detector
        params: a missing node warns, doesn't raise -- a run without vision has
        no detectors and must not crash here."""
        if name == self._live_camera:
            return
        self._publish_active_camera(name)   # HUD follows (latched topic; always safe)
        # Pause EVERY known detector except the target -- not just the previously
        # live one. This makes the invariant "exactly one detector infers" hold
        # from the first switch regardless of the launch `paused` state, so a
        # stray `paused:=false` (both inferring, the OOM config) is corrected the
        # moment the mission calls use_camera. Gated on a FAST graph existence
        # check (get_node_names) so an absent counterpart is skipped instantly --
        # otherwise pause_detector -> _ensure_detector would eat its 5 s
        # wait_for_service on every switch of a single-camera run.
        for other in self._KNOWN_CAMERAS:
            if other == name:
                continue
            if not self._detector_present(other):
                continue                     # not up -> nothing to pause (no 5 s wait)
            try:
                self.pause_detector(other)   # exclusivity: only one detector runs
            except Exception as exc:         # noqa: BLE001 -- best-effort
                self.log.debug(f'[CAM  ] pause {other!r} detector skipped: {exc}')
        switched = False
        try:
            self.resume_detector(name)
            switched = True
        except Exception as exc:            # noqa: BLE001 -- best-effort (no detector node)
            self.log.warning(f'[CAM  ] resume {name!r} detector skipped: {exc}')
        if switched:
            # Per-mission override wins; else the class default headroom.
            settle = getattr(self, 'cam_switch_settle_s', None)
            _time.sleep(self._CAM_SWITCH_SETTLE_S if settle is None else float(settle))
        # let first live frames land before the next verb steers on them
        self._live_camera = name

    def _publish_active_camera(self, name: str) -> None:
        """Publish the active camera on a LATCHED topic so the HUD auto-follows the
        mission (manual f/d keys remain an override). Lazily create the publisher."""
        try:
            if self._active_cam_pub is None:
                qos = QoSProfile(
                    depth=1,
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)   # latched
                self._active_cam_pub = self.client.node.create_publisher(
                    String, '/mongla/vision/active_camera', qos)
            self._active_cam_pub.publish(String(data=str(name)))
        except Exception as exc:            # noqa: BLE001 -- HUD follow is best-effort
            self.log.warning(f'[CAM  ] active_camera publish skipped: {exc}')

    def calc_distance(self, phase: str):
        """Downward optical-flow distance bracket. OWNS the camera switch.

        ``calc_distance('start')`` enters distance mode: pause BOTH YOLO detectors
        (LK flow is CPU -- no detector runs, freeing the GPU + honouring "not
        simultaneous with YOLO"), point the HUD at the downward camera, then latch
        the axis + reset the accumulator (facade verb). Run your axis-locked move
        between start and stop. ``calc_distance('stop')`` freezes + returns the
        accumulated METRES, then RESTORES the camera/detector that was live before
        (so forward vision missions resume as usual). move_* verbs are never touched.

            mongla.calc_distance('start')
            mongla.move_forward(4.0, gain=60)      # blind timed move, measured
            metres = mongla.calc_distance('stop')

        Returns the accumulated distance (float, metres) on 'stop'; the raw
        Move.Result on 'start'.
        """
        p = str(phase or '').strip().lower()
        if p == 'start':
            # Remember what to restore, then enter distance mode (both detectors
            # paused = no YOLO; HUD -> downward). Best-effort: absent detectors are
            # quiet no-ops (single-camera / control-only runs unaffected).
            self._pre_distance_camera = getattr(self, '_live_camera', None) or self.camera
            for cam in self._KNOWN_CAMERAS:
                try:
                    self.pause_detector(cam)
                except Exception as exc:          # noqa: BLE001
                    self.log.debug(f'[DIST ] pause {cam!r} skipped: {exc}')
            self._publish_active_camera('downward')
            self.camera = 'downward'
            self.log.info('[MISSION] distance mode ON (downward, detectors paused)')
            return self._send('calc_distance', phase='start')

        # stop: freeze + read metres, then restore the prior camera/detector.
        result = self._send('calc_distance', phase='stop')
        metres = float(getattr(result, 'final_value', 0.0))
        prior  = getattr(self, '_pre_distance_camera', None) or 'forward'
        try:
            self.use_camera(prior)                # resumes the prior detector + HUD
        except Exception as exc:                  # noqa: BLE001
            self.log.warning(f'[DIST ] restore camera {prior!r} skipped: {exc}')
        self.log.info(f'[MISSION] distance mode OFF -> {metres:+.3f}m  (camera -> {prior!r})')
        return metres

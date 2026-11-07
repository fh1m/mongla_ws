"""What the cameras see: detected(), wait_for(), where(), outline().

Mixed into `MonglaMission` (mongla_dsl.py), which owns the client, the
node and the caches these methods read.
"""
from __future__ import annotations

import time as _time

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray

from .model_context import ClassRef

from .dsl_support import _DetView, _eval_where, _outline_view, _parse_detections, _pick_outline


class SeeMixin:
    def _wait_detector_warm(self, camera: str, timeout: float | None = None) -> bool:
        """Block until the detector for `camera` is PRODUCING /detections frames.

        Gates on the detector being ALIVE (publishing any frame -- the detector
        emits a frame every inference tick, empty or not), NOT on the target being
        visible. So a just-switched / cold detector is given time to warm up before
        a vision verb's acquire clock starts, while a genuine "target simply absent"
        search is NOT delayed: a warm detector is already publishing empty frames,
        so this returns on the first pump. Returns True once producing, False on
        timeout (detector never came alive -- caller proceeds + likely falls back).
        """
        budget = self._DETECTOR_WARMUP_S if timeout is None else float(timeout)
        self._subscribe_detections(camera)
        deadline = _time.monotonic() + max(budget, 0.0)
        while _time.monotonic() < deadline:
            self._pump_detections(camera)
            entry = self._det_cache.get(camera)
            if entry is not None and (_time.monotonic() - entry[0]) <= self._DETECTOR_FRESH_S:
                return True
        return False

    def _subscribe_detections(self, camera: str) -> None:
        """Subscribe a camera's /detections + /camera_info (idempotent)."""
        if camera in self._det_subs:
            return
        node = self.client.node
        self._subscribe_heading_history(node)
        # Match VisionState's QoS so we connect to the same publishers.
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        ns = f'/mongla/vision/{camera}'
        self._det_subs[camera] = node.create_subscription(
            Detection2DArray, f'{ns}/detections',
            lambda msg, cam=camera: self._on_detections(cam, msg), qos)
        self._info_subs[camera] = node.create_subscription(
            CameraInfo, f'{ns}/camera_info',
            lambda msg, cam=camera: self._on_info(cam, msg), qos)

    def _subscribe_heading_history(self, node) -> None:
        """Keep the hull's yaw by CAPTURE stamp, so a detection's bearing can
        use the heading of its own frame (issue #37). Idempotent."""
        if getattr(self, '_state_sub', None) is not None:
            return
        try:
            from mongla_interfaces.msg import MonglaState
        except ImportError:                       # an unbuilt workspace
            return
        from collections import deque
        self._yaw_hist = deque(maxlen=400)        # ~20 s at the 20 Hz state
        self._state_sub = node.create_subscription(
            MonglaState, '/mongla/state', self._on_state_yaw, 10)

    def _on_state_yaw(self, msg) -> None:
        y = float(msg.yaw_deg)
        st = msg.header.stamp
        t = float(st.sec) + float(st.nanosec) * 1e-9
        if y == y and t > 0.0:
            self._yaw_hist.append((t, y))

    def _turned_since(self, camera: str) -> float:
        """Degrees the hull has turned since `camera`'s newest frame was
        captured, or 0.0 when either end is unknown (then nothing changes)."""
        hist = list(getattr(self, '_yaw_hist', ()) or ())
        cap = self.__dict__.get('_det_capture', {}).get(camera)
        if not hist or not cap:
            return 0.0
        t_cap = float(cap[0]) + float(cap[1]) * 1e-9
        hist.sort(key=lambda h: h[0])
        before = [y for t, y in hist if t <= t_cap]
        if not before:
            return 0.0
        return ((hist[-1][1] - before[-1] + 180.0) % 360.0) - 180.0

    def _on_detections(self, camera: str, msg) -> None:
        # Copy to plain Python immediately (rclpy reuses the C++ buffer across
        # callbacks; holding the message would corrupt the cache).
        now     = _time.monotonic()
        records = _parse_detections(msg)
        self._det_cache[camera] = (now, records)        # latest frame: where() reads this
        # The frame's CAPTURE stamp, so a fix computed from it can say when it
        # was true (the filter applies it there with `retrodict:=true`).
        stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
        if stamp is not None:
            caps = self.__dict__.setdefault('_det_capture', {})
            caps[camera] = (int(stamp.sec), int(stamp.nanosec))
        seen = self._det_seen.setdefault(camera, {})    # per-class last-seen: detected() reads this
        for rec in records:
            seen[rec[0]] = now                          # rec[0] = lowercased class
        self._det_warm.add(camera)

    def _on_info(self, camera: str, msg) -> None:
        if msg.width and msg.height:
            self._img_size[camera] = (float(msg.width), float(msg.height))
            k = [float(x) for x in msg.k]
            # A zeroed K is what CameraInfo carries before a calibration loads:
            # present is not usable, so only a positive focal is kept.
            if len(k) >= 9 and k[0] > 0.0 and k[4] > 0.0:
                self._cam_k[camera] = (k[0], k[4], k[2], k[5])
                # The lens travels with K (issue #24): every metric path below
                # removes it before the port model.
                self.__dict__.setdefault('_cam_d', {})[camera] = tuple(
                    float(x) for x in (msg.d or ()))

    def _resume_default_detector(self, camera: str) -> None:
        """Resume the default camera's detector, once, at first use.

        Why this is not inside the query. Every query funnels through
        `_pump_detections`, and none of them reached `_activate_camera` -- only
        `use_camera` and the vision verbs did. So against a launch that starts
        both detectors paused (which is the correct default: two live models
        alternate on one Hailo at 37.5 Hz per pair against 95.3 Hz for one),
        `while not mongla.detected('gate')` polls a detector that will never
        infer: an empty cache, forever, no error and no timeout.

        And why the settle is shorter than the recency window. Finding the
        detector means waiting for DDS discovery, and the wait happens INSIDE
        the caller's query. At 1.5 s it made `detected(stale_after=1.0)` --
        "seen within the last second" -- meaningless: frames that arrived just
        before the call aged out DURING it, and `wait_for('red_pipe',
        timeout=3.0)` returned False against five frames published moments
        earlier. `_DISCOVERY_SETTLE_S` must stay well under the 1.0 s default
        `stale_after`, and it is paid at most ONCE PER CAMERA -- a repeat probe
        cannot learn anything new inside one mission, and paying it per query
        would make a `while not detected(...)` search crawl.

        Best-effort by construction: no detector on the graph (a pure-control
        mission, a bag replay) means there is nothing to resume, and the
        mission proceeds exactly as it did before.
        """
        if self._live_camera is not None or camera in self._resume_probed:
            return
        # Frames already in hand beat any graph probe: a cache entry for this
        # camera IS the detector, alive and publishing. Skipping here keeps the
        # settle out of every query after the first frame lands -- which is
        # what stops it eating the recency window a later query measures.
        if self._det_cache.get(camera) is not None:
            self._resume_probed.add(camera)
            return
        self._resume_probed.add(camera)             # once per camera, win or lose
        if self._detector_present(camera, settle=self._DISCOVERY_SETTLE_S):
            self._activate_camera(camera)

    def _pump_detections(self, camera: str) -> None:
        """Spin the node until a /detections frame newer than now arrives.

        The detector publishes every frame, so a live pipeline lands a fresh
        frame within one frame period; a stalled or just-subscribed pipeline
        times out and the caller reads no (or stale) data -> correctly absent.
        """
        # BEFORE the timing reference below: see `_resume_default_detector`.
        self._resume_default_detector(camera)
        node = self.client.node
        start = _time.monotonic()
        budget = self._PUMP_WARM_S if camera in self._det_warm else self._PUMP_COLD_S
        deadline = start + budget
        while _time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=self._PUMP_SLICE_S)
            entry = self._det_cache.get(camera)
            if entry is not None and entry[0] >= start:
                return   # a frame stamped after we started -> cache is current

    def _records(self, camera: str, stale_after: float) -> list:
        """Latest-frame detection records for `camera`, or [] when stale/absent.

        Used by where()/where_offset() -- bearing must reflect the CURRENT frame.
        detected()/wait_for() use _present() instead (per-class recency window).
        """
        entry = self._det_cache.get(camera)
        if entry is None:
            return []
        stamp, records = entry
        if _time.monotonic() - stamp > stale_after:
            return []
        return records

    def _present(self, camera: str, needle: str, within: float) -> bool:
        """True iff `needle` was seen on `camera` within the last `within` s.

        Flicker-tolerant: a class that drops from individual raw frames (low
        FPS, motion blur) still counts as present until `within` elapses since
        its last sighting -- so a `while not detected()` search reacquires the
        instant the target reappears instead of chasing per-frame dropouts.
        """
        n    = str(needle).strip().lower()
        last = self._det_seen.get(camera, {}).get(n)
        return last is not None and (_time.monotonic() - last) <= within

    def _seen_since(self, camera: str, needle: str, t: float) -> bool:
        """True iff `needle` was seen on `camera` in a frame that arrived at/after `t`.

        PURE cache read (no pump) -- the caller's own spin refreshes `_det_seen`.
        Used by the vision-verb fallback interrupt: keying on "seen AFTER the search
        began" makes a stale pre-loss sighting unable to trip it, so the interrupt
        fires only on a genuine reacquisition mid-search.
        """
        last = self._det_seen.get(camera, {}).get(str(needle).strip().lower())
        return last is not None and last >= t

    def detected(self, target_class, *,
                 camera: str | None = None,
                 stale_after: float = 1.0) -> bool:
        """Was `target_class` seen within the last `stale_after` s on `camera`?

        A recency check: it pumps the detection stream so the cache is current,
        then returns True iff the class was seen within the `stale_after` window
        (per-class last-seen, NOT just the single latest frame -- so it tolerates
        the per-frame flicker raw `/detections` shows at low FPS). Use it to
        branch (``if``) or to drive a moving search (``while``):

            # branch once on what is currently visible
            if mongla.detected('gate'):
                mongla.vision.align('gate', yaw=0, lat=0)

            # search WHILE MOVING -- needs a loop (an `if` runs once!)
            while not mongla.detected('red_pipe'):
                mongla.move_left(2)
            mongla.move_forward(3)

        To wait for a target while holding station, prefer ``wait_for`` (one
        call, no busy-loop). Works inside a vision ``fallback`` too -- the
        search re-enters the verb the moment the target reappears.

        Parameters
        ----------
        target_class : str | ClassRef
            Class name (``'gate'``) or a ``mongla.models.<m>.<c>`` handle.
        camera : str | None
            Camera to query. Defaults to ``mongla.camera``.
        stale_after : float
            Recency window: the class counts as present until this many seconds
            after its last sighting (bridges per-frame flicker). A class not seen
            for longer than this counts as absent. Lower it where you need the
            False edge to be prompt (e.g. gating an irreversible fire/drop).
        """
        if isinstance(target_class, ClassRef):
            target_class = target_class.class_name
        cam = camera or self.camera
        self._subscribe_detections(cam)
        self._pump_detections(cam)
        return self._present(cam, target_class, stale_after)

    def can_see(self, camera: str | None = None, *, timeout: float = 1.0):
        """Can `camera` see at all? `'ok'`, a blind reason, or None if unknown.

        Blind reasons: `'covered'`, `'washout'`, `'glare'`, `'frozen'`. Use it
        to tell "the target is not there" from "the camera cannot show it":

            while not mongla.detected('gate'):
                if mongla.can_see() not in ('ok', None):
                    break          # searching blind wastes the whole budget
                mongla.yaw_right(20)

        `'ok'` means none of those faults, not a good image. Defocus and mild
        fog are NOT detectable from the image alone without firing on ordinary
        murky footage -- measured on the real 2025 archive, see seeing.py.
        None = the detector has not published (paused camera, not running).
        """
        import time as _t
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        from std_msgs.msg import String
        cam = camera or self.camera
        node = self.client.node
        subs = self.__dict__.setdefault('_seeing_subs', {})
        vals = self.__dict__.setdefault('_seeing_vals', {})
        if cam not in subs:
            def _keep(msg, cam=cam):
                vals[cam] = str(msg.data)
            latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            subs[cam] = node.create_subscription(
                String, f'/mongla/vision/{cam}/seeing', _keep, latched)
        deadline = _t.monotonic() + float(timeout)
        while cam not in vals and _t.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        rclpy.spin_once(node, timeout_sec=0.0)
        return vals.get(cam)

    def wait_for(self, target_class, *,
                 timeout: float = 10.0,
                 camera: str | None = None,
                 stale_after: float = 1.0) -> bool:
        """Block until `target_class` is seen on `camera`, or `timeout` elapses.

        The loop-free way to (re)acquire a target while stationary -- returns
        True the moment it appears, False if `timeout` passes first. Each poll
        pumps the stream, so this keeps the cache fresh without a busy-loop::

            if mongla.wait_for('gate', timeout=8):
                mongla.vision.align('gate', yaw=0, lat=0)
            else:
                sweep_yaw(mongla, should_stop)   # your fallback; never showed up

        For a search that should KEEP MOVING while looking, use a
        ``while not mongla.detected(...): <small move>`` loop instead.
        """
        if isinstance(target_class, ClassRef):
            target_class = target_class.class_name
        cam = camera or self.camera
        self._subscribe_detections(cam)
        deadline = _time.monotonic() + max(float(timeout), 0.0)
        while _time.monotonic() < deadline:
            self._pump_detections(cam)
            if self._present(cam, target_class, stale_after):
                return True
        return False

    def where(self, target_class, *,
              camera: str | None = None,
              stale_after: float = 1.0,
              band: float = 0.15) -> str:
        """Bearing of `target_class`: 'left' | 'center' | 'right' | 'unknown'.

        Picks the largest matching detection and reports which side of frame
        centre it sits on (``band`` = centre dead-zone half-width, normalized).
        ``'unknown'`` = not visible (or camera_info not seen yet). Image-frame
        semantics: ``'left'`` means the target is on the left, so yaw left to
        face it (same polarity as the vision-yaw axis)::

            {'left': lambda: mongla.yaw_left(20),
             'right': lambda: mongla.yaw_right(20),
            }.get(mongla.where('gate'), lambda: mongla.move_forward(1))()

        Use ``where_offset`` for the raw signed offset (fine steering).
        """
        label, _ = self._where_eval(target_class, camera, stale_after, band)
        return label

    def side_on(self, symbol: str, *, structure: str = 'gate',
                camera: str | None = None, stale_after: float = 1.0,
                use_outline: bool = False) -> str:
        """Which side OF THE STRUCTURE the symbol is on: left/right/centre.

        Not `where()`, and the difference decides the gate. `where()` answers
        "left or right of the FRAME", which is where the camera is pointing.
        The gate's divider is a property of the gate, so a hull sitting off to
        one side reads a correctly-placed placard as the wrong side and flies
        under the wrong half -- with the detector, the pose and the control loop
        all working. This measures against the structure's own midline, so it
        survives being off-axis::

            if mongla.side_on('rescue') == 'left':
                mongla.move_left(1.2)
            mongla.vision.move('gate', fwd=None)      # pass through

        Returns ``'unknown'`` when the structure is not visible or the symbol
        is not on it -- two states a mission branches differently on, so check
        `detected(structure)` to tell them apart. Geometry comes from the
        structure and identity from the symbol, never the reverse: a placard is
        occluded from oblique angles and makes a poor geometric object.
        """
        cam = self._resolve_camera(camera) if hasattr(self, '_resolve_camera') \
            else (camera or self.camera)
        self._pump_detections(cam)
        records = self._records(cam, stale_after)
        if not records:
            return 'unknown'
        from mongla_vision.identity import identify, pick_structure

        dets = [_DetView(r) for r in records]
        struct = pick_structure(dets, structure)
        if struct is None:
            return 'unknown'
        want = str(symbol).strip().lower()
        symbols = [d for d in dets if d.class_name == want]
        if use_outline:
            # OPT-IN: the symbol's segmentation OUTLINE (contours topic) instead
            # of its box -- its visible pixels decide overlap and side, which a
            # half-occluded placard's box cannot. Falls back to boxes silently
            # when no outline is available (box model, contours off).
            o = self.outline(want, camera=cam, stale_after=stale_after)
            view = _outline_view(o) if o is not None else None
            if view is not None:
                symbols = [view]
        got = identify(struct, symbols)
        return got.side if got.label else 'unknown'

    def where_offset(self, target_class, *,
                     camera: str | None = None,
                     stale_after: float = 1.0):
        """Signed normalized horizontal offset [-1,+1] of `target_class`.

        Negative = left of centre, positive = right; ``None`` when not visible
        or image width unknown. The continuous companion to ``where``.
        """
        _, offset = self._where_eval(target_class, camera, stale_after, 0.0)
        return offset

    def _where_eval(self, target_class, camera, stale_after, band) -> tuple:
        if isinstance(target_class, ClassRef):
            target_class = target_class.class_name
        cam = camera or self.camera
        self._subscribe_detections(cam)
        self._pump_detections(cam)
        width = self._img_size.get(cam, (0.0, 0.0))[0]
        return _eval_where(self._records(cam, stale_after),
                           target_class, width, band)

    def outline(self, target_class, *, camera: str | None = None,
                stale_after: float = 1.0, timeout: float = 1.0):
        """The largest outline of `target_class` in the latest contours frame, or None.

        Returns `Outline(class_name, score, angle_deg, area_px, points)`, `points`
        a list of (x, y) image pixels. A box model gives 4 corners, a segmentation
        model the mask outline -- same call either way. `angle_deg` is the
        oriented-box angle (a path marker's direction). Needs the detector's
        `publish_contours` on (`contours:=true`, the vehicle launch default).
        """
        import time as _t
        from mongla_interfaces.msg import TargetContours
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        if isinstance(target_class, ClassRef):
            target_class = target_class.class_name
        cam = camera or self.camera
        subs = self.__dict__.setdefault('_contour_subs', {})
        frames = self.__dict__.setdefault('_contours', {})
        node = self.client.node
        if cam not in subs:
            def _keep(msg, c=cam):
                frames[c] = (_t.monotonic(), msg)
            subs[cam] = node.create_subscription(
                TargetContours, f'/mongla/vision/{cam}/contours', _keep,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        deadline = _t.monotonic() + float(timeout)
        rclpy.spin_once(node, timeout_sec=0.0)
        while cam not in frames and _t.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        entry = frames.get(cam)
        if entry is None or _t.monotonic() - entry[0] > float(stale_after):
            return None
        return _pick_outline(entry[1], str(target_class))

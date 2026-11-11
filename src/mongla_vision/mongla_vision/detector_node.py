#!/usr/bin/env python3
"""detector_node -- subscribe to image_raw, run YOLO11 (yolov11n), publish detections.

Topics:
  in    /mongla/vision/<cam>/image_raw       sensor_msgs/Image
  out   /mongla/vision/<cam>/detections      vision_msgs/Detection2DArray
  out   /mongla/vision/<cam>/image_debug     sensor_msgs/Image  (rate-limited overlay)
  out   /mongla/vision/<cam>/classes_filter  std_msgs/String    (comma-sep active class list)

Single-model launch (unchanged from v1):
-----------------------------------------
  ros2 run mongla_vision detector_node --ros-args -p camera:=forward \\
      -p model_path:=gate_sharks -p classes:=gate -p conf:=0.45

  Live class switch (no restart):
      ros2 param set /mongla_detector_forward classes flare
      ros2 param set /mongla_detector_forward classes "gate,flare"
      ros2 param set /mongla_detector_forward classes ""   # all classes

Multi-model launch (registry mode):
-------------------------------------
  Declare a registry with ``models`` (CSV ``name=stem`` pairs).  Each
  named entry is loaded at startup.  Switch between them mid-mission with
  ``active_model``:

  ros2 run mongla_vision detector_node --ros-args -p camera:=forward \\
      -p models:="gate=gate_sharks,sauvc=sauvc_sim" \\
      -p active_model:=gate -p classes:=gate -p conf:=0.45

  Live model + class switch (no restart):
      ros2 param set /mongla_detector_forward active_model sauvc
      ros2 param set /mongla_detector_forward classes orange_flare

  Via launch file (bringup.launch.py):
      ros2 launch mongla_manager bringup.launch.py vision:=true \\
          vision_stack:=generic models:="gate=gate_sharks,sauvc=sauvc_sim" \\
          active_model:=gate classes:=gate

  Via mission DSL:
      mongla.use('flare')               # switch model + inherit its default classes
      mongla.use('combined', 'gate')    # switch model, immediately filter to 'gate'
      mongla.set_classes('flare')       # class-only switch (model unchanged)

THE FILES. `DetectorNode` is one class in four files:

    detector_node.py     the inference loop and its failure recovery
    detector_setup.py    parameters, model loading, live parameter changes
    detector_frames.py   how a frame reaches the detector (mailbox, direct feed)
    detector_publish.py  every topic written, and the operator log lines
    detector_params.py   pure parameter parsing, tested without a node
"""

import os
os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}')

import queue as _queue
import sys
import threading
import time

# Consecutive inference failures before the detector is rebuilt, and before the
# process gives up and exits. Not one number: an ISOLATED failure is a bad
# frame and dropping it is right, while a RUN of them is the device. Sized in
# frames rather than seconds so it behaves the same at 3 Hz and at 80 -- 15
# frames is 0.2 s at 80 Hz and 5 s at 3 Hz, and in both cases it is well past
# "one unlucky frame". See BUGS.md D16.
_INFER_FAIL_REBUILD = 15
_INFER_FAIL_EXIT = 45
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import replace
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge        import CvBridge

from mongla_vision import qos
from mongla_vision.detection.detector  import largest
from mongla_vision.detection.factory   import make_detector
from mongla_vision.detection.detector  import Detector
from mongla_vision.detection.messages  import detections_to_array
from mongla_vision.detection.preprocess import make_preprocessor
from mongla_vision.detection.rangecrop  import RangeCrop
from mongla_vision.detection.profiles   import resolve as resolve_profile
from .detector_params import (_UNSET, _is_unset, _model_stem,
                              _parse_models_param, _split_active)
from .detector_frames import _DIRECT_FALLBACK_S
from .detector_setup import DetectorSetupMixin
from .detector_frames import DetectorFramesMixin
from .detector_publish import DetectorPublishMixin


class DetectorNode(DetectorSetupMixin, DetectorFramesMixin, DetectorPublishMixin, Node):
    def __init__(self, node_name: str = 'mongla_detector', *,
                 parameter_overrides=None, init_failure_fn=None):
        """`node_name` and `parameter_overrides` exist so TWO of these can live
        in one process, which is not a nicety on the Pi -- it is the only way
        the two-camera path runs at all.

        The Hailo chip allows one VDevice per PROCESS: a second detector
        process gets HAILO_OUT_OF_PHYSICAL_DEVICES (74). Launch's own `name=`
        and `parameters=` are process-wide remappings (`__node:=`, `__params:=`)
        and cannot address two nodes in one process, so the name and the
        parameter set have to arrive as arguments instead. Defaults keep the
        single-node executable byte-identical in behaviour.
        """
        super().__init__(node_name,
                         parameter_overrides=list(parameter_overrides or []))
        # Called instead of exiting when the model fails to load -- set by a
        # composing owner that has another camera to keep. None: exit, as a
        # standalone detector must.
        self._init_failure_fn = init_failure_fn

        self._declare_parameters()

        self._cam_name = str(self.get_parameter('camera').value).strip() or 'cam'
        ns_in  = str(self.get_parameter('image_topic').value).strip() \
                 or f'/mongla/vision/{self._cam_name}/image_raw'
        ns_out = f'/mongla/vision/{self._cam_name}'

        classes_param = str(self.get_parameter('classes').value).strip()
        allowlist = (
            None if not classes_param
            else [c.strip() for c in classes_param.split(',') if c.strip()]
        )

        device   = str(self.get_parameter('device').value)
        # Resolve the profile FIRST, then let explicit parameters override it.
        # `_p()` below returns the operator's value when they set one and the
        # profile's when they did not, so `vision:=murky preprocess:=off` is
        # a coherent request rather than a contradiction.
        prof_name = str(self.get_parameter('vision_profile').value or '').strip()
        prof = {}
        if prof_name:
            try:
                prof, why = resolve_profile(prof_name)
                self.get_logger().info(
                    f"[DET  ] vision profile {prof_name!r}: {why}")
                self.get_logger().info(f"[DET  ] -> {prof}")
            except ValueError as exc:
                self.get_logger().error(f'[DET  ] {exc}')
                raise

        def _p(name, default):
            """Explicit parameter wins; else the profile; else the default.

            "explicit" is decided by a sentinel, not by comparing to the
            DEFAULT, and the difference is not academic. A launch file always
            passes every parameter -- `preprocess:='none'`, `range_crop:=False`
            -- so a comparison against the declared default sees the launch's
            own defaults as deliberate operator choices and the profile never
            applies. Measured on the vehicle: `vision:=murky` logged that it
            had applied while `range_crop` read False and `conf` read 0.15.

            So a knob is "unset" when it holds its documented sentinel:
            `preprocess:'auto'`, `range_crop:-1`, `conf:0.0`. Anything else is
            a real request and wins.
            """
            v = self.get_parameter(name).value
            sentinel = _UNSET.get(name)
            if sentinel is not None and _is_unset(name, v):
                return prof.get(name, default)
            if not prof:
                return v
            return v if not _is_unset(name, v) else prof.get(name, default)

        # NOT routed through `_p`: see the note on `_UNSET`.
        conf     = float(self.get_parameter('conf').value)
        iou      = float(self.get_parameter('iou').value)
        imgsz    = int(self.get_parameter('imgsz').value)
        max_det  = int(self.get_parameter('max_det').value)
        half     = bool(self.get_parameter('half').value)

        # ── Registry (multi-model) ─────────────────────────────────────
        models_str   = str(self.get_parameter('models').value).strip()
        active_model = str(self.get_parameter('active_model').value).strip()
        self._registry: Dict[str, Detector] = {}
        # stem -> registry-key index so set_model() accepts the STEM as well as the
        # launch key (missions/ClassRef switch by stem). Empty in single-model mode.
        self._stem_to_key: Dict[str, str] = {}
        # Stem of the one model in single-model mode (None in registry mode). Lets
        # set_model(<that stem>) be a no-op success instead of a hard reject.
        self._single_model_name: Optional[str] = None

        self._pending_allowlist = allowlist  # updated by param callback; used by async loader

        if models_str:
            model_map = _parse_models_param(models_str)
            self.get_logger().info(
                f"[DET  ] registry mode: loading {len(model_map)} model(s) in parallel: "
                f"{list(model_map)}")

            log = self.get_logger()

            def _load_one(name: str, stem: str):
                # The backend follows the extension the resolver found on THIS
                # machine: .hef on the Pi + AI HAT+, .pt on a
                # dev box. Missions pass stems, so nothing above this changes.
                det = make_detector(
                    model_path=stem,
                    device=device, conf=conf, iou=iou, imgsz=imgsz,
                    half=half, max_det=max_det, class_allowlist=allowlist,
                    logger=log)
                return name, det

            errors: list = []
            with ThreadPoolExecutor(max_workers=len(model_map)) as pool:
                futures = {pool.submit(_load_one, n, s): n for n, s in model_map.items()}
                for fut in as_completed(futures):
                    n = futures[fut]
                    try:
                        name_out, det = fut.result()
                        self._apply_assoc_conf(det)
                        self._registry[name_out] = det
                        stem = _model_stem(model_map[name_out])
                        # Collision (two keys, same stem) => last wins + warn, so
                        # a stem lookup stays deterministic.
                        if (stem in self._stem_to_key
                                and self._stem_to_key[stem] != name_out):
                            self.get_logger().warning(
                                f"[DET  ] stem {stem!r} maps to keys "
                                f"{self._stem_to_key[stem]!r} and {name_out!r}; "
                                f"set_model({stem!r}) will use {name_out!r}")
                        self._stem_to_key[stem] = name_out
                        self.get_logger().info(
                            f"[DET  ] registry[{name_out!r}] ready (stem {stem!r})")
                    except Exception as exc:
                        # Resilient: a failed model is SKIPPED, not fatal, so one
                        # missing .pt (e.g. slalom/torpedo weights not yet on the
                        # box) can't take the WHOLE pipeline down. set_model() to a
                        # skipped model fails clearly at the moment it's needed.
                        self.get_logger().error(
                            f"[DET  ] registry[{n!r}] FAILED — SKIPPED: {exc}")
                        errors.append(n)

            if errors:
                self.get_logger().error(
                    f"[DET  ] {len(errors)} model(s) FAILED and were SKIPPED: "
                    f"{errors}  |  loaded: {list(self._registry)}  "
                    f"(pipeline stays UP; set_model to a skipped model will reject)")
            if not self._registry:
                self.get_logger().fatal(
                    "[DET  ] ALL registry models failed to load — no pipeline")
                raise RuntimeError(f"empty detector registry (all failed: {errors})")

            # Registry is non-empty from here. active_model may be a KEY or a
            # STEM, and may name SEVERAL, comma-separated -- the same form the
            # live parameter takes, so a launch argument and a mid-mission
            # switch cannot mean different things.
            active_model, extra_names = _split_active(active_model)
            active_key = self._resolve_model_key(active_model) if active_model else None
            if active_key is not None:
                self._det: Detector = self._registry[active_key]
                self._active_name: Optional[str] = active_key
            else:
                first = next(iter(self._registry))
                self._det = self._registry[first]
                self._active_name = first
                if active_model:
                    # LOUD: the requested startup model failed/absent, so we are
                    # about to detect with a DIFFERENT model — an operator must see
                    # this or lose a run to a silently-substituted model.
                    self.get_logger().error(
                        f"[DET  ] active_model={active_model!r} NOT loaded "
                        f"(failed or absent) — falling back to {first!r}. "
                        f"loaded: {list(self._registry)}")
            # AFTER the primary is settled: `_set_extra` skips a name that is
            # already the primary, and it can only know that once it is chosen.
            self._set_extra(extra_names)

        else:
            # Single-model mode: load async so ROS subscriber starts immediately.
            # Frames received before the model is ready are silently dropped.
            self._active_name = None
            self._det: Optional[Detector] = None
            # Canonical name of the one loaded model, known synchronously from the
            # launch arg. set_model(<this stem>) is then a no-op SUCCESS (already
            # active) instead of a hard "no registry" reject -- so a ClassRef or
            # mongla.use('<stem>') mission works on a plain single-model launch.
            self._single_model_name = _model_stem(
                str(self.get_parameter('model_path').value))
            threading.Thread(
                target=self._load_single_model_async,
                kwargs=dict(
                    model_path=str(self.get_parameter('model_path').value),
                    device=device, conf=conf, iou=iou, imgsz=imgsz,
                    half=half, max_det=max_det, allowlist=allowlist),
                daemon=True).start()

        from vision_msgs.msg import Detection2DArray, VisionInfo
        from std_msgs.msg import String
        self._bridge       = CvBridge()
        # Depth 1 BEST_EFFORT: the publisher's mailbox is only a mailbox if
        # the subscriber is one too. A depth-5 RELIABLE queue here (which is
        # what an int `5` means, and also what `qos_profile_sensor_data` gives)
        # asks the middleware to hold and retransmit frames for a consumer that
        # is going to throw all but the newest away in `_on_image` anyway --
        # buying latency and CPU for nothing.
        # Direct feed. When a composed process hands frames straight in (see
        # `vision_stack_node`), subscribing as well would decode and infer the
        # same picture twice -- and the topic copy is the SLOWER of the two,
        # so it would also be the one the detector acted on half the time.
        # Built once. `make_preprocessor` returns None for 'off' so the hot
        # loop can skip the call entirely rather than paying for an identity.
        try:
            self._pre = make_preprocessor(
                _p('preprocess', 'off'),
                float(_p('preprocess_clip', 3.0)) or 3.0)
        except ValueError as exc:
            self.get_logger().error(f'[DET  ] {exc}')
            self._pre = None
        if self._pre is not None:
            self.get_logger().info(
                f"[DET  ] preprocessing ON "
                f"(clahe clip={float(self.get_parameter('preprocess_clip').value)}) "
                f"-- ~3.8 ms/frame, measured 10.7 % -> 56.2 % presence on "
                f"blurry footage")

        self._crop = (RangeCrop()
                      if int(_p('range_crop', 0)) > 0
                      else None)
        if self._crop is not None:
            self.get_logger().info(
                '[DET  ] range crop ON -- centre 50 % while the target is '
                'small, full frame on approach. Recall at 4x range 66 % -> '
                '100 %, at the cost of half the field of view while active.')

        self._direct  = bool(self.get_parameter('direct_feed').value)
        self._ns_in   = ns_in
        self._fed_direct = False
        self._sub = None if self._direct else self.create_subscription(
            Image, ns_in, self._on_image, qos.IMAGE)
        if self._direct:
            # The self-correction. A one-shot timer, not a permanent one: once
            # it has either seen a direct frame or subscribed, there is
            # nothing left to decide.
            self._fallback_timer = self.create_timer(
                _DIRECT_FALLBACK_S, self._check_direct_feed)
        self._pub_det      = self.create_publisher(
            Detection2DArray, f'{ns_out}/detections', qos.DETECTIONS)
        # Every box down to the association floor: `detections` plus the ones
        # under the acting bar, for the tracker and the lock ladder only. The
        # same boxes as `detections` while `assoc_conf` is off.
        self._pub_cand     = self.create_publisher(
            Detection2DArray, f'{ns_out}/candidates', qos.DETECTIONS)
        # OUTLINES, on the same stamp as the detections beside them. Built for
        # every detection, box or mask alike -- see TargetContours.msg for why
        # the two are one message. Optional: a workspace without
        # mongla_interfaces built keeps detecting.
        self._pub_contours = None
        if bool(self.get_parameter('publish_contours').value):
            try:
                from mongla_interfaces.msg import TargetContours
                self._pub_contours = self.create_publisher(
                    TargetContours, f'{ns_out}/contours', qos.DETECTIONS)
            except Exception as exc:                        # noqa: BLE001
                self.get_logger().warning(
                    f'[DET  ] no {ns_out}/contours topic: {exc!r}')
        # LATCHED: the HUD and the console both join AFTER the detector and
        # must still learn the allowlist. This topic being VOLATILE is exactly
        # why the console polls `get_parameters` for `classes` instead.
        self._pub_classes  = self.create_publisher(
            String, f'{ns_out}/classes_filter', qos.LATCHED)
        # Can this camera see? 'ok' | 'covered' | 'washout' | 'glare' | 'frozen',
        # latched, on change. No boxes means nothing is there OR the camera is
        # blind, and a mission cannot tell those apart without this. Thresholds
        # measured on the real 2025 archive; see mongla_vision/seeing.py.
        from mongla_vision.seeing import Seeing
        self._seeing = Seeing()
        self._seeing_state = None
        self._pub_seeing = self.create_publisher(
            String, f'{ns_out}/seeing', qos.LATCHED)
        # Which model produced the detections you are holding. A mission
        # switches the detector mid-run -- gate, then rescue, then red_pipe --
        # and a consumer acting on a box that a PREVIOUS model produced is
        # steering at the wrong thing while everything looks healthy. Nothing
        # said which model any message came from, and the console polls a
        # PARAMETER at 1 Hz, which cannot answer that about a message.
        #
        # `database_version` is bumped on every switch, so a consumer compares
        # it across two reads and knows a switch happened in between rather
        # than guessing. This is `vision_msgs`' own mechanism for it -- an
        # invented topic would be a second answer to a solved question.
        # LATCHED: a late joiner must still learn the model, same reason as
        # the allowlist beside it.
        self._pub_vinfo = self.create_publisher(
            VisionInfo, f'{ns_out}/vision_info', qos.LATCHED)
        self._model_epoch = 0
        self._publish_dbg = bool(self.get_parameter('publish_debug_image').value)
        if self._publish_dbg:
            self._pub_dbg = self.create_publisher(
                Image, f'{ns_out}/image_debug', qos.DEBUG_IMAGE)
            dbg_hz = max(float(self.get_parameter('debug_image_hz').value), 0.5)
            self._dbg_min_dt = 1.0 / dbg_hz
            self._last_dbg = 0.0

        self._frames        = 0
        self._with_target   = 0
        self._infer_total_s = 0.0
        self._last_log      = time.monotonic()
        self._last_align_log = 0.0
        self.create_timer(2.0, self._log_health)
        # Slow and at INFO, unlike `_log_health`, because this is an
        # OPERATIONAL property and not a debugging one: the chip's compute is
        # the scarce resource, so the pipeline's contract is that every inference
        # lands on the newest frame available. 0.1 Hz is invisible in a log
        # and enough to notice a drift.
        self.create_timer(10.0, self._log_efficiency)

        self._device_str = device
        self._deadband   = float(self.get_parameter('alignment_deadband').value)

        # Per-model conf overrides from launch (applied on top of the uniform
        # `conf`). Single-model mode loads async, so a pending override is stored
        # and re-applied when the model lands (see _load_single_model_async).
        self._pending_model_conf = str(self.get_parameter('model_conf').value)
        self._apply_model_conf(self._pending_model_conf)

        self.add_on_set_parameters_callback(self._on_parameter_change)

        # Inference runs on a background thread so the ROS executor stays free
        # for param callbacks (live class/model switches) during long inferences.
        self._infer_q: _queue.SimpleQueue = _queue.SimpleQueue()
        # Serialises the PRODUCERS (issue #37). Drain-then-put is two steps;
        # with `_on_image` and the direct feed both offering, both could drain
        # and both put, leaving two frames -- and the worker would infer the
        # stale one, which a single slot exists to prevent. The consumer only
        # takes, so it needs no lock.
        self._offer_lock = threading.Lock()
        # Set while the worker is BLOCKED waiting for a frame, i.e. exactly
        # when a new one would be consumed immediately. A composed camera reads
        # this to decide when to decode, so the detector is fed the instant it
        # goes idle instead of on the publisher's clock -- which is the whole
        # latency argument for composing them.
        self._want = threading.Event()
        # Compute-waste accounting. The chip's compute is the scarce resource, so
        # an inference spent on a frame that was already superseded is compute
        # we cannot afford. These make that measurable instead of assumed:
        #   _evicted    a decoded frame replaced in the slot before it was
        #               ever inferred -- a wasted DECODE (~1.9 ms of a core)
        #   _stale_sum  age of the frame at the instant inference STARTS,
        #               which is the floor set by capture + decode and the
        #               number to watch if it ever grows
        self._evicted = 0
        self._infers = 0
        self._stale_sum = 0.0
        self._stale_max = 0.0
        self._infer_fails = 0
        threading.Thread(target=self._infer_loop_guarded, daemon=True).start()

        registry_info = (
            f"  registry={list(self._registry)}  active={self._active_name!r}"
            if self._registry else ''
        )
        self.get_logger().info(
            f"[DET  ] subscribed {ns_in!r} -> {ns_out}/detections  "
            f"({'+ image_debug' if self._publish_dbg else 'no debug image'})"
            f"{registry_info}")

        # Publish initial classes so display_node picks up the configured list
        # on connect (even before any param change fires).
        self._publish_classes(classes_param)
        # Epoch 1 at startup, so a consumer that joins late has a model
        # name and a baseline to compare against, rather than silence
        # until the first switch.
        self._publish_vision_info()

    def _on_infer_failure(self, exc) -> None:
        """A node that cannot do its job must stop claiming to be up.

        What this replaces (BUGS.md D16). The old handler logged and
        continued, forever. Observed on the vehicle: a `HAILO_STREAM_ABORT(63)`
        left this node ALIVE -- process up, topics up, subscriptions up, `pgrep`
        satisfied -- logging a failure on every frame and publishing zero
        detections indefinitely. Every liveness check we own passed. Only the
        detection RATE showed it, and nothing was watching the rate.

        Three tiers, because the failures are not one thing:

          1. ISOLATED failures are tolerated. A single bad frame, a transient
             decode fault -- dropping it and carrying on is right, and killing
             the node for one would be worse than the fault.
          2. CONSECUTIVE failures mean the DEVICE is gone, not the frame. Try
             to rebuild the detector through the same path that built it.
          3. If rebuilding does not help either, EXIT non-zero so a supervisor
             restarts the process. Staying up is the failure mode, not the
             recovery.

        The counter resets on any success, so a chip that recovers by itself
        never reaches tier 2.
        """
        self._infer_fails = getattr(self, '_infer_fails', 0) + 1
        n = self._infer_fails
        if n < _INFER_FAIL_REBUILD:
            self.get_logger().error(
                f'[DET  ] inference failed ({n}/{_INFER_FAIL_REBUILD}): {exc!r}')
            return
        if n == _INFER_FAIL_REBUILD:
            self.get_logger().error(
                f'[DET  ] {n} consecutive inference failures -- the DEVICE is '
                f'gone, not the frame. Rebuilding the detector. Last: {exc!r}')
            self._rebuild_detector()
            return
        if n >= _INFER_FAIL_EXIT:
            self.get_logger().fatal(
                f'[DET  ] {n} consecutive inference failures and a rebuild did '
                f'not help. EXITING so a supervisor can restart this process: a '
                f'node that cannot infer must not keep claiming to be up. '
                f'Last: {exc!r}')
            os._exit(1)          # noqa: SLF001 -- rclpy shutdown cannot be
            #                       trusted from a worker thread mid-fault, and
            #                       the point is to stop, loudly and now.

    def _rebuild_detector(self) -> None:
        """Re-run the construction that produced the detector in the first
        place. Drops the old one first: on the Hailo path the device is held
        by the object, and a second VDevice while the first lives is
        `HAILO_OUT_OF_PHYSICAL_DEVICES`."""
        kw = getattr(self, '_build_kwargs', None)
        if not kw:
            self.get_logger().error('[DET  ] cannot rebuild: no build kwargs '
                                    '(registry mode) -- will exit instead')
            return
        old, self._det = self._det, None
        try:
            del old
        except Exception:                       # noqa: BLE001
            pass
        try:
            self._load_single_model_async(**kw)
            if self._det is not None:
                # The counter is not reset here (issue #59). Construction is
                # not recovery: the Hailo constructor even swallows its own
                # warm-up inference errors. Resetting on a successful BUILD let
                # a device that builds and never infers rebuild every 15
                # failures for ever (150 failures -> 10 rebuilds, no exit) and
                # never reach the exit below. Only a successful INFERENCE
                # (`_infer_loop`) clears the count -- what the docstring always
                # promised.
                self.get_logger().warn('[DET  ] detector REBUILT after '
                                       'consecutive inference failures -- it '
                                       'must now INFER to count as recovered')
        except Exception as exc:                # noqa: BLE001
            self.get_logger().error(f'[DET  ] rebuild failed: {exc!r}')

    def _infer_loop_guarded(self):
        """`_infer_loop`, but a raise is LOGGED and the loop restarts.

        A python thread that raises simply ends. No traceback in the node's
        log, no exit code, no fault anywhere -- the node stays up, the
        subscriptions stay up, the publisher stays registered, and
        `/detections` goes quiet for ever. Only `det.infer()` was guarded;
        every other statement in the loop body ran bare.

        Measured on the vehicle (B-XF1): `image_raw` flowing at 24.7 Hz with
        the detector subscribed, `/detections` carrying a live publisher and
        ZERO messages, and not one error line to say why. That is the same
        invisibility D16 removed from the infer-exception path, reached
        through every OTHER statement in the loop.

        Restarting rather than exiting: a detector that loses one frame is
        recoverable, one that silently stops is not. The fault is counted so a
        loop failing every frame is loud rather than a log flood.
        """
        faults = 0
        while rclpy.ok():
            try:
                self._infer_loop()
                return                      # clean exit: rclpy shutting down
            except Exception as exc:        # noqa: BLE001
                faults += 1
                if faults <= 3 or faults % 100 == 0:
                    import traceback
                    self.get_logger().error(
                        f'[DET  ] inference loop DIED ({faults}x) and was '
                        f'restarted: {exc!r}\n{traceback.format_exc()}')
                time.sleep(0.05)

    def _infer_loop(self):
        """Worker thread: decode + infer + publish (never touches the ROS executor).

        One frame per pass, in the order the steps are written. Each step that
        can drop the frame returns None, and the loop moves on to the next one."""
        while rclpy.ok():
            item = self._next_frame()
            if item is None:
                continue
            got = self._decode(item)
            if got is None:
                continue
            frame, header = got
            self._report_seeing(frame)
            t0 = time.monotonic()
            self._note_frame_age(header)
            frame = self._preprocess(frame)
            det = self._det  # atomic ref read under CPython GIL
            if det is None:
                continue  # model still loading — drop frame, keep queue drained
            out = self._run_detector(det, frame)
            if out is None:
                continue
            detections, infer_frame, crop_state = out
        # AFTER the counter is cleared, and outside this try. The
        # failure counter escalates to REBUILDING THE PRIMARY DETECTOR,
        # and the primary just succeeded -- letting a secondary model's
        # exception land in the same counter would tear down a healthy
        # model because a different one misbehaved. Caught by
        # `test_a_SUCCESSFUL_inference_in_the_LOOP_resets_the_counter`,
        # which went from 0 failures to 5.
            act = det.acting_conf()
            weak = [d for d in detections if d.score < act]
            if weak:
                detections = [d for d in detections if d.score >= act]
                if crop_state is not None and crop_state.active:
                    weak = [replace(d, xyxy=crop_state.to_full(d.xyxy))
                            for d in weak]
            detections = self._merge_extra(infer_frame, detections)
            detections = self._back_to_full_frame(frame, detections, crop_state)
            dt = time.monotonic() - t0

            self._frames        += 1
            self._infer_total_s += dt
            primary = largest(detections)
            if primary is not None:
                self._with_target += 1
                self._log_alignment(primary, frame)

            det_msg = detections_to_array(detections, header)
            if not rclpy.ok():
                return
            try:
                self._pub_det.publish(det_msg)
                self._pub_cand.publish(
                    detections_to_array(detections + weak, header)
                    if weak else det_msg)
            except Exception:
                return  # node being destroyed; exit thread cleanly
            self._publish_contours(frame, detections, header)
            self._publish_debug_image(frame, detections, header, dt, primary)

    # ── the steps of one inference pass ─────────────────────────────────────
    # Each was a span of `_infer_loop` until 2026-11-07; code and comments
    # moved verbatim.

    def _run_detector(self, det, frame):
        """Infer, inside the range crop when one is active.

        -> (detections, the frame the model saw, the crop state), or None on a
        failed inference (counted by `_on_infer_failure`)."""
        # RANGE CROP, decided from the PREVIOUS frame's target size --
        # the only size available before inferring this one.
        crop_state = None
        infer_frame = frame
        if self._crop is not None:
            infer_frame, crop_state = self._crop.apply(frame)
        try:
            detections = det.infer(infer_frame)
            # B-XF1 diagnostic: what the model returned AT THE SOURCE,
            # before any merge, crop remap or filtering. Throttled -- hot
            # path.
            self._infer_dbg = getattr(self, '_infer_dbg', 0) + 1
            if self._infer_dbg % 150 == 1:
                self.get_logger().info(
                    f'[DET  ] infer() returned {len(detections)} raw '
                    f'detection(s) on {infer_frame.shape[1]}x'
                    f'{infer_frame.shape[0]}'
                    + (f', best {max(d.score for d in detections):.3f}'
                       if detections else ''))
            self._infer_fails = 0
        except Exception as exc:
            self._on_infer_failure(exc)
            return None
        return detections, infer_frame, crop_state

    def _back_to_full_frame(self, frame, detections, crop_state):
        """Detections in full-frame pixels, and the crop told how big the target was."""
        if crop_state is not None and crop_state.active and detections:
            # Back to full-frame coordinates. Everything downstream -- the
            # pixel error, the bearing, the HUD -- is full-frame, and a
            # missed offset does not raise: it steers at a point displaced
            # by the crop origin, which reads as a calibration fault.
            detections = [
                replace(d, xyxy=crop_state.to_full(d.xyxy))
                for d in detections]
        if self._crop is not None:
            # Feed back the largest target's area as a fraction of the
            # frame the DETECTOR ACTUALLY SAW -- cropped or not. That is
            # the quantity the exit threshold is defined against: while
            # cropped, a target filling the crop is close even though it
            # is a small fraction of the full frame.
            big = largest(detections)
            frac = None
            if big is not None:
                area = ((big.xyxy[2] - big.xyxy[0])
                        * (big.xyxy[3] - big.xyxy[1]))
                if crop_state is not None and crop_state.active:
                    seen = crop_state.w * crop_state.h
                else:
                    seen = frame.shape[1] * frame.shape[0]
                frac = area / float(max(seen, 1))
            self._crop.observe(frac, time.monotonic())
        return detections

def main():
    from mongla_localization.orphan import die_with_parent
    die_with_parent()        # holds the accelerator
    rclpy.init()
    node = DetectorNode()
    from mongla_localization.params_file import refuse_unknown
    refuse_unknown(node)       # bringup.yaml `nodes:` typos stop here
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
    sys.exit(0)

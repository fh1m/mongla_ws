#!/usr/bin/env python3
"""detector_node -- subscribe to image_raw, run YOLO11 (yolov11n), publish detections.

Topics:
  in    /duburi/vision/<cam>/image_raw       sensor_msgs/Image
  out   /duburi/vision/<cam>/detections      vision_msgs/Detection2DArray
  out   /duburi/vision/<cam>/image_debug     sensor_msgs/Image  (rate-limited overlay)
  out   /duburi/vision/<cam>/classes_filter  std_msgs/String    (comma-sep active class list)

Single-model launch (unchanged from v1):
-----------------------------------------
  ros2 run duburi_vision detector_node --ros-args -p camera:=forward \\
      -p model_path:=gate_flare_medium_100ep -p classes:=gate -p conf:=0.45

  Live class switch (no restart):
      ros2 param set /duburi_detector classes flare
      ros2 param set /duburi_detector classes "gate,flare"
      ros2 param set /duburi_detector classes ""   # all classes

Multi-model launch (registry mode):
-------------------------------------
  Declare a registry with ``models`` (CSV ``name=stem`` pairs).  Each
  named entry is loaded at startup.  Switch between them mid-mission with
  ``active_model``:

  ros2 run duburi_vision detector_node --ros-args -p camera:=forward \\
      -p models:="gate=gate_nano_100ep,flare=flare_medium_100ep,combined=gate_flare_medium_100ep" \\
      -p active_model:=gate -p classes:=gate -p conf:=0.45

  Live model + class switch (no restart):
      ros2 param set /duburi_detector active_model flare
      ros2 param set /duburi_detector classes flare

  Via launch file (bringup.launch.py):
      ros2 launch duburi_manager bringup.launch.py vision:=true \\
          models:="gate=gate_nano_100ep,combined=gate_flare_medium_100ep" \\
          active_model:=gate classes:=gate

  Via mission DSL:
      duburi.use('flare')               # switch model + inherit its default classes
      duburi.use('combined', 'gate')    # switch model, immediately filter to 'gate'
      duburi.set_classes('flare')       # class-only switch (model unchanged)
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
from typing import Dict, List, NamedTuple, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge        import CvBridge

from duburi_vision import draw, qos
from duburi_vision.detection.detector  import largest
from duburi_vision.detection.factory   import make_detector
from duburi_vision.detection.detector  import Detector
from duburi_vision.detection.messages  import detections_to_array
from duburi_vision.detection.preprocess import make_preprocessor
from duburi_vision.detection.rangecrop  import RangeCrop
from duburi_vision.detection.profiles   import resolve as resolve_profile

# Throttle for the always-on operator alignment line (seconds). Matches the
# control-side vision throttle so the rate feels consistent between the detector
# status line and the manager's mission logs.
ALIGN_LOG_THROTTLE_S = 0.5

# How long a `direct_feed` detector waits for its first frame by
# reference before deciding there is no camera in its process and
# subscribing instead. Long enough that a slow camera open does not
# trip it, short enough to be invisible at startup.
_DIRECT_FALLBACK_S = 2.0

# "NOT SET BY THE OPERATOR". A launch file passes every parameter, so the
# only way to tell a deliberate choice from a filled-in default is a value
# that means nothing on its own. Each is outside the setting's real range.
_UNSET = {
    # `conf` is DELIBERATELY ABSENT. It also feeds the tracker's confidence
    # clamp through the launch (`detector_conf`), and a sentinel there would
    # silently disable the clamp -- which measured as the tracker emitting
    # NOTHING on real water. A profile therefore cannot lower `conf` below an
    # explicit launch value; set both, or set neither.
    'preprocess': 'auto',     # 'off'/'clahe' are the real values
    'preprocess_clip': 0.0,   # a real clip limit is > 0
    'range_crop': -1,         # an int, because a bool cannot carry a third state
}


def _is_unset(name, value) -> bool:
    """True when `value` is the documented sentinel for `name`."""
    s = _UNSET.get(name)
    if s is None:
        return False
    if isinstance(s, str):
        return str(value).strip().lower() == s
    return value == s


def _split_active(spec: str):
    """`active_model` -> (primary, [secondaries]).

    ONE param, not two, and comma-separated rather than a second
    `extra_models` key. Two params cannot be set atomically, so a mission
    switching from `a` to `b,c` would pass through a tick where the primary is
    already `b` while the secondary list still says the old model -- a frame
    detected with the wrong pair, and nothing would log it. A single CSV
    changes the whole selection in one `SetParameters` call.

    No comma is byte-for-byte the behaviour that shipped before this existed.
    """
    parts = [p.strip() for p in str(spec or '').split(',') if p.strip()]
    if not parts:
        return '', []
    return parts[0], parts[1:]


def _parse_models_param(s: str) -> Dict[str, str]:
    """Parse 'gate=gate_nano_100ep,flare=flare_medium_100ep' → {name: stem}.

    Bare entries without '=' use the stem as both name and path:
        'gate_nano_100ep' → {'gate_nano_100ep': 'gate_nano_100ep'}
    """
    result: Dict[str, str] = {}
    for part in s.split(','):
        part = part.strip()
        if not part:
            continue
        if '=' in part:
            k, v = part.split('=', 1)
            result[k.strip()] = v.strip()
        else:
            result[part] = part
    return result


def _model_stem(path: str) -> str:
    """Canonical model identity: basename without extension.

    ``'gate_rescue_repair'`` -> ``'gate_rescue_repair'``;
    ``'/models/gate_rescue_repair.pt'`` -> ``'gate_rescue_repair'``.
    Missions name models by this stem (``duburi.models('stem')`` / ClassRef /
    ``set_model('stem')``), so the node matches ``active_model`` against it
    regardless of whether the model was launched single (``model:=stem``) or in a
    registry (``models:=key=stem`` or bare ``models:=stem``).
    """
    return os.path.splitext(os.path.basename(str(path).strip()))[0]


def _parse_model_conf(s: str) -> Dict[str, float]:
    """Parse 'torpedo_blood_hole=0.55,gate=0.35' -> {name: conf_float}.

    Skips malformed / non-numeric pairs silently (a live-tuned param must never
    crash the callback). Bare entries (no '=') are ignored -- a per-model conf
    needs a model name.
    """
    result: Dict[str, float] = {}
    for part in s.split(','):
        part = part.strip()
        if not part or '=' not in part:
            continue
        name, _, value = part.partition('=')
        name, value = name.strip(), value.strip()
        if not name:
            continue
        try:
            result[name] = float(value)
        except ValueError:
            continue
    return result


class _DirectFrame(NamedTuple):
    """A decoded frame handed straight across in one process.

    A tuple rather than a bare ndarray so the HEADER travels with the pixels.
    They are separable only by accident: the header carries the CAPTURE time,
    and every freshness gate downstream -- `_freshness`, the coast ladder, the
    mid-hold torpedo fire -- reads it.
    """
    frame: object
    header: object


class DetectorNode(Node):
    def __init__(self, node_name: str = 'duburi_detector', *,
                 parameter_overrides=None):
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

        self.declare_parameter('camera',              'laptop')
        self.declare_parameter('image_topic',         '')
        self.declare_parameter('model_path',          'yolov11n')
        self.declare_parameter('models',              '')     # CSV name=stem pairs
        self.declare_parameter('active_model',        '')     # registry key to start with
        self.declare_parameter('device',              'cuda:0')
        self.declare_parameter('half',                True)   # fp16: ~half the VRAM; coerced off on non-CUDA (yolo.py)
        self.declare_parameter('conf',                0.35)
        # BYTE association floor: PUBLISH down to this, while `conf` stays the
        # floor the control loop steers on (`vision.ctrl_conf`). 0.0 = OFF, and
        # off is byte-for-byte the previous behaviour.
        #
        # Measured on real footage, publish floor the ONLY variable: presence on
        # hard clips 8.3 -> 34.8 %, and NO change on clips already at 100 %. The
        # low-score boxes are the occluded and motion-blurred ones (ByteTrack),
        # which is exactly the AUV case -- our own underwater score p50 is 0.258
        # against a shipped 0.25 floor, so we were discarding the median.
        self.declare_parameter('assoc_conf',          0.0)
        # Per-model confidence overrides: CSV 'name=conf' (e.g.
        # 'torpedo_blood_hole=0.55,gate_rescue_repair=0.35'). Applies on top of
        # the uniform `conf` above, targeting individual registry entries, and
        # PERSISTS across active_model switches (each detector holds its own
        # threshold). Empty = every model uses `conf`. Live-tunable.
        self.declare_parameter('model_conf',          '')
        self.declare_parameter('iou',                 0.5)
        self.declare_parameter('imgsz',               640)
        self.declare_parameter('max_det',             100)   # post-NMS cap (live-tunable)
        self.declare_parameter('classes',             'person')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_image_hz',      5.0)
        self.declare_parameter('alignment_deadband',  0.05)
        self.declare_parameter('paused',              False)
        # ON BY DEFAULT, WITH A FALLBACK, because "off" and "on" both have a
        # silent failure mode and only one of them is recoverable.
        #
        # Direct feed is the arrangement the vehicle runs: the camera in this
        # process hands frames over by reference. Defaulting it OFF meant the
        # composed launcher had to remember to switch it on, and forgetting
        # left the detector subscribing AND being fed -- the same picture
        # decoded and inferred twice, with the slower copy winning half the
        # time.
        #
        # Defaulting it ON has the opposite risk: a STANDALONE `detector_node`
        # (vision.launch.py, the Jetson, sim, replay) has no camera in its
        # process, so it would sit forever receiving nothing, silently. That
        # is the exact class of bug this round has been removing.
        #
        # So it is on, and it SELF-CORRECTS: if no frame arrives by reference
        # within `_DIRECT_FALLBACK_S`, the node subscribes to the topic and
        # says so loudly. A misconfiguration costs one warning and two
        # seconds, never a dead detector.
        self.declare_parameter('direct_feed',         True)
        # UNDERWATER CONTRAST PREPROCESSING. 'clahe' or 'off'.
        #
        # Measured on real RoboSub 2025 footage: on the gate approach -- 3.7x
        # blurrier than the bin clip at the same brightness -- it takes target
        # presence from 10.7 % to 56.2 % and the mean score from 0.226 to
        # 0.410. On footage that already works it costs nothing: bin stays
        # 100 %, octagon stays 100 % with a HIGHER mean score.
        #
        # Off by default because it is 3.78 ms on the Pi (77 Hz -> ~46), which
        # is a real trade the operator should make deliberately.
        self.declare_parameter('preprocess',          'auto')
        self.declare_parameter('preprocess_clip',     0.0)
        # RANGE CROP. Feed the detector a centre crop while the target is far,
        # so it occupies more of the 640x640 the chip sees. Measured on real
        # labelled data: recall at 4x the training distance goes 65.9 % ->
        # 100 %, and at 6.7x, 20.2 % -> 67.4 %. It releases automatically on
        # approach because a fixed crop LOSES close targets (69 % at 1x).
        # `imgsz` cannot do this on the Hailo -- it is baked into the HEF.
        # An INT, not a bool: 0/1 are the operator's answer and -1 is
        # 'not set', which a bool cannot express -- and without a third
        # state a profile can never turn this on.
        self.declare_parameter('range_crop',          -1)
        # ONE WORD instead of four knobs. `vision:=murky` on competition day
        # beats getting conf/preprocess/clip/crop right under a run clock.
        # A profile supplies DEFAULTS ONLY -- an explicit parameter still
        # wins, so debugging keeps the individual knobs.
        self.declare_parameter('vision_profile',      '')

        self._cam_name = str(self.get_parameter('camera').value).strip() or 'cam'
        ns_in  = str(self.get_parameter('image_topic').value).strip() \
                 or f'/duburi/vision/{self._cam_name}/image_raw'
        ns_out = f'/duburi/vision/{self._cam_name}'

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

            "EXPLICIT" IS DECIDED BY A SENTINEL, NOT BY COMPARING TO THE
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
                # machine: .hef on the Pi + AI HAT+, .engine on the Jetson, .pt
                # anywhere. Missions pass stems, so nothing above this changes.
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

            # Registry is non-empty from here. active_model may be a KEY or a STEM.
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

        else:
            # Single-model mode: load async so ROS subscriber starts immediately.
            # Frames received before the model is ready are silently dropped.
            self._active_name = None
            self._det: Optional[Detector] = None
            # Canonical name of the one loaded model, known synchronously from the
            # launch arg. set_model(<this stem>) is then a no-op SUCCESS (already
            # active) instead of a hard "no registry" reject -- so a ClassRef or
            # duburi.use('<stem>') mission works on a plain single-model launch.
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
        # DIRECT FEED. When a composed process hands frames straight in (see
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
        # OUTLINES, on the same stamp as the detections beside them. Built for
        # every detection, box or mask alike -- see TargetContours.msg for why
        # the two are one message. Optional: a workspace without
        # duburi_interfaces built keeps detecting.
        self._pub_contours = None
        try:
            from duburi_interfaces.msg import TargetContours
            self._pub_contours = self.create_publisher(
                TargetContours, f'{ns_out}/contours', qos.DETECTIONS)
        except Exception as exc:                            # noqa: BLE001
            self.get_logger().warning(
                f'[DET  ] no {ns_out}/contours topic: {exc!r}')
        # LATCHED: the HUD and the console both join AFTER the detector and
        # must still learn the allowlist. This topic being VOLATILE is exactly
        # why the console polls `get_parameters` for `classes` instead.
        self._pub_classes  = self.create_publisher(
            String, f'{ns_out}/classes_filter', qos.LATCHED)
        # ⛔ WHICH MODEL PRODUCED THE DETECTIONS YOU ARE HOLDING. A mission
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
        # OPERATIONAL property and not a debugging one: the chip has ~45 % of
        # the Orin's TOPS, so the pipeline's contract is that every inference
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
        # Set while the worker is BLOCKED waiting for a frame, i.e. exactly
        # when a new one would be consumed immediately. A composed camera reads
        # this to decide when to decode, so the detector is fed the instant it
        # goes idle instead of on the publisher's clock -- which is the whole
        # latency argument for composing them.
        self._want = threading.Event()
        # COMPUTE-WASTE ACCOUNTING. The chip has ~45 % of the Orin's TOPS, so
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
        threading.Thread(target=self._infer_loop, daemon=True).start()

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

    # ------------------------------------------------------------------ #
    #  Running more than one model on the same frame
    # ------------------------------------------------------------------ #
    def _set_extra(self, names) -> List[str]:
        """Resolve the SECONDARY models -- the ones run beside the primary.

        ⛔ WHY A SECONDARY LIST AND NOT A PLURAL `self._det`. Everything that
        makes a model "the" model -- the class allowlist, `vision_info`, the
        alignment line, the debug overlay's model tag -- is written against a
        single active detector in eighteen places. Pluralising that would turn
        a wiring change into a rewrite of the node's identity handling, and
        every one of those eighteen would then have to decide what "the model"
        means when there are two. The primary keeps its meaning; the extras
        only contribute detections.

        A key that does not resolve is dropped with a LOUD error rather than
        refused, for the same reason the registry tolerates a model whose
        weights are missing: losing the second model must not cost the run.
        """
        self._extra = []
        self._extra_names = []
        for name in names:
            key = self._resolve_model_key(name)
            if key is None or self._registry.get(key) is None:
                self.get_logger().error(
                    f"[DET  ] active_model names {name!r} as a second model "
                    f"and it is not loaded -- running without it. keys="
                    f"{sorted(self._registry)}")
                continue
            if key == self._active_name:
                continue           # naming the primary twice is not two models
            self._extra.append(self._registry[key])
            self._extra_names.append(key)
        if self._extra_names:
            self.get_logger().warning(
                f"[DET  ] running {1 + len(self._extra_names)} models on every "
                f"frame: {self._active_name!r} + {self._extra_names}. The chip "
                f"runs one graph at a time and each handover costs ~4 ms, so "
                f"the frame rate is the SUM plus the swaps -- measured 37.5 Hz "
                f"per pair against 95 Hz for one. Deliberate, not free.")
        return self._extra_names

    def _merge_extra(self, frame, detections: list) -> list:
        """Run each secondary model on the same frame and merge the results.

        ⛔ CLASS IDS ARE REMINTED FROM LABELS WHEN MODELS ARE MERGED, and only
        then. Two models each number their own classes from zero, so
        `gate`(0) from one and `person`(0) from the other collide in-process --
        the debug palette gives them one colour and any consumer that keys on
        the integer merges two classes. `class_index` is the same label->id
        table the wire round-trip already uses, so the merged ids agree with
        what a subscriber will mint. The single-model path does not go through
        here and is byte-identical to before.
        """
        # `getattr`, not `self._extra`: a node built by a harness that
        # bypasses both init paths has no such attribute, and an AttributeError
        # here would surface as an inference failure rather than as itself.
        if not getattr(self, '_extra', None):
            return detections
        from .detection.messages import remint_class_ids
        out = list(detections)
        for det in self._extra:
            try:
                out.extend(det.infer(frame))
            except Exception as exc:                        # noqa: BLE001
                self.get_logger().warning(
                    f"[DET  ] secondary model failed on this frame: {exc!r}")
        return remint_class_ids(out)

    def _load_single_model_async(self, *, model_path, device, conf, iou, imgsz, half, max_det, allowlist):
        """Background thread: load the detector, then go live. Node subscribes before this runs."""
        # Kept so a recovery can rebuild through THIS path rather than a second
        # copy of the construction that would drift from it.
        self._build_kwargs = dict(model_path=model_path, device=device,
                                  conf=conf, iou=iou, imgsz=imgsz, half=half,
                                  max_det=max_det, allowlist=allowlist)
        try:
            det = make_detector(
                model_path=model_path,
                device=device, conf=conf, iou=iou, imgsz=imgsz,
                half=half, max_det=max_det, class_allowlist=allowlist,
                logger=self.get_logger())
        except Exception as exc:
            # ⛔ D16's TWIN, AT INIT. Returning here left `self._det` None and
            # the infer loop then dropped every frame at `if det is None:
            # continue` -- silently, forever, while the node answered param
            # queries and logged "chip efficiency". Observed on the vehicle:
            # `Failed to open device file /dev/hailo0 with error 6` after a
            # restart race, and the stack looked healthy from every angle
            # except the detection rate.
            #
            # A node that has no model cannot detect anything, so it must not
            # keep claiming to be up. Exit and let a supervisor restart it --
            # which also resolves the common cause, a previous process still
            # holding the device.
            self.get_logger().fatal(
                f"[DET  ] detector init FAILED: {exc}. EXITING: without a model "
                f"this node would drop every frame silently while looking "
                f"healthy. A supervisor restart also clears a device still "
                f"held by a previous process.")
            os._exit(2)
        # Apply any allowlist change that arrived during load via a param callback.
        pending = self._pending_allowlist
        if pending is not allowlist:
            det.update_allowlist(pending)
        self._apply_assoc_conf(det)
        self._det = det  # atomic publish under CPython GIL — _infer_loop sees it next tick
        # Re-apply a per-model conf override that was set before the model landed
        # (startup or an early live set_conf): _apply_model_conf ran against a None
        # detector then, so replay it now the single model exists.
        if getattr(self, '_pending_model_conf', ''):
            self._apply_model_conf(self._pending_model_conf)
        self.get_logger().info("[DET  ] model ready — inference active")

    def _publish_vision_info(self) -> None:
        """Say which model is live, and bump the epoch so a switch is visible.

        `database_location` is the model STEM, which is this stack's model
        identity everywhere else (`set_model`, `ClassRef`, the `.engine`
        sidecar) -- a second naming scheme here would be a third answer to
        "which model is that".
        """
        from vision_msgs.msg import VisionInfo
        m = VisionInfo()
        m.header.stamp = self.get_clock().now().to_msg()
        m.method = 'yolo'
        # Every model that is actually running, primary first. A consumer
        # attributing a detection to "the model" when two are live would
        # attribute half of them to the wrong one, so the field names both --
        # the same comma form `active_model` accepts.
        names = [str(self._active_name or self._single_model_name or '')]
        names += list(getattr(self, '_extra_names', []) or [])
        m.database_location = ','.join(n for n in names if n)
        self._model_epoch += 1
        m.database_version = int(self._model_epoch)
        self._pub_vinfo.publish(m)

    def _publish_classes(self, classes_str: str) -> None:
        """Publish the current classes filter so display_node can light up active classes."""
        from std_msgs.msg import String
        msg = String()
        msg.data = classes_str
        self._pub_classes.publish(msg)

    def _on_image(self, msg: Image):
        # Single-slot: drop stale frame, enqueue latest only.
        self._offer(msg)

    # ------------------------------------------------------------------ #
    #  Direct in-process feed (composed process)                         #
    # ------------------------------------------------------------------ #
    def wants_frame(self) -> bool:
        """True when the worker is idle and would consume a frame NOW.

        A composed camera gates its DECODE on this. `paused` is included
        because a paused detector consumes frames only to discard them, and on
        the vehicle the unused camera is paused for most of a mission -- so
        this is also what stops it decoding 15 fps of pictures nobody reads.
        """
        return (self._want.is_set()
                and not self.get_parameter('paused').value)

    def _check_direct_feed(self) -> None:
        """Subscribe after all, if nothing was handed to us by reference.

        A detector with `direct_feed` on and no composed camera receives
        NOTHING, for ever, with no error -- the failure mode this whole round
        has been about. Two seconds of silence is enough to be sure, and
        cheap enough that a slow-starting camera does not trip it.
        """
        self._fallback_timer.cancel()
        if self._fed_direct or self._sub is not None:
            return
        self.get_logger().warn(
            f'[DET  ] direct_feed is ON but no frame arrived by reference in '
            f'{_DIRECT_FALLBACK_S:.0f}s -- there is no camera in this '
            f'process. Subscribing to {self._ns_in} instead. This works, but '
            f'it pays the encode+serialise+transport this setting exists to '
            f'skip; run the composed launcher, or set direct_feed:=false to '
            f'silence this.')
        self._direct = False
        self._sub = self.create_subscription(
            Image, self._ns_in, self._on_image, qos.IMAGE)

    def submit_frame(self, frame_bgr, header) -> None:
        """Hand over an ALREADY-DECODED frame plus the header that describes it.

        The header must be the one built from the frame's own capture time.
        Passing the frame without it would leave `detections` stamped with
        whatever the detector felt like, which is the defect the round-30 and
        round-32 stamp fixes removed at the two layers either side of this one.
        """
        self._fed_direct = True
        self._offer(_DirectFrame(frame_bgr, header))

    def _offer(self, item) -> None:
        while not self._infer_q.empty():
            try:
                self._infer_q.get_nowait()
                # Something was waiting and is now discarded. Under the
                # composed design this should be ~0: the camera only decodes
                # when the worker is idle, so nothing should ever queue behind
                # an unconsumed frame. A rising count means we are decoding
                # frames the chip never looks at.
                self._evicted += 1
            except _queue.Empty:
                break
        self._infer_q.put_nowait(item)

    def _on_infer_failure(self, exc) -> None:
        """A node that cannot do its job must stop claiming to be up.

        ⛔ WHAT THIS REPLACES (BUGS.md D16). The old handler logged and
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
                self.get_logger().warn('[DET  ] detector REBUILT after '
                                       'consecutive inference failures')
                self._infer_fails = 0
        except Exception as exc:                # noqa: BLE001
            self.get_logger().error(f'[DET  ] rebuild failed: {exc!r}')

    def _infer_loop(self):
        """Worker thread: decode + infer + publish (never touches the ROS executor)."""
        while rclpy.ok():
            try:
                self._want.set()
                item = self._infer_q.get(timeout=0.5)
            except _queue.Empty:
                continue
            finally:
                self._want.clear()

            if self.get_parameter('paused').value:
                continue  # frame consumed from queue; skip decode + infer

            if isinstance(item, _DirectFrame):
                # Already decoded, in this process, by the camera that captured
                # it. No serialise, no transport, no second copy.
                frame, header = item.frame, item.header
            else:
                header = item.header
                try:
                    frame = self._bridge.imgmsg_to_cv2(
                        item, desired_encoding='bgr8')
                except Exception as exc:
                    self.get_logger().warning(
                        f"[DET  ] cv_bridge decode failed: {exc!r}")
                    continue

            t0 = time.monotonic()
            # Age of this frame at the instant the chip starts on it. The
            # floor is capture->available plus the decode; anything beyond
            # that is the chip being fed something it should not be.
            try:
                cap = header.stamp.sec + header.stamp.nanosec * 1e-9
                age = time.time() - cap
                # Gated on HAVING a detector: this counter is reported as
                # "inferences", and with `_det` None the loop drops the frame a
                # few lines below. It reported "300 inferences" during a run in
                # which zero inferences happened, which is worse than no
                # counter -- it was the reason a dead detector looked busy.
                if 0.0 <= age < 5.0 and self._det is not None:
                    self._infers += 1
                    self._stale_sum += age
                    self._stale_max = max(self._stale_max, age)
            except AttributeError:
                pass
            if self._pre is not None:
                try:
                    frame = self._pre(frame)
                except Exception as exc:
                    # Never let a preprocessing fault stop detection: a
                    # degraded frame beats no frame.
                    self.get_logger().warning(
                        f'[DET  ] preprocess failed, using the raw frame: '
                        f'{exc!r}')
                    self._pre = None
            det = self._det  # atomic ref read under CPython GIL
            if det is None:
                continue  # model still loading — drop frame, keep queue drained
            # RANGE CROP, decided from the PREVIOUS frame's target size --
            # the only size available before inferring this one.
            crop_state = None
            infer_frame = frame
            if self._crop is not None:
                infer_frame, crop_state = self._crop.apply(frame)
            try:
                detections = det.infer(infer_frame)
                self._infer_fails = 0
            except Exception as exc:
                self._on_infer_failure(exc)
                continue
            # ⛔ AFTER the counter is cleared, and outside this try. The
            # failure counter escalates to REBUILDING THE PRIMARY DETECTOR,
            # and the primary just succeeded -- letting a secondary model's
            # exception land in the same counter would tear down a healthy
            # model because a different one misbehaved. Caught by
            # `test_a_SUCCESSFUL_inference_in_the_LOOP_resets_the_counter`,
            # which went from 0 failures to 5.
            detections = self._merge_extra(infer_frame, detections)

            if crop_state is not None and crop_state.active and detections:
                # BACK TO FULL-FRAME COORDINATES. Everything downstream -- the
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
            except Exception:
                return  # node being destroyed; exit thread cleanly

            if self._pub_contours is not None and detections:
                # Built from the SAME list on the SAME header, so a consumer
                # can pair them by stamp and index without a second lookup.
                try:
                    from duburi_vision.detection.messages import (
                        detections_to_contours)
                    h, w = frame.shape[:2]
                    self._pub_contours.publish(detections_to_contours(
                        detections, header, camera=self._cam_name,
                        width=w, height=h))
                except Exception as exc:                    # noqa: BLE001
                    # An outline is evidence, not control. Losing it must never
                    # cost the detection that is steering the vehicle.
                    self.get_logger().warning(
                        f'[DET  ] contour publish failed: {exc!r}', once=True)

            # Skip the overlay render + encode + publish entirely when no one is
            # subscribed to image_debug (autonomous runs with viewer:=false). This
            # is pure overhead on the inference thread otherwise -- a free win that
            # grows once TensorRT speeds inference up.
            if (self._publish_dbg
                    and self._pub_dbg.get_subscription_count() > 0
                    and (time.monotonic() - self._last_dbg) >= self._dbg_min_dt):
                try:
                    fps = 1.0 / dt if dt > 1e-6 else 0.0
                    overlay = draw.render_all(
                        frame, detections,
                        source=self._cam_name, fps=fps,
                        healthy=True, deadband=self._deadband, primary=primary)
                    dbg = self._bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
                    dbg.header = header
                    self._pub_dbg.publish(dbg)
                    self._last_dbg = time.monotonic()
                except Exception as exc:
                    if rclpy.ok():
                        self.get_logger().warning(f"[DET  ] debug image failed: {exc!r}")

    def _apply_model_conf(self, value: str) -> None:
        """Apply per-model conf overrides (CSV 'name=conf') to the registry.

        Each named entry gets its own threshold, persisting across active_model
        switches (the override lives on the detector). In single-model mode a
        pair naming the loaded model (or its stem) applies to it. Unknown names
        are warned, not fatal -- a live-tuned param must never crash the node.
        """
        overrides = _parse_model_conf(value)
        if not overrides:
            return
        for name, conf in overrides.items():
            det = None
            if self._registry:
                # Resolve by registry KEY or model STEM, mirroring the
                # active_model handler -- an aliased registry (models:=gate=stem)
                # must accept set_conf(model='<stem>') too, or the per-model conf
                # silently no-ops (the b155736 identity-bug class).
                key = self._resolve_model_key(name)
                det = self._registry[key] if key is not None else None
            elif self._det is not None and (
                    name == '' or _model_stem(name) == self._single_model_name):
                # Single-model launch: _active_name stays None, so match on the
                # loaded model's STEM instead (else a valid set_conf(model='<stem>')
                # silently drops -- the exact torpedo "run this model tight" case).
                det = self._det
            if det is None:
                self.get_logger().warning(
                    f"[DET  ] model_conf: {name!r} not in "
                    f"{sorted(self._registry) if self._registry else 'single-model'}")
                continue
            det.update_conf(conf)
            self.get_logger().info(f"[DET  ] model_conf {name!r} → {conf:.3f}")

    def _apply_assoc_conf(self, det) -> None:
        """Push the BYTE publish floor onto a freshly built detector.

        Called at EVERY construction site (registry, single, async) rather than
        once at startup: a model loaded later would otherwise silently keep the
        default floor, which is the shape of bug this package has shipped three
        times (device_path into **_, the unloaded YAML table, ros2 param set on
        a construction-time param).
        """
        a = float(self.get_parameter('assoc_conf').value)
        if a > 0.0 and hasattr(det, 'update_assoc_conf'):
            det.update_assoc_conf(a)

    def _resolve_model_key(self, name: str) -> Optional[str]:
        """Map a set_model()/active_model argument to a registry key.

        Accepts EITHER the launch registry key (``duburi.use('gate')`` with
        ``models:=gate=...``) OR the model stem (``set_model('gate_rescue_repair')``
        / a ClassRef). Returns the key, or ``None`` if neither matches. Registry
        mode only (single-model handled separately).
        """
        if name in self._registry:
            return name
        return self._stem_to_key.get(_model_stem(name))

    def _on_parameter_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'classes':
                classes_str = str(p.value).strip()
                new_allow   = (
                    None if not classes_str
                    else [c.strip() for c in classes_str.split(',') if c.strip()]
                )
                self._pending_allowlist = new_allow  # persists for async load
                if self._det is not None:
                    self._det.update_allowlist(new_allow)
                self._publish_classes(classes_str)
                self.get_logger().info(
                    f"[DET  ] classes → {new_allow or '*all*'}")

            elif p.name == 'active_model':
                name = str(p.value).strip()
                if not self._registry:
                    # Single-model launch: set_model to the loaded model is a no-op
                    # SUCCESS (so ClassRef/use('<stem>') missions work); any other
                    # name is a clear, actionable reject (no 'registry' word, so the
                    # DSL surfaces THIS message instead of a generic wrap).
                    if (name and self._single_model_name
                            and _model_stem(name) == self._single_model_name):
                        self.get_logger().info(
                            f"[DET  ] active_model {name!r} already loaded "
                            f"(single-model launch) — no-op")
                        continue
                    return SetParametersResult(
                        successful=False,
                        reason=(f"single-model launch loaded "
                                f"{self._single_model_name!r}; cannot switch to "
                                f"{name!r} live -- relaunch with model:={name} (or "
                                f"models:=... for hot switching)"))
                name, extra_names = _split_active(name)
                key = self._resolve_model_key(name)
                if key is None:
                    return SetParametersResult(
                        successful=False,
                        reason=(f"active_model={name!r} not found -- keys="
                                f"{sorted(self._registry)} stems="
                                f"{sorted(self._stem_to_key)}"))
                self._det = self._registry[key]
                self._active_name = key
                self._set_extra(extra_names)
                self._publish_vision_info()
                self.get_logger().info(
                    f"[DET  ] active_model → {key!r}"
                    + (f" (via stem {name!r})" if key != name else "")
                    + (f" + {self._extra_names}" if self._extra_names else ""))

            elif p.name == 'paused':
                state = 'paused' if p.value else 'resumed'
                self.get_logger().info(f"[DET  ] {state}")

            elif p.name == 'conf':
                new_conf = float(p.value)
                # Apply across the whole registry (mirror max_det) so a live
                # set_conf survives a later set_model switch -- otherwise it only
                # touched the active model and reverted to the launch conf on the
                # next switch. Single-model mode just updates self._det.
                for det in (self._registry.values() if self._registry
                            else ([self._det] if self._det is not None else [])):
                    det.update_conf(new_conf)
                self.get_logger().info(f"[DET  ] conf → {new_conf:.3f}")

            elif p.name == 'assoc_conf':
                a = float(p.value)
                for det in (self._registry.values() if self._registry
                            else ([self._det] if self._det is not None else [])):
                    if hasattr(det, 'update_assoc_conf'):
                        det.update_assoc_conf(a if a > 0.0 else 1.0)
                self.get_logger().info(
                    f"[DET  ] assoc_conf → {a:.3f}"
                    + ("  (OFF)" if a <= 0.0 else
                       "  -- boxes below the control floor now reach the tracker "
                       "for ASSOCIATION only"))

            elif p.name == 'model_conf':
                # Per-model override (CSV 'name=conf'); targets individual
                # registry entries and persists across active_model switches.
                self._pending_model_conf = str(p.value)
                self._apply_model_conf(self._pending_model_conf)

            elif p.name == 'max_det':
                new_max = int(p.value)
                # Apply across the whole registry so a model switch keeps the cap.
                for det in (self._registry.values() if self._registry
                            else ([self._det] if self._det is not None else [])):
                    det.update_max_det(new_max)
                self.get_logger().info(f"[DET  ] max_det → {new_max}")

        return SetParametersResult(successful=True)

    def _log_alignment(self, primary, frame) -> None:
        """Always-on operator BEARING line for the currently-loaded class.

        Fires whenever the detector sees its loaded class (``detections`` are
        already class-filtered, so ``primary`` is that class) -- independent of
        whether a vision verb is running. Reports the RAW bbox-centre pixel
        offset from frame centre on lateral (x) and depth (y). This is live
        telemetry, NOT an alignment verdict: it is deliberately worded distinct
        from a vision verb's ``aligned (N/Mpx)`` outcome so the operator never
        reads this continuous offset as "the verb aligned N px off-target".
        Throttled so it doesn't flood at detection rate.
        """
        now = time.monotonic()
        if (now - self._last_align_log) < ALIGN_LOG_THROTTLE_S:
            return
        self._last_align_log = now
        h, w   = frame.shape[:2]
        cx, cy = primary.cx, primary.cy
        x_off  = cx - w * 0.5    # +right of centre
        y_off  = cy - h * 0.5    # +below centre (drives depth)
        self.get_logger().info(
            f"[ offset lat={x_off:+.0f} depth={y_off:+.0f}px ] "
            f"'{primary.class_name}' bearing (live, off-centre)")

    def _log_efficiency(self):
        """Is the chip only ever looking at the freshest frame?

        `stale` is the frame's age when inference STARTS. Its floor is
        capture->available plus the decode -- about 6 ms on this hardware --
        and it is the number that grows first if anything upstream starts
        queueing.

        `wasted` counts frames decoded and then discarded before the chip saw
        them. Under the composed design it should be ZERO: the camera decodes
        only when the inference worker is idle, so nothing can queue behind an
        unconsumed frame. A non-zero value is ~1.9 ms of a core thrown away
        per frame, which on a 27 TOPS budget is exactly what we are trying not
        to do.
        """
        n = self._infers
        if not n:
            return
        mean_ms = 1000.0 * self._stale_sum / n
        waste = self._evicted
        line = (f'[DET  ] chip efficiency: {n} inferences, frame age at '
                f'infer-start {mean_ms:5.1f} ms mean / '
                f'{1000.0 * self._stale_max:5.1f} max, '
                f'{waste} decoded-but-never-inferred')
        if waste:
            self.get_logger().warn(
                line + ' <- WASTED DECODES: frames are queueing behind the '
                       'worker, which should be impossible on the direct feed')
        else:
            self.get_logger().info(line)
        self._infers = 0
        self._evicted = 0
        self._stale_sum = 0.0
        self._stale_max = 0.0

    def _log_health(self):
        now = time.monotonic()
        elapsed = max(now - self._last_log, 1e-3)
        in_hz   = self._frames / elapsed
        avg_ms  = (self._infer_total_s / max(self._frames, 1)) * 1000.0
        target_pct = 100.0 * self._with_target / max(self._frames, 1)
        model_tag = f'  model={self._active_name!r}' if self._active_name else ''
        self.get_logger().debug(
            f"[DET  ] in_hz={in_hz:5.1f}  avg_infer={avg_ms:5.1f}ms  "
            f"with_target={target_pct:4.0f}%  total={self._frames}{model_tag}")
        self._frames = 0
        self._with_target = 0
        self._infer_total_s = 0.0
        self._last_log = now


def main():
    rclpy.init()
    node = DetectorNode()
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

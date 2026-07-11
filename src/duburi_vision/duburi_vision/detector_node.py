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
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Dict, Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge        import CvBridge

from duburi_vision import draw
from duburi_vision.detection.detector  import largest
from duburi_vision.detection.yolo      import YoloDetector
from duburi_vision.detection.messages  import detections_to_array

# Throttle for the always-on operator alignment line (seconds). Matches the
# control-side vision throttle so the rate feels consistent between the detector
# status line and the manager's mission logs.
ALIGN_LOG_THROTTLE_S = 0.5


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


class DetectorNode(Node):
    def __init__(self):
        super().__init__('duburi_detector')

        self.declare_parameter('camera',              'laptop')
        self.declare_parameter('image_topic',         '')
        self.declare_parameter('model_path',          'yolov11n')
        self.declare_parameter('models',              '')     # CSV name=stem pairs
        self.declare_parameter('active_model',        '')     # registry key to start with
        self.declare_parameter('device',              'cuda:0')
        self.declare_parameter('half',                True)   # fp16: ~half the VRAM; coerced off on non-CUDA (yolo.py)
        self.declare_parameter('conf',                0.35)
        # Per-model confidence overrides: CSV 'name=conf' (e.g.
        # 'torpedo_blood_hole=0.55,gate_rescue_repair=0.35'). Applies on top of
        # the uniform `conf` above, targeting individual registry entries, and
        # PERSISTS across active_model switches (each YoloDetector holds its own
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
        conf     = float(self.get_parameter('conf').value)
        iou      = float(self.get_parameter('iou').value)
        imgsz    = int(self.get_parameter('imgsz').value)
        max_det  = int(self.get_parameter('max_det').value)
        half     = bool(self.get_parameter('half').value)

        # ── Registry (multi-model) ─────────────────────────────────────
        models_str   = str(self.get_parameter('models').value).strip()
        active_model = str(self.get_parameter('active_model').value).strip()
        self._registry: Dict[str, YoloDetector] = {}
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
                det = YoloDetector(
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
                self._det: YoloDetector = self._registry[active_key]
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
            self._det: Optional[YoloDetector] = None
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

        from vision_msgs.msg import Detection2DArray
        from std_msgs.msg import String
        self._bridge       = CvBridge()
        self._sub          = self.create_subscription(Image, ns_in, self._on_image, 5)
        self._pub_det      = self.create_publisher(Detection2DArray, f'{ns_out}/detections', 10)
        self._pub_classes  = self.create_publisher(String, f'{ns_out}/classes_filter', 10)
        self._publish_dbg = bool(self.get_parameter('publish_debug_image').value)
        if self._publish_dbg:
            self._pub_dbg = self.create_publisher(Image, f'{ns_out}/image_debug', 5)
            dbg_hz = max(float(self.get_parameter('debug_image_hz').value), 0.5)
            self._dbg_min_dt = 1.0 / dbg_hz
            self._last_dbg = 0.0

        self._frames        = 0
        self._with_target   = 0
        self._infer_total_s = 0.0
        self._last_log      = time.monotonic()
        self._last_align_log = 0.0
        self.create_timer(2.0, self._log_health)

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

    def _load_single_model_async(self, *, model_path, device, conf, iou, imgsz, half, max_det, allowlist):
        """Background thread: load YoloDetector, then go live. Node subscribes before this runs."""
        try:
            det = YoloDetector(
                model_path=model_path,
                device=device, conf=conf, iou=iou, imgsz=imgsz,
                half=half, max_det=max_det, class_allowlist=allowlist,
                logger=self.get_logger())
        except Exception as exc:
            self.get_logger().fatal(f"[DET  ] YoloDetector init FAILED: {exc}")
            return
        # Apply any allowlist change that arrived during load via a param callback.
        pending = self._pending_allowlist
        if pending is not allowlist:
            det.update_allowlist(pending)
        self._det = det  # atomic publish under CPython GIL — _infer_loop sees it next tick
        # Re-apply a per-model conf override that was set before the model landed
        # (startup or an early live set_conf): _apply_model_conf ran against a None
        # detector then, so replay it now the single model exists.
        if getattr(self, '_pending_model_conf', ''):
            self._apply_model_conf(self._pending_model_conf)
        self.get_logger().info("[DET  ] model ready — inference active")

    def _publish_classes(self, classes_str: str) -> None:
        """Publish the current classes filter so display_node can light up active classes."""
        from std_msgs.msg import String
        msg = String()
        msg.data = classes_str
        self._pub_classes.publish(msg)

    def _on_image(self, msg: Image):
        # Single-slot: drop stale frame, enqueue latest only.
        while not self._infer_q.empty():
            try:
                self._infer_q.get_nowait()
            except _queue.Empty:
                break
        self._infer_q.put_nowait(msg)

    def _infer_loop(self):
        """Worker thread: decode + infer + publish (never touches the ROS executor)."""
        while rclpy.ok():
            try:
                msg = self._infer_q.get(timeout=0.5)
            except _queue.Empty:
                continue

            if self.get_parameter('paused').value:
                continue  # frame consumed from queue; skip decode + infer

            try:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as exc:
                self.get_logger().warning(f"[DET  ] cv_bridge decode failed: {exc!r}")
                continue

            t0 = time.monotonic()
            det = self._det  # atomic ref read under CPython GIL
            if det is None:
                continue  # model still loading — drop frame, keep queue drained
            try:
                detections = det.infer(frame)
            except Exception as exc:
                self.get_logger().error(f"[DET  ] inference failed: {exc!r}")
                continue
            dt = time.monotonic() - t0

            self._frames        += 1
            self._infer_total_s += dt
            primary = largest(detections)
            if primary is not None:
                self._with_target += 1
                self._log_alignment(primary, frame)

            det_msg = detections_to_array(detections, msg.header)
            if not rclpy.ok():
                return
            try:
                self._pub_det.publish(det_msg)
            except Exception:
                return  # node being destroyed; exit thread cleanly

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
                    dbg.header = msg.header
                    self._pub_dbg.publish(dbg)
                    self._last_dbg = time.monotonic()
                except Exception as exc:
                    if rclpy.ok():
                        self.get_logger().warning(f"[DET  ] debug image failed: {exc!r}")

    def _apply_model_conf(self, value: str) -> None:
        """Apply per-model conf overrides (CSV 'name=conf') to the registry.

        Each named entry gets its own threshold, persisting across active_model
        switches (the override lives on the YoloDetector). In single-model mode a
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
                key = self._resolve_model_key(name)
                if key is None:
                    return SetParametersResult(
                        successful=False,
                        reason=(f"active_model={name!r} not found -- keys="
                                f"{sorted(self._registry)} stems="
                                f"{sorted(self._stem_to_key)}"))
                self._det = self._registry[key]
                self._active_name = key
                self.get_logger().info(
                    f"[DET  ] active_model → {key!r}"
                    + (f" (via stem {name!r})" if key != name else ""))

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

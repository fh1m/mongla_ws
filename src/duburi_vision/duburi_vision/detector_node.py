#!/usr/bin/env python3
"""detector_node -- subscribe to image_raw, run YOLO26, publish detections.

Topics:
  in    /duburi/vision/<cam>/image_raw     sensor_msgs/Image
  out   /duburi/vision/<cam>/detections    vision_msgs/Detection2DArray
  out   /duburi/vision/<cam>/image_debug   sensor_msgs/Image  (rate-limited overlay)

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

import sys
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge        import CvBridge

from duburi_vision import draw
from duburi_vision.detection.detector  import largest
from duburi_vision.detection.yolo      import YoloDetector
from duburi_vision.detection.messages  import detections_to_array


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


class DetectorNode(Node):
    def __init__(self):
        super().__init__('duburi_detector')

        self.declare_parameter('camera',              'laptop')
        self.declare_parameter('image_topic',         '')
        self.declare_parameter('model_path',          'yolo26n.pt')
        self.declare_parameter('models',              '')     # CSV name=stem pairs
        self.declare_parameter('active_model',        '')     # registry key to start with
        self.declare_parameter('device',              'cuda:0')
        self.declare_parameter('half',                False)
        self.declare_parameter('conf',                0.35)
        self.declare_parameter('iou',                 0.5)
        self.declare_parameter('imgsz',               640)
        self.declare_parameter('classes',             'person')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_image_hz',      5.0)
        self.declare_parameter('alignment_deadband',  0.05)

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
        half     = bool(self.get_parameter('half').value)

        # ── Registry (multi-model) ─────────────────────────────────────
        models_str   = str(self.get_parameter('models').value).strip()
        active_model = str(self.get_parameter('active_model').value).strip()
        self._registry: Dict[str, YoloDetector] = {}

        if models_str:
            model_map = _parse_models_param(models_str)
            self.get_logger().info(
                f"[DET  ] registry mode: loading {len(model_map)} model(s): "
                f"{list(model_map)}")
            for name, stem in model_map.items():
                try:
                    det = YoloDetector(
                        model_path=stem,
                        device=device, conf=conf, iou=iou, imgsz=imgsz,
                        half=half, class_allowlist=allowlist,
                        logger=self.get_logger())
                    self._registry[name] = det
                    self.get_logger().info(f"[DET  ] registry[{name!r}] = {stem!r}  ready")
                except Exception as exc:
                    self.get_logger().fatal(
                        f"[DET  ] registry[{name!r}]: load of {stem!r} FAILED: {exc}")
                    raise

            if active_model and active_model in self._registry:
                self._det: YoloDetector = self._registry[active_model]
                self._active_name: Optional[str] = active_model
            elif self._registry:
                first = next(iter(self._registry))
                self._det = self._registry[first]
                self._active_name = first
                if active_model:
                    self.get_logger().warning(
                        f"[DET  ] active_model={active_model!r} not in registry "
                        f"{list(self._registry)}; using first: {first!r}")
            else:
                self.get_logger().fatal("[DET  ] models param parsed but registry is empty")
                raise RuntimeError("empty detector registry")

        else:
            # Single-model mode (original behaviour)
            self._active_name = None
            try:
                self._det = YoloDetector(
                    model_path=str(self.get_parameter('model_path').value),
                    device=device, conf=conf, iou=iou, imgsz=imgsz,
                    half=half, class_allowlist=allowlist,
                    logger=self.get_logger())
            except Exception as exc:
                self.get_logger().fatal(f"[DET  ] YoloDetector init FAILED: {exc}")
                raise

        from vision_msgs.msg import Detection2DArray
        self._bridge   = CvBridge()
        self._sub      = self.create_subscription(Image, ns_in, self._on_image, 5)
        self._pub_det  = self.create_publisher(Detection2DArray, f'{ns_out}/detections',  10)
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
        self.create_timer(2.0, self._log_health)

        self._device_str = device
        self._deadband   = float(self.get_parameter('alignment_deadband').value)

        self.add_on_set_parameters_callback(self._on_parameter_change)

        registry_info = (
            f"  registry={list(self._registry)}  active={self._active_name!r}"
            if self._registry else ''
        )
        self.get_logger().info(
            f"[DET  ] subscribed {ns_in!r} -> {ns_out}/detections  "
            f"({'+ image_debug' if self._publish_dbg else 'no debug image'})"
            f"{registry_info}")

    def _on_image(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f"[DET  ] cv_bridge decode failed: {exc!r}")
            return

        t0 = time.monotonic()
        # Read self._det once — safe atomic ref under CPython GIL.
        det = self._det
        try:
            detections = det.infer(frame)
        except Exception as exc:
            self.get_logger().error(f"[DET  ] inference failed: {exc!r}")
            return
        dt = time.monotonic() - t0

        self._frames        += 1
        self._infer_total_s += dt
        primary = largest(detections)
        if primary is not None:
            self._with_target += 1

        det_msg = detections_to_array(detections, msg.header)
        self._pub_det.publish(det_msg)

        if self._publish_dbg and (time.monotonic() - self._last_dbg) >= self._dbg_min_dt:
            fps = 1.0 / dt if dt > 1e-6 else 0.0
            overlay = draw.render_all(
                frame, detections,
                source=self._cam_name, fps=fps, device=self._device_str,
                healthy=True, deadband=self._deadband, primary=primary)
            try:
                dbg = self._bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
                dbg.header = msg.header
                self._pub_dbg.publish(dbg)
                self._last_dbg = time.monotonic()
            except Exception as exc:
                self.get_logger().warning(f"[DET  ] debug image encode failed: {exc!r}")

    def _on_parameter_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'classes':
                classes_str = str(p.value).strip()
                new_allow   = (
                    None if not classes_str
                    else [c.strip() for c in classes_str.split(',') if c.strip()]
                )
                self._det.update_allowlist(new_allow)
                self.get_logger().info(
                    f"[DET  ] classes → {new_allow or '*all*'}")

            elif p.name == 'active_model':
                name = str(p.value).strip()
                if not self._registry:
                    return SetParametersResult(
                        successful=False,
                        reason="active_model: no registry loaded (use 'models' param at startup)")
                if name not in self._registry:
                    return SetParametersResult(
                        successful=False,
                        reason=(f"active_model={name!r} not in registry; "
                                f"available: {sorted(self._registry)}"))
                self._det = self._registry[name]
                self._active_name = name
                self.get_logger().info(f"[DET  ] active_model → {name!r}")

        return SetParametersResult(successful=True)

    def _log_health(self):
        now = time.monotonic()
        elapsed = max(now - self._last_log, 1e-3)
        in_hz   = self._frames / elapsed
        avg_ms  = (self._infer_total_s / max(self._frames, 1)) * 1000.0
        target_pct = 100.0 * self._with_target / max(self._frames, 1)
        model_tag = f'  model={self._active_name!r}' if self._active_name else ''
        self.get_logger().info(
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

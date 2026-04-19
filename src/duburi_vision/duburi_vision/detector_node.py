#!/usr/bin/env python3
"""detector_node -- subscribe to image_raw, run YOLO26, publish detections.

Topics:
  in    /duburi/vision/<cam>/image_raw     sensor_msgs/Image
  out   /duburi/vision/<cam>/detections    vision_msgs/Detection2DArray
  out   /duburi/vision/<cam>/image_debug   sensor_msgs/Image  (rate-limited overlay)

Examples
--------
# Default: subscribe to laptop camera, run YOLO26 nano, person allowlist
ros2 run duburi_vision detector_node --ros-args -p camera:=laptop

# Force CPU (no CUDA available)
ros2 run duburi_vision detector_node --ros-args -p camera:=laptop -p device:=cpu

# Listen for any class, lower threshold
ros2 run duburi_vision detector_node --ros-args \\
    -p camera:=laptop -p classes:='' -p conf:=0.25
"""

import os
os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}')

import sys
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge        import CvBridge

from duburi_vision import draw
from duburi_vision.detection.detector  import largest
from duburi_vision.detection.yolo      import YoloDetector
from duburi_vision.detection.messages  import detections_to_array


class DetectorNode(Node):
    def __init__(self):
        super().__init__('duburi_detector')

        self.declare_parameter('camera',              'laptop')
        self.declare_parameter('image_topic',         '')           # explicit override of the ns
        self.declare_parameter('model_path',          'yolo26n.pt')  # ultralytics auto-downloads
        self.declare_parameter('device',              'cuda:0')
        self.declare_parameter('half',                False)
        self.declare_parameter('conf',                0.35)
        self.declare_parameter('iou',                 0.5)
        self.declare_parameter('imgsz',               640)
        self.declare_parameter('classes',             'person')      # CSV; '' = keep all
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

        try:
            self._det = YoloDetector(
                model_path=str(self.get_parameter('model_path').value),
                device=str(self.get_parameter('device').value),
                conf=float(self.get_parameter('conf').value),
                iou=float(self.get_parameter('iou').value),
                imgsz=int(self.get_parameter('imgsz').value),
                half=bool(self.get_parameter('half').value),
                class_allowlist=allowlist,
                logger=self.get_logger(),
            )
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

        # Diagnostics
        self._frames        = 0
        self._with_target   = 0
        self._infer_total_s = 0.0
        self._last_log      = time.monotonic()
        self.create_timer(2.0, self._log_health)

        self._device_str = str(self.get_parameter('device').value)
        self._deadband   = float(self.get_parameter('alignment_deadband').value)

        self.get_logger().info(
            f"[DET  ] subscribed {ns_in!r} -> {ns_out}/detections  "
            f"({'+ image_debug' if self._publish_dbg else 'no debug image'})")

    def _on_image(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f"[DET  ] cv_bridge decode failed: {exc!r}")
            return

        t0 = time.monotonic()
        try:
            detections = self._det.infer(frame)
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

    def _log_health(self):
        now = time.monotonic()
        elapsed = max(now - self._last_log, 1e-3)
        in_hz   = self._frames / elapsed
        avg_ms  = (self._infer_total_s / max(self._frames, 1)) * 1000.0
        target_pct = 100.0 * self._with_target / max(self._frames, 1)
        self.get_logger().info(
            f"[DET  ] in_hz={in_hz:5.1f}  avg_infer={avg_ms:5.1f}ms  "
            f"with_target={target_pct:4.0f}%  total={self._frames}")
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

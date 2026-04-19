#!/usr/bin/env python3
"""vision_node -- single-process diagnostic for camera + detector.

The vision cousin of `duburi_sensors.sensors_node`: one node, no inter-node
topic hop, no ROS publishing required (still publishes image_debug if
asked). Use this to smoke-test that:

  * the chosen camera source produces frames
  * the YOLO model loads, picks GPU, and returns detections
  * the visualization layer renders correctly end-to-end

Examples
--------
# Laptop webcam, person allowlist, debug image published
ros2 run duburi_vision vision_node --ros-args -p profile:=laptop

# Headless smoke test, log only (no debug image)
ros2 run duburi_vision vision_node --ros-args -p profile:=laptop -p publish_debug_image:=false

# Force CPU
ros2 run duburi_vision vision_node --ros-args -p profile:=laptop -p device:=cpu
"""

import os
os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}')

import sys
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge        import CvBridge

from duburi_vision import (
    CAMERA_PROFILES,
    make_camera, make_camera_from_profile,
    get_profile, draw,
)
from duburi_vision.detection.yolo     import YoloDetector
from duburi_vision.detection.detector import largest


class VisionNode(Node):
    def __init__(self):
        super().__init__('duburi_vision')

        self.declare_parameter('profile',             'laptop')
        self.declare_parameter('source',              '')        # explicit override
        self.declare_parameter('topic',               '')        # for source=ros_topic
        self.declare_parameter('device',              'cuda:0')
        self.declare_parameter('model_path',          'yolo26n.pt')
        self.declare_parameter('classes',             'person')
        self.declare_parameter('conf',                0.35)
        self.declare_parameter('iou',                 0.5)
        self.declare_parameter('imgsz',               640)
        self.declare_parameter('half',                False)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_image_hz',      10.0)
        self.declare_parameter('tick_hz',             30.0)
        self.declare_parameter('alignment_deadband',  0.05)

        self._cam     = self._build_camera()
        self._info    = self._cam.info()
        self._cam_name = str(self._info.get('name') or 'cam')

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
            self.get_logger().fatal(f"[VIS  ] YoloDetector init FAILED: {exc}")
            raise

        self._device_str = str(self.get_parameter('device').value)
        self._deadband   = float(self.get_parameter('alignment_deadband').value)

        self._publish_dbg = bool(self.get_parameter('publish_debug_image').value)
        self._bridge      = CvBridge()
        if self._publish_dbg:
            ns_out = f'/duburi/vision/{self._cam_name}'
            self._pub_dbg = self.create_publisher(Image, f'{ns_out}/image_debug', 5)
            dbg_hz = max(float(self.get_parameter('debug_image_hz').value), 0.5)
            self._dbg_min_dt = 1.0 / dbg_hz
            self._last_dbg = 0.0
        else:
            self._pub_dbg = None

        tick_hz = float(self.get_parameter('tick_hz').value)
        self.create_timer(1.0 / max(tick_hz, 1.0), self._tick)

        self._frames = 0
        self._with_target = 0
        self._infer_total_s = 0.0
        self._last_log = time.monotonic()
        self.create_timer(2.0, self._log_health)

        self.get_logger().info(
            f"[VIS  ] in-process camera+detector ready  cam={self._cam_name!r}  "
            f"device={self._device_str}  classes={allowlist or '*'}  "
            f"{'(image_debug published)' if self._publish_dbg else '(no debug image)'}")

    def _build_camera(self):
        source = str(self.get_parameter('source').value).strip()
        if source:
            kwargs = {'name': source, 'frame_id': source}
            if source == 'ros_topic':
                kwargs.update(node=self,
                              topic=str(self.get_parameter('topic').value).strip())
            return make_camera(source, logger=self.get_logger(), **kwargs)

        profile_name = str(self.get_parameter('profile').value).strip() or 'laptop'
        profile = get_profile(profile_name)
        profile.setdefault('name', profile_name)
        return make_camera_from_profile(
            profile, node=self, logger=self.get_logger())

    def _tick(self):
        frame, meta = self._cam.read()
        if frame is None or not meta.fresh:
            return

        t0 = time.monotonic()
        try:
            detections = self._det.infer(frame)
        except Exception as exc:
            self.get_logger().error(f"[VIS  ] inference failed: {exc!r}")
            return
        dt = time.monotonic() - t0
        fps = 1.0 / dt if dt > 1e-6 else 0.0

        self._frames        += 1
        self._infer_total_s += dt
        primary = largest(detections)
        if primary is not None:
            self._with_target += 1

        if self._publish_dbg and (time.monotonic() - self._last_dbg) >= self._dbg_min_dt:
            overlay = draw.render_all(
                frame, detections,
                source=self._cam_name, fps=fps, device=self._device_str,
                healthy=self._cam.is_healthy(),
                deadband=self._deadband, primary=primary)
            try:
                dbg = self._bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
                dbg.header.stamp    = self.get_clock().now().to_msg()
                dbg.header.frame_id = str(self._info.get('frame_id') or self._cam_name)
                self._pub_dbg.publish(dbg)
                self._last_dbg = time.monotonic()
            except Exception as exc:
                self.get_logger().warning(f"[VIS  ] debug image encode failed: {exc!r}")

    def _log_health(self):
        now = time.monotonic()
        elapsed = max(now - self._last_log, 1e-3)
        loop_hz = self._frames / elapsed
        avg_ms  = (self._infer_total_s / max(self._frames, 1)) * 1000.0
        target_pct = 100.0 * self._with_target / max(self._frames, 1)
        cam_ok = self._cam.is_healthy()
        marker = 'OK ' if cam_ok else 'BAD'
        self.get_logger().info(
            f"[VIS  ] {marker}  loop={loop_hz:5.1f}Hz  infer_avg={avg_ms:5.1f}ms  "
            f"with_target={target_pct:4.0f}%  total={self._frames}")
        self._frames = 0
        self._with_target = 0
        self._infer_total_s = 0.0
        self._last_log = now

    def shutdown(self):
        try:
            self._cam.close()
        except Exception:
            pass


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
    sys.exit(0)

#!/usr/bin/env python3
"""camera_node -- read from a Camera and publish standard ROS image topics.

One camera per node instance. Pick the source via the `profile` ROS param
(matched against `CAMERA_PROFILES`) OR by passing `source` + source-specific
overrides. The node publishes:

  /duburi/vision/<cam>/image_raw     sensor_msgs/Image      (bgr8)
  /duburi/vision/<cam>/camera_info   sensor_msgs/CameraInfo (size only; K/D
                                                              empty until we
                                                              ship a calib file)

Examples
--------
# Laptop webcam by named profile
ros2 run duburi_vision camera_node --ros-args -p profile:=laptop

# Same thing, fully explicit
ros2 run duburi_vision camera_node --ros-args \\
    -p source:=webcam -p device:=0 -p width:=640 -p height:=480 -p fps:=30 -p name:=laptop

# Subscribe to a Gazebo camera topic and re-publish under our namespace
ros2 run duburi_vision camera_node --ros-args \\
    -p source:=ros_topic -p topic:=/duburi/sim/front_camera/image_raw -p name:=sim_front
"""

import os
os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}')

import sys
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge        import CvBridge

from duburi_vision import (
    CAMERA_PROFILES,
    make_camera, make_camera_from_profile,
    get_profile,
)


class CameraNode(Node):
    def __init__(self):
        super().__init__('duburi_camera')

        self.declare_parameter('profile',         '')        # e.g. 'laptop' / 'sim_front'
        self.declare_parameter('source',          '')        # explicit override
        self.declare_parameter('name',            '')
        self.declare_parameter('topic',           '')        # for source=ros_topic
        self.declare_parameter('device',          0)         # for source=webcam (int OR str path)
        self.declare_parameter('width',           640)
        self.declare_parameter('height',          480)
        # fps / publish_rate_hz declared as int so launch ints pass through
        # cleanly; we cast to float at the timer site.
        self.declare_parameter('fps',             30)
        self.declare_parameter('frame_id',        '')
        self.declare_parameter('publish_rate_hz', 30)

        self._cam     = self._build_camera()
        self._info    = self._cam.info()
        self._cam_name = str(self._info.get('name') or 'cam')
        self._frame_id = str(self._info.get('frame_id') or self._cam_name)

        ns = f'/duburi/vision/{self._cam_name}'
        self._pub_img  = self.create_publisher(Image,      f'{ns}/image_raw',   10)
        self._pub_info = self.create_publisher(CameraInfo, f'{ns}/camera_info', 10)
        self._bridge   = CvBridge()

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)

        self._sent  = 0
        self._dropped = 0
        self._last_log = time.monotonic()
        self.create_timer(2.0, self._log_health)

        self.get_logger().info(
            f"[CAM ] {self._cam_name!r} ({self._info.get('source_kind')}) -> "
            f"{ns}/image_raw  @ {rate:.1f} Hz")

    def _build_camera(self):
        profile_name = str(self.get_parameter('profile').value).strip()
        if profile_name:
            profile = get_profile(profile_name)
            if not str(self.get_parameter('name').value).strip():
                profile.setdefault('name', profile_name)
            else:
                profile['name'] = str(self.get_parameter('name').value).strip()
            return make_camera_from_profile(
                profile, node=self, logger=self.get_logger())

        source = str(self.get_parameter('source').value).strip()
        if not source:
            raise ValueError(
                f"camera_node: must set either 'profile' (one of "
                f"{sorted(CAMERA_PROFILES)}) or 'source' (one of webcam/ros_topic/...)")

        kwargs = {
            'name':       str(self.get_parameter('name').value).strip() or source,
            'frame_id':   str(self.get_parameter('frame_id').value).strip()
                          or str(self.get_parameter('name').value).strip()
                          or source,
        }
        if source == 'webcam':
            dev = self.get_parameter('device').value
            try:
                dev = int(dev)
            except (TypeError, ValueError):
                pass
            kwargs.update(
                device=dev,
                width=int(self.get_parameter('width').value),
                height=int(self.get_parameter('height').value),
                fps=int(self.get_parameter('fps').value),
            )
        elif source == 'ros_topic':
            kwargs.update(
                node=self,
                topic=str(self.get_parameter('topic').value).strip(),
                expected_width=int(self.get_parameter('width').value),
                expected_height=int(self.get_parameter('height').value),
                expected_fps=float(self.get_parameter('fps').value),
            )

        return make_camera(source, logger=self.get_logger(), **kwargs)

    def _tick(self):
        frame, meta = self._cam.read()
        if frame is None or not meta.fresh:
            self._dropped += 1
            return

        try:
            img_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f"[CAM ] cv_bridge encode failed: {exc!r}")
            self._dropped += 1
            return

        stamp = self.get_clock().now().to_msg()
        img_msg.header.stamp    = stamp
        img_msg.header.frame_id = self._frame_id

        info = CameraInfo()
        info.header = img_msg.header
        info.width  = int(meta.width  or img_msg.width)
        info.height = int(meta.height or img_msg.height)

        self._pub_img.publish(img_msg)
        self._pub_info.publish(info)
        self._sent += 1

    def _log_health(self):
        now = time.monotonic()
        elapsed = max(now - self._last_log, 1e-3)
        hz = self._sent / elapsed
        healthy = self._cam.is_healthy()
        marker = 'OK ' if healthy else 'BAD'
        self.get_logger().info(
            f"[CAM ] {marker}  {self._cam_name}  pub={hz:5.1f}Hz  "
            f"sent={self._sent}  dropped={self._dropped}")
        self._sent = 0
        self._dropped = 0
        self._last_log = now

    def shutdown(self):
        try:
            self._cam.close()
        except Exception as exc:
            self.get_logger().debug(f"camera close ignored: {exc!r}")


def main():
    rclpy.init()
    node = CameraNode()
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

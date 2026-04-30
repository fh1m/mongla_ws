#!/usr/bin/env python3
"""vision_display -- lightweight OpenCV viewer for the perception pipeline.

Replaces rqt_image_view for debug sessions. Starts faster, shows a live
HUD with depth/yaw/mode/battery from /duburi/state, and requires no Qt.

Topics consumed
---------------
  /duburi/vision/<cam>/image_debug   sensor_msgs/Image   annotated detector output
  /duburi/state                      duburi_interfaces/msg/DuburiState

ROS2 parameters
---------------
  camera  (string, default 'forward')  camera namespace under /duburi/vision/
  topic   (string, default '')         override full image topic (skips camera param)

Usage
-----
  ros2 run duburi_vision vision_display
  ros2 run duburi_vision vision_display --ros-args -p camera:=forward
  ros2 run duburi_vision vision_display --ros-args -p topic:=/duburi/vision/forward/image_raw

Press Q or Ctrl-C to exit.
"""

from __future__ import annotations

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

from duburi_interfaces.msg import DuburiState

# HUD layout constants
_HUD_FONT      = cv2.FONT_HERSHEY_SIMPLEX
_HUD_SCALE     = 0.55
_HUD_THICKNESS = 1
_HUD_PAD       = 8    # pixels from edge
_HUD_LINE_H    = 22   # pixels per line
_HUD_BG_ALPHA  = 0.45 # background rectangle opacity

# Log a reminder every N seconds if no frames arrive yet
_WAIT_LOG_INTERVAL = 5.0


def _draw_hud(frame, state: DuburiState) -> None:
    """Overlay depth / yaw / mode / battery in the top-left corner."""
    lines = [
        f"depth : {state.depth_m:+.2f} m",
        f"yaw   : {state.yaw_deg:.1f} deg",
        f"mode  : {state.mode or '?'}",
        f"batt  : {state.battery_voltage:.1f} V",
        f"armed : {'YES' if state.armed else 'no'}",
    ]

    max_w = max(
        cv2.getTextSize(l, _HUD_FONT, _HUD_SCALE, _HUD_THICKNESS)[0][0]
        for l in lines
    )
    box_h = _HUD_LINE_H * len(lines) + _HUD_PAD
    box_w = max_w + _HUD_PAD * 2

    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (box_w, box_h), (20, 20, 20), -1)
    cv2.addWeighted(overlay, _HUD_BG_ALPHA, frame, 1 - _HUD_BG_ALPHA, 0, frame)

    for i, line in enumerate(lines):
        y = _HUD_PAD + (i + 1) * _HUD_LINE_H - 4
        cv2.putText(frame, line, (_HUD_PAD, y),
                    _HUD_FONT, _HUD_SCALE, (220, 220, 220), _HUD_THICKNESS, cv2.LINE_AA)


class VisionDisplayNode(Node):
    def __init__(self):
        super().__init__('vision_display')

        self.declare_parameter('camera', 'forward')
        self.declare_parameter('topic', '')

        camera = self.get_parameter('camera').get_parameter_value().string_value
        topic  = self.get_parameter('topic').get_parameter_value().string_value
        img_topic = topic or f'/duburi/vision/{camera}/image_debug'

        self.get_logger().info(f'[DISP ] subscribing to {img_topic}')
        self.get_logger().info('[DISP ] waiting for first frame — make sure camera_node + detector_node are running')

        self._bridge = CvBridge()
        self._state: DuburiState | None = None
        self._frames_received = 0
        self._last_wait_log = self.get_clock().now()

        # Best-effort QoS: a slow viewer must never stall the publisher
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Image, img_topic, self._on_image, qos)
        self.create_subscription(DuburiState, '/duburi/state', self._on_state, 10)

        # Periodic "still waiting" reminder until first frame
        self.create_timer(1.0, self._check_waiting)

    def _check_waiting(self) -> None:
        if self._frames_received > 0:
            return
        now = self.get_clock().now()
        elapsed = (now - self._last_wait_log).nanoseconds / 1e9
        if elapsed >= _WAIT_LOG_INTERVAL:
            self.get_logger().warn('[DISP ] no frames yet — is the camera/detector pipeline running?')
            self._last_wait_log = now

    def _on_state(self, msg: DuburiState) -> None:
        self._state = msg

    def _on_image(self, msg: Image) -> None:
        self._frames_received += 1
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self._state is not None:
            _draw_hud(frame, self._state)
        cv2.imshow('duburi vision', frame)
        if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q')):
            raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    node = VisionDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

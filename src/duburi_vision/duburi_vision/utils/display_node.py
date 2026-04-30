#!/usr/bin/env python3
"""vision_display -- OpenCV viewer for the perception pipeline.

Subscribes to image_debug and /duburi/state and shows a live window with
a HUD overlay. No Qt or rqt required.

With launch_pipeline:=true the node also starts camera_node and
detector_node as child processes so the whole pipeline comes up with
a single command:

    ros2 run duburi_vision vision_display --ros-args -p launch_pipeline:=true

    # Choose camera and model:
    ros2 run duburi_vision vision_display --ros-args \\
        -p launch_pipeline:=true -p camera:=forward \\
        -p model:=gate_flare_medium_100ep -p classes:=gate

ROS2 parameters
---------------
  camera          string   'forward'               camera namespace
  topic           string   ''                      override full image topic
  launch_pipeline bool     false                   auto-start camera_node + detector_node
  model           string   'yolo26_nano_pretrained' model name/path (launch_pipeline only)
  classes         string   'person'                class filter   (launch_pipeline only)
  conf            float    0.35                    confidence     (launch_pipeline only)

Press Q or Ctrl-C to exit. Child processes are terminated on exit.
"""

from __future__ import annotations

import subprocess
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

from duburi_interfaces.msg import DuburiState

# HUD layout
_HUD_FONT      = cv2.FONT_HERSHEY_SIMPLEX
_HUD_SCALE     = 0.55
_HUD_THICKNESS = 1
_HUD_PAD       = 8
_HUD_LINE_H    = 22
_HUD_BG_ALPHA  = 0.45

_WAIT_LOG_INTERVAL = 5.0  # seconds between "still waiting" reminders


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


def _start_pipeline(camera: str, model: str, classes: str,
                    conf: float) -> list[subprocess.Popen]:
    """Spawn camera_node + detector_node as child processes.

    Returns the list of Popen handles so the caller can terminate them.
    Waits 1 s for camera_node to start before launching detector_node.
    """
    camera_proc = subprocess.Popen([
        'ros2', 'run', 'duburi_vision', 'camera_node',
        '--ros-args',
        '-p', f'name:={camera}',
        '-p', 'source:=webcam',
    ])

    time.sleep(1.0)  # give camera_node time to advertise its topic

    detector_proc = subprocess.Popen([
        'ros2', 'run', 'duburi_vision', 'detector_node',
        '--ros-args',
        '-p', f'camera:={camera}',
        '-p', f'model_path:={model}',
        '-p', f'classes:={classes}',
        '-p', f'conf:={conf}',
        '-p', 'publish_debug_image:=true',
    ])

    return [camera_proc, detector_proc]


class VisionDisplayNode(Node):
    def __init__(self):
        super().__init__('vision_display')

        self.declare_parameter('camera',          'forward')
        self.declare_parameter('topic',           '')
        self.declare_parameter('launch_pipeline', False)
        self.declare_parameter('model',           'yolo26_nano_pretrained')
        self.declare_parameter('classes',         'person')
        self.declare_parameter('conf',            0.35)

        camera          = self.get_parameter('camera').get_parameter_value().string_value
        topic           = self.get_parameter('topic').get_parameter_value().string_value
        launch_pipeline = self.get_parameter('launch_pipeline').get_parameter_value().bool_value
        model           = self.get_parameter('model').get_parameter_value().string_value
        classes         = self.get_parameter('classes').get_parameter_value().string_value
        conf            = self.get_parameter('conf').get_parameter_value().double_value

        img_topic = topic or f'/duburi/vision/{camera}/image_debug'

        self._pipeline_procs: list[subprocess.Popen] = []

        if launch_pipeline:
            self.get_logger().info(f'[DISP ] starting camera_node + detector_node for camera={camera}')
            self._pipeline_procs = _start_pipeline(camera, model, classes, conf)

        self.get_logger().info(f'[DISP ] subscribing to {img_topic}')
        if not launch_pipeline:
            self.get_logger().info('[DISP ] tip: add --ros-args -p launch_pipeline:=true to start the full pipeline')

        self._bridge = CvBridge()
        self._state: DuburiState | None = None
        self._frames_received = 0
        self._last_wait_log = self.get_clock().now()

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, img_topic, self._on_image, qos)
        self.create_subscription(DuburiState, '/duburi/state', self._on_state, 10)
        self.create_timer(1.0, self._check_waiting)

    def _check_waiting(self) -> None:
        if self._frames_received > 0:
            return
        now = self.get_clock().now()
        elapsed = (now - self._last_wait_log).nanoseconds / 1e9
        if elapsed >= _WAIT_LOG_INTERVAL:
            self.get_logger().warn('[DISP ] no frames yet — is the pipeline running? (try launch_pipeline:=true)')
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

    def stop_pipeline(self) -> None:
        for proc in self._pipeline_procs:
            proc.terminate()
        self._pipeline_procs.clear()


def main(args=None):
    rclpy.init(args=args)
    node = VisionDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.stop_pipeline()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

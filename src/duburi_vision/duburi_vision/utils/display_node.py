#!/usr/bin/env python3
"""vision_display -- smooth OpenCV viewer for the perception pipeline.

Subscribes to image_raw at full camera FPS and overlays the latest
detections on every frame, so the window stays smooth (30 Hz) even
when the detector runs at 5-10 Hz on GPU.

Architecture
------------
  ROS spin runs on a background daemon thread; the image callback deposits
  decoded+annotated frames into a single-slot queue (old frames are dropped
  to keep latency near zero).  The main thread drains the queue and calls
  cv2.imshow + cv2.waitKey -- both of which MUST run on the main thread
  because cv2's event loop and GUI context are not thread-safe.

With launch_pipeline:=true the node also starts camera_node and
detector_node as child processes so the whole pipeline comes up with
a single command:

    ros2 run duburi_vision vision_display --ros-args -p launch_pipeline:=true

    # Sim / webcam test with yolov11n pretrained (person class):
    ros2 run duburi_vision vision_display --ros-args \\
        -p launch_pipeline:=true -p camera:=laptop \\
        -p model:=yolov11n -p classes:=person

    # Pool run with gate+flare model:
    ros2 run duburi_vision vision_display --ros-args \\
        -p launch_pipeline:=true -p camera:=forward \\
        -p model:=gate_flare_medium_100ep -p classes:=gate

ROS2 parameters
---------------
  camera          string   'forward'    camera namespace
  launch_pipeline bool     false        auto-start camera_node + detector_node
  model           string   'yolov11n'   model name/path  (launch_pipeline only)
  classes         string   'person'     class filter     (launch_pipeline only)
  conf            float    0.35         confidence       (launch_pipeline only)
  max_display_hz  float    30.0         display refresh cap (0 = unlimited)

Press Q or Ctrl-C to exit. Child processes are terminated on exit.
"""

from __future__ import annotations

import queue
import subprocess
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from duburi_interfaces.msg import DuburiState
from duburi_vision import draw
from duburi_vision.detection.messages import array_to_detections

# HUD layout
_HUD_FONT      = cv2.FONT_HERSHEY_SIMPLEX
_HUD_SCALE     = 0.55
_HUD_THICKNESS = 1
_HUD_PAD       = 8
_HUD_LINE_H    = 22
_HUD_BG_ALPHA  = 0.45

_WAIT_LOG_INTERVAL = 5.0  # seconds between "still waiting" reminders
_WINDOW_NAME       = 'duburi vision'


def _draw_hud(frame, state: DuburiState) -> None:
    """Overlay depth / yaw / mode / battery in the top-right corner."""
    lines = [
        f"depth : {state.depth_m:+.2f} m",
        f"yaw   : {state.yaw_deg:.1f} deg",
        f"mode  : {state.mode or '?'}",
        f"batt  : {state.battery_voltage:.1f} V",
        f"armed : {'YES' if state.armed else 'no'}",
    ]

    w = frame.shape[1]
    max_w = max(
        cv2.getTextSize(l, _HUD_FONT, _HUD_SCALE, _HUD_THICKNESS)[0][0]
        for l in lines
    )
    box_h = _HUD_LINE_H * len(lines) + _HUD_PAD
    box_w = max_w + _HUD_PAD * 2
    x_off = w - box_w - 4

    overlay = frame.copy()
    cv2.rectangle(overlay, (x_off, 0), (w - 4, box_h), (20, 20, 20), -1)
    cv2.addWeighted(overlay, _HUD_BG_ALPHA, frame, 1 - _HUD_BG_ALPHA, 0, frame)

    for i, line in enumerate(lines):
        y = _HUD_PAD + (i + 1) * _HUD_LINE_H - 4
        cv2.putText(frame, line, (x_off + _HUD_PAD, y),
                    _HUD_FONT, _HUD_SCALE, (220, 220, 220), _HUD_THICKNESS, cv2.LINE_AA)


def _start_pipeline(camera: str, model: str, classes: str,
                    conf: float) -> list[subprocess.Popen]:
    """Spawn camera_node + detector_node as child processes."""
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
        '-p', 'publish_debug_image:=false',  # display renders its own overlay
    ])

    return [camera_proc, detector_proc]


class VisionDisplayNode(Node):
    def __init__(self):
        super().__init__('vision_display')

        self.declare_parameter('camera',          'forward')
        self.declare_parameter('launch_pipeline', False)
        self.declare_parameter('model',           'yolov11n')
        self.declare_parameter('classes',         'person')
        self.declare_parameter('conf',            0.35)
        self.declare_parameter('max_display_hz',  30.0)

        camera          = self.get_parameter('camera').get_parameter_value().string_value
        launch_pipeline = self.get_parameter('launch_pipeline').get_parameter_value().bool_value
        model           = self.get_parameter('model').get_parameter_value().string_value
        classes         = self.get_parameter('classes').get_parameter_value().string_value
        conf            = self.get_parameter('conf').get_parameter_value().double_value
        max_hz          = self.get_parameter('max_display_hz').get_parameter_value().double_value

        self._camera        = camera
        self._max_hz        = max_hz
        self._pipeline_procs: list[subprocess.Popen] = []

        if launch_pipeline:
            self.get_logger().info(f'[DISP ] starting camera_node + detector_node for camera={camera}')
            self._pipeline_procs = _start_pipeline(camera, model, classes, conf)

        raw_topic = f'/duburi/vision/{camera}/image_raw'
        det_topic = f'/duburi/vision/{camera}/detections'

        self.get_logger().info(f'[DISP ] subscribing {raw_topic} (full-rate) + {det_topic}')
        if not launch_pipeline:
            self.get_logger().info('[DISP ] tip: add --ros-args -p launch_pipeline:=true to start the full pipeline')

        self._bridge = CvBridge()
        self._state: DuburiState | None = None
        self._detections: list = []
        self._det_lock        = threading.Lock()
        self._frames_received = 0
        self._last_wait_log   = self.get_clock().now()

        # FPS tracking
        self._fps_t0      = time.monotonic()
        self._fps_count   = 0
        self._fps_display = 0.0

        # Single-slot frame queue: callback drops old frame and puts latest.
        # Main thread drains this; cv2 calls stay off the ROS callback thread.
        self._frame_q: queue.SimpleQueue = queue.SimpleQueue()

        qos_be = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Image, raw_topic, self._on_image, qos_be)
        self.create_subscription(Detection2DArray, det_topic, self._on_detections, 10)
        self.create_subscription(DuburiState, '/duburi/state', self._on_state, 10)
        self.create_timer(1.0, self._check_waiting)

    # ------------------------------------------------------------------ #
    #  ROS callbacks (run on the background spin thread)                  #
    # ------------------------------------------------------------------ #

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

    def _on_detections(self, msg: Detection2DArray) -> None:
        dets = array_to_detections(msg)
        with self._det_lock:
            self._detections = dets

    def _on_image(self, msg: Image) -> None:
        self._frames_received += 1

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'[DISP ] cv_bridge decode failed: {exc!r}')
            return

        # FPS counter (callback-side; display loop does NOT need its own)
        self._fps_count += 1
        now = time.monotonic()
        elapsed = now - self._fps_t0
        if elapsed >= 1.0:
            self._fps_display = self._fps_count / elapsed
            self._fps_count = 0
            self._fps_t0 = now

        # Annotate here (GPU-friendly; stays off the main thread).
        with self._det_lock:
            dets = list(self._detections)

        from duburi_vision.detection.detector import largest
        primary = largest(dets)
        frame = draw.render_all(
            frame, dets,
            source=self._camera,
            fps=self._fps_display,
            device='',
            healthy=True,
            deadband=0.05,
            primary=primary,
        )

        if self._state is not None:
            _draw_hud(frame, self._state)

        # Single-slot handoff: drop any queued frame and put the latest.
        while not self._frame_q.empty():
            try:
                self._frame_q.get_nowait()
            except queue.Empty:
                break
        self._frame_q.put_nowait(frame)

    # ------------------------------------------------------------------ #
    #  Teardown                                                            #
    # ------------------------------------------------------------------ #

    def stop_pipeline(self) -> None:
        for proc in self._pipeline_procs:
            proc.terminate()
        self._pipeline_procs.clear()


def main(args=None):
    rclpy.init(args=args)
    node = VisionDisplayNode()

    # Spin on a background daemon thread so the main thread stays free for
    # cv2.imshow + cv2.waitKey (both require the main thread on most platforms).
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    frame_budget = 1.0 / node._max_hz if node._max_hz > 0 else 0.0

    try:
        cv2.namedWindow(_WINDOW_NAME, cv2.WINDOW_NORMAL)
        while rclpy.ok():
            try:
                frame = node._frame_q.get(timeout=0.1)
            except queue.Empty:
                # No frame yet — check for Q key and loop.
                if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q')):
                    break
                continue

            t0 = time.monotonic()
            cv2.imshow(_WINDOW_NAME, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q')):
                break

            # Rate-limit: sleep the remainder of the frame budget.
            if frame_budget > 0:
                elapsed = time.monotonic() - t0
                remaining = frame_budget - elapsed
                if remaining > 0:
                    time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.stop_pipeline()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)

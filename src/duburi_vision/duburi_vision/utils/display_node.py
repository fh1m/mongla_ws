#!/usr/bin/env python3
"""vision_display -- smooth OpenCV viewer for the perception pipeline.

Subscribes to image_raw at full camera FPS and overlays the latest
detections on every frame, so the window stays smooth (30 Hz) even
when the detector runs at 5-10 Hz on GPU.

Architecture
------------
  ROS spin runs on a background daemon thread; the image callback deposits
  decoded raw frames into a single-slot queue (old frames are dropped to keep
  latency near zero).  The main thread drains the queue, annotates with the
  latest cached detections, and calls cv2.imshow + cv2.waitKey -- both of
  which MUST run on the main thread because cv2's event loop and GUI context
  are not thread-safe.  Annotation on the display thread (not the callback)
  keeps the ROS executor free to service incoming messages between frames.

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

Video file keyboard shortcuts (active when video_file_mode:=true)
-----------------------------------------------------------------
  Space        → pause / resume
  Left arrow   → seek -1 s
  Right arrow  → seek +1 s
  Up arrow     → seek +10 s
  Down arrow   → seek -10 s

ROS2 parameters
---------------
  camera           string   'forward'    camera namespace
  video_file_mode  bool     false        enable video playback keyboard controls
  launch_pipeline  bool     false        auto-start camera_node + detector_node
  model            string   'yolov11n'   model name/path  (launch_pipeline only)
  classes          string   'person'     class filter     (launch_pipeline only)
  conf             float    0.35         confidence       (launch_pipeline only)
  max_display_hz   float    30.0         display refresh cap (0 = unlimited)

Press Q or Ctrl-C to exit. Child processes are terminated on exit.
"""

from __future__ import annotations

import queue
import subprocess
import threading
import time
from collections import deque

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool
from vision_msgs.msg import Detection2DArray

from duburi_interfaces.msg import DuburiState
from duburi_vision import draw
from duburi_vision.detection.detector import Detection, largest
from duburi_vision.detection.messages import array_to_detections

_WAIT_LOG_INTERVAL = 5.0
_WINDOW_NAME       = 'duburi  //  mission control'

# Render at 2× native camera resolution so sf=2 inside draw_strip → fonts/
# widgets drawn at full pixel size → crisp on 1080p/4K monitors regardless
# of OS window scaling.  Fixed constant avoids feedback-loop instability.
_RENDER_SCALE = 2.0

# Arrow key codes from cv2.waitKey on Linux (after & 0xFF they become 81-84).
# Use waitKeyEx() codes instead — waitKeyEx returns full 32-bit extended codes.
_KEY_LEFT  = 0xFF51  # left  arrow (XK_Left)
_KEY_RIGHT = 0xFF53  # right arrow (XK_Right)
_KEY_UP    = 0xFF52  # up    arrow (XK_Up)
_KEY_DOWN  = 0xFF54  # down  arrow (XK_Down)


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


def _scale_dets(dets: list, scale: float) -> list:
    """Return a new list of Detections with xyxy scaled by `scale`."""
    return [
        Detection(class_id=d.class_id, class_name=d.class_name, score=d.score,
                  xyxy=(d.xyxy[0] * scale, d.xyxy[1] * scale,
                        d.xyxy[2] * scale, d.xyxy[3] * scale))
        for d in dets
    ]


def _build_health(node: 'VisionDisplayNode') -> dict[str, bool]:
    """Build pipeline health dict from last-seen timestamps."""
    now = time.monotonic()
    return {
        'camera':   (now - node._last_frame_t) < 2.0,
        'detector': (now - node._last_det_t)   < 3.0,
        'tracker':  (now - node._last_track_t) < 3.0,
        'state':    (now - node._last_state_t) < 5.0,
    }


class VisionDisplayNode(Node):
    def __init__(self):
        super().__init__('vision_display')

        self.declare_parameter('camera',          'forward')
        self.declare_parameter('video_file_mode', False)
        self.declare_parameter('launch_pipeline', False)
        self.declare_parameter('model',           'yolov11n')
        self.declare_parameter('classes',         'person')
        self.declare_parameter('conf',            0.35)
        self.declare_parameter('max_display_hz',  30.0)
        self.declare_parameter('yaw_source', 'mavlink_ahrs')

        camera          = self.get_parameter('camera').get_parameter_value().string_value
        video_file_mode = self.get_parameter('video_file_mode').get_parameter_value().bool_value
        launch_pipeline = self.get_parameter('launch_pipeline').get_parameter_value().bool_value
        model           = self.get_parameter('model').get_parameter_value().string_value
        classes         = self.get_parameter('classes').get_parameter_value().string_value
        conf            = self.get_parameter('conf').get_parameter_value().double_value
        max_hz          = self.get_parameter('max_display_hz').get_parameter_value().double_value

        self._camera        = camera
        self._video_mode    = video_file_mode
        self._max_hz        = max_hz
        self._yaw_source    = self.get_parameter('yaw_source').get_parameter_value().string_value
        self._pipeline_procs: list[subprocess.Popen] = []

        if launch_pipeline:
            self.get_logger().info(f'[DISP ] starting camera_node + detector_node for camera={camera}')
            self._pipeline_procs = _start_pipeline(camera, model, classes, conf)

        raw_topic = f'/duburi/vision/{camera}/image_raw'
        det_topic = f'/duburi/vision/{camera}/detections'

        self.get_logger().info(f'[DISP ] subscribing {raw_topic} (full-rate) + {det_topic}')

        self._bridge = CvBridge()
        self._state: DuburiState | None = None
        self._detections: list = []
        self._det_lock        = threading.Lock()
        self._tracked_dets: list = []
        self._track_ids:    list = []
        self._n_tracks            = 0
        self._primary_track_id: int | None = None
        self._tracks_lock         = threading.Lock()
        self._configured_classes: list[str] = []
        self._frames_received = 0
        self._last_wait_log   = self.get_clock().now()

        # FPS tracking
        self._fps_t0      = time.monotonic()
        self._fps_count   = 0
        self._fps_display = 0.0

        # Pipeline health timestamps (monotonic; default far past so initial health=False).
        self._last_frame_t = 0.0
        self._last_det_t   = 0.0
        self._last_track_t = 0.0
        self._last_state_t = 0.0

        # Rolling history for sparkline graphs (60 frames).
        self._err_x_history: deque[float] = deque(maxlen=60)
        self._err_y_history: deque[float] = deque(maxlen=60)
        self._conf_history:  deque[float] = deque(maxlen=60)

        # Video file state (updated by ROS service response).
        self._is_paused:      bool            = False
        self._video_position: tuple[int, int] = (0, 0)

        # Single-slot frame queue.
        self._frame_q: queue.SimpleQueue = queue.SimpleQueue()

        qos_be = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        cls_topic = f'/duburi/vision/{camera}/classes_filter'
        trk_topic = f'/duburi/vision/{camera}/tracks'
        self.create_subscription(Image,            raw_topic,        self._on_image,          qos_be)
        self.create_subscription(Detection2DArray, det_topic,        self._on_detections,     10)
        self.create_subscription(Detection2DArray, trk_topic,        self._on_tracks,         10)
        self.create_subscription(DuburiState,      '/duburi/state',  self._on_state,          10)
        self.create_subscription(String,           cls_topic,        self._on_classes_filter, 10)
        self.create_timer(1.0, self._check_waiting)

        # Video playback control clients (only when video_file_mode=true).
        self._pause_client = None   # rclpy.Client[SetBool] or None
        self._seek_pub = None
        if video_file_mode:
            cam_ns = f'/duburi/vision/{camera}'
            self._pause_client = self.create_client(SetBool, f'{cam_ns}/video_pause')
            self._seek_pub     = self.create_publisher(Float32, f'{cam_ns}/video_seek_rel', 10)
            self.get_logger().info(
                '[DISP ] video mode: Space=pause  ←/→=±1s  ↑/↓=±10s')

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
        self._last_state_t = time.monotonic()

    def _on_detections(self, msg: Detection2DArray) -> None:
        dets = array_to_detections(msg)
        with self._det_lock:
            self._detections = dets
        self._last_det_t = time.monotonic()

    def _on_tracks(self, msg: Detection2DArray) -> None:
        all_dets = array_to_detections(msg)
        pairs = [(d, det) for d, det in zip(all_dets, msg.detections) if d.score > 0]
        dets = [p[0] for p in pairs]
        ids: list[int | None] = []
        for _, det in pairs:
            try:
                ids.append(int(det.id))
            except (ValueError, TypeError):
                ids.append(None)
        with self._tracks_lock:
            self._tracked_dets      = dets
            self._track_ids         = ids
            self._n_tracks          = len([i for i in ids if i is not None])
            self._primary_track_id  = next((i for i in ids if i is not None), None)
        self._last_track_t = time.monotonic()

    def _on_classes_filter(self, msg: String) -> None:
        self._configured_classes = [c.strip() for c in msg.data.split(',') if c.strip()]

    def _on_image(self, msg: Image) -> None:
        self._frames_received += 1
        self._last_frame_t = time.monotonic()

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'[DISP ] cv_bridge decode failed: {exc!r}')
            return

        self._fps_count += 1
        now = time.monotonic()
        elapsed = now - self._fps_t0
        if elapsed >= 1.0:
            self._fps_display = self._fps_count / elapsed
            self._fps_count = 0
            self._fps_t0 = now

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
        for proc in self._pipeline_procs:
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()
        self._pipeline_procs.clear()


def _send_seek(node: VisionDisplayNode, seconds: float) -> None:
    if node._seek_pub is not None:
        node._seek_pub.publish(Float32(data=float(seconds)))


def _send_pause(node: VisionDisplayNode, pause: bool) -> None:
    """Fire-and-forget pause/resume request (non-blocking)."""
    if node._pause_client is None or not node._pause_client.service_is_ready():
        return
    req = SetBool.Request()
    req.data = pause
    future = node._pause_client.call_async(req)

    def _on_done(f):
        if f.result() is not None:
            node._is_paused = pause

    future.add_done_callback(_on_done)


def main(args=None):
    rclpy.init(args=args)
    node = VisionDisplayNode()

    from rclpy.executors import ExternalShutdownException

    def _spin_target(n):
        try:
            rclpy.spin(n)
        except (ExternalShutdownException, Exception):
            pass

    spin_thread = threading.Thread(target=_spin_target, args=(node,), daemon=True)
    spin_thread.start()

    frame_budget = 1.0 / node._max_hz if node._max_hz > 0 else 0.0

    try:
        cv2.namedWindow(_WINDOW_NAME, cv2.WINDOW_KEEPRATIO | cv2.WINDOW_NORMAL)
        cv2.resizeWindow(_WINDOW_NAME, 1920, 1080)  # initial hint; user can resize freely
        while rclpy.ok():
            try:
                frame = node._frame_q.get(timeout=0.1)
            except queue.Empty:
                key = cv2.waitKeyEx(1)
                if key in (ord('q'), ord('Q')):
                    break
                continue

            with node._det_lock:
                dets = list(node._detections)
            with node._tracks_lock:
                tracked_dets     = list(node._tracked_dets)
                track_ids        = list(node._track_ids)
                n_tracks         = node._n_tracks
                primary_track_id = node._primary_track_id
            configured_classes = node._configured_classes

            display_dets = tracked_dets if tracked_dets else dets
            display_ids  = track_ids if tracked_dets else None
            primary = largest(display_dets)

            # Compute err history from native-resolution coords (before upscaling)
            # so the normalised values [-1, 1] are stable regardless of render scale.
            if primary is not None:
                h, w = frame.shape[:2]
                node._err_x_history.append((primary.cx / w - 0.5) * 2.0)
                node._err_y_history.append((primary.cy / h - 0.5) * 2.0)
                node._conf_history.append(primary.score)
            else:
                node._err_x_history.append(0.0)
                node._err_y_history.append(0.0)
                node._conf_history.append(0.0)

            # Upscale frame + detection coordinates so sf = _RENDER_SCALE inside
            # draw_strip → fonts drawn at full pixel size → crisp on 1080p displays.
            native_h, native_w = frame.shape[:2]
            render_w = int(native_w * _RENDER_SCALE)
            render_h = int(native_h * _RENDER_SCALE)
            render_frame  = cv2.resize(frame, (render_w, render_h),
                                       interpolation=cv2.INTER_LINEAR)
            render_dets   = _scale_dets(display_dets, _RENDER_SCALE)
            render_primary = (_scale_dets([primary], _RENDER_SCALE)[0]
                              if primary is not None else None)
            render_ids    = display_ids  # track IDs are integers — no scaling needed

            frame = draw.render_all(
                render_frame, render_dets,
                source=node._camera,
                fps=node._fps_display,
                healthy=True,
                deadband=0.05,
                primary=render_primary,
                tracking_on=n_tracks > 0,
                n_tracks=n_tracks,
                primary_track_id=primary_track_id,
                state=node._state,
                configured_classes=configured_classes,
                track_ids=render_ids,
                yaw_source=node._yaw_source,
                err_x_history=node._err_x_history,
                err_y_history=node._err_y_history,
                conf_history=node._conf_history,
                video_mode=node._video_mode,
                is_paused=node._is_paused,
                pipeline_health=_build_health(node),
            )

            t0 = time.monotonic()
            cv2.imshow(_WINDOW_NAME, frame)
            key = cv2.waitKeyEx(1)

            if key in (ord('q'), ord('Q')):
                break

            # Video file keyboard controls.
            if node._video_mode:
                if key == ord(' '):
                    new_paused = not node._is_paused
                    _send_pause(node, new_paused)
                elif key == _KEY_LEFT:
                    _send_seek(node, -1.0)
                elif key == _KEY_RIGHT:
                    _send_seek(node, 1.0)
                elif key == _KEY_UP:
                    _send_seek(node, 10.0)
                elif key == _KEY_DOWN:
                    _send_seek(node, -10.0)

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

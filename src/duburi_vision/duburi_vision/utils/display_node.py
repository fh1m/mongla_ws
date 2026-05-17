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
  , (comma)    → step 1 frame back   (best used while paused)
  . (period)   → step 1 frame forward (best used while paused)
  [ (bracket)  → slow down playback (0.1→0.25→0.5→0.75→1.0→…)
  ] (bracket)  → speed up playback  (…→1.5→2.0→4.0)

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
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray, Int32, String
from std_srvs.srv import SetBool
from vision_msgs.msg import Detection2DArray

from duburi_interfaces.msg import DuburiState
from duburi_vision import draw
from duburi_vision.config import CAMERA_PROFILES
from duburi_vision.detection.detector import Detection, largest
from duburi_vision.detection.messages import array_to_detections
from duburi_vision.draw_widgets import pil_text, pil_text_size

_WAIT_LOG_INTERVAL = 5.0
_WINDOW_NAME       = 'duburi  //  mission control'

# Target display width. Frames are resized to this width (single step, no
# intermediate scale) so draw_strip's sf=max(0.6, w/1920) resolves to 1.0.
_RENDER_W = 1920

# Arrow key codes from cv2.waitKey on Linux (after & 0xFF they become 81-84).
# Use waitKeyEx() codes instead — waitKeyEx returns full 32-bit extended codes.
_KEY_LEFT  = 0xFF51  # left  arrow (XK_Left)
_KEY_RIGHT = 0xFF53  # right arrow (XK_Right)
_KEY_UP    = 0xFF52  # up    arrow (XK_Up)
_KEY_DOWN  = 0xFF54  # down  arrow (XK_Down)


def _start_pipeline(camera: str, model: str, classes: str,
                    conf: float) -> list[subprocess.Popen]:
    """Spawn camera_node + detector_node as child processes."""
    source = CAMERA_PROFILES.get(camera, {}).get('source', 'webcam')
    camera_proc = subprocess.Popen([
        'ros2', 'run', 'duburi_vision', 'camera_node',
        '--ros-args',
        '-p', f'name:={camera}',
        '-p', f'source:={source}',
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


def _scale_dets(dets: list, sx: float, sy: float) -> list:
    """Return a new list of Detections with xyxy scaled by sx (x-axis) and sy (y-axis)."""
    return [
        Detection(class_id=d.class_id, class_name=d.class_name, score=d.score,
                  xyxy=(d.xyxy[0] * sx, d.xyxy[1] * sy,
                        d.xyxy[2] * sx, d.xyxy[3] * sy))
        for d in dets
    ]


def _build_health(node: 'VisionDisplayNode') -> dict[str, bool]:
    """Build pipeline health dict from last-seen timestamps."""
    now = time.monotonic()
    return {
        'camera':   (now - node._last_frame_t)     < 2.0,
        'detector': (now - node._last_det_t)        < 3.0,
        'tracker':  (now - node._last_track_t)      < 3.0,
        'state':    (now - node._last_state_t)      < 5.0,
        'depth':    (now - node._last_vis_range_t)  < 3.0,
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

        # Depth rate estimation — short window, state-message timestamps
        self._depth_history: deque[tuple[float, float]] = deque(maxlen=10)
        self._depth_rate: float = 0.0

        # vis_range (monocular proximity estimate, 0=far 1=close)
        self._vis_range_values: list[float] = []
        self._vis_range_lock   = threading.Lock()
        self._last_vis_range_t = 0.0

        # depth map (colorized BGR, cached from vis_range_map topic)
        self._depth_map_bgr: np.ndarray | None = None
        self._depth_map_lock = threading.Lock()
        self._show_depth_map: bool = False   # toggled by 'D' keypress

        # Video file state.
        self._is_paused:        bool            = False
        self._video_position:   tuple[int, int] = (0, 0)
        self._video_speed:      float           = 1.0  # mirrors VideoFileCamera._speed
        # True until first detection arrives — video stays paused (splash shown) during this window.
        self._auto_paused_start: bool           = video_file_mode

        # Single-slot frame queue.
        self._frame_q: queue.SimpleQueue = queue.SimpleQueue()

        qos_be = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        cls_topic = f'/duburi/vision/{camera}/classes_filter'
        trk_topic = f'/duburi/vision/{camera}/tracks'
        vr_topic  = f'/duburi/vision/{camera}/vis_range'
        vmap_topic = f'/duburi/vision/{camera}/vis_range_map'
        self.create_subscription(Image,             raw_topic,    self._on_image,          qos_be)
        self.create_subscription(Detection2DArray,  det_topic,    self._on_detections,     10)
        self.create_subscription(Detection2DArray,  trk_topic,    self._on_tracks,         10)
        self.create_subscription(DuburiState,       '/duburi/state', self._on_state,       10)
        self.create_subscription(String,            cls_topic,    self._on_classes_filter, 10)
        self.create_subscription(Float32MultiArray, vr_topic,     self._on_vis_range,      10)
        self.create_subscription(Image,             vmap_topic,   self._on_depth_map,      2)
        self.create_timer(1.0, self._check_waiting)

        # Video playback control clients (only when video_file_mode=true).
        self._pause_client   = None   # rclpy.Client[SetBool] or None
        self._seek_pub       = None
        self._seek_frame_pub = None
        self._speed_pub      = None
        if video_file_mode:
            cam_ns = f'/duburi/vision/{camera}'
            self._pause_client      = self.create_client(SetBool, f'{cam_ns}/video_pause')
            self._seek_pub          = self.create_publisher(Float32, f'{cam_ns}/video_seek_rel',   10)
            self._seek_frame_pub    = self.create_publisher(Int32,   f'{cam_ns}/video_seek_frame', 10)
            self._speed_pub         = self.create_publisher(Float32, f'{cam_ns}/video_speed',      10)
            self.get_logger().info(
                '[DISP ] video mode: Space=pause  ←/→=±1s  ↑/↓=±10s  ,/.=frame step  [/]=speed')

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
        now = time.monotonic()
        self._last_state_t = now
        if not np.isnan(msg.depth_m):
            self._depth_history.append((now, float(msg.depth_m)))
            if len(self._depth_history) >= 2:
                t0, d0 = self._depth_history[0]
                t1, d1 = self._depth_history[-1]
                dt = t1 - t0
                self._depth_rate = (d1 - d0) / dt if dt > 0.1 else 0.0

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

    def _on_vis_range(self, msg: Float32MultiArray) -> None:
        with self._vis_range_lock:
            self._vis_range_values = list(msg.data)
            self._last_vis_range_t = time.monotonic()

    def _on_depth_map(self, msg: Image) -> None:
        try:
            raw  = self._bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            norm = np.clip(raw, 0.0, 1.0)
            colorized = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
            with self._depth_map_lock:
                self._depth_map_bgr = colorized
        except Exception:
            pass

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


# ---------------------------------------------------------------------------
# Video playback helpers
# ---------------------------------------------------------------------------

def _send_seek(node: VisionDisplayNode, seconds: float) -> None:
    if node._seek_pub is not None:
        node._seek_pub.publish(Float32(data=float(seconds)))


def _send_seek_frame(node: VisionDisplayNode, frames: int) -> None:
    if node._seek_frame_pub is not None:
        node._seek_frame_pub.publish(Int32(data=int(frames)))


_SPEED_STEPS = (0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 4.0)


def _speed_step_up(cur: float) -> float:
    for s in _SPEED_STEPS:
        if s > cur + 0.01:
            return s
    return _SPEED_STEPS[-1]


def _speed_step_down(cur: float) -> float:
    for s in reversed(_SPEED_STEPS):
        if s < cur - 0.01:
            return s
    return _SPEED_STEPS[0]


def _send_speed(node: VisionDisplayNode, speed: float) -> None:
    node._video_speed = speed
    if node._speed_pub is not None:
        node._speed_pub.publish(Float32(data=float(speed)))


def _send_pause(node: VisionDisplayNode, pause: bool) -> None:
    """Update pause state optimistically then fire service call."""
    node._is_paused = pause          # update now — don't wait for service ACK
    if node._pause_client is None or not node._pause_client.service_is_ready():
        return
    req = SetBool.Request()
    req.data = pause
    node._pause_client.call_async(req)


def _handle_video_keys(node: VisionDisplayNode, key: int) -> None:
    """Process video playback keyboard shortcuts — called in BOTH empty and frame paths."""
    if key < 0 or not node._video_mode:
        return
    if key == ord(' '):
        _send_pause(node, not node._is_paused)
    elif key == _KEY_LEFT:
        _send_seek(node, -1.0)
    elif key == _KEY_RIGHT:
        _send_seek(node, 1.0)
    elif key == _KEY_UP:
        _send_seek(node, 10.0)
    elif key == _KEY_DOWN:
        _send_seek(node, -10.0)
    elif key == ord(','):
        _send_seek_frame(node, -1)
    elif key == ord('.'):
        _send_seek_frame(node, 1)
    elif key == ord('['):
        _send_speed(node, _speed_step_down(node._video_speed))
    elif key == ord(']'):
        _send_speed(node, _speed_step_up(node._video_speed))


# ---------------------------------------------------------------------------
# Splash screen
# ---------------------------------------------------------------------------

# Blue theme: dark navy background, sky-blue accent (OpenCV BGR order)
# RGB equivalents: BG=(5,15,55) navy  ACCENT=(0,160,255) sky-blue
_C_SP_BG     = (55, 15, 5)       # dark navy  — B=55 G=15 R=5
_C_SP_ACCENT = (255, 160, 0)     # sky blue   — B=255 G=160 R=0
_C_SP_TEXT   = (240, 235, 220)   # bright warm white
_C_SP_DIM    = (130, 95, 65)     # muted slate


def _render_splash(w: int, h: int, elapsed: float, camera: str,
                   fade: float = 1.0) -> np.ndarray:
    """Blue 'Initializing Vision System' splash. fade=1.0 fully opaque, 0.0 transparent."""
    img = np.full((h, w, 3), _C_SP_BG, dtype=np.uint8)
    cx, cy = w // 2, h // 2
    sf  = max(0.5, w / 1280.0)

    # Brand header
    brand = '───  BRACU  DUBURI  ───'
    fs_b  = 0.85 * sf
    bw, bh = pil_text_size(brand, fs_b)
    bx = cx - bw // 2
    pil_text(img, brand, (bx, cy - 58), fs_b, _C_SP_ACCENT)

    # Accent separator line under brand
    cv2.line(img, (bx, cy - 58 + bh + 6), (bx + bw, cy - 58 + bh + 6), _C_SP_ACCENT, 1)

    # Main status
    status = 'INITIALIZING VISION SYSTEM'
    fs_s   = 0.65 * sf
    sw, _ = pil_text_size(status, fs_s)
    pil_text(img, status, (cx - sw // 2, cy + 22), fs_s, _C_SP_TEXT)

    # Sub-status
    sub    = 'Loading model  ·  video will play automatically when ready'
    fs_sub = 0.38 * sf
    subw, _ = pil_text_size(sub, fs_sub)
    pil_text(img, sub, (cx - subw // 2, cy + 58), fs_sub, _C_SP_DIM)

    # Animated ping-pong progress bar
    bar_len = int(w * 0.48)
    bar_x0  = cx - bar_len // 2
    bar_y0  = cy + 86
    cv2.rectangle(img, (bar_x0, bar_y0), (bar_x0 + bar_len, bar_y0 + 4), (90, 35, 12), -1)
    t    = (elapsed % 2.0) / 2.0
    prog = t * 2 if t < 0.5 else (1.0 - t) * 2
    fill = max(bar_len // 8, int(bar_len * prog))
    cv2.rectangle(img, (bar_x0, bar_y0), (bar_x0 + fill, bar_y0 + 4), _C_SP_ACCENT, -1)

    # Footer
    fs_info = 0.33 * sf
    cam_lbl = f'camera: {camera}'
    pil_text(img, cam_lbl, (16, h - 20), fs_info, _C_SP_DIM)
    ts = f'{int(elapsed)}s'
    tw, _ = pil_text_size(ts, fs_info)
    pil_text(img, ts, (w - tw - 16, h - 20), fs_info, _C_SP_DIM)

    # Apply fade: blend toward black at fade < 1
    if fade < 0.999:
        img = (img * max(0.0, fade)).astype(np.uint8)
    return img


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

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

    frame_budget  = 1.0 / node._max_hz if node._max_hz > 0 else 0.0
    splash_start  = time.monotonic()
    # Splash at target display resolution — rendered output is always scaled to this.
    _SP_W, _SP_H  = 1920, 1080

    try:
        cv2.namedWindow(_WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(_WINDOW_NAME, 1920, 1080)

        # Show splash immediately; pause video until detector fires first detections.
        cv2.imshow(_WINDOW_NAME, _render_splash(_SP_W, _SP_H, 0.0, node._camera))
        cv2.waitKeyEx(1)
        if node._video_mode:
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and rclpy.ok():
                if node._pause_client and node._pause_client.service_is_ready():
                    _send_pause(node, True)
                    break
                time.sleep(0.05)

        _splash_fade_end   = 0.0   # monotonic time when 400 ms fade-out finishes
        _SPLASH_FADE_DUR   = 0.4
        _prev_initializing = True

        while rclpy.ok():
            # ── Auto-resume when detector first becomes active ──────────────
            if node._video_mode and node._auto_paused_start and node._last_det_t > 0:
                node._auto_paused_start = False
                _send_pause(node, False)

            # Splash shows while detector has never published (all modes).
            _initializing = node._last_det_t == 0

            # Trigger fade-out on the first tick after detection fires.
            if _prev_initializing and not _initializing:
                _splash_fade_end = time.monotonic() + _SPLASH_FADE_DUR
            _prev_initializing = _initializing

            now = time.monotonic()
            _fade = max(0.0, (_splash_fade_end - now) / _SPLASH_FADE_DUR) if not _initializing else 1.0
            _show_splash = _initializing or _fade > 0.01

            # ── Try to get latest frame ─────────────────────────────────────
            try:
                frame = node._frame_q.get(timeout=0.1)
            except queue.Empty:
                # No frame — show animated splash while initializing or fading.
                if _show_splash:
                    cv2.imshow(_WINDOW_NAME,
                               _render_splash(_SP_W, _SP_H,
                                              now - splash_start,
                                              node._camera, fade=_fade))
                key = cv2.waitKeyEx(1)
                if key in (ord('q'), ord('Q')):
                    break
                _handle_video_keys(node, key)
                continue

            # ── Process and render frame ────────────────────────────────────
            t0 = time.monotonic()   # frame budget starts here, covers render + imshow
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
            primary      = largest(display_dets)

            with node._vis_range_lock:
                vis_range_vals = list(node._vis_range_values)
            with node._depth_map_lock:
                depth_map_bgr = (node._depth_map_bgr.copy()
                                 if node._show_depth_map and node._depth_map_bgr is not None
                                 else None)
            primary_vr = 0.0
            if primary is not None and vis_range_vals:
                for i, d in enumerate(display_dets):
                    if (d.class_id == primary.class_id
                            and abs(d.cx - primary.cx) < 1
                            and abs(d.cy - primary.cy) < 1
                            and i < len(vis_range_vals)):
                        primary_vr = vis_range_vals[i]
                        break

            if primary is not None:
                h0, w0 = frame.shape[:2]
                node._err_x_history.append((primary.cx / w0 - 0.5) * 2.0)
                node._err_y_history.append((primary.cy / h0 - 0.5) * 2.0)
                node._conf_history.append(primary.score)
            else:
                node._err_x_history.append(0.0)
                node._err_y_history.append(0.0)
                node._conf_history.append(0.0)

            native_h, native_w = frame.shape[:2]
            video_h      = int(native_h * _RENDER_W / native_w)
            render_frame = cv2.resize(frame, (_RENDER_W, video_h),
                                      interpolation=cv2.INTER_LINEAR)
            sx = _RENDER_W / native_w
            sy = video_h   / native_h
            render_dets    = _scale_dets(display_dets, sx, sy)
            render_primary = (_scale_dets([primary], sx, sy)[0]
                              if primary is not None else None)

            out = draw.render_all(
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
                track_ids=display_ids,
                yaw_source=node._yaw_source,
                err_x_history=node._err_x_history,
                err_y_history=node._err_y_history,
                conf_history=node._conf_history,
                video_mode=node._video_mode,
                is_paused=node._is_paused,
                pipeline_health=_build_health(node),
                depth_rate=node._depth_rate,
                vis_range_values=vis_range_vals,
                primary_vis_range=primary_vr,
                depth_map_bgr=depth_map_bgr,
            )

            # Blend splash over frame while initializing or fading out.
            if _show_splash:
                now2 = time.monotonic()
                splash = _render_splash(out.shape[1], out.shape[0],
                                        now2 - splash_start,
                                        node._camera, fade=_fade)
                alpha = 0.80 * _fade
                cv2.addWeighted(splash, alpha, out, 1.0 - alpha, 0, out)

            if out.shape[0] != _SP_H or out.shape[1] != _SP_W:
                out = cv2.resize(out, (_SP_W, _SP_H), interpolation=cv2.INTER_LINEAR)
            cv2.imshow(_WINDOW_NAME, out)
            key = cv2.waitKeyEx(1)

            if key in (ord('q'), ord('Q')):
                break
            if key in (ord('d'), ord('D')):
                node._show_depth_map = not node._show_depth_map
            _handle_video_keys(node, key)

            if frame_budget > 0:
                remaining = frame_budget - (time.monotonic() - t0)
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

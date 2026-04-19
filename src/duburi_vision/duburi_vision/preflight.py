"""Vision preflight -- block until the perception pipeline is alive.

Used by:
  * `auv_manager_node` -- once per camera, the first time a vision_*
    command is dispatched. Subsequent calls hit the cache.
  * `vision_check` CLI util -- standalone "is the chain healthy?" tool.

Two cheap smoke tests, both topic-based so we don't have to import
ultralytics here:

  1. `/duburi/vision/<cam>/image_raw`     publishing at >= require_image_hz
  2. `/duburi/vision/<cam>/camera_info`   seen at least once (so VisionState
                                          can size-normalize errors against
                                          true image dims, not a hard-coded
                                          640x480)

`require_detections=True` adds:

  3. `/duburi/vision/<cam>/detections`    publishing at >= 1 Hz

Failure mode is loud and friendly: `VisionNotReadyError` carries a
human-readable reason that tells the operator EXACTLY which stage was
silent so they can `ros2 launch duburi_vision webcam_demo.launch.py` or
debug a missing ros_topic source without reading the manager log.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional

from rclpy.node import Node
from rclpy.qos  import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray


class VisionNotReadyError(RuntimeError):
    """Raised when assert_vision_ready times out before any pipeline
    stage came up. The message names the failing stage so the operator
    can fix it without grepping logs."""


@dataclass
class VisionStatus:
    image_hz:      float
    detection_hz:  float
    image_size:    tuple   # (W, H) from CameraInfo, or (0, 0) if not seen
    info_seen:     bool
    elapsed:       float


# Cache so missions don't pay the wait twice. Key = (process id, camera).
_PREFLIGHT_CACHE: dict = {}


def assert_vision_ready(node: Node, *,
                        camera: str = 'laptop',
                        timeout: float = 10.0,
                        require_image_hz: float = 5.0,
                        require_detections: bool = False,
                        sample_window: float = 1.0,
                        log=None) -> VisionStatus:
    """Block up to `timeout` seconds for the vision pipeline to be live.

    Returns a `VisionStatus` on success. Raises `VisionNotReadyError`
    on failure with a one-line reason. Successful results are cached
    per camera for the rest of the process lifetime.
    """
    cache_key = camera
    cached = _PREFLIGHT_CACHE.get(cache_key)
    if cached is not None:
        if log is not None:
            log.info(
                f"[VPRE ] '{camera}' preflight cached "
                f"(image={cached.image_hz:.1f}Hz, info={'OK' if cached.info_seen else '--'})")
        return cached

    ns = f'/duburi/vision/{camera}'
    image_topic   = f'{ns}/image_raw'
    info_topic    = f'{ns}/camera_info'
    detect_topic  = f'{ns}/detections'

    counters = _Counters()
    qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE)

    sub_image = node.create_subscription(
        Image, image_topic, lambda _m: counters.bump_image(), qos)
    sub_info  = node.create_subscription(
        CameraInfo, info_topic, counters.set_info, qos)
    sub_det   = node.create_subscription(
        Detection2DArray, detect_topic, lambda _m: counters.bump_det(), qos)

    if log is not None:
        log.info(
            f"[VPRE ] '{camera}' waiting for image_raw>={require_image_hz:.1f}Hz"
            f"{', detections>=1Hz' if require_detections else ''}  "
            f"timeout={timeout:.1f}s")

    # Loop until either the requirements are satisfied for one full window,
    # or we run out of timeout. The tiny sleep keeps spin_once responsive
    # without burning a core.
    started = time.monotonic()
    last_check = started
    try:
        while True:
            now = time.monotonic()
            elapsed = now - started

            if (now - last_check) >= sample_window:
                window = max(now - last_check, 1e-3)
                image_hz, det_hz = counters.snapshot_hz(window)

                ok_image = image_hz >= require_image_hz
                ok_info  = counters.info_seen
                ok_det   = (det_hz >= 1.0) if require_detections else True

                if ok_image and ok_info and ok_det:
                    status = VisionStatus(
                        image_hz=image_hz, detection_hz=det_hz,
                        image_size=counters.image_size,
                        info_seen=ok_info, elapsed=elapsed)
                    _PREFLIGHT_CACHE[cache_key] = status
                    if log is not None:
                        W, H = status.image_size
                        log.info(
                            f"[VPRE ] '{camera}' READY  image={image_hz:.1f}Hz  "
                            f"det={det_hz:.1f}Hz  size={W}x{H}  "
                            f"after {elapsed:.1f}s")
                    return status
                last_check = now

            if elapsed >= timeout:
                reason = _build_failure_reason(
                    camera, counters, require_image_hz, require_detections)
                raise VisionNotReadyError(reason)

            # Drive ROS callbacks; very short slice so the loop stays snappy.
            import rclpy
            rclpy.spin_once(node, timeout_sec=0.05)

    finally:
        node.destroy_subscription(sub_image)
        node.destroy_subscription(sub_info)
        node.destroy_subscription(sub_det)


def wait_vision_state_ready(vision_state, *,
                            timeout: float = 10.0,
                            stale_after: float = 0.8,
                            require_detection: bool = False,
                            log=None) -> VisionStatus:
    """Poll an EXISTING VisionState until it's healthy.

    Used by the manager: VisionState is owned by the rclpy executor
    thread, so we only need to sleep-and-check from inside an action
    callback -- no spin_once, no second subscription set, no risk of
    re-entering rclpy from a worker.

    `require_detection=True` waits for a fresh Sample (any class) to
    appear; otherwise we settle for "image + camera_info present".
    """
    cache_key = f'state::{id(vision_state)}'
    cached = _PREFLIGHT_CACHE.get(cache_key)
    if cached is not None and not require_detection:
        return cached

    deadline = time.monotonic() + max(timeout, 0.1)
    started  = time.monotonic()
    while True:
        diag = vision_state.diagnostics()
        info_ok    = bool(diag['info_seen'])
        frames_ok  = int(diag['image_frames']) >= 5
        if require_detection:
            det_ok = vision_state.is_fresh(stale_after)
        else:
            det_ok = True

        if info_ok and frames_ok and det_ok:
            elapsed = time.monotonic() - started
            status = VisionStatus(
                image_hz=float('nan'), detection_hz=float('nan'),
                image_size=tuple(diag['image_size']),
                info_seen=info_ok, elapsed=elapsed)
            _PREFLIGHT_CACHE[cache_key] = status
            if log is not None:
                W, H = status.image_size
                log.info(
                    f"[VPRE ] state preflight READY  size={W}x{H}  "
                    f"frames={diag['image_frames']}  "
                    f"after {elapsed:.1f}s")
            return status

        if time.monotonic() >= deadline:
            missing = []
            if not info_ok:    missing.append('camera_info')
            if not frames_ok:  missing.append('image_raw frames')
            if require_detection and not det_ok:
                missing.append(f'fresh detection (<{stale_after:.2f}s)')
            raise VisionNotReadyError(
                f"vision state preflight failed: missing {', '.join(missing)} "
                f"after {timeout:.1f}s -- start the vision pipeline "
                f"(e.g. `ros2 launch duburi_vision webcam_demo.launch.py`)")

        time.sleep(0.05)


def clear_cache(camera: Optional[str] = None) -> None:
    """Test/utility hook -- forget the cached preflight for one camera or all."""
    if camera is None:
        _PREFLIGHT_CACHE.clear()
        return
    _PREFLIGHT_CACHE.pop(camera, None)


# ---------------------------------------------------------------------- #
#  Internals                                                              #
# ---------------------------------------------------------------------- #

class _Counters:
    """Thread-safe message counter; reset on each window snapshot."""

    def __init__(self):
        self._lock         = threading.Lock()
        self._image_count  = 0
        self._det_count    = 0
        self.info_seen     = False
        self.image_size    = (0, 0)

    def bump_image(self) -> None:
        with self._lock:
            self._image_count += 1

    def bump_det(self) -> None:
        with self._lock:
            self._det_count += 1

    def set_info(self, msg: CameraInfo) -> None:
        with self._lock:
            self.info_seen  = True
            self.image_size = (int(msg.width), int(msg.height))

    def snapshot_hz(self, window_seconds: float):
        with self._lock:
            image_hz = self._image_count / window_seconds
            det_hz   = self._det_count   / window_seconds
            self._image_count = 0
            self._det_count   = 0
            return image_hz, det_hz


def _build_failure_reason(camera, counters, require_image_hz, require_detections):
    """Friendly one-liner naming the failing stage."""
    bits = []
    if not counters.info_seen:
        bits.append(f"camera_info silent on /duburi/vision/{camera}/camera_info")
    if not bits:
        bits.append(
            f"image_raw below {require_image_hz:.1f}Hz on "
            f"/duburi/vision/{camera}/image_raw")
    if require_detections:
        bits.append(f"detections silent on /duburi/vision/{camera}/detections")
    hint = (
        " -- start the vision pipeline first, e.g. "
        "`ros2 launch duburi_vision webcam_demo.launch.py camera:="
        f"{camera}`")
    return "vision preflight failed: " + " ; ".join(bits) + hint

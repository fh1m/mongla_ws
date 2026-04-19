"""VisionState -- per-camera subscriber that the vision motion loop reads.

One instance per camera. Owns three topic subscriptions and a small
thread-safe snapshot the closed-loop controller in
`duburi_control.motion_vision` reads each tick.

Why this lives in `duburi_manager`, not `duburi_vision`:
  * It's a manager-process resource (single MAVLink owner, single ROS
    node). Putting it here keeps `duburi_vision` free of any motion /
    control coupling.
  * `Duburi` consumes it via dependency injection (`vision_state_provider`)
    just like it consumes `yaw_source`, so the control package never
    imports rclpy directly.

Topics consumed (one camera example, `camera='laptop'`):
  /duburi/vision/laptop/detections   vision_msgs/Detection2DArray
  /duburi/vision/laptop/camera_info  sensor_msgs/CameraInfo
  /duburi/vision/laptop/image_raw    sensor_msgs/Image     (counted only)

Public surface (called from the control loop, never spinning):
  largest(class_name)        -> Detection2D | None
  bbox_error(class_name)     -> Sample | None  (ex, ey, h_frac, age_s)
  image_size()               -> (W, H) ints
  is_fresh(stale_after_s)    -> bool
  list_classes()             -> sorted list of class_id strings seen
  close()                    -> tear down subscriptions

Threading model:
  rclpy callbacks fire in the executor thread; the control loop runs
  inside the action callback (a different thread under
  MultiThreadedExecutor). Every read takes `_lock` for the brief moment
  it copies the latest msg pointer.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import List, Optional

from rclpy.node import Node
from rclpy.qos  import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2D, Detection2DArray


@dataclass
class Sample:
    """One snapshot of where the largest target sits in the frame.

    All values are normalized to [-1, +1] for ex/ey, [0, 1] for h_frac,
    so the controller math is camera-resolution-agnostic.
    """
    ex:     float    # horizontal error: -1=left edge, 0=centre, +1=right edge
    ey:     float    # vertical error:   -1=top  edge, 0=centre, +1=bottom edge
    h_frac: float    # bbox height as fraction of image height (0..1)
    w_frac: float    # bbox width  as fraction of image width  (0..1)
    age_s:  float    # how stale this detection is (monotonic seconds)
    class_id: str
    score:    float


class VisionState:
    """One camera's worth of subscribed-and-cached vision state."""

    def __init__(self, node: Node, *, camera: str = 'laptop',
                 default_image_size: tuple = (640, 480),
                 logger=None):
        self._node    = node
        self._camera  = camera
        self._log     = logger or node.get_logger()

        self._lock          = threading.Lock()
        self._latest_array: Optional[Detection2DArray] = None
        self._latest_stamp: float = 0.0           # monotonic seconds
        self._image_size:  tuple  = default_image_size
        self._info_seen:   bool   = False
        self._frames:      int    = 0             # image_raw counter (diag only)

        ns = f'/duburi/vision/{camera}'
        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE)
        self._sub_det  = node.create_subscription(
            Detection2DArray, f'{ns}/detections',  self._on_detections, qos)
        self._sub_info = node.create_subscription(
            CameraInfo,       f'{ns}/camera_info', self._on_info,       qos)
        self._sub_img  = node.create_subscription(
            Image,            f'{ns}/image_raw',   self._on_image,      qos)

        self._log.info(
            f"[VST  ] subscribed camera={camera!r} -> "
            f"{ns}/detections (+camera_info, +image_raw counter)")

    # ------------------------------------------------------------------ #
    #  Subscriber callbacks                                              #
    # ------------------------------------------------------------------ #
    def _on_detections(self, msg: Detection2DArray) -> None:
        with self._lock:
            self._latest_array = msg
            self._latest_stamp = time.monotonic()

    def _on_info(self, msg: CameraInfo) -> None:
        if msg.width and msg.height:
            with self._lock:
                self._image_size = (int(msg.width), int(msg.height))
                self._info_seen  = True

    def _on_image(self, _msg: Image) -> None:
        # Only used as a "is producer alive" pulse; we don't decode here.
        with self._lock:
            self._frames += 1

    # ------------------------------------------------------------------ #
    #  Read API used by the control loop                                 #
    # ------------------------------------------------------------------ #
    def image_size(self) -> tuple:
        with self._lock:
            return self._image_size

    def info_seen(self) -> bool:
        with self._lock:
            return self._info_seen

    def is_fresh(self, stale_after: float) -> bool:
        with self._lock:
            if self._latest_array is None:
                return False
            return (time.monotonic() - self._latest_stamp) <= stale_after

    def largest(self, class_name: str = '') -> Optional[Detection2D]:
        """Return the largest-area detection matching `class_name`, or None.

        Empty `class_name` matches anything (handy for ad-hoc CLI checks).
        """
        with self._lock:
            detections_array = self._latest_array
        if detections_array is None:
            return None

        best_detection: Optional[Detection2D] = None
        best_area:      float = 0.0
        for detection in detections_array.detections:
            if class_name and not _hypothesis_matches(detection, class_name):
                continue
            area = float(detection.bbox.size_x) * float(detection.bbox.size_y)
            if area > best_area:
                best_area      = area
                best_detection = detection
        return best_detection

    def bbox_error(self, class_name: str = '') -> Optional[Sample]:
        """Pick the largest matching detection and return a normalized Sample.

        Returns None when no matching detection is cached. The control
        loop treats None as "stale" and applies its `on_lost` policy.
        """
        detection = self.largest(class_name)
        if detection is None:
            return None
        with self._lock:
            image_width, image_height = self._image_size
            sampled_at_monotonic      = self._latest_stamp
        if image_width <= 0 or image_height <= 0:
            return None

        center_x, center_y = _bbox_center(detection.bbox)
        bbox_height_frac   = float(detection.bbox.size_y) / float(image_height)
        bbox_width_frac    = float(detection.bbox.size_x) / float(image_width)

        # Normalize to [-1, +1]. center_y > height/2 (target lower in image)
        # -> positive vertical_error, consistent with image-coordinates
        # (y grows downward).
        horizontal_error = (center_x - image_width  * 0.5) / (image_width  * 0.5)
        vertical_error   = (center_y - image_height * 0.5) / (image_height * 0.5)

        # Clamp to [-1.5, +1.5] so a bbox that drifts outside the frame
        # doesn't spike the controller. (Real bboxes can extend slightly
        # past the image edge after NMS.)
        horizontal_error = max(-1.5, min(1.5, horizontal_error))
        vertical_error   = max(-1.5, min(1.5, vertical_error))

        class_id = _hypothesis_class_id(detection)
        score    = _hypothesis_score(detection)
        return Sample(ex=horizontal_error, ey=vertical_error,
                      h_frac=bbox_height_frac, w_frac=bbox_width_frac,
                      age_s=time.monotonic() - sampled_at_monotonic,
                      class_id=class_id, score=score)

    def list_classes(self) -> List[str]:
        """Sorted list of distinct class_id strings in the latest array."""
        with self._lock:
            detections_array = self._latest_array
        if detections_array is None:
            return []
        seen_class_ids = set()
        for detection in detections_array.detections:
            class_id = _hypothesis_class_id(detection)
            if class_id:
                seen_class_ids.add(class_id)
        return sorted(seen_class_ids)

    def diagnostics(self) -> dict:
        """Snapshot for the [STATE] / [VST  ] log line."""
        with self._lock:
            return {
                'camera':       self._camera,
                'image_size':   self._image_size,
                'info_seen':    self._info_seen,
                'image_frames': self._frames,
                'last_age_s':   (time.monotonic() - self._latest_stamp
                                 if self._latest_array is not None
                                 else float('inf')),
            }

    # ------------------------------------------------------------------ #
    #  Lifecycle                                                          #
    # ------------------------------------------------------------------ #
    def close(self) -> None:
        try:
            self._node.destroy_subscription(self._sub_det)
            self._node.destroy_subscription(self._sub_info)
            self._node.destroy_subscription(self._sub_img)
        except Exception as exc:
            self._log.debug(f"[VST  ] close() ignored: {exc!r}")


# ---------------------------------------------------------------------- #
#  vision_msgs layout helpers (Humble vs Iron+)                          #
# ---------------------------------------------------------------------- #
def _bbox_center(bbox):
    centre = bbox.center
    if hasattr(centre, 'position'):       # Iron+: Pose2D w/ Point2D
        return float(centre.position.x), float(centre.position.y)
    return float(centre.x), float(centre.y)   # Humble: flat Pose2D


def _hypothesis_class_id(det: Detection2D) -> str:
    if not det.results:
        return ''
    hyp = det.results[0]
    if hasattr(hyp, 'hypothesis') and hasattr(hyp.hypothesis, 'class_id'):
        return str(hyp.hypothesis.class_id)
    if hasattr(hyp, 'id'):
        return str(hyp.id)
    return ''


def _hypothesis_score(det: Detection2D) -> float:
    if not det.results:
        return 0.0
    hyp = det.results[0]
    if hasattr(hyp, 'hypothesis') and hasattr(hyp.hypothesis, 'score'):
        return float(hyp.hypothesis.score)
    if hasattr(hyp, 'score'):
        return float(hyp.score)
    return 0.0


def _hypothesis_matches(det: Detection2D, class_name: str) -> bool:
    return _hypothesis_class_id(det).strip().lower() == class_name.strip().lower()

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
  /duburi/vision/laptop/tracks       vision_msgs/Detection2DArray  (coast only)
  /duburi/vision/laptop/camera_info  sensor_msgs/CameraInfo
  /duburi/vision/laptop/vis_range    std_msgs/Float32MultiArray

NOT consumed: `image_raw`. The control host does not decode pixels, and a
subscription costs a full-frame deserialisation per message on the machine
that has to answer a 50 Hz loop. Anything that needs frames -- the HUD, the
console, a recorder -- subscribes on its own.

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

import math
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CameraInfo

from duburi_vision import qos
from vision_msgs.msg import Detection2D, Detection2DArray


@dataclass
class Sample:
    """One snapshot of where the largest target sits in the frame.

    All values are normalized to [-1, +1] for ex/ey, [0, 1] for h_frac,
    so the controller math is camera-resolution-agnostic. The control
    loop reads /detections directly (the same topic the HUD shows), so a
    box visible on screen is a box the controller acts on.
    """
    ex:       float    # horizontal error: -1=left edge, 0=centre, +1=right edge
    ey:       float    # vertical error:   -1=top  edge, 0=centre, +1=bottom edge
    h_frac:   float    # bbox height as fraction of image height (0..1)
    w_frac:   float    # bbox width  as fraction of image width  (0..1)
    age_s:    float    # time since the last REAL detection of this target (s)
    class_id: str
    score:     float
    vis_range: float = 0.0         # monocular depth estimate from depth_estimation_node (0=far, 1=close)
    track_id:  int   = -1          # tracker id of this target (-1 = unknown / coast off)
    coasted:   bool  = False       # True = tracker-predicted box during a detection gap (no live box)


class VisionState:
    """One camera's worth of subscribed-and-cached vision state.

    The control loop always reads ``/duburi/vision/<cam>/detections`` --
    the raw detector output, the exact topic the operator HUD overlays.
    The tracker node still runs for the display but is no longer in the
    control path, so a detection on screen is one the controller sees.
    """

    def __init__(self, node: Node, *, camera: str = 'laptop',
                 default_image_size: tuple = (0, 0),
                 logger=None):
        self._node    = node
        self._camera  = camera
        self._log     = logger or node.get_logger()

        self._lock          = threading.Lock()
        self._latest_array: Optional[Detection2DArray] = None
        self._latest_stamp: float = 0.0           # monotonic seconds
        self._image_size:  tuple  = default_image_size
        # CameraInfo K/D, kept rather than discarded -- see _on_info.
        self._K = None
        self._D = None
        self._vis_range_vals: list = []            # parallel to _latest_array.detections
        self._info_seen:   bool   = False
        # Detection message counter. This used to count `image_raw`, which
        # meant the CONTROL HOST subscribed to the full 691 kB frame stream
        # to maintain a diagnostic integer -- and it did not even work: the
        # subscription was RELIABLE against a BEST_EFFORT publisher, so it
        # received NOTHING and the counter sat at 0 for ever. See the class
        # docstring; `preflight.wait_vision_state_ready` gated on it.
        self._det_msgs:    int    = 0
        # Coast layer (opt-in, used only when bbox_error(coast_s>0)): the /tracks
        # topic is the coast SOURCE; /detections stays the authoritative primary.
        self._latest_tracks: Optional[Detection2DArray] = None
        self._last_real: dict = {}                # track_id -> (monotonic_t, score) of last REAL detection

        ns = f'/duburi/vision/{camera}'
        # From duburi_vision.qos -- the same objects the PUBLISHERS use, so the
        # two ends of each link cannot drift apart. They already did once: this
        # class asked for RELIABLE on a BEST_EFFORT image topic and received
        # nothing for ever, silently.
        self._sub_det   = node.create_subscription(
            Detection2DArray, f'{ns}/detections',   self._on_detections,
            qos.DETECTIONS)
        # Coast source. Cheap to subscribe; only CONSULTED when coast_s>0, so a
        # mission that never sets vision.coast_s behaves exactly as before. If
        # the tracker node isn't running, this simply never delivers and coast
        # silently never engages (degrades to raw-/detections behaviour).
        self._sub_trk   = node.create_subscription(
            Detection2DArray, f'{ns}/tracks',        self._on_tracks,
            qos.DETECTIONS)
        self._sub_info  = node.create_subscription(
            CameraInfo,       f'{ns}/camera_info',   self._on_info,
            qos.CAMERA_INFO)
        self._sub_vr    = node.create_subscription(
            Float32MultiArray, f'{ns}/vis_range',    self._on_vis_range,  10)

        self._log.info(
            f"[VST  ] subscribed camera={camera!r} -> "
            f"{ns}/detections (+tracks, +camera_info, +vis_range). "
            f"NOT image_raw -- the control host has no use for pixels.")

    # ------------------------------------------------------------------ #
    #  Subscriber callbacks                                              #
    # ------------------------------------------------------------------ #
    def _on_detections(self, msg: Detection2DArray) -> None:
        with self._lock:
            self._latest_array = msg
            self._latest_stamp = time.monotonic()
            self._det_msgs += 1

    def _on_tracks(self, msg: Detection2DArray) -> None:
        with self._lock:
            self._latest_tracks = msg

    def _on_info(self, msg: CameraInfo) -> None:
        if msg.width and msg.height:
            with self._lock:
                self._image_size = (int(msg.width), int(msg.height))
                self._info_seen  = True
                # K and D were being received and thrown away. They are what
                # turns a pixel error into a BEARING -- i.e. what gives a
                # control gain units of thrust-per-radian instead of
                # thrust-per-whatever-this-camera-happens-to-be. camera_node
                # rescales K to the streamed resolution before publishing, so
                # this is already correct for the frames the detector saw.
                self._K = list(msg.k) if len(msg.k) >= 9 else None
                self._D = list(msg.d) if msg.d is not None else None

    def calibration(self):
        """(K, D) as published, or (None, None).

        `CameraInfo.k` is all zeros until a calibration file is loaded, so a
        caller must test fx > 0 rather than `k is not None`. `bearing.py` does
        exactly that, falls back to an FOV, and reports which it used.
        """
        with self._lock:
            return (list(self._K) if self._K else None,
                    list(self._D) if self._D else None)

    def _on_vis_range(self, msg: Float32MultiArray) -> None:
        with self._lock:
            self._vis_range_vals = list(msg.data)

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

    def bbox_error(self, class_name: str = '', *,
                   near: Optional[Tuple[float, float]] = None,
                   gate_norm: float = 0.0,
                   min_score: float = 0.0,
                   locked_id: int = -1,
                   coast_s: float = 0.0) -> Optional[Sample]:
        """Pick a matching detection and return a normalized Sample.

        Default (``near=None`` or ``gate_norm<=0``): the LARGEST-area matching
        box, exactly as before. With ``near=(ex,ey)`` and ``gate_norm>0``
        (the continuity lock): among matching boxes within ``gate_norm`` of
        ``near`` in normalized centre space, the one NEAREST ``near`` -- so a
        second hole / spurious box can't steal the aim once a target is locked;
        if none are inside the gate, returns None (a transient loss the control
        loop rides on its grace timer). ``min_score`` drops boxes below that
        detection score from consideration (control-side conf floor).

        Coast layer (OPT-IN, ``coast_s>0``): when NO live ``/detections`` box
        matches, fall back to the tracker's coasted (Kalman-predicted) box of
        ``locked_id`` from ``/tracks`` -- but ONLY that id, and ONLY while the
        gap is shorter than ``coast_s``. The returned Sample carries
        ``coasted=True`` and ``age_s`` = the TRUE time since the last real
        detection (not message age), so the control loop's freshness/coast
        decay reduces its authority and the grace timer still fires LOST. A
        live detection ALWAYS wins (this method tries it first); coast never
        gates out or overrides a real box. ``coast_s=0`` (default) ⇒ behaviour
        is byte-identical to the no-coast path. See the prior-bug note in
        known-issues.md (predicted boxes must not be conf-gated as the locked id).

        Returns None when no qualifying detection is cached, or before the
        first CameraInfo arrives (image size still (0,0)) so the control
        loop never steers on a mis-scaled pixel error.
        """
        with self._lock:
            detections_array     = self._latest_array
            image_width, image_height = self._image_size
            sampled_at_monotonic = self._latest_stamp
            vis_range_vals       = self._vis_range_vals
            tracks_array         = self._latest_tracks

        if detections_array is None:
            return None
        if image_width <= 0 or image_height <= 0:
            return None

        half_w = image_width  * 0.5
        half_h = image_height * 0.5
        use_near = near is not None and gate_norm > 0.0

        # One pass: per candidate compute a selection metric -- nearest-to-`near`
        # (continuity lock) or largest-area (default). Track the winner's index
        # for the parallel vis_range lookup.
        best_detection: Optional[Detection2D] = None
        best_metric:    Optional[float] = None
        best_index:     int   = 0
        for idx, det in enumerate(detections_array.detections):
            if class_name and not _hypothesis_matches(det, class_name):
                continue
            if min_score > 0.0 and _hypothesis_score(det) < min_score:
                continue
            if use_near:
                cx, cy = _bbox_center(det.bbox)
                ex = (cx - half_w) / half_w
                ey = (cy - half_h) / half_h
                dist = math.hypot(ex - near[0], ey - near[1])
                if dist > gate_norm:
                    continue
                metric = -dist                      # nearest wins
            else:
                metric = float(det.bbox.size_x) * float(det.bbox.size_y)  # largest
            if best_metric is None or metric > best_metric:
                best_metric    = metric
                best_detection = det
                best_index     = idx
        if best_detection is None:
            # No live detection this tick. Coast the locked target's predicted
            # box (opt-in) before declaring a loss -- the gap-bridging path.
            if coast_s > 0.0 and locked_id >= 0:
                return self._coast_sample(class_name, locked_id, coast_s,
                                          tracks_array, image_width, image_height)
            return None

        vis_range = (float(vis_range_vals[best_index])
                     if best_index < len(vis_range_vals) else 0.0)

        detection = best_detection
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

        class_id  = _hypothesis_class_id(detection)
        score     = _hypothesis_score(detection)

        # Coast bookkeeping (only when enabled): tag this live box with its
        # tracker id (matched from /tracks by centre) and record the real-sighting
        # time so a later coast knows the true gap. Pure no-op when coast_s=0.
        track_id = -1
        if coast_s > 0.0:
            track_id = self._match_track_id(
                horizontal_error, vertical_error, tracks_array,
                image_width, image_height)
            if track_id >= 0:
                with self._lock:
                    self._last_real[track_id] = (time.monotonic(), score)

        return Sample(ex=horizontal_error, ey=vertical_error,
                      h_frac=bbox_height_frac, w_frac=bbox_width_frac,
                      age_s=time.monotonic() - sampled_at_monotonic,
                      class_id=class_id, score=score, vis_range=vis_range,
                      track_id=track_id, coasted=False)

    # ------------------------------------------------------------------ #
    #  Coast layer helpers (used only when bbox_error(coast_s>0))         #
    # ------------------------------------------------------------------ #
    _MATCH_GATE_NORM = 0.20   # max normalized centre distance to call a /tracks box "the same"

    def _match_track_id(self, ex: float, ey: float, tracks_array,
                        image_width: int, image_height: int) -> int:
        """Tracker id of the REAL /tracks box nearest the live detection at
        (ex, ey) normalized centre, within a small gate. -1 if none / no tracks."""
        if tracks_array is None or image_width <= 0 or image_height <= 0:
            return -1
        half_w, half_h = image_width * 0.5, image_height * 0.5
        best_id, best_dist = -1, self._MATCH_GATE_NORM
        for det in tracks_array.detections:
            if _track_is_predicted(det):
                continue                      # match against real boxes only
            tid = _track_id_of(det)
            if tid < 0:
                continue
            cx, cy = _bbox_center(det.bbox)
            d = math.hypot((cx - half_w) / half_w - ex, (cy - half_h) / half_h - ey)
            if d < best_dist:
                best_dist, best_id = d, tid
        return best_id

    def _coast_sample(self, class_name: str, locked_id: int, coast_s: float,
                      tracks_array, image_width: int,
                      image_height: int) -> Optional[Sample]:
        """Build a Sample from the coasted (predicted) /tracks box of locked_id.

        Returns None if: no tracks, the locked id has no predicted box this
        tick, the id was never seen as a real detection, or the gap already
        exceeds coast_s (-> caller treats as a loss). The Sample is conf-exempt
        by construction (it is not run through min_score) and carries the TRUE
        detection-age so downstream authority decays."""
        if tracks_array is None or image_width <= 0 or image_height <= 0:
            return None
        with self._lock:
            last = self._last_real.get(locked_id)
        if last is None:
            return None                        # never had a real sighting -> don't invent one
        last_t, last_score = last
        age = time.monotonic() - last_t
        if age > coast_s:
            return None                        # coast window elapsed -> loss

        for det in tracks_array.detections:
            if _track_id_of(det) != locked_id or not _track_is_predicted(det):
                continue
            if class_name and not _hypothesis_matches(det, class_name) \
                    and _hypothesis_class_id(det):
                continue
            cx, cy = _bbox_center(det.bbox)
            ex = max(-1.5, min(1.5, (cx - image_width * 0.5) / (image_width * 0.5)))
            ey = max(-1.5, min(1.5, (cy - image_height * 0.5) / (image_height * 0.5)))
            return Sample(
                ex=ex, ey=ey,
                h_frac=float(det.bbox.size_y) / float(image_height),
                w_frac=float(det.bbox.size_x) / float(image_width),
                age_s=age,                     # true time since last real detection
                class_id=class_name or _hypothesis_class_id(det),
                score=last_score,              # last real score (conf-exempt for the locked id)
                vis_range=0.0, track_id=locked_id, coasted=True)
        return None

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
                'det_msgs':     self._det_msgs,
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
            self._node.destroy_subscription(self._sub_trk)
            self._node.destroy_subscription(self._sub_info)
            self._node.destroy_subscription(self._sub_img)
            self._node.destroy_subscription(self._sub_vr)
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


def _track_id_of(det: Detection2D) -> int:
    """tracker id carried on a /tracks Detection2D (`det.id` = str(track_id)).
    -1 for a raw /detections box (no id set) or a malformed value."""
    raw = getattr(det, 'id', '')
    try:
        return int(raw)
    except (TypeError, ValueError):
        return -1


def _track_is_predicted(det: Detection2D) -> bool:
    """True for a coasted (Kalman-predicted) /tracks box -- the tracker forces
    its hypothesis score to 0.0; a real box keeps the detector confidence."""
    return _hypothesis_score(det) <= 0.0

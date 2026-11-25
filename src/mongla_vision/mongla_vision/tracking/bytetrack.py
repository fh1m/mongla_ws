"""ByteTrack wrapper using supervision.ByteTrack.

Converts between the internal List[Detection] format and supervision's
numpy-backed sv.Detections. Class labels are preserved per-track by
keeping a dict[track_id -> class_name] that is updated each frame a
real detection confirms the track.

Install: pip install supervision
"""

from __future__ import annotations

from typing import Dict, List

import numpy as np

from mongla_vision.detection.detector import Detection
from .tracker import Tracker, TrackedDetection


class ByteTrackWrapper(Tracker):
    """supervision.ByteTrack wrapper.

    Parameters
    ----------
    track_buffer : int
        A lost track's life, written as 30 Hz frames and enforced in seconds
        of capture time: 150 -> 5.0 s at any detection rate.
    min_hits : int
        Frames a track must be continuously detected before it is
        published (filters one-shot false positives).
    iou_threshold : float
        Minimum IoU (weighted by the detection's score, as supervision
        fuses them) for a detection to continue a track. Default 0.3.
    track_activation_threshold : float
        Minimum detection score to immediately activate a new track.
        Higher values suppress spurious IDs from low-confidence detections.
    """

    name = 'bytetrack'

    def __init__(self, *,
                 track_buffer: int = 30,
                 min_hits: int = 3,
                 iou_threshold: float = 0.3,
                 track_activation_threshold: float = 0.25,
                 detector_conf: float = 0.0):
        try:
            import supervision as sv
        except ImportError as exc:
            raise ImportError(
                "supervision is required. Install with: pip install supervision"
            ) from exc

        # As in RoboflowTracker: a box the detector chose to publish must be
        # able to start a track, so the activation bar never sits above the
        # detector's own floor (vl_15 F-02).
        # supervision starts a NEW track only at score >= threshold + 0.1
        # (`det_thresh`), so the clamp sits that margin below the floor.
        if detector_conf > 0.0:
            track_activation_threshold = min(track_activation_threshold,
                                             max(float(detector_conf) - 0.1, 0.0))
        # The lost window in SECONDS of capture time, written as 30 Hz frames
        # like the Roboflow path. supervision counts update calls, which is
        # 1.5 s at 99 Hz and 15 s at 10 Hz for the same 150 (vl_15 F-01).
        self.coast_max_s = int(track_buffer) / 30.0
        self._last_seen: Dict[int, float] = {}
        self._sv = sv
        # To swap to OC-SORT at pool day: replace sv.ByteTrack with sv.OCSORT (same API).
        self._bt = sv.ByteTrack(
            lost_track_buffer=track_buffer,
            minimum_consecutive_frames=min_hits,
            # supervision's threshold is a MAXIMUM COST, cost = 1 - IoU * score;
            # ours is a minimum IoU, as the Roboflow path takes it. Passed
            # straight through, 0.2 demanded IoU * score above 0.8 and every
            # moving target fragmented on the fallback engine.
            minimum_matching_threshold=1.0 - float(iou_threshold),
            track_activation_threshold=track_activation_threshold,
        )

        # class_name registry: track_id -> class_name from last real detection
        self._class_map: Dict[int, tuple] = {}     # track id -> (class id, name)

    def update(self, detections: List[Detection],
               frame_t: float) -> List[TrackedDetection]:
        if not detections:
            sv_dets = self._sv.Detections.empty()
        else:
            xyxy       = np.array([d.xyxy              for d in detections], dtype=np.float32)
            confidence = np.array([d.score             for d in detections], dtype=np.float32)
            class_ids  = np.array([d.class_id          for d in detections], dtype=int)
            sv_dets = self._sv.Detections(
                xyxy=xyxy,
                confidence=confidence,
                class_id=class_ids,
            )

        tracked = self._bt.update_with_detections(sv_dets)

        results: List[TrackedDetection] = []

        # -- Confirmed matches from this frame --------------------------------
        if tracked.tracker_id is not None and len(tracked) > 0:
            for i in range(len(tracked)):
                tid        = int(tracked.tracker_id[i])
                class_id   = int(tracked.class_id[i])   if tracked.class_id   is not None else 0
                confidence = float(tracked.confidence[i]) if tracked.confidence is not None else 0.0
                x1, y1, x2, y2 = (float(v) for v in tracked.xyxy[i])

                if confidence > 0.0:
                    name = _class_name_from_id(class_id, detections)
                    self._class_map[tid] = (class_id, name)
                    self._last_seen[tid] = float(frame_t)
                class_name = self._class_map.get(tid, (class_id, str(class_id)))[1]

                results.append(TrackedDetection(
                    class_id=class_id,
                    class_name=class_name,
                    score=confidence,
                    xyxy=(x1, y1, x2, y2),
                    track_id=tid,
                    predicted=False,
                ))

        # -- Lost-but-buffered tracks (occlusion prediction) -----------------
        # supervision's ByteTrack keeps lost tracks in `lost_tracks` until
        # track_buffer expires. Emit them as predicted=True so our Kalman
        # smoother can keep predicting forward and the control loop doesn't
        # see a gap in the detection stream.
        confirmed_ids = {td.track_id for td in results}
        for strack in getattr(self._bt, 'lost_tracks', []):
            tid = int(getattr(strack, 'external_track_id',
                              getattr(strack, 'track_id', -1)))
            if tid in confirmed_ids:
                continue
            class_id, class_name = self._class_map.get(tid, (0, ''))
            if not class_name:
                continue   # never got a real detection; skip
            last = self._last_seen.get(tid)
            if last is None or float(frame_t) - last > self.coast_max_s:
                continue   # past the window in seconds, whatever the call count
            # Use last known bbox from the STrack (Kalman state in ByteTrack).
            tlbr = getattr(strack, 'tlbr', None)
            if tlbr is None:
                continue
            x1, y1, x2, y2 = (float(v) for v in tlbr)
            results.append(TrackedDetection(
                class_id=class_id,     # its own class, not 0, while it coasts
                class_name=class_name,
                score=0.0,
                xyxy=(x1, y1, x2, y2),
                track_id=tid,
                predicted=True,
            ))

        # Prune _class_map entries for tracks that ByteTrack has fully expired
        # (neither confirmed nor in lost_tracks). Without pruning, recycled track
        # IDs inherit the old class name and targets are silently misidentified.
        live_ids: set[int] = {td.track_id for td in results}
        live_ids.update(
            int(getattr(st, 'external_track_id', getattr(st, 'track_id', -1)))
            for st in getattr(self._bt, 'lost_tracks', [])
        )
        stale = [k for k in self._class_map if k not in live_ids]
        for k in stale:
            del self._class_map[k]
            self._last_seen.pop(k, None)

        return results

    def reset(self) -> None:
        self._bt.reset()
        self._class_map.clear()
        self._last_seen.clear()


def _class_name_from_id(class_id: int,
                         detections: List[Detection]) -> str:
    """Return the class_name for the first Detection with matching class_id."""
    for d in detections:
        if d.class_id == class_id:
            return d.class_name
    return str(class_id)

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

from duburi_vision.detection.detector import Detection
from .tracker import Tracker, TrackedDetection


class ByteTrackWrapper(Tracker):
    """supervision.ByteTrack wrapper.

    Parameters
    ----------
    track_buffer : int
        Frames to keep a lost track alive (ByteTrack's `lost_track_buffer`).
        Default 30 frames at 20 Hz ≈ 1.5 s of occlusion tolerance.
    min_hits : int
        Frames a track must be continuously detected before it is
        published (filters one-shot false positives).
    iou_threshold : float
        IoU threshold for the second association pass (low-confidence
        detections). Default 0.3.
    """

    name = 'bytetrack'

    def __init__(self, *,
                 track_buffer: int = 30,
                 min_hits: int = 3,
                 iou_threshold: float = 0.3):
        try:
            import supervision as sv
        except ImportError as exc:
            raise ImportError(
                "supervision is required for ByteTrackWrapper. "
                "Install it with: pip install supervision"
            ) from exc

        self._sv = sv
        self._bt = sv.ByteTrack(
            lost_track_buffer=track_buffer,
            minimum_consecutive_frames=min_hits,
            minimum_matching_threshold=iou_threshold,
        )
        self._track_buffer = track_buffer
        self._min_hits     = min_hits
        self._iou_threshold = iou_threshold

        # class_name registry: track_id -> class_name from last real detection
        self._class_map: Dict[int, str] = {}

    def update(self, detections: List[Detection],
               frame_t: float) -> List[TrackedDetection]:  # noqa: ARG002
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
                    self._class_map[tid] = name
                class_name = self._class_map.get(tid, str(class_id))

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
            class_name = self._class_map.get(tid, '')
            if not class_name:
                continue   # never got a real detection; skip
            # Use last known bbox from the STrack (Kalman state in ByteTrack).
            tlbr = getattr(strack, 'tlbr', None)
            if tlbr is None:
                continue
            x1, y1, x2, y2 = (float(v) for v in tlbr)
            results.append(TrackedDetection(
                class_id=0,
                class_name=class_name,
                score=0.0,
                xyxy=(x1, y1, x2, y2),
                track_id=tid,
                predicted=True,
            ))

        return results

    def reset(self) -> None:
        self._bt.reset()
        self._class_map.clear()


def _class_name_from_id(class_id: int,
                         detections: List[Detection]) -> str:
    """Return the class_name for the first Detection with matching class_id."""
    for d in detections:
        if d.class_id == class_id:
            return d.class_name
    return str(class_id)

"""Roboflow `trackers` backend (OC-SORT / ByteTrack) behind the Tracker ABC.

Drop-in replacement for `ByteTrackWrapper` that uses the Roboflow `trackers`
library (https://github.com/roboflow/trackers) instead of supervision's
ByteTrack. Same contract: consume `List[Detection]`, return
`List[TrackedDetection]` with stable `track_id` and the `predicted` flag
(predicted=True, score=0.0 for a coasted/occluded box).

Why this exists
---------------
`trackers` ships clean-room OC-SORT and ByteTrack with two-stage high/low
confidence association and per-track Kalman state estimators. OC-SORT
(observation-centric, center-based XCYCSR estimator) recovers from short
dropouts better than plain SORT, which is exactly the failure that loses a
torpedo-hole lock or drifts the hull off a slalom pipe.

The coasting mechanism (verified empirically)
---------------------------------------------
`tracker.update(dets)` returns ONLY the input detections annotated with IDs.
A track that had NO detection this frame is NOT in that return -- its
Kalman-predicted box lives in `tracker.tracked_objects` (a list of length-1
`sv.Detections`). So each frame we:

  1. emit the matched real boxes from `update()`        -> predicted=False
  2. emit every alive track in `tracked_objects` whose  -> predicted=True,
     id was NOT matched this frame (the coasted boxes)     score=0.0

The library owns the drop: a coasted track leaves `tracked_objects` after
`int(frame_rate / 30 * lost_track_buffer)` frames, so size `track_buffer`
(and `frame_rate`) for the wall-time coast window you need.

Class names + last real score for a coasted box come from a per-track
registry (the coasted `Detections` carries no class data), mirroring
`ByteTrackWrapper._class_map`.

Install: pip install trackers   (pulls supervision, already a dependency)
"""

from __future__ import annotations

from typing import Dict, List, Tuple

import numpy as np

from mongla_vision.detection.detector import Detection
from .tracker import Tracker, TrackedDetection

# tracker_type -> (class_name, accepts_track_activation_threshold)
_TRACKER_TYPES = ('ocsort', 'bytetrack')


class RoboflowTracker(Tracker):
    """Roboflow `trackers` OC-SORT / ByteTrack wrapper.

    Parameters
    ----------
    tracker_type : str
        'ocsort' (default, best dropout recovery) or 'bytetrack'.
    track_buffer : int
        `lost_track_buffer` -- frames a track survives with no detection.
        NOTE the library scales this by frame_rate: the real coast window is
        ``int(frame_rate / 30 * track_buffer)`` frames. Size it so the coast
        window in seconds exceeds the control loop's ``coast_s``.
    frame_rate : float
        Detection rate (Hz). Used by the library to scale the buffer and by
        OC-SORT's velocity model. Pass the detector's real publish rate.
    min_hits : int
        `minimum_consecutive_frames` before a track gets a real id (filters
        one-shot false positives). Lower = faster acquisition, more flicker.
    iou_threshold : float
        `minimum_iou_threshold` for association.
    track_activation_threshold : float
        ByteTrack only -- min score to spawn a NEW id. (OC-SORT ignores it.)
    high_conf_det_threshold : float
        Two-stage split: detections at/above this are stage-1 (high conf),
        below are stage-2 (kept for association so weak reals still maintain
        a track without spawning a new id).
    """

    name = 'roboflow'

    def __init__(self, *,
                 tracker_type: str = 'ocsort',
                 track_buffer: int = 30,
                 frame_rate: float = 20.0,
                 min_hits: int = 2,
                 iou_threshold: float = 0.3,
                 track_activation_threshold: float = 0.25,
                 high_conf_det_threshold: float = 0.6,
                 detector_conf: float = 0.0):
        """`detector_conf` is the floor the DETECTOR is publishing at.

        THE TRACKER'S CONFIDENCE GATES MUST NOT EXCEED IT, and until this was
        measured they did, by 4x, which meant the tracker emitted NOTHING on
        real underwater footage.

        `high_conf_det_threshold` gates TRACK CREATION: below it, no track is
        ever started, so there is nothing to coast on and `/tracks` is empty.
        The shipped 0.6 was inherited from pedestrian benchmarks where scores
        run high. Measured on real RoboSub 2025 footage with the model trained
        on it -- 319 detections over the gate approach:

            score p10 0.167  p50 0.258  p90 0.439  max 0.640
            fraction at or above 0.6:  0.6 %

            high_conf_det_threshold   tracker presence
                    0.60                  0.0 %     <- SHIPPED
                    0.40                 13.9 %
                    0.25                 17.0 %
                    0.15                 45.1 %

        The detector's own presence on that footage is 15.5 %. At 0.15 the
        coast layer nearly TRIPLES it -- that is the gap-bridging working. At
        0.6 the entire tracking stack, the Kalman smoother, `vision.coast_s`
        and the continuity lock were dead: no tracks, nothing to coast, and
        no error anywhere, because a tracker with no tracks still publishes an
        empty array and looks healthy.

        Clamping to `detector_conf` is structural, not tuning. A detection the
        DETECTOR chose to publish must be allowed to start a track; anything
        else silently discards work the chip already did. The caller passes
        the detector's live `conf`, so the two move together and cannot drift
        apart again.
        """
        if detector_conf > 0.0:
            high_conf_det_threshold = min(high_conf_det_threshold,
                                          float(detector_conf))
            track_activation_threshold = min(track_activation_threshold,
                                             float(detector_conf))
        try:
            import supervision as sv
            import trackers as tr
        except ImportError as exc:  # pragma: no cover - env guard
            raise ImportError(
                "roboflow trackers is required. Install: pip install trackers"
            ) from exc

        ttype = str(tracker_type).strip().lower()
        if ttype not in _TRACKER_TYPES:
            raise ValueError(
                f"tracker_type must be one of {_TRACKER_TYPES}, got {tracker_type!r}")

        self._sv = sv
        self.name = ttype

        if ttype == 'ocsort':
            # OC-SORT: observation-centric, center-based XCYCSR estimator.
            # No track_activation_threshold knob -- high_conf_det_threshold
            # gates the two-stage split.
            self._t = tr.OCSORTTracker(
                lost_track_buffer=int(track_buffer),
                frame_rate=float(frame_rate),
                minimum_consecutive_frames=int(min_hits),
                minimum_iou_threshold=float(iou_threshold),
                high_conf_det_threshold=float(high_conf_det_threshold),
            )
        else:
            self._t = tr.ByteTrackTracker(
                lost_track_buffer=int(track_buffer),
                frame_rate=float(frame_rate),
                track_activation_threshold=float(track_activation_threshold),
                minimum_consecutive_frames=int(min_hits),
                minimum_iou_threshold=float(iou_threshold),
                high_conf_det_threshold=float(high_conf_det_threshold),
            )

        self._track_buffer = int(track_buffer)
        self._frame_rate   = float(frame_rate)
        # track_id -> (class_id, class_name, last_real_score) from the last
        # frame a real detection confirmed the track. Coasted boxes read it.
        self._meta: Dict[int, Tuple[int, str, float]] = {}

    # ------------------------------------------------------------------ #
    def update(self, detections: List[Detection],
               frame_t: float) -> List[TrackedDetection]:  # noqa: ARG002
        sv_dets = self._to_sv(detections)
        out = self._t.update(sv_dets)

        results: List[TrackedDetection] = []
        matched: set[int] = set()

        # -- Matched real detections this frame (id >= 0 only) ---------------
        # id == -1 is an immature track (min_hits not reached) -- omitted, per
        # the ABC contract. Control reads raw /detections, so a live box still
        # reaches it; /tracks only lags acquisition by (min_hits-1) frames.
        names = out.data.get('class_name') if out.data else None
        for i in range(len(out)):
            tid = int(out.tracker_id[i]) if out.tracker_id is not None else -1
            if tid < 0:
                continue
            box = tuple(float(v) for v in out.xyxy[i])
            cid = int(out.class_id[i]) if out.class_id is not None else 0
            score = float(out.confidence[i]) if out.confidence is not None else 0.0
            name = str(names[i]) if names is not None else _name_from(cid, detections)
            self._meta[tid] = (cid, name, score)
            matched.add(tid)
            results.append(TrackedDetection(
                class_id=cid, class_name=name, score=score,
                xyxy=box, track_id=tid, predicted=False))

        # -- Coasted tracks: alive in tracked_objects, NOT matched this frame -
        # tracked_objects is an sv.Detections (one row per alive track), so we
        # index it rather than iterate (iterating yields supervision's unpack
        # tuples). A row's Kalman-predicted box advances each coasted frame.
        alive = self._t.tracked_objects
        for k in range(len(alive)):
            if alive.tracker_id is None:
                break
            tid = int(alive.tracker_id[k])
            if tid < 0 or tid in matched:
                continue
            meta = self._meta.get(tid)
            if meta is None:
                continue  # never had a real detection -> nothing to coast
            cid, name, _ = meta
            box = tuple(float(v) for v in alive.xyxy[k])
            results.append(TrackedDetection(
                class_id=cid, class_name=name, score=0.0,
                xyxy=box, track_id=tid, predicted=True))

        self._prune(matched)
        return results

    def reset(self) -> None:
        self._t.reset()
        self._meta.clear()

    # ------------------------------------------------------------------ #
    def _to_sv(self, detections: List[Detection]):
        if not detections:
            return self._sv.Detections.empty()
        xyxy = np.array([d.xyxy for d in detections], dtype=np.float32)
        conf = np.array([d.score for d in detections], dtype=np.float32)
        cid  = np.array([d.class_id for d in detections], dtype=int)
        names = np.array([d.class_name for d in detections], dtype=object)
        # class_name in data so update() carries it back on matched boxes
        # (verified: the library preserves confidence/class_id/data).
        return self._sv.Detections(
            xyxy=xyxy, confidence=conf, class_id=cid,
            data={'class_name': names})

    def _prune(self, matched: set) -> None:
        """Drop registry entries for tracks the library has fully expired
        (recycled ids would otherwise inherit a stale class name)."""
        live = set(matched)
        alive = self._t.tracked_objects
        if alive.tracker_id is not None:
            for k in range(len(alive)):
                live.add(int(alive.tracker_id[k]))
        for k in [k for k in self._meta if k not in live]:
            del self._meta[k]


def _name_from(class_id: int, detections: List[Detection]) -> str:
    for d in detections:
        if d.class_id == class_id:
            return d.class_name
    return str(class_id)

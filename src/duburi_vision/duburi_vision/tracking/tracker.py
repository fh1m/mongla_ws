"""Tracker ABC and TrackedDetection dataclass.

Every tracker backend implements `Tracker`. The rest of the pipeline
only imports from this module — never from a specific backend — so
swapping ByteTrack for OC-SORT is a one-line change in tracker_node.

TrackedDetection
----------------
Extends `Detection` with a stable integer `track_id` and a `predicted`
flag that is True when the Kalman smoother is holding the track forward
without a real detector measurement. The control loop can use this to
distinguish "confident" from "interpolated" frames:

  predicted=False  real detection, score reflects YOLO confidence
  predicted=True   Kalman prediction only, score forced to 0.0

Downstream staleness logic in motion_vision.py uses `sample.age_s`:
  * age_s is time since last real measurement on this track
  * short occlusions ride through (age_s < stale_after)
  * long occlusions cross stale_after and fire on_lost naturally
No changes needed to the existing control loop for this to work.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

from duburi_vision.detection.detector import Detection


@dataclass
class TrackedDetection:
    """Detection enriched with a stable track_id from the tracker.

    Mirrors Detection fields so it can be used anywhere Detection is
    expected (duck-typed). track_id=-1 means "unconfirmed" (min_hits
    not yet reached); the publisher filters those out before publishing.
    """
    class_id:   int
    class_name: str
    score:      float
    xyxy:       tuple

    track_id:   int   = field(default=-1)
    predicted:  bool  = field(default=False)

    # ---- same convenience getters as Detection ----------------------------- #
    @property
    def width(self) -> float:
        return float(self.xyxy[2] - self.xyxy[0])

    @property
    def height(self) -> float:
        return float(self.xyxy[3] - self.xyxy[1])

    @property
    def area(self) -> float:
        return max(0.0, self.width) * max(0.0, self.height)

    @property
    def cx(self) -> float:
        return float(self.xyxy[0] + self.xyxy[2]) * 0.5

    @property
    def cy(self) -> float:
        return float(self.xyxy[1] + self.xyxy[3]) * 0.5

    @classmethod
    def from_detection(cls, d: Detection, track_id: int,
                       predicted: bool = False) -> 'TrackedDetection':
        return cls(
            class_id=d.class_id, class_name=d.class_name,
            score=d.score, xyxy=d.xyxy,
            track_id=track_id, predicted=predicted,
        )


class Tracker:
    """Abstract tracker.  update() is called once per frame."""

    name: str = 'base'

    def update(self, detections: List[Detection],
               frame_t: float) -> List[TrackedDetection]:
        """Run one tracker step.

        Parameters
        ----------
        detections : List[Detection]
            Raw detections from the detector for this frame. May be empty.
        frame_t : float
            Monotonic timestamp of this frame (seconds). Used by Kalman
            to compute dt; pass time.monotonic() from the node.

        Returns
        -------
        List[TrackedDetection]
            Active tracks after association. Includes confirmed tracks
            that are currently predicted (predicted=True, score=0.0).
            Unconfirmed tracks (min_hits not reached) are omitted.
        """
        raise NotImplementedError

    def reset(self) -> None:
        """Clear all track state (e.g. between goals or on reconnect)."""
        raise NotImplementedError

    def __repr__(self) -> str:
        return f'<{self.__class__.__name__} name={self.name!r}>'

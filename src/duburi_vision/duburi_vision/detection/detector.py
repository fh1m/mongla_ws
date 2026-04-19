"""Abstract Detector — every detection backend implements this.

Contract
--------
infer(frame_bgr) -> list[Detection]
    Run inference on ONE BGR frame. Returns a list of `Detection` (possibly
    empty). Must be safe to call repeatedly from the same thread; not
    required to be reentrant.

class_names() -> dict[int, str]
    Map class id -> human label for the model currently loaded. Used by
    the visualizer and by `messages.detections_to_array` for hypothesis
    string ids. Empty dict is allowed but visualization will only show
    "id=N".

is_ready() -> bool
    True after __init__ + warmup. Useful for the node startup banner.

Detection is a plain dataclass (no rclpy / no ultralytics types) so the
rest of the pipeline can build, test, and serialize it without a heavy
runtime. Bounding box convention is xyxy in the source frame's pixel
space (top-left origin), matching ultralytics.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

import numpy as np


@dataclass
class Detection:
    class_id:   int
    class_name: str
    score:      float                  # 0..1
    xyxy:       tuple                  # (x1, y1, x2, y2) in pixels

    # ----- Convenience derived getters -- cheap, just for readability ----- #
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


class Detector:
    name: str = 'base'

    def infer(self, frame_bgr: np.ndarray) -> List[Detection]:
        raise NotImplementedError

    def class_names(self) -> Dict[int, str]:
        return {}

    def is_ready(self) -> bool:
        return False

    def close(self) -> None:
        pass

    def __repr__(self) -> str:
        return f'<{self.__class__.__name__} name={self.name}>'


def largest(detections: List[Detection]) -> Detection | None:
    """Return the detection with the largest bbox area, or None if the
    list is empty. Centralized so the visualizer and the future visual-PID
    logic stay in agreement on what 'the' target is."""
    if not detections:
        return None
    return max(detections, key=lambda d: d.area)

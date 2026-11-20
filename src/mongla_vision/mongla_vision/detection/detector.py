"""Abstract Detector — every detection backend implements this.

Contract
--------
infer(frame_bgr) -> list[Detection]
    Run inference on ONE BGR frame. Returns a list of `Detection` (possibly
    empty). Must be safe to call repeatedly from the same thread; not
    required to be reentrant.

is_ready() -> bool
    True after __init__ + warmup. Useful for the node startup banner.

Detection is a plain dataclass (no rclpy / no ultralytics types) so the
rest of the pipeline can build, test, and serialize it without a heavy
runtime. Bounding box convention is xyxy in the source frame's pixel
space (top-left origin), matching ultralytics.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import numpy as np


@dataclass
class Detection:
    class_id:   int
    class_name: str
    score:      float                  # 0..1
    xyxy:       tuple                  # (x1, y1, x2, y2) in pixels

    # Instance mask, or None when the backend does not produce one. A uint8
    # 0/1 array the size of the INTEGER bbox and anchored at its top-left, not
    # a full-frame layer: a frame-sized mask per detection is mostly zeros, and
    # every consumer needs the box anyway. Optional with a None default so a
    # box-only backend and every existing construction site are unchanged --
    # `mask is None` means "this backend cannot", never "empty mask".
    mask:       Optional[np.ndarray] = None

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

    def is_ready(self) -> bool:
        return False

    def acting_conf(self) -> float:
        """The bar a box must clear to be acted on. Boxes `infer` returns
        under it are association-only (`assoc_conf`); a backend with no
        association floor returns only boxes at its bar, so 0.0 is exact."""
        return 0.0

    def close(self) -> None:
        pass

    def class_names(self) -> List[str]:
        """Every class this model can emit, sorted; [] before it is loaded.

        Not the allowlist: what the model CAN say, which is what a course or a
        mission naming a class must be checked against.
        """
        names = getattr(self, '_names', None) or {}
        return sorted({str(v) for v in names.values()})

    def __repr__(self) -> str:
        return f'<{self.__class__.__name__} name={self.name}>'


def allow_ids_for(names, class_allowlist, *, log=None, tag='[DET  ]'):
    """The class ids an allowlist keeps, or None for no filter.

    None, or nothing but blanks, means no filter. If every requested name is
    missing from the model, the ids would be EMPTY, and an empty filter
    drops every detection: the detector returns [] on every frame while the
    graph looks healthy, the silent-[] failure CLAUDE.md section 4 forbids,
    reached from one typo'd class name. So it refuses to narrow, loudly; the
    caller gets everything and can filter downstream. Every backend calls
    this, so they cannot disagree on it.
    """
    if class_allowlist is None:
        return None
    wanted = {str(c).strip().lower() for c in class_allowlist if str(c).strip()}
    if not wanted:
        return None
    ids = {cid for cid, cname in names.items() if str(cname).strip().lower() in wanted}
    unknown = wanted - {str(names[i]).strip().lower() for i in ids}
    if not ids:
        if log is not None:
            log.error(f'{tag} NONE of {sorted(wanted)} exist in this model -- '
                      f'ignoring the allowlist rather than filtering every '
                      f'detection away. Fix the class name, or the pipeline is '
                      f'blind while looking healthy.')
        return None
    if unknown and log is not None:
        log.warn(f'{tag} class(es) {sorted(unknown)} are not in this model. '
                 f'Available: {sorted(names.values())[:10]}')
    return ids


def largest(detections: List[Detection]) -> Detection | None:
    """Return the detection with the largest bbox area, or None if the
    list is empty. The detector's debug overlay and its range-crop size
    feedback both call this, so they agree on what 'the' target is."""
    if not detections:
        return None
    return max(detections, key=lambda d: d.area)

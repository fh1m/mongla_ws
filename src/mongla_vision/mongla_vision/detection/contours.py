"""Detections -> outlines. A box and a mask become the same thing here.

The champions' `DetectedObject2D` carries `uint16[] contour` described as "2D
bounding box OR contour surrounding the object", and no mask message exists
anywhere in their stack. Copying that is what lets a mission swap a detector
for a segmentation model, or run both, without a second consumer: a box is a
four-point contour, a mask is an N-point one, and every downstream user sees
one message.

Everything here is plain numpy + cv2 so it can be tested with no chip and no
ROS.
"""
from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

# Douglas-Peucker tolerance, as a FRACTION of the contour's own perimeter.
#
# Relative, not absolute, and that is the whole point: a torpedo hole at 3 m is
# forty pixels across and the same hole at 0.4 m is four hundred, and a fixed
# 2 px tolerance either erases the far one's shape or leaves the near one with
# six hundred points on the wire. 0.5 % of perimeter keeps roughly the same
# NUMBER of points at any range, which is what both the wire and a polygon
# solver care about.
#
# Measured on `yolov8n_seg` masks from bus.jpg: the six raw contours carry
# 195-1063 points and simplify to 9-27, with filled area changing by under
# 1 %. A tighter 0.1 % gives 30-90 points for no measurable area change; a
# looser 2 % starts cutting corners off the bus.
SIMPLIFY_FRAC = 0.005

# Below this many pixels a mask is not a shape, it is speckle. A contour of two
# points has no area and no orientation, and publishing it invites a consumer
# to fit a pose to noise.
MIN_CONTOUR_AREA_PX = 16.0

# Sentinel for "this producer did not compute an orientation". NOT 0, which is
# a real angle -- a box-only producer emitting 0 would claim every target is
# axis-aligned.
ANGLE_UNKNOWN = -1


def box_contour(xyxy: Sequence[float]) -> np.ndarray:
    """The four corners of a bbox, clockwise from top-left. Integer pixels."""
    x1, y1, x2, y2 = (float(v) for v in xyxy)
    return np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], np.float32)


def mask_contour(mask: np.ndarray, x_offset: float, y_offset: float,
                 *, simplify_frac: float = SIMPLIFY_FRAC
                 ) -> Optional[np.ndarray]:
    """Largest external contour of a box-anchored mask, in IMAGE pixels.

    `mask` is uint8 0/1 the size of the detection's own integer box, as
    `Detection.mask` carries it, so the offsets put it back in the frame.

    LARGEST ONLY, deliberately. A segmentation mask of one object routinely
    breaks into a main blob plus a few specks -- a hand, a reflection, a
    compression artefact. Publishing every piece would make one detection look
    like several to any consumer that counts contours, and a pose fitted to
    the union of a blob and a speck is worse than one fitted to the blob.
    """
    import cv2
    if mask is None or mask.size == 0:
        return None
    found, _ = cv2.findContours(np.ascontiguousarray(mask, dtype=np.uint8),
                                cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not found:
        return None
    best = max(found, key=cv2.contourArea)
    if cv2.contourArea(best) < MIN_CONTOUR_AREA_PX:
        return None
    peri = cv2.arcLength(best, True)
    if simplify_frac > 0.0 and peri > 0.0:
        best = cv2.approxPolyDP(best, simplify_frac * peri, True)
    pts = best.reshape(-1, 2).astype(np.float32)
    pts[:, 0] += float(x_offset)
    pts[:, 1] += float(y_offset)
    return pts


def obb_angle_deg(points: np.ndarray) -> int:
    """Orientation of the minimum-area rectangle, 0..180 degrees.

    Folded to 0..180 because a rectangle at 10 degrees and one at 190 are the
    same rectangle; leaving it signed would make two identical shapes look like
    a 180-degree disagreement to anything that averages angles.

    Needs a real shape: fewer than three points, or a degenerate rectangle,
    returns ANGLE_UNKNOWN rather than 0.
    """
    import cv2
    if points is None or len(points) < 3:
        return ANGLE_UNKNOWN
    (_, _), (w, h), ang = cv2.minAreaRect(points.astype(np.float32))
    if w <= 0.0 or h <= 0.0:
        return ANGLE_UNKNOWN
    # ⛔ `minAreaRect` REPORTS THE ANGLE OF ITS `w` EDGE, WHICH IS NOT ALWAYS
    # THE LONG ONE. Returning it raw makes a rectangle's reported orientation
    # jump by 90 degrees depending on which edge OpenCV happened to call the
    # width -- a 30-degree target reads as 120 and nothing errors. An OBB
    # angle only means anything to a consumer if it is the LONG axis, so
    # rotate by a quarter turn when the short edge was chosen.
    if w < h:
        ang += 90.0
    return int(round(ang)) % 180


def contour_for(det) -> Tuple[np.ndarray, int]:
    """(points, angle_deg) for one Detection -- its mask if it has one, else
    its box.

    This is the single place the two kinds converge, so no consumer ever has
    to ask which backend produced a detection.
    """
    pts = None
    mask = getattr(det, 'mask', None)
    if mask is not None:
        x1, y1 = float(det.xyxy[0]), float(det.xyxy[1])
        pts = mask_contour(mask, max(0.0, x1), max(0.0, y1))
    if pts is None:
        # A box carries no orientation information, and saying so is the point
        # of the sentinel.
        return box_contour(det.xyxy), ANGLE_UNKNOWN
    return pts, obb_angle_deg(pts)


def polygon_area_px(points: np.ndarray) -> float:
    """Shoelace area. Always positive; winding is not the caller's problem."""
    if points is None or len(points) < 3:
        return 0.0
    x, y = points[:, 0].astype(np.float64), points[:, 1].astype(np.float64)
    return abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))) / 2.0


def flatten(contours: Sequence[np.ndarray], *, width: int, height: int
            ) -> Tuple[List[int], List[int]]:
    """Ragged contours -> (offset, points) for the wire.

    `offset` gets ONE MORE entry than there are contours -- the total point
    count -- so the last contour needs no special case at the far end.

    Coordinates are clamped into the frame and rounded: the wire field is
    uint16, and a negative or out-of-frame value wraps to something enormous
    instead of erroring.
    """
    offset = [0]
    points: List[int] = []
    for c in contours:
        arr = np.asarray(c, np.float64).reshape(-1, 2)
        xs = np.clip(np.rint(arr[:, 0]), 0, max(0, width - 1)).astype(np.int64)
        ys = np.clip(np.rint(arr[:, 1]), 0, max(0, height - 1)).astype(np.int64)
        for x, y in zip(xs.tolist(), ys.tolist()):
            points.append(int(x))
            points.append(int(y))
        offset.append(len(points) // 2)
    return offset, points

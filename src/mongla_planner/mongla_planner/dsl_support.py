"""What the mission DSL's parts share: parsing detection messages, and small helpers."""
from __future__ import annotations

import os
import time as _time

from rcl_interfaces.msg import ParameterValue, ParameterType


# --------------------------------------------------------------------------- #
#  Detection-stream parsing helpers (defensive vs ROS distro field layout)    #
# --------------------------------------------------------------------------- #
# A parsed detection record: lowercased class + bbox geometry in pixels + conf.
# Tuple, not a class, so it copies trivially off the (reused) ROS message.
#   (class_lower, cx_px, cy_px, w_px, h_px, conf)

def _det_class_id(det) -> str:
    """Class id of a Detection2D's top hypothesis -- handles both layouts.

    Iron+ nests the label under ``results[0].hypothesis.class_id``; Humble's
    older message has a flat ``results[0].id``. Mirrors VisionState's
    ``_hypothesis_class_id`` so detected()/where() agree with the control path.
    """
    if not det.results:
        return ''
    hyp = det.results[0]
    if hasattr(hyp, 'hypothesis') and hasattr(hyp.hypothesis, 'class_id'):
        return str(hyp.hypothesis.class_id)
    if hasattr(hyp, 'id'):
        return str(hyp.id)
    return ''


def _det_score(det) -> float:
    """Confidence of a Detection2D's top hypothesis (both layouts)."""
    if not det.results:
        return 0.0
    hyp = det.results[0]
    if hasattr(hyp, 'hypothesis') and hasattr(hyp.hypothesis, 'score'):
        return float(hyp.hypothesis.score)
    if hasattr(hyp, 'score'):
        return float(hyp.score)
    return 0.0


def _det_center(bbox):
    """(cx, cy) of a BoundingBox2D -- Iron+ Pose2D w/ Point2D, or Humble flat."""
    centre = bbox.center
    if hasattr(centre, 'position'):       # Iron+: Pose2D with Point2D position
        return float(centre.position.x), float(centre.position.y)
    return float(centre.x), float(centre.y)   # Humble: flat Pose2D


def _parse_detections(msg) -> list:
    """Detection2DArray -> [DetRecord, ...], copied to plain Python.

    Copy eagerly: rclpy may reuse the underlying C++ buffer across callbacks,
    so holding the message objects past the callback would corrupt the cache.
    """
    out = []
    for det in msg.detections:
        cls = _det_class_id(det).strip().lower()
        if not cls:
            continue
        cx, cy = _det_center(det.bbox)
        out.append((cls, cx, cy,
                    float(det.bbox.size_x), float(det.bbox.size_y),
                    _det_score(det)))
    return out


class _DetView:
    """A cached record `(cls, cx, cy, w, h, score)` seen as a Detection.

    `mongla_vision.identity` works on anything with `class_name`, `score` and
    `xyxy`, so the adapter lives here rather than changing either side: the
    cache stays a plain tuple (copied eagerly off the C++ buffer) and the
    geometry module stays free of the DSL's storage choices.
    """

    __slots__ = ('class_name', 'score', 'xyxy', 'mask')

    def __init__(self, rec):
        cls, cx, cy, w, h, score = rec
        self.class_name = cls
        self.score = float(score)
        self.xyxy = (cx - w * 0.5, cy - h * 0.5, cx + w * 0.5, cy + h * 0.5)
        self.mask = None


def _outline_view(outline):
    """An `Outline` polygon as a Detection-shaped view with a filled mask.

    The mask is the polygon rasterised at its integer bounding box -- the same
    anchoring `Detection.mask` uses, so `identity` reads both the same way.
    None for a degenerate polygon.
    """
    import numpy as np
    pts = np.asarray(outline.points, dtype=np.int32).reshape(-1, 2)
    if len(pts) < 3:
        return None
    x1, y1 = pts.min(axis=0)
    x2, y2 = pts.max(axis=0)
    w, h = int(x2 - x1 + 1), int(y2 - y1 + 1)
    try:
        import cv2
        mask = np.zeros((h, w), np.uint8)
        cv2.fillPoly(mask, [pts - [x1, y1]], 1)
    except Exception:                   # noqa: BLE001 -- no cv2: box only
        return None
    v = _DetView((outline.class_name, (x1 + x2) / 2.0, (y1 + y2) / 2.0,
                  float(x2 - x1), float(y2 - y1), outline.score))
    v.xyxy = (float(x1), float(y1), float(x2 + 1), float(y2 + 1))
    v.mask = mask
    return v


def _eval_where(records: list, needle: str, width: float,
                band: float) -> tuple:
    """Bearing of the largest matching detection.

    Returns ``(label, offset)`` where ``label`` is 'left' | 'center' |
    'right' | 'unknown' and ``offset`` is the signed normalized horizontal
    offset in [-1, +1] (negative = left of centre, positive = right), or
    ``None`` when unknown. 'unknown' = class absent or image width not yet
    known. ``band`` is the centre dead-zone half-width (normalized).
    """
    n = str(needle).strip().lower()
    if not width or width <= 0.0:
        return ('unknown', None)
    best = None
    best_area = 0.0
    for rec in records:
        if rec[0] != n:
            continue
        area = rec[3] * rec[4]
        if area > best_area:
            best_area = area
            best = rec
    if best is None:
        return ('unknown', None)
    offset = (best[1] - width * 0.5) / (width * 0.5)   # [-1, +1]
    if offset < -band:
        return ('left', offset)
    if offset > band:
        return ('right', offset)
    return ('center', offset)


def _to_float(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return float(value)
    return value


def _param_value(value) -> ParameterValue:
    """Wrap a Python value in a typed rcl_interfaces ParameterValue.

    bool is checked before int (bool is a subclass of int in Python).
    """
    if isinstance(value, bool):
        return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    if isinstance(value, float):
        return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
    if isinstance(value, int):
        return ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
    return ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=str(value))


Outline = None


def _pick_outline(msg, target_class: str):
    """Largest-area outline of `target_class` in a TargetContours message, or None."""
    global Outline
    if Outline is None:
        from collections import namedtuple
        Outline = namedtuple('Outline', 'class_name score angle_deg area_px points')
    want = target_class.strip().lower()
    names = list(msg.class_name)
    offs, pts = list(msg.offset), list(msg.points)
    best = None
    for i, n in enumerate(names):
        if str(n).strip().lower() != want or i + 1 >= len(offs):
            continue
        a, b = int(offs[i]), int(offs[i + 1])
        poly = [(int(pts[2 * k]), int(pts[2 * k + 1])) for k in range(a, b)
                if 2 * k + 1 < len(pts)]
        area = int(msg.area_px[i]) if i < len(msg.area_px) else 0
        if best is None or area > best.area_px:
            best = Outline(str(n), float(msg.score[i]), int(msg.angle_deg[i]),
                           area, poly)
    return best


def _write_ppm(img, folder: str, stem: str):
    """sensor_msgs/Image (rgb8 | bgr8) -> binary PPM. None for other encodings."""
    enc = str(img.encoding).lower()
    if enc not in ('rgb8', 'bgr8'):
        return None
    w, h, step = int(img.width), int(img.height), int(img.step)
    data = bytes(img.data)
    rows = [data[r * step:r * step + 3 * w] for r in range(h)]
    if enc == 'bgr8':
        rows = [bytes(b for i in range(0, 3 * w, 3) for b in (row[i + 2], row[i + 1], row[i]))
                for row in rows]
    path = os.path.join(folder, f'{stem}_{_time.strftime("%Y%m%d_%H%M%S")}.ppm')
    with open(path, 'wb') as fh:
        fh.write(f'P6 {w} {h} 255\n'.encode())
        fh.write(b''.join(rows))
    return path


# A fused pose older than this is not "where the target is now" (issue #57).
# The same 1.0 s VisionState.target_pose uses for a single-frame pose.
FUSED_POSE_MAX_AGE_S = 1.0

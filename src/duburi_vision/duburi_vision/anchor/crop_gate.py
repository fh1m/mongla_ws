"""crop_gate -- pick the bbox to crop for a detection-gated anchor snap.

Pure functions (duck-typed on the vision_msgs Detection2D shape, no rclpy / no
torch): given a frame's detections, choose the largest one matching the target
class with score >= conf that sits within ``err`` px of the horizontal centre,
and return its padded, frame-clamped crop box. Kept separate from anchor_node
so the gate -- the most intricate new logic -- is unit-testable with hand-built
detection objects.
"""

from __future__ import annotations

from typing import Optional, Tuple

# Pad the detection bbox by this factor when cropping (more keypoints + absorbs
# the frame<->detection temporal skew). Mirrors anchor_node's default.
DEFAULT_PAD = 1.3
# Smallest crop edge (px) we'll accept -- below this there aren't enough pixels
# for a stable reference, so the caller keeps waiting / falls back.
_MIN_EDGE_PX = 8


def det_class_score(det) -> Tuple[str, float]:
    """(class_id, score) from a Detection2D across vision_msgs variants."""
    results = getattr(det, 'results', None) or []
    if not results:
        return '', 0.0
    r = results[0]
    if hasattr(r, 'hypothesis'):
        return str(r.hypothesis.class_id), float(r.hypothesis.score)
    return str(getattr(r, 'id', '')), float(getattr(r, 'score', 0.0))


def qualifying_bbox(detections, w: int, h: int, target: str, conf: float,
                    err_px: float, pad: float = DEFAULT_PAD
                    ) -> Optional[Tuple[int, int, int, int]]:
    """Largest detection of ``target`` (score>=conf, centred within ``err_px``)
    -> padded, frame-clamped (x1,y1,x2,y2), or None if nothing qualifies.

    The centre gate is HORIZONTAL only ("lat offset"): ``err_px <= 0`` disables
    it (snap on the first class+conf match regardless of centring). Matching is
    case-insensitive. A degenerate (too-small) crop returns None so the caller
    keeps waiting until the 3 s fallback.
    """
    want = str(target).strip().lower()
    if not want:
        return None
    best = None
    best_area = 0.0
    for det in detections:
        cls, score = det_class_score(det)
        if cls.strip().lower() != want or score < conf:
            continue
        cx = float(det.bbox.center.position.x)
        if err_px > 0.0 and abs(cx - w * 0.5) > err_px:
            continue
        area = float(det.bbox.size_x) * float(det.bbox.size_y)
        if area > best_area:
            best_area = area
            best = det
    if best is None:
        return None

    cx = float(best.bbox.center.position.x)
    cy = float(best.bbox.center.position.y)
    hw = float(best.bbox.size_x) * 0.5 * pad
    hh = float(best.bbox.size_y) * 0.5 * pad
    x1 = max(0, int(cx - hw)); x2 = min(int(w), int(cx + hw))
    y1 = max(0, int(cy - hh)); y2 = min(int(h), int(cy + hh))
    if x2 - x1 < _MIN_EDGE_PX or y2 - y1 < _MIN_EDGE_PX:
        return None
    return (x1, y1, x2, y2)

"""draw_video -- video section overlays for the mission-control HUD.

render_video_section() is the main entry point.  It returns a new ndarray
(copy of the input frame) with all per-frame overlays applied:
  - reticle + deadband box
  - supervision annotation suite (trails, boxes, corners, labels)
  - primary target: accent border, full-axis hairlines, size+AR tag
  - horizontal alignment bar at frame bottom
  - stale-frame banner (when healthy=False)

Helper functions (draw_*, highlight_*, crosshair, etc.) are kept for callers
that apply overlays individually without going through render_video_section().
"""

from __future__ import annotations

from typing import List, Optional

import cv2
import numpy as np

from .detection.detector import Detection, largest
from .draw_widgets import C_BG, C_ACCENT, C_AMBER, C_OK, C_ERR, C_TEXT

_FONT     = cv2.FONT_HERSHEY_DUPLEX
_FT       = 1
C_RETICLE  = (0, 200, 200)    # bright cyan reticle (was near-black 50,55,55)
C_HAIRLINE = (0, 180, 255)    # orange-yellow target hairlines


# ── Supervision annotators (lazy-init shared with draw.py) ────────────────── #

_SV: dict | None = None


def _get_sv() -> dict:
    global _SV
    if _SV is None:
        import supervision as sv
        _SV = {
            'trace': sv.TraceAnnotator(
                position=sv.Position.CENTER,
                trace_length=30,
                thickness=2,
                color_lookup=sv.ColorLookup.CLASS),
            'corners': sv.BoxCornerAnnotator(
                thickness=4, corner_length=18,
                color=sv.Color.WHITE),
            'label': sv.LabelAnnotator(
                text_scale=0.38, text_thickness=1,
                color_lookup=sv.ColorLookup.CLASS,
                border_radius=2,
                smart_position=False),
        }
    return _SV


def _to_sv(detections: List[Detection], track_ids=None):
    import supervision as sv
    if not detections:
        return sv.Detections.empty()
    xyxy   = np.array([list(d.xyxy) for d in detections], dtype=float)
    conf   = np.array([d.score      for d in detections], dtype=float)
    cls_id = np.array([d.class_id   for d in detections], dtype=int)
    sv_det = sv.Detections(xyxy=xyxy, confidence=conf, class_id=cls_id)
    if track_ids is not None and len(track_ids) == len(detections):
        sv_det.tracker_id = np.array(track_ids, dtype=int)
    return sv_det


# ── Main entry point ──────────────────────────────────────────────────────── #

def render_video_section(frame_bgr: np.ndarray,
                         detections: List[Detection], *,
                         show_reticle: bool = True,
                         deadband: float = 0.05,
                         primary: Optional[Detection] = None,
                         healthy: bool = True,
                         track_ids=None) -> np.ndarray:
    """Return annotated copy of frame_bgr with all video overlays applied."""
    if frame_bgr is None:
        return frame_bgr

    out = frame_bgr.copy()
    h, w = out.shape[:2]

    cx_frame, cy_frame = w // 2, h // 2

    # 1. Reticle + deadband box
    if show_reticle:
        _dashed_line(out, (cx_frame, 0), (cx_frame, h), C_RETICLE, dash=8, gap=6, thickness=1)
        _dashed_line(out, (0, cy_frame), (w, cy_frame), C_RETICLE, dash=8, gap=6, thickness=1)
        cv2.circle(out, (cx_frame, cy_frame), 4, C_RETICLE, 1, cv2.LINE_AA)
        cv2.circle(out, (cx_frame, cy_frame), 1, C_RETICLE, -1, cv2.LINE_AA)
        db_px = int(deadband * w / 2)
        db_py = int(deadband * h / 2)
        cv2.rectangle(out,
                      (cx_frame - db_px, cy_frame - db_py),
                      (cx_frame + db_px, cy_frame + db_py),
                      C_RETICLE, 1, cv2.LINE_AA)

    # 2. Supervision annotation suite (trails behind, then boxes)
    primary = primary or largest(detections)
    if detections:
        sv = _get_sv()
        sv_all = _to_sv(detections, track_ids)
        if sv_all.tracker_id is not None:
            out = sv['trace'].annotate(scene=out, detections=sv_all)
        out = sv['corners'].annotate(scene=out, detections=sv_all)
        labels = []
        for i, d in enumerate(detections):
            tid    = (track_ids[i] if track_ids and i < len(track_ids)
                      and track_ids[i] is not None else None)
            prefix = f'#{tid} ' if tid is not None else ''
            labels.append(f'{prefix}{d.class_name} {int(d.score * 100)}%')
        out = sv['label'].annotate(scene=out, detections=sv_all, labels=labels)

    # 3. Primary target overlays
    if primary is not None:
        x1p, y1p, x2p, y2p = (int(v) for v in primary.xyxy)
        cx_t, cy_t = int(primary.cx), int(primary.cy)
        ex  = (primary.cx - w / 2.0) / max(w / 2.0, 1.0)
        ey  = (primary.cy - h / 2.0) / max(h / 2.0, 1.0)
        aligned = abs(ex) < deadband and abs(ey) < deadband

        cv2.rectangle(out, (x1p, y1p), (x2p, y2p), C_ACCENT, 1, cv2.LINE_AA)
        _draw_corners(out, x1p, y1p, x2p, y2p, C_ACCENT, length=15, thickness=3)
        cv2.circle(out, (cx_t, cy_t), 4, C_HAIRLINE, -1, cv2.LINE_AA)

        # Glow hairlines: wide dim underlay + thin bright overlay
        _glow = (0, 70, 100)
        _dashed_line(out, (cx_t, 0),    (cx_t, y1p), _glow,     dash=5, gap=5, thickness=3)
        _dashed_line(out, (cx_t, 0),    (cx_t, y1p), C_HAIRLINE, dash=5, gap=5, thickness=1)
        _dashed_line(out, (cx_t, y2p),  (cx_t, h),   _glow,     dash=5, gap=5, thickness=3)
        _dashed_line(out, (cx_t, y2p),  (cx_t, h),   C_HAIRLINE, dash=5, gap=5, thickness=1)
        _dashed_line(out, (0,    cy_t), (x1p, cy_t), _glow,     dash=5, gap=5, thickness=3)
        _dashed_line(out, (0,    cy_t), (x1p, cy_t), C_HAIRLINE, dash=5, gap=5, thickness=1)
        _dashed_line(out, (x2p,  cy_t), (w,   cy_t), _glow,     dash=5, gap=5, thickness=3)
        _dashed_line(out, (x2p,  cy_t), (w,   cy_t), C_HAIRLINE, dash=5, gap=5, thickness=1)

        # Deadband fill: faint green tint when target is inside deadband
        if aligned:
            db_px = int(deadband * w / 2)
            db_py = int(deadband * h / 2)
            overlay = out.copy()
            cv2.rectangle(overlay,
                          (cx_frame - db_px, cy_frame - db_py),
                          (cx_frame + db_px, cy_frame + db_py), C_OK, -1)
            cv2.addWeighted(overlay, 0.15, out, 0.85, 0, out)

        # Correction direction arrow: from target center toward frame center
        dist = float(np.hypot(cx_t - cx_frame, cy_t - cy_frame))
        arrow_len = min(60, int(dist * 0.6))
        if arrow_len > 10:
            dx = cx_frame - cx_t
            dy = cy_frame - cy_t
            tip = (int(cx_t + dx / max(dist, 1.0) * arrow_len),
                   int(cy_t + dy / max(dist, 1.0) * arrow_len))
            arr_col = C_OK if aligned else C_AMBER
            cv2.arrowedLine(out, (cx_t, cy_t), tip, arr_col, 2,
                            cv2.LINE_AA, tipLength=0.3)

        # On-frame offset text label below bbox
        lbl = f'X:{ex:+.2f} Y:{ey:+.2f}  {int(primary.score * 100)}%'
        lbl_y = min(y2p + 16, h - 4)
        cv2.putText(out, lbl, (x1p, lbl_y), _FONT, 0.36, C_TEXT, _FT, cv2.LINE_AA)

        # Horizontal alignment bar (wider: 12px, moved to h-14)
        _bar_y = h - 14
        _bar_h = 12
        bar_col = C_OK if abs(ex) < deadband else C_AMBER
        cv2.rectangle(out, (0, _bar_y), (w, _bar_y + _bar_h), C_BG, -1)
        cv2.line(out, (cx_frame, _bar_y), (cx_frame, _bar_y + _bar_h), C_RETICLE, 1)
        db_px = int(deadband * w / 2)
        cv2.rectangle(out, (cx_frame - db_px, _bar_y),
                      (cx_frame + db_px, _bar_y + _bar_h), C_RETICLE, 1)
        dot_x = max(4, min(w - 4, cx_t))
        cv2.line(out, (cx_frame, _bar_y + _bar_h // 2),
                 (dot_x, _bar_y + _bar_h // 2), bar_col, 1, cv2.LINE_AA)
        cv2.circle(out, (dot_x, _bar_y + _bar_h // 2), 4, bar_col, -1, cv2.LINE_AA)

    # 4. Stale banner
    if not healthy:
        cv2.rectangle(out, (0, 0), (w, 28), C_ERR, -1)
        cv2.putText(out, 'STALE FRAME', (8, 20),
                    _FONT, 0.55, (255, 255, 255), _FT, cv2.LINE_AA)

    return out


# ── Standalone helpers (for external callers) ─────────────────────────────── #

def draw_detections(frame_bgr: np.ndarray, detections: List[Detection]) -> np.ndarray:
    if frame_bgr is None or not detections:
        return frame_bgr
    sv  = _get_sv()
    svd = _to_sv(detections)
    out = frame_bgr.copy()
    out = sv['trace'].annotate(scene=out, detections=svd)
    out = sv['box'].annotate(scene=out, detections=svd)
    out = sv['corners'].annotate(scene=out, detections=svd)
    out = sv['triangle'].annotate(scene=out, detections=svd)
    out = sv['pct_bar'].annotate(scene=out, detections=svd)
    labels = [f'{d.class_name} {int(d.score * 100)}%' for d in detections]
    return sv['label'].annotate(scene=out, detections=svd, labels=labels)


def draw_track_ids(frame_bgr: np.ndarray, tracks) -> np.ndarray:
    if frame_bgr is None or not tracks:
        return frame_bgr
    _PALETTE = [
        (255, 100,  50), ( 50, 220, 100), ( 50, 100, 255),
        (255, 200,  50), (180,  50, 255), ( 50, 255, 220),
    ]
    out = frame_bgr.copy()
    for td in tracks:
        color = _PALETTE[abs(int(td.track_id)) % len(_PALETTE)]
        x1, y1, x2, y2 = (int(v) for v in td.xyxy)
        thick = 1 if td.predicted else 2
        if td.predicted:
            overlay = out.copy()
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, thick, cv2.LINE_AA)
            cv2.addWeighted(overlay, 0.45, out, 0.55, 0, out)
        else:
            cv2.rectangle(out, (x1, y1), (x2, y2), color, thick, cv2.LINE_AA)
        lbl = f"#{td.track_id} {td.class_name}" + (' (pred)' if td.predicted else '')
        cv2.putText(out, lbl, (x1, max(y1 - 5, 12)), _FONT, 0.36, color, _FT, cv2.LINE_AA)
    return out


# ── Internal helpers ──────────────────────────────────────────────────────── #

def _dashed_line(img, p1, p2, color, dash=10, gap=6, thickness=1):
    x1, y1 = p1
    x2, y2 = p2
    length  = max(int(np.hypot(x2 - x1, y2 - y1)), 1)
    step    = dash + gap
    n       = length // step
    if n == 0:
        cv2.line(img, p1, p2, color, thickness, cv2.LINE_AA)
        return
    dx = (x2 - x1) / length
    dy = (y2 - y1) / length
    for i in range(n + 1):
        a = i * step
        b = min(a + dash, length)
        cv2.line(img,
                 (int(x1 + dx * a), int(y1 + dy * a)),
                 (int(x1 + dx * b), int(y1 + dy * b)),
                 color, thickness, cv2.LINE_AA)


def _draw_corners(img, x1, y1, x2, y2, color, length=14, thickness=3):
    for (px, py, dx, dy) in ((x1, y1, +1, +1), (x2, y1, -1, +1),
                               (x1, y2, +1, -1), (x2, y2, -1, -1)):
        cv2.line(img, (px, py), (px + dx * length, py), color, thickness, cv2.LINE_AA)
        cv2.line(img, (px, py), (px, py + dy * length), color, thickness, cv2.LINE_AA)

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
from .draw_widgets import C_BG, C_ACCENT, C_AMBER, C_OK, C_ERR, pil_text, pil_text_size
C_RETICLE  = (0, 200, 200)    # bright cyan reticle (was near-black 50,55,55)


def _depth_color(v: float) -> tuple:
    """Map vis_range 0=far→blue, 0.5=mid→green, 1=close→red (BGR)."""
    if v < 0.5:
        t = v * 2.0
        return (int(255 * (1.0 - t)), int(255 * t), 0)
    else:
        t = (v - 0.5) * 2.0
        return (0, int(255 * (1.0 - t)), int(255 * t))


C_HAIRLINE = (0, 180, 255)    # orange-yellow target hairlines


# NOTE: the supervision annotator suite (Box/BoxCorner/Dot/Trace) was removed from
# the render path -- it cost ~15 ms/frame at 1920p (the FPS bottleneck) and its
# per-ID trace smeared. Boxes/corners/dots are now direct cv2 (_draw_boxes_fast),
# the trail a bounded polyline (_update_and_draw_trail): ~10x cheaper, sharper.


# ── Primary-target trail: a bounded, age-faded polyline (replaces the laggy
# per-ID supervision trace). O(N) with N=_TRAIL_LEN -- negligible cost, and it
# NEVER smears because it tracks one point and clears on target loss. Keyed so
# side-by-side's two render passes don't share a buffer.
from collections import deque   # noqa: E402  (local to the trail feature)

_TRAIL: dict = {}
_TRAIL_LEN = 28


def _update_and_draw_trail(out, key, pt, sf, base_col=(0, 200, 255)) -> None:
    dq = _TRAIL.get(key)
    if dq is None:
        dq = _TRAIL[key] = deque(maxlen=_TRAIL_LEN)
    if pt is None:
        dq.clear()          # target lost -> drop the trail (no stale lag)
        return
    dq.append((int(pt[0]), int(pt[1])))
    n = len(dq)
    if n < 2:
        return
    pts = list(dq)
    b, g, r = base_col
    thick = max(1, int(3 * sf))
    for i in range(1, n):
        f = i / (n - 1)                      # 0=oldest -> 1=newest
        col = (int(b * f), int(g * f), int(r * f))   # fade toward black with age
        cv2.line(out, pts[i - 1], pts[i], col,
                 max(1, int(thick * (0.4 + 0.6 * f))), cv2.LINE_AA)


def _draw_labels(out: np.ndarray, detections: list, track_ids, sf: float) -> None:
    """Dark semi-transparent pill label above each detection bbox.

    Format: [#id] class_name  conf%
    Placed above the top edge; falls inside top edge when no space above.
    """
    h_f, w_f = out.shape[:2]
    fs = max(0.5, 0.52 * sf)   # bumped from 0.34 -- readable on a laptop
    pad_x = max(6, int(9 * sf))
    pad_y = max(4, int(6 * sf))
    for i, d in enumerate(detections):
        tid = (track_ids[i] if track_ids and i < len(track_ids)
               and track_ids[i] is not None else None)
        label = (f'#{tid} ' if tid is not None else '') + f'{d.class_name}  {int(d.score * 100)}%'
        x1, y1 = int(d.xyxy[0]), int(d.xyxy[1])
        tw, th = pil_text_size(label, fs)
        pill_w = min(tw + pad_x * 2, w_f - x1)
        pill_h = th + pad_y * 2

        # Position: above bbox if room, else inside top edge
        if y1 - pill_h >= 1:
            by1, by2 = y1 - pill_h, y1
        else:
            by1, by2 = y1, min(y1 + pill_h, h_f - 1)
        bx1, bx2 = x1, min(x1 + pill_w, w_f - 1)

        # Semi-transparent near-black background -- blend ONLY the pill ROI, not a
        # full-frame copy per label (the old cost that scaled with detection count).
        if bx2 > bx1 and by2 > by1:
            roi = out[by1:by2, bx1:bx2]
            dark = np.full_like(roi, 12)
            cv2.addWeighted(dark, 0.78, roi, 0.22, 0, roi)

        # White text; y_bottom = by2 - pad_y
        pil_text(out, label, (bx1 + pad_x, by2 - pad_y), fs, (235, 235, 235))


# ── Main entry point ──────────────────────────────────────────────────────── #

def render_video_section(frame_bgr: np.ndarray,
                         detections: List[Detection], *,
                         show_reticle: bool = True,
                         deadband: float = 0.05,
                         primary: Optional[Detection] = None,
                         healthy: bool = True,
                         track_ids=None,
                         vis_range_values: Optional[List[float]] = None,
                         depth_map_bgr: Optional[np.ndarray] = None,
                         draw_trail: bool = True,
                         trail_key: str = 'main') -> np.ndarray:
    """Return annotated copy of frame_bgr with all video overlays applied."""
    if frame_bgr is None:
        return frame_bgr

    out = frame_bgr.copy()
    h, w = out.shape[:2]
    # Scale factor: overlays are calibrated for a 1920px render and shrink with the
    # window. Floor RAISED to 0.9 (was 0.6) so boxes/labels stay legible on a
    # laptop; +0.15 so everything reads a touch larger than the old baseline.
    sf = max(0.9, w / 1920.0) + 0.15

    cx_frame, cy_frame = w // 2, h // 2
    _rt = max(1, int(2 * sf))   # reticle thickness -- scaled, was hardcoded 1px

    # 1. Reticle + deadband box
    if show_reticle:
        _dashed_line(out, (cx_frame, 0), (cx_frame, h), C_RETICLE, dash=10, gap=7, thickness=_rt)
        _dashed_line(out, (0, cy_frame), (w, cy_frame), C_RETICLE, dash=10, gap=7, thickness=_rt)
        cv2.circle(out, (cx_frame, cy_frame), max(5, int(6 * sf)), C_RETICLE, _rt, cv2.LINE_AA)
        cv2.circle(out, (cx_frame, cy_frame), 2, C_RETICLE, -1, cv2.LINE_AA)
        db_px = int(deadband * w / 2)
        db_py = int(deadband * h / 2)
        cv2.rectangle(out,
                      (cx_frame - db_px, cy_frame - db_py),
                      (cx_frame + db_px, cy_frame + db_py),
                      C_RETICLE, _rt, cv2.LINE_AA)

    # 2. Bounded primary-target trail (age-faded polyline; clears on loss so it
    #    never smears). Drawn UNDER the boxes. draw_trail=False for the secondary
    #    side-by-side pass so the two views don't share the buffer.
    primary = primary or largest(detections)
    if draw_trail:
        _pt = (int(primary.cx), int(primary.cy)) if primary is not None else None
        _update_and_draw_trail(out, trail_key, _pt, sf)

    # 3. Annotation suite: box → corners → dot → depth borders → labels.
    #    Direct cv2 (see _draw_boxes_fast) -- the supervision annotators cost ~15 ms/
    #    frame at 1920p and were the FPS bottleneck; this is ~10x cheaper.
    if detections:
        _draw_boxes_fast(out, detections, sf)

        # Depth-colored border overlay — FAR=blue, MID=green, CLOSE=red
        vr_list = vis_range_values or []
        if vr_list:
            for i, d in enumerate(detections):
                if i >= len(vr_list):
                    break
                dc = _depth_color(vr_list[i])
                x1d, y1d, x2d, y2d = (int(v) for v in d.xyxy)
                cv2.rectangle(out, (x1d, y1d), (x2d, y2d), dc,
                              max(2, int(3 * sf)), cv2.LINE_AA)

        # Dark pill labels: class  conf%  (with track ID prefix when tracked)
        _draw_labels(out, detections, track_ids, sf)

    # 3. Primary target overlays
    if primary is not None:
        x1p, y1p, x2p, y2p = (int(v) for v in primary.xyxy)
        cx_t, cy_t = int(primary.cx), int(primary.cy)
        ex  = (primary.cx - w / 2.0) / max(w / 2.0, 1.0)
        ey  = (primary.cy - h / 2.0) / max(h / 2.0, 1.0)
        aligned = abs(ex) < deadband and abs(ey) < deadband

        cv2.rectangle(out, (x1p, y1p), (x2p, y2p), C_ACCENT, max(2, int(2 * sf)), cv2.LINE_AA)
        _draw_corners(out, x1p, y1p, x2p, y2p, C_ACCENT,
                      length=max(18, int(24 * sf)), thickness=max(3, int(4 * sf)))
        cv2.circle(out, (cx_t, cy_t), max(5, int(6 * sf)), C_HAIRLINE, -1, cv2.LINE_AA)

        # Full-frame crosshair hairlines from the target centre to the frame edges.
        # SOLID (4 cv2.line calls), NOT dashed: the old dashed+glow variant built
        # ~1300 anti-aliased segments per frame and was THE HUD FPS bottleneck
        # (~15 ms/frame at 1920p). Solid reads just as clearly and is ~50x cheaper.
        _bt = max(2, int(2 * sf))
        cv2.line(out, (cx_t, 0),   (cx_t, y1p), C_HAIRLINE, _bt, cv2.LINE_AA)
        cv2.line(out, (cx_t, y2p), (cx_t, h),   C_HAIRLINE, _bt, cv2.LINE_AA)
        cv2.line(out, (0,   cy_t), (x1p, cy_t), C_HAIRLINE, _bt, cv2.LINE_AA)
        cv2.line(out, (x2p, cy_t), (w,   cy_t), C_HAIRLINE, _bt, cv2.LINE_AA)

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
        arrow_len = min(int(60 * sf), int(dist * 0.6))
        if arrow_len > 10:
            dx = cx_frame - cx_t
            dy = cy_frame - cy_t
            tip = (int(cx_t + dx / max(dist, 1.0) * arrow_len),
                   int(cy_t + dy / max(dist, 1.0) * arrow_len))
            arr_col = C_OK if aligned else C_AMBER
            cv2.arrowedLine(out, (cx_t, cy_t), tip, arr_col,
                            max(1, int(2 * sf)), cv2.LINE_AA, tipLength=0.3)

        # Horizontal alignment bar — scaled height
        _bar_h = max(8, int(12 * sf))
        _bar_y = h - _bar_h - 2
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

    # 4. Depth map inset (top-right corner)
    if depth_map_bgr is not None:
        inset_w  = max(96, w // 5)
        aspect   = depth_map_bgr.shape[0] / max(depth_map_bgr.shape[1], 1)
        inset_h  = int(inset_w * aspect)
        inset    = cv2.resize(depth_map_bgr, (inset_w, inset_h), interpolation=cv2.INTER_AREA)
        pad      = int(6 * sf)
        ix       = w - inset_w - pad
        iy       = pad
        if iy + inset_h < h:
            roi = out[iy:iy + inset_h, ix:ix + inset_w]
            cv2.addWeighted(inset, 0.88, roi, 0.12, 0, roi)
            out[iy:iy + inset_h, ix:ix + inset_w] = roi
            cv2.rectangle(out, (ix - 1, iy - 1), (ix + inset_w, iy + inset_h),
                          (100, 100, 100), 1, cv2.LINE_AA)
            pil_text(out, 'DEPTH', (ix + 2, iy + 2), 0.22 * sf, (200, 200, 200))
            pil_text(out, 'ONNX', (ix + 2, iy + inset_h - int(10 * sf)), 0.20 * sf, C_OK)

    # 5. Stale banner
    if not healthy:
        banner_h = max(20, int(28 * sf))
        cv2.rectangle(out, (0, 0), (w, banner_h), C_ERR, -1)
        pil_text(out, 'STALE FRAME', (8, int(banner_h * 0.75)), 0.55 * sf, (255, 255, 255))

    return out


# ── Standalone helpers (for external callers) ─────────────────────────────── #

def draw_track_ids(frame_bgr: np.ndarray, tracks) -> np.ndarray:
    if frame_bgr is None or not tracks:
        return frame_bgr
    _PALETTE = [
        (255, 100,  50), ( 50, 220, 100), ( 50, 100, 255),
        (255, 200,  50), (180,  50, 255), ( 50, 255, 220),
    ]
    out = frame_bgr.copy()
    w = out.shape[1]
    sf = max(1.0, w / 640.0)
    for td in tracks:
        color = _PALETTE[abs(int(td.track_id)) % len(_PALETTE)]
        x1, y1, x2, y2 = (int(v) for v in td.xyxy)
        thick = max(1, int(sf)) if td.predicted else max(1, int(2 * sf))
        if td.predicted:
            overlay = out.copy()
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, thick, cv2.LINE_AA)
            cv2.addWeighted(overlay, 0.45, out, 0.55, 0, out)
        else:
            cv2.rectangle(out, (x1, y1), (x2, y2), color, thick, cv2.LINE_AA)
        lbl = f"#{td.track_id} {td.class_name}" + (' (pred)' if td.predicted else '')
        pil_text(out, lbl, (x1, max(y1 - 5, int(12 * sf))), 0.36 * sf, color)
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


# Per-class colours for the fast box drawer (BGR). Cycles by class_id.
_CLASS_PALETTE = [
    (255, 100,  50), ( 50, 220, 100), ( 50, 100, 255),
    (255, 200,  50), (180,  50, 255), ( 50, 255, 220),
]


def _draw_boxes_fast(out, detections, sf) -> None:
    """Direct cv2 box + corner + centre-dot draw for every detection.

    Replaces the supervision BoxAnnotator/BoxCornerAnnotator/DotAnnotator suite,
    which cost ~15 ms/frame at 1920p (measured) -- the HUD's FPS bottleneck. Pure
    cv2 is ~10x cheaper and lets us size everything by sf. Class-coloured; boxes
    are bold + high-contrast so they read on a laptop (the operator's complaint)."""
    box_th = max(3, int(4 * sf))
    cor_th = max(3, int(5 * sf))
    cor_ln = max(14, int(26 * sf))
    dot_r  = max(5, int(8 * sf))
    dot_th = max(2, int(3 * sf))
    for d in detections:
        col = _CLASS_PALETTE[int(d.class_id) % len(_CLASS_PALETTE)]
        x1, y1, x2, y2 = (int(v) for v in d.xyxy)
        cv2.rectangle(out, (x1, y1), (x2, y2), col, box_th, cv2.LINE_AA)
        _draw_corners(out, x1, y1, x2, y2, (255, 255, 255),
                      length=cor_ln, thickness=cor_th)
        cx, cy = int(d.cx), int(d.cy)
        cv2.circle(out, (cx, cy), dot_r, col, -1, cv2.LINE_AA)
        cv2.circle(out, (cx, cy), dot_r, (255, 255, 255), dot_th, cv2.LINE_AA)

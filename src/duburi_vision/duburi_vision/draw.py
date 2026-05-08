"""draw -- mission-control HUD overlays for the perception pipeline.

Every glyph answers a specific operator question:
  * "Is the camera streaming?"       -> PERCEPTION panel   (strip, left)
  * "What classes am I detecting?"   -> CLASSES panel      (strip, left)
  * "How well aligned is the AUV?"   -> ALIGNMENT panel    (strip, left)
  * "What is vehicle state?"         -> STATE panel        (strip, right)
  * "Which way is the AUV heading?"  -> HEADING TAPE       (strip, bottom)
  * "How deep is the AUV?"           -> DEPTH bar          (strip, far right)
  * "Which target is being chased?"  -> primary highlight  (video)
  * "Where has the target been?"     -> motion trail       (video)
  * "Is anything broken?"            -> red STALE banner   (video)

render_all() returns np.vstack([video_frame, ui_strip]) — the output height
is frame_h + _STRIP_H. All info panels live in the strip; the video section
shows only visual overlays so the operator can read the scene without clutter.
"""

from __future__ import annotations

from typing import List, Optional

import cv2
import numpy as np

from .detection.detector import Detection, largest


# ── Palette (OpenCV BGR) — Mongla blue theme ─────────────────────────────── #
C_BG      = (26,  26,  26)    # panel fill / strip background
C_ACCENT  = (215, 130,  35)   # royal blue  titles, active, brand
C_AMBER   = ( 30, 150, 235)   # amber       arrows, warnings
C_OK      = ( 75, 200,  75)   # green       aligned, nominal
C_ERR     = ( 50,  50, 215)   # red         stale, lost, error
C_TEXT    = (225, 228, 228)   # near-white  primary values
C_DIM     = (105, 110, 110)   # gray        labels, secondary text
C_BORDER  = ( 58,  63,  63)   # dark        panel borders
C_RETICLE = ( 50,  55,  55)   # darker      reticle lines

# Legacy aliases kept for external callers
COLOR_PRIMARY   = C_ACCENT
COLOR_SECONDARY = C_DIM
COLOR_RETICLE   = C_RETICLE
COLOR_OFFSET    = C_AMBER
COLOR_OK        = C_OK
COLOR_WARN      = C_AMBER
COLOR_ERR       = C_ERR
COLOR_BG        = C_BG
COLOR_FG        = C_TEXT

# ── Typography ────────────────────────────────────────────────────────────── #
_FONT = cv2.FONT_HERSHEY_DUPLEX
_FS   = 0.40   # base font scale (video-section labels)
_FT   = 1      # font thickness (all text in this file)

# ── UI strip constants ────────────────────────────────────────────────────── #
_STRIP_H = 170   # px height of the info strip stacked below the video frame
_SFS     = 0.34  # strip panel font scale (smaller for horizontal density)
_SLH     = 13    # strip panel line height (px)
_SPAD    = 4     # strip panel inner padding (px)


# ── Supervision annotator suite (lazy-init) ───────────────────────────────── #

_SV: dict | None = None


def _get_sv() -> dict:
    """Lazy-initialize the rich supervision annotator suite."""
    global _SV
    if _SV is None:
        import supervision as sv
        _SV = {
            # Motion trail — drawn FIRST so it renders behind boxes
            'trace': sv.TraceAnnotator(
                position=sv.Position.CENTER,
                trace_length=30,
                thickness=2,
                color_lookup=sv.ColorLookup.CLASS),
            # Thin square class-colored outline
            'box': sv.BoxAnnotator(
                thickness=1,
                color_lookup=sv.ColorLookup.CLASS),
            # Thick white corner brackets — focal markers
            'corners': sv.BoxCornerAnnotator(
                thickness=4, corner_length=18,
                color=sv.Color.WHITE),
            # Triangle indicator above each box — "locked on" cue
            'triangle': sv.TriangleAnnotator(
                base=10, height=8,
                color_lookup=sv.ColorLookup.CLASS),
            # Confidence bar (thin strip above box)
            'pct_bar': sv.PercentageBarAnnotator(
                height=5, width=50,
                color_lookup=sv.ColorLookup.CLASS),
            # Class + optional track-ID label
            'label': sv.LabelAnnotator(
                text_scale=0.38, text_thickness=1,
                color_lookup=sv.ColorLookup.CLASS,
                border_radius=2,
                smart_position=False),
        }
    return _SV


def _to_sv(detections: List[Detection], track_ids=None):
    """Convert Detection list to sv.Detections, optionally with tracker_id.

    TraceAnnotator only draws trails when tracker_id is set; when track_ids is
    None (raw detections, no tracker_node) the annotator silently no-ops.
    """
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


# ── Public draw API ───────────────────────────────────────────────────────── #

def draw_detections(frame_bgr: np.ndarray,
                    detections: List[Detection]) -> np.ndarray:
    """Boxes + trail + confidence bars + labels (standalone, no primary highlight)."""
    if frame_bgr is None or not detections:
        return frame_bgr
    sv = _get_sv()
    sv_det = _to_sv(detections)
    out = frame_bgr.copy()
    out = sv['trace'].annotate(scene=out, detections=sv_det)
    out = sv['box'].annotate(scene=out, detections=sv_det)
    out = sv['corners'].annotate(scene=out, detections=sv_det)
    out = sv['triangle'].annotate(scene=out, detections=sv_det)
    out = sv['pct_bar'].annotate(scene=out, detections=sv_det)
    labels = [f'{d.class_name} {int(d.score * 100)}%' for d in detections]
    return sv['label'].annotate(scene=out, detections=sv_det, labels=labels)


def highlight_primary(frame_bgr: np.ndarray,
                      target: Optional[Detection],
                      color=None) -> np.ndarray:
    if frame_bgr is None or target is None:
        return frame_bgr
    color = color or C_ACCENT
    out = frame_bgr.copy()
    x1, y1, x2, y2 = (int(v) for v in target.xyxy)
    cv2.rectangle(out, (x1, y1), (x2, y2), color, 3, cv2.LINE_AA)
    _draw_corners(out, x1, y1, x2, y2, color, length=18, thickness=4)
    return out


def crosshair(frame_bgr: np.ndarray,
              target: Optional[Detection],
              color=None, size=14) -> np.ndarray:
    if frame_bgr is None or target is None:
        return frame_bgr
    color = color or C_ACCENT
    out = frame_bgr.copy()
    cx, cy = int(target.cx), int(target.cy)
    cv2.line(out, (cx - size, cy), (cx + size, cy), color, 2, cv2.LINE_AA)
    cv2.line(out, (cx, cy - size), (cx, cy + size), color, 2, cv2.LINE_AA)
    cv2.circle(out, (cx, cy), 3, color, -1, cv2.LINE_AA)
    return out


def dashed_reticle(frame_bgr: np.ndarray, color=None) -> np.ndarray:
    if frame_bgr is None:
        return frame_bgr
    color = color or C_RETICLE
    out = frame_bgr.copy()
    h, w = out.shape[:2]
    cx, cy = w // 2, h // 2
    _dashed_line(out, (cx, 0), (cx, h), color, dash=8, gap=6, thickness=1)
    _dashed_line(out, (0, cy), (w, cy), color, dash=8, gap=6, thickness=1)
    cv2.circle(out, (cx, cy), 4, color, 1, cv2.LINE_AA)
    cv2.circle(out, (cx, cy), 1, color, -1, cv2.LINE_AA)
    return out


def offset_arrow(frame_bgr: np.ndarray,
                 target: Optional[Detection],
                 color=None) -> np.ndarray:
    if frame_bgr is None or target is None:
        return frame_bgr
    color = color or C_AMBER
    out = frame_bgr.copy()
    h, w = out.shape[:2]
    cx, cy = w // 2, h // 2
    tx, ty = int(target.cx), int(target.cy)
    if abs(tx - cx) < 2 and abs(ty - cy) < 2:
        return out
    cv2.arrowedLine(out, (cx, cy), (tx, ty), color, 2, cv2.LINE_AA, tipLength=0.18)
    return out


def status_badge(frame_bgr: np.ndarray, *, source='?', fps=0.0,
                 device='?', n_detections=0, primary_class=None,
                 healthy=True) -> np.ndarray:
    """Standalone PERCEPTION panel (render_all draws it in the strip)."""
    if frame_bgr is None:
        return frame_bgr
    out = frame_bgr.copy()
    rows = [
        ('SRC', str(source), C_TEXT),
        ('FPS', f'{fps:.1f}', C_TEXT),
        ('DET', str(n_detections), C_ACCENT if n_detections else C_DIM),
    ]
    if primary_class:
        rows.append(('TGT', str(primary_class), C_ACCENT))
    border = C_OK if (healthy and n_detections > 0) else (C_ERR if not healthy else C_BORDER)
    _mc_panel(out, 8, 8, 'PERCEPTION', rows, border=border)
    return out


def alignment_readout(frame_bgr: np.ndarray,
                      target: Optional[Detection],
                      *, deadband=0.05) -> np.ndarray:
    """Standalone ALIGNMENT panel (render_all draws it in the strip)."""
    if frame_bgr is None:
        return frame_bgr
    out = frame_bgr.copy()
    h, w = out.shape[:2]
    if target is None:
        rows = [('STATUS', 'NO TARGET', C_ERR)]
        _mc_panel(out, 8, _panel_bottom_y(h, 1), 'ALIGNMENT', rows, border=C_ERR)
        return out
    ex = (target.cx - w / 2.0) / max(w / 2.0, 1.0)
    ey = (target.cy - h / 2.0) / max(h / 2.0, 1.0)
    aligned = abs(ex) < deadband and abs(ey) < deadband
    rows = [
        ('ERR_X', f'{ex:+.3f}', C_OK if abs(ex) < deadband else C_AMBER),
        ('ERR_Y', f'{ey:+.3f}', C_OK if abs(ey) < deadband else C_AMBER),
        ('AREA',  f'{(target.area / (w * h) * 100):.2f}%', C_TEXT),
        ('CONF',  f'{target.score:.2f}', C_TEXT),
    ]
    _mc_panel(out, 8, _panel_bottom_y(h, len(rows)), 'ALIGNMENT', rows,
              border=C_OK if aligned else C_AMBER)
    return out


def draw_vehicle_state(frame_bgr: np.ndarray, state) -> np.ndarray:
    """Standalone STATE panel. state: DuburiState or compatible."""
    if frame_bgr is None:
        return frame_bgr
    out = frame_bgr.copy()
    h, w = out.shape[:2]
    if state is None:
        _mc_panel(out, w - 145, 8, 'STATE', [('no telemetry', '', C_DIM)])
        return out
    armed = bool(state.armed)
    bv    = float(state.battery_voltage)
    rows = [
        ('DEPTH', f'{state.depth_m:+.2f}m', C_TEXT),
        ('YAW',   f'{state.yaw_deg:.1f}',   C_TEXT),
        ('MODE',  state.mode or '?',         C_ACCENT if armed else C_DIM),
        ('BATT',  f'{bv:.1f}V',              _batt_color(bv)),
        ('ARMED', 'YES' if armed else 'no',  C_OK if armed else C_DIM),
    ]
    x = _panel_right_x(w, 'STATE', rows)
    _mc_panel(out, x, 8, 'STATE', rows, border=C_OK if armed else C_BORDER)
    return out


def draw_depth_gauge(img: np.ndarray, depth_m: float,
                     x: int, y: int,
                     gauge_h: int = 200, gauge_w: int = 22,
                     max_depth: float = 10.0) -> None:
    """Vertical depth slider drawn in-place. 0m at top, max_depth at bottom."""
    if np.isnan(depth_m):
        return
    depth_abs = float(min(abs(depth_m), max_depth))

    overlay = img.copy()
    cv2.rectangle(overlay, (x, y), (x + gauge_w, y + gauge_h), C_BG, -1)
    cv2.addWeighted(overlay, 0.75, img, 0.25, 0, dst=img)
    cv2.rectangle(img, (x, y), (x + gauge_w, y + gauge_h), C_BORDER, 1, cv2.LINE_AA)
    cv2.putText(img, 'D', (x + 3, y - 3), _FONT, 0.28, C_DIM, 1, cv2.LINE_AA)

    for m in range(0, int(max_depth) + 1):
        yt = y + int(m / max_depth * gauge_h)
        tick_len = 6 if m % 2 == 0 else 3
        cv2.line(img, (x, yt), (x + tick_len, yt), C_DIM, 1)
        if m % 2 == 0 and m > 0:
            cv2.putText(img, str(m), (x - 12, yt + 4),
                        _FONT, 0.26, C_DIM, 1, cv2.LINE_AA)

    ind_y = y + int(depth_abs / max_depth * gauge_h)
    ind_y = max(y + 2, min(y + gauge_h - 2, ind_y))
    color = C_OK if depth_abs < 3.0 else (C_AMBER if depth_abs < 7.0 else C_ERR)
    cv2.line(img, (x, ind_y), (x + gauge_w, ind_y), color, 2, cv2.LINE_AA)
    label = f'{depth_abs:.1f}m'
    (tw, _), _ = cv2.getTextSize(label, _FONT, 0.28, 1)
    cv2.putText(img, label, (x - tw - 2, ind_y + 4),
                _FONT, 0.28, color, 1, cv2.LINE_AA)


def draw_heading_tape(img: np.ndarray, yaw_deg: float,
                      x: int, y: int,
                      tape_w: int = 260, tape_h: int = 30,
                      show_readout: bool = True) -> None:
    """Horizontal compass tape drawn in-place. Shows ±60° around current heading."""
    cv2.rectangle(img, (x, y), (x + tape_w, y + tape_h), C_BG, -1)
    cv2.rectangle(img, (x, y), (x + tape_w, y + tape_h), C_BORDER, 1, cv2.LINE_AA)
    if np.isnan(yaw_deg):
        msg = 'NO HDG'
        (tw, _), _ = cv2.getTextSize(msg, _FONT, 0.30, 1)
        cv2.putText(img, msg, (x + (tape_w - tw) // 2, y + tape_h // 2 + 5),
                    _FONT, 0.30, C_DIM, 1, cv2.LINE_AA)
        return
    yaw = float(yaw_deg) % 360.0
    cx  = x + tape_w // 2
    pixels_per_deg = tape_w / 120.0

    for delta in range(-65, 66, 10):
        deg_at = int(yaw + delta) % 360
        px = cx + int(delta * pixels_per_deg)
        if px < x + 2 or px > x + tape_w - 2:
            continue
        is_label = delta % 30 == 0
        tick_h = 8 if is_label else 4
        cv2.line(img, (px, y + tape_h - tick_h), (px, y + tape_h - 1), C_DIM, 1)
        if is_label:
            label = _cardinal(deg_at)
            (tw, _), _ = cv2.getTextSize(label, _FONT, 0.30, 1)
            cv2.putText(img, label, (px - tw // 2, y + tape_h - tick_h - 2),
                        _FONT, 0.30, C_TEXT, 1, cv2.LINE_AA)

    cv2.line(img, (cx, y + 2), (cx, y + tape_h - 2), C_ACCENT, 2, cv2.LINE_AA)

    if show_readout:
        hdg_label = f'{int(yaw) % 360:03d}'
        (tw, th), _ = cv2.getTextSize(hdg_label, _FONT, _FS, _FT)
        lx = cx - tw // 2
        ly = y - 3
        cv2.putText(img, hdg_label, (lx, ly), _FONT, _FS, C_ACCENT, _FT, cv2.LINE_AA)
        cv2.circle(img, (lx + tw + 4, ly - th + 3), 2, C_ACCENT, 1, cv2.LINE_AA)


def draw_classes_panel(img: np.ndarray,
                       configured_classes: List[str],
                       active_class_names,
                       x: int, y: int) -> None:
    """CLASSES panel drawn in-place on img at (x, y) with default panel sizing."""
    if not configured_classes:
        return
    active_lower = {n.lower() for n in active_class_names}
    any_active   = any(c.lower() in active_lower for c in configured_classes)
    rows = [
        ('', c.upper(), C_ACCENT if c.lower() in active_lower else C_DIM)
        for c in configured_classes
    ]
    _mc_panel(img, x, y, 'CLASSES', rows,
              border=C_ACCENT if any_active else C_BORDER)


def draw_compass_needle(img: np.ndarray, yaw_deg: float,
                        cx: int, cy: int, radius: int = 20) -> None:
    """Compass rose with directional needle drawn in-place."""
    overlay = img.copy()
    cv2.circle(overlay, (cx, cy), radius, C_BG, -1)
    cv2.addWeighted(overlay, 0.80, img, 0.20, 0, dst=img)
    cv2.circle(img, (cx, cy), radius, C_BORDER, 1, cv2.LINE_AA)

    for ang in (0, 90, 180, 270):
        rad = np.radians(ang)
        tx = cx + int((radius - 4) * np.sin(rad))
        ty = cy - int((radius - 4) * np.cos(rad))
        cv2.circle(img, (tx, ty), 1, C_DIM, -1)
    cv2.putText(img, 'N', (cx - 4, cy - radius + 9),
                _FONT, 0.25, C_DIM, 1, cv2.LINE_AA)

    if np.isnan(yaw_deg):
        cv2.putText(img, '?', (cx - 4, cy + 5), _FONT, 0.35, C_DIM, 1, cv2.LINE_AA)
        return

    needle = radius - 4
    rad = np.radians(float(yaw_deg))
    tip_x  = cx + int(needle         * np.sin(rad))
    tip_y  = cy - int(needle         * np.cos(rad))
    tail_x = cx - int((needle // 2)  * np.sin(rad))
    tail_y = cy + int((needle // 2)  * np.cos(rad))
    cv2.line(img, (tail_x, tail_y), (tip_x, tip_y), C_ACCENT, 2, cv2.LINE_AA)
    cv2.circle(img, (tip_x, tip_y), 2, C_ACCENT, -1, cv2.LINE_AA)
    cv2.circle(img, (cx, cy), 2, C_DIM, -1, cv2.LINE_AA)


def draw_sensors_panel(img: np.ndarray, yaw_source: str,
                       yaw_deg: float, x: int, y: int) -> None:
    """Heading-source status panel drawn in-place (standalone)."""
    _SOURCE_LABELS = {
        'mavlink_ahrs': 'MAVLINK',
        'bno085':       'BNO085',
        'dvl':          'DVL',
        'bno085_dvl':   'BNO+DVL',
    }
    src = _SOURCE_LABELS.get(yaw_source, yaw_source.upper() if yaw_source else '?')
    if np.isnan(yaw_deg):
        status, st_col = 'NO DATA', C_ERR
    else:
        status, st_col = 'ACTIVE', C_OK
    rows = [
        ('SRC', src,    C_TEXT),
        ('HDG', status, st_col),
    ]
    _mc_panel(img, x, y, 'HEADING SRC', rows, border=st_col)


def stale_banner(frame_bgr: np.ndarray, message='STALE FRAME') -> np.ndarray:
    if frame_bgr is None:
        return frame_bgr
    out = frame_bgr.copy()
    h, w = out.shape[:2]
    cv2.rectangle(out, (0, 0), (w, 28), C_ERR, -1)
    cv2.putText(out, message, (8, 20), _FONT, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def render_all(frame_bgr: np.ndarray,
               detections: List[Detection], *,
               source='?', fps=0.0, device='?',
               healthy=True, show_reticle=True, show_alignment=True,
               deadband=0.05, primary: Optional[Detection] = None,
               tracking_on=False, n_tracks=0, primary_track_id=None,
               state=None,
               configured_classes=None,
               track_ids=None,
               yaw_source=None) -> np.ndarray:
    """Full mission-control overlay.

    Returns np.vstack([video_section, ui_strip]).
    Output height = frame_h + _STRIP_H (150 px).  The video section carries
    only visual overlays (reticle, trails, boxes, primary highlight, stale
    banner).  All text panels live in the strip so the live feed stays clean.

    Parameters
    ----------
    configured_classes : list[str] | None
        Classes the detector is configured for (from classes_filter topic).
    track_ids : list[int] | None
        Parallel tracker IDs for each entry in `detections`.
        TraceAnnotator trails are drawn only when this is set (tracker running).
    """
    if frame_bgr is None:
        return None

    out = frame_bgr.copy()
    h, w = out.shape[:2]

    # ── 1. Reticle + deadband zone ────────────────────────────────────────── #
    if show_reticle:
        cx, cy = w // 2, h // 2
        _dashed_line(out, (cx, 0), (cx, h), C_RETICLE, dash=8, gap=6, thickness=1)
        _dashed_line(out, (0, cy), (w, cy), C_RETICLE, dash=8, gap=6, thickness=1)
        cv2.circle(out, (cx, cy), 4, C_RETICLE, 1, cv2.LINE_AA)
        cv2.circle(out, (cx, cy), 1, C_RETICLE, -1, cv2.LINE_AA)
        db_px = int(deadband * w / 2)
        db_py = int(deadband * h / 2)
        cv2.rectangle(out, (cx - db_px, cy - db_py), (cx + db_px, cy + db_py),
                      C_RETICLE, 1, cv2.LINE_AA)

    # ── 2. Trails + detection boxes (supervision annotators) ─────────────── #
    primary = primary or largest(detections)
    if detections:
        sv = _get_sv()
        sv_all = _to_sv(detections, track_ids)
        if sv_all.tracker_id is not None:
            out = sv['trace'].annotate(scene=out, detections=sv_all)
        out = sv['box'].annotate(scene=out, detections=sv_all)
        out = sv['corners'].annotate(scene=out, detections=sv_all)
        out = sv['triangle'].annotate(scene=out, detections=sv_all)
        out = sv['pct_bar'].annotate(scene=out, detections=sv_all)
        labels = []
        for i, d in enumerate(detections):
            tid = (track_ids[i] if track_ids and i < len(track_ids)
                   and track_ids[i] is not None else None)
            prefix = f'#{tid} ' if tid is not None else ''
            labels.append(f'{prefix}{d.class_name} {int(d.score * 100)}%')
        out = sv['label'].annotate(scene=out, detections=sv_all, labels=labels)

    # ── 3. Primary target: accent border, full-axis hairlines, size tag, alignment bar ── #
    if primary is not None:
        x1p, y1p, x2p, y2p = (int(v) for v in primary.xyxy)
        cx_t, cy_t = int(primary.cx), int(primary.cy)

        # Accent border + corner brackets + centre dot
        cv2.rectangle(out, (x1p, y1p), (x2p, y2p), C_ACCENT, 2, cv2.LINE_AA)
        _draw_corners(out, x1p, y1p, x2p, y2p, C_ACCENT, length=15, thickness=3)
        cv2.circle(out, (cx_t, cy_t), 3, C_ACCENT, -1, cv2.LINE_AA)

        # Full-axis hairlines: dashed from box edges to frame boundary.
        # Conveys offset from frame centre more clearly than a short crosshair.
        _dashed_line(out, (cx_t, 0),    (cx_t, y1p), C_ACCENT, dash=5, gap=5, thickness=1)
        _dashed_line(out, (cx_t, y2p),  (cx_t, h),   C_ACCENT, dash=5, gap=5, thickness=1)
        _dashed_line(out, (0,    cy_t), (x1p,  cy_t), C_ACCENT, dash=5, gap=5, thickness=1)
        _dashed_line(out, (x2p,  cy_t), (w,    cy_t), C_ACCENT, dash=5, gap=5, thickness=1)

        # Size + aspect-ratio tag: operator can estimate bbox framing at a glance
        ar       = primary.width / max(primary.height, 1.0)
        p_w_pct  = int(primary.width  / max(w, 1) * 100)
        p_h_pct  = int(primary.height / max(h, 1) * 100)
        size_lbl = f'W:{p_w_pct}%  H:{p_h_pct}%  ar:{ar:.2f}'
        (slw, slh), _ = cv2.getTextSize(size_lbl, _FONT, 0.32, 1)
        sl_x = max(x1p, 2)
        sl_y = (y2p + slh + 4) if (y2p + slh + 8 < h) else (y1p - 4)
        cv2.rectangle(out, (sl_x - 2, sl_y - slh - 1), (sl_x + slw + 2, sl_y + 2),
                      C_BG, -1)
        cv2.putText(out, size_lbl, (sl_x, sl_y), _FONT, 0.32, C_DIM, 1, cv2.LINE_AA)

        # Horizontal alignment bar at frame bottom: analogue offset gauge.
        # Dot position → how far the target is from horizontal centre.
        _bar_y = h - 12
        _bar_h = 7
        ex = (primary.cx - w / 2.0) / max(w / 2.0, 1.0)
        bar_col = C_OK if abs(ex) < deadband else C_AMBER
        cv2.rectangle(out, (0, _bar_y), (w, _bar_y + _bar_h), C_BG, -1)
        cv2.line(out, (w // 2, _bar_y), (w // 2, _bar_y + _bar_h), C_RETICLE, 1)
        db_px = int(deadband * w / 2)
        cv2.rectangle(out, (w // 2 - db_px, _bar_y),
                      (w // 2 + db_px, _bar_y + _bar_h), C_RETICLE, 1)
        dot_x = max(4, min(w - 4, cx_t))
        cv2.line(out, (w // 2, _bar_y + _bar_h // 2),
                 (dot_x, _bar_y + _bar_h // 2), bar_col, 1, cv2.LINE_AA)
        cv2.circle(out, (dot_x, _bar_y + _bar_h // 2), 3, bar_col, -1, cv2.LINE_AA)

    # ── 4. Stale banner ───────────────────────────────────────────────────── #
    if not healthy:
        cv2.rectangle(out, (0, 0), (w, 28), C_ERR, -1)
        cv2.putText(out, 'STALE FRAME', (8, 20),
                    _FONT, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    # ── 5. UI strip (all panels) ─────────────────────────────────────────── #
    strip = _render_ui_strip(
        w, h,
        detections=detections,
        primary=primary,
        source=source, fps=fps, healthy=healthy,
        tracking_on=tracking_on, n_tracks=n_tracks,
        primary_track_id=primary_track_id,
        configured_classes=configured_classes or [],
        state=state,
        yaw_source=yaw_source or '',
        deadband=deadband,
        show_alignment=show_alignment,
    )

    return np.vstack([out, strip])


# ── UI strip renderer ─────────────────────────────────────────────────────── #

def _render_ui_strip(w: int, frame_h: int, *,
                     detections, primary, source, fps, healthy,
                     tracking_on, n_tracks, primary_track_id,
                     configured_classes, state, yaw_source,
                     deadband, show_alignment) -> np.ndarray:
    """Build the _STRIP_H×w info panel below the video frame.

    Layout (left → right):
      [PERCEPTION] [CLASSES] [ALIGNMENT]   <gap>   [STATE] [HDG SRC] [◎] [depth]
    Footer: full-width heading tape.
    """
    strip = np.full((_STRIP_H, w, 3), C_BG, dtype=np.uint8)

    # Top accent bar
    cv2.line(strip, (0, 0), (w - 1, 0), C_ACCENT, 2)

    # ── Brand header ─────────────────────────────────────────────────────── #
    brand = 'BRACU  DUBURI'
    (bw_px, bh_px), _ = cv2.getTextSize(brand, _FONT, 0.44, 1)
    bx = (w - bw_px) // 2
    by = 16
    cv2.putText(strip, brand, (bx, by), _FONT, 0.44, C_TEXT, 1, cv2.LINE_AA)
    dot_y = by - bh_px // 2
    cv2.circle(strip, (bx - 10, dot_y), 3, C_ACCENT, -1, cv2.LINE_AA)
    cv2.circle(strip, (bx + bw_px + 10, dot_y), 3, C_ACCENT, -1, cv2.LINE_AA)

    # Separator under header
    _sep_y = 21
    cv2.line(strip, (6, _sep_y), (w - 6, _sep_y), C_BORDER, 1)

    # Panel row y-anchor (top of panels)
    py = 25

    # ── PERCEPTION panel (left) ───────────────────────────────────────────── #
    perc_rows = [
        ('SRC', str(source),          C_TEXT),
        ('FPS', f'{fps:.1f}',         C_TEXT),
        ('DET', str(len(detections)), C_ACCENT if detections else C_DIM),
    ]
    if primary is not None:
        perc_rows.append(('TGT', primary.class_name, C_ACCENT))
    elif tracking_on:
        perc_rows.append(('TRK', f'n={n_tracks}', C_ACCENT))
    badge_border = (C_OK if (healthy and detections)
                    else (C_ERR if not healthy else C_BORDER))
    px = 6
    _mc_panel(strip, px, py, 'PERCEPTION', perc_rows,
              border=badge_border, pad=_SPAD, line_h=_SLH, fs=_SFS)
    px += _panel_width('PERCEPTION', perc_rows, pad=_SPAD, fs=_SFS) + 6

    # ── CLASSES panel ─────────────────────────────────────────────────────── #
    if configured_classes:
        active_names  = {d.class_name for d in detections}
        active_lower  = {n.lower() for n in active_names}
        any_active    = any(c.lower() in active_lower for c in configured_classes)
        cls_rows = [
            ('', c.upper(), C_ACCENT if c.lower() in active_lower else C_DIM)
            for c in configured_classes
        ]
        _mc_panel(strip, px, py, 'CLASSES', cls_rows,
                  border=C_ACCENT if any_active else C_BORDER,
                  pad=_SPAD, line_h=_SLH, fs=_SFS)
        px += _panel_width('CLASSES', cls_rows, pad=_SPAD, fs=_SFS) + 6

    # ── ALIGNMENT panel ───────────────────────────────────────────────────── #
    if show_alignment:
        if primary is None:
            al_rows   = [('STATUS', 'NO TARGET', C_ERR)]
            al_border = C_ERR
        else:
            ex = (primary.cx - w / 2.0) / max(w / 2.0, 1.0)
            ey = (primary.cy - frame_h / 2.0) / max(frame_h / 2.0, 1.0)
            aligned = abs(ex) < deadband and abs(ey) < deadband

            if tracking_on and n_tracks > 0:
                tid_str    = f'#{primary_track_id}' if primary_track_id is not None else f'n={n_tracks}'
                status_val = f'TRACKED {tid_str}'
                status_col = C_ACCENT
            else:
                status_val = 'DETECTED'
                status_col = C_TEXT

            al_rows = [
                ('STATUS', status_val,            status_col),
                ('ERR_X',  f'{ex:+.3f}', C_OK if abs(ex) < deadband else C_AMBER),
                ('ERR_Y',  f'{ey:+.3f}', C_OK if abs(ey) < deadband else C_AMBER),
                ('AREA',   f'{(primary.area / (w * frame_h) * 100):.1f}%', C_TEXT),
                ('CONF',   f'{primary.score:.2f}',                          C_TEXT),
            ]
            al_border = C_OK if aligned else C_AMBER

        _mc_panel(strip, px, py, 'ALIGNMENT', al_rows,
                  border=al_border, pad=_SPAD, line_h=_SLH, fs=_SFS)
        px += _panel_width('ALIGNMENT', al_rows, pad=_SPAD, fs=_SFS) + 6

    # ── TRACKS panel (only when tracker is running) ───────────────────────── #
    if tracking_on or n_tracks > 0:
        trk_rows = [
            ('TRACKS',  str(n_tracks),
             C_ACCENT if n_tracks else C_DIM),
            ('PRIMARY', f'#{primary_track_id}' if primary_track_id is not None else 'none',
             C_ACCENT if primary_track_id is not None else C_DIM),
        ]
        _mc_panel(strip, px, py, 'TRACKS', trk_rows,
                  border=C_ACCENT if n_tracks else C_BORDER,
                  pad=_SPAD, line_h=_SLH, fs=_SFS)

    # ── Right block (anchored from right edge) ─────────────────────────────── #
    # Depth gauge: 22×72 px, always visible (border + ticks even with no data)
    # Scale 0-5 m covers typical AUV operating depth; STATE panel shows exact value.
    _DG_W  = 22
    _DG_H  = 72
    _MAX_D = 5.0
    _dg_x  = w - 6 - _DG_W
    _dg_y  = py + 4

    cv2.rectangle(strip, (_dg_x, _dg_y), (_dg_x + _DG_W, _dg_y + _DG_H), C_BORDER, 1, cv2.LINE_AA)
    cv2.putText(strip, 'DEP', (_dg_x + 1, _dg_y - 2), _FONT, 0.22, C_DIM, 1, cv2.LINE_AA)
    for _m in (0, 2, 4, 5):
        _yt = _dg_y + int(_m / _MAX_D * _DG_H)
        cv2.line(strip, (_dg_x, _yt), (_dg_x + 5, _yt), C_DIM, 1)
        cv2.putText(strip, f'{_m}m', (_dg_x + 7, min(_yt + 4, _dg_y + _DG_H - 1)),
                    _FONT, 0.22, C_DIM, 1, cv2.LINE_AA)

    if state is not None and not np.isnan(state.depth_m):
        depth_abs = float(min(abs(state.depth_m), _MAX_D))
        _ind_y    = _dg_y + int(depth_abs / _MAX_D * _DG_H)
        _ind_y    = max(_dg_y + 1, min(_dg_y + _DG_H - 1, _ind_y))
        d_col     = C_OK if depth_abs < 2.0 else (C_AMBER if depth_abs < 4.0 else C_ERR)
        if depth_abs > 0.05:
            cv2.rectangle(strip, (_dg_x + 1, _dg_y + 1), (_dg_x + _DG_W - 1, _ind_y), d_col, -1)
        cv2.line(strip, (_dg_x, _ind_y), (_dg_x + _DG_W, _ind_y), d_col, 2, cv2.LINE_AA)
    else:
        cv2.putText(strip, '?', (_dg_x + 6, _dg_y + _DG_H // 2 + 4),
                    _FONT, 0.32, C_DIM, 1, cv2.LINE_AA)

    # Compass needle: 40px diameter, to the left of depth bar
    _CMP_R  = 20
    _cmp_cx = _dg_x - 8 - _CMP_R
    _cmp_cy = py + _DG_H // 2 + 4
    _yaw    = state.yaw_deg if state is not None else float('nan')
    draw_compass_needle(strip, _yaw, cx=_cmp_cx, cy=_cmp_cy, radius=_CMP_R)

    # HDG SRC panel: to the left of compass
    _SOURCE_LABELS = {
        'mavlink_ahrs': 'MAVLINK',
        'bno085':       'BNO085',
        'dvl':          'DVL',
        'bno085_dvl':   'BNO+DVL',
    }
    src_label = _SOURCE_LABELS.get(yaw_source, yaw_source.upper() if yaw_source else '?')
    if np.isnan(_yaw):
        hdg_status, hdg_col = 'NO DATA', C_ERR
    else:
        hdg_status, hdg_col = 'ACTIVE', C_OK
    src_rows = [
        ('SRC', src_label,  C_TEXT),
        ('HDG', hdg_status, hdg_col),
    ]
    src_w = _panel_width('HEADING SRC', src_rows, pad=_SPAD, fs=_SFS)
    src_x = _cmp_cx - _CMP_R - 8 - src_w
    _mc_panel(strip, src_x, py, 'HEADING SRC', src_rows,
              border=hdg_col, pad=_SPAD, line_h=_SLH, fs=_SFS)

    # STATE panel: to the left of HDG SRC panel
    if state is not None:
        armed = bool(state.armed)
        bv    = float(state.battery_voltage)
        st_rows = [
            ('DEPTH', f'{state.depth_m:+.2f}m', C_TEXT),
            ('YAW',   f'{state.yaw_deg:.1f}',   C_TEXT),
            ('MODE',  state.mode or '?',          C_ACCENT if armed else C_DIM),
            ('BATT',  f'{bv:.1f}V',               _batt_color(bv)),
            ('ARMED', 'YES' if armed else 'no',   C_OK if armed else C_DIM),
        ]
        st_w = _panel_width('STATE', st_rows, pad=_SPAD, fs=_SFS)
        st_x = src_x - 8 - st_w
        _mc_panel(strip, st_x, py, 'STATE', st_rows,
                  border=C_OK if armed else C_BORDER, pad=_SPAD, line_h=_SLH, fs=_SFS)
        # YAW degree mark next to the YAW value
        # (row 2 in STATE panel, baseline at py + pad + line_h*3 - 3)
        yaw_row_y = py + _SPAD + _SLH * 3 - 3
        yaw_val   = f'{state.yaw_deg:.1f}'
        (yw, yh), _ = cv2.getTextSize(yaw_val, _FONT, _SFS, 1)
        # key column width in STATE panel
        key_ws_st = [cv2.getTextSize(r[0], _FONT, _SFS, 1)[0][0]
                     for r in st_rows if not isinstance(r, str)]
        col_w_st  = (max(key_ws_st) + 8) if key_ws_st else 0
        val_x_st  = st_x + _SPAD + col_w_st
        cv2.circle(strip, (val_x_st + yw + 3, yaw_row_y - yh + 2),
                   2, C_TEXT, 1, cv2.LINE_AA)

    # Heading tape — taller tape, always renders (NO HDG when yaw is NaN)
    _TAPE_H    = 44
    tape_y     = _STRIP_H - 2 - _TAPE_H   # STRIP_H=170 → tape at y=124
    tape_sep_y = tape_y - 4
    cv2.putText(strip, 'HDG', (8, tape_sep_y - 2), _FONT, 0.24, C_DIM, 1, cv2.LINE_AA)
    cv2.line(strip, (6, tape_sep_y), (w - 6, tape_sep_y), C_BORDER, 1)

    draw_heading_tape(strip, _yaw, x=6, y=tape_y,
                      tape_w=w - 12, tape_h=_TAPE_H, show_readout=False)

    # Accent bar at the very bottom
    cv2.line(strip, (0, _STRIP_H - 1), (w - 1, _STRIP_H - 1), C_ACCENT, 1)

    return strip


def draw_track_ids(frame_bgr: np.ndarray, tracks) -> np.ndarray:
    """Overlay stable track IDs (legacy helper for track-object display)."""
    if frame_bgr is None or not tracks:
        return frame_bgr

    _PALETTE = [
        (255, 100,  50), ( 50, 220, 100), ( 50, 100, 255),
        (255, 200,  50), (180,  50, 255), ( 50, 255, 220),
        (200, 130, 255), ( 80, 255, 130), (255, 130, 200),
        (130, 200, 255), (255,  80, 130), (130, 255,  80),
    ]

    out = frame_bgr.copy()
    for td in tracks:
        color = _PALETTE[abs(int(td.track_id)) % len(_PALETTE)]
        x1, y1, x2, y2 = (int(v) for v in td.xyxy)
        thickness = 1 if td.predicted else 2
        alpha     = 0.45 if td.predicted else 1.0

        if alpha < 1.0:
            overlay = out.copy()
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, thickness, cv2.LINE_AA)
            cv2.addWeighted(overlay, alpha, out, 1.0 - alpha, 0, out)
        else:
            cv2.rectangle(out, (x1, y1), (x2, y2), color, thickness, cv2.LINE_AA)

        label = f"#{td.track_id} {td.class_name}" + (' (pred)' if td.predicted else '')
        cv2.putText(out, label, (x1, max(y1 - 5, 12)),
                    _FONT, 0.36, color, 1, cv2.LINE_AA)

    return out


# ── Internal helpers ──────────────────────────────────────────────────────── #

def _mc_panel(img, x, y, title, rows, *,
              border=None, pad=5, line_h=17, fs=None):
    """Mission-control panel: optional title bar + aligned key-value rows.

    rows: list of
        str                  — full-width text in C_TEXT
        (label, value)       — label in C_DIM, value in C_TEXT
        (label, value, color)— label in C_DIM, value in given color

    fs : float | None
        Font scale override. Defaults to module-level _FS (0.40).
        Pass _SFS (0.34) for strip panels to keep them compact.
    """
    fs     = fs if fs is not None else _FS
    border = border or C_BORDER

    key_ws = [cv2.getTextSize(r[0], _FONT, fs, _FT)[0][0]
              for r in rows if not isinstance(r, str) and len(r) >= 2]
    col_w = (max(key_ws) + 8) if key_ws else 0

    candidates = []
    if title:
        candidates.append(cv2.getTextSize(title, _FONT, fs, _FT)[0][0])
    for row in rows:
        if isinstance(row, str):
            candidates.append(cv2.getTextSize(row, _FONT, fs, _FT)[0][0])
        elif len(row) >= 2:
            val_w = cv2.getTextSize(row[1], _FONT, fs, _FT)[0][0]
            candidates.append(col_w + val_w)
    panel_w = (max(candidates) if candidates else 60) + 2 * pad
    panel_h = line_h * (len(rows) + (1 if title else 0)) + 2 * pad

    overlay = img.copy()
    cv2.rectangle(overlay, (x, y), (x + panel_w, y + panel_h), C_BG, -1)
    cv2.addWeighted(overlay, 0.82, img, 0.18, 0, dst=img)
    cv2.rectangle(img, (x, y), (x + panel_w, y + panel_h), border, 1, cv2.LINE_AA)

    row_offset = 0
    if title:
        ty = y + pad + line_h - 3
        cv2.putText(img, title, (x + pad, ty), _FONT, fs, C_ACCENT, _FT, cv2.LINE_AA)
        sep_y = y + pad + line_h + 1
        cv2.line(img, (x + 1, sep_y), (x + panel_w - 1, sep_y), C_BORDER, 1)
        row_offset = 1

    for i, row in enumerate(rows):
        ry = y + pad + line_h * (i + row_offset + 1) - 3
        if isinstance(row, str):
            cv2.putText(img, row, (x + pad, ry), _FONT, fs, C_TEXT, _FT, cv2.LINE_AA)
        else:
            label     = row[0]
            val       = row[1] if len(row) > 1 else ''
            val_color = row[2] if len(row) > 2 else C_TEXT
            cv2.putText(img, label, (x + pad, ry),
                        _FONT, fs, C_DIM, _FT, cv2.LINE_AA)
            cv2.putText(img, val, (x + pad + col_w, ry),
                        _FONT, fs, val_color, _FT, cv2.LINE_AA)


def _panel_width(title: str, rows, pad: int = 5, fs: float | None = None) -> int:
    """Compute the pixel width of a _mc_panel without rendering it."""
    fs = fs if fs is not None else _FS
    key_ws = [cv2.getTextSize(r[0], _FONT, fs, _FT)[0][0]
              for r in rows if not isinstance(r, str) and len(r) >= 2]
    col_w = (max(key_ws) + 8) if key_ws else 0
    candidates = []
    if title:
        candidates.append(cv2.getTextSize(title, _FONT, fs, _FT)[0][0])
    for row in rows:
        if isinstance(row, str):
            candidates.append(cv2.getTextSize(row, _FONT, fs, _FT)[0][0])
        elif len(row) >= 2:
            val_w = cv2.getTextSize(row[1], _FONT, fs, _FT)[0][0]
            candidates.append(col_w + val_w)
    return (max(candidates) if candidates else 60) + 2 * pad


def _panel(img, top_left, lines, *, fg, bg, border, pad=6, line_h=16):
    """Legacy panel shim."""
    x, y = top_left
    _mc_panel(img, x, y, '', [str(l) for l in lines],
              border=border, pad=pad, line_h=line_h)


def _panel_bottom_y(frame_h: int, n_data_rows: int,
                    pad=5, line_h=17, margin=8) -> int:
    panel_h = line_h * (n_data_rows + 1) + 2 * pad
    return max(frame_h - panel_h - margin, margin)


def _panel_right_x(frame_w: int, title: str, rows,
                   pad=5, margin=8, fs=None) -> int:
    fs = fs if fs is not None else _FS
    panel_w = _panel_width(title, rows, pad=pad, fs=fs)
    return max(frame_w // 2, frame_w - panel_w - margin)


def _batt_color(voltage: float):
    if voltage > 14.0:
        return C_OK
    if voltage > 12.5:
        return C_AMBER
    return C_ERR


def _cardinal(deg: int) -> str:
    return {0: 'N', 90: 'E', 180: 'S', 270: 'W'}.get(deg % 360, f'{deg % 360:03d}')


def _draw_corners(img, x1, y1, x2, y2, color, length=14, thickness=3):
    for (px, py, dx, dy2) in (
        (x1, y1, +1, +1),
        (x2, y1, -1, +1),
        (x1, y2, +1, -1),
        (x2, y2, -1, -1),
    ):
        cv2.line(img, (px, py), (px + dx * length, py),
                 color, thickness, cv2.LINE_AA)
        cv2.line(img, (px, py), (px, py + dy2 * length),
                 color, thickness, cv2.LINE_AA)


def _dashed_line(img, p1, p2, color, dash=10, gap=6, thickness=1):
    x1, y1 = p1
    x2, y2 = p2
    length = max(int(np.hypot(x2 - x1, y2 - y1)), 1)
    step = dash + gap
    n = length // step
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

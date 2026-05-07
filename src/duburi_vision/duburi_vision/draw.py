"""draw -- mission-control HUD overlays for the perception pipeline.

Every glyph answers a specific operator question:
  * "Is the camera streaming?"       -> PERCEPTION panel (top-left)
  * "What classes am I detecting?"   -> CLASSES panel (below PERCEPTION)
  * "Which target is being chased?"  -> primary highlight + crosshair + arrow
  * "How well aligned is the AUV?"   -> ALIGNMENT panel (bottom-left)
  * "Is tracking active?"            -> STATUS row in ALIGNMENT panel
  * "What is vehicle state?"         -> STATE panel (top-right, if telemetry available)
  * "How deep is the AUV?"           -> DEPTH GAUGE (right edge, if telemetry available)
  * "Which way is the AUV heading?"  -> HEADING TAPE (bottom-center, if telemetry available)
  * "Is anything broken?"            -> red STALE banner

All functions are pure (frame in → annotated copy out) except the
instrument helpers (draw_depth_gauge, draw_heading_tape, draw_classes_panel,
_mc_panel) which modify the frame in-place — they are only called from
render_all which already works on a copy.
"""

from __future__ import annotations

from typing import List, Optional

import cv2
import numpy as np

from .detection.detector import Detection, largest


# ── Palette (OpenCV BGR) — Mongla brand-inspired ──────────────────────────── #
C_BG      = (26,  26,  26)    # #1a1a1a   panel fill
C_ACCENT  = (200, 185,   0)   # teal/cyan primary target, active, section titles
C_AMBER   = ( 30, 150, 235)   # amber     arrows, warnings
C_OK      = ( 75, 200,  75)   # green     aligned, nominal
C_ERR     = ( 50,  50, 215)   # red       stale, lost, error
C_TEXT    = (225, 228, 228)   # near-white primary values
C_DIM     = (105, 110, 110)   # gray      labels, secondary text
C_BORDER  = ( 58,  63,  63)   # dark      panel borders
C_RETICLE = ( 50,  55,  55)   # darker    reticle lines

# Legacy aliases kept for any external callers
COLOR_PRIMARY   = C_ACCENT
COLOR_SECONDARY = C_DIM
COLOR_RETICLE   = C_RETICLE
COLOR_OFFSET    = C_AMBER
COLOR_OK        = C_OK
COLOR_WARN      = C_AMBER
COLOR_ERR       = C_ERR
COLOR_BG        = C_BG
COLOR_FG        = C_TEXT

_FONT = cv2.FONT_HERSHEY_DUPLEX
_FS   = 0.40   # base font scale
_FT   = 1      # font thickness


# ── Supervision annotator suite (lazy-init) ───────────────────────────────── #

_SV: dict | None = None


def _get_sv() -> dict:
    """Lazy-initialize the rich supervision annotator suite."""
    global _SV
    if _SV is None:
        import supervision as sv
        _SV = {
            # Thin square class-colored outline — cleaner than rounded on a tactical HUD
            'box': sv.BoxAnnotator(
                thickness=1,
                color_lookup=sv.ColorLookup.CLASS),
            # Thick white corner brackets — strong focal markers without crowding the edges
            'corners': sv.BoxCornerAnnotator(
                thickness=4, corner_length=18,
                color=sv.Color.WHITE),
            # Triangle indicator above each box — "locked on" visual cue
            'triangle': sv.TriangleAnnotator(
                base=10, height=8,
                color_lookup=sv.ColorLookup.CLASS),
            # Confidence bar (small strip above box)
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
    """Convert Detection list to sv.Detections, optionally with tracker_id."""
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
    """Boxes + confidence bars + labels for every detection (no primary highlight)."""
    if frame_bgr is None or not detections:
        return frame_bgr
    sv = _get_sv()
    sv_det = _to_sv(detections)
    out = frame_bgr.copy()
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
    """Top-left PERCEPTION panel (standalone; render_all uses inline version)."""
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
    """Bottom-left ALIGNMENT panel (standalone; render_all uses inline version)."""
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
    """Top-right STATE panel (standalone). state: DuburiState or compatible."""
    if frame_bgr is None:
        return frame_bgr
    out = frame_bgr.copy()
    h, w = out.shape[:2]
    if state is None:
        _mc_panel(out, w - 145, 8, 'STATE', [('no telemetry', '', C_DIM)])
        return out
    armed  = bool(state.armed)
    bv     = float(state.battery_voltage)
    rows = [
        ('DEPTH', f'{state.depth_m:+.2f}m', C_TEXT),
        ('YAW',   f'{state.yaw_deg:.1f}',    C_TEXT),
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
    """Vertical depth slider drawn in-place. 0m at top, max_depth at bottom.

    depth_m is negative (below surface) as per DuburiState convention.
    Guard: skips silently when depth_m is NaN.
    """
    if np.isnan(depth_m):
        return
    depth_abs = float(min(abs(depth_m), max_depth))

    # Semi-transparent background panel
    overlay = img.copy()
    cv2.rectangle(overlay, (x, y), (x + gauge_w, y + gauge_h), C_BG, -1)
    cv2.addWeighted(overlay, 0.75, img, 0.25, 0, dst=img)
    cv2.rectangle(img, (x, y), (x + gauge_w, y + gauge_h), C_BORDER, 1, cv2.LINE_AA)

    # Title
    cv2.putText(img, 'D', (x + 5, y - 4), _FONT, 0.30, C_DIM, 1, cv2.LINE_AA)

    # Tick marks: every 1m, label every 2m
    for m in range(0, int(max_depth) + 1):
        yt = y + int(m / max_depth * gauge_h)
        tick_len = 6 if m % 2 == 0 else 3
        cv2.line(img, (x, yt), (x + tick_len, yt), C_DIM, 1)
        if m % 2 == 0 and m > 0:
            cv2.putText(img, str(m), (x - 12, yt + 4),
                        _FONT, 0.28, C_DIM, 1, cv2.LINE_AA)

    # Moving indicator line + depth label
    ind_y = y + int(depth_abs / max_depth * gauge_h)
    ind_y = max(y + 2, min(y + gauge_h - 2, ind_y))
    color = C_OK if depth_abs < 3.0 else (C_AMBER if depth_abs < 7.0 else C_ERR)
    cv2.line(img, (x, ind_y), (x + gauge_w, ind_y), color, 2, cv2.LINE_AA)
    # Depth value to the left of gauge
    label = f'{depth_abs:.1f}m'
    (tw, _), _ = cv2.getTextSize(label, _FONT, 0.30, 1)
    cv2.putText(img, label, (x - tw - 3, ind_y + 4),
                _FONT, 0.30, color, 1, cv2.LINE_AA)


def draw_heading_tape(img: np.ndarray, yaw_deg: float,
                      x: int, y: int,
                      tape_w: int = 260, tape_h: int = 30) -> None:
    """Horizontal compass tape drawn in-place. Shows ±60° around current heading.

    Guard: skips silently when yaw_deg is NaN.
    """
    if np.isnan(yaw_deg):
        return
    yaw = float(yaw_deg) % 360.0
    cx  = x + tape_w // 2
    pixels_per_deg = tape_w / 120.0   # 120° total visible range

    # Semi-transparent background
    overlay = img.copy()
    cv2.rectangle(overlay, (x, y), (x + tape_w, y + tape_h), C_BG, -1)
    cv2.addWeighted(overlay, 0.75, img, 0.25, 0, dst=img)
    cv2.rectangle(img, (x, y), (x + tape_w, y + tape_h), C_BORDER, 1, cv2.LINE_AA)

    # Ticks and labels
    for delta in range(-65, 66, 10):
        deg_at = int(yaw + delta) % 360
        px = cx + int(delta * pixels_per_deg)
        if px < x + 2 or px > x + tape_w - 2:
            continue
        is_label = delta % 30 == 0
        tick_h = 8 if is_label else 4
        cv2.line(img, (px, y + tape_h - tick_h), (px, y + tape_h - 1),
                 C_DIM, 1)
        if is_label:
            label = _cardinal(deg_at)
            (tw, _), _ = cv2.getTextSize(label, _FONT, 0.30, 1)
            cv2.putText(img, label, (px - tw // 2, y + tape_h - tick_h - 2),
                        _FONT, 0.30, C_TEXT, 1, cv2.LINE_AA)

    # Center marker (current heading)
    cv2.line(img, (cx, y + 2), (cx, y + tape_h - 2), C_ACCENT, 2, cv2.LINE_AA)

    # Heading readout above tape center — number + degree mark circle
    hdg_label = f'{int(yaw) % 360:03d}'
    (tw, th), _ = cv2.getTextSize(hdg_label, _FONT, _FS, _FT)
    lx = cx - tw // 2
    ly = y - 3
    cv2.putText(img, hdg_label, (lx, ly), _FONT, _FS, C_ACCENT, _FT, cv2.LINE_AA)
    # small circle to the right of the number as degree glyph
    cv2.circle(img, (lx + tw + 4, ly - th + 3), 2, C_ACCENT, 1, cv2.LINE_AA)


def draw_classes_panel(img: np.ndarray,
                       configured_classes: List[str],
                       active_class_names,
                       x: int, y: int) -> None:
    """CLASSES panel drawn in-place.

    Configured classes are shown dimmed; those currently detected light up teal.
    Guard: skips silently when configured_classes is empty.
    """
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
                        cx: int, cy: int, radius: int = 22) -> None:
    """Compass rose with directional needle drawn in-place.

    North is fixed at the top of the circle; the needle points in the
    direction of yaw_deg from North. Shows a '?' ring when yaw_deg is NaN.
    Always rendered (even without telemetry) so the operator can see at
    a glance whether heading data is present.
    """
    overlay = img.copy()
    cv2.circle(overlay, (cx, cy), radius, C_BG, -1)
    cv2.addWeighted(overlay, 0.75, img, 0.25, 0, dst=img)
    cv2.circle(img, (cx, cy), radius, C_BORDER, 1, cv2.LINE_AA)

    # Fixed N/E/S/W dots
    for ang in (0, 90, 180, 270):
        rad = np.radians(ang)
        tx = cx + int((radius - 5) * np.sin(rad))
        ty = cy - int((radius - 5) * np.cos(rad))
        cv2.circle(img, (tx, ty), 1, C_DIM, -1)
    cv2.putText(img, 'N', (cx - 4, cy - radius + 9),
                _FONT, 0.25, C_DIM, 1, cv2.LINE_AA)

    if np.isnan(yaw_deg):
        cv2.putText(img, '?', (cx - 4, cy + 5), _FONT, 0.35, C_DIM, 1, cv2.LINE_AA)
        return

    needle = radius - 5
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
    """Heading-source status panel drawn in-place.

    yaw_source: 'mavlink_ahrs' | 'bno085' | 'dvl' | 'bno085_dvl' | ''
    Status is inferred: ACTIVE when yaw_deg is a valid number, NO DATA when NaN.
    """
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
        ('SRC',  src,    C_TEXT),
        ('HDG',  status, st_col),
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
    """Full mission-control overlay — single frame copy, all panels in one pass.

    Layer order (later layers paint on top):
      1. dashed reticle
      2. detection boxes + confidence bars + labels  (supervision rich annotators)
      3. primary halo (glow) + crosshair + offset arrow
      4. PERCEPTION panel    (top-left)
      5. CLASSES panel       (below PERCEPTION, only if configured_classes set)
      6. ALIGNMENT panel     (bottom-left)
      7. STATE panel         (top-right, only if state provided)
      8. DEPTH GAUGE         (right edge, only if state has valid depth_m)
      9. HEADING TAPE        (bottom-center, only if state has valid yaw_deg)
     10. STALE banner        (only if not healthy)

    Parameters
    ----------
    configured_classes : list[str] | None
        Full list of classes the detector is configured for (from classes_filter
        topic). If provided, a CLASSES panel lights up detected members.
    track_ids : list[int] | None
        Parallel tracker IDs for each detection in `detections`. Used by the
        supervision annotators for stable per-track coloring.
    """
    if frame_bgr is None:
        return None

    out = frame_bgr.copy()
    h, w = out.shape[:2]

    # ── 1. Reticle + deadband box ─────────────────────────────────────────── #
    if show_reticle:
        cx, cy = w // 2, h // 2
        _dashed_line(out, (cx, 0), (cx, h), C_RETICLE, dash=8, gap=6, thickness=1)
        _dashed_line(out, (0, cy), (w, cy), C_RETICLE, dash=8, gap=6, thickness=1)
        cv2.circle(out, (cx, cy), 4, C_RETICLE, 1, cv2.LINE_AA)
        cv2.circle(out, (cx, cy), 1, C_RETICLE, -1, cv2.LINE_AA)
        # Deadband zone rectangle — alignment is "achieved" when target enters this box
        db_px, db_py = int(deadband * w / 2), int(deadband * h / 2)
        cv2.rectangle(out, (cx - db_px, cy - db_py), (cx + db_px, cy + db_py),
                      C_RETICLE, 1, cv2.LINE_AA)

    # ── 1b. "BRACU DUBURI" watermark (top-center) ─────────────────────────── #
    brand = 'BRACU  DUBURI'
    (bw, bh), _ = cv2.getTextSize(brand, _FONT, 0.38, 1)
    bx = (w - bw) // 2
    by = 18
    cv2.putText(out, brand, (bx, by), _FONT, 0.38, C_DIM, 1, cv2.LINE_AA)
    cv2.circle(out, (bx - 6, by - bh // 2), 2, C_ACCENT, -1, cv2.LINE_AA)
    cv2.circle(out, (bx + bw + 6, by - bh // 2), 2, C_ACCENT, -1, cv2.LINE_AA)

    # ── 2. Detection boxes (rich supervision annotators) ─────────────────── #
    if detections:
        sv = _get_sv()
        sv_all = _to_sv(detections, track_ids)
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

    # ── 3. Primary: ACCENT border + corner brackets + crosshair + arrow ─────── #
    primary = primary or largest(detections)
    if primary is not None:
        x1p, y1p, x2p, y2p = (int(v) for v in primary.xyxy)
        cv2.rectangle(out, (x1p, y1p), (x2p, y2p), C_ACCENT, 2, cv2.LINE_AA)
        _draw_corners(out, x1p, y1p, x2p, y2p, C_ACCENT, length=15, thickness=3)
        cx_t, cy_t = int(primary.cx), int(primary.cy)
        cv2.line(out, (cx_t - 14, cy_t), (cx_t + 14, cy_t), C_ACCENT, 2, cv2.LINE_AA)
        cv2.line(out, (cx_t, cy_t - 14), (cx_t, cy_t + 14), C_ACCENT, 2, cv2.LINE_AA)
        cv2.circle(out, (cx_t, cy_t), 3, C_ACCENT, -1, cv2.LINE_AA)
        if abs(cx_t - w // 2) >= 2 or abs(cy_t - h // 2) >= 2:
            cv2.arrowedLine(out, (w // 2, h // 2), (cx_t, cy_t),
                            C_AMBER, 2, cv2.LINE_AA, tipLength=0.18)

    # ── 4. PERCEPTION panel (top-left) ────────────────────────────────────── #
    perc_rows = [
        ('SRC', str(source),       C_TEXT),
        ('FPS', f'{fps:.1f}',      C_TEXT),
        ('DET', str(len(detections)), C_ACCENT if detections else C_DIM),
    ]
    if primary is not None:
        perc_rows.append(('TGT', primary.class_name, C_ACCENT))
    if tracking_on:
        perc_rows.append(('TRK', f'ON  n={n_tracks}', C_ACCENT))
    badge_border = (C_OK if (healthy and detections)
                    else (C_ERR if not healthy else C_BORDER))
    _mc_panel(out, 8, 8, 'PERCEPTION', perc_rows, border=badge_border)

    # ── 5. CLASSES panel (below PERCEPTION) ───────────────────────────────── #
    if configured_classes:
        # Compute where PERCEPTION panel ends to anchor CLASSES below it
        perc_panel_h = 17 * (len(perc_rows) + 1) + 2 * 5   # line_h=17, pad=5, +1 title
        classes_y = 8 + perc_panel_h + 4
        active_names = {d.class_name for d in detections}
        draw_classes_panel(out, configured_classes, active_names, x=8, y=classes_y)

    # ── 6. ALIGNMENT panel (bottom-left) ──────────────────────────────────── #
    if show_alignment:
        if primary is None:
            al_rows   = [('STATUS', 'NO TARGET', C_ERR)]
            al_border = C_ERR
        else:
            ex = (primary.cx - w / 2.0) / max(w / 2.0, 1.0)
            ey = (primary.cy - h / 2.0) / max(h / 2.0, 1.0)
            aligned = abs(ex) < deadband and abs(ey) < deadband

            if tracking_on and n_tracks > 0:
                tid = f'#{primary_track_id}' if primary_track_id is not None else f'n={n_tracks}'
                status_val = f'TRACKED  {tid}'
                status_col = C_ACCENT
            else:
                status_val = 'DETECTED'
                status_col = C_TEXT

            al_rows = [
                ('STATUS', status_val,                                          status_col),
                ('ERR_X',  f'{ex:+.3f}', C_OK if abs(ex) < deadband else C_AMBER),
                ('ERR_Y',  f'{ey:+.3f}', C_OK if abs(ey) < deadband else C_AMBER),
                ('AREA',   f'{(primary.area / (w * h) * 100):.2f}%',           C_TEXT),
                ('CONF',   f'{primary.score:.2f}',                              C_TEXT),
            ]
            al_border = C_OK if aligned else C_AMBER

        _mc_panel(out, 8, _panel_bottom_y(h, len(al_rows)), 'ALIGNMENT',
                  al_rows, border=al_border)

    # ── 7. STATE panel (top-right) ────────────────────────────────────────── #
    if state is not None:
        armed = bool(state.armed)
        bv    = float(state.battery_voltage)
        st_rows = [
            ('DEPTH', f'{state.depth_m:+.2f}m',  C_TEXT),
            ('YAW',   f'{state.yaw_deg:.1f}',     C_TEXT),
            ('MODE',  state.mode or '?',           C_ACCENT if armed else C_DIM),
            ('BATT',  f'{bv:.1f}V',                _batt_color(bv)),
            ('ARMED', 'YES' if armed else 'no',    C_OK if armed else C_DIM),
        ]
        x_st = _panel_right_x(w, 'STATE', st_rows)
        _mc_panel(out, x_st, 8, 'STATE', st_rows,
                  border=C_OK if armed else C_BORDER)

        # ── 8. DEPTH GAUGE (right edge, below STATE panel) ────────────────── #
        draw_depth_gauge(out, state.depth_m, x=w - 34, y=130)

        # ── 9. HEADING SOURCE panel (bottom-center-right) ─────────────────── #
        draw_sensors_panel(out, yaw_source or '', state.yaw_deg, x=(w + 60) // 2, y=h - 68)

    # ── 10. COMPASS NEEDLE (always shown — '?' when no state/heading) ─────── #
    _yaw = state.yaw_deg if state is not None else float('nan')
    draw_compass_needle(out, _yaw, cx=195, cy=h - 30)

    # ── 11. Stale banner ──────────────────────────────────────────────────── #
    if not healthy:
        cv2.rectangle(out, (0, 0), (w, 28), C_ERR, -1)
        cv2.putText(out, 'STALE FRAME', (8, 20),
                    _FONT, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    return out


def draw_track_ids(frame_bgr: np.ndarray, tracks) -> np.ndarray:
    """Overlay stable track IDs on tracked detections (legacy helper).

    Each track_id gets a consistent color; predicted=True tracks are
    drawn at half-alpha. Accepts any iterable with .xyxy, .track_id,
    .class_name, .score, .predicted attributes.
    """
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
        color     = _PALETTE[abs(int(td.track_id)) % len(_PALETTE)]
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

def _mc_panel(img, x, y, title, rows, *, border=None, pad=5, line_h=17):
    """Mission-control panel: optional title bar + aligned key-value rows.

    rows: list of
        str                  — full-width text in C_TEXT
        (label, value)       — label in C_DIM, value in C_TEXT
        (label, value, color)— label in C_DIM, value in given color
    """
    border = border or C_BORDER

    # Key column width (fixed for all rows so values align)
    key_ws = [cv2.getTextSize(r[0], _FONT, _FS, _FT)[0][0]
              for r in rows if not isinstance(r, str) and len(r) >= 2]
    col_w = (max(key_ws) + 8) if key_ws else 0

    # Panel width from all content
    candidates = []
    if title:
        candidates.append(cv2.getTextSize(title, _FONT, _FS, _FT)[0][0])
    for row in rows:
        if isinstance(row, str):
            candidates.append(cv2.getTextSize(row, _FONT, _FS, _FT)[0][0])
        elif len(row) >= 2:
            val_w = cv2.getTextSize(row[1], _FONT, _FS, _FT)[0][0]
            candidates.append(col_w + val_w)
    panel_w = (max(candidates) if candidates else 60) + 2 * pad
    n_data_rows = len(rows)
    title_rows  = 1 if title else 0
    panel_h = line_h * (n_data_rows + title_rows) + 2 * pad

    # Semi-transparent fill
    overlay = img.copy()
    cv2.rectangle(overlay, (x, y), (x + panel_w, y + panel_h), C_BG, -1)
    cv2.addWeighted(overlay, 0.75, img, 0.25, 0, dst=img)

    # Border
    cv2.rectangle(img, (x, y), (x + panel_w, y + panel_h), border, 1, cv2.LINE_AA)

    row_offset = 0
    if title:
        ty = y + pad + line_h - 3
        cv2.putText(img, title, (x + pad, ty), _FONT, _FS, C_ACCENT, _FT, cv2.LINE_AA)
        sep_y = y + pad + line_h + 1
        cv2.line(img, (x + 1, sep_y), (x + panel_w - 1, sep_y), C_BORDER, 1)
        row_offset = 1

    for i, row in enumerate(rows):
        ry = y + pad + line_h * (i + row_offset + 1) - 3
        if isinstance(row, str):
            cv2.putText(img, row, (x + pad, ry), _FONT, _FS, C_TEXT, _FT, cv2.LINE_AA)
        else:
            label     = row[0]
            val       = row[1] if len(row) > 1 else ''
            val_color = row[2] if len(row) > 2 else C_TEXT
            cv2.putText(img, label, (x + pad, ry),
                        _FONT, _FS, C_DIM, _FT, cv2.LINE_AA)
            cv2.putText(img, val, (x + pad + col_w, ry),
                        _FONT, _FS, val_color, _FT, cv2.LINE_AA)


def _panel(img, top_left, lines, *, fg, bg, border, pad=6, line_h=16):
    """Legacy panel shim — updated to use mission-control style."""
    x, y = top_left
    rows = [str(l) for l in lines]
    _mc_panel(img, x, y, '', rows, border=border, pad=pad, line_h=line_h)


def _panel_bottom_y(frame_h: int, n_data_rows: int,
                    pad=5, line_h=17, margin=8) -> int:
    """Y coordinate to bottom-anchor a panel with given data row count."""
    panel_h = line_h * (n_data_rows + 1) + 2 * pad   # +1 for title
    return max(frame_h - panel_h - margin, margin)


def _panel_right_x(frame_w: int, title: str, rows,
                   pad=5, margin=8) -> int:
    """X coordinate to right-anchor a panel (flush with frame right edge)."""
    key_ws = [cv2.getTextSize(r[0], _FONT, _FS, _FT)[0][0]
              for r in rows if not isinstance(r, str) and len(r) >= 2]
    col_w = (max(key_ws) + 8) if key_ws else 0
    candidates = [cv2.getTextSize(title, _FONT, _FS, _FT)[0][0]]
    for row in rows:
        if not isinstance(row, str) and len(row) >= 2:
            vw = cv2.getTextSize(row[1], _FONT, _FS, _FT)[0][0]
            candidates.append(col_w + vw)
    panel_w = max(candidates) + 2 * pad
    return max(frame_w // 2, frame_w - panel_w - margin)


def _batt_color(voltage: float):
    if voltage > 14.0:
        return C_OK
    if voltage > 12.5:
        return C_AMBER
    return C_ERR


def _cardinal(deg: int) -> str:
    """Degree → cardinal label (N/E/S/W) or 3-digit numeric string."""
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

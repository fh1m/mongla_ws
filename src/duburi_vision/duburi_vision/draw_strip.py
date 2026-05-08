"""draw_strip -- mission-control UI strip (260px below the video frame).

render_ui_strip() assembles the complete strip as a new numpy array.
Layout (top → bottom):

  Row 1  h=28   Header bar: "● BRACU DUBURI ●"  FPS  LIVE/VIDEO badge
  Row 2  h=36   ERR_X and ERR_Y needle gauges (each half-width)
  Row 3  h=82   Left: PERCEPTION/CLASSES/ALIGNMENT/TRACKS panels
                Right: ERR_X sparkline + ERR_Y sparkline + confidence bar
  Row 4  h=66   Left: compass + heading-source label
                Centre: STATE panel + ALIGNMENT summary
                Right: altimeter depth + battery bar
  Row 5  h=22   Full-width heading tape
  Row 6  h=26   Live mode: pipeline health row
                Video mode: video progress bar
  ──────────────────────────────────────────
  Total  260px  → _STRIP_H = 260

All panel drawing reuses _mc_panel / _panel_width helpers kept here.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence

import cv2
import numpy as np

from .draw_widgets import (
    C_BG, C_ACCENT, C_AMBER, C_OK, C_ERR, C_TEXT, C_DIM, C_BORDER,
    _FONT, _FT,
    needle_gauge, sparkline, confidence_bar,
    altimeter_depth, battery_bar, mini_compass, heading_tape,
    video_progress, health_row,
)
from .detection.detector import Detection

_STRIP_H  = 260   # total strip height in pixels

# Row y-positions and heights
_R1_Y, _R1_H = 0,   28   # header
_R2_Y, _R2_H = 28,  36   # ERR gauges
_R3_Y, _R3_H = 64,  82   # panels + sparklines
_R4_Y, _R4_H = 146, 66   # instruments
_R5_Y, _R5_H = 212, 22   # heading tape
_R6_Y, _R6_H = 234, 26   # health / video progress

# Strip panel typography
_SFS = 0.45   # font scale for strip panels
_SLH = 17     # line height in strip panels
_SPAD = 5     # inner padding


def render_ui_strip(w: int, frame_h: int, *,
                    detections: List[Detection],
                    primary: Optional[Detection],
                    source: str,
                    fps: float,
                    healthy: bool,
                    tracking_on: bool,
                    n_tracks: int,
                    primary_track_id,
                    configured_classes: List[str],
                    state,
                    yaw_source: str,
                    deadband: float,
                    show_alignment: bool = True,
                    err_x_history: Optional[Sequence[float]] = None,
                    err_y_history: Optional[Sequence[float]] = None,
                    conf_history:  Optional[Sequence[float]] = None,
                    video_mode: bool = False,
                    is_paused:  bool = False,
                    video_position: Optional[tuple] = None,
                    pipeline_health: Optional[Dict[str, bool]] = None) -> np.ndarray:
    """Build and return the _STRIP_H×w UI strip drawn directly at native width."""
    strip = np.full((_STRIP_H, w, 3), C_BG, dtype=np.uint8)
    cv2.line(strip, (0, 0), (w - 1, 0), C_ACCENT, 2)   # top accent bar

    yaw = state.yaw_deg if state is not None else float('nan')

    # ── Row 1: Header ──────────────────────────────────────────────────────── #
    _draw_header_row(strip, w, fps, video_mode, is_paused)

    # ── Row 2: ERR gauges ──────────────────────────────────────────────────── #
    _draw_err_gauges(strip, w, frame_h, primary, deadband)

    # ── Row 3: Panels + sparklines ─────────────────────────────────────────── #
    _draw_panels_row(strip, w, frame_h, detections, primary, source,
                     healthy, tracking_on, n_tracks, primary_track_id,
                     configured_classes, show_alignment, deadband,
                     err_x_history or [], err_y_history or [], conf_history or [])

    # ── Row 4: Instruments ─────────────────────────────────────────────────── #
    _draw_instruments_row(strip, w, yaw, yaw_source, state)

    # ── Row 5: Heading tape ────────────────────────────────────────────────── #
    cv2.line(strip, (4, _R5_Y - 1), (w - 4, _R5_Y - 1), C_BORDER, 1)
    heading_tape(strip, 4, _R5_Y, w - 8, _R5_H, yaw, show_readout=False)

    # ── Row 6: Health / video progress ─────────────────────────────────────── #
    if video_mode and video_position is not None:
        cur_f, tot_f = video_position
        img_fps = fps or 30.0
        video_progress(strip, 0, _R6_Y, w, _R6_H,
                       cur_f, tot_f, img_fps, is_paused)
    else:
        health = pipeline_health or {}
        health_row(strip, 0, _R6_Y, w, _R6_H, health)

    # Bottom accent bar
    cv2.line(strip, (0, _STRIP_H - 1), (w - 1, _STRIP_H - 1), C_ACCENT, 1)

    return strip


# ── Row renderers ─────────────────────────────────────────────────────────── #

def _draw_header_row(strip: np.ndarray, w: int, fps: float,
                     video_mode: bool, is_paused: bool) -> None:
    # Brand label centred
    brand = 'BRACU  DUBURI'
    (bw, bh), _ = cv2.getTextSize(brand, _FONT, 0.44, _FT)
    bx = (w - bw) // 2
    by = _R1_Y + 16
    cv2.putText(strip, brand, (bx, by), _FONT, 0.44, C_TEXT, _FT, cv2.LINE_AA)
    dot_y = by - bh // 2
    cv2.circle(strip, (bx - 10, dot_y), 3, C_ACCENT, -1, cv2.LINE_AA)
    cv2.circle(strip, (bx + bw + 10, dot_y), 3, C_ACCENT, -1, cv2.LINE_AA)

    # FPS (left side)
    fps_lbl = f'{fps:.0f} Hz'
    cv2.putText(strip, fps_lbl, (6, by), _FONT, 0.30, C_DIM, _FT, cv2.LINE_AA)

    # Mode badge (right side) — ASCII only; cv2 fonts lack Unicode glyphs
    if video_mode:
        badge     = '|| VIDEO SIM' if is_paused else '>  VIDEO SIM'
        badge_col = C_AMBER if is_paused else C_ACCENT
    else:
        badge     = 'LIVE'
        badge_col = C_OK
    (mw, mh), _ = cv2.getTextSize(badge, _FONT, 0.30, _FT)
    badge_x = w - mw - 6
    cv2.putText(strip, badge, (badge_x, by), _FONT, 0.30, badge_col, _FT, cv2.LINE_AA)
    # Draw a filled circle as the LIVE dot (replaces '●' glyph)
    if not video_mode:
        dot_cx = badge_x - 8
        dot_cy = by - mh // 2 - 1
        cv2.circle(strip, (dot_cx, dot_cy), 3, C_OK, -1, cv2.LINE_AA)

    cv2.line(strip, (4, _R2_Y - 1), (w - 4, _R2_Y - 1), C_BORDER, 1)


def _draw_err_gauges(strip: np.ndarray, w: int, frame_h: int,
                     primary: Optional[Detection], deadband: float) -> None:
    gpad = 6
    gh   = _R2_H - 10   # gauge height
    gw   = (w - 3 * gpad) // 2

    # ERR_X label
    cv2.putText(strip, 'ERR_X', (gpad, _R2_Y + 10), _FONT, 0.32, C_DIM, _FT, cv2.LINE_AA)
    cv2.putText(strip, 'ERR_Y', (gpad + gw + gpad, _R2_Y + 10), _FONT, 0.32, C_DIM, _FT, cv2.LINE_AA)

    fw = max(w, 1)
    ex = ey = 0.0
    if primary is not None:
        ex = (primary.cx - fw / 2.0) / max(fw / 2.0, 1.0)
        ey = (primary.cy - max(frame_h, 1) / 2.0) / max(frame_h / 2.0, 1.0)

    gauge_y = _R2_Y + 14
    needle_gauge(strip, gpad, gauge_y, gw, gh, ex,
                 vmin=-1.0, vmax=1.0, deadband=deadband)
    needle_gauge(strip, gpad * 2 + gw, gauge_y, gw, gh, ey,
                 vmin=-1.0, vmax=1.0, deadband=deadband)


def _draw_panels_row(strip: np.ndarray, w: int, frame_h: int,
                     detections: List[Detection],
                     primary: Optional[Detection],
                     source: str, healthy: bool,
                     tracking_on: bool, n_tracks: int, primary_track_id,
                     configured_classes: List[str],
                     show_alignment: bool, deadband: float,
                     err_x_hist: Sequence[float],
                     err_y_hist: Sequence[float],
                     conf_hist:  Sequence[float]) -> None:
    py = _R3_Y + 2
    px = 6

    # PERCEPTION
    perc_rows = [
        ('SRC', str(source),          C_TEXT),
        ('FPS', '',                   C_DIM),     # FPS shown in header
        ('DET', str(len(detections)), C_ACCENT if detections else C_DIM),
    ]
    if primary is not None:
        perc_rows[1] = ('TGT', primary.class_name, C_ACCENT)
    elif tracking_on:
        perc_rows[1] = ('TRK', f'n={n_tracks}', C_ACCENT)
    badge_border = (C_OK if (healthy and detections)
                    else (C_ERR if not healthy else C_BORDER))
    _mc_panel(strip, px, py, 'PERCEPTION', perc_rows,
              border=badge_border, pad=_SPAD, line_h=_SLH, fs=_SFS)
    px += _panel_width('PERCEPTION', perc_rows, pad=_SPAD, fs=_SFS) + 4

    # CLASSES
    if configured_classes:
        active_lower = {d.class_name.lower() for d in detections}
        any_active   = any(c.lower() in active_lower for c in configured_classes)
        cls_rows     = [('', c.upper(),
                         C_ACCENT if c.lower() in active_lower else C_DIM)
                        for c in configured_classes]
        _mc_panel(strip, px, py, 'CLASSES', cls_rows,
                  border=C_ACCENT if any_active else C_BORDER,
                  pad=_SPAD, line_h=_SLH, fs=_SFS)
        px += _panel_width('CLASSES', cls_rows, pad=_SPAD, fs=_SFS) + 4

    # ALIGNMENT
    if show_alignment:
        if primary is None:
            al_rows   = [('STATUS', 'NO TARGET', C_ERR)]
            al_border = C_ERR
        else:
            fw = strip.shape[1]
            ex = (primary.cx - fw / 2.0) / max(fw / 2.0, 1.0)
            ey = (primary.cy - frame_h  / 2.0) / max(frame_h  / 2.0, 1.0)
            aligned = abs(ex) < deadband and abs(ey) < deadband
            if tracking_on and n_tracks > 0:
                tid_str   = f'#{primary_track_id}' if primary_track_id is not None else f'n={n_tracks}'
                st_val, st_col = f'TRK {tid_str}', C_ACCENT
            else:
                st_val, st_col = 'DETECTED', C_TEXT
            al_rows = [
                ('STATUS', st_val,             st_col),
                ('ERR_X',  f'{ex:+.3f}',       C_OK if abs(ex) < deadband else C_AMBER),
                ('ERR_Y',  f'{ey:+.3f}',       C_OK if abs(ey) < deadband else C_AMBER),
                ('AREA',   f'{(primary.area / max(fw * frame_h, 1) * 100):.1f}%', C_TEXT),
                ('CONF',   f'{primary.score:.2f}', C_TEXT),
            ]
            al_border = C_OK if aligned else C_AMBER
        _mc_panel(strip, px, py, 'ALIGNMENT', al_rows,
                  border=al_border, pad=_SPAD, line_h=_SLH, fs=_SFS)
        px += _panel_width('ALIGNMENT', al_rows, pad=_SPAD, fs=_SFS) + 4

    # TRACKS
    if tracking_on or n_tracks > 0:
        trk_rows = [
            ('N',   str(n_tracks),
             C_ACCENT if n_tracks else C_DIM),
            ('PRI', f'#{primary_track_id}' if primary_track_id is not None else 'none',
             C_ACCENT if primary_track_id is not None else C_DIM),
        ]
        _mc_panel(strip, px, py, 'TRACKS', trk_rows,
                  border=C_ACCENT if n_tracks else C_BORDER,
                  pad=_SPAD, line_h=_SLH, fs=_SFS)

    # Sparklines (right-anchored)
    sp_w   = 72
    sp_h   = 18
    sp_gap = 4
    sp_x   = strip.shape[1] - sp_w - 6
    sp_y   = _R3_Y + 4

    cv2.putText(strip, 'ERR_X', (sp_x, sp_y + 12),
                _FONT, 0.30, C_DIM, _FT, cv2.LINE_AA)
    sparkline(strip, sp_x, sp_y + 14, sp_w, sp_h,
              err_x_hist, color=C_AMBER)

    sp_y2  = sp_y + sp_h + sp_gap + 16
    cv2.putText(strip, 'ERR_Y', (sp_x, sp_y2 + 12),
                _FONT, 0.30, C_DIM, _FT, cv2.LINE_AA)
    sparkline(strip, sp_x, sp_y2 + 14, sp_w, sp_h,
              err_y_hist, color=(80, 180, 240))

    # Confidence bar
    sp_y3  = sp_y2 + sp_h + sp_gap + 16
    cv2.putText(strip, 'CONF', (sp_x, sp_y3 + 12),
                _FONT, 0.30, C_DIM, _FT, cv2.LINE_AA)
    last_conf = conf_hist[-1] if conf_hist else 0.0
    confidence_bar(strip, sp_x, sp_y3 + 14, sp_w, sp_h, last_conf)


def _draw_instruments_row(strip: np.ndarray, w: int,
                          yaw: float, yaw_source: str, state) -> None:
    cv2.line(strip, (4, _R4_Y), (w - 4, _R4_Y), C_BORDER, 1)

    # Compass + heading source label (left block)
    cmp_r  = 24
    cmp_cx = 30
    cmp_cy = _R4_Y + cmp_r + 6
    mini_compass(strip, cmp_cx, cmp_cy, cmp_r, yaw)

    # Heading source label — clamped to stay within Row 4
    _SOURCE = {'mavlink_ahrs': 'MAVLINK', 'bno085': 'BNO085',
               'dvl': 'DVL', 'bno085_dvl': 'BNO+DVL'}
    src_lbl = _SOURCE.get(yaw_source, (yaw_source or '?').upper())
    src_label_y = min(cmp_cy + cmp_r + 14, _R4_Y + _R4_H - 4)
    cv2.putText(strip, src_lbl,
                (cmp_cx - cmp_r, src_label_y),
                _FONT, 0.30, C_DIM, _FT, cv2.LINE_AA)

    # Depth altimeter (right side)
    alt_w  = 24
    alt_h  = _R4_H - 10
    alt_x  = w - alt_w - 4
    alt_y  = _R4_Y + 5
    depth  = state.depth_m if state is not None and not np.isnan(state.depth_m) else float('nan')
    altimeter_depth(strip, alt_x, alt_y, alt_w, alt_h, depth, max_depth=5.0)

    # Battery bar (to the left of depth)
    bat_w  = 70
    bat_h  = 12
    bat_x  = alt_x - bat_w - 12
    bat_y  = _R4_Y + 10
    voltage = state.battery_voltage if state is not None else float('nan')
    if not np.isnan(voltage) and voltage > 0:
        battery_bar(strip, bat_x, bat_y, bat_w, bat_h, voltage)

    # STATE panel (centre)
    if state is not None:
        armed   = bool(state.armed)
        st_rows = [
            ('DEPTH', f'{state.depth_m:+.2f}m', C_TEXT),
            ('YAW',   f'{state.yaw_deg:.1f}°',  C_TEXT),
            ('MODE',  state.mode or '?',          C_ACCENT if armed else C_DIM),
            ('ARMED', 'YES' if armed else 'no',   C_OK if armed else C_DIM),
        ]
        st_x = cmp_cx + cmp_r + 12
        st_y = _R4_Y + 4
        _mc_panel(strip, st_x, st_y, 'STATE', st_rows,
                  border=C_OK if armed else C_BORDER, pad=_SPAD, line_h=_SLH, fs=_SFS)


# ── Panel helpers ─────────────────────────────────────────────────────────── #

def _mc_panel(img, x, y, title, rows, *,
              border=None, pad=5, line_h=17, fs=None):
    fs     = fs if fs is not None else 0.40
    border = border or C_BORDER

    key_ws  = [cv2.getTextSize(r[0], _FONT, fs, _FT)[0][0]
               for r in rows if not isinstance(r, str) and len(r) >= 2]
    col_w   = (max(key_ws) + 8) if key_ws else 0

    candidates = []
    if title:
        candidates.append(cv2.getTextSize(title, _FONT, fs, _FT)[0][0])
    for row in rows:
        if isinstance(row, str):
            candidates.append(cv2.getTextSize(row, _FONT, fs, _FT)[0][0])
        elif len(row) >= 2:
            candidates.append(col_w + cv2.getTextSize(row[1], _FONT, fs, _FT)[0][0])
    panel_w = (max(candidates) if candidates else 60) + 2 * pad
    panel_h = line_h * (len(rows) + (1 if title else 0)) + 2 * pad

    ov = img.copy()
    cv2.rectangle(ov, (x, y), (x + panel_w, y + panel_h), C_BG, -1)
    cv2.addWeighted(ov, 0.82, img, 0.18, 0, dst=img)
    cv2.rectangle(img, (x, y), (x + panel_w, y + panel_h), border, 1, cv2.LINE_AA)

    row_off = 0
    if title:
        ty = y + pad + line_h - 3
        cv2.putText(img, title, (x + pad, ty), _FONT, fs, C_ACCENT, _FT, cv2.LINE_AA)
        cv2.line(img, (x + 1, y + pad + line_h + 1),
                 (x + panel_w - 1, y + pad + line_h + 1), C_BORDER, 1)
        row_off = 1

    for i, row in enumerate(rows):
        ry = y + pad + line_h * (i + row_off + 1) - 3
        if isinstance(row, str):
            cv2.putText(img, row, (x + pad, ry), _FONT, fs, C_TEXT, _FT, cv2.LINE_AA)
        else:
            label     = row[0]
            val       = row[1] if len(row) > 1 else ''
            val_color = row[2] if len(row) > 2 else C_TEXT
            cv2.putText(img, label, (x + pad, ry), _FONT, fs, C_DIM, _FT, cv2.LINE_AA)
            cv2.putText(img, val,   (x + pad + col_w, ry),
                        _FONT, fs, val_color, _FT, cv2.LINE_AA)


def _panel_width(title: str, rows, pad: int = 5, fs: float | None = None) -> int:
    fs     = fs if fs is not None else 0.40
    key_ws = [cv2.getTextSize(r[0], _FONT, fs, _FT)[0][0]
              for r in rows if not isinstance(r, str) and len(r) >= 2]
    col_w  = (max(key_ws) + 8) if key_ws else 0
    cands  = []
    if title:
        cands.append(cv2.getTextSize(title, _FONT, fs, _FT)[0][0])
    for row in rows:
        if isinstance(row, str):
            cands.append(cv2.getTextSize(row, _FONT, fs, _FT)[0][0])
        elif len(row) >= 2:
            cands.append(col_w + cv2.getTextSize(row[1], _FONT, fs, _FT)[0][0])
    return (max(cands) if cands else 60) + 2 * pad

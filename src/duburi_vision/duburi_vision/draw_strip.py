"""draw_strip -- mission-control UI strip (326px below the video frame).

render_ui_strip() assembles the complete strip as a new numpy array.
Layout (top -> bottom):

  Row 1  h=26   Header bar: "BRACU DUBURI"  FPS  LIVE/VIDEO badge
  Row 2  h=40   ERR_X and ERR_Y needle gauges (full-width, each half)
  Row 3  h=120  Three proportional columns (each w//3):
                  Left   PERCEPTION panel + CLASSES panel stacked
                  Middle ALIGNMENT panel (full column height)
                  Right  ERR_X sparkline / ERR_Y sparkline / CONF bar
  Row 4  h=88   Four proportional zones:
                  A ~13%  compass rose + yaw-source label (radius 30px)
                  B ~28%  STATE panel (depth, yaw, mode, armed, battery)
                  C ~38%  ERR_X + ERR_Y sparklines side-by-side + CONF below
                  D ~21%  full-width altimeter depth gauge
  Row 5  h=26   Full-width heading tape
  Row 6  h=26   Live: pipeline health row  |  Video: progress bar
  ──────────────────────────────────────────────────────────────────────
  Total  326px  ->  _STRIP_H = 326
  Fonts scale with frame width: sf = max(1.0, w / 640)
"""
from __future__ import annotations

from typing import Dict, List, Optional, Sequence

import cv2
import numpy as np

from .draw_widgets import (
    C_BG, C_ACCENT, C_AMBER, C_OK, C_ERR, C_TEXT, C_DIM, C_BORDER,
    _FONT, _FT,
    needle_gauge, sparkline, confidence_bar,
    altimeter_depth, mini_compass, heading_tape,
    video_progress, health_row,
)
from .detection.detector import Detection

_STRIP_H  = 326   # 26+40+120+88+26+26

_R1_Y, _R1_H = 0,   26
_R2_Y, _R2_H = 26,  40
_R3_Y, _R3_H = 66,  120
_R4_Y, _R4_H = 186, 88
_R5_Y, _R5_H = 274, 26
_R6_Y, _R6_H = 300, 26

_SFS  = 0.44   # panel font scale (larger base for readability)
_SLH  = 14     # panel line height
_SPAD = 4      # panel inner padding


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
    """Build and return the _STRIP_H x w UI strip drawn at native width."""
    strip = np.full((_STRIP_H, w, 3), C_BG, dtype=np.uint8)
    cv2.line(strip, (0, 0), (w - 1, 0), C_ACCENT, 2)

    sf     = max(1.0, w / 640.0)   # font/widget scale factor — 1.0 at 640px, 2.0 at 1280px
    yaw    = state.yaw_deg if state is not None else float('nan')
    ex_h   = list(err_x_history or [])
    ey_h   = list(err_y_history or [])
    cf_h   = list(conf_history  or [])

    _draw_header_row(strip, w, fps, video_mode, is_paused, sf)
    _draw_err_gauges(strip, w, frame_h, primary, deadband, sf)
    _draw_panels_row(strip, w, frame_h, detections, primary, source,
                     healthy, tracking_on, n_tracks, primary_track_id,
                     configured_classes, show_alignment, deadband,
                     ex_h, ey_h, cf_h, sf)
    _draw_instruments_row(strip, w, yaw, yaw_source, state, ex_h, ey_h, cf_h, sf)

    cv2.line(strip, (4, _R5_Y - 1), (w - 4, _R5_Y - 1), C_BORDER, 1)
    heading_tape(strip, 4, _R5_Y, w - 8, _R5_H, yaw, show_readout=False, fs_scale=sf)

    if video_mode and video_position is not None:
        cur_f, tot_f = video_position
        video_progress(strip, 0, _R6_Y, w, _R6_H,
                       cur_f, tot_f, fps or 30.0, is_paused, fs_scale=sf)
    else:
        health_row(strip, 0, _R6_Y, w, _R6_H, pipeline_health or {}, fs_scale=sf)

    cv2.line(strip, (0, _STRIP_H - 1), (w - 1, _STRIP_H - 1), C_ACCENT, 1)
    return strip


# ── Row 1: Header ─────────────────────────────────────────────────────────── #

def _draw_header_row(strip: np.ndarray, w: int, fps: float,
                     video_mode: bool, is_paused: bool, sf: float) -> None:
    fs_brand = 0.44 * sf
    fs_side  = 0.32 * sf
    brand = 'BRACU  DUBURI'
    (bw, bh), _ = cv2.getTextSize(brand, _FONT, fs_brand, _FT)
    bx = (w - bw) // 2
    by = _R1_Y + 17
    cv2.putText(strip, brand, (bx, by), _FONT, fs_brand, C_TEXT, _FT, cv2.LINE_AA)
    dot_y = by - bh // 2
    cv2.circle(strip, (bx - 10, dot_y), 3, C_ACCENT, -1, cv2.LINE_AA)
    cv2.circle(strip, (bx + bw + 10, dot_y), 3, C_ACCENT, -1, cv2.LINE_AA)

    cv2.putText(strip, f'{fps:.0f} Hz', (8, by), _FONT, fs_side, C_DIM, _FT, cv2.LINE_AA)

    if video_mode:
        badge     = '|| VIDEO SIM' if is_paused else '>  VIDEO SIM'
        badge_col = C_AMBER if is_paused else C_ACCENT
    else:
        badge     = 'LIVE'
        badge_col = C_OK
    (mw, mh), _ = cv2.getTextSize(badge, _FONT, fs_side, _FT)
    badge_x = w - mw - 8
    cv2.putText(strip, badge, (badge_x, by), _FONT, fs_side, badge_col, _FT, cv2.LINE_AA)
    if not video_mode:
        cv2.circle(strip, (badge_x - 8, by - mh // 2 - 1), 3, C_OK, -1, cv2.LINE_AA)

    cv2.line(strip, (4, _R2_Y - 1), (w - 4, _R2_Y - 1), C_BORDER, 1)


# ── Row 2: ERR gauges ─────────────────────────────────────────────────────── #

def _draw_err_gauges(strip: np.ndarray, w: int, frame_h: int,
                     primary: Optional[Detection], deadband: float,
                     sf: float) -> None:
    gpad = 8
    gh   = _R2_H - 18
    gw   = (w - 3 * gpad) // 2
    fs_lbl = 0.32 * sf

    fw = max(w, 1)
    ex = ey = 0.0
    if primary is not None:
        ex = (primary.cx - fw / 2.0) / max(fw / 2.0, 1.0)
        ey = (primary.cy - max(frame_h, 1) / 2.0) / max(frame_h / 2.0, 1.0)

    gauge_y = _R2_Y + 20
    cv2.putText(strip, 'ERR_X', (gpad, _R2_Y + 13),
                _FONT, fs_lbl, C_DIM, _FT, cv2.LINE_AA)
    needle_gauge(strip, gpad, gauge_y, gw, gh, ex,
                 vmin=-1.0, vmax=1.0, deadband=deadband, fs_scale=sf)

    cv2.putText(strip, 'ERR_Y', (gpad * 2 + gw, _R2_Y + 13),
                _FONT, fs_lbl, C_DIM, _FT, cv2.LINE_AA)
    needle_gauge(strip, gpad * 2 + gw, gauge_y, gw, gh, ey,
                 vmin=-1.0, vmax=1.0, deadband=deadband, fs_scale=sf)

    cv2.line(strip, (4, _R3_Y - 1), (w - 4, _R3_Y - 1), C_BORDER, 1)


# ── Row 3: Panels (3 proportional columns) ────────────────────────────────── #

def _draw_panels_row(strip: np.ndarray, w: int, frame_h: int,
                     detections: List[Detection],
                     primary: Optional[Detection],
                     source: str, healthy: bool,
                     tracking_on: bool, n_tracks: int, primary_track_id,
                     configured_classes: List[str],
                     show_alignment: bool, deadband: float,
                     err_x_hist: List[float],
                     err_y_hist: List[float],
                     conf_hist:  List[float],
                     sf: float) -> None:
    col_w = w // 3
    pad   = 5
    sfs   = _SFS * sf   # scaled panel font

    cv2.line(strip, (col_w,     _R3_Y + 2), (col_w,     _R3_Y + _R3_H - 2), C_BORDER, 1)
    cv2.line(strip, (col_w * 2, _R3_Y + 2), (col_w * 2, _R3_Y + _R3_H - 2), C_BORDER, 1)

    # ── Left column: PERCEPTION + CLASSES stacked ──────────────────────────── #
    lx = pad
    ly = _R3_Y + pad
    lw = col_w - pad * 2

    perc_rows: list = [
        ('SRC', str(source),          C_TEXT),
        ('DET', str(len(detections)), C_ACCENT if detections else C_DIM),
    ]
    if primary is not None:
        perc_rows.append(('TGT', primary.class_name, C_ACCENT))
    if tracking_on:
        tid = (f'#{primary_track_id}' if primary_track_id is not None
               else f'n={n_tracks}')
        perc_rows.append(('TRK', tid, C_ACCENT if n_tracks else C_DIM))

    badge_border = (C_OK  if (healthy and detections)
                    else (C_ERR if not healthy else C_BORDER))
    _mc_panel(strip, lx, ly, 'PERCEPTION', perc_rows,
              border=badge_border, pad=_SPAD, line_h=_SLH, fs=sfs, panel_w=lw)

    if configured_classes:
        cls_ly       = ly + _panel_height('PERCEPTION', perc_rows,
                                          pad=_SPAD, line_h=_SLH) + 4
        active_lower = {d.class_name.lower() for d in detections}
        any_active   = any(c.lower() in active_lower for c in configured_classes)
        cls_rows     = [('', c.upper(),
                         C_ACCENT if c.lower() in active_lower else C_DIM)
                        for c in configured_classes]
        max_cls_y    = (_R3_Y + _R3_H - pad
                        - _panel_height('CLASSES', cls_rows, pad=_SPAD, line_h=_SLH))
        if cls_ly <= max_cls_y:
            _mc_panel(strip, lx, cls_ly, 'CLASSES', cls_rows,
                      border=C_ACCENT if any_active else C_BORDER,
                      pad=_SPAD, line_h=_SLH, fs=sfs, panel_w=lw)

    # ── Middle column: ALIGNMENT full-height ───────────────────────────────── #
    mx = col_w + pad
    my = _R3_Y + pad
    mw = col_w - pad * 2

    if show_alignment:
        if primary is None:
            al_rows: list = [('STATUS', 'NO TARGET', C_ERR)]
            al_border     = C_ERR
        else:
            fw  = strip.shape[1]
            ex  = (primary.cx - fw / 2.0) / max(fw / 2.0, 1.0)
            ey  = (primary.cy - frame_h  / 2.0) / max(frame_h  / 2.0, 1.0)
            aligned = abs(ex) < deadband and abs(ey) < deadband
            if tracking_on and n_tracks > 0:
                tid_str       = (f'#{primary_track_id}'
                                 if primary_track_id is not None else f'n={n_tracks}')
                st_val, st_col = f'TRK {tid_str}', C_ACCENT
            else:
                st_val, st_col = 'DETECTED', C_TEXT
            bbox_w = int(primary.xyxy[2] - primary.xyxy[0])
            bbox_h_px = int(primary.xyxy[3] - primary.xyxy[1])
            al_rows = [
                ('STATUS', st_val,                                            st_col),
                ('ERR_X',  f'{ex:+.3f}',           C_OK if abs(ex) < deadband else C_AMBER),
                ('ERR_Y',  f'{ey:+.3f}',           C_OK if abs(ey) < deadband else C_AMBER),
                ('AREA',   f'{(primary.area / max(fw * frame_h, 1) * 100):.1f}%', C_TEXT),
                ('CONF',   f'{primary.score:.2f}',                            C_TEXT),
                ('SIZE',   f'{bbox_w}x{bbox_h_px}px',                        C_DIM),
            ]
            al_border = C_OK if aligned else C_AMBER
        _mc_panel(strip, mx, my, 'ALIGNMENT', al_rows,
                  border=al_border, pad=_SPAD, line_h=_SLH, fs=sfs, panel_w=mw)

    # ── Right column: sparklines using full column width ───────────────────── #
    rx   = col_w * 2 + pad
    rw   = w - col_w * 2 - pad * 2
    cy   = _R3_Y + pad
    lbl  = 12    # pixels for the label text line
    rh   = _R3_H - pad * 2
    sp_h = max(18, (rh - 3 * (lbl + 3)) // 3)   # larger sparklines (was max(10,...))
    fs_lbl = 0.30 * sf

    for label, values, color in (
        ('ERR_X', err_x_hist, C_AMBER),
        ('ERR_Y', err_y_hist, (80, 180, 240)),
    ):
        cv2.putText(strip, label, (rx, cy + lbl - 1),
                    _FONT, fs_lbl, C_DIM, _FT, cv2.LINE_AA)
        sparkline(strip, rx, cy + lbl + 1, rw, sp_h, values, color=color)
        cy += lbl + 1 + sp_h + 3

    cv2.putText(strip, 'CONF', (rx, cy + lbl - 1),
                _FONT, fs_lbl, C_DIM, _FT, cv2.LINE_AA)
    confidence_bar(strip, rx, cy + lbl + 1, rw, sp_h,
                   conf_hist[-1] if conf_hist else 0.0, fs_scale=sf)


# ── Row 4: Instruments (4 proportional zones) ─────────────────────────────── #

def _draw_instruments_row(strip: np.ndarray, w: int,
                          yaw: float, yaw_source: str, state,
                          err_x_hist: List[float],
                          err_y_hist: List[float],
                          conf_hist:  List[float],
                          sf: float) -> None:
    cv2.line(strip, (4, _R4_Y), (w - 4, _R4_Y), C_BORDER, 1)

    za_w = int(w * 0.13)
    zb_w = int(w * 0.28)
    zc_w = int(w * 0.38)
    za_x = 0
    zb_x = za_w
    zc_x = za_w + zb_w
    zd_x = za_w + zb_w + zc_w
    zd_w = w - zd_x

    for zx in (zb_x, zc_x, zd_x):
        cv2.line(strip, (zx, _R4_Y + 2), (zx, _R4_Y + _R4_H - 2), C_BORDER, 1)

    # ── Zone A: Compass ─────────────────────────────────────────────────────── #
    cmp_r  = min(30, (_R4_H - 22) // 2)   # 30px radius at R4_H=88 (was 21px at 74)
    cmp_cx = za_x + za_w // 2
    cmp_cy = _R4_Y + cmp_r + 5
    mini_compass(strip, cmp_cx, cmp_cy, cmp_r, yaw, fs_scale=sf)

    _SRC_MAP = {'mavlink_ahrs': 'MAVLNK', 'bno085': 'BNO085',
                'dvl': 'DVL', 'bno085_dvl': 'BNO+DVL'}
    src_lbl = _SRC_MAP.get(yaw_source, (yaw_source or '?').upper())
    fs_src  = 0.28 * sf
    (sw, _), _ = cv2.getTextSize(src_lbl, _FONT, fs_src, _FT)
    src_y = min(cmp_cy + cmp_r + 12, _R4_Y + _R4_H - 3)
    cv2.putText(strip, src_lbl, (cmp_cx - sw // 2, src_y),
                _FONT, fs_src, C_DIM, _FT, cv2.LINE_AA)

    # ── Zone B: STATE panel ──────────────────────────────────────────────────── #
    sfs = _SFS * sf
    if state is not None:
        armed   = bool(state.armed)
        depth_v = state.depth_m         if not np.isnan(state.depth_m)         else 0.0
        yaw_v   = state.yaw_deg         if not np.isnan(state.yaw_deg)         else 0.0
        batt_v  = state.battery_voltage if not np.isnan(state.battery_voltage) else 0.0
        st_rows = [
            ('DEPTH', f'{depth_v:+.2f}m',        C_TEXT),
            ('YAW',   f'{yaw_v:.1f}deg',          C_TEXT),
            ('MODE',  state.mode or '?',           C_ACCENT if armed else C_DIM),
            ('ARMED', 'YES' if armed else 'no',    C_OK     if armed else C_DIM),
            ('BAT',   f'{batt_v:.1f}V',            _bat_color(batt_v)),
        ]
        _mc_panel(strip, zb_x + 5, _R4_Y + 4, 'STATE', st_rows,
                  border=C_OK if armed else C_BORDER,
                  pad=_SPAD, line_h=_SLH, fs=sfs, panel_w=zb_w - 10)

    # ── Zone C: ERR history sparklines + CONF bar ────────────────────────────── #
    sp_pad   = 5
    sp_w     = (zc_w - sp_pad * 3) // 2
    sp_h_top = (_R4_H - 34) // 2   # at R4_H=88 → 27px (was ~20px at 74)
    sy0      = _R4_Y + 15
    fs_sp    = 0.29 * sf

    cv2.putText(strip, 'ERR_X', (zc_x + sp_pad, sy0 - 3),
                _FONT, fs_sp, C_DIM, _FT, cv2.LINE_AA)
    sparkline(strip, zc_x + sp_pad, sy0, sp_w, sp_h_top,
              err_x_hist, color=C_AMBER)

    cv2.putText(strip, 'ERR_Y', (zc_x + sp_pad * 2 + sp_w, sy0 - 3),
                _FONT, fs_sp, C_DIM, _FT, cv2.LINE_AA)
    sparkline(strip, zc_x + sp_pad * 2 + sp_w, sy0, sp_w, sp_h_top,
              err_y_hist, color=(80, 180, 240))

    cy2  = sy0 + sp_h_top + 10
    cb_w = zc_w - sp_pad * 2
    cv2.putText(strip, 'CONF', (zc_x + sp_pad, cy2 - 3),
                _FONT, fs_sp, C_DIM, _FT, cv2.LINE_AA)
    confidence_bar(strip, zc_x + sp_pad, cy2, cb_w, sp_h_top,
                   conf_hist[-1] if conf_hist else 0.0, fs_scale=sf)

    # ── Zone D: Full-width altimeter (battery shown in STATE panel) ──────────── #
    alt_w = max(28, zd_w - 6)          # fill Zone D width (was fixed 20px stub)
    alt_x = zd_x + (zd_w - alt_w) // 2
    alt_y = _R4_Y + 5
    alt_h = max(8, _R4_H - 10)
    depth = (state.depth_m
             if state is not None and not np.isnan(state.depth_m)
             else float('nan'))
    altimeter_depth(strip, alt_x, alt_y, alt_w, alt_h, depth,
                    max_depth=5.0, fs_scale=sf)


# ── Panel helpers ─────────────────────────────────────────────────────────── #

def _mc_panel(img: np.ndarray, x: int, y: int, title: str, rows, *,
              border=None, pad: int = 5, line_h: int = 17,
              fs: float | None = None, panel_w: int | None = None) -> None:
    fs     = fs if fs is not None else 0.40
    border = border or C_BORDER

    key_ws = [cv2.getTextSize(r[0], _FONT, fs, _FT)[0][0]
              for r in rows if not isinstance(r, str) and len(r) >= 2]
    col_w  = (max(key_ws) + 8) if key_ws else 0

    if panel_w is None:
        candidates: list = []
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
            cv2.putText(img, label, (x + pad,           ry), _FONT, fs, C_DIM,     _FT, cv2.LINE_AA)
            cv2.putText(img, val,   (x + pad + col_w,   ry), _FONT, fs, val_color, _FT, cv2.LINE_AA)


def _panel_height(title: str, rows, pad: int = 5, line_h: int = 17) -> int:
    return line_h * (len(rows) + (1 if title else 0)) + 2 * pad


def _panel_width(title: str, rows, pad: int = 5, fs: float | None = None) -> int:
    fs     = fs if fs is not None else 0.40
    key_ws = [cv2.getTextSize(r[0], _FONT, fs, _FT)[0][0]
              for r in rows if not isinstance(r, str) and len(r) >= 2]
    col_w  = (max(key_ws) + 8) if key_ws else 0
    cands: list = []
    if title:
        cands.append(cv2.getTextSize(title, _FONT, fs, _FT)[0][0])
    for row in rows:
        if isinstance(row, str):
            cands.append(cv2.getTextSize(row, _FONT, fs, _FT)[0][0])
        elif len(row) >= 2:
            cands.append(col_w + cv2.getTextSize(row[1], _FONT, fs, _FT)[0][0])
    return (max(cands) if cands else 60) + 2 * pad


def _bat_color(voltage: float) -> tuple:
    if np.isnan(voltage) or voltage <= 0:
        return C_DIM
    pct = float(np.clip((voltage - 14.0) / (16.8 - 14.0), 0.0, 1.0))
    return C_OK if pct > 0.60 else (C_AMBER if pct > 0.20 else C_ERR)

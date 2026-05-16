"""draw_strip -- mission-control UI strip below the video frame.

render_ui_strip() assembles the complete strip as a new numpy array.
Layout (top -> bottom):

  Row 1  h=26*sf   Header bar: "BRACU DUBURI"  FPS  LIVE/VIDEO badge
  Row 2  h=40*sf   ERR_X and ERR_Y needle gauges (full-width, each half)
  Row 3  h=120*sf  Three proportional columns (each w//3):
                     Left   PERCEPTION panel + CLASSES panel stacked
                     Middle ALIGNMENT panel (full column height)
                     Right  ERR_X sparkline / ERR_Y sparkline / CONF bar
  Row 4  h=88*sf   Three proportional zones:
                     A ~13%  compass rose + yaw-source label
                     B ~55%  STATE panel (depth, yaw, mode, armed, battery)
                     D ~32%  full-width altimeter depth gauge
  Row 5  h=26*sf   Full-width heading tape
  Row 6  h=26*sf   Live: pipeline health row  |  Video: progress bar
  ──────────────────────────────────────────────────────────────────────
  Base  326px  at 640px native width (sf=1.0)
  All row heights scale with sf = max(1.0, render_width / 640).
"""
from __future__ import annotations

from typing import Dict, List, Optional, Sequence

import cv2
import numpy as np

from .draw_widgets import (
    C_BG, C_ACCENT, C_AMBER, C_OK, C_ERR, C_TEXT, C_DIM, C_BORDER,
    pil_text, pil_text_size,
    needle_gauge, sparkline, confidence_bar,
    altimeter_depth, mini_compass, heading_tape,
    video_progress, health_row,
)
from .detection.detector import Detection

# Base row heights at native 640px (sf=1.0).  Actual heights scale with sf.
_R1_H = 26
_R2_H = 40
_R3_H = 120
_R4_H = 88
_R5_H = 26
_R6_H = 26

# Base strip height — used for backwards-compat export only; actual height is dynamic.
_STRIP_H = _R1_H + _R2_H + _R3_H + _R4_H + _R5_H + _R6_H  # 326

_SFS  = 0.374  # base panel font scale (multiplied by sf at runtime) — 15% smaller than 0.44
_SLH  = 12     # base panel line height (scaled with sf at runtime)
_SPAD = 4      # panel inner padding (NOT scaled — prevents overflow at sf=2)


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
                    pipeline_health: Optional[Dict[str, bool]] = None,
                    depth_rate: float = 0.0) -> np.ndarray:
    """Build and return the UI strip.  Height scales with render width."""
    sf = max(1.0, w / 640.0)

    # Dynamic row heights and Y positions
    r1_h = int(_R1_H * sf);  r1_y = 0
    r2_h = int(_R2_H * sf);  r2_y = r1_y + r1_h
    r3_h = int(_R3_H * sf);  r3_y = r2_y + r2_h
    r4_h = int(_R4_H * sf);  r4_y = r3_y + r3_h
    r5_h = int(_R5_H * sf);  r5_y = r4_y + r4_h
    r6_h = int(_R6_H * sf);  r6_y = r5_y + r5_h
    strip_h = r6_y + r6_h

    slh  = max(_SLH, int(_SLH * sf))  # scale line height
    spad = _SPAD                        # NOT scaled — prevents STATE panel overflow

    yaw  = state.yaw_deg if state is not None else float('nan')
    ex_h = list(err_x_history or [])
    ey_h = list(err_y_history or [])
    cf_h = list(conf_history  or [])

    strip = np.full((strip_h, w, 3), C_BG, dtype=np.uint8)
    cv2.line(strip, (0, 0), (w - 1, 0), C_ACCENT, 2)

    _draw_header_row(strip, w, fps, video_mode, is_paused, sf, r1_y, r2_y)
    _draw_err_gauges(strip, w, frame_h, primary, deadband, sf, r2_y, r2_h, r3_y)
    _draw_panels_row(strip, w, frame_h, detections, primary, source,
                     healthy, tracking_on, n_tracks, primary_track_id,
                     configured_classes, show_alignment, deadband,
                     ex_h, ey_h, cf_h, sf, r3_y, r3_h, slh, spad)
    _draw_instruments_row(strip, w, yaw, yaw_source, state,
                          sf, r4_y, r4_h, slh, spad, depth_rate=depth_rate)

    cv2.line(strip, (4, r5_y - 1), (w - 4, r5_y - 1), C_BORDER, 1)
    heading_tape(strip, 4, r5_y, w - 8, r5_h, yaw, show_readout=False, fs_scale=sf)

    if video_mode and video_position is not None:
        cur_f, tot_f = video_position
        video_progress(strip, 0, r6_y, w, r6_h,
                       cur_f, tot_f, fps or 30.0, is_paused, fs_scale=sf)
    else:
        health_row(strip, 0, r6_y, w, r6_h, pipeline_health or {}, fs_scale=sf)

    cv2.line(strip, (0, strip_h - 1), (w - 1, strip_h - 1), C_ACCENT, 1)
    return strip


# ── Row 1: Header ─────────────────────────────────────────────────────────── #

def _draw_header_row(strip: np.ndarray, w: int, fps: float,
                     video_mode: bool, is_paused: bool, sf: float,
                     r1_y: int, r2_y: int) -> None:
    fs_brand = 0.44 * sf
    fs_side  = 0.32 * sf
    brand = 'BRACU  DUBURI'
    bw, bh = pil_text_size(brand, fs_brand)
    bx = (w - bw) // 2
    by = r1_y + int(17 * sf)
    pil_text(strip, brand, (bx, by), fs_brand, C_TEXT)
    dot_y = by - bh // 2
    cv2.circle(strip, (bx - 10, dot_y), 3, C_ACCENT, -1, cv2.LINE_AA)
    cv2.circle(strip, (bx + bw + 10, dot_y), 3, C_ACCENT, -1, cv2.LINE_AA)

    pil_text(strip, f'{fps:.0f} Hz', (8, by), fs_side, C_DIM)

    if video_mode:
        badge     = '|| VIDEO SIM' if is_paused else '>  VIDEO SIM'
        badge_col = C_AMBER if is_paused else C_ACCENT
    else:
        badge     = 'LIVE'
        badge_col = C_OK
    mw, mh = pil_text_size(badge, fs_side)
    badge_x = w - mw - 8
    pil_text(strip, badge, (badge_x, by), fs_side, badge_col)
    if not video_mode:
        cv2.circle(strip, (badge_x - 8, by - mh // 2 - 1), 3, C_OK, -1, cv2.LINE_AA)

    cv2.line(strip, (4, r2_y - 1), (w - 4, r2_y - 1), C_BORDER, 1)


# ── Row 2: ERR gauges ─────────────────────────────────────────────────────── #

def _draw_err_gauges(strip: np.ndarray, w: int, frame_h: int,
                     primary: Optional[Detection], deadband: float,
                     sf: float, r2_y: int, r2_h: int, r3_y: int) -> None:
    gpad = 8
    gh   = r2_h - int(18 * sf)
    gw   = (w - 3 * gpad) // 2
    fs_lbl = 0.32 * sf

    frame_w = max(w, 1)      # video frame width == strip render width
    ex = ey = 0.0
    if primary is not None:
        ex = (primary.cx - frame_w / 2.0) / max(frame_w / 2.0, 1.0)
        ey = (primary.cy - max(frame_h, 1) / 2.0) / max(frame_h / 2.0, 1.0)

    gauge_y = r2_y + int(20 * sf)
    lbl_y   = r2_y + int(13 * sf)
    pil_text(strip, 'ERR_X', (gpad, lbl_y), fs_lbl, C_DIM)
    needle_gauge(strip, gpad, gauge_y, gw, gh, ex,
                 vmin=-1.0, vmax=1.0, deadband=deadband, fs_scale=sf)

    pil_text(strip, 'ERR_Y', (gpad * 2 + gw, lbl_y), fs_lbl, C_DIM)
    needle_gauge(strip, gpad * 2 + gw, gauge_y, gw, gh, ey,
                 vmin=-1.0, vmax=1.0, deadband=deadband, fs_scale=sf)

    cv2.line(strip, (4, r3_y - 1), (w - 4, r3_y - 1), C_BORDER, 1)


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
                     sf: float,
                     r3_y: int, r3_h: int,
                     slh: int, spad: int) -> None:
    col_w = w // 3
    pad   = 7
    sfs   = _SFS * sf

    cv2.line(strip, (col_w,     r3_y + 2), (col_w,     r3_y + r3_h - 2), C_BORDER, 1)
    cv2.line(strip, (col_w * 2, r3_y + 2), (col_w * 2, r3_y + r3_h - 2), C_BORDER, 1)

    # ── Left column: PERCEPTION + CLASSES stacked ──────────────────────────── #
    lx = pad
    ly = r3_y + pad
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
              border=badge_border, pad=spad, line_h=slh, fs=sfs, panel_w=lw)

    if configured_classes:
        cls_ly       = ly + _panel_height('PERCEPTION', perc_rows,
                                          pad=spad, line_h=slh) + max(4, int(6 * sf))
        active_lower = {d.class_name.lower() for d in detections}
        any_active   = any(c.lower() in active_lower for c in configured_classes)
        cls_rows     = [('', c.upper(),
                         C_ACCENT if c.lower() in active_lower else C_DIM)
                        for c in configured_classes]
        max_cls_y    = (r3_y + r3_h - pad
                        - _panel_height('CLASSES', cls_rows, pad=spad, line_h=slh))
        if cls_ly <= max_cls_y:
            _mc_panel(strip, lx, cls_ly, 'CLASSES', cls_rows,
                      border=C_ACCENT if any_active else C_DIM,
                      pad=spad, line_h=slh, fs=sfs, panel_w=lw)

    # ── Middle column: ALIGNMENT full-height ───────────────────────────────── #
    mx = col_w + pad
    my = r3_y + pad
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
            bbox_w    = int(primary.xyxy[2] - primary.xyxy[0])
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
                  border=al_border, pad=spad, line_h=slh, fs=sfs, panel_w=mw)

    # ── Right column: sparklines using full column width ───────────────────── #
    r_margin = max(4, int(4 * sf))
    rx   = col_w * 2 + pad
    rw   = w - col_w * 2 - pad * 2 - r_margin
    cy   = r3_y + pad
    lbl  = max(12, int(12 * sf))
    gap  = max(4, int(4 * sf))
    rh   = r3_h - pad * 2
    sp_h = max(18, (rh - 3 * (lbl + 1 + gap)) // 3)
    fs_lbl = 0.30 * sf

    for label, values, color in (
        ('ERR_X', err_x_hist, C_AMBER),
        ('ERR_Y', err_y_hist, (80, 180, 240)),
    ):
        pil_text(strip, label, (rx, cy + lbl - 1), fs_lbl, C_DIM)
        sparkline(strip, rx, cy + lbl + 1, rw, sp_h, values, color=color)
        cy += lbl + 1 + sp_h + gap

    pil_text(strip, 'CONF', (rx, cy + lbl - 1), fs_lbl, C_DIM)
    confidence_bar(strip, rx, cy + lbl + 1, rw, sp_h,
                   conf_hist[-1] if conf_hist else 0.0, fs_scale=sf)


# ── Row 4: Instruments (4 proportional zones) ─────────────────────────────── #

def _draw_instruments_row(strip: np.ndarray, w: int,
                          yaw: float, yaw_source: str, state,
                          sf: float,
                          r4_y: int, r4_h: int,
                          slh: int, spad: int,
                          depth_rate: float = 0.0) -> None:
    cv2.line(strip, (4, r4_y), (w - 4, r4_y), C_BORDER, 1)

    _ZA_FRAC = 0.13   # compass zone width fraction
    _ZB_FRAC = 0.55   # STATE panel zone width fraction
    za_w = int(w * _ZA_FRAC)
    zb_w = int(w * _ZB_FRAC)
    za_x = 0
    zb_x = za_w
    zd_x = za_w + zb_w
    zd_w = w - zd_x

    for zx in (zb_x, zd_x):
        cv2.line(strip, (zx, r4_y + 2), (zx, r4_y + r4_h - 2), C_BORDER, 1)

    # ── Zone A: Compass ─────────────────────────────────────────────────────── #
    cmp_r  = min(int(30 * sf), (r4_h - int(22 * sf)) // 2)
    cmp_cx = za_x + za_w // 2
    cmp_cy = r4_y + cmp_r + int(5 * sf)
    mini_compass(strip, cmp_cx, cmp_cy, cmp_r, yaw, fs_scale=sf)

    _SRC_MAP = {'mavlink_ahrs': 'MAVLNK', 'bno085': 'BNO085',
                'dvl': 'DVL', 'bno085_dvl': 'BNO+DVL'}
    src_lbl = _SRC_MAP.get(yaw_source, (yaw_source or '?').upper())
    fs_src  = 0.28 * sf
    sw, _   = pil_text_size(src_lbl, fs_src)
    src_y   = min(cmp_cy + cmp_r + int(12 * sf), r4_y + r4_h - 3)
    pil_text(strip, src_lbl, (cmp_cx - sw // 2, src_y), fs_src, C_DIM)

    # ── Zone B: STATE panel ──────────────────────────────────────────────────── #
    sfs = _SFS * sf
    if state is not None:
        armed   = bool(state.armed)
        depth_v = state.depth_m         if not np.isnan(state.depth_m)         else 0.0
        yaw_v   = state.yaw_deg         if not np.isnan(state.yaw_deg)         else 0.0
        batt_v  = state.battery_voltage if not np.isnan(state.battery_voltage) else 0.0
        dep_str = f'{depth_v:+.2f}m'
        if abs(depth_rate) > 0.02:
            arrow = '↑' if depth_rate < 0 else '↓'
            dep_str += f' {arrow}{abs(depth_rate):.2f}'
        st_rows = [
            ('DEPTH', dep_str,                     C_TEXT),
            ('YAW',   f'{yaw_v:.1f}deg',          C_TEXT),
            ('MODE',  state.mode or '?',           C_ACCENT if armed else C_DIM),
            ('ARMED', 'YES' if armed else 'no',    C_OK     if armed else C_DIM),
            ('BAT',   f'{batt_v:.1f}V',            _bat_color(batt_v)),
        ]
        _mc_panel(strip, zb_x + int(5 * sf), r4_y + int(4 * sf), 'STATE', st_rows,
                  border=C_OK if armed else C_BORDER,
                  pad=spad, line_h=slh, fs=sfs, panel_w=zb_w - 10)

    # ── Zone D: Full-width altimeter ─────────────────────────────────────────── #
    alt_w = max(28, zd_w - 6)
    alt_x = zd_x + (zd_w - alt_w) // 2
    alt_y = r4_y + int(5 * sf)
    alt_h = max(int(8 * sf), r4_h - int(10 * sf))
    depth = (state.depth_m
             if state is not None and not np.isnan(state.depth_m)
             else float('nan'))
    altimeter_depth(strip, alt_x, alt_y, alt_w, alt_h, depth,
                    max_depth=8.0, fs_scale=sf)


# ── Panel helpers ─────────────────────────────────────────────────────────── #

def _mc_panel(img: np.ndarray, x: int, y: int, title: str, rows, *,
              border=None, pad: int = 5, line_h: int = 17,
              fs: float | None = None, panel_w: int | None = None) -> None:
    fs     = fs if fs is not None else 0.40
    border = border or C_BORDER

    key_ws = [pil_text_size(r[0], fs)[0]
              for r in rows if not isinstance(r, str) and len(r) >= 2]
    col_w  = (max(key_ws) + 8) if key_ws else 0

    if panel_w is None:
        candidates: list = []
        if title:
            candidates.append(pil_text_size(title, fs)[0])
        for row in rows:
            if isinstance(row, str):
                candidates.append(pil_text_size(row, fs)[0])
            elif len(row) >= 2:
                candidates.append(col_w + pil_text_size(row[1], fs)[0])
        panel_w = (max(candidates) if candidates else 60) + 2 * pad

    panel_h = line_h * (len(rows) + (1 if title else 0)) + 2 * pad

    ov = img.copy()
    cv2.rectangle(ov, (x, y), (x + panel_w, y + panel_h), C_BG, -1)
    cv2.addWeighted(ov, 0.82, img, 0.18, 0, dst=img)
    cv2.rectangle(img, (x, y), (x + panel_w, y + panel_h), border, 1, cv2.LINE_AA)

    row_off = 0
    if title:
        ty = y + pad + line_h - 3
        pil_text(img, title, (x + pad, ty), fs, C_ACCENT)
        cv2.line(img, (x + 1, y + pad + line_h + 1),
                 (x + panel_w - 1, y + pad + line_h + 1), C_BORDER, 1)
        row_off = 1

    for i, row in enumerate(rows):
        ry = y + pad + line_h * (i + row_off + 1) - 3
        if isinstance(row, str):
            pil_text(img, row, (x + pad, ry), fs, C_TEXT)
        else:
            label     = row[0]
            val       = row[1] if len(row) > 1 else ''
            val_color = row[2] if len(row) > 2 else C_TEXT
            pil_text(img, label, (x + pad,         ry), fs, C_DIM)
            pil_text(img, val,   (x + pad + col_w, ry), fs, val_color)


def _panel_height(title: str, rows, pad: int = 5, line_h: int = 17) -> int:
    return line_h * (len(rows) + (1 if title else 0)) + 2 * pad


def _bat_color(voltage: float) -> tuple:
    if np.isnan(voltage) or voltage <= 0:
        return C_DIM
    pct = float(np.clip((voltage - 14.0) / (16.8 - 14.0), 0.0, 1.0))
    return C_OK if pct > 0.60 else (C_AMBER if pct > 0.20 else C_ERR)

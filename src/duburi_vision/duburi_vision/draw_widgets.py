"""draw_widgets -- reusable HUD micro-widgets for the mission-control strip.

All functions draw in-place on a numpy BGR image passed as the first argument.
They return nothing; call them for their side effects on the image buffer.

Widget catalogue
----------------
  needle_gauge(img, x, y, w, h, value, ...)   horizontal ±1 error meter
  sparkline(img, x, y, w, h, values, color)   rolling line chart (no axes)
  confidence_bar(img, x, y, w, h, conf)        horizontal fill bar 0-1
  altimeter_depth(img, x, y, w, h, depth_m)   vertical tape depth gauge
  battery_bar(img, x, y, w, h, voltage, ...)   segmented battery indicator
  mini_compass(img, cx, cy, r, heading_deg)    compact compass rose + needle
  heading_tape(img, x, y, w, h, yaw_deg)       horizontal compass tape
  video_progress(img, x, y, w, h, cur, total, fps, paused)  playback bar
  health_row(img, x, y, w, h, health)          pipeline OK/FAIL status
"""

from __future__ import annotations

from typing import Dict, Sequence

import cv2
import numpy as np

# Import palette from draw.py to stay consistent.
# These are duplicated here to avoid a circular import; draw.py re-imports
# them from here after the split.
_FONT = cv2.FONT_HERSHEY_DUPLEX
_FT   = 1

# Shared palette (BGR)
C_BG      = (26,  26,  26)
C_ACCENT  = (215, 130,  35)
C_AMBER   = ( 30, 150, 235)
C_OK      = ( 75, 200,  75)
C_ERR     = ( 50,  50, 215)
C_TEXT    = (225, 228, 228)
C_DIM     = (105, 110, 110)
C_BORDER  = ( 58,  63,  63)
C_PANEL   = ( 38,  40,  46)
C_HEADER  = (200, 210, 230)


# ── Needle gauge ─────────────────────────────────────────────────────────── #

def needle_gauge(img: np.ndarray,
                 x: int, y: int, w: int, h: int,
                 value: float,
                 vmin: float = -1.0, vmax: float = 1.0,
                 label: str = '',
                 deadband: float = 0.05) -> None:
    """Horizontal needle meter showing signed error.

    Left = negative, right = positive, centre = zero.
    Deadband zone highlighted; needle color encodes magnitude.
    """
    if w < 10 or h < 4:
        return
    value = float(np.clip(value, vmin, vmax))
    span  = max(vmax - vmin, 1e-6)

    # Background bar
    cv2.rectangle(img, (x, y), (x + w, y + h), C_BG, -1)
    cv2.rectangle(img, (x, y), (x + w, y + h), C_BORDER, 1, cv2.LINE_AA)

    # Deadband zone
    cx      = x + w // 2
    db_frac = deadband / (span / 2.0)
    db_px   = max(1, int(db_frac * w / 2))
    cv2.rectangle(img, (cx - db_px, y + 1), (cx + db_px, y + h - 1),
                  C_PANEL, -1)

    # Centre line
    cv2.line(img, (cx, y + 1), (cx, y + h - 1), C_DIM, 1)

    # Needle position
    norm  = (value - vmin) / span           # [0, 1]
    nx    = x + int(norm * w)
    nx    = max(x + 2, min(x + w - 2, nx))

    # Color by magnitude
    abs_n  = abs(value) / max(abs(vmin), abs(vmax), 1e-6)
    if abs_n < deadband / (span / 2):
        color = C_OK
    elif abs_n < 0.5:
        color = C_AMBER
    else:
        color = C_ERR

    # Fill from centre to needle
    if nx > cx:
        cv2.rectangle(img, (cx, y + 2), (nx, y + h - 2), color, -1)
    elif nx < cx:
        cv2.rectangle(img, (nx, y + 2), (cx, y + h - 2), color, -1)

    # Needle tick (tall)
    cv2.line(img, (nx, y), (nx, y + h), C_TEXT, 2, cv2.LINE_AA)

    # Label (left-aligned, 2px above bar)
    if label:
        fs = 0.27
        (tw, th), _ = cv2.getTextSize(label, _FONT, fs, _FT)
        cv2.putText(img, label, (x, y - 2), _FONT, fs, C_DIM, _FT, cv2.LINE_AA)

    # Value text (right-aligned, 2px above bar)
    val_str = f'{value:+.2f}'
    fs_v    = 0.27
    (vw, _), _ = cv2.getTextSize(val_str, _FONT, fs_v, _FT)
    cv2.putText(img, val_str, (x + w - vw, y - 2),
                _FONT, fs_v, color, _FT, cv2.LINE_AA)


# ── Sparkline ────────────────────────────────────────────────────────────── #

def sparkline(img: np.ndarray,
              x: int, y: int, w: int, h: int,
              values: Sequence[float],
              color=(120, 200, 80),
              bg: bool = True) -> None:
    """Tiny rolling line chart — no axes, just the shape of the signal."""
    if w < 4 or h < 4:
        return
    if bg:
        cv2.rectangle(img, (x, y), (x + w, y + h), C_BG, -1)
        cv2.rectangle(img, (x, y), (x + w, y + h), C_BORDER, 1)

    pts = list(values)
    if len(pts) < 2:
        return

    mn  = min(pts)
    mx  = max(pts)
    rng = mx - mn if mx != mn else 1.0

    def _px(i):
        px = x + int(i / (len(pts) - 1) * (w - 1))
        py = y + h - 1 - int((pts[i] - mn) / rng * (h - 2))
        return (int(np.clip(px, x, x + w - 1)),
                int(np.clip(py, y, y + h - 1)))

    points = [_px(i) for i in range(len(pts))]
    for i in range(len(points) - 1):
        cv2.line(img, points[i], points[i + 1], color, 1, cv2.LINE_AA)


# ── Confidence bar ────────────────────────────────────────────────────────── #

def confidence_bar(img: np.ndarray,
                   x: int, y: int, w: int, h: int,
                   conf: float) -> None:
    """Horizontal fill bar 0-1 with colour-coded confidence level."""
    if w < 6 or h < 4:
        return
    conf = float(np.clip(conf, 0.0, 1.0))
    color = C_OK if conf > 0.70 else (C_AMBER if conf > 0.40 else C_ERR)
    fill  = int(conf * (w - 2))

    cv2.rectangle(img, (x, y), (x + w, y + h), C_BG, -1)
    cv2.rectangle(img, (x, y), (x + w, y + h), C_BORDER, 1, cv2.LINE_AA)
    if fill > 0:
        cv2.rectangle(img, (x + 1, y + 1), (x + 1 + fill, y + h - 1), color, -1)

    val_str = f'{int(conf * 100)}%'
    fs = 0.26
    (tw, _), _ = cv2.getTextSize(val_str, _FONT, fs, _FT)
    lx = x + w - tw - 2
    cv2.putText(img, val_str, (max(x + 2, lx), y + h - 2),
                _FONT, fs, C_TEXT, _FT, cv2.LINE_AA)


# ── Altimeter depth ───────────────────────────────────────────────────────── #

def altimeter_depth(img: np.ndarray,
                    x: int, y: int, w: int, h: int,
                    depth_m: float,
                    max_depth: float = 5.0) -> None:
    """Vertical tape depth gauge — 0m at top, max_depth at bottom.

    Shows: scale ticks every 0.5m, labeled every 1m, current depth indicator
    with arrow and numeric readout. Color: cyan/green shallow, amber/red deep.
    """
    if w < 12 or h < 20:
        return

    cv2.rectangle(img, (x, y), (x + w, y + h), C_BG, -1)
    cv2.rectangle(img, (x, y), (x + w, y + h), C_BORDER, 1, cv2.LINE_AA)

    # Units header
    cv2.putText(img, 'DEPTH', (x + 1, y - 2), _FONT, 0.22, C_DIM, _FT, cv2.LINE_AA)

    # NaN guard
    if np.isnan(depth_m):
        cv2.putText(img, '?', (x + w // 2 - 4, y + h // 2 + 4),
                    _FONT, 0.32, C_DIM, _FT, cv2.LINE_AA)
        return

    depth_abs = float(min(abs(depth_m), max_depth))

    # Ticks and labels
    step = 0.5
    m    = 0.0
    while m <= max_depth + 0.01:
        yt       = y + int(m / max_depth * h)
        is_major = abs(round(m) - m) < 0.01
        tick_len = 6 if is_major else 3
        cv2.line(img, (x, yt), (x + tick_len, yt), C_DIM, 1)
        if is_major and int(m) > 0:
            lbl = f'{int(m)}m'
            cv2.putText(img, lbl, (x + 8, min(yt + 4, y + h - 1)),
                        _FONT, 0.22, C_DIM, _FT, cv2.LINE_AA)
        m += step

    # Fill bar (depth column)
    fill_h = int(depth_abs / max_depth * h)
    if fill_h > 0:
        d_col = (C_OK   if depth_abs < 1.5
                 else C_ACCENT if depth_abs < 3.0
                 else C_AMBER  if depth_abs < 4.0
                 else C_ERR)
        cv2.rectangle(img,
                      (x + 1, y + 1),
                      (x + w - 1, y + 1 + min(fill_h, h - 2)),
                      d_col, -1)

    # Current depth indicator line + label
    ind_y = y + int(depth_abs / max_depth * h)
    ind_y = max(y + 1, min(y + h - 1, ind_y))
    d_col = (C_OK   if depth_abs < 1.5
             else C_ACCENT if depth_abs < 3.0
             else C_AMBER  if depth_abs < 4.0
             else C_ERR)
    cv2.line(img, (x, ind_y), (x + w, ind_y), d_col, 2, cv2.LINE_AA)

    lbl = f'{depth_abs:.1f}m'
    fs  = 0.28
    (lw, lh), _ = cv2.getTextSize(lbl, _FONT, fs, _FT)
    lx = x - lw - 3
    if lx < 0:
        lx = x + w + 2
    cv2.putText(img, lbl, (max(0, lx), ind_y + 4),
                _FONT, fs, d_col, _FT, cv2.LINE_AA)


# ── Battery bar ───────────────────────────────────────────────────────────── #

def battery_bar(img: np.ndarray,
                x: int, y: int, w: int, h: int,
                voltage: float,
                cell_count: int = 4) -> None:
    """Segmented battery indicator with voltage text.

    Estimates percentage from 4S LiPo voltage curve:
      full = 16.8V (4.2V/cell), empty = 14.0V (3.5V/cell).
    """
    v_full  = cell_count * 4.2
    v_empty = cell_count * 3.5
    pct     = float(np.clip((voltage - v_empty) / max(v_full - v_empty, 0.1), 0.0, 1.0))
    color   = C_OK if pct > 0.60 else (C_AMBER if pct > 0.20 else C_ERR)

    n_segs   = 10
    seg_w    = (w - 2) // n_segs
    filled   = int(pct * n_segs)

    cv2.rectangle(img, (x, y), (x + w, y + h), C_BORDER, 1, cv2.LINE_AA)
    for i in range(n_segs):
        sx1 = x + 1 + i * seg_w
        sx2 = sx1 + seg_w - 1
        if i < filled:
            cv2.rectangle(img, (sx1, y + 1), (sx2, y + h - 1), color, -1)
        else:
            cv2.rectangle(img, (sx1, y + 1), (sx2, y + h - 1), C_BG, -1)

    # Voltage + percentage label
    lbl = f'{voltage:.1f}V  {int(pct * 100)}%'
    fs  = 0.26
    (tw, _), _ = cv2.getTextSize(lbl, _FONT, fs, _FT)
    lx = x + (w - tw) // 2
    cv2.putText(img, lbl, (lx, y - 2), _FONT, fs, C_DIM, _FT, cv2.LINE_AA)


# ── Mini compass ──────────────────────────────────────────────────────────── #

def mini_compass(img: np.ndarray,
                 cx: int, cy: int,
                 radius: int,
                 heading_deg: float) -> None:
    """Compass rose with needle.  N at top, tick marks every 30°."""
    overlay = img.copy()
    cv2.circle(overlay, (cx, cy), radius, C_BG, -1)
    cv2.addWeighted(overlay, 0.80, img, 0.20, 0, dst=img)
    cv2.circle(img, (cx, cy), radius, C_BORDER, 1, cv2.LINE_AA)

    # Cardinal labels
    for ang, lbl in ((0, 'N'), (90, 'E'), (180, 'S'), (270, 'W')):
        rad  = np.radians(ang)
        tx   = cx + int((radius - 5) * np.sin(rad))
        ty   = cy - int((radius - 5) * np.cos(rad))
        fs   = 0.22
        (tw, th), _ = cv2.getTextSize(lbl, _FONT, fs, _FT)
        cv2.putText(img, lbl, (tx - tw // 2, ty + th // 2),
                    _FONT, fs, C_DIM, _FT, cv2.LINE_AA)

    # 30° tick marks
    for deg in range(0, 360, 30):
        if deg % 90 == 0:
            continue
        rad = np.radians(deg)
        r1  = radius - 1
        r2  = radius - 4
        cv2.line(img,
                 (cx + int(r1 * np.sin(rad)), cy - int(r1 * np.cos(rad))),
                 (cx + int(r2 * np.sin(rad)), cy - int(r2 * np.cos(rad))),
                 C_DIM, 1, cv2.LINE_AA)

    if np.isnan(heading_deg):
        cv2.putText(img, '?', (cx - 4, cy + 5), _FONT, 0.32, C_DIM, _FT, cv2.LINE_AA)
        return

    # Needle (filled wedge)
    needle = radius - 5
    rad    = np.radians(float(heading_deg))
    tip_x  = cx + int(needle * np.sin(rad))
    tip_y  = cy - int(needle * np.cos(rad))
    tail_x = cx - int((needle // 2) * np.sin(rad))
    tail_y = cy + int((needle // 2) * np.cos(rad))
    cv2.line(img, (tail_x, tail_y), (tip_x, tip_y), C_ACCENT, 2, cv2.LINE_AA)
    cv2.circle(img, (tip_x, tip_y), 2, C_ACCENT, -1, cv2.LINE_AA)
    cv2.circle(img, (cx, cy), 2, C_DIM, -1, cv2.LINE_AA)

    # Heading label below circle
    hdg_lbl = f'{int(heading_deg) % 360:03d}°'
    fs = 0.26
    (tw, _), _ = cv2.getTextSize(hdg_lbl, _FONT, fs, _FT)
    cv2.putText(img, hdg_lbl, (cx - tw // 2, cy + radius + 10),
                _FONT, fs, C_ACCENT, _FT, cv2.LINE_AA)


# ── Heading tape ──────────────────────────────────────────────────────────── #

def heading_tape(img: np.ndarray,
                 x: int, y: int, w: int, h: int,
                 yaw_deg: float,
                 show_readout: bool = False) -> None:
    """Horizontal compass tape showing ±60° around current heading."""
    cv2.rectangle(img, (x, y), (x + w, y + h), C_BG, -1)
    cv2.rectangle(img, (x, y), (x + w, y + h), C_BORDER, 1, cv2.LINE_AA)

    if np.isnan(yaw_deg):
        msg = 'NO HDG'
        (tw, _), _ = cv2.getTextSize(msg, _FONT, 0.30, _FT)
        cv2.putText(img, msg, (x + (w - tw) // 2, y + h // 2 + 5),
                    _FONT, 0.30, C_DIM, _FT, cv2.LINE_AA)
        return

    yaw = float(yaw_deg) % 360.0
    cx  = x + w // 2
    ppd = w / 120.0   # pixels per degree

    for delta in range(-65, 66, 10):
        deg_at = int(yaw + delta) % 360
        px     = cx + int(delta * ppd)
        if px < x + 2 or px > x + w - 2:
            continue
        is_label = delta % 30 == 0
        tick_h   = 8 if is_label else 4
        cv2.line(img, (px, y + h - tick_h), (px, y + h - 1), C_DIM, 1)
        if is_label:
            lbl = _cardinal(deg_at)
            (tw, _), _ = cv2.getTextSize(lbl, _FONT, 0.30, _FT)
            cv2.putText(img, lbl, (px - tw // 2, y + h - tick_h - 2),
                        _FONT, 0.30, C_TEXT, _FT, cv2.LINE_AA)

    # Centre heading marker
    cv2.line(img, (cx, y + 2), (cx, y + h - 2), C_ACCENT, 2, cv2.LINE_AA)

    if show_readout:
        lbl = f'{int(yaw) % 360:03d}'
        _FONT_FS = 0.40
        (tw, th), _ = cv2.getTextSize(lbl, _FONT, _FONT_FS, _FT)
        cv2.putText(img, lbl, (cx - tw // 2, y - 3),
                    _FONT, _FONT_FS, C_ACCENT, _FT, cv2.LINE_AA)


# ── Video progress bar ────────────────────────────────────────────────────── #

def video_progress(img: np.ndarray,
                   x: int, y: int, w: int, h: int,
                   cur_frame: int, total_frames: int,
                   fps: float = 30.0,
                   paused: bool = False) -> None:
    """Playback progress bar with timestamp and pause indicator."""
    cv2.rectangle(img, (x, y), (x + w, y + h), C_BG, -1)
    cv2.rectangle(img, (x, y), (x + w, y + h), C_BORDER, 1, cv2.LINE_AA)

    total  = max(total_frames, 1)
    fps    = max(fps, 1.0)
    frac   = float(np.clip(cur_frame / total, 0.0, 1.0))
    fill_w = int(frac * (w - 2))

    bar_color = C_AMBER if paused else C_ACCENT
    if fill_w > 0:
        cv2.rectangle(img, (x + 1, y + 1), (x + 1 + fill_w, y + h - 1), bar_color, -1)

    def _fmt(frames: int) -> str:
        secs  = int(frames / fps)
        m, s  = divmod(secs, 60)
        return f'{m:02d}:{s:02d}'

    state  = '⏸' if paused else '▶'
    lbl    = f'{state} {_fmt(cur_frame)} / {_fmt(total_frames)}  {int(frac * 100)}%'
    fs     = 0.26
    (tw, _), _ = cv2.getTextSize(lbl, _FONT, fs, _FT)
    lx = x + (w - tw) // 2
    cv2.putText(img, lbl, (max(x + 2, lx), y + h - 2),
                _FONT, fs, C_TEXT, _FT, cv2.LINE_AA)


# ── Pipeline health row ───────────────────────────────────────────────────── #

def health_row(img: np.ndarray,
               x: int, y: int, w: int, h: int,
               health: Dict[str, bool]) -> None:
    """One-line pipeline health strip: [OK] camera  [OK] detector  etc."""
    cv2.rectangle(img, (x, y), (x + w, y + h), C_BG, -1)
    cv2.line(img, (x, y), (x + w, y), C_BORDER, 1)

    nodes  = [('camera', 'CAM'), ('detector', 'DET'), ('tracker', 'TRK'), ('state', 'STATE')]
    col_w  = w // len(nodes)
    fs     = 0.26

    for i, (key, short) in enumerate(nodes):
        ok    = health.get(key, False)
        badge = '[OK]' if ok else '[--]'
        color = C_OK   if ok else C_DIM
        bx    = x + i * col_w + 4
        by    = y + h - 3
        cv2.putText(img, badge, (bx, by), _FONT, fs, color, _FT, cv2.LINE_AA)
        (bw, _), _ = cv2.getTextSize(badge, _FONT, fs, _FT)
        cv2.putText(img, short, (bx + bw + 3, by), _FONT, fs, C_DIM, _FT, cv2.LINE_AA)


# ── Internal helpers ──────────────────────────────────────────────────────── #

def _cardinal(deg: int) -> str:
    return {0: 'N', 90: 'E', 180: 'S', 270: 'W'}.get(deg % 360, f'{deg % 360:03d}')

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

All widget functions accept an optional `fs_scale` keyword argument (default 1.0)
that multiplies every internal font size, enabling resolution-adaptive rendering.
Pass `fs_scale = max(1.0, frame_width / 640.0)` from the strip renderer.

Text is rendered via Pillow (TrueType) for crisp sub-pixel output at all scales.
Font priority: IosevkaNerdFontMono → NotoSansMono → DejaVuSansMono → PIL default.
"""

from __future__ import annotations

import os as _os
from typing import Dict, Sequence

import cv2
import numpy as np
from PIL import Image as _PILImage, ImageDraw as _PILDraw, ImageFont as _PILFont

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


# ── TrueType text engine ──────────────────────────────────────────────────── #

_PTF: dict = {}  # int → FreeTypeFont (or default ImageFont)
_PTF_PATH: str | None = None

_MONO_CANDIDATES = (
    '/usr/local/share/fonts/TTF/IosevkaNerdFontMono-Regular.ttf',
    '/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf',
    '/usr/local/share/fonts/noto/NotoSansMono-Regular.ttf',
    '/usr/local/share/fonts/TTF/DejaVuSansMono.ttf',
    '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf',
)

_CV2FS_TO_PX = 34  # cv2 SIMPLEX scale=1.0 → ~34px cap height


def _pil_font(px: int) -> _PILFont.FreeTypeFont:
    global _PTF_PATH
    if _PTF_PATH is None:
        for c in _MONO_CANDIDATES:
            if _os.path.exists(c):
                _PTF_PATH = c
                break
        else:
            _PTF_PATH = ''
    if px not in _PTF:
        try:
            _PTF[px] = (_PILFont.truetype(_PTF_PATH, px) if _PTF_PATH
                        else _PILFont.load_default())
        except Exception:
            _PTF[px] = _PILFont.load_default()
    return _PTF[px]


def _blit_rgba(img: np.ndarray, surf: np.ndarray, x: int, y: int) -> None:
    """Alpha-composite RGBA surface patch onto BGR image at (x, y) top-left."""
    ih, iw = img.shape[:2]
    sh, sw = surf.shape[:2]
    sx1 = max(0, -x);   dx1 = max(0, x)
    sy1 = max(0, -y);   dy1 = max(0, y)
    sx2 = min(sw, iw - x);  dx2 = min(iw, x + sw)
    sy2 = min(sh, ih - y);  dy2 = min(ih, y + sh)
    if sx2 <= sx1 or sy2 <= sy1 or dx2 <= dx1 or dy2 <= dy1:
        return
    patch = surf[sy1:sy2, sx1:sx2]
    alpha = patch[:, :, 3:4].astype(np.float32) / 255.0
    fg    = patch[:, :, :3][:, :, ::-1].astype(np.float32)  # RGBA → BGR
    bg    = img[dy1:dy2, dx1:dx2].astype(np.float32)
    img[dy1:dy2, dx1:dx2] = np.clip(fg * alpha + bg * (1.0 - alpha), 0, 255).astype(np.uint8)


def pil_text(img: np.ndarray, text: str, org: tuple,
             cv2_fs: float, color_bgr: tuple) -> None:
    """Render TrueType text onto BGR array.

    org is (x, y_bottom) — same bottom-left baseline convention as cv2.putText.
    """
    if not text:
        return
    px   = max(8, round(cv2_fs * _CV2FS_TO_PX))
    font = _pil_font(px)
    r, g, b = int(color_bgr[2]), int(color_bgr[1]), int(color_bgr[0])
    x, y_base = int(org[0]), int(org[1])
    try:
        bbox = font.getbbox(text)   # (left, top, right, bottom) at draw origin
    except Exception:
        return
    if bbox[2] <= bbox[0]:
        return
    tw = int(bbox[2] - bbox[0])
    th = int(bbox[3] - bbox[1])
    surf = _PILImage.new('RGBA', (tw + 4, th + 4), (0, 0, 0, 0))
    _PILDraw.Draw(surf).text((2 - bbox[0], 2 - bbox[1]),
                              text, font=font, fill=(r, g, b, 255))
    # y_base is bottom of text → top of surface sits at y_base - th
    _blit_rgba(img, np.array(surf), x, int(y_base - th - 2))


def pil_text_size(text: str, cv2_fs: float) -> tuple[int, int]:
    """Return (width, height) of text rendered at given cv2-style scale."""
    if not text:
        return (0, 0)
    px   = max(8, round(cv2_fs * _CV2FS_TO_PX))
    font = _pil_font(px)
    try:
        bbox = font.getbbox(text)
        return (max(0, int(bbox[2] - bbox[0])), max(0, int(bbox[3] - bbox[1])))
    except Exception:
        return (0, 0)


# ── Needle gauge ─────────────────────────────────────────────────────────── #

def needle_gauge(img: np.ndarray,
                 x: int, y: int, w: int, h: int,
                 value: float,
                 vmin: float = -1.0, vmax: float = 1.0,
                 label: str = '',
                 deadband: float = 0.05,
                 *,
                 fs_scale: float = 1.0) -> None:
    """Horizontal needle meter showing signed error."""
    if w < 10 or h < 4:
        return
    value = float(np.clip(value, vmin, vmax))
    span  = max(vmax - vmin, 1e-6)

    cv2.rectangle(img, (x, y), (x + w, y + h), C_BG, -1)
    cv2.rectangle(img, (x, y), (x + w, y + h), C_BORDER, 1, cv2.LINE_AA)

    cx      = x + w // 2
    db_frac = deadband / (span / 2.0)
    db_px   = max(1, int(db_frac * w / 2))
    cv2.rectangle(img, (cx - db_px, y + 1), (cx + db_px, y + h - 1), C_PANEL, -1)
    cv2.line(img, (cx, y + 1), (cx, y + h - 1), C_DIM, 1)

    norm = (value - vmin) / span
    nx   = max(x + 2, min(x + w - 2, x + int(norm * w)))

    abs_n = abs(value) / max(abs(vmin), abs(vmax), 1e-6)
    if abs_n < deadband / (span / 2):
        color = C_OK
    elif abs_n < 0.5:
        color = C_AMBER
    else:
        color = C_ERR

    if nx > cx:
        _fill_rect(img, cx, y + 2, nx, y + h - 2, color, alpha=0.50)
    elif nx < cx:
        _fill_rect(img, nx, y + 2, cx, y + h - 2, color, alpha=0.50)

    cv2.line(img, (nx, y), (nx, y + h), C_TEXT, 2, cv2.LINE_AA)

    fs = 0.30 * fs_scale
    if label:
        pil_text(img, label, (x, y - 2), fs, C_DIM)

    val_str = f'{value:+.2f}'
    vw, _   = pil_text_size(val_str, fs)
    pil_text(img, val_str, (x + w - vw, y - 2), fs, color)


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
                   conf: float,
                   *,
                   fs_scale: float = 1.0) -> None:
    """Horizontal fill bar 0-1 with colour-coded confidence level."""
    if w < 6 or h < 4:
        return
    conf  = float(np.clip(conf, 0.0, 1.0))
    color = C_OK if conf > 0.70 else (C_AMBER if conf > 0.40 else C_ERR)
    fill  = int(conf * (w - 2))

    cv2.rectangle(img, (x, y), (x + w, y + h), C_BG, -1)
    cv2.rectangle(img, (x, y), (x + w, y + h), C_BORDER, 1, cv2.LINE_AA)
    if fill > 0:
        _fill_rect(img, x + 1, y + 1, x + 1 + fill, y + h - 1, color, alpha=0.55)

    val_str = f'{int(conf * 100)}%'
    fs      = 0.28 * fs_scale
    tw, _   = pil_text_size(val_str, fs)
    lx      = x + w - tw - 2
    pil_text(img, val_str, (max(x + 2, lx), y + h - 2), fs, C_TEXT)


# ── Altimeter depth ───────────────────────────────────────────────────────── #

def altimeter_depth(img: np.ndarray,
                    x: int, y: int, w: int, h: int,
                    depth_m: float,
                    max_depth: float = 8.0,
                    *,
                    fs_scale: float = 1.0) -> None:
    """Vertical tape depth gauge — 0m at top, max_depth at bottom."""
    if w < 12 or h < 20:
        return

    cv2.rectangle(img, (x, y), (x + w, y + h), C_BG, -1)
    cv2.rectangle(img, (x, y), (x + w, y + h), C_BORDER, 1, cv2.LINE_AA)

    fs_small = 0.24 * fs_scale
    pil_text(img, 'D', (x + 2, y + 10), fs_small, C_DIM)

    if np.isnan(depth_m):
        pil_text(img, '?', (x + w // 2 - 4, y + h // 2 + 4), 0.34 * fs_scale, C_DIM)
        return

    depth_abs = float(min(abs(depth_m), max_depth))

    def _depth_color(d: float) -> tuple:
        return (C_OK    if d < 2.0
                else C_ACCENT if d < 4.5
                else C_AMBER  if d < 6.5
                else C_ERR)

    # 1. Transparent fill first so tick marks remain visible
    fill_h = int(depth_abs / max_depth * h)
    if fill_h > 0:
        _fill_rect(img, x + 1, y + 1, x + w - 1, y + 1 + min(fill_h, h - 2),
                   _depth_color(depth_abs), alpha=0.38)

    # 2. Tick marks + labels drawn on top of fill
    step = 1.0 if max_depth > 6 else 0.5
    m    = 0.0
    while m <= max_depth + 0.01:
        yt       = y + int(m / max_depth * h)
        is_major = abs(round(m) - m) < 0.01
        tick_len = 8 if is_major else 4
        cv2.line(img, (x, yt), (x + tick_len, yt), C_DIM, 1)
        if is_major and int(m) > 0:
            pil_text(img, f'{int(m)}m', (x + 10, min(yt + 4, y + h - 1)),
                     fs_small, C_DIM)
        m += step

    # 3. Bright indicator line + readout
    ind_y = max(y + 1, min(y + h - 1, y + int(depth_abs / max_depth * h)))
    d_col = _depth_color(depth_abs)
    cv2.line(img, (x, ind_y), (x + w, ind_y), d_col, 2, cv2.LINE_AA)

    lbl    = f'{depth_abs:.1f}m'
    fs_lbl = 0.34 * fs_scale
    lw, _  = pil_text_size(lbl, fs_lbl)
    lx     = x - lw - 3
    if lx < 0:
        lx = x + w + 2
    pil_text(img, lbl, (max(0, lx), ind_y + 4), fs_lbl, d_col)


# ── Battery bar ───────────────────────────────────────────────────────────── #

def battery_bar(img: np.ndarray,
                x: int, y: int, w: int, h: int,
                voltage: float,
                cell_count: int = 4,
                *,
                fs_scale: float = 1.0) -> None:
    """Segmented battery indicator with voltage text."""
    v_full  = cell_count * 4.2
    v_empty = cell_count * 3.5
    pct     = float(np.clip((voltage - v_empty) / max(v_full - v_empty, 0.1), 0.0, 1.0))
    color   = C_OK if pct > 0.60 else (C_AMBER if pct > 0.20 else C_ERR)

    n_segs = 10
    seg_w  = (w - 2) // n_segs
    filled = int(pct * n_segs)

    cv2.rectangle(img, (x, y), (x + w, y + h), C_BORDER, 1, cv2.LINE_AA)
    for i in range(n_segs):
        sx1 = x + 1 + i * seg_w
        sx2 = sx1 + seg_w - 1
        if i < filled:
            _fill_rect(img, sx1, y + 1, sx2, y + h - 1, color, alpha=0.60)
        else:
            cv2.rectangle(img, (sx1, y + 1), (sx2, y + h - 1), C_BG, -1)

    lbl  = f'{voltage:.1f}V  {int(pct * 100)}%'
    fs   = 0.28 * fs_scale
    tw, _ = pil_text_size(lbl, fs)
    lx   = x + (w - tw) // 2
    pil_text(img, lbl, (max(x + 2, lx), y + h - 2), fs, C_TEXT)


# ── Mini compass ──────────────────────────────────────────────────────────── #

def mini_compass(img: np.ndarray,
                 cx: int, cy: int,
                 radius: int,
                 heading_deg: float,
                 *,
                 fs_scale: float = 1.0) -> None:
    """Compass rose with needle.  N at top, tick marks every 30°."""
    overlay = img.copy()
    cv2.circle(overlay, (cx, cy), radius, C_BG, -1)
    cv2.addWeighted(overlay, 0.80, img, 0.20, 0, dst=img)
    cv2.circle(img, (cx, cy), radius, C_BORDER, 1, cv2.LINE_AA)

    fs_card = 0.25 * fs_scale
    for ang, lbl in ((0, 'N'), (90, 'E'), (180, 'S'), (270, 'W')):
        rad   = np.radians(ang)
        tx    = cx + int((radius - 6) * np.sin(rad))
        ty    = cy - int((radius - 6) * np.cos(rad))
        tw, th = pil_text_size(lbl, fs_card)
        color = C_TEXT if lbl == 'N' else C_DIM
        pil_text(img, lbl, (tx - tw // 2, ty + th // 2), fs_card, color)

    for deg in range(0, 360, 30):
        if deg % 90 == 0:
            continue
        rad = np.radians(deg)
        r1  = radius - 1
        r2  = radius - 5
        cv2.line(img,
                 (cx + int(r1 * np.sin(rad)), cy - int(r1 * np.cos(rad))),
                 (cx + int(r2 * np.sin(rad)), cy - int(r2 * np.cos(rad))),
                 C_DIM, 1, cv2.LINE_AA)

    if np.isnan(heading_deg):
        pil_text(img, '?', (cx - 4, cy + 5), 0.34 * fs_scale, C_DIM)
        return

    needle = radius - 5
    rad    = np.radians(float(heading_deg))
    tip_x  = cx + int(needle * np.sin(rad))
    tip_y  = cy - int(needle * np.cos(rad))
    tail_x = cx - int((needle // 2) * np.sin(rad))
    tail_y = cy + int((needle // 2) * np.cos(rad))
    cv2.line(img, (tail_x, tail_y), (tip_x, tip_y), C_ACCENT, 2, cv2.LINE_AA)
    cv2.circle(img, (tip_x, tip_y), 2, C_ACCENT, -1, cv2.LINE_AA)
    cv2.circle(img, (cx, cy), 2, C_DIM, -1, cv2.LINE_AA)

    hdg_lbl = f'{int(heading_deg) % 360:03d}'
    fs_hdg  = 0.28 * fs_scale
    tw, _   = pil_text_size(hdg_lbl, fs_hdg)
    pil_text(img, hdg_lbl, (cx - tw // 2, cy + radius + 9), fs_hdg, C_ACCENT)


# ── Heading tape ──────────────────────────────────────────────────────────── #

def heading_tape(img: np.ndarray,
                 x: int, y: int, w: int, h: int,
                 yaw_deg: float,
                 show_readout: bool = False,
                 *,
                 fs_scale: float = 1.0) -> None:
    """Horizontal compass tape showing ±60° around current heading."""
    cv2.rectangle(img, (x, y), (x + w, y + h), C_BG, -1)
    cv2.rectangle(img, (x, y), (x + w, y + h), C_BORDER, 1, cv2.LINE_AA)

    if np.isnan(yaw_deg):
        fs_nan = 0.32 * fs_scale
        msg    = 'NO HDG'
        tw, _  = pil_text_size(msg, fs_nan)
        pil_text(img, msg, (x + (w - tw) // 2, y + h // 2 + 5), fs_nan, C_DIM)
        return

    yaw = float(yaw_deg) % 360.0
    cx  = x + w // 2
    ppd = w / 120.0

    fs_lbl = 0.32 * fs_scale
    for delta in range(-65, 66, 10):
        deg_at = int(yaw + delta) % 360
        px     = cx + int(delta * ppd)
        if px < x + 2 or px > x + w - 2:
            continue
        is_label = delta % 30 == 0
        tick_h   = 9 if is_label else 4
        cv2.line(img, (px, y + h - tick_h), (px, y + h - 1), C_DIM, 1)
        if is_label:
            lbl    = _cardinal(deg_at)
            tw, _  = pil_text_size(lbl, fs_lbl)
            pil_text(img, lbl, (px - tw // 2, y + h - tick_h - 2), fs_lbl, C_TEXT)

    cv2.line(img, (cx, y + 2), (cx, y + h - 2), C_ACCENT, 2, cv2.LINE_AA)

    if show_readout:
        lbl     = f'{int(yaw) % 360:03d}'
        fs_read = 0.42 * fs_scale
        tw, _   = pil_text_size(lbl, fs_read)
        pil_text(img, lbl, (cx - tw // 2, y - 3), fs_read, C_ACCENT)


# ── Video progress bar ────────────────────────────────────────────────────── #

def video_progress(img: np.ndarray,
                   x: int, y: int, w: int, h: int,
                   cur_frame: int, total_frames: int,
                   fps: float = 30.0,
                   paused: bool = False,
                   *,
                   fs_scale: float = 1.0) -> None:
    """Playback progress bar with timestamp and pause indicator."""
    cv2.rectangle(img, (x, y), (x + w, y + h), C_BG, -1)
    cv2.rectangle(img, (x, y), (x + w, y + h), C_BORDER, 1, cv2.LINE_AA)

    total  = max(total_frames, 1)
    fps    = max(fps, 1.0)
    frac   = float(np.clip(cur_frame / total, 0.0, 1.0))
    fill_w = int(frac * (w - 2))

    bar_color = C_AMBER if paused else C_ACCENT
    if fill_w > 0:
        _fill_rect(img, x + 1, y + 1, x + 1 + fill_w, y + h - 1, bar_color, alpha=0.55)

    def _fmt(frames: int) -> str:
        secs = int(frames / fps)
        m, s = divmod(secs, 60)
        return f'{m:02d}:{s:02d}'

    state = '||' if paused else '>'
    lbl   = f'{state} {_fmt(cur_frame)} / {_fmt(total_frames)}  {int(frac * 100)}%'
    fs    = 0.28 * fs_scale
    tw, _ = pil_text_size(lbl, fs)
    lx    = x + (w - tw) // 2
    pil_text(img, lbl, (max(x + 2, lx), y + h - 2), fs, C_TEXT)


# ── Pipeline health row ───────────────────────────────────────────────────── #

def health_row(img: np.ndarray,
               x: int, y: int, w: int, h: int,
               health: Dict[str, bool],
               *,
               fs_scale: float = 1.0) -> None:
    """One-line pipeline health strip: [OK] camera  [OK] detector  etc."""
    cv2.rectangle(img, (x, y), (x + w, y + h), C_BG, -1)
    cv2.line(img, (x, y), (x + w, y), C_BORDER, 1)

    nodes = [('camera', 'CAM'), ('detector', 'DET'), ('tracker', 'TRK'), ('state', 'STATE')]
    col_w = w // len(nodes)
    fs    = 0.28 * fs_scale

    for i, (key, short) in enumerate(nodes):
        ok    = health.get(key, False)
        badge = '[OK]' if ok else '[--]'
        color = C_OK   if ok else C_DIM
        bx    = x + i * col_w + 4
        by    = y + h - 3
        pil_text(img, badge, (bx, by), fs, color)
        bw, _ = pil_text_size(badge, fs)
        pil_text(img, short, (bx + bw + 3, by), fs, C_DIM)


# ── Internal helpers ──────────────────────────────────────────────────────── #

def _fill_rect(img: np.ndarray, x1: int, y1: int, x2: int, y2: int,
               color: tuple, alpha: float = 0.45) -> None:
    """Semi-transparent filled rectangle — content underneath stays visible."""
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(img.shape[1], x2), min(img.shape[0], y2)
    if x2 <= x1 or y2 <= y1:
        return
    roi     = img[y1:y2, x1:x2]
    overlay = roi.copy()
    overlay[:] = color
    cv2.addWeighted(overlay, alpha, roi, 1.0 - alpha, 0, roi)


def _cardinal(deg: int) -> str:
    return {0: 'N', 90: 'E', 180: 'S', 270: 'W'}.get(deg % 360, f'{deg % 360:03d}')

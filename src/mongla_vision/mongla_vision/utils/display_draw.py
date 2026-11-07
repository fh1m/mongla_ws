"""What the vision display draws besides the frame: the splash, the
camera-switch card, and the mission and distance panels."""

from __future__ import annotations

import cv2
import numpy as np

from mongla_vision.draw_widgets import pil_text, pil_text_size


# ---------------------------------------------------------------------------
# Splash screen
# ---------------------------------------------------------------------------

# Blue theme: dark navy background, sky-blue accent (OpenCV BGR order)
# RGB equivalents: BG=(5,15,55) navy  ACCENT=(0,160,255) sky-blue
_C_SP_BG     = (55, 15, 5)       # dark navy  — B=55 G=15 R=5
_C_SP_ACCENT = (255, 160, 0)     # sky blue   — B=255 G=160 R=0
_C_SP_TEXT   = (240, 235, 220)   # bright warm white
_C_SP_DIM    = (130, 95, 65)     # muted slate


def _draw_mission_panel(out, *, camera, fps, primary, native_w, native_h,
                        deadband, state) -> None:
    """Compact top-left mission-status panel: what the AUV sees + how well it's on it.

    Answers the operator's questions at a glance: which camera frame is live
    (FORWARD/DOWNWARD, colour-coded), the target class + confidence, the signed
    alignment offset in px + IN-BAND/OFF, live FPS, and depth/armed. On the
    downward frame it also prints the rotated axis legend (Y->surge, X->lat) so the
    offset numbers are unambiguous. Cheap: one ROI blend + a few text lines --
    drawn in the display node (off the detector/GPU path), no FPS cost."""
    h, w = out.shape[:2]
    sf = max(0.9, w / 1920.0)
    downward = camera == 'downward'
    badge = 'DOWNWARD' if downward else 'FORWARD'
    badge_col = (255, 210, 40) if downward else (60, 230, 120)   # cyan-ish / green (BGR)

    # Build the info lines.
    if primary is not None:
        dx = primary.cx - native_w / 2.0
        dy = primary.cy - native_h / 2.0
        ex = dx / max(native_w / 2.0, 1.0)
        ey = dy / max(native_h / 2.0, 1.0)
        in_band = abs(ex) < deadband and abs(ey) < deadband
        tgt_line = f'{primary.class_name}  {int(primary.score * 100)}%'
        off_line = f'dx {dx:+.0f}  dy {dy:+.0f} px'
        status, status_col = (('ON TARGET', (60, 230, 120)) if in_band
                              else ('OFF  ', (40, 190, 255)))
    else:
        tgt_line = 'no target'
        off_line = 'searching...'
        status, status_col = 'LOST', (60, 60, 235)

    lines = [
        (tgt_line, (235, 235, 235)),
        (off_line, (200, 200, 200)),
        (status,   status_col),
        (f'FPS {fps:4.1f}', (200, 200, 200)),
    ]
    if downward:
        lines.append(('axes: Y->surge  X->lat', (180, 180, 120)))
    if state is not None:
        arm = 'ARMED' if getattr(state, 'armed', False) else 'safe'
        lines.append((f'{state.depth_m:+.2f} m   {arm}',
                      (60, 60, 235) if getattr(state, 'armed', False) else (170, 170, 170)))

    fs_badge = 0.95 * sf
    fs_line  = 0.62 * sf
    pad      = int(14 * sf)
    line_h   = int(pil_text_size('Ag', fs_line)[1] * 1.5)
    bw_badge = pil_text_size(badge, fs_badge)[0]
    box_w = max(bw_badge, max(pil_text_size(t, fs_line)[0] for t, _ in lines)) + pad * 2
    box_h = pad * 2 + int(pil_text_size(badge, fs_badge)[1] * 1.6) + line_h * len(lines)
    box_w = min(box_w, w - 4)

    # Semi-transparent dark panel background (single ROI blend).
    x0, y0 = 4, 4
    x1, y1 = min(x0 + box_w, w - 1), min(y0 + box_h, h - 1)
    roi = out[y0:y1, x0:x1]
    if roi.size:
        dark = np.full_like(roi, 15)
        cv2.addWeighted(dark, 0.62, roi, 0.38, 0, roi)
        cv2.rectangle(out, (x0, y0), (x1, y1), badge_col, max(1, int(2 * sf)), cv2.LINE_AA)

    yy = y0 + pad + int(pil_text_size(badge, fs_badge)[1])
    pil_text(out, badge, (x0 + pad, yy), fs_badge, badge_col)
    yy += int(pil_text_size(badge, fs_badge)[1] * 0.6)
    for text, col in lines:
        yy += line_h
        pil_text(out, text, (x0 + pad, yy), fs_line, col)


def _render_switching(w: int, h: int, camera: str) -> np.ndarray:
    """Dark 'switching camera' screen shown between a switch and the first frame
    of the new camera -- so the HUD never silently shows the frozen OLD frame."""
    img = np.full((h, w, 3), (35, 20, 8), dtype=np.uint8)   # dark navy (BGR)
    cx, cy = w // 2, h // 2
    sf = max(0.6, w / 1280.0)
    title = f'CAMERA -> {str(camera).upper()}'
    tw, th = pil_text_size(title, 0.9 * sf)
    pil_text(img, title, (cx - tw // 2, cy - th), 0.9 * sf, (255, 200, 60))
    sub = 'waiting for stream...'
    sw, _ = pil_text_size(sub, 0.5 * sf)
    pil_text(img, sub, (cx - sw // 2, cy + int(30 * sf)), 0.5 * sf, (170, 170, 170))
    return img


# How long after a switch to keep showing the "waiting for stream" screen while
# the new camera has produced no frame (past this it's a real stall, not a switch).
_SWITCH_WAIT_S = 3.0


def _draw_distance_panel(out, dbg) -> None:
    """Top-right ODOMETRY panel while the distance estimator is ACTIVE.

    `dbg` = [dist_m, height_m, n_tracks, active] from distance_debug, or None.
    Drawn only when active>0.5 so it never clutters normal vision missions.
    """
    if not dbg or len(dbg) < 4 or dbg[3] < 0.5:
        return
    dist, height, tracks = dbg[0], dbg[1], int(dbg[2])
    h, w = out.shape[:2]
    sf = max(0.9, w / 1920.0)
    lines = [
        ('ODOMETRY', (40, 210, 255)),
        (f'dist  {dist:+.3f} m', (235, 235, 235)),
        (f'height {height:.2f} m', (200, 200, 200)),
        (f'tracks {tracks}', (60, 230, 120) if tracks >= 6 else (60, 60, 235)),
    ]
    fs = 0.62 * sf
    line_h = int(pil_text_size('Ag', fs)[1] * 1.5)
    pad = int(12 * sf)
    box_w = max(pil_text_size(t, fs)[0] for t, _ in lines) + pad * 2
    box_h = pad * 2 + line_h * len(lines)
    x1, y0 = w - 4, 4
    x0, y1 = max(0, x1 - box_w), min(h - 1, y0 + box_h)
    roi = out[y0:y1, x0:x1]
    if roi.size:
        dark = np.full_like(roi, 15)
        cv2.addWeighted(dark, 0.62, roi, 0.38, 0, roi)
        cv2.rectangle(out, (x0, y0), (x1, y1), (40, 210, 255), max(1, int(2 * sf)), cv2.LINE_AA)
    yy = y0 + pad
    for text, col in lines:
        yy += line_h
        pil_text(out, text, (x0 + pad, yy), fs, col)


def _render_splash(w: int, h: int, elapsed: float, camera: str,
                   fade: float = 1.0) -> np.ndarray:
    """Blue 'Initializing Vision System' splash. fade=1.0 fully opaque, 0.0 transparent."""
    img = np.full((h, w, 3), _C_SP_BG, dtype=np.uint8)
    cx, cy = w // 2, h // 2
    sf  = max(0.5, w / 1280.0)

    # Brand header
    brand = '───  MONGLA  ───'
    fs_b  = 0.85 * sf
    bw, bh = pil_text_size(brand, fs_b)
    bx = cx - bw // 2
    pil_text(img, brand, (bx, cy - 58), fs_b, _C_SP_ACCENT)

    # Accent separator line under brand
    cv2.line(img, (bx, cy - 58 + bh + 6), (bx + bw, cy - 58 + bh + 6), _C_SP_ACCENT, 1)

    # Main status
    status = 'INITIALIZING VISION SYSTEM'
    fs_s   = 0.65 * sf
    sw, _ = pil_text_size(status, fs_s)
    pil_text(img, status, (cx - sw // 2, cy + 22), fs_s, _C_SP_TEXT)

    # Sub-status
    sub    = 'Loading model  ·  video will play automatically when ready'
    fs_sub = 0.38 * sf
    subw, _ = pil_text_size(sub, fs_sub)
    pil_text(img, sub, (cx - subw // 2, cy + 58), fs_sub, _C_SP_DIM)

    # Animated ping-pong progress bar
    bar_len = int(w * 0.48)
    bar_x0  = cx - bar_len // 2
    bar_y0  = cy + 86
    cv2.rectangle(img, (bar_x0, bar_y0), (bar_x0 + bar_len, bar_y0 + 4), (90, 35, 12), -1)
    t    = (elapsed % 2.0) / 2.0
    prog = t * 2 if t < 0.5 else (1.0 - t) * 2
    fill = max(bar_len // 8, int(bar_len * prog))
    cv2.rectangle(img, (bar_x0, bar_y0), (bar_x0 + fill, bar_y0 + 4), _C_SP_ACCENT, -1)

    # Footer
    fs_info = 0.33 * sf
    cam_lbl = f'camera: {camera}'
    pil_text(img, cam_lbl, (16, h - 20), fs_info, _C_SP_DIM)
    ts = f'{int(elapsed)}s'
    tw, _ = pil_text_size(ts, fs_info)
    pil_text(img, ts, (w - tw - 16, h - 20), fs_info, _C_SP_DIM)

    # Apply fade: blend toward black at fade < 1
    if fade < 0.999:
        img = (img * max(0.0, fade)).astype(np.uint8)
    return img

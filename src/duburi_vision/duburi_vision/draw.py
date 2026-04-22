"""draw -- rich on-image visualizations for diagnostics.

Design rule: every glyph must answer a specific question an operator will
ask while staring at the debug image:

  * "Is the camera streaming?"          -> top-left status badge (FPS, source)
  * "Is the detector seeing anything?"  -> per-detection box + label
  * "Which target is the AUV chasing?"  -> primary target highlighted (thicker
                                            box + filled corners) + crosshair
  * "How well aligned is the AUV?"      -> dashed image-center reticle +
                                            target-to-center offset arrow +
                                            normalized error readout (-1..+1)
  * "How close is the target?"          -> bbox area badge (% of frame area)
  * "Is the model confident?"           -> per-box conf bar
  * "Anything stale / broken?"          -> red banner overlay

All functions are pure (frame in, frame out — modifies a copy). Annotators
are constructed lazily so importing draw never touches `supervision` if
the caller only wants `dashed_reticle` or `status_badge`.
"""

from __future__ import annotations

from typing import Iterable, List, Optional, Tuple

import cv2
import numpy as np

from .detection.detector import Detection, largest


# ----- Color palette (BGR). Kept small so the eye groups things quickly. -- #
COLOR_PRIMARY    = (50, 220, 255)    # amber: the target the AUV is chasing
COLOR_SECONDARY  = (200, 200, 200)   # light gray: other detections
COLOR_RETICLE    = (180, 180, 180)
COLOR_OFFSET     = (50, 220, 255)
COLOR_OK         = (90, 220, 120)    # green: nominal
COLOR_WARN       = (50, 200, 240)    # amber: degraded
COLOR_ERR        = (60, 60, 230)     # red: stale / no detection / failure
COLOR_BG         = (30, 30, 30)
COLOR_FG         = (240, 240, 240)


# --------------------------------------------------------------------------- #
#  Annotators (supervision boxes + labels)                                    #
# --------------------------------------------------------------------------- #

_BOX_ANNOTATOR = None
_LBL_ANNOTATOR = None


def _annotators():
    """Lazy singleton — builds supervision annotators on first use."""
    global _BOX_ANNOTATOR, _LBL_ANNOTATOR
    if _BOX_ANNOTATOR is None:
        import supervision as sv
        _BOX_ANNOTATOR = sv.BoxAnnotator(thickness=2)
        _LBL_ANNOTATOR = sv.LabelAnnotator(text_thickness=1, text_scale=0.5)
    return _BOX_ANNOTATOR, _LBL_ANNOTATOR


def _to_sv(detections: List[Detection]):
    import supervision as sv
    if not detections:
        return sv.Detections.empty()
    xyxy   = np.array([list(d.xyxy)       for d in detections], dtype=float)
    conf   = np.array([d.score            for d in detections], dtype=float)
    cls_id = np.array([d.class_id         for d in detections], dtype=int)
    return sv.Detections(xyxy=xyxy, confidence=conf, class_id=cls_id)


# --------------------------------------------------------------------------- #
#  Public draw API                                                            #
# --------------------------------------------------------------------------- #

def draw_detections(frame_bgr: np.ndarray, detections: List[Detection]) -> np.ndarray:
    """Draw boxes + labels for every detection (no primary highlight)."""
    if frame_bgr is None or not detections:
        return frame_bgr if frame_bgr is not None else None
    box, lbl = _annotators()
    sv_det = _to_sv(detections)
    out = box.annotate(scene=frame_bgr.copy(), detections=sv_det)
    labels = [f"{d.class_name} {int(d.score * 100)}%" for d in detections]
    out = lbl.annotate(scene=out, detections=sv_det, labels=labels)
    return out


def highlight_primary(frame_bgr: np.ndarray, target: Optional[Detection],
                      color=COLOR_PRIMARY) -> np.ndarray:
    """Thicker box + filled corners for the chosen primary target."""
    if frame_bgr is None or target is None:
        return frame_bgr
    out = frame_bgr.copy() if frame_bgr is not None else None
    x1, y1, x2, y2 = (int(v) for v in target.xyxy)
    cv2.rectangle(out, (x1, y1), (x2, y2), color, 3, cv2.LINE_AA)
    _draw_corners(out, x1, y1, x2, y2, color, length=18, thickness=4)
    return out


def crosshair(frame_bgr: np.ndarray, target: Optional[Detection],
              color=COLOR_PRIMARY, size=14) -> np.ndarray:
    """Crosshair on the target's center -- the visual-PID setpoint anchor."""
    if frame_bgr is None or target is None:
        return frame_bgr
    out = frame_bgr.copy()
    cx, cy = int(target.cx), int(target.cy)
    cv2.line(out, (cx - size, cy), (cx + size, cy), color, 2, cv2.LINE_AA)
    cv2.line(out, (cx, cy - size), (cx, cy + size), color, 2, cv2.LINE_AA)
    cv2.circle(out, (cx, cy), 3, color, -1, cv2.LINE_AA)
    return out


def dashed_reticle(frame_bgr: np.ndarray, color=COLOR_RETICLE) -> np.ndarray:
    """Dashed center cross + tiny scale ticks. The operator's reference frame."""
    if frame_bgr is None:
        return frame_bgr
    out = frame_bgr.copy()
    h, w = out.shape[:2]
    cx, cy = w // 2, h // 2
    _dashed_line(out, (cx, 0), (cx, h),  color, dash=8, gap=6, thickness=1)
    _dashed_line(out, (0, cy), (w, cy),  color, dash=8, gap=6, thickness=1)
    cv2.circle(out, (cx, cy), 4, color, 1, cv2.LINE_AA)
    cv2.circle(out, (cx, cy), 1, color, -1, cv2.LINE_AA)
    return out


def offset_arrow(frame_bgr: np.ndarray, target: Optional[Detection],
                 color=COLOR_OFFSET) -> np.ndarray:
    """Arrow from image center -> target center. Length and direction
    immediately tell you which way the AUV needs to rotate / strafe."""
    if frame_bgr is None or target is None:
        return frame_bgr
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
    """Top-left badge: source, fps, device, detections-this-frame, target."""
    if frame_bgr is None:
        return frame_bgr
    out = frame_bgr.copy()
    lines = [
        f"src: {source}      fps: {fps:5.1f}",
        f"dev: {device}      det: {n_detections}",
    ]
    if primary_class is not None:
        lines.append(f"target: {primary_class}")

    _panel(out, (10, 10), lines,
           fg=COLOR_FG, bg=(20, 20, 20),
           border=COLOR_OK if healthy else COLOR_ERR)
    return out


def alignment_readout(frame_bgr: np.ndarray, target: Optional[Detection],
                      *, deadband=0.05) -> np.ndarray:
    """Bottom-left panel: normalized bbox-error readout for visual PID.

    err_x in [-1, +1]: -1 = target on left edge,  +1 = target on right edge
    err_y in [-1, +1]: -1 = target on top edge,   +1 = target on bottom edge
    Within `deadband` we paint OK; outside, WARN.
    """
    if frame_bgr is None:
        return frame_bgr
    out = frame_bgr.copy()
    h, w = out.shape[:2]
    if target is None:
        _panel(out, (10, h - 60), ['no target', 'err=NA'],
               fg=COLOR_FG, bg=(20, 20, 20), border=COLOR_ERR)
        return out

    cx, cy = w / 2.0, h / 2.0
    ex = (target.cx - cx) / max(cx, 1.0)        # -1..+1
    ey = (target.cy - cy) / max(cy, 1.0)
    aligned = abs(ex) < deadband and abs(ey) < deadband
    border = COLOR_OK if aligned else COLOR_WARN
    lines = [
        f"err_x: {ex:+.2f}    err_y: {ey:+.2f}",
        f"area:  {(target.area / (w * h) * 100):5.2f}%   conf: {target.score:.2f}",
    ]
    _panel(out, (10, h - 60), lines, fg=COLOR_FG, bg=(20, 20, 20), border=border)
    return out


def stale_banner(frame_bgr: np.ndarray, message='STALE FRAME') -> np.ndarray:
    """Full-width red banner for catastrophic states: source unhealthy,
    detector dead, frame too old. Hard to miss."""
    if frame_bgr is None:
        return frame_bgr
    out = frame_bgr.copy()
    h, w = out.shape[:2]
    cv2.rectangle(out, (0, 0), (w, 28), COLOR_ERR, -1)
    cv2.putText(out, message, (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    return out


def render_all(frame_bgr: np.ndarray, detections: List[Detection], *,
               source='?', fps=0.0, device='?',
               healthy=True, show_reticle=True, show_alignment=True,
               deadband=0.05, primary: Optional[Detection] = None) -> np.ndarray:
    """One-shot: layered overlay used by detector_node.

    Layer order matters -- later layers paint on top of earlier ones:
      1. raw frame
      2. dashed reticle (operator's reference)
      3. all detections (light boxes + labels)
      4. primary target (thick box + corners)
      5. crosshair on primary
      6. offset arrow center -> primary
      7. alignment readout (bottom-left)
      8. status badge      (top-left)
      9. stale banner      (only if !healthy)
    """
    if frame_bgr is None:
        return None

    out = frame_bgr
    if show_reticle:
        out = dashed_reticle(out)

    out = draw_detections(out, detections)

    primary = primary or largest(detections)
    if primary is not None:
        out = highlight_primary(out, primary)
        out = crosshair(out, primary)
        out = offset_arrow(out, primary)

    if show_alignment:
        out = alignment_readout(out, primary, deadband=deadband)

    out = status_badge(
        out, source=source, fps=fps, device=device,
        n_detections=len(detections),
        primary_class=primary.class_name if primary else None,
        healthy=healthy)

    if not healthy:
        out = stale_banner(out)

    return out


def draw_track_ids(frame_bgr: np.ndarray, tracks) -> np.ndarray:
    """Overlay stable track IDs on tracked detections.

    Each track_id gets a consistent color (id % palette size), so the same
    target keeps the same color across frames even if other tracks come and go.
    predicted=True tracks are drawn with a dashed/dimmer style (half-alpha).

    `tracks` accepts any iterable of objects with .xyxy, .track_id, .class_name,
    .score, and .predicted attributes (TrackedDetection or compatible).
    """
    if frame_bgr is None or not tracks:
        return frame_bgr

    # 12-color BGR palette — perceptually distinct, distinguishable in dim pools
    _TRACK_PALETTE = [
        (255, 100,  50),   # blue
        ( 50, 220, 100),   # green
        ( 50, 100, 255),   # red
        (255, 200,  50),   # cyan
        (180,  50, 255),   # magenta
        ( 50, 255, 220),   # yellow
        (200, 130, 255),   # lavender
        ( 80, 255, 130),   # lime
        (255, 130, 200),   # pink
        (130, 200, 255),   # peach
        (255,  80, 130),   # violet
        (130, 255,  80),   # mint
    ]

    out = frame_bgr.copy()
    for td in tracks:
        color = _TRACK_PALETTE[abs(int(td.track_id)) % len(_TRACK_PALETTE)]
        x1, y1, x2, y2 = (int(v) for v in td.xyxy)

        thickness = 1 if td.predicted else 2
        alpha     = 0.45 if td.predicted else 1.0

        if alpha < 1.0:
            overlay = out.copy()
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, thickness, cv2.LINE_AA)
            cv2.addWeighted(overlay, alpha, out, 1.0 - alpha, 0, out)
        else:
            cv2.rectangle(out, (x1, y1), (x2, y2), color, thickness, cv2.LINE_AA)

        label = f"#{td.track_id} {td.class_name}"
        if td.predicted:
            label += " (pred)"
        lx, ly = x1, max(y1 - 5, 12)
        cv2.putText(out, label, (lx, ly),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

    return out


# --------------------------------------------------------------------------- #
#  Internal: cv2 plumbing (no third-party deps)                               #
# --------------------------------------------------------------------------- #

def _draw_corners(img, x1, y1, x2, y2, color, length=14, thickness=3):
    """Filled L-corners on each bbox corner -- distinctive against scenery."""
    for (px, py, dx, dy1, dx2, dy2) in (
        (x1, y1,  +1,  0,  0, +1),
        (x2, y1,  -1,  0,  0, +1),
        (x1, y2,  +1,  0,  0, -1),
        (x2, y2,  -1,  0,  0, -1),
    ):
        cv2.line(img, (px, py), (px + dx * length, py),       color, thickness, cv2.LINE_AA)
        cv2.line(img, (px, py), (px,               py + dy2 * length), color, thickness, cv2.LINE_AA)


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
        ax, ay = int(x1 + dx * a), int(y1 + dy * a)
        bx, by = int(x1 + dx * b), int(y1 + dy * b)
        cv2.line(img, (ax, ay), (bx, by), color, thickness, cv2.LINE_AA)


def _panel(img, top_left, lines, *, fg, bg, border, pad=6, line_h=16):
    x, y = top_left
    width = max(cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)[0][0]
                for s in lines) + 2 * pad
    height = line_h * len(lines) + 2 * pad
    overlay = img.copy()
    cv2.rectangle(overlay, (x, y), (x + width, y + height), bg, -1)
    cv2.addWeighted(overlay, 0.55, img, 0.45, 0, dst=img)
    cv2.rectangle(img, (x, y), (x + width, y + height), border, 1, cv2.LINE_AA)
    for i, s in enumerate(lines):
        cv2.putText(img, s, (x + pad, y + pad + line_h * (i + 1) - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, fg, 1, cv2.LINE_AA)

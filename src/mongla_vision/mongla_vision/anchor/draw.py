"""Draw what the matcher is actually doing.

The first version of this overlay drew a crosshair at the pose. It was
technically correct and told an operator almost nothing: a crosshair says the
lock MOVED, and the question you actually have standing at the poolside is
"locked onto WHAT?" -- two hundred inliers spread across the pool wall and two
hundred on the torpedo board are the same number and completely different
situations.

So this is the view the image-matching literature ships, and the one BumbleBee
put on their perception page: the reference template beside the live frame,
with a line per surviving correspondence and a quad around where the template
landed. You can see the match is real, see what it is on, and see it break --
lines fanning out to scattered points is a lock coming apart, and it looks
wrong long before the inlier count crosses any threshold.

Two decisions worth stating because both were wrong in the first draft:

  * INLIERS ONLY. Drawing every putative match produces a green haystack that
    looks impressive and hides the failure. The RANSAC survivors are the ones
    the pose was actually built from.
  * LINES ARE SUBSAMPLED, evenly across the sorted set rather than taking the
    first N. Four hundred lines is a solid block of colour; taking the first N
    biases toward one corner of the image because keypoints arrive in raster
    order.
"""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

# Enough to read the geometry, few enough to see through.
MAX_LINES = 60

_GREEN = (90, 255, 120)
_DARK = (16, 40, 24)
_AMBER = (60, 190, 255)


def _to_bgr(img: np.ndarray) -> np.ndarray:
    import cv2
    if img is None:
        return None
    if img.ndim == 2:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img.copy()


def match_panel(reference: np.ndarray, live: np.ndarray, pose,
                *, roi: Optional[Tuple[float, float, float, float]] = None,
                max_lines: int = MAX_LINES,
                ref_width: int = 260) -> np.ndarray:
    """Reference on the left, live on the right, correspondences between.

    `pose` is an `AnchorPose`. Returns a BGR canvas; if the pose is not locked
    the panel still renders with the reference and a LOST banner, because an
    operator needs to see the template that is failing just as much as one that
    is working.
    """
    import cv2

    live_bgr = _to_bgr(live)
    if live_bgr is None:
        return np.zeros((240, 320, 3), np.uint8)
    lh, lw = live_bgr.shape[:2]

    ref_bgr = _to_bgr(reference)
    if ref_bgr is None:
        ref_bgr = np.full((lh, ref_width, 3), 22, np.uint8)
        cv2.putText(ref_bgr, 'no reference', (12, lh // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (110, 120, 130), 1,
                    cv2.LINE_AA)
        rh, rw = ref_bgr.shape[:2]
        rs = 1.0
    else:
        rh0, rw0 = ref_bgr.shape[:2]
        rs = ref_width / float(rw0)
        ref_bgr = cv2.resize(ref_bgr, (ref_width, max(1, int(rh0 * rs))))
        rh, rw = ref_bgr.shape[:2]
        # Dim the reference slightly so the live frame reads as the primary
        # panel and the lines stay legible across both.
        ref_bgr = (ref_bgr * 0.75).astype(np.uint8)
        if roi is not None:
            x1, y1, x2, y2 = (v * rs for v in roi)
            cv2.rectangle(ref_bgr, (int(x1), int(y1)), (int(x2), int(y2)),
                          _AMBER, 1, cv2.LINE_AA)

    H = max(rh, lh)
    canvas = np.full((H, rw + lw, 3), 14, np.uint8)
    canvas[:rh, :rw] = ref_bgr
    canvas[:lh, rw:rw + lw] = live_bgr
    cv2.line(canvas, (rw, 0), (rw, H), (40, 48, 56), 1)

    ok = bool(getattr(pose, 'ok', False))
    rp = getattr(pose, 'ref_pts', None)
    lp = getattr(pose, 'live_pts', None)

    if ok and rp is not None and lp is not None and len(rp):
        rp = np.asarray(rp, np.float32)
        lp = np.asarray(lp, np.float32)
        # The anchor works at the backend's resolution; both panels are drawn at
        # their own. Scale each side independently -- assuming they match is how
        # a line lands on the right image at the wrong place.
        bw = bh = None
        if reference is not None:
            bh, bw = reference.shape[:2]
        sxl = lw / float(bw) if bw else 1.0
        syl = lh / float(bh) if bh else 1.0

        n = len(rp)
        if n > max_lines:
            idx = np.linspace(0, n - 1, max_lines).astype(int)
        else:
            idx = np.arange(n)
        for i in idx:
            a = (int(rp[i, 0] * rs), int(rp[i, 1] * rs))
            b = (int(lp[i, 0] * sxl) + rw, int(lp[i, 1] * syl))
            cv2.line(canvas, a, b, _GREEN, 1, cv2.LINE_AA)
            cv2.circle(canvas, a, 2, _GREEN, -1, cv2.LINE_AA)
            cv2.circle(canvas, b, 2, _GREEN, -1, cv2.LINE_AA)

        corners = getattr(pose, 'corners', None)
        if corners is not None:
            q = np.asarray(corners, np.float32).copy()
            q[:, 0] = q[:, 0] * sxl + rw
            q[:, 1] = q[:, 1] * syl
            pts = q.astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(canvas, [pts], True, _DARK, 5, cv2.LINE_AA)
            cv2.polylines(canvas, [pts], True, _GREEN, 2, cv2.LINE_AA)

    # Banner
    if ok:
        txt = (f'MATCHED  {pose.inliers}/{pose.matches} inliers'
               f'   scale x{pose.scale:.2f}')
        col = _GREEN
    else:
        txt = f'NO LOCK  {getattr(pose, "inliers", 0)} inliers'
        col = (90, 110, 255)
    cv2.rectangle(canvas, (0, H - 22), (rw + lw, H), (10, 14, 18), -1)
    cv2.putText(canvas, txt, (10, H - 7), cv2.FONT_HERSHEY_SIMPLEX, .48,
                col, 1, cv2.LINE_AA)
    return canvas

"""Which pixels the flow node may trust: corners, caustics, and the round trip.

Optical flow is only as good as the points it follows. This module picks them
and vets them, before `flow_math` turns their motion into metres:

  * `detect_corners`            the points worth tracking on a pool floor;
  * `caustic_score`,
    `suppress_caustics`,
    `patch_ncc_median`          sunlight drawn on the floor moves with the
                                waves, not the hull, so it is measured, eroded
                                away, or the frame is refused;
  * `forward_backward_error`    a point that does not track back to where it
                                started was never tracked.

No ROS. OpenCV is imported inside each function, so `flow_math` and the bench
tools load this without it.
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np


# ── sun caustics ─────────────────────────────────────────────────────────────
# An outdoor pool lights its floor through a moving lens. Refracted sunlight
# draws bright filaments that sweep across the floor at the surface waves'
# speed, independent of the hull -- and a corner detector prefers exactly those
# filaments. Measured 2026-09-15 on REAL caustics (RoboSub 2025 downward
# footage, `octagon_1.mp4`) composited over three REAL floors moved by a KNOWN
# 13.4 px baseline:
#
#     floor          raw LK        eroded 5x5    eroded 7x7     no caustics
#     tiles         11.19 px        0.18 px       0.09 px        0.000 px
#     octagon mat    1.01           0.14          0.11           0.000
#     slalom floor  13.31          12.86         12.47           0.000
#
# Zero refusals in every row: the node would have published the WAVES' speed as
# the hull's. The filaments are bright and thinner than the kernel, so a grey
# erosion (a local minimum) removes them while every dark feature a pool floor
# actually has -- grout, lane tiles, prop outlines -- survives. On a floor with
# no dark texture of its own (the slalom row) this does NOT fix it; the node
# refuses that case instead, see `CAUSTIC_NCC_MIN`.
CAUSTIC_ERODE_PX = 7

# Bright thin-structure energy, top-hat(9x9) mean / frame mean. MEASURED on the
# real archive, one frame every 5 s:
#   caustic-free (Mirpur indoor x3, final_run x5)     max 0.051
#   RoboSub downward in sun (bin+oct_1, octagon_1/2)  min 0.090, up to 0.21
# 0.07 sits in the gap. RoboSub FORWARD clips read 0.04-0.06, so a forward
# camera is near the line; this is for the DOWNWARD camera.
CAUSTIC_TOPHAT_MAX = 0.07


# ...and when erosion leaves nothing of the floor. Caustic CELLS survive the
# erosion and change shape between frames; a real floor does not. Median NCC of
# the tracked 15x15 patches, eroded, on REAL RoboSub footage in sun
# (`octagon_1.mp4`, the same frames, two regions, B = 2 and 6):
#   plain concrete   p50 0.53-0.58, max 0.646
#   lane tiles       p10 0.797, p50 0.83
#   whole sun frames (4 segments, props/lanes in view)   min 0.774
#   whole shaded frames (4 segments)                      min 0.950
# PROVISIONAL: one clip carries the plain-floor side of the gap.
CAUSTIC_NCC_MIN = 0.71


def patch_ncc_median(a, b, p0, p1, half: int = 7) -> float:
    """Median zero-mean NCC between each point's patch in `a` and its match in `b`."""
    import cv2
    size = (2 * half + 1, 2 * half + 1)
    vals = []
    for (x0, y0), (x1, y1) in zip(np.asarray(p0).reshape(-1, 2),
                                  np.asarray(p1).reshape(-1, 2)):
        pa = cv2.getRectSubPix(a, size, (float(x0), float(y0))).astype(np.float64)
        pb = cv2.getRectSubPix(b, size, (float(x1), float(y1))).astype(np.float64)
        pa -= pa.mean()
        pb -= pb.mean()
        d = math.sqrt(float((pa * pa).sum() * (pb * pb).sum()))
        vals.append(float((pa * pb).sum()) / d if d > 1e-9 else 0.0)
    return float(np.median(vals)) if vals else 0.0


def caustic_score(gray) -> float:
    """Bright thin-structure energy relative to brightness: sun caustics score high."""
    import cv2
    g = np.asarray(gray, dtype=np.float32)
    mean = float(g.mean())
    if mean < 1.0:
        return 0.0
    ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    return float(cv2.morphologyEx(g, cv2.MORPH_TOPHAT, ker).mean()) / mean


def suppress_caustics(gray, size: int = CAUSTIC_ERODE_PX):
    """Grey erosion: removes bright filaments thinner than `size`, keeps dark texture."""
    import cv2
    return cv2.erode(gray, np.ones((size, size), np.uint8))


def detect_corners(gray, *, want: int = 80, max_corners: int = 200,
                   min_distance: int = 8, block: int = 7,
                   quality_ladder=(0.01, 0.004, 0.0015),
                   buckets: int = 4):
    """Corners, spread over a grid, with the quality bar LOWERED until there
    are enough of them.

    A fixed `qualityLevel` is a fixed assumption about the floor, and it is
    the wrong one for the surface we actually fly over. Shi-Tomasi's threshold
    is RELATIVE to the strongest corner in the frame, so 0.01 keeps only
    features within 100x of the best one -- fine over gravel or a tiled grout
    line, and starving over smooth concrete or a painted pool bottom.
    Measured on this bench: a textured surface gave 150-160 points and a plain
    one gave 19, with the same code and the same camera. Nineteen is above the
    8-point minimum the rigid fit needs, so nothing REFUSES -- the fit just
    gets progressively less over-determined and its residual less meaningful,
    which is degradation without a warning.

    So the bar descends until the frame yields `want` points. A weak corner is
    a worse feature than a strong one, but forward-backward rejection and the
    RANSAC fit both cull the ones that do not survive, and having more
    candidates strictly improves what those two have to work with.

    Grid bucketing on top, because a plain floor with ONE bright mark produces
    every corner in one spot, and a clustered set cannot distinguish a
    rotation from a translation -- the two are separated by how the flow field
    VARIES across the frame.

    The ladder is a trade, measured, not a free win. On this bench, static
    frames, forward-backward error as the score:

        quality   corners   fb p50   fb p90   survive <=1px
        0.05         66      0.114    2.24      47  (71 %)
        0.01        132      0.287    9.81      63  (48 %)
        0.0015      300      0.698   32.85      82  (27 %)

    Descending the ladder yields MORE absolute survivors (47 -> 82) at a much
    lower survival RATE, and it costs 300 LK tracks instead of 66. It is worth
    it here because the Pi has cores to spare and because forward-backward and
    RANSAC both cull what does not hold up -- but a weak corner IS a worse
    feature, and anyone reading a high corner count as a healthy image will be
    wrong. Count survivors, never candidates.

    The old 0.0006 rung is REMOVED: it returned results identical to 0.0015
    (both saturate `max_corners`), so it was pure cost.
    """
    import cv2
    h, w = gray.shape[:2]
    n = max(1, int(buckets))
    per = max(4, int(max_corners) // (n * n))
    best = None
    for q in quality_ladder:
        out = []
        for iy in range(n):
            for ix in range(n):
                y0, y1 = iy * h // n, (iy + 1) * h // n
                x0, x1 = ix * w // n, (ix + 1) * w // n
                sub = gray[y0:y1, x0:x1]
                if sub.size == 0:
                    continue
                pts = cv2.goodFeaturesToTrack(
                    sub, maxCorners=per, qualityLevel=q,
                    minDistance=min_distance, blockSize=block)
                if pts is None:
                    continue
                out.append(pts.reshape(-1, 2)
                           + np.array([x0, y0], dtype=np.float32))
        got = np.concatenate(out) if out else None
        if got is not None and (best is None or len(got) > len(best)):
            best = got
        if got is not None and len(got) >= want:
            break
    if best is None:
        return None
    return best.reshape(-1, 1, 2).astype(np.float32)


def forward_backward_error(prev_gray, next_gray, prev_pts, next_pts,
                           lk_params) -> Optional["np.ndarray"]:
    """Per-point round-trip error in px: track forward, then track back.

    LK does not report its own failures honestly. `status=1` means the
    solver converged, not that it converged on the RIGHT patch. Over a
    repetitive texture -- a tiled pool floor is the worst case we will
    actually fly over -- it converges confidently one tile away, and the
    displacement it returns is a clean multiple of the tile pitch. Nothing in
    the residual or the status flag distinguishes that from a correct match.

    Tracking the result BACK to the original frame does distinguish it: a
    correct correspondence returns to where it started, a one-tile-off match
    returns one tile away. This is the standard forward-backward (bidirectional)
    check and it is the cheapest real defence against the exact failure a pool
    invites.

    Returns per-point |p0 - p0_roundtrip|, or None if it cannot be computed.
    """
    import cv2
    if prev_pts is None or next_pts is None:
        return None
    back, st_b, _ = cv2.calcOpticalFlowPyrLK(next_gray, prev_gray,
                                             next_pts, None, **lk_params)
    if back is None:
        return None
    p0 = np.asarray(prev_pts, dtype=np.float64).reshape(-1, 2)
    pb = np.asarray(back, dtype=np.float64).reshape(-1, 2)
    err = np.hypot(p0[:, 0] - pb[:, 0], p0[:, 1] - pb[:, 1])
    if st_b is not None:
        lost = ~np.asarray(st_b).reshape(-1).astype(bool)
        err[lost] = np.inf          # never silently pass an unmatched point
    return err

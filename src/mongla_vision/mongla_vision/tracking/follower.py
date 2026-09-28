"""The fast rung: carry a box across the frames the detector has nothing for.

WHERE IT SITS. Three timescales, each bounding the one below:

    ~50 Hz    THIS -- sparse LK inside the last box   frame-to-frame, DRIFTS
    ~5-10 Hz  the anchor (XFeat homography)           frame-to-reference, bounded
    on detect full reset                              semantic, resets both

Measured detection gaps on real competition footage are p50 0.155 s / p90
0.651 s / p99 2.418 s. At 50 Hz that is a median of ~8 frames with no target
position at all, and the control loop's authority decays through every one.

WHY OPTICAL FLOW AND NOT A BIGGER DETECTOR. `goodFeaturesToTrack` +
`calcOpticalFlowPyrLK` on a 320x240 ROI measured **8.0 ms on ONE core of the
Pi** with the full vision stack running -- 125 Hz for something the loop needs
at 50. It is the cheapest possible way to answer "where did the box go" and it
runs on cores the detector is not using.

WHY IT MUST BE RESET, AND CANNOT STAND ALONE. Frame-to-frame tracking
accumulates error without bound -- that is not a tuning problem, it is what
integrating a noisy displacement does. Every accepted detection resets it, and
the anchor bounds it in between. A follower left running on its own is a
confident lie with a growing error bar.

THE QUALITY SIGNAL IS FORWARD-BACKWARD ERROR. Track the points forward, then
track them back; a point that does not return to where it started was not
tracked, it was guessed. This is what lets the follower REFUSE rather than
publish a drifting box, and refusing is the whole reason it is safe to act on.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

# Points that survive the round trip by more than this many pixels are
# discarded. 1.0 px is the usual value in the FB-error literature and is
# comfortably inside the sub-pixel accuracy LK claims.
FB_MAX_PX = 1.0

# Below this many surviving points the box is not being tracked, it is being
# extrapolated from noise. Refusing is the honest answer.
MIN_POINTS = 8

# Fraction of the original points that must survive. A box that keeps 3 of 60
# points has lost its subject even if those 3 round-trip perfectly -- they are
# most likely background that happened to be inside the box.
MIN_SURVIVAL = 0.30

# ⛔ A CORNER COUNT IS NOT EVIDENCE THE PATCH IS TRACKABLE, and this is the
# measurement that says so. Over 155 boxes on four archive clips, binned by the
# grey-level standard deviation of the patch:
#
#     patch std     boxes   corners found (p50)   LK survival (p50)
#       0 - 2          30            98                 0.12
#       2 - 5          65            23                 0.36
#       5 - 10         21            22                 0.91
#      10 - 20         20            23                 0.83
#      20+             19            21                 0.95
#
# `goodFeaturesToTrack` returns NINETY-EIGHT corners from a patch with two grey
# levels of variation -- more than from a richly textured one, because sensor
# noise in a flat region produces plenty of local maxima -- and 88 % of them die
# within 30 frames. So `reset()` returning a healthy `n0` said nothing at all
# about whether the box could be carried, and the ladder learned the truth
# thirty frames later, holding a box it believed in, built on noise.
#
# The bar below is chosen from the same data, by the cost of each mistake.
# Refusing a trackable box costs one gap the detector must cover; accepting an
# untrackable one costs a CONFIDENTLY WRONG carried box, which is the failure
# mode that ends runs. At std < 2.0 the refusal is right 86.7 % of the time,
# for a 19.4 % refusal rate -- the highest-precision operating point measured:
#
#     bar   refused   of refused, really dead   of kept, really dead
#     2.0    19.4 %          86.7 %                   25.6 %
#     4.0    49.7 %          64.9 %                   10.3 %
#
# ⚠ THE POPULATION IS NOT REAL DETECTION BOXES. Those 155 boxes are a GRID
# across the frame, so most are water and wall. A real detection sits on a prop
# and is textured, so the 19.4 % refusal rate does NOT transfer -- it is an
# upper bound on the cost, measured on the worst possible sample. What does
# transfer is the cliff: below ~2 grey levels there is no information to track,
# and no appearance-based rung can change that.
#
# ⛔ MEASURED AND REJECTED, so nobody re-derives them: a colour back-projection
# rung (HSV hue+saturation) and NCC template matching were both tried as a
# third, model-free rung for exactly these frames. The histogram LOSES to a
# decoy box on 69 % of gate.mkv frames (margin -0.049), and NCC scores a
# confident-looking 0.891 peak with a margin over its own second-best of
# +0.005 -- no uniqueness at all. Neither can work, because the failure is not
# in the algorithm: the patch carries 0.92 bits of entropy where a trackable
# one carries 1.58. Redundancy here has to come from a different KIND of
# information, not another appearance tracker.
MIN_PATCH_STD = 2.0

_GFTT = dict(maxCorners=120, qualityLevel=0.01, minDistance=7, blockSize=7)
_LK = dict(winSize=(15, 15), maxLevel=2)


@dataclass
class FollowResult:
    ok: bool
    xyxy: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    points: int = 0
    survival: float = 0.0
    fb_median: float = float('nan')
    # Uniform scale from the similarity fit; 1.0 when the fit
    # was refused and the box was carried by translation alone.
    scale: float = 1.0

    @property
    def confidence(self) -> float:
        """0..1 from point survival, so the control path can weigh it like a
        detection score rather than treating a follow as a certainty."""
        return float(min(1.0, self.survival)) if self.ok else 0.0


class Follower:
    """Carry one box forward on optical flow. No ROS, so the bench and the node
    exercise the same object."""

    # Minimum inliers before a similarity fit is trusted. Below this a fit is
    # free to be wildly wrong and the median translation is the safer answer.
    _MIN_FIT_PTS = 12
    # Per-frame scale bounds. A target does not double in one frame; a fit
    # that says so has latched onto something else.
    _SCALE_LO, _SCALE_HI = 0.90, 1.11

    def __init__(self, *, fb_max_px: float = FB_MAX_PX,
                 min_points: int = MIN_POINTS,
                 min_survival: float = MIN_SURVIVAL):
        self._fb = float(fb_max_px)
        self._min_fit_pts = self._MIN_FIT_PTS
        self._scale_lo, self._scale_hi = self._SCALE_LO, self._SCALE_HI
        self._min_pts = int(min_points)
        self._min_surv = float(min_survival)
        self._prev: Optional[np.ndarray] = None
        self._pts: Optional[np.ndarray] = None
        self._box: Optional[Tuple[float, float, float, float]] = None
        # Grey-level spread of the last patch offered to `reset`. Kept so a
        # refusal can say WHY, rather than looking like "no corners found".
        self.last_patch_std = float('nan')
        self._n0 = 0

    @property
    def active(self) -> bool:
        return self._pts is not None and len(self._pts) >= self._min_pts

    def reset(self, gray: np.ndarray, xyxy) -> int:
        """Seed from a DETECTION. Returns the point count.

        Called on every accepted detection, which is what keeps the drift
        bounded -- the follower never runs longer than one detection gap.
        """
        import cv2
        x1, y1, x2, y2 = (int(round(v)) for v in xyxy)
        h, w = gray.shape[:2]
        x1 = max(0, min(x1, w - 2)); x2 = max(x1 + 2, min(x2, w))
        y1 = max(0, min(y1, h - 2)); y2 = max(y1 + 2, min(y2, h))
        roi = gray[y1:y2, x1:x2]
        # Refuse a patch with no information BEFORE asking for corners -- see
        # MIN_PATCH_STD. Cheap, and it is the only check that distinguishes
        # "98 corners of noise" from "22 corners of target".
        self.last_patch_std = float(roi.std()) if roi.size else 0.0
        if self.last_patch_std < MIN_PATCH_STD:
            self._pts = None
            return 0
        p = cv2.goodFeaturesToTrack(roi, **_GFTT)
        if p is None or len(p) < self._min_pts:
            self._pts = None
            return 0
        p = p.reshape(-1, 2) + np.array([x1, y1], np.float32)
        self._pts = p.astype(np.float32)
        self._prev = gray.copy()
        self._box = (float(x1), float(y1), float(x2), float(y2))
        self._n0 = len(p)
        return self._n0

    def drop(self) -> None:
        self._pts = self._prev = self._box = None
        self._n0 = 0

    def step(self, gray: np.ndarray) -> FollowResult:
        """Advance one frame. Returns the carried box, or a refusal."""
        import cv2
        if self._pts is None or self._prev is None or self._box is None:
            return FollowResult(ok=False)

        p0 = self._pts.reshape(-1, 1, 2)
        p1, st, _e = cv2.calcOpticalFlowPyrLK(self._prev, gray, p0, None, **_LK)
        if p1 is None:
            self.drop()
            return FollowResult(ok=False)
        # FORWARD-BACKWARD: track them back and keep only the points that
        # return. Without this, LK reports a confident displacement for points
        # that have left the frame or landed on a repeating texture.
        p0r, st2, _e2 = cv2.calcOpticalFlowPyrLK(gray, self._prev, p1, None, **_LK)
        if p0r is None:
            self.drop()
            return FollowResult(ok=False)
        fb = np.linalg.norm(p0.reshape(-1, 2) - p0r.reshape(-1, 2), axis=1)
        good = (st.ravel() == 1) & (st2.ravel() == 1) & (fb < self._fb)
        n = int(good.sum())
        surv = n / max(self._n0, 1)
        if n < self._min_pts or surv < self._min_surv:
            self.drop()
            return FollowResult(ok=False, points=n, survival=surv,
                                fb_median=float(np.median(fb)) if len(fb) else float('nan'))

        a = self._pts[good]
        b = p1.reshape(-1, 2)[good]
        x1, y1, x2, y2 = self._box

        # ⭐ SIMILARITY FIT, NOT TRANSLATION ALONE. Median translation carries
        # the box but cannot express SCALE or ROTATION, so an approaching
        # target kept a fixed-size box while it grew on screen -- and section
        # 23 measured that apparent SIZE is exactly what the approach
        # controller reads. A partial-affine (similarity) fit recovers
        # translation + rotation + uniform scale from the correspondences we
        # already have, for microseconds on ~100 points.
        #
        # This is the third leg of the classic recipe -- pyramidal LK, a
        # forward-backward check, and a RANSAC similarity fit -- which a 2026
        # comparison found beats CSRT on mobile robots by a wide margin
        # (optical flow RMSE 10.79 px at 30 fps against CSRT's 252.35 px at
        # 4 fps). We had the first two.
        #
        # ⛔ RANSAC, and it must be able to FAIL. A similarity fit on points
        # that have partly latched onto the background silently reports a
        # scale change that is really parallax, so a failed or degenerate fit
        # falls back to the median translation rather than being trusted.
        scale = 1.0
        box = None
        if n >= self._min_fit_pts:
            M, inl = cv2.estimateAffinePartial2D(
                a.reshape(-1, 1, 2), b.reshape(-1, 1, 2),
                method=cv2.RANSAC, ransacReprojThreshold=3.0,
                maxIters=200, confidence=0.99)
            if M is not None and inl is not None \
                    and int(inl.sum()) >= self._min_fit_pts:
                # A similarity matrix is [[s*cos, -s*sin, tx], [s*sin, s*cos, ty]]
                s_fit = float(np.hypot(M[0, 0], M[1, 0]))
                # ⛔ A scale jump per frame is a fit failure, not a target that
                # doubled in one frame. Clamp rather than trust.
                if self._scale_lo <= s_fit <= self._scale_hi:
                    cx, cy = 0.5 * (x1 + x2), 0.5 * (y1 + y2)
                    nc = M @ np.array([cx, cy, 1.0])
                    hw = 0.5 * (x2 - x1) * s_fit
                    hh = 0.5 * (y2 - y1) * s_fit
                    box = (float(nc[0] - hw), float(nc[1] - hh),
                           float(nc[0] + hw), float(nc[1] + hh))
                    scale = s_fit

        if box is None:
            # Median translation, not mean: a handful of points latching onto a
            # passing feature would drag a mean and leave a median alone.
            d = np.median(b - a, axis=0)
            box = (x1 + d[0], y1 + d[1], x2 + d[0], y2 + d[1])

        self._pts = b.astype(np.float32)
        self._prev = gray.copy()
        self._box = box
        return FollowResult(ok=True, xyxy=box, points=n, survival=surv,
                            fb_median=float(np.median(fb[good])),
                            scale=scale)

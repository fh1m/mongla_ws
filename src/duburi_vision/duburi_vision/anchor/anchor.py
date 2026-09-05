"""Hold a target with NO detection at all.

THE GOAL THIS SERVES: the vehicle should always have a target position, and a
detection is only the *best* of several ways to get one. Measured gaps on real
competition footage are p50 0.155 s / p90 0.651 s / p99 2.418 s -- so a stack
whose only source of truth is the detector spends a real fraction of every run
blind.

WHY A FEATURE ANCHOR IS NOT REDUNDANT WITH A BETTER DETECTOR. The detector
answers a SEMANTIC question -- "is there a gate here?" -- and fails on domain
shift, blur, unusual pose, low contrast. Matching answers a GEOMETRIC one --
"where did THIS patch of texture go?" -- and fails on texture loss and large
viewpoint change. **The failure modes are decorrelated.** That is the whole
value, and it is why this belongs beside the detector rather than behind it.

FRAME-TO-REFERENCE, NEVER FRAME-TO-FRAME. Every live frame is matched against
the STORED reference, so error is bounded by match quality and does not
accumulate. Optical flow (the fast rung) is frame-to-frame and drifts without
bound; this is what bounds it. Two rungs, two timescales, and the distinction is
the reason both exist.

WHAT IT PRODUCES. A homography from reference to live gives more than a bbox
centre ever could:

    tx, ty    where the reference view sits in the live frame  -> lat / depth
    theta     in-plane rotation                                -> yaw (weak axis
              on a forward camera; see the sign note below)
    scale     how much bigger the reference has become         -> RANGE change
    inliers   how much to believe any of it                    -> the gate

SIGN CONVENTION, LOAD-BEARING. `H` maps REFERENCE -> LIVE, so pushing the
reference centre through it answers "where does the reference appear now",
which has the SAME sign as a detection's offset-from-centre (`+x` = right).
That makes the control laws identical to the pool-verified vision loop.
Swapping the argument order to `findHomography` inverts every axis into
positive feedback and the hull drives AWAY from the lock -- a real bug from an
earlier attempt at this, and the reason the order is asserted in tests.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

# A homography needs roughly this many inliers before its pose means anything.
# Measured on real footage: good locks carry 56-682 inliers, and the cases that
# genuinely fail (ORB on murky water) sit at 0-7. Fifteen separates them with
# room on both sides.
MIN_INLIERS = 15

# Cosine similarity floor for a mutual match. XFeat's own default.
MIN_COSSIM = 0.82

# RANSAC reprojection tolerance, pixels at the matcher's working resolution.
RANSAC_PX = 3.0


@dataclass
class AnchorPose:
    """Where the reference sits in the live frame, and how much to trust it."""
    ok: bool
    tx: float = float('nan')      # px from frame centre, + = reference is RIGHT
    ty: float = float('nan')      # px from frame centre, + = reference is BELOW
    theta: float = float('nan')   # rad, in-plane rotation
    scale: float = float('nan')   # >1 = reference appears LARGER, i.e. closer
    inliers: int = 0
    matches: int = 0
    # Where the reference's four corners land in the live frame, (4,2) or None.
    # Carried because a point tells you the lock MOVED and a quad tells you what
    # it is locked ONTO -- and because a footprint leaving the frame is the
    # earliest warning that the reference is about to become unusable, which a
    # centre offset alone cannot express.
    corners: object = None
    # The surviving correspondences: (N,2) in the REFERENCE and (N,2) in the
    # LIVE frame, RANSAC inliers only. Carried for display and for diagnosis.
    #
    # An inlier COUNT says the match is good; it cannot say good *at what*. Two
    # hundred inliers spread over the pool wall and two hundred on the torpedo
    # board are the same number and completely different situations, and only
    # drawing the correspondences tells them apart. This is the view every
    # image-matching paper ships for exactly that reason.
    ref_pts: object = None
    live_pts: object = None

    @property
    def confidence(self) -> float:
        """0..1, saturating. Used the way detection confidence is used."""
        if not self.ok:
            return 0.0
        return min(1.0, self.inliers / 100.0)


def reference_corners(H: np.ndarray, w: int, h: int, roi=None) -> np.ndarray:
    """The reference REGION's four corners pushed through H -> (4,2) live px.

    `roi` is the region the reference was snapped from, in backend pixels. It is
    NOT optional detail: with a whole-frame quad, an ROI-snapped anchor reports
    a footprint covering the entire image, and anything deriving a target box
    from it -- `lock_node` does -- publishes the whole frame as the target
    position. That is a confident, useless answer, and it looks completely
    normal until you draw it.
    """
    import cv2
    if roi is not None:
        x1, y1, x2, y2 = (float(v) for v in roi)
    else:
        x1, y1, x2, y2 = 0.0, 0.0, float(w), float(h)
    quad = np.array([[[x1, y1]], [[x2, y1]], [[x2, y2]], [[x1, y2]]], np.float32)
    return cv2.perspectiveTransform(quad, H).reshape(-1, 2)


def pose_from_homography(H: np.ndarray, w: int, h: int, roi=None) -> tuple:
    """(tx, ty, theta, scale) for the reference CENTRE pushed through H.

    The centre is used rather than a corner because it is the point the control
    loop steers, and because pushing a near-centre point through a homography is
    interpolation while a corner is extrapolation -- the corner answer degrades
    fast when the match is marginal, and does so without any change in inlier
    count to warn you.
    """
    import cv2
    if roi is not None:
        x1, y1, x2, y2 = (float(v) for v in roi)
        rcx, rcy = (x1 + x2) * 0.5, (y1 + y2) * 0.5
    else:
        rcx, rcy = w * 0.5, h * 0.5
    p = cv2.perspectiveTransform(np.array([[[rcx, rcy]]], np.float32), H)[0, 0]
    # Offset is always measured from the FRAME centre -- that is what the
    # control loop steers on -- but the point pushed through H is the
    # REGION's centre, so an ROI lock reports where the TARGET is rather than
    # where the middle of the old view went.
    tx = float(p[0] - w * 0.5)
    ty = float(p[1] - h * 0.5)
    # Rotation and scale from the linear part. sqrt(det) is used for scale
    # rather than |H[0,0]| so that a rotated match does not read as a scale
    # change -- they are different axes and the control loop treats them so.
    theta = float(math.atan2(H[1, 0], H[0, 0]))
    det = float(H[0, 0] * H[1, 1] - H[0, 1] * H[1, 0])
    scale = float(math.sqrt(abs(det))) if det != 0.0 else float('nan')
    return tx, ty, theta, scale


class Anchor:
    """A snapped reference view, and the ability to relocate it.

    Deliberately holds NO ROS: it takes frames and returns a pose, so the same
    object is exercised by the node, by a bag replay and by the archive bench
    with one implementation. Every measurement in this module's docstrings came
    from driving this class over recorded competition footage.
    """

    def __init__(self, backend, *, min_inliers: int = MIN_INLIERS,
                 min_cossim: float = MIN_COSSIM):
        self._be = backend
        self._min_inliers = int(min_inliers)
        self._min_cossim = float(min_cossim)
        self._ref_kpts: Optional[np.ndarray] = None
        self._ref_desc: Optional[np.ndarray] = None
        self._ref_shape = (0, 0)

    # -- reference ---------------------------------------------------------- #
    def snap(self, gray: np.ndarray, roi=None) -> int:
        """Store this view as the reference. Returns the keypoint count.

        `roi` = (x1, y1, x2, y2) in the ORIGINAL frame's pixels. Pass it to
        lock the TARGET; omit it to lock the SCENE. The difference is not
        cosmetic and it is the first thing that surprises anyone watching:

          whole frame -- hundreds of background keypoints outvote the subject,
                         so the homography reports what the ROOM is doing. On a
                         static camera the lock correctly sits still even as
                         someone walks through it. That is the right answer for
                         station-keeping and for ego-motion, and the wrong one
                         for following a prop.
          roi         -- only keypoints inside the box become the reference, so
                         the homography follows the target. This is what a
                         torpedo run needs: lock the BOARD, not the pool wall
                         behind it.

        Keypoints keep FULL-FRAME coordinates either way, so every sign and
        scale downstream is identical and `pose_from_homography` does not need
        to know which mode was used. Cropping the image instead would shift the
        origin and silently move every pose the anchor reports.

        The count is returned rather than a bool because it is a real health
        signal: a workable reference carries ~1000 keypoints and a hopeless one
        carries single digits (ORB found SEVEN in a whole Mirpur frame). A
        caller that snaps a near-empty reference should learn it now, not when
        the lock silently never engages.
        """
        k, d = self._be.detect(gray)
        if roi is not None and len(k):
            # The backend works at its own resolution; the ROI arrives in the
            # caller's. Scale the box, never the keypoints.
            fh, fw = gray.shape[:2]
            sx = self._be.w / float(fw)
            sy = self._be.h / float(fh)
            x1, y1, x2, y2 = (float(v) for v in roi)
            x1, x2 = sorted((x1 * sx, x2 * sx))
            y1, y2 = sorted((y1 * sy, y2 * sy))
            m = ((k[:, 0] >= x1) & (k[:, 0] <= x2)
                 & (k[:, 1] >= y1) & (k[:, 1] <= y2))
            k, d = k[m], d[m]
        self._ref_kpts, self._ref_desc = k, d
        self._ref_shape = (self._be.h, self._be.w)
        self._ref_gray = gray.copy()
        # Stored in BACKEND pixels (already scaled above), which is the frame
        # `locate()` and the homography both work in.
        self._ref_roi = (float(x1), float(y1), float(x2), float(y2)) \
            if roi is not None else None
        return int(len(k))

    @property
    def reference_image(self):
        """The snapped frame, for display. Kept so an operator can SEE what the
        lock is matching against -- a number of inliers says the match is good
        and cannot say good *at what*."""
        return getattr(self, '_ref_gray', None)

    @property
    def reference_roi(self):
        return getattr(self, '_ref_roi', None)

    def clear(self) -> None:
        self._ref_kpts = self._ref_desc = None
        self._ref_gray = None
        self._ref_roi = None

    @property
    def has_reference(self) -> bool:
        return self._ref_desc is not None and len(self._ref_desc) > 0

    @property
    def reference_keypoints(self) -> int:
        return 0 if self._ref_kpts is None else int(len(self._ref_kpts))

    # -- relocate ----------------------------------------------------------- #
    def locate(self, gray: np.ndarray) -> AnchorPose:
        """Where is the reference in this frame?"""
        import cv2
        if not self.has_reference:
            return AnchorPose(ok=False)
        k1, d1 = self._be.detect(gray)
        if len(k1) < 8:
            return AnchorPose(ok=False)
        i0, i1 = self._be.match(self._ref_desc, d1, self._min_cossim)
        if len(i0) < 8:
            return AnchorPose(ok=False, matches=int(len(i0)))
        src = self._ref_kpts[i0].reshape(-1, 1, 2)
        dst = k1[i1].reshape(-1, 1, 2)
        # USAC_MAGSAC, not plain RANSAC: it is the least sensitive of OpenCV's
        # estimators to the inlier threshold, which is the one number here we
        # have no principled way to set.
        H, mask = cv2.findHomography(src, dst, cv2.USAC_MAGSAC, RANSAC_PX)
        if H is None or mask is None:
            return AnchorPose(ok=False, matches=int(len(i0)))
        inl = int(mask.sum())
        if inl < self._min_inliers:
            # Below the bar the pose is not "less accurate", it is arbitrary.
            # Reporting it with a low confidence would invite a caller to use
            # it anyway; refusing is the honest answer.
            return AnchorPose(ok=False, inliers=inl, matches=int(len(i0)))
        h, w = self._ref_shape
        roi = getattr(self, '_ref_roi', None)
        tx, ty, theta, scale = pose_from_homography(H, w, h, roi)
        keep = mask.ravel().astype(bool)
        return AnchorPose(ok=True, tx=tx, ty=ty, theta=theta, scale=scale,
                          inliers=inl, matches=int(len(i0)),
                          corners=reference_corners(H, w, h, roi),
                          ref_pts=src.reshape(-1, 2)[keep],
                          live_pts=dst.reshape(-1, 2)[keep])

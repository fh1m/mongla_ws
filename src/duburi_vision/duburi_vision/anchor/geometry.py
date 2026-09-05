"""The half of the homography we were throwing away.

`pose_from_homography` keeps tx, ty, theta and scale -- four numbers out of a
transform that also encodes, given a calibrated camera, the **orientation of
the plane being looked at**. That last one is not a nicety: it is the torpedo
shot's actual precondition. A centred bounding box says the board is in front
of us; it cannot say whether we are square to it, and a round fired at 30 deg
off-normal misses an opening it was perfectly centred on.

WHAT THIS ADDS, and what it deliberately does not:

    tilt_deg    angle between the plane normal and the optical axis.
                0 = square on. THE new quantity.
    yaw/pitch   that tilt split into the two axes a hull can act on --
                yaw_deg is fixed by strafing, pitch_deg by depth.
    normal      the unit normal itself, camera frame.

It does NOT report range. `AnchorPose.scale` already carries the apparent size
change and, measured against a synthetic ground truth, `sqrt(det)` on a
`findHomography`-normalised H is EXACT for a square-on approach -- 0.00 % error
at 2.0, 1.5, 1.0 and 0.6 m. (An earlier draft of this module claimed 25-70 %
error there and was WRONG: the synthetic H had not been normalised the way
`findHomography` returns it, and normalisation is where that scale lives.)

Under tilt the two diverge -- at 30 deg and d=1.0 the determinant reads +28.6 %
against the plane-distance ratio -- but that is not a defect in the formula. It
is that **a tilted plane has no single range**: its near and far edges are at
different distances, and the apparent scale genuinely differs across the image
(measured: centre ROI 1.7653 vs off-axis 1.3914, a 21 % spread on one frame).
The honest response is not a cleverer scalar. It is to report the tilt, so a
caller knows when the scalar means anything -- which is what this module is for.

⛔ THE CORRESPONDENCES ARE REQUIRED, AND THIS IS THE WHOLE DESIGN.
`decomposeHomographyMat` returns FOUR solutions. Two of them face the camera,
so the obvious rule -- "take the normal with n_z < 0" -- picks between them by
accident. Measured against known ground truth:

    truth      point-filtered      "first facing" heuristic
     0 deg          0.00                0.00
    10 deg         10.00                1.42      <-- wrong
    30 deg         30.00                4.01      <-- wrong, and confidently
    45 deg         45.00               45.00
    60 deg         60.00               60.00

The heuristic is exact where the tilt is large and obvious, and wrong where the
tilt is small -- which is precisely the terminal-alignment regime this exists to
serve. It reports "nearly square" for a board 30 deg off. So the points are a
required argument rather than an optional refinement: there is no defensible
answer without them, and an optional one would be taken.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

# The optical axis in the camera frame. A plane facing the camera has a normal
# antiparallel to it.
_AXIS = np.array([0.0, 0.0, -1.0])


def _normalised(H, K):
    """H scaled so the EUCLIDEAN homography has unit middle singular value.

    Not a nicety -- it is the precondition the decomposition is derived under,
    and OpenCV does not apply it robustly. Measured on synthetic planes at a
    fixed 0.5 m approach, `decomposeHomographyMat` returns **all-NaN normals**
    at 5, 15 and 20 degrees of tilt while working at 0, 10, 25, 30 and beyond:
    a scattered numerical failure, not a clean degeneracy, so no amount of
    reasoning about "degenerate cases" would have predicted which angles.
    Pre-normalising fixes every one of them and recovers the true normal
    exactly. Returns None if H is not invertible enough to try.
    """
    try:
        He = np.linalg.inv(K) @ H @ K
        s = np.linalg.svd(He, compute_uv=False)
    except np.linalg.LinAlgError:
        return None
    if not np.isfinite(s).all() or s[1] <= 1e-12:
        return None
    return K @ (He / s[1]) @ np.linalg.inv(K)


@dataclass
class PlaneGeometry:
    """Orientation of the matched plane, in the LIVE camera frame."""
    ok: bool
    tilt_deg: float = float('nan')    # 0 = square on
    yaw_deg: float = float('nan')     # + = plane's near edge is to the RIGHT
    pitch_deg: float = float('nan')   # + = plane's near edge is BELOW
    normal: Optional[Tuple[float, float, float]] = None
    ambiguous: bool = False           # more than one solution survived the filter

    @property
    def square_within(self) -> float:
        """Convenience for a gate: how far from square, always positive."""
        return abs(self.tilt_deg)


def plane_geometry(H, K, ref_pts, live_pts) -> PlaneGeometry:
    """Plane orientation from a homography and a calibrated camera.

    `ref_pts` / `live_pts` are the RANSAC inliers `AnchorPose` already carries,
    (N,2) each, in the same pixel frame `H` was fitted in. See the module
    docstring for why they are not optional.

    Returns `ok=False` rather than a guess whenever the decomposition cannot be
    disambiguated -- a wrong tilt is worse than no tilt, because the caller
    would act on it.
    """
    import cv2
    if H is None or K is None or ref_pts is None or live_pts is None:
        return PlaneGeometry(ok=False)
    ref = np.asarray(ref_pts, np.float32).reshape(-1, 1, 2)
    live = np.asarray(live_pts, np.float32).reshape(-1, 1, 2)
    if len(ref) < 4 or len(ref) != len(live):
        return PlaneGeometry(ok=False)
    Kd = np.asarray(K, np.float64)
    Hn = _normalised(np.asarray(H, np.float64), Kd)
    if Hn is None:
        return PlaneGeometry(ok=False)
    try:
        _n, _Rs, _Ts, Ns = cv2.decomposeHomographyMat(Hn, Kd)
        keep = cv2.filterHomographyDecompByVisibleRefpoints(_Rs, Ns, ref, live)
    except cv2.error:
        return PlaneGeometry(ok=False)
    if keep is None or len(np.ravel(keep)) == 0:
        return PlaneGeometry(ok=False)
    idxs = [int(i) for i in np.ravel(keep)]
    n = np.asarray(Ns[idxs[0]], np.float64).ravel()
    # NON-FINITE IS A REFUSAL, and this guard is load-bearing rather than
    # defensive. Without it `norm <= 0.0` is FALSE for NaN, `n / NaN` is NaN,
    # and `max(-1, min(1, NaN))` evaluates to 1.0 in CPython -- so `acos` gives
    # 0.0 and a failed decomposition is reported as A PERFECTLY SQUARE BOARD.
    # That is the one wrong answer a firing gate must never be handed.
    norm = float(np.linalg.norm(n))
    if not np.isfinite(n).all() or not np.isfinite(norm) or norm <= 0.0:
        return PlaneGeometry(ok=False)
    n = n / norm

    # The normal's SIGN is ambiguous -- a plane and its flip describe the same
    # surface, and at zero tilt the filter legitimately returns BOTH facing
    # solutions, so which one arrives is not something to rely on. Orienting it
    # toward the camera is what makes the tilt and the yaw/pitch split mean the
    # same thing every frame; without it a flipped normal reads as 180 deg off
    # square on a board that is dead on.
    if float(np.dot(n, _AXIS)) < 0.0:
        n = -n
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(n, _AXIS))))))

    # Split the tilt into the two axes a hull acts on separately: yaw is fixed
    # by strafing around the board, pitch by changing depth. One combined angle
    # would tell the operator something is wrong and not which way to move.
    yaw = math.degrees(math.atan2(n[0], abs(n[2]) if n[2] else 1e-9))
    pitch = math.degrees(math.atan2(n[1], abs(n[2]) if n[2] else 1e-9))
    return PlaneGeometry(ok=True, tilt_deg=tilt, yaw_deg=yaw, pitch_deg=pitch,
                         normal=(float(n[0]), float(n[1]), float(n[2])),
                         ambiguous=len(idxs) > 1)

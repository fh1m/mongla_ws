"""Metric 6-DoF pose of a known-size planar target. The firing solution.

WHAT THIS IS FOR. "The hole is detected and centred" is not the same as "a
torpedo fired now goes through it". A round leaves along the hull's axis, so
what matters is the angle between that axis and the board's normal, and a
centred bbox cannot express it. This returns the correction directly:

    yaw_deg / pitch_deg / roll_deg   how far off-square we are, per axis
    range_m                          how far, IN METRES
    reproj_px                        how well the solution explains the pixels
    ambiguity                        how much to believe any of it

WHY `solvePnP(SOLVEPNP_IPPE)` AND NOT `decomposeHomographyMat`. The homography
route (`anchor/geometry.py`) recovers the plane's orientation up to scale and
was measured COARSE on real footage -- reference-dependent by 4.7 deg median,
17.5 p90, which is unusable for a firing gate. Three things change here:

  * **Metric.** The reference patch has a KNOWN size in metres, so the object
    points are real coordinates and the answer is a distance, not a ratio.
  * **Better conditioned.** IPPE is built for coplanar points, is 50-80x faster
    than OpenCV's default PnP solver, and is more accurate in most cases
    (Collins & Bartoli, IJCV 2014).
  * **The ambiguity becomes MEASURABLE.** This is the important one.

⛔ PLANAR POSE FLIP AMBIGUITY -- the thing that made the homography route
unusable, named. When a planar target's projection is close to affine (small,
distant, or viewed near head-on) there are genuinely TWO poses that explain the
correspondences, related by a flip about the line of sight. Both are correct
answers to the question asked. Picking by minimum reprojection error works only
when the errors actually differ.

Two consequences worth stating because both were learned the expensive way:

  1. **Temporal filtering cannot fix it.** A Kalman filter, a rolling median --
     none of them help, because the error is not zero-mean noise, it is a
     discrete wrong branch. Measured on our own footage: a 31-frame median took
     frame-to-frame swing from 39 deg p90 down to 1.3, and two independent
     references of the same board still disagreed by 17.5 deg p90. It bought
     smoothness and not accuracy, which is the worst outcome -- a confident,
     stable, wrong angle.
  2. **It is WORST head-on**, which is exactly the terminal firing geometry.
     The regime where we need it most is the regime where it is weakest.

⛔ AND THE GATE THAT LOOKS OBVIOUS IS WRONG. Refusing whenever the two branches
fit equally well throws away the answer in exactly the firing geometry.
Measured, true yaw against the two branches at 2.0 m with 0.5 px of noise:

    truth   branch A        branch B        ratio
      0     -0.97 deg       +2.33 deg       0.85   <- "ambiguous"
      2     +2.45           -1.09           0.83   <- "ambiguous"
      5     +5.09           -3.74           0.42
     20    +20.01          -18.73           0.11

The flip is a mirror ABOUT ZERO, so where there is little tilt there is little
to be ambiguous about: at true 0 the branches are "ambiguous" and they disagree
by 3.3 deg, both saying *you are square*. That is a usable answer, and a ratio
gate discards it.

So this reports an INTERVAL rather than refusing. `yaw_spread_deg` /
`pitch_spread_deg` are how far apart the two branches are, and a caller fires
when `off_axis_deg + spread` is inside its own tolerance. The ambiguity becomes
a bounded uncertainty the mission can reason about instead of a coin flip or a
blanket refusal.

The chosen branch is then refined by an iterative (Levenberg-Marquardt) solve
seeded from it -- the virtual-visual-servoing step the model-based-tracking
literature uses to turn an analytic initialisation into a precise pose.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

# Below this ratio of (best reprojection error / second-best) the two branches
# are treated as distinguishable. Measured in `test_anchor_pose.py`.
AMBIGUITY_MAX = 0.55

# A pose whose own reprojection is worse than this is not describing the pixels
# it was fitted to, whatever the ambiguity says.
MAX_REPROJ_PX = 4.0


@dataclass
class TargetPose:
    """Where the target is, and how much of it to believe."""
    ok: bool
    yaw_deg: float = float('nan')     # + = target's normal points RIGHT of us
    pitch_deg: float = float('nan')   # + = normal points DOWN
    roll_deg: float = float('nan')    # in-plane
    range_m: float = float('nan')
    reproj_px: float = float('nan')
    ambiguity: float = float('nan')   # best/second reproj error; 1.0 = hopeless
    # How far apart the two flip branches are. THE number a firing gate reads:
    # the pose could legitimately be either, so this is the width of the answer.
    yaw_spread_deg: float = 0.0
    pitch_spread_deg: float = 0.0
    n_points: int = 0
    reason: str = ''

    @property
    def off_axis_deg(self) -> float:
        """Single number for a gate: total angle off the board's normal."""
        if not self.ok:
            return float('nan')
        return math.degrees(math.acos(max(-1.0, min(1.0,
            math.cos(math.radians(self.yaw_deg))
            * math.cos(math.radians(self.pitch_deg))))))

    def square_within(self, tol_deg: float) -> bool:
        """Is the target square to us to within `tol_deg`, ALLOWING for the
        flip ambiguity? This is what a firing gate should ask.

        Both branches are legitimate answers, so being square only counts if
        the WORSE of them is also inside tolerance. Reading `off_axis_deg`
        alone would fire on the lucky branch.
        """
        if not self.ok:
            return False
        worst = self.off_axis_deg + max(self.yaw_spread_deg,
                                        self.pitch_spread_deg)
        return worst <= float(tol_deg)


def target_pose(ref_pts, live_pts, K, *, width_m: float,
                ref_size_px: Tuple[float, float],
                dist=None,
                ambiguity_max: float = AMBIGUITY_MAX,
                max_reproj_px: float = MAX_REPROJ_PX) -> TargetPose:
    """6-DoF pose of the snapped patch, given how wide it really is.

    `ref_pts` / `live_pts` are the anchor's RANSAC inliers. `ref_size_px` is the
    reference REGION's (width, height) in the same pixel frame, and `width_m` is
    that region's true width in metres -- the one piece of world knowledge this
    needs, and the reason the answer is metric. `K` must be the camera matrix at
    the resolution the points are in.

    Refuses -- with a reason -- rather than returning a pose it cannot defend.
    """
    import cv2
    ref = np.asarray(ref_pts, np.float64).reshape(-1, 2)
    live = np.asarray(live_pts, np.float64).reshape(-1, 2)
    if len(ref) < 4 or len(ref) != len(live):
        return TargetPose(ok=False, n_points=len(ref), reason='too few points')
    w_px, h_px = float(ref_size_px[0]), float(ref_size_px[1])
    if w_px <= 0 or h_px <= 0 or width_m <= 0:
        return TargetPose(ok=False, reason='bad reference size')

    # Reference pixels -> object-plane metres, origin at the patch centre. The
    # patch is planar BY CONSTRUCTION (it is one snapped view), so Z = 0.
    m_per_px = float(width_m) / w_px
    obj = np.zeros((len(ref), 3), np.float64)
    obj[:, 0] = (ref[:, 0] - w_px * 0.5) * m_per_px
    obj[:, 1] = (ref[:, 1] - h_px * 0.5) * m_per_px

    D = np.zeros((1, 5)) if dist is None else np.asarray(dist, np.float64)
    try:
        n, rvecs, tvecs, errs = cv2.solvePnPGeneric(
            obj, live, np.asarray(K, np.float64), D, flags=cv2.SOLVEPNP_IPPE)
    except cv2.error as exc:
        return TargetPose(ok=False, n_points=len(ref),
                          reason=f'solvePnP: {exc.err if hasattr(exc, "err") else exc}')
    if not n:
        return TargetPose(ok=False, n_points=len(ref), reason='no solution')

    e = [float(x) for x in np.asarray(errs).ravel()] or [float('nan')]
    best = 0
    ratio = (e[0] / e[1]) if len(e) > 1 and e[1] > 0 else 0.0
    # Both branches' angles, so the ambiguity can be REPORTED as a width rather
    # than resolved by luck or refused wholesale. See the module docstring.
    angs = [_angles(np.asarray(rvecs[i], np.float64)) for i in range(n)]
    yaw_spread = (abs(angs[0][0] - angs[1][0]) if n > 1 else 0.0)
    pitch_spread = (abs(angs[0][1] - angs[1][1]) if n > 1 else 0.0)
    if e[0] > max_reproj_px:
        return TargetPose(ok=False, n_points=len(ref), reproj_px=e[0],
                          ambiguity=ratio, yaw_spread_deg=yaw_spread,
                          pitch_spread_deg=pitch_spread,
                          reason='reprojection too large')

    rvec, tvec = np.asarray(rvecs[best], np.float64), np.asarray(tvecs[best], np.float64)
    # Refine the analytic solution with an iterative solve seeded from it --
    # IPPE is the initialisation the literature recommends for exactly this.
    try:
        okr, rvec2, tvec2 = cv2.solvePnP(
            obj, live, np.asarray(K, np.float64), D, rvec.copy(), tvec.copy(),
            useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
        if okr:
            proj, _ = cv2.projectPoints(obj, rvec2, tvec2,
                                        np.asarray(K, np.float64), D)
            r2 = float(np.sqrt(np.mean(np.sum(
                (proj.reshape(-1, 2) - live) ** 2, axis=1))))
            # Only keep the refinement if it actually improved the fit. An
            # iterative solver seeded near a flip can walk to the other branch.
            if r2 <= e[0]:
                rvec, tvec, e[0] = rvec2, tvec2, r2
    except cv2.error:
        pass

    yaw, pitch, roll = _angles(rvec)
    rng = float(np.linalg.norm(tvec))
    return TargetPose(ok=True, yaw_deg=yaw, pitch_deg=pitch, roll_deg=roll,
                      range_m=rng, reproj_px=float(e[0]), ambiguity=ratio,
                      yaw_spread_deg=yaw_spread, pitch_spread_deg=pitch_spread,
                      n_points=len(ref))


def _angles(rvec):
    """(yaw, pitch, roll) in degrees from a rotation vector.

    SIGN, and it is verified against a constructed rotation rather than
    reasoned: `yaw_deg` matches a right-handed rotation of the BOARD about the
    camera's vertical axis, so a board turned to show us its left face reads
    positive. Getting this backwards sends the hull the wrong way to square up
    -- the same class of error as the anchor's `findHomography` argument order,
    which is why both are pinned by tests.
    """
    import cv2
    R, _ = cv2.Rodrigues(np.asarray(rvec, np.float64))
    nrm = R[:, 2]
    if nrm[2] > 0:
        nrm = -nrm
    return (math.degrees(math.atan2(-nrm[0], -nrm[2])),
            math.degrees(math.atan2(nrm[1], -nrm[2])),
            math.degrees(math.atan2(R[1, 0], R[0, 0])))

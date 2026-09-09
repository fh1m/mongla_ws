"""Metric 6-DoF pose of a known-size planar target. The firing solution.

WHAT THIS IS FOR. "The hole is detected and centred" is not the same as "a
torpedo fired now goes through it". A round leaves along the hull's axis, so
what matters is the angle between that axis and the board's normal, and a
centred bbox cannot express it. This returns the correction directly:

    yaw_deg / pitch_deg / roll_deg   how far off-square we are, per axis
    range_m                          how far, IN METRES
    reproj_px                        how well the solution explains the pixels
    ambiguity                        how much to believe any of it

⛔ TWO SOLVERS, AND NEITHER REPLACES THE OTHER -- measured on our own data
rather than taken from a citation. IPPE is from 2014; SQPnP (Terzakis &
Lourakis, ECCV 2020) is newer and, on constructed ground truth with our patch
size and our tilt range, better at the tail:

    pixel noise   IPPE p90 yaw   SQPnP p90   IPPE range   SQPnP range
       0.50 px        1.47 deg     0.94 deg      0.8 mm       0.6 mm
       1.55 px        3.38         1.40         2.7 mm       2.1 mm   <- ours
       3.00 px        6.27         3.96         5.4 mm       4.3 mm

At the noise our real matches actually carry, SQPnP's p90 is **2.4x better**,
for 0.098 ms against 0.072. But SQPnP returns ONE solution -- the global
optimum -- and the flip interval is the whole safety story here. IPPE always
returns BOTH branches.

So: **IPPE supplies the interval, SQPnP the point estimate.** Verified that
SQPnP's answer lands inside IPPE's interval at every tilt tested (0/5/10/20/35
deg, 5 of 5), which is what makes combining them legitimate rather than two
numbers stapled together. `SOLVEPNP_ITERATIVE` then refines, seeded from
SQPnP -- it is the best of the four at high noise (p90 2.76 deg at 3 px) and
5x slower, which is affordable once, on one seed, at 3 Hz.

WHY PnP AT ALL AND NOT `decomposeHomographyMat`. The homography
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

    Kd = np.asarray(K, np.float64)

    def rms(rv, tv):
        """OUR reprojection RMS, computed the same way for every candidate.

        `solvePnPGeneric`'s own reportedError is NOT an RMS -- measured, it
        gives 1.3862 where the true RMS is 1.9604 for the same pose. Comparing
        a locally computed RMS against it is comparing two different
        quantities, and it silently made a better solver always lose. The
        reported errors are still used for the ambiguity RATIO, where both
        sides are the same quantity and the comparison is legitimate.
        """
        proj, _ = cv2.projectPoints(obj, rv, tv, Kd, D)
        return float(np.sqrt(np.mean(np.sum(
            (proj.reshape(-1, 2) - live) ** 2, axis=1))))

    rvec, tvec = np.asarray(rvecs[best], np.float64), np.asarray(tvecs[best], np.float64)
    best_rms = rms(rvec, tvec)
    # THE POINT ESTIMATE comes from SQPnP, which is measurably better at the
    # tail than IPPE's branch (see the table above). IPPE's job was the
    # interval, and it has already done it.
    try:
        oks, rs, ts = cv2.solvePnP(obj, live, np.asarray(K, np.float64), D,
                                   flags=cv2.SOLVEPNP_SQPNP)
        if oks:
            # Like for like: both measured with `rms`, never against OpenCV's
            # reportedError. A global optimum of the wrong cost is still wrong,
            # and the reprojection is the only thing here that can tell.
            rs_err = rms(rs, ts)
            if rs_err <= best_rms * 1.02:
                rvec, tvec, best_rms = rs, ts, rs_err
    except cv2.error:
        pass
    # Then one iterative polish from that seed -- best of the four at high
    # noise, and 5x slower, which is affordable once at 3 Hz.
    try:
        okr, rvec2, tvec2 = cv2.solvePnP(
            obj, live, np.asarray(K, np.float64), D, rvec.copy(), tvec.copy(),
            useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
        if okr:
            # Keep the polish only if it improved the fit: an iterative solver
            # seeded near a flip can walk to the other branch.
            r2 = rms(rvec2, tvec2)
            if r2 <= best_rms:
                rvec, tvec, best_rms = rvec2, tvec2, r2
    except cv2.error:
        pass

    yaw, pitch, roll = _angles(rvec)
    rng = float(np.linalg.norm(tvec))

    # ⛔ NON-FINITE IS A REFUSAL -- the same guard `geometry.py` calls
    # load-bearing rather than defensive, applied at the other end of the same
    # pipeline. MEASURED 2026-09-10: a refraction-rectified 0.30 m square at
    # 1.0 m and 0.35 m off-axis comes back from solvePnP as ok with
    # range=nan, reproj=nan, yaw=nan. Every downstream comparison against NaN
    # is silently False, so `range_m < tol` reads as OUT of tolerance while
    # `abs(yaw) < tol` reads as out too -- and a caller that tests the other
    # way round gets "in tolerance" from an answer that does not exist.
    # `lock_node` nan-guards reproj and ambiguity but publishes range raw.
    if not all(math.isfinite(v) for v in (yaw, pitch, roll, rng, best_rms)):
        return TargetPose(ok=False, n_points=len(ref), ambiguity=ratio,
                          yaw_spread_deg=yaw_spread,
                          pitch_spread_deg=pitch_spread,
                          reason='non-finite pose')

    return TargetPose(ok=True, yaw_deg=yaw, pitch_deg=pitch, roll_deg=roll,
                      range_m=rng, reproj_px=float(best_rms), ambiguity=ratio,
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

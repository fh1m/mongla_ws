"""flow_math -- pure functions for downward-camera optical-flow distance.

No ROS, no OpenCV imports here -- just the metric/geometry so it is unit-testable
in isolation (the node in distance_estimation_node.py wires these to LK flow +
topics). The method (given, not reinvented):

    height_m         = pool_depth_m + depth_m          # depth_m < 0 below surface
    flow_rot_px      = f_px * [pitch_rate, roll_rate] * dt
    flow_trans_px    = flow_measured_px - flow_rot_px
    d += project(flow_trans_px, axis_dir) * height_m / f_px

ONE AXIS ONLY: we integrate the flow component along a latched world direction
(axis_yaw). This is deliberately NOT 2D dead reckoning -- that needs an EKF and is
out of scope. Do not generalize the projection to accumulate both components.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np


def height_above_floor(pool_depth_m: float, depth_m: float) -> Optional[float]:
    """Metric camera height above the pool floor, or None if non-physical.

    `depth_m` is NEGATIVE below the surface (this stack's AHRS2 convention), so
    height = pool_depth + depth_m (= pool_depth - |depth|). At 0.5 m deep in a
    4 m pool: 4 + (-0.5) = 3.5 m. Returns None when the result is <= 0 (bad
    depth / bad pool_depth) so the caller holds the last good height instead of
    scaling by a garbage number.
    """
    h = float(pool_depth_m) + float(depth_m)
    return h if h > 1e-3 else None


def rotation_flow_px(f_px: float, pitch_rate: float, roll_rate: float,
                     dt: float) -> Tuple[float, float]:
    """Rotation-induced image shift (px) over `dt` for a downward camera.

    A pitch about the body Y-axis sweeps the ground vertically in the image;
    a roll about the body X-axis sweeps it horizontally. Small-angle:
    shift = f_px * rate * dt. Returns (dx_px, dy_px) = (roll term, pitch term)
    in image axes -- x horizontal, y vertical. The node subtracts this from the
    measured flow BEFORE projecting, so residual is pure translation.

    NOTE the camera<->body axis mapping (which body rate drives which image
    axis, and the signs) is mount-specific and CALIBRATED by the static-tilt
    test; this returns the canonical mapping (image-y<-pitch, image-x<-roll),
    the node applies a configured sign/swap on top.
    """
    dx = f_px * roll_rate  * dt
    dy = f_px * pitch_rate * dt
    return dx, dy


def axis_unit(axis_yaw_rad: float, lateral: bool = False) -> Tuple[float, float]:
    """Unit vector of the projection axis in the image plane.

    `axis_yaw_rad` is the latched world heading at calc_distance('start').
    `lateral=False` -> project along the heading (fore/aft move); True -> along
    heading+90 deg (left/right move). The downward image is assumed aligned so
    image +y = vehicle forward at the latch (the node applies its frame/sign
    config); we return the direction the translational flow is projected onto.
    """
    yaw = axis_yaw_rad + (math.pi / 2.0 if lateral else 0.0)
    # forward (yaw=0) -> +y image; right (yaw=+90) -> +x image.
    return math.sin(yaw), math.cos(yaw)


def project(vec_xy: Tuple[float, float], axis_xy: Tuple[float, float]) -> float:
    """Scalar projection of a 2D vector onto a unit axis (dot product)."""
    return float(vec_xy[0] * axis_xy[0] + vec_xy[1] * axis_xy[1])


def robust_flow(prev_pts: np.ndarray, next_pts: np.ndarray,
                status: np.ndarray, *, min_tracks: int = 6,
                inlier_sigma: float = 2.0) -> Optional[Tuple[float, float]]:
    """Reduce per-point LK displacements to ONE robust (dx, dy) in pixels.

    Rejects outlier tracks (surface glare, turbidity sparkles, mis-tracks) the
    way the tracker's min_hits guards spurious ids: take the component-wise
    MEDIAN, then keep only points within `inlier_sigma` MADs of it and average
    the inliers. Returns None when too few tracks survived (caller re-seeds /
    holds) -- degrades gracefully on a low-texture floor.

    prev_pts / next_pts: (N,1,2) or (N,2) float arrays. status: (N,) or (N,1)
    LK found-flag (1=tracked).
    """
    if prev_pts is None or next_pts is None or status is None:
        return None
    st = np.asarray(status).reshape(-1).astype(bool)
    p0 = np.asarray(prev_pts, dtype=np.float64).reshape(-1, 2)[st]
    p1 = np.asarray(next_pts, dtype=np.float64).reshape(-1, 2)[st]
    if p0.shape[0] < min_tracks:
        return None
    disp = p1 - p0                      # (M, 2) per-track pixel displacement
    med  = np.median(disp, axis=0)      # component-wise median (robust centre)
    # MAD-based inlier gate per component; keep tracks close on BOTH axes.
    mad  = np.median(np.abs(disp - med), axis=0)
    scale = np.where(mad > 1e-6, mad, 1.0)
    keep = np.all(np.abs(disp - med) <= inlier_sigma * scale, axis=1)
    inliers = disp[keep] if keep.any() else disp
    dx, dy = np.mean(inliers, axis=0)
    return float(dx), float(dy)


def interp_rate(buffer: Sequence[Tuple[float, float, float]],
                t: float) -> Optional[Tuple[float, float]]:
    """Interpolate (pitch_rate, roll_rate) to time `t` from a timestamped buffer.

    `buffer` is a time-ordered sequence of (stamp_s, pitch_rate, roll_rate).
    Rotation-comp is ~1:1 with the signal, so "latest sample" is wrong at 20-50
    Hz async -- linearly interpolate to the flow frame-pair midpoint instead.
    Returns None on an empty buffer; clamps to the ends outside the range.
    """
    if not buffer:
        return None
    if t <= buffer[0][0]:
        return buffer[0][1], buffer[0][2]
    if t >= buffer[-1][0]:
        return buffer[-1][1], buffer[-1][2]
    for i in range(1, len(buffer)):
        t1, p1, r1 = buffer[i]
        if t1 >= t:
            t0, p0, r0 = buffer[i - 1]
            span = t1 - t0
            a = 0.0 if span <= 1e-9 else (t - t0) / span
            return p0 + a * (p1 - p0), r0 + a * (r1 - r0)
    return buffer[-1][1], buffer[-1][2]


def detect_corners(gray, *, want: int = 80, max_corners: int = 200,
                   min_distance: int = 8, block: int = 7,
                   quality_ladder=(0.01, 0.004, 0.0015),
                   buckets: int = 4):
    """Corners, spread over a grid, with the quality bar LOWERED until there
    are enough of them.

    ⛔ A FIXED `qualityLevel` IS A FIXED ASSUMPTION ABOUT THE FLOOR, and it is
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

    ⚠ THE LADDER IS A TRADE, MEASURED, NOT A FREE WIN. On this bench, static
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

    ⛔ LK DOES NOT REPORT ITS OWN FAILURES HONESTLY. `status=1` means the
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


class PlanarMotion:
    """Translation at the principal point, image rotation, and scale change."""

    __slots__ = ('dx_px', 'dy_px', 'yaw_rate', 'scale_rate', 'n_inliers',
                 'n_points', 'residual_px', 'ok', 'reason')

    def __init__(self, ok=False, dx_px=0.0, dy_px=0.0, yaw_rate=0.0,
                 scale_rate=0.0, n_inliers=0, n_points=0, residual_px=0.0,
                 reason=''):
        self.ok, self.dx_px, self.dy_px = ok, dx_px, dy_px
        self.yaw_rate, self.scale_rate = yaw_rate, scale_rate
        self.n_inliers, self.n_points = n_inliers, n_points
        self.residual_px, self.reason = residual_px, reason


def solve_planar_motion(prev_pts, next_pts, status, dt: float, *,
                        cx: float, cy: float,
                        min_points: int = 8,
                        ransac_px: float = 2.0) -> PlanarMotion:
    """Fit ONE rigid motion to all the correspondences at once.

    ⛔ WHY THIS BEATS A MEDIAN, and it is not "the papers do it". A camera
    pointed at a plane does not see N independent displacements -- it sees ONE
    body moving, so the flow field is a single similarity transform and every
    point is a measurement of the same four numbers. A component-wise median
    models only the translation and discards the rest:

      * IMAGE ROTATION is thrown away. For a downward camera that IS the
        vehicle's yaw about the optical axis, so the field carries an
        independent yaw rate -- free, and derived from a sensor that fails for
        completely different reasons than the gyro. Cross-checking the two is
        an always-on detector for the axis/sign class of bug that cost this
        session four bench runs, and it needs no extra hardware.
      * SCALE CHANGE is thrown away. Divergence of the field is range rate: a
        second opinion on altitude, against a barometer that a wake or a
        thruster transient can disturb.
      * THE GLOBAL CONSTRAINT is thrown away, and this is the one that matters
        in a pool. A median is a per-point vote, so if half the points lock one
        tile off over a repetitive floor the median follows them. A single
        transform cannot be satisfied by a mixture of correct and one-tile-off
        matches -- they are geometrically inconsistent -- so RANSAC rejects
        them as outliers instead of averaging them in.

    ⚠ RANSAC, NOT `USAC_MAGSAC` -- and this CORRECTS a recorded plan note.
    The note said MAGSAC was "already in OpenCV 4.6 on the Pi, a free upgrade".
    It is, for `findHomography` and `findFundamentalMat`. It is NOT accepted by
    `estimateAffinePartial2D`, which raises "Unknown or unsupported robust
    estimation method"; measured on the vehicle, only RANSAC and LMEDS are
    supported here. The loss is small and worth stating: MAGSAC's advantage is
    insensitivity to the inlier threshold, which matters most for an 8-DOF
    homography fitted to a marginal point set. This model is 4 DOF and is
    over-determined by ~100 points, so the threshold is far less critical.

    A SIMILARITY (4 DOF) IS ALSO THE RIGHT MODEL, not merely the available one.
    A full homography has 8 and would happily spend the extra four on fitting
    noise as a phantom tilt; a camera looking straight down at a flat floor
    genuinely has only translation, rotation and scale to offer.

    ⚠ THE TRANSLATION IS EVALUATED AT THE PRINCIPAL POINT, not read out of the
    matrix. `estimateAffinePartial2D` returns tx/ty about the ORIGIN (the
    top-left corner), so under any rotation those include a lever term of
    `theta * |c|` -- at 640x360 and a 1 deg rotation that is 3.3 px of pure
    fiction added to the translation, silently. Only the motion of the optical
    axis is the translation we want.
    """
    import cv2
    if prev_pts is None or next_pts is None or dt <= 0.0:
        return PlanarMotion(reason='no correspondences')
    st = (np.asarray(status).reshape(-1).astype(bool) if status is not None
          else np.ones(len(np.asarray(prev_pts).reshape(-1, 2)), dtype=bool))
    p0 = np.asarray(prev_pts, dtype=np.float32).reshape(-1, 2)[st]
    p1 = np.asarray(next_pts, dtype=np.float32).reshape(-1, 2)[st]
    n = p0.shape[0]
    if n < min_points:
        return PlanarMotion(n_points=n,
                            reason=f'only {n} tracked points, need {min_points}')

    M, inl = cv2.estimateAffinePartial2D(
        p0, p1, method=cv2.RANSAC, ransacReprojThreshold=ransac_px,
        maxIters=2000, confidence=0.995)
    if M is None:
        return PlanarMotion(n_points=n, reason='no consistent rigid motion')
    inliers = int(inl.sum()) if inl is not None else n
    if inliers < min_points:
        return PlanarMotion(n_points=n, n_inliers=inliers,
                            reason=f'only {inliers}/{n} points agree on one '
                                   f'motion')

    a, b = float(M[0, 0]), float(M[1, 0])
    scale = math.hypot(a, b)
    theta = math.atan2(b, a)

    # Translation OF THE OPTICAL AXIS. See the docstring: reading M[:,2] is a
    # silent lever-arm error under any rotation.
    c = np.array([cx, cy, 1.0])
    moved = M @ c
    dx = float(moved[0] - cx)
    dy = float(moved[1] - cy)

    # Residual of the surviving points: how well one rigid motion explains
    # them. This is the honest quality signal -- it is large when the scene is
    # not planar, when something is moving in view, or when the fit is riding
    # a lattice ambiguity.
    keep = inl.reshape(-1).astype(bool) if inl is not None else slice(None)
    pin, pout = p0[keep], p1[keep]
    pred = (M[:, :2] @ pin.T).T + M[:, 2]
    resid = float(np.median(np.hypot(pred[:, 0] - pout[:, 0],
                                     pred[:, 1] - pout[:, 1])))
    return PlanarMotion(ok=True, dx_px=dx, dy_px=dy,
                        yaw_rate=theta / dt, scale_rate=(scale - 1.0) / dt,
                        n_inliers=inliers, n_points=n, residual_px=resid)


def integrate_rate(buffer: Sequence[Tuple[float, float, float]],
                   t0: float, t1: float) -> Optional[Tuple[float, float]]:
    """Mean (pitch_rate, roll_rate) over [t0, t1], trapezoidally integrated.

    ⛔ WHY NOT `interp_rate` AT THE MIDPOINT. Sampling the middle of an
    interval assumes the rate is linear across it, which is fine over one
    frame period and wrong over an adaptive keyframe baseline: at 2 cm/s the
    baseline stretches to ~0.5 s, and a hull that yaws through half a swing in
    that time has a midpoint rate near its PEAK while its mean is near zero.
    De-rotation subtracts `f * omega * dt`, so the error is proportional to the
    whole baseline -- exactly the regime where it is largest.

    The mean is what the correction actually needs: the total rotational shift
    over the interval is the INTEGRAL of the rate, and `mean * dt` is that
    integral by definition.

    Returns None on an empty buffer; clamps to the ends outside the range.
    """
    if not buffer:
        return None
    if t1 <= t0:
        return interp_rate(buffer, t0)
    inside = [(t, p, r) for (t, p, r) in buffer if t0 <= t <= t1]
    # Anchor both ends so a baseline shorter than the sample interval, or one
    # straddling a gap, still integrates over its true span.
    ends = []
    for t in (t0, t1):
        got = interp_rate(buffer, t)
        if got is not None:
            ends.append((t, got[0], got[1]))
    pts = sorted(set(inside + ends), key=lambda x: x[0])
    if len(pts) < 2:
        return interp_rate(buffer, 0.5 * (t0 + t1))
    area_p = area_r = 0.0
    for (ta, pa, ra), (tb, pb, rb) in zip(pts, pts[1:]):
        dt = tb - ta
        area_p += 0.5 * (pa + pb) * dt
        area_r += 0.5 * (ra + rb) * dt
    span = pts[-1][0] - pts[0][0]
    if span <= 1e-9:
        return pts[0][1], pts[0][2]
    return area_p / span, area_r / span


class DistanceAccumulator:
    """Integrate axis-projected metric displacement between start() and stop().

    IDLE -> ACTIVE on start(axis_yaw, lateral): latch the projection axis + zero
    the accumulator. Each add() folds one frame's translational flow into metres.
    stop() freezes and returns the total. The whole point of the bounded segment
    is to keep dead-reckoning drift acceptable (short integrations only).
    """

    def __init__(self):
        self.active = False
        self.distance_m = 0.0
        self._axis: Tuple[float, float] = (0.0, 1.0)
        self._axis_yaw = 0.0
        self._lateral = False

    def start(self, axis_yaw_rad: float, lateral: bool) -> None:
        self._axis = axis_unit(axis_yaw_rad, lateral)
        self._axis_yaw = float(axis_yaw_rad)
        self._lateral = bool(lateral)
        self.distance_m = 0.0
        self.active = True

    def add(self, flow_trans_px: Tuple[float, float], height_m: float,
            f_px: float) -> None:
        """Fold one frame: project flow onto the latched axis, scale to metres.

        distance += proj(flow_trans_px, axis) * height / f_px. dt already
        cancelled (per-frame pixel displacement, not velocity). No-op when not
        active or on a bad height / f_px.
        """
        if not self.active or height_m is None or f_px <= 1e-6:
            return
        self.distance_m += project(flow_trans_px, self._axis) * height_m / f_px

    def add_body_velocity(self, vx: float, vy: float, yaw_rad: float,
                          dt: float) -> None:
        """Fold one interval of BODY-frame velocity onto the latched axis.

        Prefer this over `add()` when you already have metric velocity. The
        pixel path exists because the old node only ever had pixels; going
        velocity -> pixels -> metres to reuse it means re-deriving an axis
        convention on every call, and the first attempt at exactly that
        SWAPPED THE AXES: `flow_velocity` returns `vx` from the image's y
        component, so the obvious tuple is reversed and the error is a plausible
        number rather than a crash.

        `vx` is body forward, `vy` body right, `yaw_rad` the CURRENT heading.
        The axis was latched at start(), so a hull that yaws mid-move still
        accumulates along the direction it set out on -- which is the whole
        reason the axis is latched rather than taken from the current heading.

        Derivation, so the trig is checkable rather than trusted. Rotating body
        into world and projecting onto the latched direction `a`:

            v_n = vx*cos(y) - vy*sin(y)      v_e = vx*sin(y) + vy*cos(y)
            proj = v_n*cos(a) + v_e*sin(a)
                 = vx*cos(y-a) - vy*sin(y-a)

        so only the heading ERROR since the latch matters, and at e=0 this is
        exactly vx (axial) -- the sanity check worth keeping in mind.
        """
        if not self.active or dt <= 0.0:
            return
        e = float(yaw_rad) - self._axis_yaw
        if self._lateral:
            e -= math.pi / 2.0
        self.distance_m += (vx * math.cos(e) - vy * math.sin(e)) * dt

    def stop(self) -> float:
        self.active = False
        return self.distance_m


def flow_dispersion(prev_pts, next_pts, status) -> Optional[float]:
    """Median distance of each point's displacement from the median one, in px.

    The COHERENCE measure `flow_velocity` gates on. Over a flat surface at
    constant range a translation moves every point by nearly the same vector --
    that is what a translation IS in this geometry -- so this is small for real
    motion and comparable to the motion itself for noise. Measured on a bench
    camera: 0.096-0.127 px while still, 7.7 px on the intervals that produced
    spurious metre-per-second readings.

    Returns None when there is nothing to measure, never 0.0: an absent
    dispersion means "unknown", and 0.0 would read as perfect coherence.
    """
    if prev_pts is None or next_pts is None or status is None:
        return None
    st = np.asarray(status).reshape(-1).astype(bool)
    p0 = np.asarray(prev_pts, dtype=np.float64).reshape(-1, 2)[st]
    p1 = np.asarray(next_pts, dtype=np.float64).reshape(-1, 2)[st]
    if len(p0) < 3:
        return None
    d = p1 - p0
    med = np.median(d, axis=0)
    return float(np.median(np.hypot(d[:, 0] - med[0], d[:, 1] - med[1])))

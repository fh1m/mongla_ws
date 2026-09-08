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
from collections import deque
from typing import Optional, Sequence, Tuple

import numpy as np

from ..optics import N_WATER


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


class RefractiveRectifier:
    """Undo a FLAT PORT, so one focal length is exact instead of a compromise.

    ⛔ THE DEFECT THIS FIXES, and it is invisible in air. `f_water = 741` was
    measured from the in-water FOV and is therefore exact AT THE FRAME EDGE.
    A flat port is not a pinhole: the ray from a point at water angle `tw`
    leaves the port at air angle `ta` with `sin ta = n * sin tw`, and lands at
    `r = f_air * tan(ta)`. The LOCAL effective focal length `r / tan(tw)`
    therefore grows with field angle. Computed for this camera:

        r=0 px    f_eff 685.1     -7.5 % vs the shipped 741
        r=200     f_eff 707.4     -4.5 %
        r=320     f_eff ~741       0.0 %   <- where it was calibrated
        r=367     f_eff 757.7     +2.3 %

    **10.6 % centre-to-corner.** Velocity is `flow_px * h / (f * dt)`, so a
    wrong `f` is a clean multiplier: a centre point scaled by the edge-fitted
    741 reads **7.5 % LOW**, and the size of the error depends on where the
    corners happened to land that interval. That is worse than a fixed bias --
    it is a bias that moves with the texture.

    THE SAME ARITHMETIC ALSO EXPLAINS 1.44 vs THE TEXTBOOK 1.33. The refractive
    index scales SINES; a focal length is about TANGENTS. At a 63.8 deg air
    FOV the tangent ratio is 1.4416 while the sine ratio is 1.333 by
    definition, and our measured water FOV matches Snell's prediction to
    **0.004 deg**. The old note calling the discrepancy "expected -- port
    thickness and geometry" is RETRACTED: it is neither, it is paraxial versus
    wide-angle, and a 1.33 default would have been 7.5 % wrong.

    THE FIX (Luczynski et al., Pinax model, Ocean Eng. 2017): calibrate in AIR
    once, then correct refraction analytically. Map each point to the angle it
    actually came from and re-project it through one chosen focal length:

        x_n = (u - cx) / fx          normalised, so fx != fy is handled
        ta  = atan(|x_n|)            air-side ray angle
        tw  = asin(sin(ta) / n)      Snell, into the water
        x_n' = x_n * tan(tw) / |x_n| rectified: now a TRUE pinhole at f_ref

    After this a single `f_ref` is exact everywhere, and the planar fit runs
    on coordinates where equal ground displacement means equal pixel
    displacement wherever it happens in the frame.

    ⛔ WHAT THIS DOES **NOT** MODEL, stated so it is not assumed away. A flat
    port is strictly an AXIAL camera: rays do not pass through one centre,
    they are displaced by the port glass, and the residual depends on OBJECT
    DISTANCE. This correction is the single-viewpoint limit, valid while the
    object distance greatly exceeds the port offset -- ~0.7-2 m of water
    against ~1 cm of port here, so the neglected term is second order against
    the 10.6 % first-order one it removes. Pinax handles the rest by fixing a
    virtual pinhole at a chosen distance; if the pool numbers still show a
    height-dependent scale, that is the next term, not a mystery.
    """

    __slots__ = ('fx', 'fy', 'cx', 'cy', 'n', 'f_ref', 'f_ref_y')

    def __init__(self, fx: float, fy: float, cx: float, cy: float,
                 n: float = N_WATER, f_ref: Optional[float] = None):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.n = float(n)
        # Default: the PARAXIAL water focal length, f_air * n. Chosen so the
        # frame centre is a fixed point of the rectification -- a point at the
        # principal point does not move, which makes the transform inspectable.
        self.f_ref = float(f_ref) if f_ref else float(fx) * self.n
        # ⛔ THE ASPECT MUST SURVIVE. Writing both output axes through one
        # f_ref silently rescales y by fx/fy -- 513.94/516.93 = 0.9942, a
        # 0.58 % error on the vertical axis. That is precisely the fx!=fy
        # defect round 38 measured as part of a 3.08 % axis asymmetry and
        # fixed; re-introducing it inside the fix for a DIFFERENT axis bug is
        # exactly how a correction becomes a regression. Caught by asserting
        # that n = 1 is the identity, which it is not unless this line exists.
        self.f_ref_y = self.f_ref * (float(fy) / float(fx))

    def rectify(self, pts):
        """Image points -> water-linear points. Same shape in, same shape out."""
        p = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
        xn = (p[:, 0] - self.cx) / self.fx
        yn = (p[:, 1] - self.cy) / self.fy
        rn = np.hypot(xn, yn)                       # = tan(air ray angle)
        ta = np.arctan(rn)
        # sin(ta)/n <= 1 always for n > 1 -- no total internal reflection on
        # this side of the interface, so no clipping is needed for physics.
        tw = np.arcsin(np.sin(ta) / self.n)
        # scale = tan(tw)/tan(ta); the limit at rn -> 0 is 1/n, not 0/0.
        with np.errstate(invalid='ignore', divide='ignore'):
            scale = np.where(rn > 1e-12, np.tan(tw) / np.maximum(rn, 1e-12),
                             1.0 / self.n)
        out = np.empty_like(p)
        out[:, 0] = self.f_ref * xn * scale + self.cx
        out[:, 1] = self.f_ref_y * yn * scale + self.cy
        return out.reshape(np.asarray(pts).shape)

    def local_focal_px(self, r_px: float) -> float:
        """Effective focal length for a point at image radius `r_px`.

        Diagnostic: this is the number a single-`f` model gets wrong, and
        printing it beside the configured `f` is how the error becomes
        visible instead of arriving as an unexplained scale factor.
        """
        if r_px <= 0.0:
            return self.fx * self.n
        ta = math.atan(r_px / self.fx)
        tw = math.asin(min(1.0, math.sin(ta) / self.n))
        return r_px / math.tan(tw)


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


class Intrinsics:
    """fx, fy, cx, cy and distortion, at the resolution actually in use.

    ⛔ THREE THINGS THE FLOW PATH WAS GETTING WRONG BY ASSUMING, and the
    vehicle's own runs are what exposed them. Three real 30 cm slides:

        lateral   30.13 cm   ratio 1.0044
        forward   31.09 cm   ratio 1.0363
        back      31.04 cm   ratio 1.0345

    Forward and back over-read by 3.5 % while lateral over-read by 0.4 %. A
    height error CANNOT do that -- height scales both axes identically -- so
    something axis-dependent was wrong, and there were three candidates, all
    of them real:

    1. ONE FOCAL LENGTH FOR BOTH AXES. Pixels are not square here: fx 513.94,
       fy 516.93 at 640 px. The axis riding image-y reads 0.58 % high.
    2. THE PRINCIPAL POINT ASSUMED TO BE THE FRAME CENTRE. It is (308.7,
       186.5) at 640x360, not (320, 180) -- 11.4 px out in x. The rigid fit
       evaluates its translation AT that point, so under any rotation the
       error is a lever arm about the wrong pivot.
    3. NO UNDISTORTION AT ALL, with k2 = 0.098 and k3 = -0.207. Distortion
       scales displacement by a factor that depends on RADIUS, and a 640x360
       frame is 1.8x wider than tall -- so the horizontal axis samples a
       radial range the vertical one never reaches. That asymmetry is
       structural: it appears as a different scale per axis, which is exactly
       the shape of the error measured.

    Loaded from the same calibration JSON `camera_node` uses, and scaled to
    the working resolution, because a calibration taken at 1280x720 and
    applied at 640x360 without scaling is off by exactly 2x -- silently.
    """

    __slots__ = ('fx', 'fy', 'cx', 'cy', 'dist', 'width', 'height',
                 'applies_to')

    def __init__(self, fx, fy, cx, cy, dist=None, width=0, height=0):
        self.fx, self.fy = float(fx), float(fy)
        self.cx, self.cy = float(cx), float(cy)
        self.dist = None if dist is None else np.asarray(dist,
                                                         dtype=np.float64).reshape(-1)
        self.width, self.height = int(width), int(height)
        self.applies_to = ()      # camera profiles this file describes

    @classmethod
    def from_json(cls, path, width: int, height: int) -> "Intrinsics":
        import json
        d = json.load(open(path))
        k = d.get('camera_matrix') or d.get('K')
        k = np.asarray(k, dtype=np.float64).reshape(3, 3)
        cw = int(d.get('image_width') or width)
        ch = int(d.get('image_height') or height)
        # Scale to the resolution in use. Separately per axis: cropping and
        # scaling are not the same operation and only the caller knows which
        # happened, but a pure resize is by far the common case and getting
        # the factor wrong is a clean 2x error nobody sees.
        sx, sy = width / float(cw), height / float(ch)
        dist = d.get('distortion_coefficients') or d.get('D')
        if dist is not None:
            dist = np.asarray(dist, dtype=np.float64).reshape(-1)
        out = cls(k[0, 0] * sx, k[1, 1] * sy, k[0, 2] * sx, k[1, 2] * sy,
                  dist, width, height)
        # Carry the binding through. It was DISCARDED before, which is why
        # nothing could catch a calibration wired to the wrong camera.
        out.applies_to = tuple(d.get('applies_to') or ())
        return out

    @property
    def K(self):
        return np.array([[self.fx, 0.0, self.cx],
                         [0.0, self.fy, self.cy],
                         [0.0, 0.0, 1.0]], dtype=np.float64)

    def undistort_points(self, pts):
        """Map distorted pixel coordinates to ideal pinhole ones.

        Returns points in the SAME pixel frame (P=K), so everything
        downstream -- the rigid fit, the principal point, the focal lengths --
        keeps working in pixels and only the lens is removed.
        """
        import cv2
        if self.dist is None or pts is None or len(pts) == 0:
            return pts
        p = np.asarray(pts, dtype=np.float32).reshape(-1, 1, 2)
        out = cv2.undistortPoints(p, self.K, self.dist, P=self.K)
        return out.reshape(np.asarray(pts).shape).astype(np.float32)

    def __repr__(self):
        d = 'none' if self.dist is None else np.array2string(
            self.dist, precision=4, separator=',')
        return (f'Intrinsics(fx={self.fx:.2f} fy={self.fy:.2f} '
                f'cx={self.cx:.1f} cy={self.cy:.1f} {self.width}x{self.height} '
                f'dist={d})')


def phase_correlate_motion(prev_gray, cur_gray, dt: float, *,
                           hann=None, with_rotation: bool = True,
                           min_response: float = 0.05) -> "PlanarMotion":
    """Whole-image motion with NO FEATURES. Fourier-Mellin.

    ⛔ WHY THIS EXISTS ALONGSIDE THE FEATURE PATH. Lucas-Kanade needs corners,
    and corners are the first thing a real floor stops providing: measured on
    this camera, a textured surface gave 150 usable points and a plain one
    gave 19, and the image itself is blurry (sharpness 70 against 321 and 1180
    on archived competition footage). Feature-based flow degrades exactly where
    an AUV spends its time -- smooth concrete, silt, flat paint, dim water.

    Phase correlation does not look for anything. It correlates the whole
    frame in the Fourier domain and reads the shift off the response peak, so
    it has nothing to lose when the texture thins out. It is also
    ILLUMINATION-ROBUST BY CONSTRUCTION: the cross-power spectrum is
    normalised by magnitude, so only PHASE survives, and phase is what encodes
    position. A brightness or contrast change moves magnitude, not phase --
    which matters underwater, where a light sweeps and the water column
    attenuates.

    ROTATION AND SCALE COME FROM THE SAME TOOL, via the classic Fourier-Mellin
    construction (Correlation Flow, ICRA 2018, uses the same idea and reports
    robustness to motion blur that feature methods lack):

      * the MAGNITUDE of an FFT is translation-invariant, so it carries only
        rotation and scale;
      * resampling that magnitude into LOG-POLAR coordinates turns a rotation
        into a shift along the angle axis and a scale into a shift along the
        log-radius axis;
      * so a second phase correlation on the log-polar images reads both off
        as translations -- the one thing phase correlation does well.

    Returns the same `PlanarMotion` as `solve_planar_motion`, so the two are
    interchangeable and can be compared on one footing.

    ⚠ ITS FAILURE MODE IS DIFFERENT AND MUST BE GATED DIFFERENTLY. There are no
    inliers to count, so `n_inliers` is meaningless here; the honest quality
    signal is the correlation RESPONSE, which falls when the two frames do not
    share content. It also assumes ONE global motion, so an object moving
    through view biases it rather than being rejected -- where RANSAC on the
    feature path would throw it out.

    ⛔ MEASURED AND NOT SHIPPED ON. Built because the plan called for it as the
    low-texture fallback, then benched against the feature path on this
    camera's own imagery with exact synthetic truth. It lost on every count:

        case                      LK err   LK pts   PHASE err   LK ms  PC ms
        translation 8 px          0.014     171      0.012      12.1   84.8
        translation 3 px          0.008     175      0.013      11.1   79.1
        translation 0.5 px        0.015     177      0.057      11.1   78.4
        trans 8 px + rot 2 deg    0.019     165      6.029      11.8   82.1
        pure rotation 2 deg       0.007     168      6.001      11.7   78.4
        blur+noise, trans 8 px    0.011     182      0.021        -      -
        blur+noise, trans 3 px    0.005     192      0.032        -      -

    Seven times slower, and it BREAKS UNDER ROTATION: translational phase
    correlation has no rotation term, so 2 degrees puts 6 px into the
    translation -- the same phantom-travel failure the median has, from a
    different cause. The log-polar stage recovers the rotation but does not
    correct the translation estimate for it.

    And the premise was wrong. It was built for a "low texture" regime that
    this camera does not have: the feature path returns 165-192 inliers on the
    same frames, INCLUDING deliberately blurred and noised ones. An earlier
    reading of 9 points was a transient, not the steady state, and the real fix
    was the LK window (21 -> 31) and the round-trip threshold (1 -> 2 px).

    KEPT, DEFAULT OFF, because the argument for it remains sound where it is
    actually true -- a frame yielding under 8 corners cannot support a 4-DOF
    fit, and this needs none. It should be reached for on evidence of that
    regime, never on the assumption of it.
    """
    import cv2
    if prev_gray is None or cur_gray is None or dt <= 0.0:
        return PlanarMotion(reason='no frames')
    a = np.float32(prev_gray)
    b = np.float32(cur_gray)
    if a.shape != b.shape:
        return PlanarMotion(reason='frame size changed')
    if hann is None:
        # Without a window the frame edges are a step discontinuity and the
        # FFT reads them as strong structure, which anchors the peak at zero.
        hann = cv2.createHanningWindow((a.shape[1], a.shape[0]), cv2.CV_32F)
    (sx, sy), response = cv2.phaseCorrelate(a, b, hann)
    if response < min_response:
        return PlanarMotion(reason=f'weak correlation peak ({response:.3f})')

    yaw_rate = scale_rate = 0.0
    if with_rotation:
        try:
            yaw_rate, scale_rate = _log_polar_rotation(a, b, dt)
        except Exception:
            yaw_rate = scale_rate = 0.0

    return PlanarMotion(ok=True, dx_px=float(sx), dy_px=float(sy),
                        yaw_rate=yaw_rate, scale_rate=scale_rate,
                        n_inliers=0, n_points=0,
                        residual_px=float(1.0 - response),
                        reason='phase')


def _log_polar_rotation(a, b, dt: float):
    """Rotation and scale from log-polar phase correlation of FFT magnitudes."""
    import cv2

    def spectrum(img):
        f = np.fft.fftshift(np.abs(np.fft.fft2(img)))
        # Log compresses the enormous DC-to-high-frequency range so the peak
        # is not decided by the DC term alone.
        return np.log1p(f).astype(np.float32)

    sa, sb = spectrum(a), spectrum(b)
    h, w = sa.shape
    centre = (w / 2.0, h / 2.0)
    m = w / math.log(max(w / 2.0, 2.0))
    flags = cv2.INTER_LINEAR + cv2.WARP_FILL_OUTLIERS + cv2.WARP_POLAR_LOG
    pa = cv2.warpPolar(sa, (w, h), centre, w / 2.0, flags)
    pb = cv2.warpPolar(sb, (w, h), centre, w / 2.0, flags)
    hann = cv2.createHanningWindow((w, h), cv2.CV_32F)
    (dlog, dang), _resp = cv2.phaseCorrelate(pa, pb, hann)
    # angle axis spans 360 deg over `h` rows; the FFT magnitude of a real
    # image is symmetric, so the recoverable range is 180 deg.
    theta = -(dang / h) * 2.0 * math.pi
    if theta > math.pi:
        theta -= 2.0 * math.pi
    elif theta < -math.pi:
        theta += 2.0 * math.pi
    scale = math.exp(dlog / m) if m > 0 else 1.0
    return theta / dt, (scale - 1.0) / dt


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


class HeightFromDivergence:
    """Height above the floor from the image, WITHOUT a hand-typed pool depth.

    ⛔ THE ERROR THIS ATTACKS IS THE BIGGEST ONE WE HAVE. Flow velocity is
    `v = h * flow / (f * dt)`, so height is a CLEAN MULTIPLIER on every number
    the sensor produces. Today it comes from `pool_depth_m - |depth|`, and
    `pool_depth_m` is typed in by a human who paced out a pool. A 7 % error
    there is a 7 % error in every distance, invisible in every plot, and
    integrated by everything downstream. The whole reason `flow_node` REFUSES
    without it is that there is no safe default.

    THE OBSERVATION NOBODY WAS USING. The planar fit already returns
    `scale_rate` -- the divergence of the flow field, which for a downward
    camera is the rate the floor grows in the image. Geometrically:

        scale_rate = -vz / h            (descending -> closer -> scale grows)

    and the barometer measures `vz` directly and independently. So:

        h = -vz / scale_rate

    That is a height measured by the CAMERA, cross-checked against a
    barometer, with no pool depth in it at all. Both quantities are already
    computed and one of them was being discarded.

    ⚠ IT IS ONLY OBSERVABLE WHILE THE VEHICLE CHANGES DEPTH. In level flight
    `scale_rate` is zero and this says nothing -- dividing by it would turn
    noise into a confident height. So it gates hard on both terms being
    meaningfully non-zero, accumulates a robust median rather than trusting any
    single estimate, and reports how many samples it is standing on. A descent
    at the start of a dive is enough to calibrate the height for the whole run,
    which is exactly when an AUV descends anyway.

    This does not replace the barometer path; it CHECKS it. A disagreement
    means the typed pool depth is wrong, which is the failure that would
    otherwise be discovered as a scale error in the mission.
    """

    __slots__ = ('_samples', '_min_scale_rate', '_min_vz', '_max_h', '_n_gated')

    def __init__(self, *, min_scale_rate: float = 0.02, min_vz: float = 0.03,
                 max_h: float = 12.0, keep: int = 64):
        self._samples = deque(maxlen=int(keep))
        self._min_scale_rate = float(min_scale_rate)
        self._min_vz = float(min_vz)
        self._max_h = float(max_h)
        self._n_gated = 0

    def add(self, scale_rate: float, vz_ms: float) -> Optional[float]:
        """One (divergence, vertical speed) pair. Returns the height it implies.

        `vz_ms` is POSITIVE DOWNWARD (descending), matching the sign convention
        that makes a descent grow the image.
        """
        if not (math.isfinite(scale_rate) and math.isfinite(vz_ms)):
            return None
        # Both terms must be real motion. Near zero this is 0/0 and the answer
        # is noise wearing a number -- the same trap as a rotation fraction
        # computed on a motionless bench.
        if abs(scale_rate) < self._min_scale_rate or abs(vz_ms) < self._min_vz:
            self._n_gated += 1
            return None
        h = vz_ms / scale_rate
        if not (0.05 < h < self._max_h):
            self._n_gated += 1
            return None
        self._samples.append(h)
        return h

    @property
    def height_m(self) -> Optional[float]:
        """Robust height, or None until there is enough evidence."""
        if len(self._samples) < 5:
            return None
        return float(np.median(self._samples))

    @property
    def n_samples(self) -> int:
        return len(self._samples)

    def disagreement(self, height_m: Optional[float]) -> Optional[float]:
        """How wrong the OTHER height is, as a fraction of the measured one.

        The denominator is the measured height deliberately. This is asked as
        "how far off is the typed pool depth", and the honest reference for
        that is the quantity that was measured rather than the quantity under
        suspicion -- dividing by the suspect value flatters a large error and
        exaggerates a small one. Returns None when there is no opinion yet:
        absence of a measurement is not agreement.
        """
        mine = self.height_m
        if mine is None or height_m is None or mine <= 0.0:
            return None
        return abs(mine - height_m) / mine


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

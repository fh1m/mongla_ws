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

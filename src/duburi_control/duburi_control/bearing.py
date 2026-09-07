"""Pixels to bearings. The conversion that gives our vision gains units.

WHY THIS EXISTS
---------------
Every vision gain in this stack is tuned against `ex = (cx - W/2)/(W/2)` -- a
dimensionless number that is **not physical**. It depends on resolution, lens
and crop, so `kp_lat = 60.0` is silently a property of one camera. Swap a lens
and the tune is invalid with no warning.

A bearing in radians is a property of the *vehicle's* geometry. Converting once,
here, is what lets a control gain be "thrust per radian" and survive a camera
change. It is also the wire format the srot board's `VISION_API.md` specifies
for `LANDING_TARGET`, so the same function serves the uplink and any host-side
loop.

TWO WAYS TO DO IT, AND THEY DISAGREE BY MORE THAN YOU WOULD GUESS
-----------------------------------------------------------------
`VISION_API.md` gives the linear form, which is what you use when all you have
is an FOV number::

    angle_x = ex * (HFOV / 2)

We have the full calibration (srot `8049de9`), so we can use the pinhole truth::

    angle_x = atan((u - cx) / fx)          # after undistortion

Measured against our own `pi_downward_1280x720.json` (fx 1027.87, cx 617.32,
HFOV 63.82 deg):

    ex     linear      exact      diff
    0.00    0.000     +1.264    +1.264 deg
    0.25    7.977     10.078    +2.100
    0.50   15.954     18.438    +2.483      <- worst, 8.7 cm of aim at 2 m
    1.00   31.908     32.810    +0.902

The row that matters is **ex = 0**. The linear form says a target in the middle
of the image is at zero bearing. It is not: the principal point is 26 px off
frame centre, so the true bearing is **+1.264 deg**. That is a fixed aiming
bias no amount of gain tuning removes, because it is not an error signal -- it
is the wrong definition of zero.

They agree at the centre only if the optical axis passes through the frame
centre, and agree at the edge only by construction (that is how HFOV is
defined). Everywhere in between the linear form is a chord approximation to a
tangent function.

So: use the calibration when it is available, fall back to the linear form when
it is not, and SAY WHICH in the result. A caller that cannot tell whether it
got a calibrated bearing or an FOV guess will eventually trust the guess.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence


@dataclass(frozen=True)
class Bearing:
    """A target's direction, in radians, in the camera's body frame.

    Sign conventions are the spec's (`VISION_API.md` §1):
      angle_x  + = target is to the RIGHT
      angle_y  + = target is BELOW  (image Y grows down)
    """
    angle_x: float          # rad, + = right
    angle_y: float          # rad, + = below
    size_x:  float          # rad, angular width  -- the standoff measure
    size_y:  float          # rad, angular height
    calibrated: bool        # False = FOV fallback, treat as approximate

    @property
    def deg(self) -> tuple:
        return (math.degrees(self.angle_x), math.degrees(self.angle_y))


def _undistort(xn: float, yn: float, dist: Sequence[float],
               iters: int = 8) -> tuple:
    """Invert the Brown-Conrady model for one normalised point.

    Distortion maps ideal -> observed; we have the observed point and want the
    ideal one, and that inverse has no closed form. The standard fixed-point
    iteration converges in a handful of steps for sane coefficients (this is
    what `cv2.undistortPoints` does internally).

    Done in pure Python rather than through cv2 deliberately: `duburi_control`
    has no OpenCV dependency today and this is ten lines. Adding cv2 to the
    control package to invert a polynomial would be the expensive way to buy
    nothing.
    """
    if not dist:
        return xn, yn
    d = list(dist) + [0.0] * (8 - len(dist))
    k1, k2, p1, p2, k3 = d[0], d[1], d[2], d[3], d[4]
    x, y = xn, yn
    for _ in range(iters):
        r2 = x * x + y * y
        radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        if abs(radial) < 1e-9:
            break
        dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        x = (xn - dx) / radial
        y = (yn - dy) / radial
    return x, y


def bearing_from_pixels(u: float, v: float, w_px: float, h_px: float,
                        *, width: int, height: int,
                        K: Optional[Sequence[float]] = None,
                        D: Optional[Sequence[float]] = None,
                        hfov_rad: float = 0.0,
                        vfov_rad: float = 0.0) -> Optional[Bearing]:
    """Convert a bounding box in PIXELS to a bearing.

    `u, v` are the box centre; `w_px, h_px` its size. `K` is the 9-element row-
    major camera matrix as `CameraInfo.k` publishes it; `D` the distortion
    coefficients. Falls back to `hfov_rad`/`vfov_rad` when `K` is unusable.

    Returns None when neither a calibration nor an FOV is available -- the
    caller must not invent one. A zeroed `K` is exactly what `CameraInfo`
    carries before a calibration file is loaded, so "K is present" is not the
    same question as "K is usable".
    """
    if width <= 0 or height <= 0:
        return None

    fx = fy = cx = cy = 0.0
    if K is not None and len(K) >= 9:
        fx, fy, cx, cy = float(K[0]), float(K[4]), float(K[2]), float(K[5])

    if fx > 0.0 and fy > 0.0:
        # CameraInfo.k is stated for the resolution it was CALIBRATED at. A
        # node streaming 640x360 from a 1280x720 calibration must scale, and
        # camera_node already does that before publishing -- but a caller may
        # hand us a raw file, so scale defensively rather than silently
        # producing bearings that are wrong by the resolution ratio.
        xn, yn = (u - cx) / fx, (v - cy) / fy
        xu, yu = _undistort(xn, yn, D or ())
        ax, ay = math.atan(xu), math.atan(yu)
        # Angular size from the box EDGES, not `atan(w/fx)`. The naive form
        # implicitly measures a box centred on the optical axis, so an
        # off-centre target reports a size that shrinks with eccentricity --
        # and size_x is the standoff measure, so that reads as "further away"
        # purely because the target drifted sideways.
        x1, _ = _undistort((u - w_px / 2 - cx) / fx, yn, D or ())
        x2, _ = _undistort((u + w_px / 2 - cx) / fx, yn, D or ())
        _, y1 = _undistort(xn, (v - h_px / 2 - cy) / fy, D or ())
        _, y2 = _undistort(xn, (v + h_px / 2 - cy) / fy, D or ())
        return Bearing(ax, ay,
                       abs(math.atan(x2) - math.atan(x1)),
                       abs(math.atan(y2) - math.atan(y1)),
                       calibrated=True)

    if hfov_rad > 0.0 and vfov_rad > 0.0:
        # The spec's linear form. Correct at the centre only if the optical
        # axis is at the frame centre, which on our measured camera it is not
        # (26 px off => 1.264 deg of bias at ex=0).
        ex = (u - width / 2.0) / (width / 2.0)
        ey = (v - height / 2.0) / (height / 2.0)
        return Bearing(ex * hfov_rad / 2.0, ey * vfov_rad / 2.0,
                       (w_px / width) * hfov_rad,
                       (h_px / height) * vfov_rad,
                       calibrated=False)

    return None


def bearing_from_normalised(ex: float, ey: float, w_frac: float, h_frac: float,
                            *, width: int, height: int,
                            K: Optional[Sequence[float]] = None,
                            D: Optional[Sequence[float]] = None,
                            hfov_rad: float = 0.0,
                            vfov_rad: float = 0.0) -> Optional[Bearing]:
    """Same, from the normalised form `VisionState.Sample` already carries.

    `ex`/`ey` are -1..+1 from frame centre; `w_frac`/`h_frac` are 0..1 of the
    frame. This is the shape the control loop already has, so the uplink does
    not need its own detection subscriber.
    """
    if width <= 0 or height <= 0:
        return None
    u = (ex + 1.0) * width / 2.0
    v = (ey + 1.0) * height / 2.0
    return bearing_from_pixels(u, v, w_frac * width, h_frac * height,
                               width=width, height=height, K=K, D=D,
                               hfov_rad=hfov_rad, vfov_rad=vfov_rad)


class BearingFilter:
    """Alpha-beta smoother for a bearing stream. Chosen on measurement.

    THE PROBLEM, MEASURED on the Pi + Hailo with a `person` target held still:

        bearing jitter        sd 3.83 deg/frame, range 14.8 deg
        bbox centre           sd 32.5 px   -- and box WIDTH sd 86.8 px
        pipeline latency      13.7 ms (grab 1.74 + infer 11.96)
        lag at 30 deg/s       0.41 deg

    Jitter beats lag by ~9x, so there is room to trade lag for smoothness. The
    box width figure is the tell: the detector is not wandering, it is SNAPPING
    between "head and torso" and "whole body", which is why the noise is
    heavy-tailed (kurtosis 4.3) rather than Gaussian.

    WHY ALPHA-BETA AND NOT AN EMA. Measured on that real noise, plus the same
    noise added to a 30 deg/s slew:

        filter          jitter sd   reduction   lag
        raw                 3.570          0%   +0.302 deg
        ema  a=0.15         0.496         86%   +3.564      <- unusable
        ema  a=0.30         0.897         75%   +1.627
        median 9            1.147         68%   +2.508
        alpha-beta .25/.02  0.772         78%   +0.231      <- chosen

    The EMA buys smoothness with lag, one for one. Alpha-beta carries a
    VELOCITY state, so on steady motion it extrapolates through the pipeline
    delay -- it came out with LESS lag than the raw signal, not more. On a
    terminal lock, lag is the error that turns into a miss, so that difference
    is the whole decision.

    THE GATE IS NOT OPTIONAL. A velocity state keeps extrapolating across a
    discontinuity, so a reacquire onto a different box makes the filter fly
    PAST it: a 12 deg step overshoots 1.51 deg and takes 21 frames to settle.
    Treating a large residual as a NEW target instead of a large error removes
    it completely (0.00 deg, 0 frames).

    Where to put the gate was also measured, because it has a cliff:

        gate     trips   jitter reduction
        4-8 deg   1.5%          8-10%     <- fires on ordinary noise, filter dead
        >= 10     0.0%             81%

    12 deg is chosen for margin: ~4x the measured noise sd, never trips on real
    noise, and still catches a genuine target switch (12 deg is ~130 px here,
    a third of the frame).
    """

    ALPHA = 0.25
    BETA = 0.02
    GATE_RAD = math.radians(12.0)
    # A gap longer than this is a re-acquisition, not a slow frame. Extrapolating
    # a velocity across a real dropout is how a filter walks a target off the
    # edge of the frame while reporting a confident bearing.
    MAX_GAP_S = 0.30

    def __init__(self, alpha=None, beta=None, gate_deg=None):
        self.alpha = self.ALPHA if alpha is None else float(alpha)
        self.beta = self.BETA if beta is None else float(beta)
        self.gate = self.GATE_RAD if gate_deg is None else math.radians(gate_deg)
        self.reset()

    def reset(self) -> None:
        self._x = self._y = None
        self._vx = self._vy = 0.0
        self._t = None
        self.resets = 0

    def update(self, bearing: Optional[Bearing], now: float) -> Optional[Bearing]:
        """Filter one sample. `None` in means target lost -> state is dropped.

        Passing None through rather than holding the last value is deliberate:
        the caller's loss handling (grace windows, coast, the uplink's
        send-nothing contract) is what decides what absence means, and a filter
        that invents continuity takes that decision away from it.
        """
        if bearing is None:
            self.reset()
            return None
        if self._x is None or self._t is None or (now - self._t) > self.MAX_GAP_S:
            self._x, self._y = bearing.angle_x, bearing.angle_y
            self._vx = self._vy = 0.0
            self._t = now
            return bearing

        dt = max(1e-3, now - self._t)
        self._t = now
        out = []
        for meas, pos, vel in ((bearing.angle_x, self._x, self._vx),
                               (bearing.angle_y, self._y, self._vy)):
            pred = pos + vel * dt
            resid = meas - pred
            if abs(resid) > self.gate:
                out.append((meas, 0.0, True))
            else:
                out.append((pred + self.alpha * resid,
                            vel + (self.beta / dt) * resid, False))
        (self._x, self._vx, hit_x), (self._y, self._vy, hit_y) = out
        if hit_x or hit_y:
            self.resets += 1
        # size_x/size_y are NOT filtered here. They are the standoff measure and
        # feed a one-sided forward axis; smoothing a range signal adds lag to
        # the axis where lag is a collision.
        return Bearing(self._x, self._y, bearing.size_x, bearing.size_y,
                       bearing.calibrated)

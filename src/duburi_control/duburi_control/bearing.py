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

Measured against our own `pi_forward_1280x720.json` (fx 1027.87, cx 617.32,
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

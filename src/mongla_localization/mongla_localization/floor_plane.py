"""Range to a point on a KNOWN horizontal plane, from one pixel. No object size.

⛔ WHAT THIS ADDS. `range_to` reads distance off a box WIDTH, so it needs the
prop's real width, a box that is not cut by the frame edge, and a detector
that draws the box tight. Lose any of those -- a drum half out of frame, a pipe
whose width the model never learned, a partly occluded base -- and there is no
range at all. But the pool hands us a second, size-free relation: things STAND
ON THE FLOOR (drums, flares, the slalom feet) or HANG FROM A KNOWN DEPTH (the
RoboSub gate bar). A ray through the pixel where the object meets that plane
hits it at exactly one point, and the depth sensor already knows how far above
the plane we are. SYNTHESIS item 7; BumblebeeAS intersect corner rays with a
known plane for the same reason.

⛔ WHEN IT IS WORSE THAN `range_to`, written down so nobody assumes otherwise.
The ground range is `h / tan(theta)`, theta the ray's angle below horizontal,
so `dR/dtheta = h / sin(theta)^2`. At h = 1 m and R = 3 m, ONE DEGREE of
unmodelled pitch moves the answer 17 cm; a 0.30 m drum at the same range read
by width is ~4 cm per box pixel. So for a known-size prop fully in frame, width
wins at range. This wins when width is unavailable, when the target is close
and steep (sigma shrinks as theta grows), and -- the reason it exists -- when
the size-based method returns nothing. Both sigmas are derived, so a caller
can take the smaller.

⛔ THE PLANE IS A CLAIM ABOUT THE POOL. The SAUVC floor SLOPES, 1.6 m at the
centre and 1.2 m at the ends, so a flat-floor assumption is up to 0.4 m wrong
in `h`, which scales the range by the same fraction (R * dh / h). Pass the
depth of the floor WHERE THE TARGET IS, and a `plane_sigma_m` that says how
well you know it.

⛔ OPTICS. Pixels must be in the same optics as `fx, fy, cx, cy`: water focal
length, and rectified pixels with the rectified K (the 1/n scale trap
`pnp_node` documents). Roll is NOT modelled: srot holds roll near zero
(measured -0.81 deg on the bench); at 5 deg the lateral error is ~9 % of range.

Frames. Camera optical: x right, y down, z out of the lens. Hull: X forward,
Y right, Z down. `pitch_deg` is the optical axis's angle BELOW horizontal --
0 for a level forward camera, 90 for a downward camera, where the top of the
image faces hull-forward (a mount rotated from that must remap its pixels
first). Pure math: no ROS, no cv2.
"""

from __future__ import annotations

import math
from typing import NamedTuple, Optional

# Below this angle the ray is nearly parallel to the plane and `h / sin^2`
# makes the answer meaningless: at 5 deg one degree of pitch is 131 % of h per
# degree. A refusal, not a clamp -- a clamped range is a plausible wrong number.
MIN_GRAZING_DEG = 5.0


class PlanePoint(NamedTuple):
    forward_m: float        # hull X from the camera to the point
    right_m: float          # hull Y
    range_m: float          # horizontal distance, hypot(forward, right)
    sigma_m: float          # 1-sigma on range_m from pixel, pitch and plane error
    grazing_deg: float      # ray angle to the plane; small = unreliable


def intersect(u: float, v: float, *, fx: float, fy: float, cx: float, cy: float,
              plane_below_m: float, pitch_deg: float = 0.0,
              pixel_sigma: float = 1.0, pitch_sigma_deg: float = 1.0,
              plane_sigma_m: float = 0.0) -> Optional[PlanePoint]:
    """Where the ray through pixel (u, v) meets a horizontal plane.

    `plane_below_m` is the vertical distance from the camera DOWN to the
    plane: positive for the floor, negative for a plane above the camera (a
    surface-hung gate bar). None when the ray never reaches it -- pointing away
    from it, or within MIN_GRAZING_DEG of parallel.
    """
    if fx <= 0.0 or fy <= 0.0 or plane_below_m == 0.0:
        return None
    xc = (float(u) - cx) / fx
    yc = (float(v) - cy) / fy
    p = math.radians(pitch_deg)
    # Camera-optical ray rotated into the hull frame by the mount pitch.
    fwd = math.cos(p) - yc * math.sin(p)
    right = xc
    down = math.sin(p) + yc * math.cos(p)
    t = plane_below_m / down if down != 0.0 else -1.0
    if t <= 0.0:
        return None
    horiz = math.hypot(fwd, right)
    grazing = math.degrees(math.atan2(abs(down), horiz))
    if grazing < MIN_GRAZING_DEG:
        return None
    rng = t * horiz
    h = abs(plane_below_m)
    s2 = math.sin(math.radians(grazing)) ** 2
    # dR/dtheta = h / sin^2(theta); a pixel is ~1/fy rad near the axis.
    ang_sigma = math.hypot(pixel_sigma / fy, math.radians(pitch_sigma_deg))
    sigma = math.hypot(h / s2 * ang_sigma, rng / h * plane_sigma_m)
    return PlanePoint(t * fwd, t * right, rng, sigma, grazing)

"""Where a prop is, from several sightings of it taken from where we think we are.

The inverse of `fix_from_prop`. That verb trusts the prop and places the hull;
this one trusts the hull and places the prop. A comparable one-person vehicle
lost a scoring drop because its course file put a prop one spacing away from
where the prop actually stood. Its filter refined prop positions online. Ours
took every position from the course file and could only refuse an unset one.

The error that does not average away. Each sighting is `hull + range x
direction`. The range and the bearing noise are fresh on every frame, so they
shrink with the number of sightings. The hull's own position error is NOT
fresh: every sighting in one pass is taken from the same drifted estimate, and
the heading bias is the same for all of them. Averaging eight sightings divides
the first kind by eight and leaves the second exactly where it was. A placement
that divides both reports an ellipse several times too small and is confidently
wrong. `test_prop_placement` measures that: the naive ellipse contains the
truth well under 60 % of the time, and this one about 95 %.

The same fact is what makes the result useful for SPACING. Two props placed
from nearly the same pose share that pose's error, so the distance BETWEEN them
is much better than either position -- and spacing is what the failure above
got wrong.

Pure numpy. No ROS.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple

import numpy as np

# 2-degree-of-freedom chi-square quantiles: -2 ln(1 - p), exact for 2 dof.
CHI2_2DOF_95 = 5.991464547107979
CHI2_2DOF_99 = 9.210340371976182

# Three, because a sample covariance from two points is rank one: it can
# describe a spread along one line and says nothing across it.
MIN_SIGHTINGS = 3

_NAN2 = ((math.nan, math.nan), (math.nan, math.nan))


@dataclass(frozen=True)
class Sighting:
    """One frame: where the hull thought it was, and the prop's range and bearing."""
    hull_xy: Tuple[float, float]   # pool frame, metres
    range_m: float
    range_sigma_m: float
    bearing_deg: float             # compass, same convention as course_map


@dataclass(frozen=True)
class Placement:
    ok: bool
    x_m: float = math.nan
    y_m: float = math.nan
    cov: tuple = _NAN2             # 2x2: what to believe about (x_m, y_m)
    cov_noise: tuple = _NAN2       # 2x2: ONE sighting's fresh noise
    n: int = 0
    prior_offset_m: float = math.nan
    prior_disagrees: bool = False  # outside the 99 % ellipse of the placement
    reason: str = ''
    extra: dict = field(default_factory=dict)

    @property
    def sigma_m(self) -> float:
        """Root of the larger eigenvalue: the worst-axis one-sigma, in metres."""
        return math.sqrt(max(np.linalg.eigvalsh(np.asarray(self.cov))))


def place(sightings: Sequence[Sighting], *, hull_cov_xy, yaw_sigma_deg: float,
          prior_xy: Optional[Tuple[float, float]] = None) -> Placement:
    """Fuse sightings of ONE prop into a position with an honest covariance.

    `hull_cov_xy` is the filter's own 2x2 position covariance and
    `yaw_sigma_deg` its heading sigma, both at the time of the pass; they enter
    ONCE, because they are common to every sighting. `prior_xy`, when given, is
    the course file's position, checked against the result.
    """
    good = [s for s in sightings
            if all(math.isfinite(float(v)) for v in
                   (*s.hull_xy, s.range_m, s.range_sigma_m, s.bearing_deg))
            and s.range_m > 0.0]
    n = len(good)
    if n < MIN_SIGHTINGS:
        return Placement(False, n=n, reason=(
            f'{n} usable sightings; {MIN_SIGHTINGS} are needed before a spread '
            f'can be measured in both axes'))
    u = np.array([[math.cos(math.radians(s.bearing_deg)),
                   math.sin(math.radians(s.bearing_deg))] for s in good])
    r = np.array([s.range_m for s in good])
    pts = np.array([s.hull_xy for s in good], dtype=float) + r[:, None] * u
    est = pts.mean(axis=0)

    # Fresh noise of ONE sighting: the spread actually seen, plus the range
    # sigma the pinhole model derives along each line of sight. Adding both
    # counts range noise roughly twice -- conservative on purpose, because the
    # sample spread of a handful of points underestimates as often as not.
    spread = np.cov(pts.T, ddof=1)
    along = np.mean([s.range_sigma_m ** 2 * np.outer(ui, ui)
                     for s, ui in zip(good, u)], axis=0)
    noise = spread + along

    # Common to the whole pass: the hull's position and its heading. A heading
    # error swings the prop sideways by range x angle.
    mean_dir = u.mean(axis=0)
    mean_dir /= max(np.linalg.norm(mean_dir), 1e-12)
    perp = np.array([-mean_dir[1], mean_dir[0]])
    r_bar = float(r.mean())
    yaw_var = (r_bar * math.radians(float(yaw_sigma_deg))) ** 2
    common = np.asarray(hull_cov_xy, dtype=float) + yaw_var * np.outer(perp, perp)

    cov = noise / n + common
    offset = math.nan
    disagrees = False
    if prior_xy is not None:
        d = np.subtract(est, np.asarray(prior_xy, dtype=float))
        offset = float(np.hypot(*d))
        disagrees = float(d @ np.linalg.solve(cov, d)) > CHI2_2DOF_99
    return Placement(True, x_m=float(est[0]), y_m=float(est[1]),
                     cov=tuple(map(tuple, cov)), cov_noise=tuple(map(tuple, noise)),
                     n=n, prior_offset_m=offset, prior_disagrees=disagrees)

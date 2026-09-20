"""Absolute position from bearings to props whose positions we know.

⛔ THE INVERSION THAT MAKES THIS WORTH BUILDING. A team that could not localise
underwater fixed it by putting extra obstacles in their pool, and wrote the
method off because you cannot do that at a venue. That has it backwards: a
COMPETITION COURSE IS THE DENSEST LANDMARK FIELD WE WILL EVER SEE. Gate,
slalom, bins, torpedo board, octagon -- every one at a surveyed position, with
the rulebook fixing its size, and a detector that already finds them. The thing
they had to fake is the thing the venue hands us.

So this is not SLAM. There is no map to build and no loop to close: the map is
the rulebook plus a tape measure, and the only unknown is where WE are.
Surveyors solved that in 1615.

TWO REGIMES, AND THE DIFFERENCE IS THE HEADING.

  * **Heading known** (Tier 3.2 anchored it on a landmark): each sighting puts
    us on a LINE through that prop, and two lines cross. Well conditioned
    unless the props are nearly in line with us, which is a geometry check, not
    a tuning knob.

  * **Heading unknown**: only the ANGLES BETWEEN sightings are usable, three
    props are needed, and this is the Snellius-Pothenot problem. It carries a
    failure that returns a perfectly plausible answer -- see `danger_circle`.

Pure geometry. No ROS, no cv2.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

# Two bearings that differ by less than this cross at a shallow angle, and the
# crossing point slides a long way for a small bearing error. A geometric
# condition, not a tuned threshold: at 10 deg of separation, 1 deg of bearing
# error moves the fix by roughly a tenth of the range.
MIN_SEPARATION_DEG = 12.0

# How close to the circle through the three props counts as ON it. Expressed as
# a fraction of the circle's radius, because the ambiguity scales with it.
DANGER_CIRCLE_FRAC = 0.06


@dataclass(frozen=True)
class Fix:
    ok: bool
    x_m: float = float('nan')
    y_m: float = float('nan')
    used: int = 0                     # landmarks that contributed
    residual_m: float = float('nan')  # how far the fix misses the bearing lines
    separation_deg: float = float('nan')  # widest angle between sightings
    reason: str = ''


def _wrap360(deg: float) -> float:
    d = math.fmod(float(deg), 360.0)
    return d + 360.0 if d < 0.0 else d


def _wrap180(deg: float) -> float:
    d = math.fmod(float(deg) + 180.0, 360.0)
    if d <= 0.0:
        d += 360.0
    return d - 180.0


def _unit(bearing_deg: float) -> Tuple[float, float]:
    """Unit vector for a compass bearing in the pool frame.

    Frame stated once, and it is the course file's: +x is bearing 0, +y is
    bearing 90. Every other reading of these numbers puts the fix somewhere
    else with nothing logging a fault.
    """
    a = math.radians(float(bearing_deg))
    return math.cos(a), math.sin(a)


def fix_from_bearings(sightings: Dict[str, float],
                      positions: Dict[str, Tuple[float, float]], *,
                      min_separation_deg: float = MIN_SEPARATION_DEG) -> Fix:
    """Where we are, from ABSOLUTE bearings to props at known positions.

    `sightings` maps prop name to the compass bearing FROM us TO that prop,
    already in world terms (relative bearing plus an anchored heading).

    Each sighting says we lie on the ray leaving the prop along the reverse
    bearing. Two such lines intersect; more are least-squared. Solved in normal
    form -- for a line through `L` with direction `d`, every point `P` on it
    satisfies `n . (P - L) = 0` where `n` is perpendicular to `d` -- because
    that is linear in `P` and needs no per-landmark range as an unknown.

    ⛔ THE GEOMETRY IS CHECKED BEFORE THE ALGEBRA. Two nearly parallel lines
    still intersect, just a long way away and with enormous sensitivity: at
    10 deg of separation a 1 deg bearing error moves the fix by a tenth of the
    range. The solve would succeed and hand back a confident, wrong position,
    so a thin geometry is REFUSED rather than reported with a large residual.
    """
    named = [(k, v) for k, v in sightings.items() if k in positions]
    if len(named) < 2:
        return Fix(False, used=len(named),
                   reason=f'{len(named)} sighted props have known positions, '
                          f'need 2')

    bearings = [b for _, b in named]
    sep = _widest_separation(bearings)
    if sep < min_separation_deg:
        return Fix(False, used=len(named), separation_deg=sep,
                   reason=f'sightings span only {sep:.1f} deg; below '
                          f'{min_separation_deg:.0f} the crossing slides too '
                          f'far for a small bearing error')

    # Normal-form least squares: rows n^T, right-hand side n . L.
    ata = [[0.0, 0.0], [0.0, 0.0]]
    atb = [0.0, 0.0]
    for name, bearing in named:
        lx, ly = positions[name]
        dx, dy = _unit(bearing)
        nx, ny = -dy, dx                      # perpendicular to the sight line
        c = nx * lx + ny * ly
        ata[0][0] += nx * nx
        ata[0][1] += nx * ny
        ata[1][0] += ny * nx
        ata[1][1] += ny * ny
        atb[0] += nx * c
        atb[1] += ny * c

    det = ata[0][0] * ata[1][1] - ata[0][1] * ata[1][0]
    if abs(det) < 1e-12:
        return Fix(False, used=len(named), separation_deg=sep,
                   reason='sight lines are singular (collinear geometry)')
    x = (atb[0] * ata[1][1] - ata[0][1] * atb[1]) / det
    y = (ata[0][0] * atb[1] - atb[0] * ata[1][0]) / det

    resid = 0.0
    for name, bearing in named:
        lx, ly = positions[name]
        dx, dy = _unit(bearing)
        nx, ny = -dy, dx
        resid += abs(nx * (x - lx) + ny * (y - ly))
    resid /= len(named)
    return Fix(True, x_m=x, y_m=y, used=len(named), residual_m=resid,
               separation_deg=sep)


def _widest_separation(bearings: Sequence[float]) -> float:
    """Largest angle between any two sightings, in [0, 180]."""
    widest = 0.0
    for i in range(len(bearings)):
        for j in range(i + 1, len(bearings)):
            widest = max(widest, abs(_wrap180(bearings[i] - bearings[j])))
    return widest


def circumcircle(a: Tuple[float, float], b: Tuple[float, float],
                 c: Tuple[float, float]) -> Optional[Tuple[Tuple[float, float], float]]:
    """(centre, radius) of the circle through three points, or None if collinear."""
    (ax, ay), (bx, by), (cx, cy) = a, b, c
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        return None
    a2, b2, c2 = ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy
    ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d
    uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d
    return (ux, uy), math.hypot(ax - ux, ay - uy)


def on_danger_circle(p: Tuple[float, float],
                     a: Tuple[float, float], b: Tuple[float, float],
                     c: Tuple[float, float], *,
                     frac: float = DANGER_CIRCLE_FRAC) -> bool:
    """Is the observer on the circle through the three props?

    ⛔ THE FAILURE THAT RETURNS A PLAUSIBLE NUMBER. With the heading unknown,
    only the angles BETWEEN sightings are measurable, and every point on the
    circle through the three props subtends the SAME pair of angles. The
    problem then has infinitely many solutions and a solver will still hand
    back one of them, indistinguishable from a good fix. Surveyors have called
    it the danger circle since the 1600s and the instruction has always been to
    avoid observing from it, not to solve harder.

    Two consequences for a mission. Detect it and refuse. And when a fix is
    refused for this reason, MOVING OFF the circle is what fixes it -- a metre
    in any direction off the arc, which is a thing a vehicle can do and a
    solver cannot.
    """
    circ = circumcircle(a, b, c)
    if circ is None:
        return True            # collinear props: no circle, and no fix either
    (ux, uy), r = circ
    if r <= 1e-9:
        return True
    return abs(math.hypot(p[0] - ux, p[1] - uy) - r) <= frac * r

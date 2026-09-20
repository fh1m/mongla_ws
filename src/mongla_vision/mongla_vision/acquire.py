"""How close is close enough to SEE a prop, and what to do when you cannot.

⛔ THE DEAD ZONE THIS EXISTS TO REMOVE. A waypoint that stops at an arbitrary
standoff can put the hull somewhere the prop is present and undetectable: too
far for the box to survive the detector, too close for a search pattern to make
sense. The vehicle then sits in open water with nothing to steer on, and the
mission has no way to tell "not there" from "too far to see".

The fix is not a better search. It is to stop choosing the standoff arbitrarily.
A prop of known width `w` projects to a box of `f * w / Z` pixels, so the range
at which it is still detectable is arithmetic:

    Z_max = f_px * width_m / min_box_px

MEASURED ON THE VEHICLE (2026-09-11, `yolov8n_seg`, Hailo-8, conf 0.20): a real
object survives down to a box **9.3-11.4 px wide** and is lost within one scale
step below that. The loss is a cliff, not a fade, and the four survivors sat at
conf 0.21-0.28 -- right on the threshold. So the physical floor is ~10 px and a
standoff must aim WELL inside it; `DETECT_MARGIN` is why.

⚠ That floor is for `person` on a COCO model. Our competition models are
trained differently and each one needs its own measurement -- the method is the
deliverable, not the constant. `tools` can re-run it per model by shrinking a
frame with a known detection until the class drops out.

Pure geometry. No ROS, no cv2.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

# Measured cliff, in pixels of box width. Below this the detector does not
# degrade, it stops.
DETECT_FLOOR_PX = 10.0

# Aim for a box this many times the floor. Detecting AT the floor happened at
# conf 0.21-0.28 with the threshold at 0.20, which is not a margin, it is a
# coin toss. 4x the floor puts the box at ~40 px, where confidence is not the
# binding constraint.
DETECT_MARGIN = 4.0

# A search leg shorter than this is inside the noise of the distance verb
# itself, so stepping by less than it buys nothing and costs a turn.
MIN_SEARCH_STEP_M = 0.5

# ⛔ HOW FAR YOU CAN SEE, WHICH IS USUALLY THE BINDING CONSTRAINT AND NOT THE
# PIXELS. Running the arithmetic above on real props exposed it immediately:
#
#     prop            width     pixel-limited range    standoff
#     gate            3.000 m        154.2 m            38.6 m
#     torpedo board   0.600 m         30.8 m             7.7 m
#     bin             0.335 m         17.2 m             4.3 m
#     slalom pipe     0.033 m          1.7 m             0.8 m
#
# Nobody sees a gate at 38 m underwater. For anything large the detector is not
# what runs out first, the water is, so the standoff is clamped by visibility
# and the crossover between the two regimes is computable per prop. 6 m is a
# deliberately optimistic competition-pool figure and is NOT measured by us --
# measure it at the venue and override it, because a standoff built on a
# visibility we never checked is a waypoint that stops where nothing is
# visible.
#
# ★ AND THE SLALOM IS THE OTHER END OF THAT TABLE. A 33 mm pipe is detectable
# from 1.7 m and no further, whatever the water does. No prior map can put a
# hull "in detection range" of one from across the pool -- you arrive nearly on
# top of it or you do not see it at all. That is a mission-design fact, and it
# is very likely why the team that wins dead-reckons past the slalom instead of
# perceiving their way through it.
VISIBILITY_M = 6.0


@dataclass(frozen=True)
class Leg:
    """One step of a search: turn by `turn_deg`, then run `run_m`."""
    turn_deg: float
    run_m: float
    index: int = 0


def detection_range_m(width_m: float, f_px: float, *,
                      min_box_px: float = DETECT_FLOOR_PX) -> float:
    """Furthest range at which a prop of `width_m` still makes a `min_box_px` box.

    The pinhole relation, and the only reason it is worth a function is that
    every caller otherwise inlines it with a different pixel floor.
    """
    if width_m <= 0.0 or f_px <= 0.0 or min_box_px <= 0.0:
        return 0.0
    return float(f_px) * float(width_m) / float(min_box_px)


def standoff_for(width_m: float, f_px: float, *,
                 margin: float = DETECT_MARGIN,
                 min_box_px: float = DETECT_FLOOR_PX,
                 floor_m: float = 0.8,
                 visibility_m: float = VISIBILITY_M) -> float:
    """Where to stop so the prop is comfortably detectable, not marginally.

    Three terms, and which one binds depends on the prop. Pixels bind for a
    slalom pipe (1.7 m and no further). WATER binds for a gate, whose
    pixel-limited range is 154 m and whose real one is whatever the pool
    allows. `floor_m` keeps the answer off the hull.

    Taking the minimum is the whole point: a standoff chosen from optics alone
    parks a gate approach 38 m out, where the camera shows silt.
    """
    z = detection_range_m(width_m, f_px, min_box_px=min_box_px * float(margin))
    z = min(z, float(visibility_m))
    return max(float(floor_m), z)


def search_radius_m(fix_residual_m: float, leg_m: float, *,
                    heading_sigma_deg: float = 3.0,
                    fix_floor_m: float = 0.3) -> float:
    """How far the search must reach, from the error we actually carry.

    ⛔ NOT A GUESSED BOX. Two terms, both of which we measure rather than
    assume: what the position fix already disagreed with itself by, and how far
    a heading error throws a leg of this length off its line. A search sized
    smaller than the uncertainty cannot find the prop; one sized much larger
    spends the run.
    """
    cross = abs(float(leg_m)) * math.tan(math.radians(abs(float(heading_sigma_deg))))
    # The floor applies to the TOTAL, not to the residual term. Flooring the
    # residual instead would quietly inflate a good fix into a worse one and
    # make the returned number stop meaning "the error we carry".
    return max(float(fix_floor_m), abs(float(fix_residual_m)) + cross)


def expanding_box(step_m: float, *, reach_m: float,
                  max_legs: int = 12) -> List[Leg]:
    """An expanding square spiral, in metric legs, bounded by `reach_m`.

    ⛔ A SQUARE, NOT A LAWNMOWER, AND NOT A SPIN. A spin in place re-searches
    the same water and only helps if the prop is in view but off-bearing. A
    lawnmower needs a box to sweep and a heading to keep, which is exactly what
    we are short of. An expanding square starts at the best estimate and grows
    outward, so the FIRST legs cover where the prop most likely is, and every
    leg is a straight run the distance verb can actually close.

    Leg lengths go 1,1,2,2,3,3... times `step_m`, each preceded by a 90 deg
    turn. Generation stops when the pattern would leave `reach_m`, so a
    confident fix searches a small box and a shaky one searches a big one
    without anyone choosing a number.
    """
    step = max(float(MIN_SEARCH_STEP_M), float(step_m))
    legs: List[Leg] = []
    length = 1
    while len(legs) < max_legs:
        for _ in range(2):                       # two legs per side length
            run = length * step
            if run > float(reach_m):
                return legs
            legs.append(Leg(90.0, run, index=len(legs)))
            if len(legs) >= max_legs:
                return legs
        length += 1
    return legs


def total_path_m(legs: List[Leg]) -> float:
    """How far the vehicle actually swims to complete a pattern.

    Reported because a search is a budget item: at 0.3 m/s a 20 m pattern is
    over a minute of a run that has a few.
    """
    return sum(l.run_m for l in legs)

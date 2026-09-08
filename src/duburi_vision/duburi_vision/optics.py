"""Flat-port refraction -- the ONE place the water refractive index lives.

WHY THIS MODULE EXISTS (B22). `N_WATER` was declared in `calibration/solver.py`
as "the single source of truth" and then the literal `1.333` was written out by
hand at five more sites -- including `fov_for_medium`, the INVERSE transform, in
the same file as the forward one. Forward and inverse read the index from two
different places, so changing `N_WATER` to 1.34 for salt water silently stopped
them being inverses.

That is worse than an ordinary DRY complaint because the quantity is PHYSICAL
and the two uses are INVERSES: the failure is not "one copy is stale", it is
"the round trip no longer closes", and the result is a plausible FOV that is
self-inconsistent. `solver.py`'s own docstring warns that applying the
refraction the wrong way is "a ~1.33x error, in the direction that still looks
like a plausible camera" -- exactly the class of defect this codebase keeps
finding.

So the two directions are defined ONCE, here, as a pair. They cannot drift apart
without this file changing, and `test_optics.py` asserts they remain inverses at
an index deliberately DIFFERENT from the shipped one -- because at n = 1.333 the
broken code round-trips too, and a test that cannot tell the two states apart is
not a test.

NO cv2, NO numpy, NO ROS. `distance/flow_math.py` is deliberately
dependency-light (math + numpy only) and could not import `solver`, which is
part of how the literal got copied there in the first place.
"""

from __future__ import annotations

import math

# Sea/fresh water. A flat port refracts by Snell's law; salt water is ~1.34, and
# changing THIS LINE is the supported way to explore that -- every transform
# below and every caller reads it from here.
N_WATER = 1.333


def fov_air_to_water(fov_air_deg: float, n: float | None = None) -> float:
    """In-air full FOV -> in-water full FOV, through a flat port.

    The half-angle in water is asin(sin(half-angle in air) / n). This is why an
    80 deg air lens is ~58 deg underwater -- the single biggest surprise for
    anyone sizing a search pattern off a datasheet.
    """
    n = N_WATER if n is None else n           # read at CALL time, see note below
    half = math.radians(fov_air_deg / 2.0)
    return 2.0 * math.degrees(math.asin(min(1.0, math.sin(half) / n)))


def fov_water_to_air(fov_water_deg: float, n: float | None = None) -> float:
    """In-water full FOV -> in-air full FOV. The exact inverse of the above.

    Used when the measurement was taken in water and the air figure has to be
    DERIVED -- which is why it must be the inverse and not an independent
    re-derivation.
    """
    n = N_WATER if n is None else n           # read at CALL time, see note below
    half = math.radians(fov_water_deg / 2.0)
    return 2.0 * math.degrees(math.asin(min(1.0, n * math.sin(half))))


# WHY `n=None` AND NOT `n=N_WATER` AS A DEFAULT. A default argument is evaluated
# ONCE, when the function is defined, so `n: float = N_WATER` would freeze the
# index at import time: changing `N_WATER` afterwards -- which is the documented
# way to try salt water -- would silently keep using 1.333. It also makes the
# defect untestable, since a test cannot vary the constant to show that forward
# and inverse read the SAME one. Reading it at call time fixes both.

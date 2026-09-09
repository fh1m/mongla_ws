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
from typing import Optional

import numpy as np

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

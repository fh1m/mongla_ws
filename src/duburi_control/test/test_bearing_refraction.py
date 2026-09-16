"""A bearing sent to the board must be the angle IN THE WATER.

This module exists to give control gains UNITS -- "thrust per radian" instead of
"thrust per fraction-of-a-frame". A flat port refracts, so the pinhole model
recovers the ray angle INSIDE the housing, and the vehicle needs the direction
to the object outside it: `sin(air) = n * sin(water)`.

MEASURED on `pi_forward_1280x720.json` (fx 851.2, cx 675.4):

    px from cx   air (ta)   true (tw)     error
         100      6.70 deg   5.02 deg    +1.68 deg  (+33.4 %)
         300     19.41      14.44        +4.97      (+34.5 %)
         640     36.94      26.80       +10.14      (+37.8 %)

Near the axis `sin x ~ x`, so this is not an edge effect: it is the refractive
index applied to the entire angular scale. The uplink already warns about a
~2.5 deg linear-approximation error while carrying this one silently.
"""
import json
import math
from pathlib import Path

import pytest

from duburi_control.bearing import bearing_from_pixels, bearing_from_normalised

_CAL = (Path(__file__).resolve().parents[2] / 'duburi_vision' / 'config'
        / 'calibration' / 'pi_forward_1280x720.json')
N = 1.333


def _K():
    d = json.loads(_CAL.read_text())
    m = d['camera_matrix']
    return [m[0][0], 0.0, m[0][2], 0.0, m[1][1], m[1][2], 0.0, 0.0, 1.0], d


def test_air_is_the_exact_identity():
    """A bench run must be byte-for-byte the previous behaviour, or 'no
    correction' becomes its own silent correction."""
    K, d = _K()
    w, h = d['image_width'], d['image_height']
    for u in (10.0, w / 2, w - 10.0):
        a = bearing_from_pixels(u, h / 2, 50, 50, width=w, height=h, K=K, n_medium=1.0)
        b = bearing_from_pixels(u, h / 2, 50, 50, width=w, height=h, K=K)
        assert a.angle_x == b.angle_x and a.size_x == b.size_x


@pytest.mark.parametrize('u_off', [100.0, 300.0, 640.0])
def test_the_water_bearing_matches_snell_exactly(u_off):
    K, d = _K()
    fx, cx = K[0], K[2]
    w, h = d['image_width'], d['image_height']
    # On the principal ROW (v = cy), so the ray is purely horizontal and the
    # 1-D Snell form is exact; off that row the polar angle is what refracts.
    got = bearing_from_pixels(cx + u_off, K[5], 40, 40,
                              width=w, height=h, K=K, n_medium=N)
    ta = math.atan(u_off / fx)
    want = math.asin(math.sin(ta) / N)
    assert got.angle_x == pytest.approx(want, rel=1e-9)


def test_the_uncorrected_bearing_is_a_THIRD_too_large():
    """The defect, pinned as a number. If the correction is reverted this is
    what comes back."""
    K, d = _K()
    fx, cx = K[0], K[2]
    w, h = d['image_width'], d['image_height']
    air = bearing_from_pixels(cx + 300.0, h / 2, 40, 40,
                              width=w, height=h, K=K, n_medium=1.0)
    water = bearing_from_pixels(cx + 300.0, h / 2, 40, 40,
                                width=w, height=h, K=K, n_medium=N)
    ratio = air.angle_x / water.angle_x
    assert 1.30 < ratio < 1.40, f'expected ~1.34x, got {ratio:.3f}'
    assert math.degrees(air.angle_x - water.angle_x) == pytest.approx(4.97, abs=0.05)


def test_the_optical_axis_is_still_a_fixed_point():
    """Refraction bends nothing at normal incidence. If a target on the axis
    moved, the transform is being applied in the wrong frame."""
    K, d = _K()
    got = bearing_from_pixels(K[2], K[5], 40, 40, width=d['image_width'],
                              height=d['image_height'], K=K, n_medium=N)
    assert got.angle_x == pytest.approx(0.0, abs=1e-12)
    assert got.angle_y == pytest.approx(0.0, abs=1e-12)


def test_the_ANGULAR_SIZE_is_refracted_per_edge():
    """`size_x` is the standoff measure. Refracting the difference instead of
    each edge would reintroduce the eccentricity error the edge-based form
    exists to avoid -- an off-centre box's two edges sit at different field
    angles and compress by different amounts.
    """
    K, d = _K()
    fx, cx = K[0], K[2]
    w, h = d['image_width'], d['image_height']
    u, box = cx + 400.0, 120.0
    got = bearing_from_pixels(u, K[5], box, box, width=w, height=h, K=K, n_medium=N)
    e1 = math.asin(math.sin(math.atan((u - box / 2 - cx) / fx)) / N)
    e2 = math.asin(math.sin(math.atan((u + box / 2 - cx) / fx)) / N)
    assert got.size_x == pytest.approx(abs(e2 - e1), rel=1e-9)
    # and it must NOT equal the naive "refract the whole width" form
    naive = math.asin(math.sin(math.atan(box / fx)) / N)
    assert abs(got.size_x - naive) > 1e-4


def test_the_FOV_fallback_is_refracted_too():
    """"No calibration" must not silently also mean "and 33 % off in the other
    direction" -- that is a second unrelated error, not a fallback."""
    w, h = 1280, 720
    hf = math.radians(73.88)
    air = bearing_from_pixels(w * 0.9, h / 2, 40, 40, width=w, height=h,
                              hfov_rad=hf, vfov_rad=hf * 0.6, n_medium=1.0)
    water = bearing_from_pixels(w * 0.9, h / 2, 40, 40, width=w, height=h,
                                hfov_rad=hf, vfov_rad=hf * 0.6, n_medium=N)
    assert not air.calibrated and not water.calibrated
    assert abs(water.angle_x) < abs(air.angle_x)


def test_the_normalised_entry_point_forwards_the_medium():
    """`bearing_from_normalised` is what the uplink actually calls; a parameter
    it accepted and dropped would be the knob-wired-to-nothing defect again."""
    K, d = _K()
    w, h = d['image_width'], d['image_height']
    a = bearing_from_normalised(0.6, 0.0, 0.05, 0.05, width=w, height=h,
                                K=K, n_medium=1.0)
    b = bearing_from_normalised(0.6, 0.0, 0.05, 0.05, width=w, height=h,
                                K=K, n_medium=N)
    assert abs(b.angle_x) < abs(a.angle_x), 'n_medium is dropped on this path'


# --------------------------------------------------------------------------
#  ⛔ THE INDEPENDENT CHECK. Everything above compares the implementation with
#  `asin(sin(ta)/N)` -- THE SAME EXPRESSION THE IMPLEMENTATION USES. Those tests
#  are self-consistent: they pin the arithmetic, and they would ALL still pass
#  if the correction ran in the wrong direction (multiplying by n instead of
#  dividing), which would make every bearing 33 % too SMALL instead of too
#  large. That is the exact "a passing test that checks nothing" failure this
#  repo keeps paying for.
#
#  Ground truth below is PURE GEOMETRY -- atan2(X, Z) of a known point in the
#  water -- reached through a ray trace that shares no code with bearing.py.
# --------------------------------------------------------------------------

_NG, _DG, _D0 = 1.49, 0.006, 0.015


def _ray_trace_to_pixel(X, Z, fx, cx):
    """TRUE image column of a water point at (X, 0, Z). Independent of the model."""
    lo, hi = 0.0, abs(X) + _D0 + 1.0
    for _ in range(200):
        h = 0.5 * (lo + hi)
        ta = math.atan2(h, _D0)
        tg = math.asin(min(1.0, math.sin(ta) / _NG))
        tw = math.asin(min(1.0, math.sin(ta) / N))
        hit = h + _DG * math.tan(tg) + (Z - _D0 - _DG) * math.tan(tw)
        lo, hi = (h, hi) if hit < abs(X) else (lo, h)
    return cx + math.copysign(fx * h / _D0, X)


@pytest.mark.parametrize('X,Z', [(0.05, 1.0), (0.20, 1.0), (0.05, 2.0),
                                 (0.20, 2.0), (0.50, 2.0), (0.90, 2.0)])
def test_the_corrected_bearing_matches_PURE_GEOMETRY(X, Z):
    """The one test that could catch an inverted correction."""
    K, d = _K()
    fx, cx, cy = K[0], K[2], K[5]
    w, h = d['image_width'], d['image_height']
    u = _ray_trace_to_pixel(X, Z, fx, cx)
    assert 0 <= u < w, 'probe point left the frame -- adjust the case'

    truth = math.degrees(math.atan2(X, Z))
    got = math.degrees(bearing_from_pixels(u, cy, 30, 30, width=w, height=h,
                                           K=K, n_medium=N).angle_x)
    # 0.1 deg covers the NON-CENTRAL residual the central model omits
    # (measured worst 0.065 deg over these cases); it does NOT cover a sign or
    # scale error, which is the point.
    assert abs(got - truth) < 0.1, f'{got:.3f} vs true {truth:.3f}'


def test_the_correction_REDUCES_the_error_it_claims_to_fix():
    """Direction, stated as an inequality rather than a formula.

    A correction applied the wrong way makes the error LARGER than doing
    nothing. Measured here: worst 0.065 deg corrected vs 8.84 deg uncorrected.
    """
    K, d = _K()
    fx, cx, cy = K[0], K[2], K[5]
    w, h = d['image_width'], d['image_height']
    worst_corr = worst_raw = 0.0
    for X, Z in ((0.2, 1.0), (0.5, 2.0), (0.9, 2.0)):
        u = _ray_trace_to_pixel(X, Z, fx, cx)
        truth = math.degrees(math.atan2(X, Z))
        corr = math.degrees(bearing_from_pixels(u, cy, 30, 30, width=w, height=h,
                                                K=K, n_medium=N).angle_x)
        raw = math.degrees(bearing_from_pixels(u, cy, 30, 30, width=w, height=h,
                                               K=K, n_medium=1.0).angle_x)
        worst_corr = max(worst_corr, abs(corr - truth))
        worst_raw = max(worst_raw, abs(raw - truth))
    assert worst_corr < 0.1 < worst_raw
    assert worst_raw / max(worst_corr, 1e-9) > 20, (
        f'the correction only improved things {worst_raw / worst_corr:.1f}x -- '
        f'expected ~100x; suspect a sign or a scale error')


def test_an_off_axis_corner_matches_a_ray_traced_flat_port():
    """Truth by construction: a point in WATER, traced through a flat port.

    The water ray to (X, Y, Z) enters the port and bends in its own plane, so
    the pixel it lands on has the SAME azimuth and a larger polar angle. Put the
    target at a frame corner, where both axes are off, and the bearing must
    return the water angles exactly -- per-axis refraction does not.
    """
    import math
    from duburi_control.bearing import bearing_from_pixels
    n = 1.333
    fx = fy = 851.2
    cx, cy = 640.0, 360.0
    X, Y, Z = 1.2, 0.7, 2.0                       # water, camera frame
    rw = math.hypot(X / Z, Y / Z)
    ra = math.tan(math.asin(n * math.sin(math.atan(rw))))
    u = cx + fx * (X / Z) * ra / rw
    v = cy + fy * (Y / Z) * ra / rw
    b = bearing_from_pixels(u, v, 1.0, 1.0, width=1280, height=720,
                            K=[fx, 0, cx, 0, fy, cy, 0, 0, 1], n_medium=n)
    assert math.degrees(b.angle_x) == pytest.approx(math.degrees(math.atan(X / Z)), abs=1e-6)
    assert math.degrees(b.angle_y) == pytest.approx(math.degrees(math.atan(Y / Z)), abs=1e-6)

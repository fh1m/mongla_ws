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
    got = bearing_from_pixels(cx + u_off, h / 2, 40, 40,
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
    got = bearing_from_pixels(u, h / 2, box, box, width=w, height=h, K=K, n_medium=N)
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

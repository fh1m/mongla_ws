"""floor_plane scored against CONSTRUCTED truth, never against range_to.

Truth comes from a camera built a different way -- its three axes written as
hull-frame vectors and points projected by dot products -- so a sign error in
the trig of `intersect` cannot be copied into the reference.
"""

import math
import random

import pytest

from duburi_localization.floor_plane import MIN_GRAZING_DEG, intersect

F, CX, CY = 741.0, 320.0, 240.0


def _project(point_hull, pitch_deg):
    """Hull-frame point (camera at origin) -> pixel, via explicit camera axes."""
    p = math.radians(pitch_deg)
    z = (math.cos(p), 0.0, math.sin(p))       # optical axis
    x = (0.0, 1.0, 0.0)                       # image right = hull right
    y = (-math.sin(p), 0.0, math.cos(p))      # image down
    dot = lambda a, b: sum(i * j for i, j in zip(a, b))
    xc, yc, zc = dot(x, point_hull), dot(y, point_hull), dot(z, point_hull)
    assert zc > 0.0
    return F * xc / zc + CX, F * yc / zc + CY


@pytest.mark.parametrize('pitch', [0.0, 12.0, 35.0, 90.0])
@pytest.mark.parametrize('plane', [1.2, 0.4])
def test_recovers_constructed_floor_points(pitch, plane):
    rng = random.Random(7)
    checked = 0
    p = math.radians(pitch)
    ax = ((-math.sin(p), 0.0, math.cos(p)), (0.0, 1.0, 0.0),
          (math.cos(p), 0.0, math.sin(p)))          # image-down, right, optical
    for _ in range(200):
        # Truth: a pixel's ray assembled from the camera axes, scaled to the plane.
        u, v = rng.uniform(0, 640), rng.uniform(0, 480)
        a, b = (v - CY) / F, (u - CX) / F
        ray = [a * ax[0][i] + b * ax[1][i] + ax[2][i] for i in range(3)]
        if ray[2] <= 0.0:
            continue
        pt = [c * plane / ray[2] for c in ray]
        assert _project(pt, pitch) == pytest.approx((u, v))
        got = intersect(u, v, fx=F, fy=F, cx=CX, cy=CY,
                        plane_below_m=plane, pitch_deg=pitch)
        if got is None:
            continue
        assert got.forward_m == pytest.approx(pt[0], abs=1e-9)
        assert got.right_m == pytest.approx(pt[1], abs=1e-9)
        checked += 1
    assert checked > 20


def test_a_plane_above_the_camera_is_reached_upward():
    pt = (3.0, 0.5, -0.3)                      # gate bar 0.3 m above the lens
    u, v = _project(pt, 0.0)
    got = intersect(u, v, fx=F, fy=F, cx=CX, cy=CY, plane_below_m=-0.3)
    assert got.forward_m == pytest.approx(3.0) and got.right_m == pytest.approx(0.5)


def test_a_ray_that_never_reaches_the_floor_is_refused():
    # Level camera, pixel above centre: ray rises, floor is below.
    assert intersect(320, 100, fx=F, fy=F, cx=CX, cy=CY, plane_below_m=1.0) is None


def test_near_parallel_rays_are_refused_not_clamped():
    v = CY + F * math.tan(math.radians(MIN_GRAZING_DEG * 0.5))
    assert intersect(320, v, fx=F, fy=F, cx=CX, cy=CY, plane_below_m=1.0) is None


def test_sigma_matches_a_one_degree_pitch_error_measured_by_perturbation():
    pt = (3.0, 0.0, 1.0)
    u, v = _project(pt, 0.0)
    tilted = intersect(u, v, fx=F, fy=F, cx=CX, cy=CY, plane_below_m=1.0,
                       pitch_deg=1.0)
    moved = abs(tilted.range_m - 3.0)
    got = intersect(u, v, fx=F, fy=F, cx=CX, cy=CY, plane_below_m=1.0,
                    pixel_sigma=0.0, pitch_sigma_deg=1.0)
    assert got.sigma_m == pytest.approx(moved, rel=0.15)


def test_sigma_grows_with_range_and_with_floor_uncertainty():
    near = intersect(*_project((1.0, 0, 1.0), 0.0), fx=F, fy=F, cx=CX, cy=CY,
                     plane_below_m=1.0)
    far = intersect(*_project((5.0, 0, 1.0), 0.0), fx=F, fy=F, cx=CX, cy=CY,
                    plane_below_m=1.0)
    sloped = intersect(*_project((5.0, 0, 1.0), 0.0), fx=F, fy=F, cx=CX, cy=CY,
                       plane_below_m=1.0, plane_sigma_m=0.4)
    assert far.sigma_m > 5 * near.sigma_m
    assert sloped.sigma_m > far.sigma_m

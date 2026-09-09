"""The metric pose path reads through a flat port, so it must undo one.

`lock_node` fed RAW pixels and the IN-AIR `K` into `target_pose`. A flat port
is not a pinhole: a ray at water angle `tw` leaves at air angle `ta` with
`sin ta = n sin tw`, so the effective focal length grows with field angle and a
single air `f` is wrong everywhere except where it was fitted.

Measured here, by ray-tracing a 0.30 m square through 6 mm of acrylic and
solving for its range: the air-`K` path reports **25-30 % SHORT** at every
range tested. That is not a subtle bias -- it is the kind of number a firing
gate or a standoff would act on.

The correction already existed as `RefractiveRectifier`, written for the flow
pipeline and imported by nothing else. These tests pin (a) that there is now
exactly ONE copy of it, (b) that the rectified points are paired with the
RECTIFIED `K`, and (c) the accuracy actually recovered.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_vision.anchor.pose import target_pose            # noqa: E402
from duburi_vision.optics import N_WATER, RefractiveRectifier  # noqa: E402

# The measured downward camera (`pi_downward_1280x720.json`).
FX, FY, CX, CY = 1027.9, 1033.9, 617.3, 373.0
NG, DG, D0 = 1.49, 0.006, 0.015      # acrylic index, glass thickness, air gap


def _project_flat_port(X, Y, Z):
    """TRUE image point of a water point, by ray trace -- the ground truth.

    Deliberately NOT the model under test: bisect on the height at which the
    ray crosses the port, refracting air->glass->water by Snell, so the test
    cannot pass by sharing the rectifier's own approximation.
    """
    R = math.hypot(X, Y)
    if R < 1e-12:
        return CX, CY
    lo, hi = 0.0, R + D0 + 1.0
    for _ in range(200):
        h = 0.5 * (lo + hi)
        ta = math.atan2(h, D0)
        tg = math.asin(min(1.0, math.sin(ta) / NG))
        tw = math.asin(min(1.0, math.sin(ta) / N_WATER))
        hit = h + DG * math.tan(tg) + (Z - D0 - DG) * math.tan(tw)
        lo, hi = (h, hi) if hit < R else (lo, h)
    r_px = FX * h / D0
    return CX + r_px * X / R, CY + (FY / FX) * r_px * Y / R


def _square(width_m, Z, off=0.0):
    w = width_m / 2.0
    corners = [(-w + off, -w), (w + off, -w), (w + off, w), (-w + off, w)]
    live = np.array([_project_flat_port(x, y, Z) for x, y in corners], np.float64)
    ref = np.array([[0., 0.], [200., 0.], [200., 200.], [0., 200.]], np.float64)
    return ref, live


def _rect_and_K():
    r = RefractiveRectifier(FX, FY, CX, CY)
    return r, np.array([[r.f_ref, 0.0, CX],
                        [0.0, r.f_ref_y, CY],
                        [0.0, 0.0, 1.0]], np.float64)


K_AIR = np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]], np.float64)


# --------------------------------------------------------------------------
#  One copy of the model
# --------------------------------------------------------------------------

def test_the_rectifier_has_exactly_one_home():
    """`optics` exists to be "the ONE place the water refractive index lives"
    (its own docstring, written after B22 split that constant across five
    files). The MODEL built on the constant belongs with it, or the metric
    pose path and the flow path drift into two flat-port models."""
    from duburi_vision.flow import flow_math
    import duburi_vision.optics as optics
    assert flow_math.RefractiveRectifier is optics.RefractiveRectifier
    src = (Path(__file__).resolve().parents[1] / 'duburi_vision' / 'flow'
           / 'flow_math.py').read_text()
    assert 'class RefractiveRectifier' not in src, (
        'flow_math defines its own copy again -- one truth, two copies')


# --------------------------------------------------------------------------
#  The pairing that is easy to get wrong
# --------------------------------------------------------------------------

def test_rectified_points_require_the_rectified_focal_length():
    """`rectify` re-projects each ray through `f_ref` (= fx * n by default),
    so the rectified image is a pinhole of focal `f_ref`, NOT `fx`.

    Pairing rectified points with the air `K` leaves a clean 1/n scale error --
    every range ~25 % short, with a plausible number and no warning. This test
    states the pairing as arithmetic so the two cannot drift apart.
    """
    r, K_rect = _rect_and_K()
    assert r.f_ref == pytest.approx(FX * N_WATER)
    assert K_rect[0][0] == pytest.approx(r.f_ref)
    # aspect survives -- fx != fy is a real 0.58 % on this camera
    assert K_rect[1][1] / K_rect[0][0] == pytest.approx(FY / FX, rel=1e-12)

    # a ray at water angle tw must land at f_ref * tan(tw)
    tw = math.radians(20.0)
    ta = math.asin(min(1.0, N_WATER * math.sin(tw)))
    u = CX + FX * math.tan(ta)
    got = r.rectify([[u, CY]])[0][0] - CX
    assert got == pytest.approx(r.f_ref * math.tan(tw), rel=1e-9)


@pytest.mark.parametrize('Z', [0.5, 1.0, 2.0])
def test_the_air_K_path_reports_the_range_25_percent_short(Z):
    """The defect, pinned as a number. If someone reverts the rectification,
    this is what they get back."""
    ref, live = _square(0.30, Z)
    tp = target_pose(ref, live, K_AIR, width_m=0.30, ref_size_px=(200, 200))
    assert tp.ok
    err = (tp.range_m - Z) / Z
    assert -0.35 < err < -0.20, f'expected ~-25 %, got {err:+.1%}'


@pytest.mark.parametrize('Z', [0.5, 1.0, 2.0])
def test_the_rectified_path_recovers_the_true_range(Z):
    """Within a few percent, against a ray trace the model never sees."""
    r, K_rect = _rect_and_K()
    ref, live = _square(0.30, Z)
    tp = target_pose(ref, r.rectify(live), K_rect,
                     width_m=0.30, ref_size_px=(200, 200))
    assert tp.ok
    err = abs(tp.range_m - Z) / Z
    assert err < 0.03, f'{tp.range_m:.3f} m vs {Z} m ({err:+.1%})'


def test_air_medium_is_the_exact_identity():
    """A bench run must be byte-for-byte the old behaviour, or 'no correction'
    becomes its own silent correction."""
    r = RefractiveRectifier(FX, FY, CX, CY, n=1.0)
    pts = np.array([[0., 0.], [CX, CY], [1279., 719.], [10., 700.]], np.float64)
    assert np.allclose(r.rectify(pts), pts, atol=1e-9)


# --------------------------------------------------------------------------
#  A pose that does not exist must not be published as one
# --------------------------------------------------------------------------

def test_a_non_finite_solution_is_refused_not_published():
    """MEASURED: a rectified 0.30 m square at 1.0 m, 0.35 m off-axis, comes
    back from solvePnP with range/reproj/yaw all NaN -- and `ok=True`.

    NaN fails every comparison silently, so `range_m < tol` is False and
    `range_m > tol` is False: two callers testing opposite ways both get the
    answer they were not warned about. `lock_node` nan-guards reproj and
    ambiguity and publishes range raw, so this has to be refused at the source.
    `geometry.py` already calls this exact guard load-bearing.
    """
    r, K_rect = _rect_and_K()
    ref, live = _square(0.30, 1.0, off=0.35)
    tp = target_pose(ref, r.rectify(live), K_rect,
                     width_m=0.30, ref_size_px=(200, 200))
    if tp.ok:
        for name in ('range_m', 'yaw_deg', 'pitch_deg', 'roll_deg', 'reproj_px'):
            v = getattr(tp, name)
            assert math.isfinite(v), f'ok=True with {name}={v}'
    else:
        assert tp.reason, 'a refusal must say why'


def test_lock_node_pairs_the_rectified_points_with_the_rectified_K():
    """Source-level, deliberately: the failure is a MISPAIRING, and a live
    rclpy fixture would test the harness rather than which K reaches PnP."""
    src = (Path(__file__).resolve().parents[1] / 'duburi_vision'
           / 'lock_node.py').read_text()
    assert 'self._rect.rectify(ref_pts)' in src.replace('\n', ' ') or \
           'rect.rectify(ref_pts)' in src, 'ref points are not rectified'
    assert 'target_pose(ref_pts, live_pts, self._K_rect' in src, (
        'rectified points must be solved with the RECTIFIED K')
    assert "self.declare_parameter('medium', 'water')" in src


def test_camera_info_does_not_rebuild_the_rectifier_every_frame():
    """`CameraInfo` is published with every frame. Rebuilding the rectifier and
    logging on each one costs an allocation at camera rate and buries the log --
    measured on the vehicle: the medium line printed once per frame.

    Only a genuine change of intrinsics is an event.
    """
    src = (Path(__file__).resolve().parents[1] / 'duburi_vision'
           / 'lock_node.py').read_text()
    i = src.index('def _on_info')
    body = src[i:i + 2200]
    assert 'np.array_equal(K, self._K)' in body, (
        '_on_info rebuilds the rectifier for every CameraInfo message')
    assert body.index('np.array_equal') < body.index('RefractiveRectifier('), (
        'the unchanged-K early return must come BEFORE the rebuild')

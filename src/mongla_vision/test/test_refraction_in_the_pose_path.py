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
import contextlib
import math
import types
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_vision.anchor.pose import target_pose            # noqa: E402
from mongla_vision.optics import N_WATER, RefractiveRectifier  # noqa: E402


@contextlib.contextmanager
def _node(medium='water'):
    """A LockNode with no ROS graph.

    Shuts down exactly the rclpy context it started: the context is
    process-global, so leaving it initialised makes another test file's own
    `rclpy.init()` raise and errors every test in it.
    """
    import rclpy
    from rclpy.parameter import Parameter
    from mongla_vision.lock_node import LockNode
    started = not rclpy.ok()
    if started:
        rclpy.init()
    node = LockNode(parameter_overrides=[Parameter('medium', value=medium)])
    try:
        yield node
    finally:
        node.destroy_node()
        if started:
            rclpy.shutdown()

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
    from mongla_vision.flow import flow_math
    import mongla_vision.optics as optics
    assert flow_math.RefractiveRectifier is optics.RefractiveRectifier
    src = (Path(__file__).resolve().parents[1] / 'mongla_vision' / 'flow'
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


@contextlib.contextmanager
def _pnp(medium='water', **params):
    import rclpy
    from rclpy.parameter import Parameter
    from mongla_localization.pnp_node import PnPNode
    started = not rclpy.ok()
    if started:
        rclpy.init()
    overrides = [Parameter('medium', value=medium)]
    overrides += [Parameter(k, value=v) for k, v in params.items()]
    node = PnPNode(parameter_overrides=overrides)
    try:
        yield node
    finally:
        node.destroy_node()
        if started:
            rclpy.shutdown()


def _corr_msg(obj_m, img_px, **kw):
    from mongla_interfaces.msg import TargetCorrespondences as TC
    m = TC()
    m.camera = kw.get('camera', 'forward')
    m.target_label = 'gate'
    m.source = 'test'
    m.optics = kw.get('optics', TC.OPTICS_RAW)
    m.image_width = kw.get('image_width', 0)
    m.image_height = kw.get('image_height', 0)
    obj = np.asarray(obj_m, np.float64).reshape(-1, 3)
    m.object_points = obj.reshape(-1).tolist()
    m.image_points = np.asarray(img_px, np.float64).reshape(-1).tolist()
    return m


def _feed_info(node, w=None, h=None):
    from sensor_msgs.msg import CameraInfo
    info = CameraInfo()
    info.width = int(w or (CX * 2))
    info.height = int(h or (CY * 2))
    info.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
    node._on_info(info)


def _square_obj_and_img(width_m, Z):
    """A planar square: object points in METRES, image points ray-traced."""
    w = width_m / 2.0
    corners = [(-w, -w), (w, -w), (w, w), (-w, w)]
    obj = np.array([[x, y, 0.0] for x, y in corners], np.float64)
    img = np.array([_project_flat_port(x, y, Z) for x, y in corners],
                   np.float64)
    return obj, img


@pytest.mark.parametrize('Z', [0.6, 1.0, 2.0])
def test_the_pose_survives_the_node_boundary_and_recovers_the_range(Z):
    """END TO END across the split, in metres -- the check the old one was not.

    This replaced a source-text assertion that `target_pose(ref_pts, live_pts,
    self._K_rect` appeared in `lock_node.py`. That guard was protecting the
    highest-consequence property of this whole item -- rectified pixels must be
    solved against the RECTIFIED K, or every range is a clean 1/n short with a
    plausible number and no warning -- and it could only ever prove that a line
    of source had not been retyped. Moving the solve into `pnp_node` broke it
    while changing nothing it cared about.

    So: ray-trace a square of known width through the flat port at a known
    range, hand the correspondences to the node exactly as `lock_node` sends
    them (raw pixels, `OPTICS_RAW`), and require the published range back in
    METRES. Nothing here can pass by sharing the rectifier's own approximation:
    the projection is an independent Snell bisection.
    """
    pytest.importorskip('rclpy')
    got = []
    with _pnp() as node:
        node._pub.publish = got.append
        _feed_info(node)
        obj, img = _square_obj_and_img(0.30, Z)
        node._on_corr(_corr_msg(obj, img))
    assert len(got) == 1, 'the node published nothing'
    tp = got[0]
    assert tp.ok, tp.reason
    assert abs(tp.range_m - Z) < 0.02 * Z, (
        f'range {tp.range_m:.4f} m for a true {Z} m -- the rectified points '
        f'are not being solved against the rectified K')


def test_the_air_K_would_have_been_caught_by_that_test():
    """The negative control: prove the check above can FAIL.

    A test that only ever sees the right answer cannot distinguish a correct
    pipeline from a broken one. Solving the same water pixels as if the camera
    were in air must read short by roughly the refractive index.
    """
    pytest.importorskip('rclpy')
    got = []
    with _pnp(medium='air') as node:
        node._pub.publish = got.append
        _feed_info(node)
        obj, img = _square_obj_and_img(0.30, 1.0)
        node._on_corr(_corr_msg(obj, img))
    tp = got[0]
    assert tp.ok, tp.reason
    assert tp.range_m < 0.85, (
        f'air optics on water pixels read {tp.range_m:.4f} m for a true 1.0 m '
        f'-- expected roughly 1/n short; if this passes at ~1.0 the water '
        f'path above is not testing anything')


def test_a_reduced_resolution_producer_is_not_silently_scaled():
    """The anchor matches in its BACKEND's pixels; CameraInfo is the full frame.

    Solving reduced pixels against a full-resolution K scales every recovered
    angle and range with no error and a plausible number -- the trap
    `lock_node._on_info` already documented, now living across a topic. The
    producer states its own frame; this proves the solver uses it.
    """
    pytest.importorskip('rclpy')
    obj, img_full = _square_obj_and_img(0.30, 1.0)
    half = img_full * 0.5                      # the same view at half scale

    def solve(**kw):
        got = []
        with _pnp() as node:
            node._pub.publish = got.append
            _feed_info(node)
            node._on_corr(_corr_msg(obj, half, **kw))
        return got[0]

    stated = solve(image_width=int(CX), image_height=int(CY))
    assert stated.ok, stated.reason
    assert abs(stated.range_m - 1.0) < 0.03, (
        f'declared half-resolution recovered {stated.range_m:.4f} m')
    silent = solve()                           # image_width 0 = "full frame"
    assert silent.range_m > 1.5, (
        f'half-resolution pixels solved as full frame gave {silent.range_m:.4f} '
        f'm -- if this is near 1.0 the scaling is not being applied at all')


def test_unset_optics_is_refused_rather_than_assumed():
    """There is no safe default. Guessing wrong is a 33 % range error."""
    pytest.importorskip('rclpy')
    from mongla_interfaces.msg import TargetCorrespondences as TC
    got = []
    with _pnp() as node:
        node._pub.publish = got.append
        _feed_info(node)
        obj, img = _square_obj_and_img(0.30, 1.0)
        node._on_corr(_corr_msg(obj, img, optics=TC.OPTICS_UNSET))
    assert got[0].ok is False
    assert 'optics' in got[0].reason


def test_already_rectified_points_are_not_rectified_twice():
    """`OPTICS_RECTIFIED` means the producer did it. Doing it again is the same
    class of error as not doing it at all, in the other direction."""
    pytest.importorskip('rclpy')
    from mongla_interfaces.msg import TargetCorrespondences as TC
    rect, _ = _rect_and_K()
    obj, img = _square_obj_and_img(0.30, 1.0)
    pre = np.asarray(rect.rectify(img), np.float64).reshape(-1, 2)
    got = []
    with _pnp() as node:
        node._pub.publish = got.append
        _feed_info(node)
        node._on_corr(_corr_msg(obj, pre, optics=TC.OPTICS_RECTIFIED))
    tp = got[0]
    assert tp.ok, tp.reason
    assert abs(tp.range_m - 1.0) < 0.02, (
        f'pre-rectified points recovered {tp.range_m:.4f} m -- the map was '
        f'applied twice')


def test_a_crossed_camera_is_refused():
    """The two cameras' udev names were bound backwards once and nothing
    noticed. A correspondence set names its camera; a mismatch is not solved."""
    pytest.importorskip('rclpy')
    got = []
    with _pnp() as node:
        node._pub.publish = got.append
        _feed_info(node)
        obj, img = _square_obj_and_img(0.30, 1.0)
        node._on_corr(_corr_msg(obj, img, camera='downward'))
    assert got[0].ok is False
    assert 'downward' in got[0].reason


def test_lock_node_publishes_evidence_and_no_longer_solves():
    """Two publishers of one claim is the defect this split removes."""
    src = (Path(__file__).resolve().parents[1] / 'mongla_vision'
           / 'lock_node.py').read_text()
    assert 'TargetCorrespondences' in src
    assert 'target_pose(' not in src, (
        'lock_node solves a pose again -- `pnp_node` is the solver, and two '
        'publishers of one claim is exactly what this split removed')
    assert "self.declare_parameter('medium', 'water')" in src


def test_camera_info_does_not_rebuild_the_rectifier_every_frame():
    """`CameraInfo` is published with every frame. Rebuilding the rectifier and
    logging on each one costs an allocation at camera rate and buries the log --
    measured on the vehicle: the medium line printed once per frame.

    Only a genuine change of intrinsics is an event.

    DRIVEN, not read off the source. The first version of this guard asserted
    that the literal `RefractiveRectifier(` appeared after the early return in
    `_on_info`. That broke the moment the pair-building moved into
    `optics.rectifier_for` -- where it belongs, so that `lock_node` and
    `pnp_node` cannot disagree about what optics a point set is in -- even
    though the behaviour it was protecting was untouched. A text guard fails on
    a refactor and passes on a regression it cannot see.
    """
    pytest.importorskip('rclpy')
    import numpy as np
    from sensor_msgs.msg import CameraInfo
    with _node() as node:
        # The anchor is off by default and `_on_info` returns early without
        # one, because K is scaled to the BACKEND's resolution. The stub
        # supplies only that resolution -- it stands in for two numbers, not
        # for the thing under test, which is the rectifier lifecycle.
        node._anchor = types.SimpleNamespace(
            _be=types.SimpleNamespace(w=320, h=240))
        info = CameraInfo()
        info.width, info.height = 640, 480
        info.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
        node._on_info(info)
        first = node._rect
        assert first is not None, 'water medium built no rectifier'
        node._on_info(info)
        assert node._rect is first, (
            'an identical CameraInfo rebuilt the rectifier -- CameraInfo '
            'arrives with every frame, so this allocates at camera rate and '
            'buries the medium log line')
        info.k = [640.0, 0.0, 320.0, 0.0, 640.0, 240.0, 0.0, 0.0, 1.0]
        node._on_info(info)
        assert node._rect is not first, (
            'genuinely new intrinsics must rebuild the rectifier')
        assert not np.array_equal(node._K_rect, np.eye(3))

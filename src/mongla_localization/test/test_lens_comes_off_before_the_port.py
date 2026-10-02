"""Every metric forward path removes the LENS before the PORT (issue #24).

The forward camera is strongly barrel-distorted (k1 = -0.363, measured). The
flow node undistorted before its port model; PnP, the lock anchor and every DSL
bearing read only `K` and fed raw pixels straight into the flat-port
rectifier, which assumes an ideal pinhole behind the glass. Off-centre poses
and bearings came out biased by degrees and ranges by tens of percent, with a
small reprojection error the whole time -- the solver fits distorted points
with a slightly wrong pose.

Truth here is constructed, not agreed: a board at a KNOWN pose is projected
by OpenCV's forward lens model (and, in water, a ray trace through an acrylic
port), and the node under test must recover the pose it was given. Nothing
below shares code with the inverse it checks.
"""
from __future__ import annotations

import contextlib
import json
import math
from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

_CAL = (Path(__file__).resolve().parents[2] / 'mongla_vision' / 'config'
        / 'calibration' / 'pi_forward_1280x720.json')
_cal = json.loads(_CAL.read_text())
K = np.asarray(_cal['camera_matrix'], float)
D = np.asarray(_cal['distortion_coefficients'], float)
W, H = int(_cal['image_width']), int(_cal['image_height'])

N_WATER = 1.333
NG, DG, D0 = 1.49, 0.006, 0.015      # acrylic index, glass thickness, air gap

RANGE_M = 3.0
# The issue's 30 deg is past the edge of the in-water field (53.6 deg wide), so
# water uses 20 deg -- the board's far edge still lands in the outer tenth.
BEARING_DEG = {'air': 30.0, 'water': 20.0}


def _board():
    """A 0.6 m square board as a 3x3 grid, in its own frame (z = 0)."""
    g = np.array([-0.3, 0.0, 0.3])
    return np.array([[x, y, 0.0] for y in g for x in g], float)


def _in_camera(obj, bearing_deg):
    """The board fronto-parallel, centred 3 m away at `bearing_deg`."""
    b = math.radians(bearing_deg)
    t = np.array([RANGE_M * math.sin(b), 0.0, RANGE_M * math.cos(b)])
    return obj + t


def _air_side(p):
    """Water point -> the AIR-side ray that images it, by ray trace through the
    port (bisection on the crossing height), returned as a point on that ray."""
    X, Y, Z = p
    R = math.hypot(X, Y)
    if R < 1e-12:
        return np.array([0.0, 0.0, 1.0])
    lo, hi = 0.0, R + D0 + 1.0
    for _ in range(200):
        h = 0.5 * (lo + hi)
        ta = math.atan2(h, D0)
        tg = math.asin(min(1.0, math.sin(ta) / NG))
        tw = math.asin(min(1.0, math.sin(ta) / N_WATER))
        hit = h + DG * math.tan(tg) + (Z - D0 - DG) * math.tan(tw)
        lo, hi = (h, hi) if hit < R else (lo, h)
    return np.array([X / R * h, Y / R * h, D0])


def _image(medium):
    pts = _in_camera(_board(), BEARING_DEG[medium])
    if medium == 'water':
        pts = np.array([_air_side(p) for p in pts])
    img, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), K, D)
    img = img.reshape(-1, 2)
    assert np.all((img[:, 0] > 0) & (img[:, 0] < W)), 'board must be in frame'
    return img


@contextlib.contextmanager
def _pnp(medium):
    import rclpy
    from rclpy.parameter import Parameter
    from mongla_localization.pnp_node import PnPNode
    started = not rclpy.ok()
    if started:
        rclpy.init()
    node = PnPNode(parameter_overrides=[Parameter('medium', value=medium)])
    got = []
    node._pub.publish = got.append
    try:
        yield node, got
    finally:
        node.destroy_node()
        if started:
            rclpy.shutdown()


def _info(with_d=True):
    from sensor_msgs.msg import CameraInfo
    m = CameraInfo()
    m.width, m.height = W, H
    m.k = [float(v) for v in K.reshape(-1)]
    m.d = [float(v) for v in D] if with_d else []
    return m


def _solve(medium, with_d=True):
    from mongla_interfaces.msg import TargetCorrespondences
    with _pnp(medium) as (node, got):
        node._on_info(_info(with_d))
        c = TargetCorrespondences()
        c.camera = 'forward'
        c.optics = TargetCorrespondences.OPTICS_RAW
        c.image_width, c.image_height = W, H
        c.object_points = [float(v) for v in _board().reshape(-1)]
        c.image_points = [float(v) for v in _image(medium).reshape(-1)]
        node._on_corr(c)
    assert got and got[-1].ok, got[-1].reason if got else 'nothing published'
    return got[-1]


@pytest.mark.parametrize('medium', ['air', 'water'])
def test_the_pose_node_recovers_an_off_axis_board(medium):
    """The issue's falsifier: 3 m ahead and well off-axis, through the measured
    lens (and port). Range within 2 %, bearing and yaw within 0.5 deg.

    Measured on this fixture: air 3.000 m / 30.00 deg / yaw 0.00; water
    3.005 m / 19.96 / -0.14 (the residual is the port glass, which the
    single-viewpoint port model neglects by design)."""
    tp = _solve(medium)
    assert tp.range_m == pytest.approx(RANGE_M, rel=0.02)
    assert abs(tp.bearing_deg) == pytest.approx(BEARING_DEG[medium], abs=0.5)
    # ⚠ 0.25, not the issue's 0.5: the ORDER is what this has to catch. At
    # f_ref = n * fx the port map is nearly the identity in pixels, so running
    # it BEFORE the lens moves an edge point only ~3 px -- measured here as yaw
    # -0.49 and reprojection 0.10 px, against -0.135 and 0.012 in order.
    assert abs(tp.yaw_deg) < 0.25


@pytest.mark.parametrize('medium', ['air', 'water'])
def test_without_the_lens_model_the_same_board_is_wrong(medium):
    """The defect, measured, so the guard above is known to be able to fail:
    drop `d` from CameraInfo and the same board comes back as air 3.285 m /
    27.08 deg / yaw -18.4, water 3.253 m / 18.29 / -18.9 -- a 9.5 % long range
    and an 18 deg tilt that is not there, with a passing reprojection gate."""
    tp = _solve(medium, with_d=False)
    off = (abs(tp.range_m / RANGE_M - 1.0) > 0.02
           or abs(abs(tp.bearing_deg) - BEARING_DEG[medium]) > 0.5
           or abs(tp.yaw_deg) > 0.5)
    assert off, (tp.range_m, tp.bearing_deg, tp.yaw_deg)


def test_a_changed_lens_rebuilds_the_optics():
    """CameraInfo arrives with every frame and the node caches on K. A new D
    under an unchanged K must still reach the solver."""
    with _pnp('air') as (node, _got):
        node._on_info(_info(with_d=False))
        rect0, _ = node._optics_for(W, H)
        node._on_info(_info(with_d=True))
        rect1, _ = node._optics_for(W, H)
    assert rect0 is None
    assert rect1 is not None


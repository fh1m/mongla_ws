"""Before CameraInfo arrives, a camera is modelled through its port and its
fixes say how much less they know (B-108).

The FOV fallback used to be one in-water focal, which a flat port does not
have: ranges and bearings went wrong by up to ~12 % at the frame edge while
the fix sent to the filter claimed the calibrated sigma. The truth here is
ray-traced through the port, not computed by the code under test.
"""
import math
from unittest.mock import MagicMock

import pytest

from mongla_localization.course_map import Course, Prop
from mongla_planner.mongla_dsl import MonglaMission

W, H = 640.0, 480.0
N = 1.333


def _fov_k(camera='forward'):
    """The FOV-derived air K, from the committed calibration's in-water FOV."""
    from mongla_vision.optics import fov_water_to_air
    hfov = fov_water_to_air(MonglaMission.HFOV_WATER_DEG_BY_CAMERA[camera])
    f = (W / 2.0) / math.tan(math.radians(hfov / 2.0))
    return f, f, W / 2.0, H / 2.0


def _through_port(X, Y, Z, fx, fy, cx, cy):
    rw = math.hypot(X / Z, Y / Z)
    if rw == 0.0:
        return cx, cy
    ra = math.tan(math.asin(N * math.sin(math.atan(rw))))
    return cx + fx * (X / Z) * ra / rw, cy + fy * (Y / Z) * ra / rw


def _uncalibrated(boxes):
    m = MagicMock()
    m.camera = 'forward'
    m._img_size = {}
    m._cam_k = {}
    m._cam_d = {}
    m.medium = 'water'
    m._records.return_value = boxes
    m.log = MagicMock()
    return m


def _gate_at(Z, X, width=1.5):
    fx, fy, cx, cy = _fov_k()
    ul, v = _through_port(X - width / 2.0, 0.0, Z, fx, fy, cx, cy)
    ur, _ = _through_port(X + width / 2.0, 0.0, Z, fx, fy, cx, cy)
    return ('gate', (ul + ur) / 2.0, v, ur - ul, 200.0, 0.9)


def test_an_off_axis_range_is_true_without_camera_info(monkeypatch):
    import mongla_vision.target_geometry as tg
    monkeypatch.setattr(tg, 'width_for', lambda name: 1.5)
    box = _gate_at(4.0, 1.4)
    got = MonglaMission.range_to(_uncalibrated([box]), 'gate')
    assert got[0] == pytest.approx(4.0, rel=0.01)
    # The single focal it replaced, on the same pixels: the case must tell
    # the two apart, or it proves nothing.
    f_one = (W / 2.0) / math.tan(math.radians(53.6 / 2.0))
    assert abs(f_one * 1.5 / box[3] - 4.0) / 4.0 > 0.05


def test_an_off_axis_bearing_is_true_without_camera_info():
    # One point, ray-traced: a box's edge midpoint is not the image of the
    # prop's centre through a port, so the box is centred on the point itself.
    fx, fy, cx, cy = _fov_k()
    u, v = _through_port(1.4, 0.6, 4.0, fx, fy, cx, cy)
    m = _uncalibrated([('gate', u, v, 60.0, 60.0, 0.9)])
    m.absolute_heading.return_value = 0.0
    m.where_offset.return_value = u / (W / 2.0) - 1.0
    got = MonglaMission.bearing_to(m, 'gate')
    truth = math.degrees(math.atan2(1.4, 4.0))
    assert got == pytest.approx(truth, abs=0.05)
    # The linear FOV reading it replaced misses by more than the tolerance.
    assert abs((u / (W / 2.0) - 1.0) * 53.6 / 2.0 - truth) > 0.5


def test_floor_range_refuses_rather_than_centre_the_principal_row():
    m = _uncalibrated([('drum', 320.0, 300.0, 40.0, 60.0, 0.8)])
    assert MonglaMission.floor_range(m, 'drum', plane_below_m=1.0, pitch_deg=0.0) is None
    assert 'no CameraInfo' in m.log.warning.call_args[0][0]


def _fixer(calibrated):
    m = MagicMock()
    m._course = Course('t', {'gate': Prop('gate', x_m=10.0, y_m=0.0,
                                          detect_class='gate', measured=True)})
    m._heading_offset = 0.0
    m.camera = 'forward'
    m.log = MagicMock()
    m.bearing_to.return_value = 0.0
    m.range_to.return_value = (4.0, 0.16)
    m._publish_fix = MagicMock()
    m._cam_k = {'forward': _fov_k()} if calibrated else {}
    m._img_size = {'forward': (W, H)} if calibrated else {}
    return m


def test_an_uncalibrated_fix_carries_the_principal_point_bias():
    m = _fixer(calibrated=False)
    assert MonglaMission.fix_from_prop(m, 'gate').ok
    sigma = m._publish_fix.call_args[1]['sigma']
    assert sigma == pytest.approx(math.hypot(0.16, 4.0 * math.radians(1.264)))
    assert 'uncalibrated' in m.log.info.call_args[0][0]


def test_a_calibrated_fix_keeps_the_range_sigma():
    m = _fixer(calibrated=True)
    MonglaMission.fix_from_prop(m, 'gate')
    assert m._publish_fix.call_args[1]['sigma'] == pytest.approx(0.16)


def test_placing_a_prop_uncalibrated_widens_the_bearing_term(monkeypatch):
    import mongla_localization.prop_placement as pp
    seen = {}
    real = pp.place

    def spy(sightings, **kw):
        seen.update(kw)
        return real(sightings, **kw)
    monkeypatch.setattr(pp, 'place', spy)
    m = _fixer(calibrated=False)
    c = [0.0] * 36
    c[0] = c[7] = 0.04
    c[35] = math.radians(1.0) ** 2
    m._odom = type('O', (), {'pose': type('P', (), {'covariance': c})()})()
    m.pose.return_value = (2.0, 0.0, -0.5, 0.0)
    frames = iter(range(1000))
    m.__dict__['_det_capture'] = {}

    def _bearing(*a, **k):
        m.__dict__['_det_capture']['forward'] = (next(frames), 0)
        return 0.0
    m.bearing_to.side_effect = _bearing
    MonglaMission.place_prop(m, 'gate', samples=3, timeout=2.0)
    assert seen['yaw_sigma_deg'] == pytest.approx(math.hypot(1.0, 1.264))

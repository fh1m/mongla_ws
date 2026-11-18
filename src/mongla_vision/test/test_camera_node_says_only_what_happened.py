"""The fps-override warning appears when there is an override, and only then.

It sat inside the V4L2 block, so every camera start on the vehicle logged
"fps override -> 0 (profile asked 210)" while nothing was overridden -- read
off Kutum's own log on 2026-11-18. A warning that fires on every start is one
an operator learns to ignore, which hides the real override it exists for.
"""
import types

import pytest

pytest.importorskip('rclpy')

from mongla_vision import camera_node  # noqa: E402


class _Log:
    def __init__(self):
        self.warnings = []

    def warning(self, m):
        self.warnings.append(m)

    warn = warning

    def info(self, m):
        pass

    def error(self, m):
        pass


def _build(monkeypatch, fps):
    params = {'device_path': '/dev/null', 'profile': 'pi_downward', 'name': '',
              'fps': fps, 'device': -1, 'controls': '', 'exposure_us': 0}
    log = _Log()
    me = types.SimpleNamespace(
        get_parameter=lambda n: types.SimpleNamespace(value=params[n]),
        get_logger=lambda: log)
    me._asked_controls = lambda base, *a: camera_node.CameraNode._asked_controls(me, base, *a)
    got = {}
    monkeypatch.setattr(camera_node, 'make_camera_from_profile',
                        lambda profile, **kw: got.setdefault('p', profile))
    camera_node.CameraNode._build_camera(me)
    return got['p'], [w for w in log.warnings if 'fps override' in w]


def test_no_override_no_warning(monkeypatch):
    profile, warned = _build(monkeypatch, 0)
    assert profile['fps'] == 210
    assert warned == []


def test_an_override_is_applied_and_said(monkeypatch):
    profile, warned = _build(monkeypatch, 60)
    assert profile['fps'] == 60
    assert len(warned) == 1 and '60' in warned[0] and '210' in warned[0]


def test_a_calibration_is_not_stretched_to_another_aspect_ratio():
    """16:9 calibration, 4:3 stream: size only, never a stretched K."""
    from sensor_msgs.msg import CameraInfo
    log = _Log()
    me = types.SimpleNamespace(
        _calib={'K': [1000.0, 0.0, 640.0, 0.0, 1000.0, 360.0, 0.0, 0.0, 1.0], 'D': [0.0] * 5,
                'w': 1280, 'h': 720},
        get_logger=lambda: log)
    info = CameraInfo(width=640, height=480)
    camera_node.CameraNode._fill_calibration(me, info)
    assert list(info.k) == [0.0] * 9 and log.warnings
    info = CameraInfo(width=640, height=360)
    camera_node.CameraNode._fill_calibration(me, info)
    assert info.k[0] == pytest.approx(500.0) and info.k[4] == pytest.approx(500.0)

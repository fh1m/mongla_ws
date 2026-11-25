"""The camera node applies, or says it did not apply, what it is given (audit vl_6 F2, F4, F5, F8)."""
import threading
import types

import pytest

pytest.importorskip('rclpy')

from mongla_vision import camera_node  # noqa: E402


class _Log:
    def __init__(self):
        self.lines = []

    def warning(self, m):
        self.lines.append(m)

    warn = info = error = debug = warning


def _me(params):
    log = _Log()
    me = types.SimpleNamespace(
        get_parameter=lambda n: types.SimpleNamespace(value=params[n]),
        get_logger=lambda: log)
    me._asked_controls = lambda base, *a: camera_node.CameraNode._asked_controls(me, base, *a)
    return me, log


def test_an_index_override_wins_over_the_profiles_symlink(monkeypatch):
    me, log = _me({'device_path': '', 'profile': 'pi_forward', 'name': '', 'fps': 0,
                   'device': 2, 'controls': '', 'exposure_us': 0,
                   'width': 640, 'height': 480})
    got = {}
    monkeypatch.setattr(camera_node, 'make_camera_from_profile',
                        lambda profile, **kw: got.setdefault('p', profile))
    camera_node.CameraNode._build_camera(me)
    assert got['p']['device'] == 2 and 'device_path' not in got['p']


def test_a_size_the_profile_ignores_is_said(monkeypatch):
    me, log = _me({'device_path': '', 'profile': 'pi_forward', 'name': '', 'fps': 0,
                   'device': -1, 'controls': '', 'exposure_us': 0,
                   'width': 1920, 'height': 1080})
    monkeypatch.setattr(camera_node, 'make_camera_from_profile', lambda p, **kw: p)
    camera_node.CameraNode._build_camera(me)
    assert any('1920x1080' in m for m in log.lines)


def test_no_invented_thirty_hz():
    src = open(camera_node.__file__).read()
    assert "get('fps') or 30.0" not in src


def test_a_control_set_during_a_reopen_is_refused(monkeypatch):
    monkeypatch.setattr(camera_node, 'changes', lambda node, ps: ps)
    me, _ = _me({})
    me.LIVE_PARAMS = camera_node.CameraNode.LIVE_PARAMS
    me.JPEG_RATES = camera_node.CameraNode.JPEG_RATES
    me._cam_lock = threading.Lock()
    me._cam = types.SimpleNamespace(set_controls=lambda w: w, auto_mode=None,
                                    source_kind='v4l2_mailbox')
    me._asked_controls = lambda *a: {'controls': {'brightness': 1}, 'exposure_us': 0}
    me._cam_lock.acquire()
    from rclpy.parameter import Parameter
    res = camera_node.CameraNode._on_set(me, [Parameter('controls', value='brightness=1')])
    assert not res.successful and 'reopen' in res.reason

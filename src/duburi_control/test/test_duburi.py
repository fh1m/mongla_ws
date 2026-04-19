"""Integration-ish tests for the Duburi facade.

We don't talk to a real autopilot -- a FakePixhawk records the calls
and reports a fixed attitude. Everything else (lock, return type, side
effects) is exercised through the real Duburi class.
"""

import logging
import math
import threading
import time

import pytest

from duburi_interfaces.action import Move

from duburi_control.duburi import Duburi


class ThrottleLogger:
    """Stdlib logging.Logger but tolerates rclpy's `throttle_duration_sec`.

    The motion_* modules call `log.info(msg, throttle_duration_sec=...)`
    which is rclpy-only. In tests we use a stdlib Logger so we have to
    drop the unknown kwarg before forwarding.
    """

    def __init__(self, inner):
        self._inner = inner

    def info(self, msg, *args, **kwargs):
        kwargs.pop('throttle_duration_sec', None)
        self._inner.info(msg, *args, **kwargs)

    def __getattr__(self, name):
        return getattr(self._inner, name)


class FakePixhawk:
    """Records calls; reports a fixed attitude.

    Keeps the surface intentionally narrow so tests fail loudly if
    Duburi reaches for a method we forgot to implement here.
    """

    def __init__(self, *, depth=-0.5, yaw=10.0, mode='ALT_HOLD'):
        self._attitude = {'yaw': yaw, 'pitch': 0.0, 'roll': 0.0, 'depth': depth}
        self._mode     = mode
        self._armed    = False
        self.calls     = []
        self.lock      = threading.Lock()

    def get_attitude(self):
        return dict(self._attitude)

    def get_mode(self):
        return self._mode

    def is_armed(self):
        return self._armed

    def send_neutral(self):
        self.calls.append(('send_neutral',))

    def release_rc_override(self):
        self.calls.append(('release_rc_override',))

    def send_rc_override(self, **kw):
        self.calls.append(('send_rc_override', kw))

    def send_rc_translation(self, **kw):
        self.calls.append(('send_rc_translation', kw))

    def set_attitude_setpoint(self, **kw):
        self.calls.append(('set_attitude_setpoint', kw))

    def set_target_depth(self, depth):
        self.calls.append(('set_target_depth', depth))
        self._attitude['depth'] = depth

    def arm(self, timeout=15.0):
        self._armed = True
        return True, 'ACCEPTED'

    def disarm(self, timeout=20.0):
        self._armed = False
        return True, 'ACCEPTED'

    def set_mode(self, name, timeout=8.0):
        self._mode = name
        return True, 'ACCEPTED'


@pytest.fixture
def duburi():
    pixhawk = FakePixhawk()
    log     = ThrottleLogger(logging.getLogger('test.duburi'))
    return Duburi(pixhawk, log)


def test_arm_returns_move_result(duburi):
    result = duburi.arm()
    assert isinstance(result, Move.Result)
    assert result.success is True
    assert result.message.startswith('arm:')


def test_set_mode_returns_move_result(duburi):
    result = duburi.set_mode('MANUAL')
    assert isinstance(result, Move.Result)
    assert result.success is True


def test_stop_sends_neutral_and_returns_result(duburi):
    result = duburi.stop(settle_time=0)
    assert ('send_neutral',) in duburi.pixhawk.calls
    assert result.success is True


def test_pause_releases_then_neutralises(duburi):
    result = duburi.pause(duration=0.05)
    kinds = [call[0] for call in duburi.pixhawk.calls]
    assert 'release_rc_override' in kinds
    # We always settle back to active hold afterwards so the next
    # command starts from a known state.
    assert kinds[-1] == 'send_neutral'
    assert result.success is True


def test_move_forward_short_burst_returns_depth(duburi, monkeypatch):
    # Hot loop: shrink any sleep so the test doesn't actually wait.
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    result = duburi.move_forward(duration=0.05, gain=50.0)
    assert isinstance(result, Move.Result)
    assert result.success is True
    assert math.isclose(result.final_value, -0.5, abs_tol=0.01)

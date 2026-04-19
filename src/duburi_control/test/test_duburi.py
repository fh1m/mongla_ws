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


def test_arc_dispatches_to_motion_forward(duburi, monkeypatch):
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    result = duburi.arc(duration=0.05, gain=50.0, yaw_rate_pct=30.0)
    assert isinstance(result, Move.Result)
    assert result.success is True
    # arc emits packets with both forward + yaw -- this is the
    # single-packet contract that makes curved motion possible.
    arc_packets = [
        c for c in duburi.pixhawk.calls
        if c[0] == 'send_rc_override' and 'forward' in c[1] and 'yaw' in c[1]]
    assert arc_packets, 'arc must emit Ch5 + Ch4 in the same packet'


def test_lock_heading_starts_thread_and_unlock_stops(duburi, monkeypatch):
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    assert duburi._heading_lock is None
    duburi.lock_heading(target=90.0, timeout=2.0)
    assert duburi._heading_lock is not None
    duburi.unlock_heading()
    assert duburi._heading_lock is None


def test_lock_heading_target_zero_locks_current_heading(duburi, monkeypatch):
    """target=0.0 (rosidl unset) means "lock at current heading"."""
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    result = duburi.lock_heading(target=0.0, timeout=2.0)
    # Fixture FakePixhawk reports yaw=10.0, so locked target should be 10.0.
    assert math.isclose(result.final_value, 10.0, abs_tol=0.1)
    duburi.unlock_heading()


def test_lock_depth_starts_thread_and_unlock_stops(duburi, monkeypatch):
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    assert duburi._depth_lock is None
    duburi.lock_depth(target=-1.5, timeout=2.0)
    assert duburi._depth_lock is not None
    duburi.unlock_depth()
    assert duburi._depth_lock is None


def test_lock_depth_target_zero_locks_current_depth(duburi, monkeypatch):
    """target=0.0 (rosidl unset) means "lock at current depth"."""
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    result = duburi.lock_depth(target=0.0, timeout=2.0)
    # Fixture FakePixhawk reports depth=-0.5, so the locked target
    # should match.
    assert math.isclose(result.final_value, -0.5, abs_tol=1e-3)
    duburi.unlock_depth()


def test_set_depth_retargets_active_depth_lock(duburi, monkeypatch):
    """If a lock_depth is running, set_depth must update the lock's target.

    Otherwise the lock would keep streaming the OLD setpoint and fight
    the just-completed dive.
    """
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    duburi.lock_depth(target=-0.5, timeout=5.0)
    duburi.set_depth(target=-1.5, timeout=2.0)
    assert duburi._depth_lock is not None
    assert math.isclose(duburi._depth_lock.target_m, -1.5, abs_tol=1e-3), (
        'set_depth must retarget the active lock_depth so the streamer '
        'follows the freshly commanded depth')
    duburi.unlock_depth()


def test_set_depth_engages_alt_hold(duburi, monkeypatch):
    """set_depth must guarantee ALT_HOLD is engaged before driving."""
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    duburi.pixhawk._mode = 'MANUAL'
    duburi.set_depth(target=-1.0, timeout=2.0)
    assert duburi.pixhawk._mode == 'ALT_HOLD', (
        'set_depth must auto-engage ALT_HOLD; otherwise ArduSub silently '
        'drops the depth setpoint')


def test_motion_uses_translation_when_lock_active(duburi, monkeypatch):
    """When heading-lock is engaged, motion commands MUST stay off Ch4.

    With the rate-based HeadingLock the LOCK thread now writes Ch4
    itself (``send_rc_override(yaw=...)``). Translation commands
    (``move_forward``, ``move_lateral``) must keep using
    ``send_rc_translation`` which touches Ch5/Ch6/Ch3 only; any Ch4
    write from the motion path would fight the lock.
    """
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    duburi.lock_heading(target=0.0, timeout=2.0)
    duburi.pixhawk.calls.clear()
    duburi.move_forward(duration=0.05, gain=50.0)
    # Ignore packets the lock thread itself is emitting in the background
    # -- those are *expected* Ch4 writes. We only care about packets that
    # touch forward/lateral/throttle here, i.e. translation packets.
    translation_calls = [c for c in duburi.pixhawk.calls
                         if c[0] == 'send_rc_translation']
    motion_yaw_overrides = [
        c for c in duburi.pixhawk.calls
        if c[0] == 'send_rc_override'
        and ('forward' in c[1] or 'lateral' in c[1] or 'throttle' in c[1])]
    duburi.unlock_heading()
    assert not motion_yaw_overrides, (
        'with heading-lock active, translation commands must NOT call '
        'send_rc_override for fwd/lat/throttle (they would clobber Ch4)')
    assert translation_calls, 'move_forward must emit translation packets'

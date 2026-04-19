"""Tests for motion_forward (Ch5 forward/back + Ch5+Ch4 arc)."""

import logging
import time

import pytest

from duburi_control.motion_writers import make_writers
from duburi_control.motion_forward import (
    arc, drive_forward_constant, drive_forward_eased,
)


class ThrottleLogger:
    """Stdlib logger that tolerates rclpy's `throttle_duration_sec`."""
    def __init__(self):
        self._inner = logging.getLogger('test.motion_forward')

    def info(self, msg, *args, **kwargs):
        kwargs.pop('throttle_duration_sec', None)
        self._inner.info(msg, *args, **kwargs)

    def __getattr__(self, name):
        return getattr(self._inner, name)


class FakePixhawk:
    def __init__(self):
        self.calls = []
        self._attitude = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
                          'depth': -0.5}

    def get_attitude(self):
        return dict(self._attitude)

    def send_neutral(self):
        self.calls.append(('send_neutral',))

    def send_rc_override(self, **kw):
        self.calls.append(('send_rc_override', kw))

    def send_rc_translation(self, **kw):
        self.calls.append(('send_rc_translation', kw))


@pytest.fixture
def patched_sleep(monkeypatch):
    monkeypatch.setattr(time, 'sleep', lambda *_: None)


def test_drive_forward_constant_writes_forward_only(patched_sleep):
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()
    writers = make_writers(pixhawk, release_yaw=False)

    drive_forward_constant(pixhawk, +1, duration=0.1, gain=60, log=log,
                           writers=writers, settle=0.0)

    forward_writes = [c for c in pixhawk.calls
                      if c[0] == 'send_rc_override' and 'forward' in c[1]]
    assert forward_writes, 'expected forward-channel writes'
    pwms = [c[1]['forward'] for c in forward_writes]
    # 60% forward -> percent_to_pwm(60) = 1740
    assert 1700 <= max(pwms) <= 1900


def test_drive_forward_eased_uses_translation_when_lock_active(patched_sleep):
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()
    writers = make_writers(pixhawk, release_yaw=True)

    drive_forward_eased(pixhawk, +1, duration=0.1, gain=60, log=log,
                        writers=writers, settle=0.0)

    # All channel writes should be send_rc_translation, not send_rc_override.
    overrides = [c for c in pixhawk.calls if c[0] == 'send_rc_override']
    translations = [c for c in pixhawk.calls
                    if c[0] == 'send_rc_translation']
    assert not overrides, (
        'release_yaw=True must NOT emit send_rc_override (Ch4 would clobber lock)')
    assert translations


def test_arc_writes_ch5_and_ch4_in_same_packet(patched_sleep):
    """Critical: arc's whole point is the simultaneous Ch5 + Ch4 packet."""
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()

    arc(pixhawk, signed_dir=+1, duration=0.1, gain=50,
        yaw_rate_pct=30.0, log=log, settle=0.0)

    arc_writes = [c for c in pixhawk.calls
                  if c[0] == 'send_rc_override'
                  and 'forward' in c[1] and 'yaw' in c[1]]
    assert arc_writes, (
        'arc must emit RC override packets containing BOTH forward and yaw')
    # Forward should be positive thrust, yaw should be positive (right turn)
    last = arc_writes[-1][1]
    assert last['forward'] > 1500
    assert last['yaw']     > 1500


def test_arc_negative_yaw_rate_turns_left(patched_sleep):
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()

    arc(pixhawk, signed_dir=+1, duration=0.1, gain=50,
        yaw_rate_pct=-40.0, log=log, settle=0.0)

    arc_writes = [c for c in pixhawk.calls
                  if c[0] == 'send_rc_override'
                  and 'forward' in c[1] and 'yaw' in c[1]]
    assert arc_writes
    last = arc_writes[-1][1]
    assert last['yaw'] < 1500   # left turn -> yaw stick below neutral

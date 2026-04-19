"""Tests for motion_lateral (Ch6 strafe)."""

import logging
import time

import pytest

from duburi_control.motion_common import make_writers
from duburi_control.motion_lateral import (
    drive_lateral_constant, drive_lateral_eased,
)


class ThrottleLogger:
    def __init__(self):
        self._inner = logging.getLogger('test.motion_lateral')

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


def test_drive_lateral_constant_writes_lateral_only(patched_sleep):
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()
    writers = make_writers(pixhawk, release_yaw=False)

    drive_lateral_constant(pixhawk, +1, duration=0.1, gain=60, log=log,
                           writers=writers, settle=0.0)

    lateral_writes = [c for c in pixhawk.calls
                      if c[0] == 'send_rc_override' and 'lateral' in c[1]]
    assert lateral_writes, 'expected lateral-channel writes'
    pwms = [c[1]['lateral'] for c in lateral_writes]
    # +60% lateral -> 1740
    assert 1700 <= max(pwms) <= 1900


def test_drive_lateral_left_signed_dir_negative(patched_sleep):
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()
    writers = make_writers(pixhawk, release_yaw=False)

    drive_lateral_eased(pixhawk, -1, duration=0.1, gain=60, log=log,
                        writers=writers, settle=0.0)

    lateral_writes = [c for c in pixhawk.calls
                      if c[0] == 'send_rc_override' and 'lateral' in c[1]]
    assert lateral_writes
    pwms = [c[1]['lateral'] for c in lateral_writes]
    assert min(pwms) < 1500   # leftward strafe = lateral PWM below neutral


def test_drive_lateral_eased_uses_translation_when_lock_active(patched_sleep):
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()
    writers = make_writers(pixhawk, release_yaw=True)

    drive_lateral_eased(pixhawk, +1, duration=0.1, gain=60, log=log,
                        writers=writers, settle=0.0)

    overrides = [c for c in pixhawk.calls if c[0] == 'send_rc_override']
    translations = [c for c in pixhawk.calls
                    if c[0] == 'send_rc_translation']
    assert not overrides
    assert translations

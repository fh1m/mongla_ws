"""Tests for heading_lock (background SET_ATTITUDE_TARGET streamer)."""

import logging
import time

import pytest

from duburi_control.heading_lock import HeadingLock


class ThrottleLogger:
    def __init__(self):
        self._inner = logging.getLogger('test.heading_lock')

    def info(self, msg, *args, **kwargs):
        kwargs.pop('throttle_duration_sec', None)
        self._inner.info(msg, *args, **kwargs)

    def warn(self, msg, *args, **kwargs):
        self._inner.warning(msg, *args, **kwargs)

    def __getattr__(self, name):
        return getattr(self._inner, name)


class FakePixhawk:
    def __init__(self, yaw=10.0):
        self.attitude_targets = []
        self._yaw = yaw

    def set_attitude_setpoint(self, **kw):
        self.attitude_targets.append(kw)

    def get_attitude(self):
        return {'yaw': self._yaw, 'depth': -0.5,
                'roll': 0.0, 'pitch': 0.0}


class FakeYawSource:
    """Drop-in YawSource. `name` is what HeadingLock prints in logs."""
    name = 'fake_source'

    def __init__(self, value=42.0):
        self._v = value

    def read_yaw(self):
        return self._v


def test_heading_lock_streams_target_to_pixhawk():
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()
    src     = FakeYawSource(value=90.0)

    lock = HeadingLock(pixhawk, target_deg=90.0, yaw_source=src, log=log,
                       timeout=2.0)
    lock.start()
    time.sleep(0.2)
    lock.stop()

    assert pixhawk.attitude_targets, 'lock thread must stream attitude targets'
    yaw_targets = [t['yaw_deg'] for t in pixhawk.attitude_targets]
    assert all(abs(t - 90.0) < 1e-6 for t in yaw_targets)


def test_heading_lock_retarget_swaps_value():
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()
    src     = FakeYawSource()

    lock = HeadingLock(pixhawk, target_deg=0.0, yaw_source=src, log=log,
                       timeout=2.0)
    lock.start()
    time.sleep(0.15)
    lock.retarget(180.0)
    time.sleep(0.2)
    lock.stop()

    yaw_targets = [t['yaw_deg'] for t in pixhawk.attitude_targets]
    # First samples are 0, later samples are 180
    assert any(abs(y - 0.0)   < 1e-6 for y in yaw_targets)
    assert any(abs(y - 180.0) < 1e-6 for y in yaw_targets)


def test_heading_lock_suspend_pauses_streaming():
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()
    src     = FakeYawSource()

    lock = HeadingLock(pixhawk, target_deg=0.0, yaw_source=src, log=log,
                       timeout=2.0)
    lock.start()
    time.sleep(0.15)
    pre_count = len(pixhawk.attitude_targets)
    lock.suspend()
    time.sleep(0.25)
    post_count = len(pixhawk.attitude_targets)
    lock.resume()
    time.sleep(0.15)
    final_count = len(pixhawk.attitude_targets)
    lock.stop()

    # Suspended -> at most 1-2 in-flight packets after suspend.
    assert post_count - pre_count <= 2, (
        f'suspend should stop streaming, but got '
        f'{post_count - pre_count} extra packets')
    assert final_count > post_count


def test_heading_lock_works_with_no_yaw_source():
    """yaw_source=None -> falls back to AHRS via pixhawk.get_attitude.

    Verifies the source-agnostic plug: same lock object works in
    Gazebo/SITL with `mavlink_ahrs` (no external IMU).
    """
    pixhawk = FakePixhawk(yaw=42.0)
    log     = ThrottleLogger()

    lock = HeadingLock(pixhawk, target_deg=42.0, yaw_source=None, log=log,
                       timeout=2.0)
    lock.start()
    time.sleep(0.15)
    lock.stop()

    assert pixhawk.attitude_targets

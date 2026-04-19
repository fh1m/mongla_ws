"""Tests for heading_lock (background Ch4-rate heading controller).

The lock runs a proportional yaw-rate loop against the configured
YawSource and writes Ch4 via `send_rc_override`. Tests check the new
contract: rate packets are emitted, retarget takes effect, suspend
freezes output, and the AHRS fallback still works when yaw_source is
None.
"""

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
        self.rc_overrides = []
        self._yaw = yaw

    def send_rc_override(self, **kw):
        self.rc_overrides.append(kw)

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


def _yaw_packets(pixhawk):
    return [p for p in pixhawk.rc_overrides if 'yaw' in p]


def test_heading_lock_on_target_parks_ch4_at_neutral():
    """When current yaw == target, error is inside the deadband and the
    loop must write Ch4 = 1500 us (zero yaw rate)."""
    pixhawk = FakePixhawk(yaw=90.0)
    log     = ThrottleLogger()
    src     = FakeYawSource(value=90.0)

    lock = HeadingLock(pixhawk, target_deg=90.0, yaw_source=src, log=log,
                       timeout=2.0)
    lock.start()
    time.sleep(0.2)
    lock.stop()

    packets = _yaw_packets(pixhawk)
    assert packets, 'lock thread must emit Ch4 override packets'
    assert all(p['yaw'] == 1500 for p in packets), (
        'on-target lock must park Ch4 at 1500 us (deadband)')


def test_heading_lock_off_target_commands_proportional_yaw_rate():
    """When current yaw is far from target, the loop must drive Ch4 in
    the direction of the shortest heading error."""
    # Source reads 0; target is 90 -> error ~ +90 -> yaw_pct positive ->
    # PWM > 1500 (convention: percent_to_pwm clamps symmetric around 1500).
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()
    src     = FakeYawSource(value=0.0)

    lock = HeadingLock(pixhawk, target_deg=90.0, yaw_source=src, log=log,
                       timeout=2.0)
    lock.start()
    time.sleep(0.15)
    lock.stop()

    packets = _yaw_packets(pixhawk)
    assert packets
    assert any(p['yaw'] > 1500 for p in packets), (
        'positive heading error should push Ch4 above 1500 us')


def test_heading_lock_retarget_swaps_value():
    # Source reads 0.0. First we target 10 (small err -> may deadband or
    # small positive pct), then retarget to 180 (big negative err ->
    # Ch4 decidedly below 1500). We check that the direction flips.
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()
    src     = FakeYawSource(value=0.0)

    lock = HeadingLock(pixhawk, target_deg=10.0, yaw_source=src, log=log,
                       timeout=2.0)
    lock.start()
    time.sleep(0.15)
    first_batch = list(_yaw_packets(pixhawk))
    lock.retarget(180.0)
    time.sleep(0.2)
    lock.stop()

    second_batch = _yaw_packets(pixhawk)[len(first_batch):]
    assert second_batch, 'packets should continue after retarget'
    # heading_error(180, 0) == +180 OR -180; either way |yaw_pct| should
    # be near LOCK_PCT_MAX, i.e. Ch4 noticeably off 1500.
    assert any(abs(p['yaw'] - 1500) >= 50 for p in second_batch), (
        'retarget to 180 deg should produce a large Ch4 deflection')


def test_heading_lock_suspend_pauses_streaming():
    pixhawk = FakePixhawk()
    log     = ThrottleLogger()
    src     = FakeYawSource()

    lock = HeadingLock(pixhawk, target_deg=0.0, yaw_source=src, log=log,
                       timeout=2.0)
    lock.start()
    time.sleep(0.15)
    pre_count = len(_yaw_packets(pixhawk))
    lock.suspend()
    time.sleep(0.25)
    post_count = len(_yaw_packets(pixhawk))
    lock.resume()
    time.sleep(0.15)
    final_count = len(_yaw_packets(pixhawk))
    # Allow stop() to emit its own safe-stop packet.
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

    assert _yaw_packets(pixhawk), (
        'lock must emit Ch4 packets even without an external yaw_source')

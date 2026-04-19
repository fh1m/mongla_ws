"""Tests for depth_lock (background set_target_depth streamer).

Mirror of test_heading_lock. The lock streams ``set_target_depth`` at
5 Hz; tests verify the public contract: setpoint packets are emitted,
retarget swaps the value, suspend freezes output, and the AHRS-silent
watchdog exits cleanly when telemetry stops.
"""

import logging
import time

from duburi_control.depth_lock import DepthLock


class ThrottleLogger:
    """Stdlib logger that tolerates rclpy's `throttle_duration_sec`."""

    def __init__(self):
        self._inner = logging.getLogger('test.depth_lock')

    def info(self, msg, *args, **kwargs):
        kwargs.pop('throttle_duration_sec', None)
        self._inner.info(msg, *args, **kwargs)

    def warn(self, msg, *args, **kwargs):
        self._inner.warning(msg, *args, **kwargs)

    def __getattr__(self, name):
        return getattr(self._inner, name)


class FakePixhawk:
    def __init__(self, depth=-0.5):
        self.depth_setpoints = []
        self._depth = depth
        self._attitude_alive = True

    def set_target_depth(self, depth_m):
        self.depth_setpoints.append(float(depth_m))

    def get_attitude(self):
        if not self._attitude_alive:
            return None
        return {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
                'depth': self._depth}


def test_depth_lock_streams_setpoint_at_target():
    pixhawk = FakePixhawk(depth=-0.5)
    log = ThrottleLogger()

    lock = DepthLock(pixhawk, target_m=-0.5, log=log, timeout=2.0)
    lock.start()
    time.sleep(0.5)
    lock.stop()

    assert pixhawk.depth_setpoints, (
        'lock thread must emit set_target_depth packets')
    assert all(abs(d - (-0.5)) < 1e-6 for d in pixhawk.depth_setpoints), (
        'on-target lock must keep streaming the same setpoint')


def test_depth_lock_retarget_swaps_value():
    pixhawk = FakePixhawk(depth=-0.5)
    log = ThrottleLogger()

    lock = DepthLock(pixhawk, target_m=-0.5, log=log, timeout=2.0)
    lock.start()
    time.sleep(0.4)
    pre_count = len(pixhawk.depth_setpoints)
    lock.retarget(-1.5)
    time.sleep(0.6)
    lock.stop()

    after_retarget = pixhawk.depth_setpoints[pre_count:]
    assert after_retarget, 'packets should continue after retarget'
    assert any(abs(d - (-1.5)) < 1e-6 for d in after_retarget), (
        'retarget(-1.5) must change the streamed setpoint to -1.5m')


def test_depth_lock_suspend_pauses_streaming():
    pixhawk = FakePixhawk(depth=-0.5)
    log = ThrottleLogger()

    lock = DepthLock(pixhawk, target_m=-0.5, log=log, timeout=2.0)
    lock.start()
    time.sleep(0.4)
    pre_count = len(pixhawk.depth_setpoints)
    lock.suspend()
    time.sleep(0.6)
    post_count = len(pixhawk.depth_setpoints)
    lock.resume()
    time.sleep(0.4)
    final_count = len(pixhawk.depth_setpoints)
    lock.stop()

    # Suspended -> at most ~1 in-flight packet right after suspend.
    assert post_count - pre_count <= 1, (
        f'suspend should stop streaming, but got '
        f'{post_count - pre_count} extra packets')
    assert final_count > post_count, (
        'resume must restart the streaming')


def test_depth_lock_watchdog_releases_when_ahrs_silent():
    pixhawk = FakePixhawk(depth=-0.5)
    log = ThrottleLogger()

    lock = DepthLock(pixhawk, target_m=-0.5, log=log, timeout=10.0)
    pixhawk._attitude_alive = False    # AHRS depth telemetry "dies"
    lock.start()
    # SOURCE_DEAD_S = 2.0 -- give the loop enough wall-clock to notice.
    time.sleep(2.6)
    lock.stop()
    assert not lock._thread.is_alive(), (
        'watchdog must exit the streamer when AHRS goes silent')

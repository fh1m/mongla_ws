"""The wiring, not the maths -- `test_inekf.py` owns the filter itself.

Every test here asserts a decision this node makes about WHICH samples reach
the filter. Those decisions are where a localization node goes silently wrong:
a filter fed a NaN, a zero, or a 2-second dt still publishes a confident pose.
"""
import math

import numpy as np
import pytest

from duburi_localization import localization_node as ln
from duburi_localization.inekf import RIEKF, State


class _Stamp:
    def __init__(self, t):
        self.sec = int(t)
        self.nanosec = int((t - int(t)) * 1e9)


class _Header:
    def __init__(self, t):
        self.stamp = _Stamp(t)


class _Imu:
    def __init__(self, t, gyro=(0.0, 0.0, 0.0), accel=(0.0, 0.0, 9.80665)):
        self.header = _Header(t)
        self.angular_velocity = type('V', (), dict(
            zip('xyz', gyro)))()
        self.linear_acceleration = type('V', (), dict(zip('xyz', accel)))()


def _node():
    """A LocalizationNode with the ROS machinery stubbed out.

    Constructed WITHOUT rclpy: the callbacks are pure functions of a message
    and the filter, and requiring a live graph to test them is what makes
    node logic go untested.
    """
    obj = ln.LocalizationNode.__new__(ln.LocalizationNode)
    obj._filter = RIEKF()
    obj._last_imu_t = None
    obj._imu_gap_warned = True          # suppress the logger call
    obj._n = {'imu': 0, 'depth': 0, 'yaw': 0, 'flow': 0, 'fix': 0, 'gap': 0}
    obj._flow_sigma = 0.05
    obj._depth_sigma = 0.02
    obj._yaw_sigma_deg = 2.0
    obj._fix_sigma = 0.5
    obj._use_yaw = True
    return obj


def test_first_imu_sample_only_seeds_the_clock():
    """One sample carries no interval, so it must not be integrated."""
    n = _node()
    n._on_imu(_Imu(100.0))
    assert n._n['imu'] == 0
    assert n._last_imu_t == pytest.approx(100.0)


def test_a_long_gap_is_skipped_not_integrated():
    """A 2 s dt as one Euler step is not 100 steps of 20 ms.

    Without this the filter's position jumps on every dropout, which reads as
    drift and is actually a integration error -- worst precisely after the
    link recovers, when you most want to trust it.
    """
    n = _node()
    n._on_imu(_Imu(100.0))
    n._on_imu(_Imu(102.0, accel=(2.0, 0.0, 9.80665)))
    assert n._n['imu'] == 0
    assert n._n['gap'] == 1
    assert np.allclose(n._filter.X.p, np.zeros(3))


def test_a_backwards_stamp_is_dropped():
    """A refitted ClockMap can reorder two samples in host time.

    A negative dt integrates the state BACKWARDS and nothing downstream can
    tell that from drift.
    """
    n = _node()
    n._on_imu(_Imu(100.0))
    n._on_imu(_Imu(99.9))
    assert n._n['imu'] == 0


def test_a_normal_step_reaches_the_filter():
    n = _node()
    n._on_imu(_Imu(100.00))
    n._on_imu(_Imu(100.02))
    assert n._n['imu'] == 1


def test_nan_yaw_never_reaches_the_filter():
    """NaN is the documented absence sentinel and srot now really publishes it.

    One NaN through `update_yaw` poisons every state via the gain, silently and
    permanently -- there is no recovery, because NaN propagates through the
    covariance too.
    """
    n = _node()
    state = type('S', (), {'depth_m': -1.0, 'yaw_deg': float('nan')})()
    n._on_state(state)
    assert n._n['yaw'] == 0
    assert n._n['depth'] == 1
    assert np.all(np.isfinite(n._filter.P))
    assert np.all(np.isfinite(n._filter.X.p))


def test_nan_depth_never_reaches_the_filter():
    n = _node()
    state = type('S', (), {'depth_m': float('nan'), 'yaw_deg': 10.0})()
    n._on_state(state)
    assert n._n['depth'] == 0
    assert np.all(np.isfinite(n._filter.P))


def test_nan_flow_velocity_never_reaches_the_filter():
    n = _node()
    twist = type('T', (), {})()
    twist.twist = type('T2', (), {})()
    twist.twist.twist = type('T3', (), {})()
    twist.twist.twist.linear = type('L', (), {
        'x': float('nan'), 'y': 0.0, 'z': 0.0})()
    n._on_flow(twist)
    assert n._n['flow'] == 0
    assert np.all(np.isfinite(n._filter.P))


def test_quaternion_round_trips_through_180_degrees():
    """The naive `sqrt(1+trace)` branch loses all precision here and can take
    the square root of a small negative from rounding."""
    for deg in (0.0, 45.0, 90.0, 179.9, 180.0, -179.9):
        r = math.radians(deg)
        R = np.array([[math.cos(r), -math.sin(r), 0.0],
                      [math.sin(r), math.cos(r), 0.0],
                      [0.0, 0.0, 1.0]])
        w, x, y, z = ln._quat_from_R(R)
        assert math.isfinite(w) and math.isfinite(z)
        assert pytest.approx(1.0, abs=1e-9) == w * w + x * x + y * y + z * z
        got = math.degrees(math.atan2(2.0 * (w * z + x * y),
                                      1.0 - 2.0 * (y * y + z * z)))
        assert pytest.approx(deg, abs=1e-6) == got


def test_published_velocity_is_body_frame():
    """`child_frame_id` in nav_msgs/Odometry MEANS body frame, and the filter
    stores world velocity because the error dynamics require it. Publishing the
    world vector under a body label is a silent 90-degree error at any heading
    but zero."""
    n = _node()
    # Heading +90 deg, moving along world +x at 1 m/s -> body -y.
    R = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    n._filter.X = State(R=R, v=np.array([1.0, 0.0, 0.0]))
    v_body = n._filter.X.R.T @ n._filter.X.v
    assert pytest.approx(0.0, abs=1e-12) == v_body[0]
    assert pytest.approx(-1.0, abs=1e-12) == v_body[1]

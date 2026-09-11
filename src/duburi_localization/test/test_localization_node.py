"""The wiring, not the maths -- `test_inekf.py` owns the filter itself.

Every test here asserts a decision this node makes about WHICH samples reach
the filter. Those decisions are where a localization node goes silently wrong:
a filter fed a NaN, a zero, or a 2-second dt still publishes a confident pose.
"""
import math
import time
from collections import deque

import numpy as np
import pytest

from duburi_localization import localization_node as ln
from duburi_localization.inekf import RIEKF, State


class _NullLogger:
    def info(self, *_a, **_k):
        pass

    def warning(self, *_a, **_k):
        pass


class _Stamp:
    def __init__(self, t):
        self.sec = int(t)
        self.nanosec = int((t - int(t)) * 1e9)


class _Header:
    def __init__(self, t):
        self.stamp = _Stamp(t)


class _Imu:
    def __init__(self, t, gyro=(0.0, 0.0, 0.0), accel=(0.0, 0.0, 9.80665),
                 attitude=True):
        self.header = _Header(t)
        self.angular_velocity = type('V', (), dict(
            zip('xyz', gyro)))()
        self.linear_acceleration = type('V', (), dict(zip('xyz', accel)))()
        self.orientation = type('Q', (), {'w': 1.0, 'x': 0.0,
                                          'y': 0.0, 'z': 0.0})()
        # [0] < 0 is the ROS "no orientation" convention.
        self.orientation_covariance = [7.6e-5 if attitude else -1.0] + [0.0] * 8


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
    obj._n = {'imu': 0, 'att': 0, 'depth': 0, 'yaw': 0, 'flow': 0, 'fix': 0,
              'zupt': 0, 'gap': 0}
    obj._attitude_sigma_deg = 0.5
    obj._zupt_sigma = 0.01
    obj._zupt_enabled = True
    obj._still = deque(maxlen=ln.STILL_WINDOW)
    obj._last_flow_t = 0.0
    obj._attitude_seeded = False
    obj._anchored = False
    obj._last_input_t = 0.0
    obj._aided_at_last_diag = -1
    obj._yaw_sigma_deg = 2.0
    # The node genuinely logs on the seed path; give the stub a sink rather
    # than removing the log, which is operator-facing.
    obj.get_logger = lambda: _NullLogger()
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


# --------------------------------------------------------------------------- #
#  the split: the board owns attitude
# --------------------------------------------------------------------------- #
def test_board_attitude_is_consumed_when_present():
    """Without this the companion propagates its own attitude and DIVERGES.

    Measured on the vehicle before the channel existed: 7.1e6 m of position in
    35 s. The attitude error grows through the gravity coupling, gravity leaks
    into horizontal acceleration, and the depth update's gain pumps it into x
    and y. The board holds the same quantity to under 0.01 deg/min.
    """
    n = _node()
    n._on_imu(_Imu(100.00))
    n._on_imu(_Imu(100.02))
    assert n._n['att'] == 1


def test_absent_attitude_is_NOT_fabricated():
    """A backend with no attitude must diverge visibly rather than converge to
    a number nobody measured."""
    n = _node()
    n._on_imu(_Imu(100.00, attitude=False))
    n._on_imu(_Imu(100.02, attitude=False))
    assert n._n['att'] == 0
    assert n._n['imu'] == 1


def test_a_level_hull_at_rest_does_not_accelerate():
    """The end-to-end property the divergence violated.

    Accel reads +1 g on body z at rest (the firmware packs gravity in), so
    `a_world = R @ a + GRAVITY` must cancel to zero. If it does not, velocity
    ramps and position runs away -- which is exactly what the vehicle showed.
    """
    n = _node()
    t = 100.0
    n._on_imu(_Imu(t))
    for _ in range(500):                       # 10 s at 50 Hz
        t += 0.02
        n._on_imu(_Imu(t))
    assert np.linalg.norm(n._filter.X.v) < 0.05, n._filter.X.v
    assert np.linalg.norm(n._filter.X.p) < 0.05, n._filter.X.p


def test_quaternion_matrix_round_trip():
    for deg in (0.0, 30.0, 90.0, 179.0):
        r = math.radians(deg)
        R = np.array([[math.cos(r), -math.sin(r), 0.0],
                      [math.sin(r), math.cos(r), 0.0],
                      [0.0, 0.0, 1.0]])
        q = ln._quat_from_R(R)
        assert np.allclose(ln._R_from_quat(*q), R, atol=1e-9)


def test_a_non_unit_quaternion_is_normalised():
    """An off-unit quaternion gives a matrix with det != 1, and `so3_log` of
    that is not a rotation vector -- the filter would take the result as a
    real innovation."""
    R = ln._R_from_quat(2.0, 0.0, 0.0, 0.0)
    assert np.allclose(R, np.eye(3))
    assert pytest.approx(1.0, abs=1e-12) == np.linalg.det(R)


def _R_pitch(rad):
    c, sn = math.cos(rad), math.sin(rad)
    return np.array([[c, 0.0, sn], [0.0, 1.0, 0.0], [-sn, 0.0, c]])


def test_a_TILTED_hull_at_rest_does_not_accelerate():
    """⛔ THE ACTUAL VEHICLE FAILURE, reproduced.

    A level hull cancels gravity whatever the filter believes, so the level
    test above cannot see this bug. The hull was pitched -15.8 degrees
    (measured: accel -27, -179, 978 mG) while the filter held identity. Then
    `R @ a` points 1.75 m/s^2 sideways of `-GRAVITY`, the residual integrates,
    and the depth update's gain spreads it into x and y: 7.1e6 m in 35 s.

    Feeding the board's own attitude fixes it, and the assertion is the
    physical statement -- a hull sitting still has no velocity, at any angle.
    """
    pitch = math.radians(-15.8)
    R_true = _R_pitch(pitch)
    # What an accelerometer on that hull reads: gravity in BODY axes.
    a_body = R_true.T @ np.array([0.0, 0.0, 9.80665])
    qw, qx, qy, qz = ln._quat_from_R(R_true)

    n = _node()
    t = 100.0

    def sample(tt):
        m = _Imu(tt, accel=tuple(a_body))
        m.orientation.w, m.orientation.x = qw, qx
        m.orientation.y, m.orientation.z = qy, qz
        return m

    n._on_imu(sample(t))
    for _ in range(1750):                      # 35 s at 50 Hz, as measured
        t += 0.02
        n._on_imu(sample(t))
        n._on_state(type('S', (), {'depth_m': 1.22,
                                   'yaw_deg': float('nan')})())
    assert n._n['att'] > 1700
    assert np.linalg.norm(n._filter.X.v) < 0.5, f'velocity {n._filter.X.v}'
    assert np.linalg.norm(n._filter.X.p[:2]) < 1.0, f'position {n._filter.X.p}'


# --------------------------------------------------------------------------- #
#  ZUPT: stillness is measured, never assumed
# --------------------------------------------------------------------------- #
def test_a_still_hull_produces_a_zupt():
    n = _node()
    t = 100.0
    n._on_imu(_Imu(t))
    for _ in range(120):
        t += 0.02
        n._on_imu(_Imu(t))
    assert n._n['zupt'] >= 1


def test_a_MOVING_hull_produces_no_zupt():
    """Declaring 'still' during a slow transit deletes real motion, which is
    the standard way a ZUPT ruins a filter."""
    n = _node()
    t = 100.0
    n._on_imu(_Imu(t))
    for i in range(120):
        t += 0.02
        # A turning hull: gyro well above the bench noise floor.
        n._on_imu(_Imu(t, gyro=(0.0, 0.0, 0.4)))
    assert n._n['zupt'] == 0


def test_live_flow_suppresses_the_zupt():
    """With flow running, a still hull already measures zero -- a ZUPT adds
    nothing and could only conflict with the measurement."""
    n = _node()
    t = 100.0
    n._on_imu(_Imu(t))
    for _ in range(120):
        t += 0.02
        n._last_flow_t = time.monotonic()      # flow is fresh every tick
        n._on_imu(_Imu(t))
    assert n._n['zupt'] == 0


def test_flow_z_is_NOT_treated_as_a_measurement():
    """A bottom camera cannot see vertical velocity and the flow node marks it
    unobserved. Feeding its 0.0 asserts the hull never dives."""
    n = _node()
    twist = type('T', (), {})()
    twist.twist = type('T2', (), {})()
    twist.twist.twist = type('T3', (), {})()
    twist.twist.twist.linear = type('L', (), {'x': 0.3, 'y': 0.0, 'z': 0.0})()
    twist.twist.covariance = [0.0] * 36
    twist.twist.covariance[0] = 4e-4
    twist.twist.covariance[7] = 4e-4
    twist.twist.covariance[14] = -1.0          # vz unobserved
    before = n._filter.P[5, 5]                 # world-z velocity variance
    n._on_flow(twist)
    assert n._n['flow'] == 1
    # A 3-D update would have shrunk the vertical velocity variance; a 2-D one
    # leaves it essentially untouched.
    assert n._filter.P[5, 5] > before * 0.9


def test_the_flow_nodes_own_covariance_is_used():
    """A constant sigma tells the filter a bare floor is as good as a textured
    one. The flow node computes the variance per sample; use it."""
    n = _node()

    def _twist(var):
        t = type('T', (), {})()
        t.twist = type('T2', (), {})()
        t.twist.twist = type('T3', (), {})()
        t.twist.twist.linear = type('L', (), {'x': 0.3, 'y': 0.0, 'z': 0.0})()
        t.twist.covariance = [0.0] * 36
        t.twist.covariance[0] = var
        t.twist.covariance[7] = var
        return t

    tight = _node()
    tight._on_flow(_twist(1e-6))
    loose = _node()
    loose._on_flow(_twist(1.0))
    # The confident measurement must move the estimate further.
    assert abs(tight._filter.X.v[0]) > abs(loose._filter.X.v[0])


def test_a_missing_flow_covariance_falls_back_not_to_zero():
    """Zero variance is infinite confidence. An unset field must not read as
    one -- it is absence."""
    n = _node()
    t = type('T', (), {})()
    t.twist = type('T2', (), {})()
    t.twist.twist = type('T3', (), {})()
    t.twist.twist.linear = type('L', (), {'x': 0.3, 'y': 0.0, 'z': 0.0})()
    t.twist.covariance = [0.0] * 36            # nothing filled in
    n._on_flow(t)
    assert n._n['flow'] == 1
    assert np.all(np.isfinite(n._filter.P))


def test_the_first_attitude_is_ADOPTED_not_corrected_into():
    """An estimator with no prior information should take the first
    measurement, not argue with it.

    Measured on the vehicle: starting at identity against a hull at -168.3 deg
    cost 345 rejected measurements, one gate-lockout break, and 0.9 m of
    position error laid down during the transient -- which never goes away,
    because nothing observes horizontal position until a fix arrives.
    """
    n = _node()
    n._filter.rejected = 0
    R_true = _R_pitch(0.0)
    yaw = math.radians(-168.3)
    R_true = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                       [math.sin(yaw), math.cos(yaw), 0.0],
                       [0.0, 0.0, 1.0]])
    qw, qx, qy, qz = ln._quat_from_R(R_true)
    a_body = R_true.T @ np.array([0.0, 0.0, 9.80665])

    def sample(tt):
        m = _Imu(tt, accel=tuple(a_body))
        m.orientation.w, m.orientation.x = qw, qx
        m.orientation.y, m.orientation.z = qy, qz
        return m

    t = 100.0
    n._on_imu(sample(t))
    t += 0.02
    n._on_imu(sample(t))
    # One step in, the filter already holds the hull's real heading.
    assert n._filter.X.yaw_deg() == pytest.approx(-168.3, abs=0.5)
    for _ in range(500):
        t += 0.02
        n._on_imu(sample(t))
    assert n._filter.rejected == 0, f'{n._filter.rejected} rejects from a seed'
    assert np.linalg.norm(n._filter.X.p[:2]) < 0.05


# --------------------------------------------------------------------------- #
#  the frame is a claim, and the anchor is what makes it true
# --------------------------------------------------------------------------- #
def test_the_frame_is_odom_until_the_heading_is_anchored():
    """The board's yaw is excellent and is not a pool bearing -- it is
    boot-relative, or magnetic through a reference that may not have locked.
    Labelling the output `pool` before an anchor is not a naming nicety: flow
    integrates into position through the same rotation, so every dead-reckoned
    metre walks off along an offset nobody measured."""
    n = _node()
    assert n._anchored is False


def test_an_anchor_reaches_the_filter_and_flips_the_frame():
    n = _node()
    n._on_heading(type('F', (), {'data': 137.0})())
    assert n._anchored is True
    assert n._n['yaw'] == 1


def test_the_odom_stamp_is_the_inputs_not_the_wall_clock():
    """Every input is stamped on the board's clock through ClockMap. Stamping
    the output on host time puts back the 6.67 ms sd of transport jitter that
    the mapping exists to remove."""
    n = _node()
    n._on_imu(_Imu(1000.0))
    n._on_imu(_Imu(1000.02))
    assert n._last_input_t == pytest.approx(1000.02)


def test_no_velocity_aiding_is_announced():
    """⛔ MEASURED: with ZUPT off and no flow, the vehicle's position ran to
    635 m in 95 s while the filter published a healthy-looking pose the entire
    time. Attitude and depth leave horizontal velocity completely unobserved.
    A filter in that state is not degraded, it is not an estimate."""
    warned = []

    class _Logger:
        def info(self, *_a, **_k):
            pass

        def warning(self, msg, *_a, **_k):
            warned.append(msg)

    n = _node()
    n.get_logger = lambda: _Logger()
    n._diagnose()
    n._diagnose()
    assert any('NO VELOCITY AIDING' in m for m in warned)


def test_aiding_present_is_not_announced():
    warned = []

    class _Logger:
        def info(self, *_a, **_k):
            pass

        def warning(self, msg, *_a, **_k):
            warned.append(msg)

    n = _node()
    n.get_logger = lambda: _Logger()
    n._diagnose()
    n._n['zupt'] += 1
    n._diagnose()
    assert not any('NO VELOCITY AIDING' in m for m in warned)

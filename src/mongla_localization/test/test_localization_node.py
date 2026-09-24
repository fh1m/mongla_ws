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

from mongla_localization import localization_node as ln
from mongla_localization.inekf import RIEKF, State


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
    def __init__(self, t, gyro=(0.0, 0.0, 0.0), accel=(0.0, 0.0, -9.80665),
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
              'zupt': 0, 'grid': 0, 'grid_refused': 0,
              'lane': 0, 'lane_refused': 0, 'model': 0, 'gap': 0, 'gated': 0}
    obj._attitude_sigma_deg = 0.5
    obj._zupt_sigma = 0.01
    obj._zupt_enabled = True
    obj._still = deque(maxlen=ln.STILL_WINDOW)
    obj._last_flow_t = 0.0
    obj._attitude_seeded = False
    obj._anchored = False
    obj._yaw_offset_deg = 0.0
    obj._board_R = None
    obj._grid_sigma_deg = 1.0
    obj._grid_max_corr_deg = 20.0
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
    obj._model = ln.CommandVelocityModel()
    obj._model_aid = True
    obj._last_demand_t = None
    obj._motion = ln.MotionCheck()
    obj._motion_state = None
    obj._aiding_state = None
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
    n._on_imu(_Imu(102.0, accel=(2.0, 0.0, -9.80665)))
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
    a_body = R_true.T @ np.array([0.0, 0.0, -9.80665])
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
    a_body = R_true.T @ np.array([0.0, 0.0, -9.80665])

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
    n._on_imu(_Imu(100.0))                     # clock only
    n._on_imu(_Imu(100.02))                    # the board attitude to pair with
    n._on_heading(type('F', (), {'data': 137.0})())
    assert n._anchored is True
    assert n._n['yaw'] == 1
    assert n._filter.X.yaw_deg() == pytest.approx(137.0, abs=0.5)


def test_an_anchor_with_no_board_attitude_is_REFUSED_not_claimed():
    """An absolute heading is only an offset once it is paired with the board
    yaw it was taken against. A latched anchor reaching a node that has not
    heard the board may be minutes old; claiming `pool` on it is a wrong
    heading zero that nothing downstream can detect."""
    n = _node()
    n._on_heading(type('F', (), {'data': 137.0})())
    assert n._anchored is False
    assert n._yaw_offset_deg == 0.0


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


# --------------------------------------------------------------------------- #
#  the anchor must HOLD against the board, not just arrive (truth tests)
# --------------------------------------------------------------------------- #
G = 9.80665


def _board_at(t, yaw_deg, accel_x=0.0):
    """The board's IMU sample for a level hull at BOARD yaw `yaw_deg`."""
    m = _Imu(t, accel=(accel_x, 0.0, -G))
    a = math.radians(yaw_deg)
    Rz = np.array([[math.cos(a), -math.sin(a), 0.0],
                   [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])
    qw, qx, qy, qz = ln._quat_from_R(Rz)
    m.orientation.w, m.orientation.x = qw, qx
    m.orientation.y, m.orientation.z = qy, qz
    return m


def _fly_board(n, t, seconds, yaw_deg, speed=None, accel_x=0.0):
    """`seconds` of 50 Hz board samples at BOARD yaw `yaw_deg`.

    `speed(t)`, when given, is the TRUE forward speed and is reported as
    body-frame flow; `accel_x` is the matching forward specific force, so the
    accelerometer and the camera tell the same story."""
    for _ in range(int(round(seconds * 50))):
        t += 0.02
        n._on_imu(_board_at(t, yaw_deg, accel_x))
        if speed is not None:
            f = type('T', (), {})()
            f.header = _Header(t)
            f.twist = type('T2', (), {})()
            f.twist.twist = type('T3', (), {})()
            f.twist.twist.linear = type('L', (), {'x': speed(t), 'y': 0.0,
                                                  'z': 0.0})()
            f.twist.covariance = [0.0] * 36
            f.twist.covariance[0] = f.twist.covariance[7] = 1e-4
            n._on_flow(f)
    return t


def _published_frame(n):
    out = []
    n._pub = type('P', (), {'publish': lambda _s, m: out.append(m)})()
    n._publish()
    return out[-1].header.frame_id


def test_the_anchor_HOLDS_against_50hz_board_attitude():
    """⛔ TRUTH: the board boots with the hull at +30 deg (its own frame); the
    hull's true pool heading is -60. After the anchor and 5 s of the board
    pinning attitude at 50 Hz, the published yaw must be the POOL's.

    The defect this guards: the anchor went in as one `update_yaw` against a
    filter the board pins at 0.5 deg. A 90 deg offset is chi-square rejected
    outright (and an accepted one is dragged back within a second) -- yet
    `_anchored` flipped anyway, so the output said `pool` at +30."""
    n = _node()
    t = _fly_board(n, 100.0, 1.0, 30.0)
    assert n._filter.X.yaw_deg() == pytest.approx(30.0, abs=0.5)
    assert _published_frame(n) == 'odom'
    n._on_heading(type('F', (), {'data': -60.0})())
    t = _fly_board(n, t, 5.0, 30.0)
    assert n._filter.X.yaw_deg() == pytest.approx(-60.0, abs=1.0)
    assert _published_frame(n) == 'pool'
    # And it is an OFFSET, not a sticky number: the hull turns 45 deg right
    # (board +30 -> +75), so its true pool heading is -15.
    _fly_board(n, t, 2.0, 75.0)
    assert n._filter.X.yaw_deg() == pytest.approx(-15.0, abs=1.0)


def test_a_later_anchor_replaces_the_offset_rather_than_stacking():
    n = _node()
    t = _fly_board(n, 100.0, 1.0, 30.0)
    n._on_heading(type('F', (), {'data': -60.0})())
    t = _fly_board(n, t, 1.0, 30.0)
    n._on_heading(type('F', (), {'data': -55.0})())     # a better look
    _fly_board(n, t, 2.0, 30.0)
    assert n._filter.X.yaw_deg() == pytest.approx(-55.0, abs=1.0)


def test_flow_dead_reckons_along_the_POOL_heading_after_the_anchor():
    """⛔ TRUTH: hull faces pool -60 (board +30), accelerates straight ahead at
    0.5 m/s^2 for 1 s and cruises at 0.5 m/s for 3 s: 1.75 m along its nose.
    Flow and the accelerometer both report it in the BODY frame, and the
    filter turns them into the world through its R -- so the track must run
    along -60 in the pool frame, toward (+0.50, -0.87) NED. On the boot-frame
    R it runs along +30, labelled `pool`."""
    n = _node()
    t = _fly_board(n, 100.0, 1.0, 30.0)
    n._on_heading(type('F', (), {'data': -60.0})())
    t0 = _fly_board(n, t, 1.0, 30.0)
    p0 = n._filter.X.p[:2].copy()
    t = _fly_board(n, t0, 1.0, 30.0, speed=lambda tt: 0.5 * (tt - t0),
                   accel_x=0.5)
    _fly_board(n, t, 3.0, 30.0, speed=lambda tt: 0.5)
    d = n._filter.X.p[:2] - p0
    bearing = math.degrees(math.atan2(d[1], d[0]))
    assert bearing == pytest.approx(-60.0, abs=2.0)
    assert np.linalg.norm(d) == pytest.approx(1.75, rel=0.10)


def test_the_anchor_survives_a_retrodicted_replay_across_it():
    """The rotation is an EVENT, like the seed: a late measurement stamped
    before the anchor replays the tail through it, and the board samples
    after it were already rotated. Were the rotation applied outside the
    event log, the replay would restore a pre-anchor snapshot and lose it."""
    n = _node()
    n._retro = ln.Retrodictor(n._filter, horizon_s=2.0)
    t = _fly_board(n, 100.0, 1.0, 30.0)
    n._on_heading(type('F', (), {'data': -60.0})())
    t = _fly_board(n, t, 0.5, 30.0)
    late = type('T', (), {})()
    late.header = _Header(t - 0.8)                  # before the anchor
    late.twist = type('T2', (), {})()
    late.twist.twist = type('T3', (), {})()
    late.twist.twist.linear = type('L', (), {'x': 0.0, 'y': 0.0, 'z': 0.0})()
    late.twist.covariance = [0.0] * 36
    n._on_flow(late)
    assert n._retro.late == 1
    _fly_board(n, t, 0.5, 30.0)
    assert n._filter.X.yaw_deg() == pytest.approx(-60.0, abs=1.0)


def test_a_floor_correction_is_KEPT_not_undone_by_the_board():
    """⛔ TRUTH: anchored at pool -60 (board +30). The board then drifts +2 deg
    (reads +32 while the hull has not turned), and the grid keeps reporting
    the true -60. Without folding the correction into the offset, the next
    board sample pulls yaw straight back and the drift bound bounds nothing."""
    n = _node()
    t = _fly_board(n, 100.0, 1.0, 30.0)
    n._on_heading(type('F', (), {'data': -60.0})())
    t = _fly_board(n, t, 1.0, 32.0)                     # drifted board
    assert n._filter.X.yaw_deg() == pytest.approx(-58.0, abs=0.5)
    for _ in range(100):                                # 10 s of 10 Hz grid
        t = _fly_board(n, t, 0.1, 32.0)
        n._on_floor_grid(type('F', (), {'data': -60.0})())
    t = _fly_board(n, t, 1.0, 32.0)                     # board has the last word
    assert n._n['grid'] > 0
    assert n._filter.X.yaw_deg() == pytest.approx(-60.0, abs=0.5)


# --------------------------------------------------------------------------- #
#  the floor grid as a DRIFT BOUND on yaw
# --------------------------------------------------------------------------- #
def test_the_grid_is_ignored_until_the_heading_is_anchored():
    """Before the anchor our yaw is in the board's boot frame. Pulling a
    boot-frame heading onto a pool-frame grid combines two unrelated angles
    into a confident wrong one."""
    n = _node()
    n._anchored = False
    # ⛔ 0.0 DELIBERATELY, and the choice is the test. The filter starts at yaw
    # 0, so a grid at 0 needs NO correction and sails through the aliasing
    # guard -- leaving the anchored check as the only thing that can stop it.
    # An earlier version of this test used 30.0, which the aliasing guard
    # refused on its own, so the test passed with the anchored gate deleted.
    n._on_floor_grid(type('F', (), {'data': 0.0})())
    assert n._n['grid'] == 0, 'the grid was applied in the boot frame'
    assert n._n['grid_refused'] == 0, 'it should not even be considered'


def test_a_small_drift_is_corrected_by_the_floor():
    """⛔ NO MAGNETOMETER, NO PROP, NO DETECTION. Our hull deliberately never
    fuses a magnetometer -- the thrusters sit beside it -- so a free-running
    BNO drifts without bound. The floor's grid bounds it."""
    n = _node()
    n._anchored = True
    n._on_floor_grid(type('F', (), {'data': 0.0})())
    assert n._n['grid'] == 1
    assert n._n['grid_refused'] == 0


def test_a_large_disagreement_is_REFUSED_and_COUNTED():
    """A correction beyond the bound means the estimate and the floor disagree
    about which grid line is which. Applying it snaps the hull 90 degrees onto
    the wrong branch -- worse than the drift it was fixing."""
    n = _node()
    n._anchored = True
    n._filter.X.R = np.array([[0.0, -1.0, 0.0],      # yaw = +90 deg
                              [1.0, 0.0, 0.0],
                              [0.0, 0.0, 1.0]])
    n._grid_max_corr_deg = 5.0
    n._on_floor_grid(type('F', (), {'data': 60.0})())
    assert n._n['grid'] == 0
    assert n._n['grid_refused'] == 1



def test_the_lane_line_uses_its_OWN_symmetry_not_the_grids():
    """⛔ THE DISCRIMINATING CASE. Hull at yaw 90, feature at 0. For a square
    grid that is zero residual (90 is a grid direction) and applies. For a
    lane line, 90 degrees is the WRONG BRANCH -- the line runs across the hull
    -- and must be refused. If the node passed the grid's period to the lane,
    this would be applied and the test fails."""
    n = _node()
    n._anchored = True
    n._filter.X.R = np.array([[0.0, -1.0, 0.0],      # yaw = +90 deg
                              [1.0, 0.0, 0.0],
                              [0.0, 0.0, 1.0]])
    n._on_floor_grid(type('F', (), {'data': 0.0})())
    assert n._n['grid'] == 1
    n._on_lane_line(type('F', (), {'data': 0.0})())
    assert n._n['lane'] == 0
    assert n._n['lane_refused'] == 1


def test_a_lane_line_along_the_hull_is_applied_and_counted():
    n = _node()
    n._anchored = True
    n._on_lane_line(type('F', (), {'data': 0.0})())
    assert n._n['lane'] == 1


def test_the_lane_line_is_ignored_until_anchored():
    n = _node()
    n._anchored = False
    n._on_lane_line(type('F', (), {'data': 0.0})())
    assert n._n['lane'] == 0 and n._n['lane_refused'] == 0


def test_the_refusal_is_visible_in_the_diagnostic():
    """A channel that silently refuses half its measurements looks exactly like
    one that is not running."""
    import inspect
    src = inspect.getsource(ln.LocalizationNode._diagnose)
    assert 'grid' in src


# --------------------------------------------------------------------------- #
#  velocity from commanded demand, when flow is dead
# --------------------------------------------------------------------------- #
def _taught(n):
    """Teach the node's model on a simulated hull, the way flow would."""
    import sys, pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    from test_command_velocity import _fly
    _fly(n._model, 120.0)
    n._last_demand_t = time.monotonic()


def test_the_model_aids_ONLY_when_flow_is_stale():
    """With flow live the model would be handed back the numbers it was
    fitted to: agreement by construction, measuring nothing."""
    n = _node()
    _taught(n)
    n._last_flow_t = time.monotonic()          # flow live
    n._maybe_model_aid()
    assert n._n['model'] == 0
    n._last_flow_t = time.monotonic() - 2.0 * ln.FLOW_FRESH_S
    n._maybe_model_aid()
    assert n._n['model'] == 1


def test_an_UNTAUGHT_model_aids_with_nothing():
    """ON by default is safe only because an unready model is silent."""
    n = _node()
    n._last_demand_t = time.monotonic()
    n._model.step(0.5, 0.0, 5.0)
    n._maybe_model_aid()
    assert n._n['model'] == 0


def test_an_UNKNOWN_demand_on_the_wire_silences_the_aid():
    n = _node()
    _taught(n)
    n._last_flow_t = 0.0
    nan = type('V', (), {'x': float('nan'), 'y': float('nan')})()
    n._on_demand(type('M', (), {'vector': nan})())
    n._maybe_model_aid()
    assert n._n['model'] == 0


def test_a_STALE_demand_stream_silences_the_aid():
    n = _node()
    _taught(n)
    n._last_flow_t = 0.0
    n._last_demand_t = time.monotonic() - 5.0
    n._maybe_model_aid()
    assert n._n['model'] == 0


def test_the_filter_drifts_LESS_through_a_flow_outage_with_the_aid():
    """Closed loop, scored against truth: 30 s with no flow, a 0.02 m/s^2
    accelerometer bias, the hull cruising at 0.3 m/s. The aid is the model's
    own prediction error level (~0.02 m/s), not the truth."""
    rng = np.random.default_rng(3)

    def run(aid):
        f = RIEKF()
        f.X = State(v=np.array([0.3, 0.0, 0.0]))
        dt = 0.02
        for k in range(int(30.0 / dt)):
            f.predict((0.0, 0.0, 0.0), (0.02, 0.0, -9.80665), dt)
            if aid and k % 5 == 0:
                f.update_body_velocity_xy(0.3 + rng.normal(0, 0.02), rng.normal(0, 0.02),
                                          0.05 ** 2, 0.05 ** 2)
        return abs(f.X.p[0] - 0.3 * 30.0)

    free, aided = run(False), run(True)
    assert free > 5.0 * aided, (free, aided)


def test_a_hull_pinned_on_a_prop_is_REPORTED_and_does_not_poison_the_model():
    """Through `_on_flow`, the path the vehicle runs: demand held, flow at 0."""
    n = _node()
    _taught(n)
    g0 = n._model.x.theta[0]
    published = []
    n._pub_motion = type('P', (), {'publish': lambda self, m: published.append(m.data)})()
    # 15 s: LONGER than RELEARN_AFTER rejections. The model's own innovation
    # gate holds a short block; past it the model would reset and relearn the
    # wall, and only the node refusing to feed a blocked hull prevents that.
    for _ in range(300):                      # 15 s at 20 Hz of demand 0.6, no motion
        n._model.step(0.6, 0.0, 0.05)
        n._last_flow_t = time.monotonic() - 0.05
        twist = type('T', (), {})()
        twist.twist = type('T2', (), {'covariance': [0.0004] * 36})()
        twist.twist.twist = type('T3', (), {})()
        twist.twist.twist.linear = type('L', (), {'x': 0.0, 'y': 0.0, 'z': 0.0})()
        n._on_flow(twist)
    assert published and published[-1] == 'blocked', published
    assert n._model.x.theta[0] == pytest.approx(g0, rel=0.05)


def test_twist_covariance_is_in_the_body_frame_like_the_twist():
    """World-frame velocity variance (x tight, y loose) on a 90 deg heading
    is body x LOOSE, body y TIGHT. An unrotated copy reports the opposite."""
    import numpy as np
    from nav_msgs.msg import Odometry
    from mongla_localization.inekf import RIEKF
    from mongla_localization.localization_node import _fill_covariance
    f = RIEKF()
    c, s = 0.0, 1.0
    f.X.R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    f.P[3:6, 3:6] = np.diag([1e-4, 4e-2, 1e-3])
    m = Odometry()
    _fill_covariance(m, f)
    assert abs(m.twist.covariance[0] - 4e-2) < 1e-9
    assert abs(m.twist.covariance[7] - 1e-4) < 1e-9


# --------------------------------------------------------------------------- #
#  retrodict: measurements applied at the instant they describe                #
# --------------------------------------------------------------------------- #

def _flow_at(t, vx=0.4, vy=-0.1, var=1e-3):
    m = type('T', (), {})()
    m.header = _Header(t)
    m.twist = type('T2', (), {})()
    m.twist.twist = type('T3', (), {})()
    m.twist.twist.linear = type('L', (), {'x': vx, 'y': vy, 'z': 0.0})()
    m.twist.covariance = [0.0] * 36
    m.twist.covariance[0] = var
    m.twist.covariance[7] = var
    return m


def _run(retro, flow_arrives_after, n=80, flow_index=50):
    """IMU at 50 Hz with a forward push; one flow sample measured at IMU
    `flow_index` and delivered after IMU `flow_index + flow_arrives_after`."""
    from mongla_localization.retro import Retrodictor
    node = _node()
    node._zupt_enabled = False
    node._model_aid = False
    node._retro = Retrodictor(node._filter, horizon_s=1.0) if retro else None
    t0 = 1000.0
    for k in range(n):
        t = t0 + k * 0.02
        node._on_imu(_Imu(t, accel=(0.3, 0.0, -9.80665)))
        if k == flow_index + flow_arrives_after:
            node._on_flow(_flow_at(t0 + flow_index * 0.02 + 1e-4))
    return node


def test_retrodict_makes_a_late_flow_sample_land_where_an_on_time_one_would():
    on_time = _run(retro=True, flow_arrives_after=0)
    late = _run(retro=True, flow_arrives_after=3)          # 60 ms late
    arrival = _run(retro=False, flow_arrives_after=3)      # today's default
    np.testing.assert_allclose(late._filter.X.v, on_time._filter.X.v, atol=1e-9)
    np.testing.assert_allclose(late._filter.X.p, on_time._filter.X.p, atol=1e-9)
    assert late._retro.late == 1 and late._retro.replayed > 0
    assert np.abs(arrival._filter.X.v - on_time._filter.X.v).max() > 1e-4


def test_retrodict_off_is_exactly_the_old_on_arrival_path():
    a = _run(retro=False, flow_arrives_after=3)
    b = _node()
    b._zupt_enabled = False
    b._model_aid = False
    t0 = 1000.0
    for k in range(80):
        t = t0 + k * 0.02
        b._on_imu(_Imu(t, accel=(0.3, 0.0, -9.80665)))
        if k == 53:
            b._on_flow(_flow_at(t))
    np.testing.assert_allclose(a._filter.X.v, b._filter.X.v, atol=1e-12)


# ═══════════════════════════════════════════════════════════════════════════ #
#  The aiding signal -- B-56's observability gate, made visible
# ═══════════════════════════════════════════════════════════════════════════ #

class _Recorder:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg.data)


def test_aiding_is_reported_as_aided_on_a_healthy_filter():
    n = _node()
    n._pub_aiding = _Recorder()
    n._report_aiding()
    assert n._pub_aiding.sent == ['aided']


def test_aiding_flips_to_unaided_when_velocity_stops_being_observed():
    """⛔ THE SIGNAL A MISSION NEEDS. The pose keeps publishing and keeps
    looking healthy the whole time velocity is unobserved -- that is exactly
    what made B-56 hard to see. This is the machine-readable version of the
    warning the node already logs."""
    n = _node()
    n._pub_aiding = _Recorder()
    n._report_aiding()
    for _ in range(200):
        n._filter.predict([0, 0, 0], -ln.np.array([0.0, 0.0, 9.80665]), 0.02)
    n._report_aiding()
    assert n._pub_aiding.sent == ['aided', 'unaided']


def test_aiding_is_published_only_on_CHANGE():
    """Latched and edge-triggered, like `/mongla/localization/motion`. A 10 Hz
    restatement of an unchanged fact is noise an operator learns to ignore."""
    n = _node()
    n._pub_aiding = _Recorder()
    for _ in range(5):
        n._report_aiding()
    assert n._pub_aiding.sent == ['aided']


def test_aiding_recovers_when_velocity_is_measured_again():
    n = _node()
    n._pub_aiding = _Recorder()
    n._report_aiding()
    for _ in range(200):
        n._filter.predict([0, 0, 0], -ln.np.array([0.0, 0.0, 9.80665]), 0.02)
    n._report_aiding()
    for _ in range(50):
        n._filter.update_body_velocity([0.0, 0.0, 0.0], sigma=0.01)
    n._report_aiding()
    assert n._pub_aiding.sent == ['aided', 'unaided', 'aided']


def test_a_filter_without_the_gate_reports_aided_rather_than_crashing():
    """⚠ The node must survive a filter that predates the gate -- a replayed
    bag, or a swapped estimator. An unreadable gate is reported as aided, which
    is the pre-B-56 behaviour, rather than stopping the node."""
    n = _node()
    n._pub_aiding = _Recorder()

    class _Old:
        pass

    n._filter = _Old()
    n._report_aiding()
    assert n._pub_aiding.sent == ['aided']


def test_the_aiding_topic_is_not_the_motion_topic():
    """⚠ TWO DIFFERENT QUESTIONS, and conflating them would be easy.
    `motion` answers "is the hull moving as commanded" -- a prop, a snag, a
    dead thruster. `aiding` answers "can the filter see where it is". A blocked
    hull is perfectly well localised; an unaided one is not."""
    n = _node()
    n._pub_aiding, n._pub_motion = _Recorder(), _Recorder()
    for _ in range(200):
        n._filter.predict([0, 0, 0], -ln.np.array([0.0, 0.0, 9.80665]), 0.02)
    n._report_aiding()
    assert n._pub_aiding.sent == ['unaided']
    assert n._pub_motion.sent == [], 'the motion topic must be untouched'

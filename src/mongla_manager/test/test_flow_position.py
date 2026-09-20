"""FlowPositionSource -- the bottom camera standing in for the DVL, read off
the RIEKF's `/mongla/odom`.

Driven with a fake node and a fake yaw source, so these exercise the real
class rather than a description of it.
"""
import math
import time

import pytest

pytest.importorskip('geometry_msgs')
pytest.importorskip('nav_msgs')
pytest.importorskip('rclpy')

from geometry_msgs.msg import TwistWithCovarianceStamped   # noqa: E402
from nav_msgs.msg import Odometry                          # noqa: E402
from std_msgs.msg import UInt8                             # noqa: E402

from mongla_manager.flow_position import FlowPositionSource  # noqa: E402

ODOM = '/mongla/odom'
VEL = '/mongla/vision/downward/velocity'
QUAL = '/mongla/vision/downward/flow_quality'


class _Log:
    def info(self, *a, **k):
        pass

    warning = warn = error = debug = info


class _Node:
    """Just enough node for the source to construct: it only subscribes."""

    def __init__(self):
        self.subs = {}

    def create_subscription(self, msg_type, topic, cb, qos):
        self.subs[topic] = cb
        return object()

    def destroy_subscription(self, sub):
        pass

    def get_logger(self):
        return _Log()


class _Yaw:
    """A yaw source with NO position -- which is the whole premise."""

    name = 'bno085'

    def __init__(self, deg=0.0):
        self.deg = deg
        self.calibrated = 'sentinel'

    def read_yaw(self):
        return self.deg


def _src(**kw):
    node = _Node()
    inner = _Yaw()
    s = FlowPositionSource(node, inner, camera='downward', **kw)
    return s, node, inner


def _stamp(msg, t):
    msg.header.stamp.sec = int(t)
    msg.header.stamp.nanosec = int((t - int(t)) * 1e9)


def _flow(node, quality=200, stamp_wall=None):
    q = UInt8()
    q.data = quality
    node.subs[QUAL](q)
    m = TwistWithCovarianceStamped()
    _stamp(m, time.time() if stamp_wall is None else stamp_wall)
    node.subs[VEL](m)


def _odom(node, vx, vy=0.0, yaw_deg=0.0, sigma=0.03, stamp_wall=None,
          px=0.0, py=0.0):
    """One RIEKF output: BODY velocity, attitude, velocity covariance."""
    m = Odometry()
    _stamp(m, time.time() if stamp_wall is None else stamp_wall)
    h = math.radians(yaw_deg) / 2.0
    m.pose.pose.orientation.w = math.cos(h)
    m.pose.pose.orientation.z = math.sin(h)
    m.pose.pose.position.x = float(px)
    m.pose.pose.position.y = float(py)
    m.twist.twist.linear.x = float(vx)
    m.twist.twist.linear.y = float(vy)
    m.twist.covariance[0] = sigma ** 2
    m.twist.covariance[7] = sigma ** 2
    node.subs[ODOM](m)


def _healthy(node, n=3, **kw):
    for _ in range(n):
        _flow(node)
        _odom(node, 0.0, **kw)


# --------------------------------------------------------------------------- #
#  the duck-typed contract the motion layer checks
# --------------------------------------------------------------------------- #
def test_it_satisfies_the_contract_drive_forward_dist_looks_for():
    s, _, _ = _src()
    assert hasattr(s, 'get_position') and hasattr(s, 'reset_position')


def test_it_delegates_everything_else_to_the_real_yaw_source():
    s, _, inner = _src()
    assert s.read_yaw() == inner.deg
    assert s.calibrated == 'sentinel'


def test_it_reads_the_one_estimator():
    """The point of the merge: the RIEKF is the only velocity."""
    _s, node, _ = _src()
    assert ODOM in node.subs


# --------------------------------------------------------------------------- #
#  refusing is the feature
# --------------------------------------------------------------------------- #
class TestRefusal:
    def test_it_refuses_to_arm_with_no_fix_ever(self):
        from mongla_control.errors import MovementError
        s, _, _ = _src()
        ok, why = s.position_ready()
        assert not ok and 'no velocity fix' in why, why
        with pytest.raises(MovementError):
            s.reset_position()

    def test_it_refuses_when_the_last_fix_is_stale(self):
        from mongla_control.errors import MovementError
        s, node, _ = _src(fix_stale_s=0.05)
        _healthy(node)
        assert s.position_ready()[0]
        time.sleep(0.08)
        ok, why = s.position_ready()
        assert not ok and 'old' in why
        with pytest.raises(MovementError):
            s.reset_position()

    def test_it_refuses_without_the_localization_node(self):
        from mongla_control.errors import MovementError
        s, node, _ = _src()
        _flow(node)
        ok, why = s.position_ready()
        assert not ok and '/mongla/odom' in why, why
        with pytest.raises(MovementError):
            s.reset_position()

    def test_it_refuses_when_odometry_is_stale_but_flow_is_live(self):
        s, node, _ = _src(fix_stale_s=0.3)
        _odom(node, 0.0, stamp_wall=time.time() - 0.6)
        _flow(node)
        ok, why = s.position_ready()
        assert not ok and 'odom' in why and 'old' in why, why

    def test_it_refuses_an_unaided_velocity(self):
        """The RIEKF publishes whether or not anything observes velocity; its
        own sigma is the only thing that says so."""
        from mongla_control.errors import MovementError
        s, node, _ = _src()
        _healthy(node, sigma=0.5)
        ok, why = s.position_ready()
        assert not ok and 'sigma' in why, why
        with pytest.raises(MovementError):
            s.reset_position()

    def test_an_unfilled_covariance_is_absence_not_confidence(self):
        s, node, _ = _src()
        _healthy(node, sigma=0.0)
        assert not s.position_ready()[0]

    def test_the_refusal_names_what_to_check(self):
        from mongla_control.errors import MovementError
        s, _, _ = _src()
        with pytest.raises(MovementError) as e:
            s.reset_position()
        msg = str(e.value)
        assert 'flow_quality' in msg and 'move_forward' in msg

    def test_arming_succeeds_once_both_inputs_are_healthy(self):
        s, node, _ = _src()
        _healthy(node)
        ok, why = s.position_ready()
        assert ok, why
        s.reset_position()
        x, y = s.get_position()
        assert math.hypot(x, y) < 1e-9, (x, y)


# --------------------------------------------------------------------------- #
#  the frame and the integral
# --------------------------------------------------------------------------- #
class TestLatchedFrame:
    """Body velocity rotated by the RIEKF's own attitude into world, then back
    into the frame latched at the reset. Drop either rotation and forward
    travel leaks into the lateral channel -- a plausible number, no error."""

    def _travel(self, yaw_deg, seconds=1.0, vx=0.5, hz=20):
        # the whole leg is replayed from the past, so the origin is older
        # than the default freshness bound
        s, node, _ = _src(fix_stale_s=4.0)
        t0 = time.time() - seconds - 0.05
        _flow(node)
        _odom(node, vx, yaw_deg=yaw_deg, stamp_wall=t0)
        s.reset_position()
        for k in range(1, int(seconds * hz) + 1):
            _odom(node, vx, yaw_deg=yaw_deg, stamp_wall=t0 + k / hz)
        return s.get_position()

    @pytest.mark.parametrize('yaw', [0.0, 90.0, 225.0, -135.0])
    def test_forward_travel_is_the_x_channel_on_any_heading(self, yaw):
        x, y = self._travel(yaw)
        assert 0.45 < x < 0.60, (yaw, x, y)
        assert abs(y) < 0.01, (yaw, x, y)

    def test_a_yaw_mid_leg_puts_travel_where_it_happened(self):
        """Hull drives 1 m, turns 90 deg right, drives 1 m. In the frame
        latched at the start that is (+1, +1) -- right is +y."""
        s, node, _ = _src(fix_stale_s=4.0)
        t0 = time.time() - 2.2
        _flow(node)
        _odom(node, 1.0, yaw_deg=0.0, stamp_wall=t0)
        s.reset_position()
        for k in range(1, 21):
            _odom(node, 1.0, yaw_deg=0.0, stamp_wall=t0 + k * 0.05)
        for k in range(21, 41):
            _odom(node, 1.0, yaw_deg=90.0, stamp_wall=t0 + k * 0.05)
        x, y = s.get_position()
        # one trapezoid step straddles the turn
        assert abs(x - 1.0) < 0.06 and abs(y - 1.0) < 0.06, (x, y)

    def test_a_position_fix_mid_leg_does_not_move_the_leg(self):
        """Why velocity and not p_now - p_reset: a fix corrects error laid down
        before the reset, and that correction is not this leg's travel."""
        s, node, _ = _src()
        t0 = time.time() - 0.6
        _flow(node)
        _odom(node, 0.0, stamp_wall=t0, px=0.0)
        s.reset_position()
        for k in range(1, 11):
            _odom(node, 0.0, stamp_wall=t0 + k * 0.05, px=3.0 if k > 5 else 0.0)
        x, y = s.get_position()
        assert abs(x) < 1e-9 and abs(y) < 1e-9, (x, y)

    def test_a_gap_is_skipped_and_counted_not_integrated(self):
        s, node, _ = _src()
        t0 = time.time() - 0.9
        _flow(node)
        _odom(node, 1.0, stamp_wall=t0)
        s.reset_position()
        _odom(node, 1.0, stamp_wall=t0 + 0.8)
        assert s.status()['n_gap'] == 1
        x, _ = s.get_position()
        assert x < 0.3, x      # at most the capped extrapolation, never 0.8 m


# --------------------------------------------------------------------------- #
#  diagnostics and clocks
# --------------------------------------------------------------------------- #
def test_status_reports_absence_as_absence():
    s, _, _ = _src()
    st = s.status()
    assert st['fix_age_s'] == math.inf and st['odom_age_s'] == math.inf
    assert st['n_fixes'] == 0 and st['quality'] == 0
    assert st['vel_sigma_ms'] == math.inf


def test_status_counts_zero_quality_intervals():
    s, node, _ = _src()
    _flow(node, quality=0)
    _flow(node, quality=180)
    assert s.status()['n_zero_quality'] == 1


def test_dt_comes_from_the_odom_stamp_not_arrival_time():
    """Two outputs whose stamps are 0.2 s apart, delivered back to back.
    Arrival time gives microseconds of dt and no travel."""
    s, node, _ = _src()
    t0 = time.time() - 0.3
    _flow(node)
    _odom(node, 0.5, stamp_wall=t0)
    s.reset_position()
    _odom(node, 0.5, stamp_wall=t0 + 0.2)
    x, _ = s.get_position()
    assert x > 0.09, x
    assert s.status()['n_stamp_fallback'] == 0


def test_an_unstamped_publisher_falls_back_loudly_and_is_counted():
    s, node, _ = _src()
    _odom(node, 0.2, stamp_wall=0.0)
    _odom(node, 0.2, stamp_wall=0.0)
    assert s.status()['n_stamp_fallback'] == 2

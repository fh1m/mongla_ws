"""FlowPositionSource -- the bottom camera standing in for the DVL.

Driven with a fake node and a fake yaw source, so these exercise the real
class rather than a description of it.
"""
import math
import time

import pytest

pytest.importorskip('geometry_msgs')
pytest.importorskip('rclpy')

from geometry_msgs.msg import TwistWithCovarianceStamped   # noqa: E402
from std_msgs.msg import UInt8                             # noqa: E402

from duburi_manager.flow_position import FlowPositionSource  # noqa: E402


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


def _feed(node, vx, vy, sigma=0.01, quality=200, stamp_wall=None):
    """Deliver one velocity fix.

    `stamp_wall` is the WALL-clock capture instant `flow_node` puts in the
    header (it stamps the capture-interval MIDPOINT). Defaults to "now", which
    is what a healthy live pipeline looks like. Pass `stamp_wall=0.0` for an
    unstamped publisher.

    ⛔ This helper did NOT set a stamp at all until the timing fix, so every
    test here ran the fallback branch and none of them could see which clock
    the integrator used. A harness that cannot observe the property under test
    passes identically before and after the fix.
    """
    q = UInt8()
    q.data = quality
    node.subs['/duburi/vision/downward/flow_quality'](q)
    m = TwistWithCovarianceStamped()
    m.twist.twist.linear.x = float(vx)
    m.twist.twist.linear.y = float(vy)
    m.twist.covariance[0] = float(sigma) ** 2
    t = time.time() if stamp_wall is None else float(stamp_wall)
    m.header.stamp.sec = int(t)
    m.header.stamp.nanosec = int((t - int(t)) * 1e9)
    node.subs['/duburi/vision/downward/velocity'](m)


# --------------------------------------------------------------------------- #
#  the duck-typed contract the motion layer checks
# --------------------------------------------------------------------------- #
def test_it_satisfies_the_contract_drive_forward_dist_looks_for():
    """`drive_forward_dist` gates on exactly these two attributes. If this
    ever fails the verb refuses with 'no DVL position source' and the whole
    fold is inert -- while every other test here still passes."""
    s, _, _ = _src()
    assert hasattr(s, 'get_position') and hasattr(s, 'reset_position')


def test_it_delegates_everything_else_to_the_real_yaw_source():
    """It wraps, it does not replace. Heading, calibration state and any
    source-specific method must still reach the inner object."""
    s, _, inner = _src()
    assert s.read_yaw() == inner.deg
    assert s.calibrated == 'sentinel'


# --------------------------------------------------------------------------- #
#  refusing is the feature
# --------------------------------------------------------------------------- #
class TestRefusal:
    def test_it_refuses_to_arm_with_no_fix_ever(self):
        from duburi_control.errors import MovementError
        s, _, _ = _src()
        ok, why = s.position_ready()
        assert not ok and 'no velocity fix' in why, why
        with pytest.raises(MovementError):
            s.reset_position()

    def test_it_refuses_when_the_last_fix_is_stale(self):
        from duburi_control.errors import MovementError
        s, node, _ = _src(fix_stale_s=0.05)
        _feed(node, 0.2, 0.0)
        assert s.position_ready()[0]
        time.sleep(0.08)
        ok, why = s.position_ready()
        assert not ok and 'old' in why
        with pytest.raises(MovementError):
            s.reset_position()

    def test_the_refusal_names_what_to_check(self):
        """An operator reading this at the poolside needs the next action, not
        a category."""
        from duburi_control.errors import MovementError
        s, _, _ = _src()
        with pytest.raises(MovementError) as e:
            s.reset_position()
        msg = str(e.value)
        assert 'flow_quality' in msg and 'move_forward' in msg

    def test_arming_succeeds_once_flow_is_healthy(self):
        s, node, _ = _src()
        for _ in range(5):
            _feed(node, 0.2, 0.0)
        ok, why = s.position_ready()
        assert ok, why
        s.reset_position()          # must not raise
        # Near zero, not exactly: the filter holds a live 0.2 m/s and
        # `get_position` predicts to NOW, so a moving hull has genuinely
        # travelled between the two calls. Asserting an exact zero here would
        # be asserting that time does not pass.
        x, y = s.get_position()
        assert math.hypot(x, y) < 0.02, (x, y)


# --------------------------------------------------------------------------- #
#  the frame -- the part most likely to be silently wrong
# --------------------------------------------------------------------------- #
class TestLatchedFrame:
    """`NucleusDVLSource.get_position` is documented as 'body-frame position
    since last reset'. NavEstimator integrates in a LOCAL frame, so the two
    differ by the heading at the reset. Drop that rotation and forward travel
    leaks into the lateral channel -- a plausible number, no error, and
    `drive_forward_dist` reads the wrong axis."""

    def _travel(self, yaw_deg, seconds=0.4, vx=0.5):
        s, node, inner = _src()
        inner.deg = yaw_deg
        for _ in range(3):
            _feed(node, vx, 0.0)
        s.reset_position()
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            _feed(node, vx, 0.0)
            time.sleep(0.01)
        return s.get_position()

    def test_on_a_north_heading_forward_travel_is_the_x_channel(self):
        x, y = self._travel(0.0)
        assert x > 0.05
        assert abs(y) < 0.2 * abs(x)

    def test_on_an_EAST_heading_forward_travel_is_STILL_the_x_channel(self):
        """The one that catches a missing rotation. At yaw=90 the LOCAL
        displacement is almost entirely in py, so an unrotated read reports
        the travel as lateral and the along-track channel as ~0."""
        x, y = self._travel(90.0)
        assert x > 0.05, 'along-track travel vanished -- frame not rotated'
        assert abs(y) < 0.2 * abs(x)

    def test_and_on_a_south_west_heading_too(self):
        x, y = self._travel(225.0)
        assert x > 0.05
        assert abs(y) < 0.2 * abs(x)


# --------------------------------------------------------------------------- #
#  diagnostics
# --------------------------------------------------------------------------- #
def test_status_reports_absence_as_absence():
    """`fix_age_s` must be inf, never 0.0, before any fix -- 0.0 reads as
    'perfectly fresh', which is the absence-is-not-zero trap that has already
    caught the barometer and the ESC gate here."""
    s, _, _ = _src()
    st = s.status()
    assert st['fix_age_s'] == math.inf
    assert st['n_fixes'] == 0
    assert st['quality'] == 0


def test_status_counts_zero_quality_intervals():
    s, node, _ = _src()
    _feed(node, 0.2, 0.0, quality=0)
    _feed(node, 0.2, 0.0, quality=180)
    assert s.status()['n_zero_quality'] == 1


# --------------------------------------------------------------------------- #
#  WHICH CLOCK the position integrates on
# --------------------------------------------------------------------------- #
def test_dt_comes_from_the_capture_stamp_not_arrival_time():
    """The discriminating test for the timing fix.

    Two fixes whose CAPTURE stamps are 1.0 s apart, delivered back to back in
    real time. Integrating on arrival time gives a dt of microseconds and so
    essentially zero displacement; integrating on the stamp gives ~1 s of
    travel at the commanded speed.

    `flow_node` stamps the capture-interval MIDPOINT because flow measures an
    AVERAGE velocity over an interval, and with an adaptive baseline reaching
    0.75 s that correction is worth up to 375 ms. Reading a local clock here
    discarded it.
    """
    s, node, _ = _src()
    # Both stamps must be INSIDE fix_stale_s of now, or reset_position()
    # rightly refuses -- staleness is measured from the capture instant, which
    # is the same fix under test.
    t0 = time.time() - 0.5
    _feed(node, 0.5, 0.0, stamp_wall=t0)
    s.reset_position()
    _feed(node, 0.5, 0.0, stamp_wall=t0 + 0.5)
    x, _y = s.get_position()
    assert x > 0.15, (
        f'travelled {x:.3f} m over a 0.5 s stamped interval at 0.5 m/s -- '
        'the integrator is using arrival time, not the capture stamp')
    assert s.status()['n_stamp_fallback'] == 0


def test_an_unstamped_publisher_falls_back_loudly_and_is_counted():
    """Fail safe, not fail silent. A publisher that stamps nothing still has
    to work -- arrival time is the honest answer there -- but it must be
    visible in `status()` rather than silently degrading every distance."""
    s, node, _ = _src()
    _feed(node, 0.2, 0.0, stamp_wall=0.0)
    _feed(node, 0.2, 0.0, stamp_wall=0.0)
    assert s.status()['n_stamp_fallback'] == 2

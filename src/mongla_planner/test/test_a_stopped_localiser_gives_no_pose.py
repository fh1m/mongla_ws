"""A localiser that has stopped gives the mission no pose, and no motion verdict.

`pose()` cached the first odometry it heard and returned it for the rest of
the run: once a message existed it neither pumped for a newer one nor asked
how old it was. The localiser restarts three times and then stays down, so a
dead one left every later `pose()` answering with where the hull had been.
`motion()` read a latched topic published on change, so a dead localiser's
last `'ok'` read forever. Both now require `/mongla/odom` to be live, by the
rule the lock's loop closure already used: the STAMP moved within
`ODOM_MAX_AGE_S`, because the node's timer publishes whether or not its
inputs still arrive.
"""
import math
import time

import pytest
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from mongla_localization.localization_publish import ODOM_MAX_AGE_S
from mongla_planner.mongla_dsl import MonglaMission


@pytest.fixture
def node():
    started = not rclpy.ok()
    if started:
        rclpy.init()
    n = Node('stale_pose_probe')
    yield n
    n.destroy_node()
    if started and rclpy.ok():
        rclpy.shutdown()


class _M:
    """The fields `pose()` and `motion()` read, on a real node."""

    def __init__(self, node):
        self.client = type('C', (), {'node': node})()
        self._odom = None
        self._odom_sub = None
        self._pose_frame_warned = False
        self.log = type('L', (), {'warning': lambda *a, **k: None,
                                  'info': lambda *a, **k: None})()

    def _odom_fresh(self, timeout):
        return MonglaMission._odom_fresh(self, timeout)


def _odom(sec, frame='pool'):
    m = Odometry()
    m.header.frame_id = frame
    m.header.stamp.sec = sec
    m.pose.pose.position.x = 3.0
    m.pose.pose.orientation.w = 1.0
    return m


def _pump(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def test_a_live_estimate_is_returned(node):
    m = _M(node)
    pub = node.create_publisher(Odometry, '/mongla/odom', 10)
    MonglaMission.pose(m, timeout=0.0)            # subscribe
    for k in range(5):
        pub.publish(_odom(100 + k))
        _pump(node, 0.03)
    got = MonglaMission.pose(m, timeout=0.5)
    assert got is not None and got[0] == pytest.approx(3.0)


def test_a_frozen_stamp_gives_no_pose_after_the_bound(node):
    """On-time messages around a filter whose inputs stopped: same stamp."""
    m = _M(node)
    pub = node.create_publisher(Odometry, '/mongla/odom', 10)
    MonglaMission.pose(m, timeout=0.0)
    pub.publish(_odom(100))
    _pump(node, 0.1)
    assert MonglaMission.pose(m, timeout=0.2) is not None
    end = time.monotonic() + ODOM_MAX_AGE_S + 0.2
    while time.monotonic() < end:                 # the timer keeps publishing
        pub.publish(_odom(100))
        _pump(node, 0.05)
    assert MonglaMission.pose(m, timeout=0.0) is None


def test_silence_gives_no_pose_after_the_bound(node):
    m = _M(node)
    m._odom_sub = object()
    m._odom = _odom(100)
    m._odom_moved = time.monotonic() - 2 * ODOM_MAX_AGE_S
    assert MonglaMission.pose(m, timeout=0.0) is None
    m._odom_moved = time.monotonic()
    assert MonglaMission.pose(m, timeout=0.0) is not None


def test_a_dead_localisers_last_motion_verdict_is_not_read(node):
    m = _M(node)
    m._odom_sub = object()
    m._odom = _odom(100)
    m._motion_sub = object()
    m._motion = 'ok'
    m._odom_moved = time.monotonic()
    assert MonglaMission.motion(m, timeout=0.0) == 'ok'
    m._odom_moved = time.monotonic() - 2 * ODOM_MAX_AGE_S
    assert MonglaMission.motion(m, timeout=0.0) is None


def test_the_bound_is_five_publish_periods():
    from mongla_localization.localization_publish import PUBLISH_PERIOD_S
    assert ODOM_MAX_AGE_S == pytest.approx(5 * PUBLISH_PERIOD_S)
    assert math.isfinite(ODOM_MAX_AGE_S)

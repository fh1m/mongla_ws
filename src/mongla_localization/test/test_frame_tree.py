"""The frame tree, checked through tf2 itself -- not through our own algebra.

Issue #19. The stack is FRD/NED internally and stays so; REP-103 tools expect
FLU/ENU. These tests put `frames.static_edges()` into a real `tf2_ros.Buffer`
and ask tf2 to do the conversion, so a wrong quaternion fails here rather than
in a Foxglove panel where someone "fixes" a sign to make the picture match.

Premise, checked on the bench board 2026-09-30: yaw right (clockwise from
above) is +z in FRD -- heading +87.5 deg on a ~90 deg clockwise turn, gyro z
agreeing in sign in 33 of 35 moving samples.
"""
import math

import numpy as np
import pytest

tf2_ros = pytest.importorskip('tf2_ros')
tf2_geometry_msgs = pytest.importorskip('tf2_geometry_msgs')
from geometry_msgs.msg import PointStamped, Vector3Stamped   # noqa: E402
from rclpy.time import Time                                   # noqa: E402

from mongla_localization import frames as F                   # noqa: E402
from mongla_localization.frames import Offsets                # noqa: E402


def _buffer(offsets=Offsets(None, None, None), body_pose=None, anchored=False):
    b = tf2_ros.Buffer()
    stamp = Time().to_msg()
    for e in F.static_edges(offsets):
        b.set_transform_static(F.to_msg(e, stamp), 'test')
    if body_pose is not None:
        xyz, q = body_pose
        b.set_transform_static(
            F.to_msg(F.Edge(F.ODOM, F.BODY, xyz, q), stamp), 'test')
    if anchored:
        b.set_transform_static(F.to_msg(F.anchored_edge(), stamp), 'test')
    return b


def _vec(b, frame_from, frame_to, v):
    s = Vector3Stamped()
    s.header.frame_id = frame_from
    s.vector.x, s.vector.y, s.vector.z = (float(c) for c in v)
    out = b.transform(s, frame_to)
    return np.array([out.vector.x, out.vector.y, out.vector.z])


def _pt(b, frame_from, frame_to, p):
    s = PointStamped()
    s.header.frame_id = frame_from
    s.point.x, s.point.y, s.point.z = (float(c) for c in p)
    out = b.transform(s, frame_to)
    return np.array([out.point.x, out.point.y, out.point.z])


def test_issue_19_truth_case_forward_accel_and_a_right_yaw():
    """+1 m/s^2 forward and +0.1 rad/s yaw right, FRD, read in base_link (FLU):
    forward stays +x, and a RIGHT turn is a NEGATIVE z rate in FLU."""
    b = _buffer()
    assert np.allclose(_vec(b, F.BODY, F.BODY_FLU, (1.0, 0.0, 0.0)),
                       (1.0, 0.0, 0.0))
    assert np.allclose(_vec(b, F.BODY, F.BODY_FLU, (0.0, 0.0, 0.1)),
                       (0.0, 0.0, -0.1))


def test_starboard_in_frd_is_negative_y_in_flu():
    b = _buffer()
    assert np.allclose(_vec(b, F.BODY, F.BODY_FLU, (0.0, 1.0, 0.0)),
                       (0.0, -1.0, 0.0))


def test_ned_down_is_enu_minus_up_and_north_is_enu_y():
    """A hull 2 m NORTH and 0.5 m DOWN in odom_ned is at ENU (0, 2, -0.5)."""
    b = _buffer()
    assert np.allclose(_pt(b, F.ODOM, F.ODOM_ENU, (2.0, 0.0, 0.5)),
                       (0.0, 2.0, -0.5))
    assert np.allclose(_pt(b, F.ODOM, F.ODOM_ENU, (0.0, 3.0, 0.0)),
                       (3.0, 0.0, 0.0))                       # east is ENU x


def test_a_diving_hull_is_drawn_going_DOWN():
    """The failure #19 describes: `/mongla/odom` under a frame tools read as
    ENU drew the hull climbing as it dived. Through the tree it is below."""
    b = _buffer(body_pose=((0.0, 0.0, 1.5), F.Q_IDENTITY))
    assert _pt(b, F.BODY, F.ODOM_ENU, (0.0, 0.0, 0.0))[2] == pytest.approx(-1.5)


def test_base_link_resolves_from_odom_through_the_filter_edge():
    b = _buffer(body_pose=((1.0, 2.0, 0.5), F.Q_IDENTITY))
    assert b.can_transform(F.ODOM_ENU, F.BODY_FLU, Time())


def test_the_pool_is_UNREACHABLE_until_the_anchor_is_applied():
    """Before a pool fix there is no pool position, and a lookup must fail
    rather than return a boot-relative number under the pool's name."""
    b = _buffer(body_pose=((1.0, 2.0, 0.5), F.Q_IDENTITY))
    assert not b.can_transform(F.POOL, F.BODY, Time())
    b2 = _buffer(body_pose=((1.0, 2.0, 0.5), F.Q_IDENTITY), anchored=True)
    assert b2.can_transform(F.POOL, F.BODY, Time())
    # and once anchored, pool coordinates ARE the filter's coordinates
    assert np.allclose(_pt(b2, F.BODY, F.POOL, (0, 0, 0)), (1.0, 2.0, 0.5))


def test_an_unmeasured_camera_has_NO_edge_rather_than_a_zero_one():
    b = _buffer()
    assert not b.can_transform(F.BODY, F.cam_frame('downward'), Time())


def test_a_measured_camera_is_where_the_tape_says():
    r = (0.25, 0.0, 0.10)
    b = _buffer(Offsets(r, None, None))
    assert np.allclose(_pt(b, F.cam_frame('downward'), F.BODY, (0, 0, 0)), r)
    # body-aligned: a flow velocity along camera-frame x IS body forward
    assert np.allclose(_vec(b, F.cam_frame('downward'), F.BODY, (1, 0, 0)),
                       (1, 0, 0))


def test_every_parent_is_unique():
    """tf2's rule, checked on the definition so a new edge cannot break it."""
    edges = F.static_edges(Offsets((0.1, 0, 0), (0.2, 0, 0), (0, 0, 0))) + [
        F.Edge(F.ODOM, F.BODY, (0, 0, 0), F.Q_IDENTITY), F.anchored_edge()]
    children = [e.child for e in edges]
    assert len(children) == len(set(children)), children


def test_the_rotations_are_unit_quaternions():
    for q in (F.Q_FRD_TO_FLU, F.Q_ENU_TO_NED, F.Q_IDENTITY):
        assert math.isclose(sum(c * c for c in q), 1.0, abs_tol=1e-12)


def test_the_localization_node_SENDS_the_moving_edge_and_the_anchor_edge():
    """No capability ships unreachable: the node, not just this module, must
    put `odom_ned -> mongla` on the wire every publish, and `map -> odom` once,
    at the anchor. Captured from the real `_publish` / `_on_heading` paths."""
    import sys, pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    import test_localization_node as T

    sent, static = [], []
    n = T._node()
    n._tf = type('B', (), {'sendTransform': lambda _s, t: sent.append(t)})()
    n._tf_static = type('B', (), {'sendTransform': lambda _s, t: static.append(t)})()
    n.get_clock = lambda: type('C', (), {'now': lambda _s: Time()})()
    n.get_logger = lambda: type('L', (), {'info': lambda *a, **k: None,
                                          'warning': lambda *a, **k: None})()
    t = T._fly_board(n, 100.0, 1.0, 30.0)
    T._published_frame(n)
    assert sent and (sent[-1].header.frame_id, sent[-1].child_frame_id) == (
        F.ODOM, F.BODY)
    assert not static, 'map -> odom was sent before any anchor'
    n._on_heading(type('H', (), {'data': -60.0})())
    assert [(s.header.frame_id, s.child_frame_id) for s in static] == [
        (F.MAP_ENU, F.ODOM_ENU)]
    n._on_heading(type('H', (), {'data': -55.0})())
    assert len(static) == 1, 'the anchor edge must be sent once, not per anchor'

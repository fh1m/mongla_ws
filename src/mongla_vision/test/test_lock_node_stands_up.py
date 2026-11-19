"""Loop closure must START when it is switched on.

Every other closure test reads source text (`test_loop_closure_is_actually_wired`)
or drives the policy object (`test_loop_closure_refuses`). None stood the node
up with `loop_closure:=true`, so for its whole life the setup read
`self._camera`, an attribute nothing sets (the node keeps `self._cam`): the
switch raised AttributeError on every camera, and the capability was built,
tested and unreachable (second-pass audit 2026-11-19).
"""
import contextlib
import time
import types
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter as P            # noqa: E402


@contextlib.contextmanager
def _node(**params):
    from mongla_vision.lock_node import LockNode
    started = not rclpy.ok()
    if started:
        rclpy.init()
    node = LockNode(parameter_overrides=[P(k, value=v) for k, v in params.items()])
    try:
        yield node
    finally:
        node.destroy_node()
        if started:
            rclpy.shutdown()


def test_downward_starts_it_and_listens_for_odometry():
    with _node(camera='downward', loop_closure=True) as n:
        assert n._closer_on
        assert '/mongla/odom' in {s.topic_name for s in n.subscriptions}


def test_forward_refuses_it_and_keeps_running():
    with _node(camera='forward', loop_closure=True) as n:
        assert not n._closer_on


class _Anchor:
    """Stands in for the XFeat bank, which CI has no model for. The node's own
    path -- odometry, place bookkeeping, `consider`, the fix message -- is
    what runs; the matcher is told to recognise place 0 when the vehicle is
    back within 0.3 m of it."""

    class _Be:
        w, h = 320, 240

    def __init__(self):
        self._be, self.grid_centre, self.places = self._Be(), (160.0, 120.0), []
        self.xy = None

    def has_reference(self, *_a, **_k):        # the ladder thread asks; no target here
        return False

    @property
    def size(self):
        return len(self.places)

    def enrol(self, gray, *, roi, det_conf, label, position):
        self.places.append(tuple(position))
        return types.SimpleNamespace(accepted=True, keypoints=500)

    def locate(self, gray, *, label, near, radius_m):
        x, y = self.xy
        if self.places and (x - self.places[0][0]) ** 2 + (y - self.places[0][1]) ** 2 < 0.09:
            return types.SimpleNamespace(ok=True, label=label, ref_position=self.places[0],
                                         inliers=200, index=0,
                                         pose=types.SimpleNamespace(H=np.eye(3)))
        return None


def test_a_survey_that_comes_back_publishes_one_fix_at_the_first_place():
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo
    with _node(camera='downward', loop_closure=True) as n:
        a = n._anchor = _Anchor()
        info = CameraInfo(width=320, height=240)
        info.k = [300.0, 0.0, 160.0, 0.0, 300.0, 120.0, 0.0, 0.0, 1.0]
        n._on_info(info)
        fixes = []
        n._fix_pub.publish = fixes.append
        gray = np.zeros((240, 320), np.uint8)
        out = [(0.25 * i, 0.0) for i in range(13)]            # 0 to 3 m, a place every 5 s
        for i, xy in enumerate(out + out[::-1] + [(0.05, 0.0)] * 3):
            t0 = 1000.0 + 5.0 * i
            o = Odometry()
            o.header.stamp.sec = i + 1
            o.pose.pose.position.x, o.pose.pose.position.y = xy
            o.pose.covariance[0] = o.pose.covariance[7] = 0.01
            n._on_odom(o)
            n._odom_t = t0                      # the node's clock is the test's
            n._floor_h, n._floor_h_t = 1.0, time.monotonic()
            a.xy = xy
            n._place_tick(gray, o.header, t0)
        assert n._closures >= 1, dict(n._closure_refusals)
        assert (fixes[0].point.x, fixes[0].point.y) == a.places[0]
        assert fixes[0].point.z > 0.0           # its own sigma, never zero


def test_a_refused_parameter_call_changes_nothing():
    """All or nothing: a call whose later parameter is refused must not keep
    the earlier ones (authority applied, then the pair refused; a target
    switched, then publish_hz refused)."""
    with _node(camera='forward') as n:
        full, zero, cls = n._full, n._zero, n._cls
        r = n.set_parameters([P('full_authority_s', value=zero + 1.0)])[0]
        assert not r.successful and (n._full, n._zero) == (full, zero)
        r = n.set_parameters_atomically([P('zero_authority_s', value=zero + 5.0),
                                         P('full_authority_s', value=zero + 9.0)])
        assert not r.successful and (n._full, n._zero) == (full, zero)
        r = n.set_parameters_atomically([P('target_class', value='gate'),
                                         P('publish_hz', value=-1.0)])
        assert not r.successful and n._cls == cls

"""pose_fuse pairs a pose with the heading IN EFFECT when its frame was captured.

`/mongla/state` is published on change, so during a turn the latest heading to
arrive is from after the frame. The egomotion rule compares pose yaw against hull
yaw; pairing with a later heading feeds it a turn that had not happened yet.
"""
import time
from collections import deque
from types import SimpleNamespace

from mongla_localization.pose_fuse_node import PoseFuseNode


def _hdr(t):
    return SimpleNamespace(stamp=SimpleNamespace(sec=int(t), nanosec=int((t % 1) * 1e9)))


def _node():
    n = PoseFuseNode.__new__(PoseFuseNode)
    n._yaw_deg = None
    n._yaw_hist = deque(maxlen=256)
    return n


def test_the_heading_at_capture_is_used_not_the_latest():
    n = _node()
    now = time.time()
    for dt, yaw in ((-0.9, 10.0), (-0.5, 20.0), (-0.1, 30.0)):
        n._on_state(SimpleNamespace(yaw_deg=yaw, header=_hdr(now + dt)))
    from mongla_vision.stamps import capture_monotonic
    t_pose, _ = capture_monotonic(_hdr(now - 0.3))
    assert n.yaw_at(t_pose) == 20.0          # in effect at -0.3 s
    assert n._yaw_deg == 30.0                # what the old code would have used


def test_a_pose_before_any_heading_gets_none():
    n = _node()
    now = time.time()
    n._on_state(SimpleNamespace(yaw_deg=5.0, header=_hdr(now - 0.1)))
    from mongla_vision.stamps import capture_monotonic
    t_pose, _ = capture_monotonic(_hdr(now - 0.8))
    assert n.yaw_at(t_pose) is None

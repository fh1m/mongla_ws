"""The vertical speed flow checks its height against is this moment's or
none (audit vl_12 F12-9): when depth stops, the history keeps its last
samples, and a speed differenced from them is old."""
import time
from collections import deque
from types import SimpleNamespace

from mongla_vision.flow.flow_constants import HEIGHT_STALE_S
from mongla_vision.flow.flow_node import FlowVelocityNode


def _hist(newest):
    return deque([(newest - 0.3 * (3 - i), -1.0 - 0.1 * i) for i in range(4)], maxlen=16)


def test_a_fresh_descent_reads_positive_down():
    me = SimpleNamespace(_depth_hist=_hist(time.monotonic()))
    vz = FlowVelocityNode._vz_down(me)
    assert vz is not None and vz > 0


def test_an_old_history_gives_no_speed():
    me = SimpleNamespace(_depth_hist=_hist(time.monotonic() - HEIGHT_STALE_S - 1.0))
    assert FlowVelocityNode._vz_down(me) is None

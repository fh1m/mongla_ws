"""DistanceState liveness tests.

After the latched-topic refactor, start()/stop() are fire-and-forget publishes
with no ack. The one contract that must hold: stop() reports a FAILURE (not a
clean 0.0 m) when the estimator node never publishes distance_traveled -- else a
distance-gated move would trust a phantom 0 m. No ROS graph needed: the node is a
MagicMock and we drive _on_distance directly.
"""

from unittest.mock import MagicMock

import pytest
from std_msgs.msg import Float32

from mongla_manager.distance_state import DistanceState


def _state():
    return DistanceState(MagicMock(), camera='downward')


def test_stop_reports_failure_when_estimator_never_publishes():
    st = _state()
    st.start(lateral=False)
    ok, msg, dist = st.stop()          # no _on_distance ever arrives
    assert ok is False
    assert 'not publishing' in msg
    assert dist == 0.0


def test_stop_succeeds_after_a_real_sample():
    st = _state()
    st.start(lateral=False)
    st._on_distance(Float32(data=0.385))   # estimator streamed a metre reading
    ok, msg, dist = st.stop()
    assert ok is True
    assert msg == 'stop'
    assert dist == pytest.approx(0.385)


def test_start_resets_and_reports_axis():
    st = _state()
    st._on_distance(Float32(data=9.9))     # stale value from a prior bracket
    ok, cmd = st.start(lateral=True)
    assert ok is True
    assert cmd == 'start_lateral'
    assert st.distance_m == 0.0            # reset on start

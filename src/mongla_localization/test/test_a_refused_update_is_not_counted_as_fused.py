"""A measurement the filter refused is not a measurement the filter used.

The node counted what it HANDED the filter: `_apply` dropped the update's own
answer and returned True, so a flow sample the chi-square gate rejected, or a
depth the B-56 gate refused, was counted as fused. The diagnostic line then
over-reported every channel, and the operator's one safety warning, "NO
VELOCITY AIDING", reads those counters: a stream of rejected flow kept it
silent while nothing was aiding velocity at all.
"""
import numpy as np

import test_localization_node as tl


def _flow(t, vx, vy, var):
    m = type('Tw', (), {})()
    m.header = tl._Header(t)
    m.twist = type('T2', (), {})()
    m.twist.twist = type('T3', (), {})()
    m.twist.twist.linear = type('L', (), {'x': vx, 'y': vy, 'z': 0.0})()
    cov = [0.0] * 36
    cov[0] = cov[7] = var
    cov[14] = -1.0
    m.twist.covariance = cov
    return m


def _confidently_still(n):
    """A filter certain the hull is not moving: velocity sigma 1 mm/s."""
    n._filter.P[3:6, 3:6] = np.eye(3) * 1e-6


def test_a_flow_sample_the_filter_REJECTS_is_counted_refused():
    n = tl._node()
    _confidently_still(n)
    n._on_flow(_flow(10.0, 5.0, 0.0, 1e-4))         # 5 m/s against 1 mm/s
    assert n._filter.rejected == 1, 'the case is not a rejection; fix the test'
    assert n._n['flow'] == 0
    assert n._n['flow_refused'] == 1


def test_a_flow_sample_the_filter_ACCEPTS_is_counted_fused():
    n = tl._node()
    n._on_flow(_flow(10.0, 0.05, 0.0, 1e-3))
    assert n._filter.accepted == 1
    assert n._n['flow'] == 1 and n._n['flow_refused'] == 0


def test_rejected_flow_does_not_silence_the_no_aiding_warning():
    warned = []

    class _Logger:
        def info(self, *_a, **_k):
            pass

        def warning(self, msg, *_a, **_k):
            warned.append(msg)

    n = tl._node()
    n.get_logger = lambda: _Logger()
    _confidently_still(n)
    n._diagnose()
    for k in range(3):
        n._on_flow(_flow(10.0 + k, 5.0, 0.0, 1e-4))
    n._diagnose()
    assert any('NO VELOCITY AIDING' in m for m in warned)


def test_a_depth_the_observability_gate_refuses_is_counted_refused():
    n = tl._node()
    n._filter.P[3, 3] = n._filter.P[4, 4] = 4.0      # horizontal velocity unobserved
    st = type('S', (), {'depth_m': -1.0, 'yaw_deg': float('nan')})()
    n._on_state(st)
    assert n._filter.gated == 1, 'the gate did not refuse; fix the test'
    assert n._n['depth'] == 0
    assert n._n['depth_refused'] == 1


def test_the_same_holds_with_retrodiction_on():
    from mongla_localization.retro import Retrodictor
    n = tl._node()
    n._retro = Retrodictor(n._filter)
    _confidently_still(n)
    n._on_flow(_flow(10.0, 5.0, 0.0, 1e-4))
    assert n._n['flow'] == 0 and n._n['flow_refused'] == 1

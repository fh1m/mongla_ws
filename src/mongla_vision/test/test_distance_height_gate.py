"""B05 / B10 -- the height that scales optical flow into metres.

Two defects, one file, one class: a value that means "we could not measure
this" was turned into a number that means "we measured zero".

  B05  the caller passed `self._last_height or 0.0` into
       `DistanceAccumulator.add`, whose FIRST guard is `if height_m is None:
       return`. The coercion made that guard unreachable, so an unmeasurable
       frame was integrated as `+= proj * 0.0 / f_px` -- recorded as "the
       vehicle did not move" instead of skipped as "we could not tell".

  B10  `_last_height` had no clock. If depth stopped arriving, flow kept being
       scaled by the last known height for ever. Metres-per-pixel is directly
       proportional to height, so the distance kept integrating at a stale
       scale, with a plausible number and no warning.

These test the accumulator contract and the freshness rule directly; the node's
own wiring is covered by test_flow_node.py. The node these were written
against (distance_estimation_node) was retired for flow_node; the B10 tests
now guard flow_node, which had lost the fix.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_vision.flow.flow_math import DistanceAccumulator   # noqa: E402


def _active_acc():
    """An accumulator armed along the body-forward axis (yaw 0, not lateral).

    Flow is (dx, dy) and fore/aft rides IMAGE-Y on the downward camera, so the
    test vectors are y-only -- an x-only flow projects to exactly zero on this
    axis and would make every assertion below pass for the wrong reason.
    """
    acc = DistanceAccumulator()
    acc.start(0.0, False)
    return acc


# --------------------------------------------------------------------------- #
#  B05 -- None must SKIP the frame, 0.0 must not stand in for it
# --------------------------------------------------------------------------- #
def test_a_None_height_skips_the_frame_entirely():
    acc = _active_acc()
    acc.add((0.0, 40.0), None, 500.0)
    assert acc.distance_m == 0.0, 'a frame with no height must not be integrated'


def test_zero_height_is_NOT_the_same_as_no_height():
    """The heart of B05. Both leave distance at 0.0, and they are different
    facts: one is a skipped frame, the other is a recorded non-movement."""
    skipped = _active_acc()
    skipped.add((0.0, 40.0), None, 500.0)

    recorded = _active_acc()
    recorded.add((0.0, 40.0), 0.0, 500.0)

    # Both are 0.0 -- which is exactly why the bug was invisible.
    assert skipped.distance_m == recorded.distance_m == 0.0
    # The distinction is only observable on the NEXT good frame: a real height
    # must move the distance, proving the accumulator was not poisoned.
    skipped.add((0.0, 40.0), 0.8, 500.0)
    assert skipped.distance_m != 0.0, 'a good frame after a skip must integrate'


def test_a_real_height_integrates_proportionally():
    """Guards the scale itself: distance is linear in height."""
    a, b = _active_acc(), _active_acc()
    a.add((0.0, 40.0), 0.5, 500.0)
    b.add((0.0, 40.0), 1.0, 500.0)
    assert b.distance_m == pytest.approx(2.0 * a.distance_m, rel=1e-6)


# --------------------------------------------------------------------------- #
#  B10 -- the latch expires
# --------------------------------------------------------------------------- #
def _flow_stub(depth_m, rx):
    from mongla_vision.flow.flow_node import FlowVelocityNode

    class _Stub:
        _pool_depth = 1.6
        _lens_below_baro = 0.0
        _depth_m = depth_m
        _depth_t = rx
        _fresh_height = FlowVelocityNode._fresh_height
    return _Stub()


def test_the_height_latch_expires():
    """flow_node's `_fresh_height` gives the height while the depth is fresh
    and None once it is stale -- the B10 fix, carried over when the old
    distance node was retired."""
    from mongla_vision.flow.flow_constants import HEIGHT_STALE_S
    s = _flow_stub(-0.7, 100.0)
    assert s._fresh_height(100.0) == pytest.approx(0.9)          # same instant
    assert s._fresh_height(100.0 + HEIGHT_STALE_S - 0.01) == pytest.approx(0.9)
    assert s._fresh_height(100.0 + HEIGHT_STALE_S + 0.01) is None
    assert HEIGHT_STALE_S > 1.0, 'must not gate on ordinary depth jitter'


def test_a_height_that_never_arrived_is_None_not_zero():
    assert _flow_stub(None, None)._fresh_height(1.0) is None


def test_the_evaluation_asks_for_a_fresh_height_every_frame():
    """The guard is only a guard if the frame path uses it: the height must
    be recomputed from `_fresh_height`, never held from an earlier frame."""
    import inspect
    from mongla_vision.flow.flow_node import FlowVelocityNode
    src = inspect.getsource(FlowVelocityNode._evaluate)
    assert 'self._last_height = self._fresh_height(time.monotonic())' in src


def test_the_debug_topic_reports_an_absent_height_as_nan():
    """B05's rule on the output side: no height is NaN, not 0.0."""
    import inspect
    from mongla_vision.flow.flow_publish import FlowPublishMixin
    src = inspect.getsource(FlowPublishMixin._publish_distance)
    assert "float('nan') if self._last_height is None" in src
    assert 'self._last_height or 0.0' not in src

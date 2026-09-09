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
own wiring is covered by test_flow_node.py.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_vision.flow.flow_math import DistanceAccumulator   # noqa: E402


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
def test_the_height_latch_expires():
    """`_fresh_height` returns the height while fresh and None once stale."""
    from duburi_vision.flow import distance_estimation_node as dn

    class _Stub:
        _last_height = 0.9
        _last_height_t = 100.0
        _fresh_height = dn.DistanceEstimationNode._fresh_height

    s = _Stub()
    assert s._fresh_height(100.0) == 0.9                        # same instant
    assert s._fresh_height(100.0 + dn.HEIGHT_STALE_S - 0.01) == 0.9
    assert s._fresh_height(100.0 + dn.HEIGHT_STALE_S + 0.01) is None
    assert dn.HEIGHT_STALE_S > 1.0, 'must not gate on ordinary depth jitter'


def test_a_height_that_never_arrived_is_None_not_zero():
    from duburi_vision.flow import distance_estimation_node as dn

    class _Stub:
        _last_height = None
        _last_height_t = None
        _fresh_height = dn.DistanceEstimationNode._fresh_height

    assert _Stub()._fresh_height(1.0) is None

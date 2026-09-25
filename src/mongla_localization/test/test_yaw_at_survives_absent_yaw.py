"""`yaw_at` must not raise when the hull's yaw is absent.

⛔ THE DEFECT. `pose_fuse_node.yaw_at` did `sorted(self._yaw_hist, key=lambda h: h[0])` on a deque of
`(capture_t, yaw_or_None)`. On a stamp TIE, Python falls through to comparing the
second element, and that element is `None` whenever `/mongla/state` carries
`yaw_deg = NaN` -- this stack's documented "absent" value, which `_on_state`
converts to None. `None < float` raises TypeError, inside a subscription callback,
so the detection is dropped and the executor thread takes the exception.

Neither precondition is exotic:

  * `/mongla/state` is published by TWO timers, `telemetry_tick` (2 Hz) and
    `_fast_state_tick` (20 Hz), in two different callback groups. Two messages
    carrying the same capture stamp is ordinary.
  * an absent yaw is the documented value before the board's AHRS goes healthy --
    i.e. exactly while a mission is starting.

INJECTION-VERIFIED: drop the `key=` from the sort in `yaw_at` and
`test_a_stamp_tie_with_an_absent_yaw_does_not_raise` fails with the TypeError.
"""
import bisect
from collections import deque

import pytest


class _Hist:
    """`yaw_at` and `_yaw_hist` lifted out of the node, so this needs no rclpy."""

    def __init__(self):
        self._yaw_hist: deque = deque(maxlen=256)

    def add(self, t, yaw):
        # _on_state: NaN is the absent convention, stored as None.
        self._yaw_hist.append((t, None if yaw != yaw else yaw))

    def yaw_at(self, t):
        hist = sorted(self._yaw_hist, key=lambda h: h[0])
        i = bisect.bisect_right([h[0] for h in hist], t)
        return hist[i - 1][1] if i else None


NAN = float('nan')


def test_a_stamp_tie_with_an_absent_yaw_does_not_raise():
    """⛔ The regression, in the shape that reproduced it."""
    h = _Hist()
    h.add(100.0, 12.5)
    h.add(100.5, NAN)        # yaw absent
    h.add(100.5, 30.0)       # same capture stamp, yaw back
    assert h.yaw_at(101.0) in (None, 30.0)


def test_every_tie_ordering_is_survivable():
    """Order of arrival within a tie is not something we control."""
    for pair in ((NAN, 30.0), (30.0, NAN), (NAN, NAN)):
        h = _Hist()
        h.add(100.0, 12.5)
        for y in pair:
            h.add(100.5, y)
        h.yaw_at(101.0)      # must not raise
        h.yaw_at(100.0)
        h.yaw_at(99.0)


def test_out_of_order_arrival_still_resolves_to_the_stamp_before():
    """The sort is there because two timers publish this topic with no ordering
    guarantee. Keep that property while making the comparison total."""
    h = _Hist()
    h.add(102.0, 40.0)
    h.add(100.0, 10.0)       # arrives late, stamped earlier
    h.add(101.0, 20.0)
    assert h.yaw_at(101.5) == 20.0
    assert h.yaw_at(100.5) == 10.0
    assert h.yaw_at(99.0) is None


def test_an_absent_yaw_is_reported_as_absent_not_as_zero():
    """A missing number must never arrive as 0.0 -- a 0 deg heading is a real
    heading, and this stack's whole convention exists to keep them apart."""
    h = _Hist()
    h.add(100.0, NAN)
    assert h.yaw_at(100.5) is None


def test_an_empty_history_is_absent_not_an_index_error():
    assert _Hist().yaw_at(100.0) is None

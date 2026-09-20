"""The one clock conversion, and the rung that was reading the wrong one.

Everything here RUNS the shipping code. The recorded reason: round 26's guard
on the ported verbs grepped the source for `_srot_drive(`, and renaming the
DEFINITION left every call site containing the string -- so it stayed green
through exactly the change it existed to catch.
"""
import sys
import time
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_vision.stamps import (            # noqa: E402
    MAX_AGE_S, SKEW_TOL_S, capture_monotonic)


def _hdr(wall_s):
    return types.SimpleNamespace(stamp=types.SimpleNamespace(
        sec=int(wall_s), nanosec=int((wall_s - int(wall_s)) * 1e9)))


# --------------------------------------------------------------------------- #
#  The conversion
# --------------------------------------------------------------------------- #
def test_a_good_stamp_yields_the_CAPTURE_instant_not_now():
    """The whole point. A frame captured 120 ms ago must report 120 ms of age,
    not the ~0 that reading the local clock here would give."""
    t, why = capture_monotonic(_hdr(time.time() - 0.120))
    assert why == ''
    assert time.monotonic() - t == pytest.approx(0.120, abs=0.02)


def test_the_two_clock_domains_are_not_mixed():
    """`header.stamp` is WALL and the answer is MONOTONIC. On a box where the
    two are far apart -- they always are; monotonic counts from boot -- a
    conversion that forgot to subtract would return an epoch-scale number."""
    t, _ = capture_monotonic(_hdr(time.time()))
    assert abs(t - time.monotonic()) < 0.05
    assert t < time.time() / 2, 'returned a wall-clock value'


def test_a_missing_stamp_falls_back_and_SAYS_SO():
    """Both shapes: a header with no stamp, and NO HEADER AT ALL. The second
    is not hypothetical -- it is what a bare `SimpleNamespace(detections=[])`
    looks like, and moving this guard into a shared function briefly lost it,
    which turned a fallback into an exception inside a subscription callback."""
    for h in (types.SimpleNamespace(), None):
        t, why = capture_monotonic(h)
        assert why, f'accepted {h!r}'
        assert t == pytest.approx(time.monotonic(), abs=0.05)


def test_an_UNSTAMPED_message_is_not_treated_as_1970():
    """A zero stamp is what an unstamped message looks like. Trusting it would
    report 56 years of age -- and every gate would read that as LOST forever."""
    t, why = capture_monotonic(_hdr(0.0))
    assert 'zero' in why
    assert time.monotonic() - t < 0.05


def test_a_stamp_from_the_FUTURE_never_produces_a_negative_age():
    """The dangerous direction. Every freshness gate in the stack treats a
    smaller age as fresher, so a negative age reads as maximally fresh -- the
    one failure that makes the hull act with more confidence, not less."""
    t, why = capture_monotonic(_hdr(time.time() + 0.05))
    assert why == '', 'sub-skew clock lead should still be usable'
    assert time.monotonic() - t >= 0.0


def test_a_wildly_skewed_clock_falls_back_rather_than_lying():
    for wall in (time.time() + SKEW_TOL_S + 5.0,     # host clock ahead
                 time.time() - MAX_AGE_S - 5.0):     # stopped clock
        t, why = capture_monotonic(_hdr(wall))
        assert why, f'accepted a stamp {wall - time.time():+.1f}s out'
        assert time.monotonic() - t < 0.05


def test_there_is_exactly_ONE_copy_of_the_bounds():
    """Two copies of a bound is how the simulator's scorer came to grade a
    torpedo board that no longer existed."""
    root = Path(__file__).resolve().parents[2]
    hits = [p for p in root.rglob('*.py')
            if 'SKEW_TOL_S' in p.read_text() and p.name != 'test_stamps.py']
    assert [p.name for p in hits] == ['stamps.py'], hits


# --------------------------------------------------------------------------- #
#  The ladder: which frame each rung is actually speaking about
# --------------------------------------------------------------------------- #
from mongla_vision.lock_node import LockNode, header_for      # noqa: E402
from mongla_vision.tracking.lock_state import Rung            # noqa: E402


def test_the_ANCHOR_rung_is_stamped_with_the_frame_IT_ran_on():
    """THE DEFECT. The anchor runs at 3 Hz while the loop runs at frame rate,
    so between evaluations its pose is REUSED -- up to 333 ms old. Publishing
    it under the current frame's header reported that third of a second as
    ~18 ms, and it did so only on the lower rungs: correct-looking exactly
    while a detection is present, wrong exactly when the ladder is working."""
    assert header_for(Rung.ANCHOR, detection='d', frame='now',
                      anchor='old') == 'old'


def test_each_rung_reports_the_instant_IT_observed():
    assert header_for(Rung.DETECTION, detection='d', frame='now',
                      anchor='old') == 'd'
    assert header_for(Rung.FOLLOW, detection='d', frame='now',
                      anchor='old') == 'now'
    assert header_for(Rung.LOST, detection='d', frame='now',
                      anchor='old') == 'now'


def test_a_rung_with_no_header_yet_falls_back_to_the_CURRENT_frame():
    """Too-new makes the consumer act with LESS authority than it could, which
    is the safe direction; `None` would suppress the publish entirely."""
    assert header_for(Rung.ANCHOR, detection=None, frame='now',
                      anchor=None) == 'now'


class _FakeDet:
    """One detection message, shaped the way `_on_det` reads it."""

    class _Hyp:
        def __init__(self, name, score):
            self.hypothesis = types.SimpleNamespace(class_id=name, score=score)

    def __init__(self, name, score, wall):
        box = types.SimpleNamespace(
            center=types.SimpleNamespace(position=types.SimpleNamespace(
                x=100.0, y=80.0)), size_x=40.0, size_y=30.0)
        self.detections = [types.SimpleNamespace(
            results=[self._Hyp(name, score)], bbox=box)]
        self.header = _hdr(wall)


def _bare_node():
    """The real `LockNode` with `Node.__init__` skipped.

    Subclassing rather than reimplementing: `_on_det` under test is the
    SHIPPING method, not a copy of it that can drift from the one that runs.
    """
    import threading
    n = LockNode.__new__(LockNode)
    n._lock = threading.Lock()
    n._cls = ''
    n._det_box = n._det_header = None
    n._det_conf = n._det_t = 0.0
    n._stamp_warned = False
    n.get_logger = lambda: types.SimpleNamespace(
        warn=lambda *_a, **_k: None, info=lambda *_a, **_k: None)
    return n


def test_the_authority_DECAY_CLOCK_starts_at_capture_not_arrival():
    """`_det_t` is what `arbitrate` measures authority against. Reading the
    local clock on arrival under-reports every gap by the whole
    capture->inference->transport chain -- 32 ms median, 48 p95 -- always in
    the direction that keeps full authority past what the measurement
    justifies."""
    n = _bare_node()
    n._on_det(_FakeDet('gate', 0.8, time.time() - 0.150))
    assert n._det_box is not None
    age = time.monotonic() - n._det_t
    assert age == pytest.approx(0.150, abs=0.03), (
        f'decay clock reads {age * 1000:.0f} ms for a 150 ms-old frame')


def test_the_detections_own_header_is_kept_for_publication():
    n = _bare_node()
    msg = _FakeDet('gate', 0.8, time.time() - 0.05)
    n._on_det(msg)
    assert n._det_header is msg.header


def test_a_refused_detection_does_not_move_the_decay_clock():
    """A coasted track publishes score 0. Letting it reset the clock would
    make the ladder hold full authority off a box nothing observed -- the
    fabrication the arbiter's second rule exists to forbid."""
    n = _bare_node()
    n._on_det(_FakeDet('gate', 0.0, time.time()))
    assert n._det_box is None and n._det_t == 0.0

"""An expired Kalman filter must be REVIVED by a real detection.

THE BUG THIS PINS. `tracker_node._on_detections` had:

    if self._kalman.is_expired(td.track_id):
        continue

`is_expired()` reads `predict_streak`, and `predict_streak` resets ONLY inside
`smooth()` (`kalman.py:83`) -- which the `continue` skips. `prune()` only drops
filters whose id is ABSENT from `tracked`. So once a filter expired while the
backend still held the id, a returning target with that id was suppressed
**for ever**: it could never reach the one call that would clear the condition
suppressing it.

Reachable at the shipped config. The Kalman expires at 30 frames (1.5 s) and
the Roboflow backend holds an id for 100 (`int(20/30*150)`), so frames 30-100
are a dead zone where a reacquired target is silently missing from `/tracks`.
The control coast follows `locked_id`, so it sees nothing -- in the exact path
whose job is to survive a gap.

WHY THE EXISTING SUITE COULD NOT SEE IT. `test_roboflow_tracker.py:107`
composes the tracker and the smoother and then *reimplements* the node's
publish logic in a local helper. It therefore tests a copy of the code, and
the copy did not have the bug. These tests drive `TrackKalmanSmoother` through
the SHIPPING sequence instead.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

pytest.importorskip('filterpy', reason='TrackKalmanSmoother needs filterpy')

from duburi_vision.tracking.kalman import TrackKalmanSmoother   # noqa: E402


from duburi_vision.tracker_node import kalman_pass          # noqa: E402
from duburi_vision.tracking.tracker import TrackedDetection  # noqa: E402


def _TD(track_id, cx, cy, predicted, size=20.0):
    """A TrackedDetection centred at (cx, cy) -- the real type the node uses."""
    h = size * 0.5
    return TrackedDetection(
        class_id=0, class_name='t', score=0.0 if predicted else 0.9,
        xyxy=(cx - h, cy - h, cx + h, cy + h),
        track_id=track_id, predicted=predicted)


def _node_pass(kal, tracked, frame_t):
    """THE SHIPPING FUNCTION, imported -- not a copy of it. That distinction
    is the whole reason this bug survived: the existing suite reimplemented
    this loop, so it tested a copy that did not have the defect."""
    out = kalman_pass(kal, tracked, frame_t)
    return [(t.track_id, t.cx, t.cy) for t in out]


def _smoother(max_predict=5):
    return TrackKalmanSmoother(process_noise=0.05, measurement_noise=2.0,
                               max_predict_frames=max_predict)


def test_a_real_detection_revives_an_expired_track():
    """THE ASSERTION. Coast past the predict horizon with the id still alive,
    then deliver a REAL box with that id. It must reach the output."""
    kal = _smoother(max_predict=5)
    t = 0.0
    _node_pass(kal, [_TD(7, 100.0, 100.0, False)], t)      # a real sighting

    for _ in range(8):                                     # coast past expiry
        t += 0.05
        _node_pass(kal, [_TD(7, 100.0, 100.0, True)], t)
    assert kal.is_expired(7), 'fixture broken -- the filter did not expire'

    t += 0.05
    out = _node_pass(kal, [_TD(7, 120.0, 130.0, False)], t)
    assert out, ('a REAL detection was suppressed because the filter had '
                 'expired -- the track can never come back')
    assert out[0][0] == 7


def test_the_revived_track_follows_the_NEW_observation():
    """Reviving must not resurrect a cold extrapolation. The filter is rebuilt
    from the real box, so the first output is that box -- not wherever the
    dead track had drifted to."""
    kal = _smoother(max_predict=3)
    t = 0.0
    _node_pass(kal, [_TD(3, 50.0, 50.0, False)], t)
    for _ in range(6):
        t += 0.05
        _node_pass(kal, [_TD(3, 50.0, 50.0, True)], t)
    t += 0.05
    (_tid, cx, cy), = _node_pass(kal, [_TD(3, 400.0, 300.0, False)], t)
    assert cx == pytest.approx(400.0, abs=1.0)
    assert cy == pytest.approx(300.0, abs=1.0)


def test_a_still_coasting_track_stays_suppressed():
    """The other half. Expiry exists to stop us steering on a stale
    extrapolation -- a PREDICTED box past the horizon must remain suppressed,
    or the fix would have removed the guard instead of the bug."""
    kal = _smoother(max_predict=4)
    t = 0.0
    _node_pass(kal, [_TD(9, 10.0, 10.0, False)], t)
    for _ in range(6):
        t += 0.05
        out = _node_pass(kal, [_TD(9, 10.0, 10.0, True)], t)
    assert kal.is_expired(9)
    assert out == [], 'a coasted track past the horizon must not be published'


def test_the_dead_zone_is_gone_across_a_realistic_gap():
    """End to end at the shipped shape: 1.5 s predict horizon, a 2 s gap, then
    reacquisition with the same id -- which is what the backend does, since it
    holds an id for 5 s. Before the fix the target stayed invisible for the
    remaining 3 s of backend lifetime.

    Not hypothetical: the gate approach measured 53 gaps in 57 s, four of them
    past 1.5 s.
    """
    hz = 20.0
    kal = _smoother(max_predict=int(1.5 * hz))
    t = 0.0
    for _ in range(10):
        _node_pass(kal, [_TD(1, 200.0, 200.0, False)], t)
        t += 1 / hz
    for _ in range(int(2.0 * hz)):                    # 2 s coast
        _node_pass(kal, [_TD(1, 200.0, 200.0, True)], t)
        t += 1 / hz
    revived = []
    for _ in range(10):                               # target is back
        revived += _node_pass(kal, [_TD(1, 260.0, 210.0, False)], t)
        t += 1 / hz
    assert len(revived) == 10, (
        f'only {len(revived)}/10 frames survived after reacquisition -- the '
        f'track is still in the dead zone')


def test_prune_still_drops_a_track_the_backend_forgot():
    """The pre-existing behaviour the fix must not disturb."""
    kal = _smoother()
    _node_pass(kal, [_TD(5, 1.0, 1.0, False)], 0.0)
    _node_pass(kal, [_TD(6, 2.0, 2.0, False)], 0.05)   # 5 absent -> pruned
    assert kal.is_expired(5) is False                  # gone, not expired

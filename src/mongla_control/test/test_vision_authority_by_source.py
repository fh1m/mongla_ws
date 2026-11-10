"""Each box steers with the authority of the source that made it (#77, #78).

Three sources reach the loops: a live detection, the tracker's Kalman
prediction through a gap, and the lock ladder's follower / anchor box. They
used to share two curves: every non-live box decayed on the tracker's
`coast_s`, so `coast_s=0` disabled the ladder outright and `coast_s=0.8` cut
its 2.5 s horizon to 0.8 s; and yaw ignored age entirely, so a stale bearing
kept the hull turning. And a box with no authority left could still count
toward ALIGNED, or end a fill-stop move, because it sat in band.
"""
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from mongla_control.motion_vision import ALIGNED, VISION_FRESH_ZERO_MAX_S
from mongla_control.motion_vision.freshness import (
    _authority, _is_evidence, _ladder_authority)

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_motion_vision import _FakeBoard, _FakeVision, _align, _move  # noqa: E402

AGES = (0.0, 0.05, 0.2, 0.5, 0.8, 1.0)


def _box(source='detector', ex=0.0, age_s=0.0, score=0.9, fill=0.0):
    return SimpleNamespace(ex=ex, ey=0.0, w_frac=fill, h_frac=fill, age_s=age_s,
                           track_id=-1, score=score, source=source,
                           coasted=source != 'detector')


# ---------------------------------------------------------------- pure

@pytest.mark.parametrize('age', AGES)
def test_the_ladder_does_not_read_the_trackers_knob(age):
    box = _box('ladder', age_s=age, score=0.8)
    assert _authority(box, coast_s=0.0) == _authority(box, coast_s=0.8)


def test_a_healthy_ladder_box_steers_with_coasting_disabled():
    """The acceptance case: lock_s on, coast_s off, a fresh follower box."""
    assert _authority(_box('ladder', age_s=0.02, score=1.0), coast_s=0.0) == 1.0
    assert _authority(_box('ladder', age_s=0.02, score=0.6), coast_s=0.0) \
        == pytest.approx(0.6)


def test_the_ladders_authority_is_applied_once_and_bounded():
    """No second time decay of the lock, no boost past full, NaN refused."""
    assert _ladder_authority(1.7, 0.0) == 1.0
    assert _ladder_authority(float('nan'), 0.0) == 0.0
    assert _ladder_authority(0.0, 0.0) == 0.0


@pytest.mark.parametrize('source', ['detector', 'ladder'])
def test_a_stalled_box_stops_steering(source):
    """An expired box -- a dead lock node's last message, a frozen detector --
    always stops influencing control, whatever the producer last claimed."""
    box = _box(source, age_s=VISION_FRESH_ZERO_MAX_S + 0.01, score=1.0)
    assert _authority(box, coast_s=0.8) == 0.0


def test_a_tracker_prediction_still_expires_on_its_own_horizon():
    assert _authority(_box('tracker', age_s=0.9), coast_s=0.8) == 0.0
    assert _authority(_box('tracker', age_s=0.1), coast_s=0.0) == 0.0


@pytest.mark.parametrize('source', ['detector', 'ladder', 'tracker'])
def test_authority_never_rises_with_age(source):
    a = [_authority(_box(source, age_s=t, score=1.0), coast_s=0.8) for t in AGES]
    assert all(x >= y for x, y in zip(a, a[1:])), a


def test_evidence_is_a_measurement_that_still_has_authority():
    assert _is_evidence(_box('detector'), 1.0)
    assert _is_evidence(_box('ladder'), 0.4)
    assert not _is_evidence(_box('ladder'), 0.0)
    assert not _is_evidence(_box('detector'), 0.0)
    assert not _is_evidence(_box('tracker'), 1.0), 'a Kalman guess is not a sighting'


# ---------------------------------------------------------------- the loop

def _peak(board, axis):
    return max(abs(c.get(axis, 0.0)) for c in board.rc)


@pytest.mark.parametrize('coast_s', [0.0, 0.8])
def test_the_ladder_moves_the_hull_whatever_coast_s_is(coast_s):
    board = _FakeBoard()
    _align(_FakeVision(_box('ladder', ex=0.5, age_s=0.02, score=1.0)), pix=board,
           axes={'lat', 'yaw'}, kp_lat=60.0, kp_yaw=60.0, gain=30.0,
           coast_s=coast_s, lock_s=1.0, duration=0.2)
    assert _peak(board, 'lateral') == 30.0 and _peak(board, 'yaw') == 30.0


@pytest.mark.parametrize('age,moves', [(0.02, True),
                                       (VISION_FRESH_ZERO_MAX_S + 0.05, False)])
def test_every_axis_takes_the_same_authority(age, moves):
    """Lateral and yaw fade together: vision never asks one axis to act on a
    bearing it has told the other to stop trusting."""
    board = _FakeBoard()
    _align(_FakeVision(_box('detector', ex=0.5, age_s=age)), pix=board,
           axes={'lat', 'yaw'}, kp_lat=60.0, kp_yaw=60.0, gain=30.0, duration=0.2)
    assert (_peak(board, 'lateral') > 0) is moves
    assert (_peak(board, 'yaw') > 0) is moves


@pytest.mark.parametrize('box', [_box('tracker', ex=0.0, age_s=0.05),
                                 _box('ladder', ex=0.0, age_s=0.05, score=0.0),
                                 _box('detector', ex=0.0,
                                      age_s=VISION_FRESH_ZERO_MAX_S + 0.05)])
def test_a_box_nothing_steers_on_cannot_declare_aligned(box):
    out, _, _ = _align(_FakeVision(box), axes={'lat'}, coast_s=0.8, lock_s=1.0,
                       duration=0.3, verify=False)
    assert out.code != ALIGNED, out.reason


def test_a_live_centred_box_still_aligns():
    """The control: the gate bites because of evidence, not because align broke."""
    out, _, _ = _align(_FakeVision(_box('detector', ex=0.0, age_s=0.02)),
                       axes={'lat'}, duration=0.5, verify=False)
    assert out.code == ALIGNED, out.reason


def test_a_predicted_box_at_the_fill_does_not_end_a_move():
    out, board, _ = _move(_FakeVision(_box('tracker', age_s=0.05, fill=0.9)),
                          coast_s=0.8, duration=0.3)
    assert out.code != ALIGNED, out.reason
    assert _peak(board, 'forward') == 0.0, 'it closed on a guess'
    out, _, _ = _move(_FakeVision(_box('detector', age_s=0.02, fill=0.9)),
                      duration=0.3)
    assert out.code == ALIGNED, out.reason

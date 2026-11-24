"""tracker_check passed on a tracker publishing empty arrays (audit 2026-11-24)."""
from collections import Counter

from mongla_vision.utils.check_tracker import failures_of


def _state(reused):
    return {'reused_ids': reused, 'track_frames': 0, 'msg_count': 100,
            'classes': Counter()}


def test_empty_arrays_at_full_rate_fail():
    assert any('no stable track' in f for f in failures_of(_state(0), 15.0, 5.0))


def test_a_track_seen_twice_passes():
    assert failures_of(_state(3), 15.0, 5.0) == []

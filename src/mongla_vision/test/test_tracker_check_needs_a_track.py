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


def test_nothing_to_count_prints_no_percentage():
    """With no tracks, 0.0 % read as a measured 0 % reuse (audit vl_16 VL16-04)."""
    from mongla_vision.utils.check_tracker import pct, ratios
    empty = {'reused_ids': 0, 'id_frames': 0, 'predicted_tracks': 0, 'total_tracks': 0}
    some = {'reused_ids': 3, 'id_frames': 4, 'predicted_tracks': 0, 'total_tracks': 5}
    assert [pct(x) for x in ratios(empty)] == ['--', '--']
    assert [pct(x) for x in ratios(some)] == ['75.0%', '0.0%']

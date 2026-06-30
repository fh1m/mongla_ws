"""Behaviour tests for the Roboflow `trackers` backend (RoboflowTracker).

Pins the contract the control/HUD path depends on: persistent IDs across a
detection gap, coasted boxes emitted while the track is buffered
(predicted=True, score=0.0) with the right id/class, and the track dropped
after the frame-rate-scaled buffer. Skips cleanly if `trackers` isn't
installed (the node falls back to legacy_bytetrack at runtime).
"""

import pytest

pytest.importorskip("trackers", reason="roboflow trackers not installed")
pytest.importorskip("supervision")

from duburi_vision.detection.detector import Detection
from duburi_vision.tracking.roboflow_tracker import RoboflowTracker


def _det(cx, cls='gate', cid=0, score=0.9, w=50, h=50):
    return Detection(class_id=cid, class_name=cls, score=score,
                     xyxy=(cx - w / 2, 100, cx + w / 2, 100 + h))


def _run(tracker, frames):
    """frames: list of List[Detection]; returns list of update() outputs."""
    return [tracker.update(dets, i / 20.0) for i, dets in enumerate(frames)]


# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("ttype", ["ocsort", "bytetrack"])
def test_assigns_stable_id_to_a_steady_track(ttype):
    t = RoboflowTracker(tracker_type=ttype, track_buffer=12, frame_rate=20.0,
                        min_hits=2)
    outs = _run(t, [[_det(100 + i * 5)] for i in range(6)])
    # after maturity the same id is held every frame
    ids = [td.track_id for out in outs[2:] for td in out]
    assert ids, "expected confirmed tracks after min_hits"
    assert len(set(ids)) == 1, f"id should be stable, got {set(ids)}"
    assert all(td.predicted is False for out in outs[2:] for td in out)


@pytest.mark.parametrize("ttype", ["ocsort", "bytetrack"])
def test_coasts_through_a_gap_then_drops(ttype):
    # 5 real frames, then a long gap that exceeds the (scaled) buffer.
    t = RoboflowTracker(tracker_type=ttype, track_buffer=12, frame_rate=20.0,
                        min_hits=2)
    frames = [[_det(100 + i * 10)] for i in range(5)] + [[] for _ in range(12)]
    outs = _run(t, frames)

    # During the gap, the first few empty-detection frames still yield a
    # coasted box (predicted=True, score=0.0) carrying the class + a real id.
    gap_first = outs[5]
    assert len(gap_first) == 1, "expected a coasted box right after the gap starts"
    td = gap_first[0]
    assert td.predicted is True and td.score == 0.0
    assert td.class_name == 'gate' and td.track_id >= 0

    # The coasted box advances (constant-velocity Kalman), not frozen.
    assert outs[6][0].cx > outs[5][0].cx

    # Eventually the track is dropped (buffer scaled by frame_rate: 20/30*12=8).
    assert len(outs[-1]) == 0, "track must be dropped after the buffer expires"


def test_ocsort_reassociates_same_id_after_gap():
    # The competition-critical property: a target that reappears after a brief
    # gap keeps its id (no phantom/duplicate), so a lock doesn't jump objects.
    t = RoboflowTracker(tracker_type='ocsort', track_buffer=20, frame_rate=20.0,
                        min_hits=2)
    frames = ([[_det(100 + i * 10)] for i in range(5)]
              + [[] for _ in range(3)]
              + [[_det(140 + i * 10)] for i in range(3)])
    outs = _run(t, frames)
    pre = [td.track_id for td in outs[4] if not td.predicted]
    post = [td.track_id for out in outs[-2:] for td in out if not td.predicted]
    assert pre and post and pre[0] == post[-1], \
        f"OC-SORT must keep the id across the gap (pre={pre} post={post})"


def test_reset_clears_state():
    t = RoboflowTracker(tracker_type='ocsort', frame_rate=20.0, min_hits=2)
    _run(t, [[_det(100 + i * 5)] for i in range(4)])
    t.reset()
    assert not t._meta
    # a fresh acquisition starts immature again
    out0 = t.update([_det(300)], 0.0)
    assert all(td.track_id == -1 for td in out0) or out0 == []


def test_two_objects_get_distinct_ids():
    t = RoboflowTracker(tracker_type='ocsort', track_buffer=12, frame_rate=20.0,
                        min_hits=2)
    frames = [[_det(100 + i * 5, cls='hole'), _det(400 + i * 5, cls='hole')]
              for i in range(5)]
    outs = _run(t, frames)
    last = [td.track_id for td in outs[-1]]
    assert len(set(last)) == 2, f"two objects must get two ids, got {last}"

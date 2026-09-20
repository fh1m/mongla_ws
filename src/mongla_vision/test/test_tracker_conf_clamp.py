"""The tracker's confidence gates must not exceed the detector's own floor.

THE DEFECT, and it is the largest one this codebase has had.

`high_conf_det_threshold` gates TRACK CREATION: below it the backend never
starts a track, so `/tracks` is empty, so there is nothing for the coast layer
to coast on. The shipped value was 0.6, inherited from pedestrian benchmarks
where detection scores run high. Underwater scores do not.

Measured on real RoboSub 2025 footage with the model trained on that footage
-- 319 detections across the gate approach:

    score p10 0.167  p50 0.258  p90 0.439  max 0.640
    fraction at or above 0.6:  0.6 %

    high_conf_det_threshold   tracker presence
            0.60                  0.0 %      <- SHIPPED
            0.40                 13.9 %
            0.25                 17.0 %
            0.15                 45.1 %

The detector's own presence on that footage is 15.5 %. So at the shipped
setting the ENTIRE tracking stack -- the Kalman smoother, `/tracks`,
`vision.coast_s`, the continuity lock -- produced nothing, and at 0.15 it
nearly TRIPLES presence and takes losses past `lost_grace_s` from SIX to ONE.

Nothing raised anything, ever, because a tracker with no tracks still
publishes an empty array and every node looks healthy. Same family as the
truncated-coast bug in `test_tracker_rate.py`.

THE FIX IS STRUCTURAL, NOT A TUNED NUMBER: a detection the DETECTOR chose to
publish must be allowed to start a track. Anything else silently discards
work the chip already did. The gates are clamped to the detector's live
`conf`, so the two move together and cannot drift apart again.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

pytest.importorskip('trackers', reason='needs the roboflow trackers lib')

from mongla_vision.detection.detector import Detection          # noqa: E402
from mongla_vision.tracking.roboflow_tracker import RoboflowTracker  # noqa: E402


def _box(score, x=100.0):
    return Detection(class_id=0, class_name='t', score=score,
                     xyxy=(x, 100.0, x + 60.0, 160.0))


def _emitted(detector_conf, score, ttype='ocsort', frames=30, hc=0.6):
    tr = RoboflowTracker(tracker_type=ttype, track_buffer=150, frame_rate=32.0,
                         min_hits=3, iou_threshold=0.2,
                         track_activation_threshold=0.40,
                         high_conf_det_threshold=hc,
                         detector_conf=detector_conf)
    return sum(1 for i in range(frames) if tr.update([_box(score)], i / 32.0))


def test_the_shipped_gates_emit_NOTHING_at_real_underwater_scores():
    """The bug, pinned. 0.258 is the measured p50 on real competition water."""
    assert _emitted(detector_conf=0.0, score=0.258) == 0, (
        'this asserts the DEFECT -- if it now emits, the unclamped path was '
        'changed and this test needs rewriting, not deleting')


def test_clamping_to_the_detector_floor_restores_tracking():
    """THE FIX. Same detections, same 0.6 request, clamped."""
    assert _emitted(detector_conf=0.15, score=0.258) > 25


@pytest.mark.parametrize('score', [0.16, 0.20, 0.258, 0.44, 0.64])
def test_every_score_the_real_footage_produces_can_start_a_track(score):
    """p10 through max of the measured distribution. A box the detector
    published must be trackable at ALL of them, or the coast layer is dead
    for that part of the range and nothing says so."""
    assert _emitted(detector_conf=0.15, score=score) > 25, (
        f'score {score} -- within the measured real range -- cannot start a '
        f'track')


def test_bytetrack_is_clamped_too():
    """Both backends read these gates; fixing one would leave the other
    silently broken for whoever switches `tracker_type`."""
    assert _emitted(0.0, 0.258, ttype='bytetrack') == 0
    assert _emitted(0.15, 0.258, ttype='bytetrack') > 25


def test_the_clamp_never_LOOSENS_a_threshold():
    """It is a ceiling, not an assignment. A detector running at 0.45 must not
    drag a deliberately tight 0.2 tracker gate upward."""
    tr = RoboflowTracker(tracker_type='ocsort', track_buffer=150,
                         frame_rate=32.0, min_hits=3, iou_threshold=0.2,
                         track_activation_threshold=0.40,
                         high_conf_det_threshold=0.20, detector_conf=0.45)
    # A 0.25 box is above the tracker's own 0.20 gate and below the
    # detector's 0.45 -- it must still track.
    assert sum(1 for i in range(30)
               if tr.update([_box(0.25)], i / 32.0)) > 25


def test_zero_means_not_told_and_leaves_the_old_behaviour():
    """`detector_conf=0` is the honest default for callers that do not know
    the detector's floor -- a guess would be worse than the status quo."""
    assert _emitted(detector_conf=0.0, score=0.9) > 25   # high scores fine
    assert _emitted(detector_conf=0.0, score=0.258) == 0  # unchanged

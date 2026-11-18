"""Track box SIZE is smoothed on capture time, not per frame.

It was an EMA of 0.7 per frame: about 0.1 s at 30 Hz, 0.4 s on the forward
camera's 10 fps in dim light. The frame-count defect B-114 removed from the
coast windows, in the size a fill-based approach reads.
"""
import pytest

pytest.importorskip('rclpy')

from mongla_vision.tracker_node import smooth_sizes          # noqa: E402
from mongla_vision.tracking.tracker import TrackedDetection  # noqa: E402


def _box(w, t_id=1):
    return TrackedDetection(class_id=0, class_name='gate', score=0.9,
                            xyxy=(100.0, 100.0, 100.0 + w, 200.0),
                            track_id=t_id, predicted=False)


def _run(hz, seconds=0.3):
    ema = {}
    smooth_sizes(ema, [_box(100.0)], 0.0)
    n = int(round(seconds * hz))
    out = None
    for i in range(1, n + 1):
        out = smooth_sizes(ema, [_box(200.0)], i / hz)
    return out[0].width


def test_one_frame_at_30_hz_keeps_what_it_always_kept():
    ema = {}
    smooth_sizes(ema, [_box(100.0)], 0.0)
    w = smooth_sizes(ema, [_box(200.0)], 1.0 / 30.0)[0].width
    assert w == pytest.approx(0.7 * 100.0 + 0.3 * 200.0)


def test_the_same_seconds_give_the_same_size_at_any_rate():
    slow, mid, fast = _run(10.0), _run(30.0), _run(99.0)
    assert slow == pytest.approx(mid, abs=0.5)
    assert fast == pytest.approx(mid, abs=0.5)


def test_a_track_that_left_starts_again_from_its_own_box():
    ema = {}
    smooth_sizes(ema, [_box(100.0)], 0.0)
    smooth_sizes(ema, [], 0.1)
    assert smooth_sizes(ema, [_box(300.0)], 0.2)[0].width == 300.0

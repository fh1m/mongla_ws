"""The supervision fallback tracker reports a coasting track's own class.

`legacy_bytetrack` is what the tracker node falls back to when the `trackers`
library will not import. A track coasting through a missed frame was
reported with `class_id=0` whatever it was, so a consumer keying on the id
read a gate as class 0 for exactly the frames the target was hardest to see.
"""
import pytest

pytest.importorskip('supervision')

from mongla_vision.detection.detector import Detection  # noqa: E402
from mongla_vision.tracking.bytetrack import ByteTrackWrapper  # noqa: E402


def test_a_coasting_track_keeps_its_class_id():
    t = ByteTrackWrapper(min_hits=1)
    d = Detection(class_id=3, class_name='gate', score=0.9,
                  xyxy=(100.0, 100.0, 200.0, 200.0))
    for i in range(5):
        out = t.update([d], frame_t=i * 0.1)
    assert out and out[0].class_id == 3 and not out[0].predicted
    coast = [r for r in t.update([], frame_t=0.6) if r.predicted]
    assert coast, 'the fixture needs a coasting track'
    assert all(r.class_id == 3 and r.class_name == 'gate' for r in coast)

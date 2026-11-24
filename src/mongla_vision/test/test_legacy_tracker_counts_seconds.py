"""The legacy ByteTrack engine keeps a lost track for seconds of capture time,
not update calls, and lets any published box start a track (audit vl_15
F-01, F-02). The truth is chosen: known timestamps and scores."""
import pytest

pytest.importorskip('supervision')

from mongla_vision.detection.detector import Detection  # noqa: E402
from mongla_vision.tracking.bytetrack import ByteTrackWrapper  # noqa: E402


def _det(score=0.9):
    return Detection(class_id=0, class_name='gate', score=score,
                     xyxy=(100.0, 100.0, 200.0, 200.0))


def _last_predicted_t(dt):
    trk = ByteTrackWrapper(track_buffer=150, min_hits=1)
    t = 0.0
    for _ in range(10):
        trk.update([_det()], frame_t=t)
        t += dt
    last = None
    for _ in range(400):
        if any(d.predicted for d in trk.update([], frame_t=t)):
            last = t
        t += dt
    return last, t


@pytest.mark.parametrize('dt', [0.1, 0.01])
def test_a_lost_track_coasts_five_seconds_at_any_rate(dt):
    last, _ = _last_predicted_t(dt)
    seen = 10 * dt - dt          # the last real detection
    assert last is not None, 'no coasting at all'
    assert last - seen <= 5.0 + 1e-6, (dt, last - seen)


def test_a_box_above_the_detector_floor_starts_a_track():
    trk = ByteTrackWrapper(min_hits=1, track_activation_threshold=0.40,
                           iou_threshold=0.2, detector_conf=0.30)
    out = []
    for i in range(20):
        out = trk.update([_det(score=0.35)], frame_t=0.05 * i)
    assert out and not out[0].predicted


def test_a_moving_target_keeps_its_id():
    """25 px a frame on a 100 px box (IoU 0.6 before the Kalman learns the
    motion) at the node's default iou_threshold 0.2: one id throughout."""
    trk = ByteTrackWrapper(min_hits=1, iou_threshold=0.2)
    ids = set()
    for i in range(30):
        x = 100.0 + 25.0 * i
        d = Detection(class_id=0, class_name='gate', score=0.9,
                      xyxy=(x, 100.0, x + 100.0, 200.0))
        ids |= {o.track_id for o in trk.update([d], frame_t=0.05 * i)
                if not o.predicted}
    assert len(ids) == 1, ids

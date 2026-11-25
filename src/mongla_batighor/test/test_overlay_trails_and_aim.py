"""Where a target has been, and how far the lock is from the image centre."""
from builtin_interfaces.msg import Time
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from mongla_batighor.overlay import RED, Trails, annotate


def test_a_trail_keeps_recent_points_and_forgets_an_old_track():
    t = Trails(keep_s=2.0)
    for k in range(5):
        t.note(7, 10.0 + k, 20.0, k * 0.1, RED)
    t.note(9, 1.0, 1.0, 0.0, RED)
    lines = t.lines(0.4)
    assert len(lines) == 1 and len(lines[0][0]) == 5   # one point is no line
    assert t.lines(3.0) == [] and t.paths == {}          # forgotten when idle


def _lock(x1, y1, x2, y2):
    d = Detection2D()
    d.bbox.center.position.x, d.bbox.center.position.y = (x1 + x2) / 2, (y1 + y2) / 2
    d.bbox.size_x, d.bbox.size_y = x2 - x1, y2 - y1
    h = ObjectHypothesisWithPose()
    h.hypothesis.class_id, h.hypothesis.score = 'detection', 0.9
    d.results.append(h)
    d.id = 'gate'
    return Detection2DArray(detections=[d])


def test_the_aim_says_the_pixel_error_to_the_lock():
    out = annotate(Time(), lock=_lock(400.0, 200.0, 440.0, 240.0), size=(640, 360),
                   trails=[([(1.0, 1.0), (2.0, 2.0)], RED)])
    words = [x.text for x in out.texts]
    assert any(w == 'err +100 +40 px' for w in words), words
    assert out.points[0].type == out.points[0].LINE_STRIP      # the trail, beneath

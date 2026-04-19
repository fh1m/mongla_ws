"""messages.py tests -- Detection -> vision_msgs/Detection2D round-trip."""

import pytest

vision_msgs = pytest.importorskip('vision_msgs.msg')
from std_msgs.msg import Header

from duburi_vision.detection.detector import Detection
from duburi_vision.detection.messages import detection_to_msg, detections_to_array


def _read_center(pose2d):
    if hasattr(pose2d, 'position'):
        return pose2d.position.x, pose2d.position.y, pose2d.theta
    return pose2d.x, pose2d.y, pose2d.theta


def _read_hypothesis(result):
    if hasattr(result, 'hypothesis'):
        return result.hypothesis.class_id, result.hypothesis.score
    return result.id, result.score


def test_single_detection_geometry():
    d = Detection(class_id=0, class_name='person', score=0.81, xyxy=(10, 20, 50, 80))
    msg = detection_to_msg(d)
    cx, cy, theta = _read_center(msg.bbox.center)
    assert cx == pytest.approx(30.0)
    assert cy == pytest.approx(50.0)
    assert theta == pytest.approx(0.0)
    assert msg.bbox.size_x == pytest.approx(40.0)
    assert msg.bbox.size_y == pytest.approx(60.0)
    assert len(msg.results) == 1
    cid, score = _read_hypothesis(msg.results[0])
    assert cid == 'person'   # we publish the human label, not the numeric id
    assert score == pytest.approx(0.81, abs=1e-5)


def test_hypothesis_falls_back_to_numeric_id_when_no_name():
    d = Detection(class_id=7, class_name='', score=0.4, xyxy=(0, 0, 10, 10))
    msg = detection_to_msg(d)
    cid, _ = _read_hypothesis(msg.results[0])
    assert cid == '7'


def test_array_carries_header_to_each_detection():
    header = Header()
    header.frame_id = 'laptop_cam'

    detections = [
        Detection(class_id=0, class_name='person', score=0.9, xyxy=(0, 0, 10, 10)),
        Detection(class_id=1, class_name='cat',    score=0.5, xyxy=(5, 5, 15, 25)),
    ]

    arr = detections_to_array(detections, header)
    assert arr.header.frame_id == 'laptop_cam'
    assert len(arr.detections) == 2
    for sub in arr.detections:
        assert sub.header.frame_id == 'laptop_cam'


def test_empty_array_is_safe():
    header = Header()
    header.frame_id = 'cam'
    arr = detections_to_array([], header)
    assert arr.header.frame_id == 'cam'
    assert arr.detections == []

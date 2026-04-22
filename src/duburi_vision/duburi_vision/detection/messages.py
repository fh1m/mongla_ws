"""Convert internal Detection objects to vision_msgs.

Kept in its own module so detector code (yolo.py) stays free of rclpy
imports — easier to unit-test and easier to reuse from non-ROS scripts.

We populate `vision_msgs/Detection2D`:
  bbox.center.position : (cx, cy)        in pixels, top-left origin
  bbox.center.theta    : 0.0             (axis-aligned)
  bbox.size_x          : width           in pixels
  bbox.size_y          : height          in pixels
  results              : one ObjectHypothesisWithPose with the human
                         class label ("person", "gate", ...) as the
                         `hypothesis.class_id` string. vision_msgs
                         specifies `class_id` as a string identifier;
                         using the readable label keeps downstream
                         filters (`largest('person')`) human-friendly
                         and matches how YOLO models name their classes.
  header               : passed through (frame_id + stamp).

ROS Iron+ uses `Pose2D.position`/`Pose2D.theta` instead of the older
`Pose2D.x`/`y`/`theta` layout. We support both via the helper below so
the code keeps working when we move off Humble.
"""

from __future__ import annotations

from typing import Iterable

from vision_msgs.msg import (
    Detection2D, Detection2DArray,
    ObjectHypothesisWithPose,
)

from .detector import Detection


def _set_center(pose2d, cx, cy, theta=0.0):
    """vision_msgs Pose2D layout differs across ROS versions; support both."""
    if hasattr(pose2d, 'position'):
        pose2d.position.x = float(cx)
        pose2d.position.y = float(cy)
        pose2d.theta = float(theta)
    else:
        pose2d.x = float(cx)
        pose2d.y = float(cy)
        pose2d.theta = float(theta)


def _set_hypothesis(result, class_id, score):
    """vision_msgs Hypothesis layout differs across versions:
       Humble: result.hypothesis.class_id (str), result.hypothesis.score (f64)
       Older:  result.id (str), result.score (f64)
    """
    if hasattr(result, 'hypothesis'):
        result.hypothesis.class_id = str(class_id)
        result.hypothesis.score    = float(score)
    else:
        result.id    = str(class_id)
        result.score = float(score)


def detection_to_msg(d: Detection) -> Detection2D:
    msg = Detection2D()
    _set_center(msg.bbox.center, d.cx, d.cy, theta=0.0)
    msg.bbox.size_x = float(d.width)
    msg.bbox.size_y = float(d.height)

    # Use the human label ("person") so downstream filters are readable.
    # Falls back to the numeric id stringified when the model didn't ship names.
    label = d.class_name if d.class_name else str(d.class_id)
    hypo = ObjectHypothesisWithPose()
    _set_hypothesis(hypo, label, d.score)
    msg.results.append(hypo)
    return msg


def detections_to_array(detections: Iterable[Detection], header) -> Detection2DArray:
    arr = Detection2DArray()
    arr.header = header
    for d in detections:
        sub = detection_to_msg(d)
        sub.header = header
        arr.detections.append(sub)
    return arr


def array_to_detections(array: Detection2DArray) -> list:
    """Inverse of detections_to_array.

    Converts a Detection2DArray back to a list of Detection objects.
    Used by tracker_node to receive raw detections from detector_node.

    Returns List[Detection] in pixel space. bbox.size_x/y are width/height
    in pixels; bbox.center is cx,cy. xyxy is reconstructed from these.
    """
    from .detector import Detection as _Detection

    out = []
    for msg in array.detections:
        cx, cy = _msg_bbox_center(msg.bbox)
        w = float(msg.bbox.size_x)
        h = float(msg.bbox.size_y)
        x1 = cx - w * 0.5
        y1 = cy - h * 0.5
        x2 = cx + w * 0.5
        y2 = cy + h * 0.5

        if not msg.results:
            continue
        hyp = msg.results[0]
        if hasattr(hyp, 'hypothesis'):
            class_name = str(hyp.hypothesis.class_id)
            score      = float(hyp.hypothesis.score)
        else:
            class_name = str(getattr(hyp, 'id', ''))
            score      = float(getattr(hyp, 'score', 0.0))

        out.append(_Detection(
            class_id=0,           # integer class_id not preserved; use 0
            class_name=class_name,
            score=score,
            xyxy=(x1, y1, x2, y2),
        ))
    return out


def _msg_bbox_center(bbox):
    """Extract (cx, cy) from vision_msgs BoundingBox2D (Humble vs Iron+)."""
    centre = bbox.center
    if hasattr(centre, 'position'):
        return float(centre.position.x), float(centre.position.y)
    return float(centre.x), float(centre.y)

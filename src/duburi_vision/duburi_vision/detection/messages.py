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

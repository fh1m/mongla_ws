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
            class_id=class_index(class_name),
            class_name=class_name,
            score=score,
            xyxy=(x1, y1, x2, y2),
        ))
    return out


_CLASS_INDEX: dict = {}


def class_index(class_name: str) -> int:
    """A stable integer for a class LABEL. Never 0 for everything.

    ⛔ WHAT THIS FIXES, MEASURED. `array_to_detections` used to hard-code
    `class_id=0` with the note "integer class_id not preserved". It is not a
    cosmetic loss. Round-tripping `blood`(3) and `fire`(7) returns
    `blood`(0) and `fire`(0), and `tracking/*.py::_name_from(cid, dets)`
    resolves a class id back to a name by finding the FIRST detection carrying
    it -- so with both in frame every track resolves to `'blood'`, whichever
    one it actually is. Those are the bin task's two classes. The trackers also
    receive one single class for everything, so class-aware association cannot
    keep two different objects apart.

    ⛔ AND WHY THIS IS NOT "PUT THE ORIGINAL INTEGER ON THE WIRE". A detector's
    integer index is a property of WHICHEVER MODEL produced it -- `gate` is 0
    in `gate_rescue_repair` and something else in the next model -- so the
    integer is only meaningful alongside the model that minted it, which the
    message does not carry. `srot_protocol.py` records the same conclusion for
    the same reason. The LABEL is the identity that survives a model switch, so
    the integer is derived from it here instead.

    First-seen order, process-local. That is all any consumer needs: the
    trackers want two different classes to differ and the same class to match
    within a run, and `_name_from` maps back through this same table. It is
    deliberately NOT a hash -- a hash is stable across processes but can
    collide, and a collision here silently merges two classes.
    """
    name = str(class_name)
    if name not in _CLASS_INDEX:
        _CLASS_INDEX[name] = len(_CLASS_INDEX)
    return _CLASS_INDEX[name]


def remint_class_ids(detections):
    """Re-derive every in-process `class_id` from its LABEL. Returns the list.

    Needed only when detections from TWO models are merged. Each model numbers
    its own classes from zero, so `gate`(0) from one and `person`(0) from the
    other are the same integer in one list -- the debug palette gives them one
    colour, and anything that groups by the integer merges two classes with no
    error anywhere. `class_index` is the same label->id table the wire
    round-trip already uses, so a merged id agrees with what a subscriber will
    mint for the same label.

    Not applied on the single-model path: there the integers are the model's
    own and are already consistent with its `class_names()`.
    """
    for d in detections:
        d.class_id = class_index(d.class_name or str(d.class_id))
    return detections


def _msg_bbox_center(bbox):
    """Extract (cx, cy) from vision_msgs BoundingBox2D (Humble vs Iron+)."""
    centre = bbox.center
    if hasattr(centre, 'position'):
        return float(centre.position.x), float(centre.position.y)
    return float(centre.x), float(centre.y)


def detections_to_contours(detections, header, *, camera: str,
                           width: int, height: int):
    """Detections -> `mongla_interfaces/TargetContours`.

    Imported lazily so this module keeps working in a workspace where
    `mongla_interfaces` is not built -- the detection path must not stop
    because an optional evidence topic cannot be constructed.

    Every detection contributes exactly one contour, whether or not it carries
    a mask: a box becomes its four corners. That is what makes the message the
    same for a detector and a segmentation model, and it means a consumer can
    count on `len(class_name)` matching the detection array on the same stamp.
    """
    from mongla_interfaces.msg import TargetContours

    from .contours import contour_for, flatten, polygon_area_px

    msg = TargetContours()
    msg.header = header
    msg.camera = str(camera)
    msg.image_width = int(width)
    msg.image_height = int(height)

    polys = []
    for d in detections:
        pts, angle = contour_for(d)
        polys.append(pts)
        msg.class_name.append(str(d.class_name or d.class_id))
        msg.score.append(float(d.score))
        msg.angle_deg.append(int(angle))
        msg.area_px.append(int(round(polygon_area_px(pts))))
    offset, points = flatten(polys, width=width, height=height)
    msg.offset = offset
    msg.points = points
    return msg

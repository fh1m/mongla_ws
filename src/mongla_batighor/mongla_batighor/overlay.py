"""What the vision stack sees, drawn by the browser over the camera's own picture.

    ros2 run mongla_batighor overlay          # bringup starts it with batighor:=true

For each camera this turns the detector's boxes, the tracker's ids and the
lock's one chosen target into `foxglove_msgs/ImageAnnotations` on
`/mongla/vision/<cam>/annotations`. Lichtblick's image panel draws those over
`<cam>/stream/compressed`, the JPEG the camera sent, so Kutum never draws on
or encodes a frame for an operator (`image_debug` does both, for a recorder).

Kept out of `mongla_vision` so the vision stack does not depend on a viewer's
message package: this is the one node that knows both.
"""
from __future__ import annotations

from foxglove_msgs.msg import (Color, ImageAnnotations, Point2,
                               PointsAnnotation, TextAnnotation)

# The three layers, and how each is drawn. The lock is the one the vehicle
# steers by, so it is the one an operator should see first.
DETECTION = ((0.35, 0.85, 1.0, 0.9), 1.5)
TRACK = ((1.0, 0.8, 0.2, 0.9), 1.5)
LOCK = ((1.0, 0.42, 0.1, 1.0), 3.5)
_TEXT_BG = Color(r=0.0, g=0.0, b=0.0, a=0.6)


def _color(rgba) -> Color:
    return Color(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])


def _box(det):
    """(x1, y1, x2, y2, label, score) from one `vision_msgs/Detection2D`."""
    c = det.bbox.center
    cx, cy = (c.position.x, c.position.y) if hasattr(c, 'position') else (c.x, c.y)
    w, h = det.bbox.size_x, det.bbox.size_y
    label, score = '', None
    if det.results:
        hyp = det.results[0]
        hyp = getattr(hyp, 'hypothesis', hyp)
        label = str(getattr(hyp, 'class_id', getattr(hyp, 'id', '')))
        score = float(hyp.score)
    return cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2, label, score


def annotate(stamp, detections=None, tracks=None, lock=None) -> ImageAnnotations:
    """One frame's annotations from the newest message of each layer.

    Any layer may be None (its node is not running). Boxes are in the image's
    own pixels, which is what the detector publishes, so they land on the
    picture with no scaling.
    """
    out = ImageAnnotations(timestamp=stamp)
    for layer, style, text in ((detections, DETECTION, 'score'),
                               (tracks, TRACK, 'id'),
                               (lock, LOCK, 'lock')):
        if layer is None:
            continue
        rgba, thickness = style
        for det in layer.detections:
            x1, y1, x2, y2, label, score = _box(det)
            out.points.append(PointsAnnotation(
                timestamp=stamp, type=PointsAnnotation.LINE_LOOP,
                points=[Point2(x=x1, y=y1), Point2(x=x2, y=y1),
                        Point2(x=x2, y=y2), Point2(x=x1, y=y2)],
                outline_color=_color(rgba), thickness=thickness))
            if text == 'score':
                words = f'{label} {score:.2f}' if score is not None else label
            elif text == 'id':
                words = f'#{det.id}' if det.id else ''
            else:
                words = f'LOCK {label}'.strip()
            if not words:
                continue
            # A text's position is its lower left corner. The detector's label
            # sits above the box, a track's below it and the lock's inside it,
            # so a box carrying all three shows all three (on Kutum, the lock's
            # label above the box covered the detector's).
            x, y = x1, y1 - 2
            if text == 'id':
                x, y = x1, y2 + 15
            elif text == 'lock':
                x, y = x1 + 3, y1 + 16
            out.texts.append(TextAnnotation(
                timestamp=stamp, position=Point2(x=x, y=max(y, 13.0)),
                text=words, font_size=13.0, text_color=_color(rgba),
                background_color=_TEXT_BG))
    return out


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from vision_msgs.msg import Detection2DArray

    from mongla_vision import qos

    class Overlay(Node):
        def __init__(self):
            super().__init__('batighor_overlay')
            cams = list(self.declare_parameter(
                'cameras', ['forward', 'downward']).value or [])
            for cam in cams:
                ns = f'/mongla/vision/{cam}'
                latest = {'tracks': None, 'lock': None}
                pub = self.create_publisher(ImageAnnotations, f'{ns}/annotations', 5)

                def on_detections(msg, latest=latest, pub=pub):
                    if pub.get_subscription_count() == 0:
                        return
                    pub.publish(annotate(msg.header.stamp, msg,
                                         latest['tracks'], latest['lock']))

                def keep(key, latest=latest):
                    return lambda msg: latest.__setitem__(key, msg)

                # The detector paces the overlay: one set of boxes per frame
                # it saw. The tracker and the lock arrive on their own clocks
                # and are drawn as they last were.
                self.create_subscription(Detection2DArray, f'{ns}/detections',
                                         on_detections, qos.DETECTIONS)
                self.create_subscription(Detection2DArray, f'{ns}/tracks',
                                         keep('tracks'), qos.DETECTIONS)
                self.create_subscription(Detection2DArray, f'{ns}/lock',
                                         keep('lock'), qos.DETECTIONS)

    from mongla_localization.params_file import refuse_unknown
    from mongla_localization.spin import spin
    rclpy.init(args=args)
    node = Overlay()
    refuse_unknown(node)       # bringup.yaml `nodes:` typos stop here
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

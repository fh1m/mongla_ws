"""What the vision stack sees, drawn by the browser over the camera's own picture.

    ros2 run mongla_batighor overlay          # bringup starts it with batighor:=true

For each camera this turns into `foxglove_msgs/ImageAnnotations` on
`/mongla/vision/<cam>/annotations`:

  * the detector's boxes with class and score, and the candidates below its
    floor, faint;
  * the tracker's ids;
  * the lock, coloured by the rung of the ladder holding it, labelled with
    what it holds and its authority;
  * the targets' contours;
  * the anchor's matched features (its learned points), while it runs.

The flow node's tracked pairs go to `<cam>/flow_annotations` (a segment from
where each point was on the anchor frame to where it is now), and a running
calibration's state to `/mongla/vision/calibration/annotations` (the corners,
the cell the next pose belongs in, the point the fit is least sure of).
Lichtblick's image panel draws them over `<cam>/stream/compressed`, the JPEG
the camera sent, so Kutum never draws on or encodes a frame for an operator.

Nothing here runs unwatched: each group subscribes only while a browser
subscribes to what it publishes (`Watched`).

Kept out of `mongla_vision` so the vision stack does not depend on a viewer's
message package: this is the one node that knows both.
"""
from __future__ import annotations

import json

from foxglove_msgs.msg import (Color, ImageAnnotations, Point2,
                               PointsAnnotation, TextAnnotation)

# The Mongla palette (docs/assets/site.css): RED marks something measured,
# OCEAN what the machine perceives, and the rest is the dark it works in. The
# lock is the one the vehicle steers by, so it is the one an operator should
# see first, and its colour says which rung of the ladder is holding it.


def _hex(h, a=1.0):
    return (int(h[1:3], 16) / 255, int(h[3:5], 16) / 255, int(h[5:7], 16) / 255, a)


SEA = _hex('#5a8dff')           # --sea-lit: what the machine perceives
RED = _hex('#ff2a14')           # --red-lit: measured, live
AMBER = _hex('#f88701')         # --cad-amber: caution, carried
GLASS = _hex('#c4e2f3')         # --cad-glass
VIOLET = _hex('#9a7bff')        # --state-soft: learned features
TEAL = _hex('#22d3c5')          # --state-water
INK_2 = _hex('#98a2b4')
INK_3 = _hex('#7b8598', 0.7)

DETECTION = (SEA, 1.5)
CANDIDATE = (INK_3, 1.0)
TRACK = (INK_2, 1.5)
LOCK = (RED, 3.5)
FLOW = (TEAL, 1.5)
CONTOUR = (_hex('#5a8dff', 0.8), 1.5)
FEATURE = (VIOLET, 4.0)
TARGET = (AMBER, 3.0)
WORST = (RED, 3.0)
# Each rung of the lock ladder (`tracking/lock_state.Rung`), by how much of it
# was measured this frame: a detection is a measurement, a held box a weak
# one kept, the follower an image tracker carrying it, the anchor learned
# features matched against a stored view.
RUNG = {'detection': RED, 'held': AMBER, 'follow': GLASS, 'anchor': VIOLET}
_TEXT_BG = Color(r=0.02, g=0.027, b=0.047, a=0.75)


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


# The anchor publishes its matches only while it runs, so the last set would
# stay on the picture after it stopped. Older than this against the frame
# being drawn, it is left off.
FEATURES_MAX_AGE_S = 0.5


def fresh(msg, stamp, max_age_s) -> bool:
    """Whether `msg` (anything with a header) is within `max_age_s` of `stamp`."""
    if msg is None:
        return False
    s = msg.header.stamp
    return abs((s.sec - stamp.sec) + (s.nanosec - stamp.nanosec) * 1e-9) <= max_age_s


def _loop(stamp, xy, rgba, thickness):
    return PointsAnnotation(
        timestamp=stamp, type=PointsAnnotation.LINE_LOOP,
        points=[Point2(x=float(x), y=float(y)) for x, y in xy],
        outline_color=_color(rgba), thickness=thickness)


def _text(stamp, x, y, words, rgba, size=13.0):
    return TextAnnotation(
        timestamp=stamp, position=Point2(x=float(x), y=max(float(y), 13.0)),
        text=words, font_size=size, text_color=_color(rgba), background_color=_TEXT_BG)


def annotate(stamp, detections=None, tracks=None, lock=None, candidates=None,
             contours=None, features=None, size=None) -> ImageAnnotations:
    """One frame's annotations from the newest message of each layer.

    Any layer may be None (its node is not running). Boxes are in the image's
    own pixels, which is what the detector publishes, so they land on the
    picture with no scaling. `features` (the anchor's matched points) are in
    the matcher's pixels and are scaled by `size`, the camera's (width, height).
    """
    out = ImageAnnotations(timestamp=stamp)
    if candidates is not None:
        # Below the detector's floor: what the model almost saw, faint.
        rgba, th = CANDIDATE
        for det in candidates.detections:
            x1, y1, x2, y2, _, _ = _box(det)
            out.points.append(_loop(stamp, ((x1, y1), (x2, y1), (x2, y2), (x1, y2)), rgba, th))
    if contours is not None:
        rgba, th = CONTOUR
        offs, pts = list(contours.offset), list(contours.points)
        for i in range(len(offs)):
            a, b = offs[i], offs[i + 1] if i + 1 < len(offs) else len(pts) // 2
            ring = [(pts[2 * k], pts[2 * k + 1]) for k in range(a, b)]
            if len(ring) >= 3:
                out.points.append(_loop(stamp, ring, rgba, th))
    if features is not None and features.image_points:
        sx = sy = 1.0
        if size and features.image_width and features.image_height:
            sx = size[0] / features.image_width
            sy = size[1] / features.image_height
        p = list(features.image_points)
        rgba, th = FEATURE
        out.points.append(PointsAnnotation(
            timestamp=stamp, type=PointsAnnotation.POINTS,
            points=[Point2(x=p[2 * i] * sx, y=p[2 * i + 1] * sy) for i in range(len(p) // 2)],
            outline_color=_color(rgba), thickness=th))
    for layer, style, text in ((detections, DETECTION, 'score'),
                               (tracks, TRACK, 'id'),
                               (lock, LOCK, 'lock')):
        if layer is None:
            continue
        rgba, thickness = style
        for det in layer.detections:
            x1, y1, x2, y2, label, score = _box(det)
            if text == 'lock':
                rgba = RUNG.get(label, LOCK[0])
            out.points.append(_loop(stamp, ((x1, y1), (x2, y1), (x2, y2), (x1, y2)),
                                    rgba, thickness))
            if text == 'score':
                words = f'{label} {score:.2f}' if score is not None else label
            elif text == 'id':
                words = f'#{det.id}' if det.id else ''
            else:
                # What it holds, which rung holds it, and its authority.
                auth = f'{score:.2f}' if score is not None and det.id else ''
                words = ' '.join(filter(None, ('LOCK', det.id, label, auth)))
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
            out.texts.append(_text(stamp, x, y, words, rgba))
    return out


def flow_annotate(tracks) -> ImageAnnotations:
    """One flow interval's pairs: a segment anchor -> now, and a dot at now."""
    stamp = tracks.header.stamp
    a, b = list(tracks.anchor_xy), list(tracks.now_xy)
    n = min(len(a), len(b)) // 2
    rgba, thickness = FLOW
    out = ImageAnnotations(timestamp=stamp)
    if not n:
        return out
    segs, dots = [], []
    for i in range(n):
        p0 = Point2(x=float(a[2 * i]), y=float(a[2 * i + 1]))
        p1 = Point2(x=float(b[2 * i]), y=float(b[2 * i + 1]))
        segs += [p0, p1]
        dots.append(p1)
    out.points.append(PointsAnnotation(
        timestamp=stamp, type=PointsAnnotation.LINE_LIST, points=segs,
        outline_color=_color(rgba), thickness=thickness))
    out.points.append(PointsAnnotation(
        timestamp=stamp, type=PointsAnnotation.POINTS, points=dots,
        outline_color=_color(rgba), thickness=4.0))
    out.texts.append(TextAnnotation(
        timestamp=stamp, position=Point2(x=8.0, y=20.0), text=f'{n} tracks',
        font_size=13.0, text_color=_color(rgba), background_color=_TEXT_BG))
    return out


def calibrate_annotate(state: dict, stamp) -> ImageAnnotations:
    """A calibration's state (`calibrate_node.state_dict`) over its camera:
    the board corners found, the cell the next pose belongs in, the point
    the fit is least sure of, and the instruction. All in full-frame pixels."""
    out = ImageAnnotations(timestamp=stamp)

    def pts(kind, xy, rgba, thickness):
        out.points.append(PointsAnnotation(
            timestamp=stamp, type=kind, outline_color=_color(rgba), thickness=thickness,
            points=[Point2(x=float(x), y=float(y)) for x, y in xy]))

    if state.get('corners'):
        pts(PointsAnnotation.POINTS, state['corners'], FLOW[0], 5.0)
    if state.get('target'):
        x0, y0, x1, y1 = state['target']
        pts(PointsAnnotation.LINE_LOOP, [(x0, y0), (x1, y0), (x1, y1), (x0, y1)], *TARGET)
    if state.get('worst_px'):
        x, y = state['worst_px']
        pts(PointsAnnotation.LINE_LIST, [(x - 12, y - 12), (x + 12, y + 12),
                                         (x - 12, y + 12), (x + 12, y - 12)], *WORST)
    if state.get('done'):
        line = 'all poses captured: solve'
    else:
        line = (f"{state.get('i', 0) + 1}/{state.get('n', 0)}  "
                f"{state.get('want_cell')}, {state.get('want_tilt')}  "
                f"hold {state.get('hold', 0):.0%}")
    for k, text in enumerate(filter(None, (line, state.get('msg'), state.get('err')))):
        out.texts.append(TextAnnotation(
            timestamp=stamp, position=Point2(x=8.0, y=24.0 + 22.0 * k), text=str(text),
            font_size=16.0, text_color=_color(TARGET[0] if k else FLOW[0]),
            background_color=_TEXT_BG))
    return out


class Watched:
    """Subscriptions that exist only while someone watches what they feed.

    The overlay serves browsers, and with none open it must cost the vehicle
    nothing: no deliveries into this process, and no subscriber that keeps a
    publisher working for nobody. `flow_node` builds its track pairs only while
    `flow_tracks` has a subscriber, so an overlay subscribed for good kept the
    flow node doing that work all session. `poll` subscribes when `pub` gains
    a subscriber and unsubscribes when it has none; `on_drop` forgets what the
    layers held, so the next viewer never sees a lock from minutes ago.
    """

    def __init__(self, node, pub, specs, on_drop=None):
        self.node, self.pub, self.specs, self.on_drop = node, pub, specs, on_drop
        self.subs = []

    def poll(self):
        watched = self.pub.get_subscription_count() > 0
        if watched and not self.subs:
            self.subs = [self.node.create_subscription(*s) for s in self.specs]
        elif not watched and self.subs:
            for s in self.subs:
                self.node.destroy_subscription(s)
            self.subs = []
            if self.on_drop:
                self.on_drop()


# How soon a new viewer's annotations start, and the cost of asking: one
# subscriber count per published layer, twice a second.
WATCH_PERIOD_S = 0.5


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import CameraInfo
    from std_msgs.msg import String
    from vision_msgs.msg import Detection2DArray

    from mongla_interfaces.msg import FlowTracks, TargetContours, TargetCorrespondences
    from mongla_vision import qos

    LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    LAYERS = ('tracks', 'lock', 'candidates', 'contours', 'features', 'size')

    class Overlay(Node):
        def __init__(self):
            super().__init__('batighor_overlay')
            cams = list(self.declare_parameter(
                'cameras', ['forward', 'downward']).value or [])
            self.watched = []
            for cam in cams:
                ns = f'/mongla/vision/{cam}'
                latest = dict.fromkeys(LAYERS)
                pub = self.create_publisher(ImageAnnotations, f'{ns}/annotations', 5)

                def on_detections(msg, latest=latest, pub=pub):
                    feats = latest['features']
                    pub.publish(annotate(
                        msg.header.stamp, msg, latest['tracks'], latest['lock'],
                        latest['candidates'], latest['contours'],
                        feats if fresh(feats, msg.header.stamp, FEATURES_MAX_AGE_S) else None,
                        latest['size']))

                def on_info(msg, latest=latest):
                    latest['size'] = (msg.width, msg.height) if msg.width else None

                def keep(key, latest=latest):
                    return lambda msg: latest.__setitem__(key, msg)

                # The detector paces the overlay: one set of boxes per frame
                # it saw. The other layers arrive on their own clocks and are
                # drawn as they last were. The anchor's matched features are
                # in the matcher's pixels; the camera's size scales them.
                self.watched.append(Watched(self, pub, [
                    (Detection2DArray, f'{ns}/detections', on_detections, qos.DETECTIONS),
                    (Detection2DArray, f'{ns}/tracks', keep('tracks'), qos.DETECTIONS),
                    (Detection2DArray, f'{ns}/lock', keep('lock'), qos.DETECTIONS),
                    (Detection2DArray, f'{ns}/candidates', keep('candidates'), qos.DETECTIONS),
                    (TargetContours, f'{ns}/contours', keep('contours'), qos.DETECTIONS),
                    (TargetCorrespondences, f'{ns}/correspondences', keep('features'),
                     qos.DETECTIONS),
                    (CameraInfo, f'{ns}/camera_info', on_info, qos.CAMERA_INFO),
                ], on_drop=lambda latest=latest: latest.update(dict.fromkeys(LAYERS))))

                flow_pub = self.create_publisher(
                    ImageAnnotations, f'{ns}/flow_annotations', 5)
                self.watched.append(Watched(self, flow_pub, [
                    (FlowTracks, f'{ns}/flow_tracks',
                     lambda msg, p=flow_pub: p.publish(flow_annotate(msg)), 5)]))

            # A calibration, when one runs (`calibrate.launch.py`): its state
            # is latched, so a new viewer gets the current one at once.
            cal = '/mongla/vision/calibration'
            cal_pub = self.create_publisher(ImageAnnotations, f'{cal}/annotations', 5)

            def on_calibration(msg):
                cal_pub.publish(calibrate_annotate(
                    json.loads(msg.data), self.get_clock().now().to_msg()))

            self.watched.append(Watched(self, cal_pub, [
                (String, f'{cal}/state', on_calibration, LATCHED)]))
            self.create_timer(WATCH_PERIOD_S, self._poll)

        def _poll(self):
            for w in self.watched:
                w.poll()

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

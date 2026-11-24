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
  * the anchor's matched features (its learned points), while it runs;
  * a fading trail behind every track and the lock (`Trails`), and the aim:
    the image centre joined to the lock with the pixel error it shows.

The flow node's tracked pairs go to `<cam>/flow_annotations` (a segment from
where each point was on the anchor frame to where it is now), what it read off
the floor to `<cam>/floor_annotations` (every grout line the tile grating puts
on the picture, and the lane line's centreline), and a running
calibration's state to `/mongla/vision/calibration/annotations` (the corners,
the cell the next pose belongs in, the point the fit is least sure of).
Lichtblick's image panel draws them over `<cam>/stream/compressed`, the JPEG
the camera sent, so Kutum never draws on or encodes a frame for an operator.

Nothing here runs unwatched: each group subscribes only while a browser
subscribes to what it publishes (`Watched`).

Kept out of `mongla_vision` so the vision stack does not depend on a viewer's
message package: this is the one node that knows both.

For the 3D scene it also turns `/mongla/demand` into an arrow on the hull
(`demand_scene`, on `/mongla/batighor/scene`), the same way: only while
watched.
"""
from __future__ import annotations

import json
import time
import zlib
from collections import deque

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

CANDIDATE = (INK_3, 1.0)
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
# Text is in the image's pixels, and the picture is drawn smaller than it
# is: 13 px on a 480-line frame in a cockpit tile read ~9 px on screen
# (desktop app, 2026-11-12). A 26th of a 480-line frame.
LABEL_PX = 18.0


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


def _key(box):
    """A box to the half pixel, without its label: the same box on two topics."""
    return tuple(round(v * 2) for v in box[:4])


def iou(a, b) -> float:
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


# A track and a detection are the same thing when they overlap this much.
SAME_THING_IOU = 0.3

# One colour per class, so two kinds of thing never look alike. None of them
# is the lock's red, the flow's teal or a rung's colour, which say other things.
CLASS_COLOURS = [_hex(h) for h in ('#5a8dff', '#ffd166', '#06d6a0', '#c77dff',
                                   '#4cc9f0', '#f4a261', '#ff9ebb', '#90be6d')]
COASTING = (INK_3, 1.5)


def class_colour(label: str, alpha: float = 1.0):
    """The same colour for a class on every frame, camera and run."""
    r, g, b, _ = CLASS_COLOURS[zlib.crc32(label.encode()) % len(CLASS_COLOURS)]
    return (r, g, b, alpha)


def same_frame(msg, stamp) -> bool:
    """Whether `msg` describes the frame stamped `stamp`."""
    return msg is not None and msg.header.stamp == stamp


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


def _text(stamp, x, y, words, rgba, size=LABEL_PX):
    return TextAnnotation(
        timestamp=stamp, position=Point2(x=float(x), y=max(float(y), size)),
        text=words, font_size=size, text_color=_color(rgba), background_color=_TEXT_BG)


# Lichtblick draws annotation text monospace; one character is about 0.6 of
# the font size wide (measured off a cockpit shot: 14 characters, 145 px at 18).
CHAR_EM = 0.6


class _Labels:
    """Places each label in the first free slot so no two cover each other.

    A label's position is its baseline's left end. `want` is the slots in
    order of preference; the first that overlaps nothing placed and stays on
    the picture wins, else the first, clamped. Placing nothing would hide a
    detection, which is worse than a touch."""

    def __init__(self, stamp, size):
        self.stamp, self.size, self.taken = stamp, size, []

    def _rect(self, x, y, words, px):
        w = len(words) * px * CHAR_EM
        if self.size:
            x = min(x, self.size[0] - w)
        x = max(x, 0.0)
        y = max(y, px)
        if self.size:
            y = min(y, float(self.size[1]))
        return (x, y - px, x + w, y)

    def _free(self, r):
        return all(r[2] <= o[0] or o[2] <= r[0] or r[3] <= o[1] or o[3] <= r[1]
                   for o in self.taken)

    def place(self, want, words, rgba, px=LABEL_PX):
        rects = [self._rect(x, y, words, px) for x, y in want]
        r = next((r for r in rects if self._free(r)), rects[0])
        self.taken.append(r)
        return _text(self.stamp, r[0], r[3], words, rgba, px)


def _slots(b, px):
    """Above the box and higher, then below and lower, then just inside."""
    x1, y1, _, y2 = b[:4]
    step = px + 2
    return ([(x1, y1 - 2 - k * step) for k in range(3)]
            + [(x1, y2 + px + 2 + k * step) for k in range(3)]
            + [(x1 + 3, y1 + px + 3)])


# A detector at its slowest answers several times a second; a second of
# nothing is a detector that is not running.
SILENT_S = 1.0


def silence(arrivals, since, now):
    """Seconds the detector has said nothing, or None while it answers.

    `arrivals`: monotonic times of its answers; `since`: when this node began
    watching, which stands in for an answer until the first one comes."""
    gap = now - (arrivals[-1] if arrivals else since)
    return gap if gap >= SILENT_S else None


UNREAD = 'unread'


def hud(cam, model, size, arrivals, latency_s, silent_s=None, paused=UNREAD):
    """What the camera and detector are doing, in two lines over the picture.

    `arrivals`: monotonic times the detector's last answers came (its rate);
    `latency_s`: from the frame's capture to its boxes reaching this node;
    `silent_s`: how long the detector has said nothing (`silence`), which
    replaces the rate, said by `paused`, the detector's own parameter: True
    (paused until a mission asks: the picture must not look like a dead
    camera), False (silent and not paused: stopped or stalled, an alarm),
    None (no node publishes this camera's detections), or UNREAD."""
    shape = f'{size[0]}x{size[1]}' if size else '--'
    first = '  '.join(filter(None, (cam, model or 'no model named', shape)))
    if silent_s is not None:
        s = f'{silent_s:.0f} s'
        return [first, {True: f'detector paused {s}: it runs when a mission asks',
                        False: f'DETECTOR SILENT {s}, NOT paused: stopped or stalled',
                        None: f'NO DETECTOR publishes {cam} detections ({s})'}.get(
                            paused, f'DETECTOR SILENT {s}, its pause not yet read')]
    if len(arrivals) >= 2 and arrivals[-1] > arrivals[0]:
        rate = f'{(len(arrivals) - 1) / (arrivals[-1] - arrivals[0]):.0f} Hz'
    else:
        rate = '-- Hz'
    late = f'{latency_s * 1000:.0f} ms' if latency_s is not None else '-- ms'
    return [first, f'detector {rate}  {late} behind the frame']


def caption(cam, model, size, arrivals, latency_s, silent_s=None, paused=UNREAD) -> dict:
    """The camera's and detector's state, for the caption under the picture.

    It was drawn into the picture's bottom left, where it covered the floor a
    downward box sits on and read as part of the scene (owner, 2026-11-24).
    `state`: running, paused (waits for a mission), silent (not paused: an
    alarm), none (nothing publishes this camera's detections) or unread (the
    detector's pause not yet read). `lines` is the same said in words."""
    if silent_s is None:
        state = 'running'
    else:
        state = {True: 'paused', False: 'silent', None: 'none'}.get(paused, 'unread')
    rate = None
    if silent_s is None and len(arrivals) >= 2 and arrivals[-1] > arrivals[0]:
        rate = round((len(arrivals) - 1) / (arrivals[-1] - arrivals[0]), 1)
    return {'cam': cam, 'model': model or '', 'size': list(size) if size else None,
            'state': state, 'rate_hz': rate,
            'late_ms': round(latency_s * 1000) if latency_s is not None and silent_s is None else None,
            'silent_s': round(silent_s, 1) if silent_s is not None else None,
            'alarm': state not in ('running', 'paused'),
            'lines': hud(cam, model, size, arrivals, latency_s, silent_s, paused)}


# How long a caption waits before it is sent again with only its numbers
# changed: the detector paces the overlay at 20-30 Hz, a caption is read.
CAPTION_EVERY_S = 0.5


# How long a trail is remembered: about two seconds of a target's path at the
# detector's rate, enough to read a drift or a swing, short enough that the
# picture does not fill with the past.
TRAIL_S = 2.0
TRAIL_POINTS = 60
AIM = (GLASS, 1.5)


class Trails:
    """Where each track and the lock have been, as fading polylines.

    Keyed by track id (or 'lock'); a key not seen for TRAIL_S is forgotten.
    Pure bookkeeping, so the node and a test drive it the same way."""

    def __init__(self, keep_s: float = TRAIL_S, points: int = TRAIL_POINTS):
        self.keep_s, self.points, self.paths = keep_s, points, {}

    def note(self, key, x: float, y: float, now: float, rgba) -> None:
        path = self.paths.setdefault(key, [deque(maxlen=self.points), rgba])
        path[0].append((now, x, y))
        path[1] = rgba

    def lines(self, now: float):
        """[(points, rgba)] for every path with two or more fresh points."""
        out = []
        for key in list(self.paths):
            pts, rgba = self.paths[key]
            while pts and now - pts[0][0] > self.keep_s:
                pts.popleft()
            if not pts:
                del self.paths[key]
            elif len(pts) >= 2:
                out.append(([(x, y) for _, x, y in pts], rgba))
        return out


def _centre(b):
    return (b[0] + b[2]) * 0.5, (b[1] + b[3]) * 0.5


def _corners(b):
    x1, y1, x2, y2 = b[:4]
    return ((x1, y1), (x2, y1), (x2, y2), (x1, y2))


def annotate(stamp, detections=None, tracks=None, lock=None, candidates=None,
             contours=None, features=None, size=None,
             trails=()) -> ImageAnnotations:
    """One frame's annotations: everything the vision stack holds, each as itself.

    Nothing is left off; each thing says what it is. A detection is drawn in
    its class's colour with class, score and the track carrying it. A track
    on a box below the floor is faint and says `weak`; one with nothing under
    it this frame is grey and says `coasting` (the tracker predicting). A
    candidate below the floor is thin, with its class and score. The lock is
    drawn over all of them in its rung's colour. What the camera and detector
    are doing is not drawn here: it is the caption under the picture
    (`caption`, on `<camera>/hud`).

    Any layer may be None (its node is not running). Boxes are in the image's
    own pixels, which is what the detector publishes, so they land on the
    picture with no scaling. `features` (the anchor's matched points) are in
    the matcher's pixels and are scaled by `size`, the camera's (width, height).
    `trails`: [(points, rgba)] from `Trails.lines`, drawn under everything.
    With a lock and a known size, the image centre is crossed and joined to
    the lock's centre, labelled with the pixel error an alignment closes.
    """
    out = ImageAnnotations(timestamp=stamp)
    for pts, rgba in trails:
        out.points.append(PointsAnnotation(
            timestamp=stamp, type=PointsAnnotation.LINE_STRIP,
            points=[Point2(x=x, y=y) for x, y in pts],
            outline_color=_color(rgba[:3] + (0.6,)), thickness=2.0))
    labels = _Labels(stamp, size)
    dets = [_box(d) for d in (detections.detections if detections else [])]
    # `candidates` carries the detections too (detector_node); drawn again
    # beneath them each box showed twice.
    shown = {_key(b) for b in dets}
    weak = [_box(d) for d in (candidates.detections if candidates else [])
            if _key(_box(d)) not in shown]
    trks = [(d.id, _box(d)) for d in (tracks.detections if tracks else [])]
    # Each track is named on one box, its best overlap, and each box carries
    # one track. Matched by any overlap, two boxes on one bin each said
    # "#783 #785" and neither said which track was following it.
    boxes = dets + weak
    pairs = sorted(((iou(b, t), i, j) for i, b in enumerate(boxes)
                    for j, (tid, t) in enumerate(trks) if tid), reverse=True)
    owner, held = {}, set()
    for ov, i, j in pairs:
        if ov < SAME_THING_IOU:
            break
        if i not in owner and j not in held:
            owner[i] = trks[j][0]
            held.add(j)

    def carried(b):
        i = next(k for k, x in enumerate(boxes) if x is b)
        return [owner[i]] if i in owner else []

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
    # Boxes below the floor are drawn first, so the detections lie over them.
    # Labels are placed in order of importance (detections, then the lock, then
    # the weak and coasting ones), so the most important get their first slot.
    small = LABEL_PX * 0.8
    texts = []
    for b in dets:
        rgba = class_colour(b[4])
        ids = ' '.join(f'#{t}' for t in carried(b))
        words = ' '.join(filter(None, (b[4], f'{b[5]:.2f}' if b[5] is not None else '', ids)))
        texts.append(labels.place(_slots(b, LABEL_PX), words, rgba))
    if lock is not None:
        for det in lock.detections:
            x1, y1, x2, y2, label, score = _box(det)
            rgba = RUNG.get(label, LOCK[0])
            # What it holds, which rung holds it, and its authority; inside the
            # box first, where the detector's label above it is not.
            auth = f'{score:.2f}' if score is not None and det.id else ''
            words = ' '.join(filter(None, ('LOCK', det.id, label, auth)))
            texts.append(labels.place([(x1 + 3, y1 + LABEL_PX + 3)] + _slots(_box(det), LABEL_PX),
                                      words, rgba))
    for b in weak:
        rgba = class_colour(b[4], 0.55)
        out.points.append(_loop(stamp, _corners(b), rgba, CANDIDATE[1]))
        ids = ' '.join(f'#{t}' for t in carried(b))
        words = ' '.join(filter(None, (ids, b[4], 'weak', f'{b[5]:.2f}' if b[5] is not None else '')))
        texts.append(labels.place(_slots(b, small)[3:] + _slots(b, small)[:3], words, rgba, small))
    for b in dets:
        out.points.append(_loop(stamp, _corners(b), class_colour(b[4]), 2.5))
    for j, (tid, t) in enumerate(trks):
        if j in held or (not tid and any(iou(t, b) >= SAME_THING_IOU for b in boxes)):
            continue                          # named on the box it carries
        rgba, th = COASTING
        out.points.append(_loop(stamp, _corners(t), rgba, th))
        words = ' '.join(filter(None, (f'#{tid}' if tid else '', t[4], 'coasting')))
        texts.append(labels.place(_slots(t, small)[3:] + _slots(t, small)[:3], words, rgba, small))
    if lock is not None:
        for det in lock.detections:
            x1, y1, x2, y2, label, _ = _box(det)
            out.points.append(_loop(stamp, _corners((x1, y1, x2, y2)),
                                    RUNG.get(label, LOCK[0]), LOCK[1]))
        if size and lock.detections:
            cx, cy = size[0] * 0.5, size[1] * 0.5
            lx, ly = _centre(_box(lock.detections[0]))
            rgba, th = AIM
            out.points.append(PointsAnnotation(
                timestamp=stamp, type=PointsAnnotation.LINE_LIST, outline_color=_color(rgba),
                thickness=th, points=[Point2(x=cx - 14, y=cy), Point2(x=cx + 14, y=cy),
                                      Point2(x=cx, y=cy - 14), Point2(x=cx, y=cy + 14),
                                      Point2(x=cx, y=cy), Point2(x=lx, y=ly)]))
            texts.append(labels.place([(cx + 8, cy + LABEL_PX + 8), (cx + 8, cy - 8)],
                                      f'err {lx - cx:+.0f} {ly - cy:+.0f} px', rgba,
                                      LABEL_PX * 0.8))
    out.texts.extend(texts)
    if not (out.points or out.texts or out.circles):
        # Lichtblick files a synchronised annotation under each shape's own
        # stamp and never reads the message's: with no shape the frame is
        # never matched and the picture stays blank. One empty point set on
        # the frame's stamp draws nothing and keeps it showing.
        out.points.append(PointsAnnotation(timestamp=stamp, type=PointsAnnotation.POINTS))
    return out


# The flow count sits on the picture's bottom line, left. At the top left it
# covered the label of any box that touched the frame's top edge, which is
# pinned there; the camera's own state is the caption under the picture.
FLOW_LINE_Y = LABEL_PX + 6                 # until the camera has said its size


def flow_line_y(size) -> float:
    return float(size[1] - 8) if size else FLOW_LINE_Y


def flow_annotate(tracks, size=None) -> ImageAnnotations:
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
        timestamp=stamp, position=Point2(x=8.0, y=flow_line_y(size)), text=f'flow {n} points',
        font_size=LABEL_PX, text_color=_color(rgba), background_color=_TEXT_BG))
    return out


# The floor as the flow node read it: the grout in pale glass, so the tiles
# stay visible beneath it, the lane's centreline in amber.
GROUT = (_hex('#c4e2f3', 0.55), 1.5)
LANE = (AMBER, 3.0)


def floor_annotate(read) -> ImageAnnotations:
    """One floor read (`FloorRead`) over its frame: every grout line the
    grating's period, angle and offsets put on the picture, the lane line's
    centreline, and what each one says in words, top left."""
    from mongla_vision.flow.floor_read import grid_segments, lane_segment
    stamp = read.header.stamp
    out = ImageAnnotations(timestamp=stamp)
    size = (read.width, read.height)
    words = []
    if read.tiles and read.period_px > 0 and len(read.grout_normal_deg) == len(read.grout_offset_px):
        segs = grid_segments(list(zip(read.grout_normal_deg, read.grout_offset_px)),
                             read.period_px, size)
        rgba, th = GROUT
        out.points.append(PointsAnnotation(
            timestamp=stamp, type=PointsAnnotation.LINE_LIST, outline_color=_color(rgba),
            thickness=th, points=[Point2(x=float(x), y=float(y)) for s in segs for x, y in s]))
        h = read.tile_height_m
        words.append((f'tiles {read.period_px:.0f} px'
                      + (f'  {h:.2f} m above' if h == h else '  no tile size')
                      + f'  grid {read.grout_normal_deg[0] % 90:.0f} deg', GROUT[0]))
    else:
        words.append(('tiles: none read', INK_2))
    if read.lane:
        seg = lane_segment(read.lane_direction_deg, read.lane_offset_px, size)
        if seg:
            rgba, th = LANE
            out.points.append(PointsAnnotation(
                timestamp=stamp, type=PointsAnnotation.LINE_STRIP, outline_color=_color(rgba),
                thickness=th, points=[Point2(x=float(x), y=float(y)) for x, y in seg]))
        words.append((f'lane {read.lane_direction_deg:.0f} deg  {read.lane_offset_px:+.0f} px off centre',
                      AMBER))
    # Each line in the colour of what it names: keyed on the read rather than
    # the line, "tiles: none read" was drawn in the lane's amber whenever a
    # lane was read.
    for k, (line, rgba) in enumerate(words):
        out.texts.append(TextAnnotation(
            timestamp=stamp, position=Point2(x=8.0, y=LABEL_PX + 6 + k * (LABEL_PX + 6)),
            text=line, font_size=LABEL_PX * 0.9,
            text_color=_color(rgba),
            background_color=_TEXT_BG))
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
            timestamp=stamp, position=Point2(x=8.0, y=28.0 + 28.0 * k), text=str(text),
            font_size=LABEL_PX + 4, text_color=_color(TARGET[0] if k else FLOW[0]),
            background_color=_TEXT_BG))
    return out


# The demand in the 3D scene: one arrow from the body origin, in the body
# frame the hull hangs on, its length the demand's share of full force times
# DEMAND_ARROW_M (a full demand reaches about the 0.7 m hull's length).
DEMAND_TOPIC = '/mongla/batighor/scene'
DEMAND_ARROW_M = 0.6
# 14 px read about 7 px tall in the cockpit's 3D tile (desktop app, 2026-11-21).
DEMAND_LABEL_PX = 22.0


def demand_scene(x, y, stamp, frame):
    """`foxglove_msgs/SceneUpdate` for a (surge, sway) demand in [-1, 1].

    The arrow points where the board is told to push, in FRD: rotating x
    toward y is a turn about +z (down). NaN, which the manager publishes
    when it does not know the demand, is said in words with no arrow; zero
    is a known demand of nothing and also draws none, but says so.
    """
    import math

    from builtin_interfaces.msg import Duration
    from foxglove_msgs.msg import (ArrowPrimitive, SceneEntity, SceneEntityDeletion,
                                   SceneUpdate, TextPrimitive)
    from geometry_msgs.msg import Point, Pose, Quaternion
    update = SceneUpdate()
    update.deletions.append(SceneEntityDeletion(
        timestamp=stamp, type=SceneEntityDeletion.MATCHING_ID, id='demand'))
    e = SceneEntity(timestamp=stamp, frame_id=frame, id='demand',
                    lifetime=Duration(sec=1), frame_locked=True)
    known = not (math.isnan(x) or math.isnan(y))
    size = math.hypot(x, y) if known else 0.0
    if size > 1e-3:
        yaw = math.atan2(y, x)
        length = DEMAND_ARROW_M * min(size, 1.0)
        e.arrows.append(ArrowPrimitive(
            pose=Pose(orientation=Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))),
            shaft_length=0.75 * length, head_length=0.25 * length,
            shaft_diameter=0.025, head_diameter=0.07, color=_color(SEA)))
    words = (f'demand {size:.0%}' if size > 1e-3 else 'demand none') if known \
        else 'demand unknown'
    e.texts.append(TextPrimitive(
        pose=Pose(position=Point(z=-0.25)), billboard=True, font_size=DEMAND_LABEL_PX,
        scale_invariant=True, color=_color(SEA if known else INK_2), text=words))
    update.entities.append(e)
    return update


# Mongla's track in the 3D scene: the odometry's last TRACK_S, a point each
# TRACK_STEP_M moved, so a vehicle at rest adds nothing and a long run does
# not grow the line without end.
TRACK_S, TRACK_STEP_M = 180.0, 0.05


class Track:
    """Where the vehicle has been, in one frame: a new frame starts again."""

    def __init__(self):
        self.frame, self.points = None, []

    def note(self, t, x, y, z, frame) -> bool:
        """Keep (t, x, y, z) if it moved; True when the line changed."""
        if frame != self.frame:
            self.frame, self.points = frame, []
        old = len(self.points)
        self.points = [p for p in self.points if t - p[0] <= TRACK_S]
        last = self.points[-1] if self.points else None
        if last is None or (x - last[1]) ** 2 + (y - last[2]) ** 2 + (z - last[3]) ** 2 >= TRACK_STEP_M ** 2:
            self.points.append((t, x, y, z))
        return len(self.points) != old


def track_scene(track: Track, stamp):
    """The track as one line in its own frame, sea blue as everything live is."""
    from foxglove_msgs.msg import LinePrimitive, SceneEntity, SceneUpdate
    from geometry_msgs.msg import Point
    e = SceneEntity(timestamp=stamp, frame_id=track.frame or '', id='track', frame_locked=True)
    if len(track.points) >= 2:
        e.lines.append(LinePrimitive(
            type=LinePrimitive.LINE_STRIP, thickness=2.0, scale_invariant=True, color=_color(SEA),
            points=[Point(x=float(x), y=float(y), z=float(z)) for _, x, y, z in track.points]))
    return SceneUpdate(entities=[e])


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


# Every stage a camera's picture goes through, the topic it publishes (under
# /mongla/vision/<cam>/; a name with a suffix counts), the bringup argument
# that turns it on, and the cameras it runs on (None: both). Published as
# `/mongla/batighor/stages` for the Vision panel: on means a node publishes
# the topic now, which the browser cannot tell, because the graph also lists
# a topic that only has subscribers.
STAGES = (
    ('camera', 'stream/compressed', 'vision:=true', None),
    ('detector', 'detections', 'vision:=true', None),
    ('below floor', 'candidates', 'vision:=true', None),
    ('tracker', 'tracks', 'tracking:=true', None),
    ('lock ladder', 'lock', 'lock:=true', None),
    # XFeat publishes once the lock has learned a view of the target.
    ('anchor (XFeat)', 'correspondences', 'lock:=true', None),
    ('contours', 'contours', 'vision:=true', None),
    # pnp_node is gated on `lock` (vision_pi.launch.py: solver()).
    ('pose (PnP)', 'target_pose', 'lock:=true', None),
    ('flow', 'velocity', 'flow:=true', ('downward',)),
    ('overlay', 'annotations', 'batighor:=true', None),
)
STAGES_PERIOD_S = 1.0


def stages(cam, names, publishers) -> list:
    """Each stage of `cam`: on while some node publishes its topic.

    `names`: every topic name in the graph; `publishers(name)`: how many
    nodes publish it."""
    ns = f'/mongla/vision/{cam}/'
    out = []
    for label, rel, switch, cams in STAGES:
        if cams and cam not in cams:
            continue
        topics = [n for n in names if n == ns + rel or n.startswith(ns + rel + '_')]
        on = [t for t in topics if publishers(t) > 0]
        out.append({'stage': label, 'on': bool(on), 'topic': on[0] if on else ns + rel,
                    'switch': switch})
    return out


# The detector's rate over its last this-many answers (~1.5 s at 20 Hz).
RATE_WINDOW = 30

# How soon a new viewer's annotations start, and the cost of asking: one
# subscriber count per published layer, twice a second.
WATCH_PERIOD_S = 0.5


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_parameter_events
    from rcl_interfaces.msg import ParameterEvent
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import CameraInfo
    from std_msgs.msg import String
    from vision_msgs.msg import Detection2DArray, VisionInfo

    from mongla_interfaces.msg import FloorRead, FlowTracks, TargetContours, TargetCorrespondences
    from mongla_vision import qos

    LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    LAYERS = ('detections', 'tracks', 'lock', 'candidates', 'contours', 'features', 'size',
              'model', 'latency')

    class Overlay(Node):
        def __init__(self):
            super().__init__('batighor_overlay')
            cams = list(self.declare_parameter(
                'cameras', ['forward', 'downward']).value or [])
            self.watched = []
            # Each detector's own `paused`, by node: a silent detector that is
            # paused waits for a mission, one that is not is an alarm. Learnt
            # from its parameter events, and asked once when first needed.
            self.paused, self.asked, self.whose, self.askers = {}, set(), {}, {}
            self.create_subscription(ParameterEvent, '/parameter_events', self._on_params,
                                     qos_profile_parameter_events)
            for cam in cams:
                ns = f'/mongla/vision/{cam}'
                latest = dict.fromkeys(LAYERS)
                latest['arrivals'] = deque(maxlen=RATE_WINDOW)
                pub = self.create_publisher(ImageAnnotations, f'{ns}/annotations', 5)
                hud_pub = self.create_publisher(String, f'{ns}/hud', 1)
                trails = Trails()

                def say(c, latest=latest, hud_pub=hud_pub):
                    # A changed state goes at once; the same state with new
                    # numbers waits CAPTION_EVERY_S.
                    now = time.monotonic()
                    if (c['state'] == latest.get('said_state')
                            and now - latest.get('said_at', 0.0) < CAPTION_EVERY_S):
                        return
                    latest['said_state'], latest['said_at'] = c['state'], now
                    hud_pub.publish(String(data=json.dumps(c)))

                def draw(latest=latest, pub=pub, cam=cam, trails=trails, say=say):
                    det = latest['detections']
                    if det is None:
                        return
                    stamp = det.header.stamp
                    feats = latest['features']
                    # Boxes from the frame being drawn and no other: a track
                    # or candidate from the frame before sat beside each
                    # detection, offset by the motion between them.
                    say(caption(cam, latest['model'], latest['size'], latest['arrivals'],
                                latest['latency']))
                    now = time.monotonic()
                    trks = latest['tracks'] if same_frame(latest['tracks'], stamp) else None
                    for d in (trks.detections if trks else []):
                        if d.id:
                            b = _box(d)
                            trails.note(d.id, *_centre(b), now, class_colour(b[4]))
                    for d in (latest['lock'].detections if latest['lock'] else [])[:1]:
                        b = _box(d)
                        trails.note('lock', *_centre(b), now, RUNG.get(b[4], LOCK[0]))
                    pub.publish(annotate(
                        stamp, det,
                        latest['tracks'] if same_frame(latest['tracks'], stamp) else None,
                        latest['lock'],
                        latest['candidates'] if same_frame(latest['candidates'], stamp) else None,
                        latest['contours'] if same_frame(latest['contours'], stamp) else None,
                        feats if fresh(feats, stamp, FEATURES_MAX_AGE_S) else None,
                        latest['size'], trails=trails.lines(now)))

                def on_detections(msg, latest=latest, draw=draw):
                    latest['detections'] = msg
                    latest['arrivals'].append(time.monotonic())
                    now = self.get_clock().now().nanoseconds * 1e-9
                    latest['latency'] = now - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
                    draw()

                def on_model(msg, latest=latest):
                    latest['model'] = msg.database_location or None

                def on_same_frame(key, latest=latest, draw=draw):
                    # The tracker answers after the detector, on its stamp:
                    # the frame is drawn again with its tracks.
                    def cb(msg):
                        latest[key] = msg
                        det = latest['detections']
                        if det is not None and same_frame(msg, det.header.stamp):
                            draw()
                    return cb

                def on_info(msg, latest=latest, pub=pub, cam=cam, say=say):
                    latest['size'] = (msg.width, msg.height) if msg.width else None
                    # The camera's own beat, one per frame on the frame's
                    # stamp: while the detector is silent the overlay marks
                    # the frame itself, so the Vision view (synchronised)
                    # still shows the picture, and says why it has no boxes.
                    now = time.monotonic()
                    if latest.get('since') is None:
                        latest['since'] = now
                    quiet = silence(latest['arrivals'], latest['since'], now)
                    if quiet is not None:
                        paused = self._paused(f'/mongla/vision/{cam}/detections')
                        pub.publish(annotate(msg.header.stamp, size=latest['size']))
                        say(caption(cam, latest['model'], latest['size'], (), None,
                                    silent_s=quiet, paused=paused))

                def keep(key, latest=latest):
                    return lambda msg: latest.__setitem__(key, msg)

                # The detector paces the overlay: one set of boxes per frame
                # it saw. The other layers arrive on their own clocks and are
                # drawn as they last were. The anchor's matched features are
                # in the matcher's pixels; the camera's size scales them.
                self.watched.append(Watched(self, pub, [
                    (Detection2DArray, f'{ns}/detections', on_detections, qos.DETECTIONS),
                    (Detection2DArray, f'{ns}/tracks', on_same_frame('tracks'), qos.DETECTIONS),
                    (Detection2DArray, f'{ns}/lock', keep('lock'), qos.DETECTIONS),
                    (Detection2DArray, f'{ns}/candidates', on_same_frame('candidates'),
                     qos.DETECTIONS),
                    (TargetContours, f'{ns}/contours', on_same_frame('contours'), qos.DETECTIONS),
                    (TargetCorrespondences, f'{ns}/correspondences', keep('features'),
                     qos.DETECTIONS),
                    (CameraInfo, f'{ns}/camera_info', on_info, qos.CAMERA_INFO),
                    (VisionInfo, f'{ns}/vision_info', on_model, qos.LATCHED),
                ], on_drop=lambda latest=latest: (latest.update(dict.fromkeys(LAYERS), since=None),
                                                  latest.update(arrivals=deque(maxlen=RATE_WINDOW)))))

                floor_pub = self.create_publisher(
                    ImageAnnotations, f'{ns}/floor_annotations', 5)
                self.watched.append(Watched(self, floor_pub, [
                    (FloorRead, f'{ns}/floor_read',
                     lambda msg, p=floor_pub: p.publish(floor_annotate(msg)), 5)]))

                flow_pub = self.create_publisher(
                    ImageAnnotations, f'{ns}/flow_annotations', 5)
                self.watched.append(Watched(self, flow_pub, [
                    (FlowTracks, f'{ns}/flow_tracks',
                     lambda msg, p=flow_pub, latest=latest:
                     p.publish(flow_annotate(msg, latest['size'])), 5),
                    # Its own reader of the size: with only the Flow view
                    # open, the camera's layers above are unwatched, the size
                    # stayed unknown, and the flow line sat on the floor
                    # read's caption at the top (seen 2026-11-21).
                    (CameraInfo, f'{ns}/camera_info', on_info, qos.CAMERA_INFO)]))

            # A calibration, when one runs (`calibrate.launch.py`): its state
            # is latched, so a new viewer gets the current one at once.
            cal = '/mongla/vision/calibration'
            cal_pub = self.create_publisher(ImageAnnotations, f'{cal}/annotations', 5)

            def on_calibration(msg):
                cal_pub.publish(calibrate_annotate(
                    json.loads(msg.data), self.get_clock().now().to_msg()))

            self.watched.append(Watched(self, cal_pub, [
                (String, f'{cal}/state', on_calibration, LATCHED)]))
            # The demand, for the 3D scene, at the manager's 20 Hz.
            from foxglove_msgs.msg import SceneUpdate
            from geometry_msgs.msg import Vector3Stamped
            scene_pub = self.create_publisher(SceneUpdate, DEMAND_TOPIC, 5)

            def on_demand(msg):
                scene_pub.publish(demand_scene(msg.vector.x, msg.vector.y,
                                               msg.header.stamp, msg.header.frame_id))

            from nav_msgs.msg import Odometry
            track = Track()

            def on_odom(msg):
                p = msg.pose.pose.position
                st = msg.header.stamp
                if track.note(st.sec + st.nanosec * 1e-9, p.x, p.y, p.z, msg.header.frame_id):
                    scene_pub.publish(track_scene(track, st))

            self.watched.append(Watched(self, scene_pub, [
                (Vector3Stamped, '/mongla/demand', on_demand, 5),
                (Odometry, '/mongla/odom', on_odom, 10)]))
            self.create_timer(WATCH_PERIOD_S, self._poll)
            self.cams = cams
            self.stages_pub = self.create_publisher(String, '/mongla/batighor/stages', LATCHED)
            self.create_timer(STAGES_PERIOD_S, self._stages)

        def _on_params(self, ev):
            for p in (*ev.new_parameters, *ev.changed_parameters):
                if p.name == 'paused':
                    self.paused[ev.node] = p.value.bool_value

        def _paused(self, topic):
            """The `paused` of the node publishing `topic`: True, False, None
            (nobody publishes it) or UNREAD while its answer is on its way."""
            now = time.monotonic()
            at, node = self.whose.get(topic, (0.0, None))
            if now - at > 1.0:                # the graph, once a second at most
                infos = self.get_publishers_info_by_topic(topic)
                node = None if not infos else \
                    f"{infos[0].node_namespace.rstrip('/')}/{infos[0].node_name}"
                self.whose[topic] = (now, node)
            if node is None:
                return None
            if node in self.paused:
                return self.paused[node]
            if node not in self.asked:
                cli = self.askers.get(node) or self.askers.setdefault(
                    node, self.create_client(GetParameters, f'{node}/get_parameters'))
                if cli.service_is_ready():
                    self.asked.add(node)

                    def answered(fut, node=node):
                        res = fut.result()
                        if res is not None and res.values:
                            self.paused.setdefault(node, res.values[0].bool_value)
                        else:
                            self.asked.discard(node)      # ask again next frame
                    cli.call_async(GetParameters.Request(names=['paused'])).add_done_callback(answered)
            return UNREAD

        def _stages(self):
            names = [n for n, _ in self.get_topic_names_and_types()]
            table = {c: stages(c, names, self.count_publishers) for c in self.cams}
            self.stages_pub.publish(String(data=json.dumps(table)))

        def _poll(self):
            for w in self.watched:
                w.poll()

    from mongla_runtime.params_file import refuse_unknown
    from mongla_runtime.spin import spin
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

"""The course on Batighor's map: the deck's file, as the vehicle would read it.

The Map instrument draws the pool a run is planned in: its outline, the props
where the course file puts them, the floor soundings, and the vehicle live on
it. This node reads the course with the vehicle's own loader
(`mongla_localization.course_map.load_course`), never a second parser, and
publishes it on `TOPIC` (`std_msgs/String`, JSON, latched).

Which course: the one bringup was started on (`course`, the deck's
`bringup.yaml`), until a mission names its own with `use_course`, which it
publishes on `/mongla/course`; a map of the wrong pool is worse than none. The
file is read again whenever it changes, so a prop surveyed with
`course_survey` at the pool side appears on every operator's map within a
second, with no restart.

What the file does not say is not drawn as if it did. A prop with no position
is listed as unplaced, never put at the origin; one that is a rulebook nominal
says so (`measured: false`); and a course that does not load is published as
its refusal, which the map shows instead of a pool.
"""
from __future__ import annotations

import json
import os
import time

TOPIC = '/mongla/batighor/course'
MISSION_COURSE = '/mongla/course'          # dsl_heading.COURSE_TOPIC
PERIOD_S = 1.0
# The same pool for the 3D scene (foxglove SceneUpdate, latched), in the
# `pool` frame (NED, frames.POOL), which /tf hangs under `map`: until the
# filter is anchored `map` and `odom` coincide, so the pool is drawn by the
# vehicle's start, as the Map draws it.
SCENE_TOPIC = '/mongla/batighor/pool_scene'
# The pool once the filter is anchored (`map -> odom` exists); before that,
# odometry's own NED frame, which "by its start" takes for the pool, as the
# instruments' map does. Drawn in `pool` alone, the scene was in a tree with
# no hull and the 3D view showed nothing (bench, 2026-11-24).
from mongla_localization import frames as _frames  # noqa: E402

POOL_FRAME = _frames.POOL
START_FRAME = _frames.ODOM
# A prop whose size the file does not give is drawn this big, and says so.
PROP_M = 0.3
# Floor tile lines past this many a side are not drawn: a 50 m pool at 0.3 m
# tiles is 160 lines a side, which hides the props it is there to place.
MAX_GRID_LINES = 60

# The site's palette, as RGBA: measured red, nominal amber, the pool in glass.
_RED, _AMBER = (1.0, 0.165, 0.078, 0.9), (0.973, 0.529, 0.004, 0.45)
_GLASS, _DIM = (0.769, 0.886, 0.953, 0.8), (0.769, 0.886, 0.953, 0.18)
_ZONE = (0.973, 0.529, 0.004, 0.55)


def pool_scene(doc: dict, stamp=None, frame: str = POOL_FRAME):
    """The course as a 3D scene: the pool's surface and floor outlines with
    their corners joined, the floor's tile grid, and each placed prop as a
    block at its depth, red when measured and translucent amber when a
    rulebook nominal, with its name. A prop with no depth is a post from the
    surface to the floor, said so; an outline or depth the file does not give
    is not drawn. A refused course clears the scene and says why."""
    import math

    from foxglove_msgs.msg import (Color, CubePrimitive, LinePrimitive, SceneEntity,
                                   SceneEntityDeletion, SceneUpdate, TextPrimitive)
    from geometry_msgs.msg import Point, Pose, Quaternion, Vector3

    def col(c):
        return Color(r=c[0], g=c[1], b=c[2], a=c[3])

    def pt(x, y, z=0.0):
        return Point(x=float(x), y=float(y), z=float(z))

    def text(x, y, z, words, c, px=14.0):
        return TextPrimitive(pose=Pose(position=pt(x, y, z)), billboard=True, font_size=px,
                             scale_invariant=True, color=col(c), text=words)

    out = SceneUpdate()
    if stamp is not None:
        out.deletions.append(SceneEntityDeletion(timestamp=stamp, type=SceneEntityDeletion.ALL))
    ent = SceneEntity(frame_id=frame, id='pool', frame_locked=True)
    if stamp is not None:
        ent.timestamp = stamp
    if frame != POOL_FRAME:
        ent.texts.append(text(0, 0, -0.4, 'not anchored: the pool drawn by its start', _AMBER, 12.0))
    if 'error' in doc:
        ent.texts.append(text(0, 0, 0, f'no pool: {doc["error"]}', _AMBER))
        out.entities.append(ent)
        return out
    pool = doc.get('pool') or {}
    # The floor: the venue's depth, else its deepest sounding; neither, none.
    soundings = [f[2] for f in pool.get('floor') or [] if len(f) > 2]
    depth = pool.get('depth_m') or (max(soundings) if soundings else None)
    ring = [list(c)[:2] for c in pool.get('outline') or []]
    if len(ring) >= 2:
        loop = ring + [ring[0]] if len(ring) > 2 else ring
        ent.lines.append(LinePrimitive(type=LinePrimitive.LINE_STRIP, thickness=2.0, scale_invariant=True,
                                       points=[pt(x, y) for x, y in loop], color=col(_GLASS)))
        if depth:
            ent.lines.append(LinePrimitive(type=LinePrimitive.LINE_STRIP, thickness=1.5, scale_invariant=True,
                                           points=[pt(x, y, depth) for x, y in loop], color=col(_DIM)))
            ent.lines.append(LinePrimitive(type=LinePrimitive.LINE_LIST, thickness=1.0, scale_invariant=True,
                                           points=[p for x, y in ring for p in (pt(x, y), pt(x, y, depth))],
                                           color=col(_DIM)))
            tile = pool.get('tile_m')
            xs, ys = [c[0] for c in ring], [c[1] for c in ring]
            if tile and tile > 0:
                grid = []
                for lo, hi, other, along in ((min(xs), max(xs), (min(ys), max(ys)), 'x'),
                                             (min(ys), max(ys), (min(xs), max(xs)), 'y')):
                    n = int((hi - lo) / tile)
                    step = tile * max(1, math.ceil(n / MAX_GRID_LINES))
                    k = math.ceil(lo / step) * step
                    while k <= hi + 1e-9:
                        a, b = (pt(k, other[0], depth), pt(k, other[1], depth)) if along == 'x' \
                            else (pt(other[0], k, depth), pt(other[1], k, depth))
                        grid += [a, b]
                        k += step
                ent.lines.append(LinePrimitive(type=LinePrimitive.LINE_LIST, thickness=0.5,
                                               scale_invariant=True, points=grid, color=col(_DIM)))
    # The rulebook's zones on the surface, amber and thin: where a prop may
    # be, apart from where one is.
    for z in doc.get('zones') or []:
        (x0, y0), (x1, y1) = z['points']
        pts = [pt(x0, y0), pt(x1, y1)] if z['kind'] == 'line' else \
            [pt(x0, y0), pt(x1, y0), pt(x1, y1), pt(x0, y1), pt(x0, y0)]
        ent.lines.append(LinePrimitive(type=LinePrimitive.LINE_STRIP, thickness=1.5, scale_invariant=True,
                                       points=pts, color=col(_ZONE)))
        words = z['name'].replace('_', ' ') + ('' if z['source'] == 'rulebook' else ' (figure)')
        ent.texts.append(text((x0 + x1) / 2, (y0 + y1) / 2, 0.0, words, _ZONE, 11.0))
    for prop in doc.get('props') or []:
        x, y, c = prop['x_m'], prop['y_m'], _RED if prop.get('measured') else _AMBER
        # A prop's depth_m is NEGATIVE below the surface (course_map, this
        # stack's altitude sign); the pool frame is NED, z down. The pool's
        # own depth_m is positive.
        z = -prop['depth_m'] if prop.get('depth_m') is not None else None
        name = prop['name'] + ('' if prop.get('measured') else ' (nominal)')
        if z is None:
            ent.lines.append(LinePrimitive(type=LinePrimitive.LINE_LIST, thickness=3.0, scale_invariant=True,
                                           points=[pt(x, y), pt(x, y, depth or 1.0)], color=col(c)))
            ent.texts.append(text(x, y, -0.2, f'{name}, depth not set', c))
            continue
        w = prop.get('width_m') or PROP_M
        yaw = math.radians(prop.get('bearing_deg') or 0.0)
        ent.cubes.append(CubePrimitive(
            pose=Pose(position=pt(x, y, z), orientation=Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))),
            size=Vector3(x=0.12, y=float(w), z=PROP_M), color=col(c)))
        ent.texts.append(text(x, y, z - PROP_M, name, c))
    out.entities.append(ent)
    return out


def describe(course) -> dict:
    """A loaded `Course` as the map draws it: pool, placed props, unplaced ones."""
    pool = course.pool
    placed, unplaced = [], []
    for name, p in sorted(course.props.items()):
        row = {'name': name, 'class': p.detect_class, 'measured': p.measured,
               'note': p.note, 'depth_m': p.depth_m, 'bearing_deg': p.bearing_deg,
               'width_m': p.width_m}
        if p.has_position:
            placed.append({**row, 'x_m': p.x_m, 'y_m': p.y_m})
        else:
            unplaced.append(row)
    return {
        'course': course.name, 'source': course.source,
        'pool': {'outline': [list(c) for c in course.pool_outline()],
                 'floor': [list(f) for f in course.pool_floor()],
                 'depth_m': pool.get('depth_m'), 'tile_m': pool.get('tile_m'),
                 'shield_max_depth_m': pool.get('shield_max_depth_m'),
                 'lane_lines': pool.get('lane_lines')},
        'props': placed, 'unplaced': unplaced,
        # Where the rulebook says a prop may be: drawn, never steered by.
        'zones': [{'name': n, **z} for n, z in sorted(course.zones.items())],
    }


def read(name: str) -> tuple:
    """(document, the file it came from or ''): the course, or why there is none."""
    if not name:
        return ({'course': '', 'error': 'no course: set `course:` in the deck\'s '
                 'bringup.yaml, or run a mission that calls use_course()'}, '')
    from mongla_localization.course_map import load_course
    try:
        c = load_course(name)
    except Exception as exc:                 # noqa: BLE001
        # Anything a hand-edited file can raise (a YAML syntax error, `props:`
        # written as a list) is the refusal this topic promises, not a crash
        # that leaves the map on the last good pool.
        return {'course': name, 'error': f'{type(exc).__name__}: {exc}'}, ''
    return describe(c), c.source


def CourseNode():
    """The node: the course on `TOPIC`, again whenever the name or the file changes."""
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    class _Course(Node):
        def __init__(self):
            super().__init__('batighor_course')
            self.declare_parameter('course', '')
            self.name = str(self.get_parameter('course').value or '').strip()
            self.mission = ''
            self.seen = None                    # (name, file, mtime) last published
            self.pub = self.create_publisher(String, TOPIC, latched)
            from foxglove_msgs.msg import SceneUpdate
            self.scene_pub = self.create_publisher(SceneUpdate, SCENE_TOPIC, latched)
            self.create_subscription(String, MISSION_COURSE, self._on_mission, latched)
            from tf2_ros import Buffer, TransformListener
            self.tf = Buffer()
            self._tf_listener = TransformListener(self.tf, self)
            self.create_timer(PERIOD_S, self._tick)
            self._tick()

        def _on_mission(self, msg):
            self.mission = msg.data.strip()
            self._tick()

        def _anchored(self) -> bool:
            from rclpy.time import Time
            try:
                return self.tf.can_transform(_frames.MAP_ENU, _frames.ODOM_ENU, Time())
            except Exception:              # noqa: BLE001 -- no tree yet is not anchored
                return False

        def _tick(self):
            name = self.mission or self.name
            doc, path = read(name)
            try:
                mtime = os.stat(path).st_mtime if path else None
            except OSError:
                mtime = None
            frame = POOL_FRAME if self._anchored() else START_FRAME
            key = (name, path, mtime, doc.get('error'), frame)
            if key == self.seen:
                return
            self.seen = key
            doc['from'] = 'mission' if self.mission else 'deck'
            doc['stamp'] = time.time()
            self.pub.publish(String(data=json.dumps(doc)))
            self.scene_pub.publish(pool_scene(doc, self.get_clock().now().to_msg(), frame))
            self.get_logger().info(
                f'[CRSE ] {name or "(none)"}: '
                + (f'refused: {doc["error"]}' if 'error' in doc else
                   f'{len(doc["props"])} props placed, {len(doc["unplaced"])} unplaced '
                   f'({path})'))

    return _Course()


def main(args=None):
    import rclpy
    from mongla_runtime.params_file import refuse_unknown
    from mongla_runtime.spin import spin
    rclpy.init(args=args)
    node = CourseNode()
    refuse_unknown(node)
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

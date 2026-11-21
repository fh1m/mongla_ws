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
    }


def read(name: str) -> tuple:
    """(document, the file it came from or ''): the course, or why there is none."""
    if not name:
        return ({'course': '', 'error': 'no course: set `course:` in the deck\'s '
                 'bringup.yaml, or run a mission that calls use_course()'}, '')
    from mongla_localization.course_map import load_course
    try:
        c = load_course(name)
    except (FileNotFoundError, ValueError, KeyError, OSError) as exc:
        return {'course': name, 'error': str(exc)}, ''
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
            self.create_subscription(String, MISSION_COURSE, self._on_mission, latched)
            self.create_timer(PERIOD_S, self._tick)
            self._tick()

        def _on_mission(self, msg):
            self.mission = msg.data.strip()
            self._tick()

        def _tick(self):
            name = self.mission or self.name
            doc, path = read(name)
            try:
                mtime = os.stat(path).st_mtime if path else None
            except OSError:
                mtime = None
            key = (name, path, mtime, doc.get('error'))
            if key == self.seen:
                return
            self.seen = key
            doc['from'] = 'mission' if self.mission else 'deck'
            doc['stamp'] = time.time()
            self.pub.publish(String(data=json.dumps(doc)))
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

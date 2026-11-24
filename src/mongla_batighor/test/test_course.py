"""The course on Batighor's map (course.py): the vehicle's own loader, nothing invented."""
import json
import time

import pytest

from mongla_batighor import course
from mongla_localization import course_map as cm
from mongla_localization.course_survey import main as survey


def test_an_unsurveyed_course_places_nothing_and_lists_every_prop(tmp_path, monkeypatch):
    monkeypatch.setenv(cm._ENV, str(tmp_path))
    doc, src = course.read('sauvc26')
    assert src.endswith('sauvc26.yaml') and 'error' not in doc
    assert doc['props'] == [], 'a prop with no position was placed somewhere'
    assert {p['name'] for p in doc['unplaced']} == set(cm.load_course('sauvc26').props)
    assert doc['pool']['outline'] == [] and doc['pool']['floor'] == []


def test_a_surveyed_prop_and_outline_are_drawn_where_the_file_says(tmp_path, monkeypatch):
    monkeypatch.setenv(cm._ENV, str(tmp_path))
    assert survey(['--course', 'sauvc26', '--prop', 'final_gate', '--x', '16', '--y', '-1',
                   '--bearing', '0', '--width', '1.5']) == 0
    assert survey(['--course', 'sauvc26', '--corner', '0', '-8', '--corner', '0', '8',
                   '--corner', '25', '8', '--floor', '3', '0', '1.4']) == 0
    doc, _ = course.read('sauvc26')
    gate = next(p for p in doc['props'] if p['name'] == 'final_gate')
    assert (gate['x_m'], gate['y_m'], gate['bearing_deg'], gate['width_m'], gate['measured']) \
        == (16.0, -1.0, 0.0, 1.5, True)
    assert doc['pool']['outline'] == [[0, -8], [0, 8], [25, 8]]
    assert doc['pool']['floor'] == [[3, 0, 1.4]]
    json.dumps(doc)                                 # what the node publishes


def test_a_missing_or_broken_course_is_published_as_its_refusal(tmp_path, monkeypatch):
    monkeypatch.setenv(cm._ENV, str(tmp_path))
    assert 'deck' in course.read('')[0]['error']
    assert 'no course' in course.read('nowhere_pool')[0]['error']
    (tmp_path / 'broken.yaml').write_text('course: broken\npool:\n  outline: [[0, 0]]\n')
    assert 'outline' in course.read('broken')[0]['error']


def test_the_node_follows_a_mission_course_and_an_edited_file(tmp_path, monkeypatch):
    """The deck's course first; a mission's /mongla/course over it; a survey seen live."""
    rclpy = pytest.importorskip('rclpy')
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    monkeypatch.setenv(cm._ENV, str(tmp_path))
    rclpy.init()
    try:
        from rclpy.parameter import Parameter
        node = course.CourseNode()
        node.set_parameters([Parameter('course', value='pool')])
        node.name = 'pool'
        node.seen = None
        node._tick()
        probe = rclpy.create_node('course_probe')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        got = []
        probe.create_subscription(String, course.TOPIC, lambda m: got.append(json.loads(m.data)),
                                  latched)
        mission = probe.create_publisher(String, course.MISSION_COURSE, latched)

        def spin_until(pred):
            t = time.monotonic()
            while time.monotonic() - t < 3.0 and not (got and pred(got[-1])):
                rclpy.spin_once(node, timeout_sec=0.02)
                rclpy.spin_once(probe, timeout_sec=0.02)
            return got[-1] if got else None

        assert spin_until(lambda d: d['course'] == 'pool')['from'] == 'deck'
        mission.publish(String(data='sauvc26'))
        assert spin_until(lambda d: d['course'] == 'sauvc26')['from'] == 'mission'
        survey(['--course', 'sauvc26', '--prop', 'final_gate', '--x', '16', '--y', '0'])
        time.sleep(1.1)                             # a new mtime
        node._tick()
        d = spin_until(lambda d: any(p['name'] == 'final_gate' for p in d.get('props', [])))
        assert d and d['course'] == 'sauvc26', 'a surveyed prop never reached the map'
        node.destroy_node()
        probe.destroy_node()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize('body', ['pool: {depth_m: 2\n\tbad: 1}\n', 'props: [a, b]\n'])
def test_a_broken_course_file_is_refused_not_a_crash(tmp_path, monkeypatch, body):
    # YAMLError and a list-shaped `props:` escaped read() and killed the node,
    # leaving the map on the last good pool (audit 2026-11-24).
    monkeypatch.setenv(cm._ENV, str(tmp_path))
    (tmp_path / 'broken.yaml').write_text(body)
    doc, src = course.read('broken')
    assert src == '' and doc['error']


def test_the_pool_is_drawn_in_3d_where_the_file_puts_it(tmp_path, monkeypatch):
    """The 3D scene draws the course the Map draws: a surveyed prop where the
    file puts it, red; the outline at the surface and on the floor; nothing
    the file does not give."""
    monkeypatch.setenv(cm._ENV, str(tmp_path))
    survey(['--course', 'sauvc26', '--prop', 'final_gate', '--x', '16', '--y', '-1',
            '--bearing', '0', '--width', '1.5'])
    survey(['--course', 'sauvc26', '--corner', '0', '-8', '--corner', '0', '8',
            '--corner', '25', '8', '--corner', '25', '-8'])
    doc, _ = course.read('sauvc26')
    (e,) = course.pool_scene(doc).entities
    assert e.frame_id == course.POOL_FRAME == 'pool'
    surface = e.lines[0]
    assert [(p.x, p.y, p.z) for p in surface.points][0] == (0, -8, 0) and len(surface.points) == 5
    gate = next(t for t in e.texts if t.text.startswith('final_gate'))
    # The survey gave no depth: a post, said so, never a block at a made-up depth.
    assert doc['props'][0]['depth_m'] is None
    assert 'depth not set' in gate.text and not e.cubes
    # A refused course clears the scene and says why.
    (bad,) = course.pool_scene({'error': 'no course here'}).entities
    assert not bad.lines and not bad.cubes and 'no course here' in bad.texts[0].text


def test_a_nominal_prop_is_drawn_apart_from_a_measured_one():
    # A prop's depth is negative below the surface (course_map); the pool's is
    # positive. NED down is +z, so both props sit under the surface.
    doc = {'pool': {'outline': [], 'depth_m': 2.0},
           'props': [{'name': 'gate', 'x_m': 5, 'y_m': 0, 'depth_m': -1.0, 'measured': False},
                     {'name': 'drum', 'x_m': 9, 'y_m': 2, 'depth_m': -2.0, 'measured': True}]}
    (e,) = course.pool_scene(doc).entities
    nominal, measured = e.cubes
    assert (nominal.pose.position.z, measured.pose.position.z) == (1.0, 2.0)
    assert nominal.color.a < 0.5 < measured.color.a
    assert [t.text for t in e.texts] == ['gate (nominal)', 'drum']
    assert nominal.size.y == course.PROP_M        # no width given: the default, drawn as such


def test_a_pool_known_only_by_soundings_still_has_a_floor():
    doc = {'pool': {'outline': [[0, 0], [10, 0], [10, 5]], 'floor': [[2, 2, 1.2], [8, 3, 1.6]],
                    'depth_m': None},
           'props': []}
    (e,) = course.pool_scene(doc).entities
    floor = e.lines[1]
    assert {p.z for p in floor.points} == {1.6}


def test_the_rulebook_zones_are_drawn_and_say_their_source():
    doc, _ = course.read('sauvc_olympic')
    names = {z['name'] for z in doc['zones']}
    assert {'gate_line', 'orange_flare_zone'} <= names and doc['props'] == []
    (e,) = course.pool_scene(doc).entities
    words = [t.text for t in e.texts]
    assert 'gate line' in words and 'target mat (figure)' in words
    gate = next(z for z in doc['zones'] if z['name'] == 'gate_line')
    assert any(len(ln.points) == 2 and ln.points[0].x == gate['points'][0][0] for ln in e.lines)


def test_before_the_anchor_the_pool_is_drawn_by_its_start_and_says_so():
    """`pool` hangs off `map`, and `map -> odom` exists only once anchored:
    drawn in `pool` before then, the scene was in a tree with no hull and
    the 3D view showed nothing. By its start it is odometry's NED frame."""
    doc, _ = course.read('sauvc_olympic')
    (before,) = course.pool_scene(doc, frame=course.START_FRAME).entities
    (after,) = course.pool_scene(doc).entities
    assert before.frame_id == 'odom_ned' and after.frame_id == 'pool'
    assert any('not anchored' in t.text for t in before.texts)
    assert not any('not anchored' in t.text for t in after.texts)

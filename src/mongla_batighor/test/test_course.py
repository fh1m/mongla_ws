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

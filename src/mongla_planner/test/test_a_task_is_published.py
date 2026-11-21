"""A mission's tasks reach every operator on /mongla/mission/task (dsl_plan.TASK_TOPIC).

Batighor's Mission instrument shows which task the mission is in, its
deadline and how the last one ended. Through a real node: a publisher that
was never created, or a topic name that drifted, fails here.
"""
import json
import logging
import time
import types

import pytest

rclpy = pytest.importorskip('rclpy')

from mongla_planner.client import TaskAbandoned            # noqa: E402
from mongla_planner.dsl_plan import TASK_TOPIC, PlanMixin   # noqa: E402


@pytest.fixture
def mission():
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('task_topic_test')
    got = []
    node.create_subscription(String, TASK_TOPIC, lambda m: got.append(json.loads(m.data)),
                             QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    m = PlanMixin()
    m.client = types.SimpleNamespace(node=node, _task_deadline=None)
    m.log = logging.getLogger('task_topic_test')

    def spin():
        t = time.monotonic()
        while time.monotonic() - t < 0.5:
            rclpy.spin_once(node, timeout_sec=0.05)
        return got
    yield m, spin
    node.destroy_node()
    rclpy.shutdown()


def test_a_task_publishes_its_start_and_its_end(mission):
    m, spin = mission
    with m.task('gate', deadline_s=60):
        pass
    got = spin()
    assert [(g['task'], g['state']) for g in got] == [('gate', 'running'), ('gate', 'done')]
    assert got[0]['deadline_s'] == 60 and got[1]['elapsed_s'] >= 0


def test_an_abandoned_task_says_so(mission):
    m, spin = mission
    with pytest.raises(TaskAbandoned):
        with m.task('torpedo', deadline_s=1):
            raise TaskAbandoned('deadline')
    assert spin()[-1] == {**spin()[-1], 'task': 'torpedo', 'state': 'abandoned'}


def test_the_course_a_mission_flies_is_published_latched():
    """`use_course` names the course on /mongla/course, so Batighor's map draws
    the pool the mission flies, not the deck's default; a late subscriber gets it."""
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from mongla_planner.dsl_heading import COURSE_TOPIC, HeadingMixin
    rclpy.init()
    try:
        node = rclpy.create_node('course_topic_test')
        m = HeadingMixin()
        m.client = types.SimpleNamespace(node=node)
        m.log = logging.getLogger('course_topic_test')
        m._say_course('sauvc26')
        got = []
        node.create_subscription(String, COURSE_TOPIC, lambda x: got.append(x.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        t = time.monotonic()
        while not got and time.monotonic() - t < 2.0:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert got == ['sauvc26']
        node.destroy_node()
    finally:
        rclpy.shutdown()

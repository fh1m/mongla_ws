"""`mongla.floor_height()`: the floor's own height, or None -- never a stale one."""
import time
from unittest.mock import MagicMock

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

from mongla_planner.mongla_dsl import MonglaMission

TOPIC = '/mongla/vision/downward/floor_height'


def _run(publish_age_s=None, max_age_s=1.0):
    started = not rclpy.ok()
    if started:
        rclpy.init()
    node = Node('floor_height_probe')
    pub = node.create_publisher(Range, TOPIC, 10)
    try:
        m = MagicMock()
        m._floor_h_sub = None
        m.client = type('C', (), {'node': node})()
        MonglaMission.floor_height(m, timeout=0.05)          # subscribe
        if publish_age_s is not None:
            r = Range()
            r.range = 0.83
            for _ in range(20):
                pub.publish(r)
                rclpy.spin_once(node, timeout_sec=0.02)
                if m._floor_h is not None:
                    break
            if m._floor_h is not None:
                m._floor_h = (m._floor_h[0], time.time() - publish_age_s)
        return MonglaMission.floor_height(m, timeout=0.05, max_age_s=max_age_s)
    finally:
        node.destroy_node()
        if started and rclpy.ok():
            rclpy.shutdown()


def test_a_fresh_reading_is_returned():
    assert abs(_run(publish_age_s=0.0) - 0.83) < 1e-6


def test_a_stale_reading_is_None():
    assert _run(publish_age_s=5.0) is None


def test_nothing_published_is_None():
    assert _run() is None

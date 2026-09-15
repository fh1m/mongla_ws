"""`duburi.flare_order()`: this mission's order, or None -- never a practice run's."""
import json
import time
from unittest.mock import MagicMock

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

from duburi_planner.duburi_dsl import DuburiMission

TOPIC = '/duburi/flare_order'


def _run(latched_offset_s=None, rx_age_s=0.0):
    """Publish an order latched `latched_offset_s` relative to mission start."""
    started = not rclpy.ok()
    if started:
        rclpy.init()
    node = Node('flare_order_probe')
    pub = node.create_publisher(
        String, TOPIC, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        m = MagicMock()
        m._flare_sub = None
        m._mission_start_wall = time.time()
        m.client = type('C', (), {'node': node})()
        DuburiMission.flare_order(m)                                   # subscribe
        if latched_offset_s is not None:
            msg = String()
            msg.data = json.dumps({'colours': ['red', 'blue', 'yellow'], 'nonce': 5,
                                   'latched_at': m._mission_start_wall + latched_offset_s})
            for _ in range(40):
                pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.02)
                if m._flare is not None:
                    break
            assert m._flare is not None, 'probe never delivered the order'
            c, lat, rx = m._flare
            m._flare = (c, lat, rx - rx_age_s)
        return DuburiMission.flare_order(m, timeout=0.05)
    finally:
        node.destroy_node()
        if started and rclpy.ok():
            rclpy.shutdown()


def test_an_order_latched_this_mission_is_returned():
    assert _run(latched_offset_s=+5.0) == ('red', 'blue', 'yellow')


def test_an_order_latched_before_this_mission_is_refused():
    assert _run(latched_offset_s=-30.0) is None


def test_an_order_the_manager_stopped_republishing_is_None():
    assert _run(latched_offset_s=+5.0, rx_age_s=10.0) is None


def test_no_order_is_None():
    assert _run() is None

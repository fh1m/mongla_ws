"""In-process integration test for the detected()/where() LIVE pump.

The unit tests in test_vision_dsl.py stub `_pump_detections` -- so they cover
the pure eval logic but NOT the thing the 2026-06 fix actually changed:
subscribe -> spin the node -> cache updates from a real /detections frame.
This test exercises that path for real with rclpy (no sim, no MAVLink): one
node both publishes a Detection2DArray + CameraInfo and runs the MonglaMission
subscriptions, and the pump is LIVE (not stubbed).

If the eager-subscribe + active-pump regresses, `detected()` here goes back to
false-negating and these fail -- which the stubbed unit tests cannot catch.
"""

import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.qos import QoSProfile, QoSReliabilityPolicy        # noqa: E402
from sensor_msgs.msg import CameraInfo                        # noqa: E402
from vision_msgs.msg import (                                 # noqa: E402
    Detection2D, Detection2DArray, ObjectHypothesisWithPose,
)

import mongla_planner.mongla_dsl as dd                        # noqa: E402
# Reuse the production message builders so the test message matches whatever
# vision_msgs layout this distro ships (Humble flat vs Iron+ nested).
from mongla_vision.detection.messages import (                # noqa: E402
    _set_center, _set_hypothesis,
)


_QOS = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)


def _make_array(frame_id, items):
    """items: list of (class, cx, cy, w, h). Build a Detection2DArray."""
    arr = Detection2DArray()
    arr.header.frame_id = frame_id
    for cls, cx, cy, w, h in items:
        det = Detection2D()
        _set_center(det.bbox.center, cx, cy)
        det.bbox.size_x = float(w)
        det.bbox.size_y = float(h)
        hyp = ObjectHypothesisWithPose()
        _set_hypothesis(hyp, cls, 0.9)
        det.results.append(hyp)
        arr.detections.append(det)
    return arr


class _Log:
    def info(self, *_a, **_k): pass
    def warning(self, *_a, **_k): pass
    def warn(self, *_a, **_k): pass
    def error(self, *_a, **_k): pass


@pytest.fixture
def ros_node():
    rclpy.init()
    node = rclpy.create_node('test_detected_live')
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _publish_until(node, pub, msg, predicate, *, timeout=5.0):
    """Republish msg + spin until predicate() is True or timeout. Returns it."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def test_detected_sees_live_published_frame(ros_node):
    # The node both publishes /detections and hosts the MonglaMission subs.
    pub = ros_node.create_publisher(
        Detection2DArray, '/mongla/vision/forward/detections', _QOS)
    client = SimpleNamespace(node=ros_node)
    mongla = dd.MonglaMission(client, _Log(), camera='forward')

    arr = _make_array('forward', [('gate', 320.0, 240.0, 80.0, 80.0)])
    # The LIVE pump must converge: detected('gate') becomes True once a frame
    # lands. Republish to ride past in-process discovery, then assert.
    got = _publish_until(ros_node, pub, arr, lambda: mongla.detected('gate'))
    assert got, "detected('gate') never saw the live published frame"
    # A class that is NOT in the frame stays False (no false-positive).
    assert mongla.detected('hole') is False


def test_where_uses_live_camera_info(ros_node):
    det_pub = ros_node.create_publisher(
        Detection2DArray, '/mongla/vision/forward/detections', _QOS)
    info_pub = ros_node.create_publisher(
        CameraInfo, '/mongla/vision/forward/camera_info', _QOS)
    client = SimpleNamespace(node=ros_node)
    mongla = dd.MonglaMission(client, _Log(), camera='forward')

    info = CameraInfo()
    info.width = 640
    info.height = 480
    # gate well to the LEFT of a 640px frame (cx=64 -> offset -0.8).
    arr = _make_array('forward', [('gate', 64.0, 240.0, 60.0, 60.0)])

    def _ready():
        info_pub.publish(info)
        det_pub.publish(arr)
        rclpy.spin_once(ros_node, timeout_sec=0.05)
        return mongla.where('gate') == 'left'

    deadline = time.monotonic() + 5.0
    ok = False
    while time.monotonic() < deadline:
        if _ready():
            ok = True
            break
    assert ok, "where('gate') did not resolve to 'left' from live frames"
    assert mongla.where_offset('gate') < 0.0   # left of centre


def test_wait_for_returns_true_on_live_frame(ros_node):
    pub = ros_node.create_publisher(
        Detection2DArray, '/mongla/vision/forward/detections', _QOS)
    client = SimpleNamespace(node=ros_node)
    mongla = dd.MonglaMission(client, _Log(), camera='forward')

    arr = _make_array('forward', [('red_pipe', 300.0, 240.0, 50.0, 120.0)])
    # Publish a few frames so the topic is live, then wait_for must succeed.
    for _ in range(5):
        pub.publish(arr)
        rclpy.spin_once(ros_node, timeout_sec=0.02)
    assert mongla.wait_for('red_pipe', timeout=3.0) is True
    # A class never published times out to False.
    assert mongla.wait_for('octagon', timeout=0.3) is False

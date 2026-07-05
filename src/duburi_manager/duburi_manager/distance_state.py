"""DistanceState -- manager-side bridge to the downward distance estimator.

Mirrors VisionState's role: a manager-process ROS resource injected into the
`Duburi` facade (as `distance_provider`) so the control package stays rclpy-free.
The heavy optical-flow integration runs in
`duburi_vision.distance_estimation_node`; this just:
  * subscribes /duburi/vision/<cam>/distance_traveled  (running metres, cached)
  * publishes /duburi/vision/<cam>/distance_control     (LATCHED String:
    'start_axial' | 'start_lateral' | 'stop')

Deliberately NO service round-trip: the manager node is already spinning under
the executor, so a synchronous service call (spin_until_future_complete on self)
is a rclpy footgun. Everything cross-node here is cached-subscription reads +
fire-and-forget latched publishes -- the same pattern VisionState /
_publish_active_camera use.
"""

from __future__ import annotations

import threading
import time

from std_msgs.msg import Float32, String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# After 'stop', wait this long for the node's frozen distance_traveled to land in
# the cache before reading it (the node republishes the frozen total on stop, and
# distance_traveled streams per frame, so this is comfortably enough).
_STOP_SETTLE_S = 0.4


class DistanceState:
    def __init__(self, node, *, camera: str = 'downward'):
        self._node = node
        self._cam  = camera
        ns = f'/duburi/vision/{camera}'

        self._lock = threading.Lock()
        self._distance_m = 0.0

        node.create_subscription(
            Float32, f'{ns}/distance_traveled', self._on_distance, 10)
        # Latched control publisher so a late-joining node still gets the last cmd.
        ctrl_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._ctrl_pub = node.create_publisher(String, f'{ns}/distance_control', ctrl_qos)

    def _on_distance(self, msg: Float32) -> None:
        with self._lock:
            self._distance_m = float(msg.data)

    @property
    def distance_m(self) -> float:
        with self._lock:
            return self._distance_m

    def start(self, *, lateral: bool) -> tuple[bool, str]:
        with self._lock:
            self._distance_m = 0.0
        cmd = 'start_lateral' if lateral else 'start_axial'
        self._ctrl_pub.publish(String(data=cmd))
        return True, cmd

    def stop(self) -> tuple[bool, str, float]:
        self._ctrl_pub.publish(String(data='stop'))
        # Let the node's frozen total land in the cache (no service ack needed).
        time.sleep(_STOP_SETTLE_S)
        return True, 'stop', self.distance_m

"""DistanceState -- manager-side bridge to the downward distance estimator.

Mirrors VisionState's role: a manager-process ROS resource injected into the
`Duburi` facade (as `distance_provider`) so the control package stays rclpy-free.
The heavy optical-flow integration runs in `duburi_vision.distance_estimation_node`;
this just:
  * subscribes /duburi/vision/<cam>/distance_traveled  (running metres, cached)
  * calls the node's SetBool `distance_control` service (true=start, false=stop)
  * sets the node's `distance_lateral` param (SetParameters) so the projection
    axis matches the move currently running (axial vs lateral)

`Duburi.calc_distance('start'/'stop')` calls start()/stop(); stop() returns the
last cached distance (the node freezes it on the stop control call).
"""

from __future__ import annotations

import threading

from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


class DistanceState:
    def __init__(self, node, *, camera: str = 'downward'):
        self._node = node
        self._cam  = camera
        ns = f'/duburi/vision/{camera}'
        self._node_name = 'duburi_distance_estimator'

        self._lock = threading.Lock()
        self._distance_m = 0.0

        node.create_subscription(
            Float32, f'{ns}/distance_traveled', self._on_distance, 10)
        self._ctrl_cli  = node.create_client(SetBool, f'{ns}/distance_control')
        self._param_cli = node.create_client(
            SetParameters, f'/{self._node_name}/set_parameters')

    def _on_distance(self, msg: Float32) -> None:
        with self._lock:
            self._distance_m = float(msg.data)

    @property
    def distance_m(self) -> float:
        with self._lock:
            return self._distance_m

    def _set_lateral(self, lateral: bool, timeout: float = 2.0) -> None:
        """Best-effort: tell the node which axis to project onto before start."""
        if not self._param_cli.wait_for_service(timeout_sec=timeout):
            return
        req = SetParameters.Request(parameters=[Parameter(
            name='distance_lateral',
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                 bool_value=bool(lateral)))])
        self._param_cli.call_async(req)   # fire-and-forget; node reads it at start

    def _control(self, start: bool, timeout: float = 3.0) -> tuple[bool, str]:
        if not self._ctrl_cli.wait_for_service(timeout_sec=timeout):
            return False, 'distance node absent (distance_control service)'
        import rclpy
        req = SetBool.Request(); req.data = bool(start)
        fut = self._ctrl_cli.call_async(req)
        rclpy.spin_until_future_complete(self._node, fut, timeout_sec=timeout + 2.0)
        resp = fut.result()
        if resp is None:
            return False, 'distance_control timed out'
        return bool(resp.success), str(resp.message)

    def start(self, *, lateral: bool) -> tuple[bool, str]:
        self._set_lateral(lateral)
        with self._lock:
            self._distance_m = 0.0
        return self._control(True)

    def stop(self) -> tuple[bool, str, float]:
        ok, msg = self._control(False)
        return ok, msg, self.distance_m

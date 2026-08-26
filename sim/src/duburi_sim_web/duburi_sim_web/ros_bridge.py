#!/usr/bin/env python3

"""Background rclpy node used by the FastAPI lab server."""

from __future__ import annotations

import threading
from typing import Optional

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

try:
    from duburi_interfaces.msg import DuburiState
except ImportError:
    DuburiState = None  # type: ignore


FRONT_FX = '/duburi/sim/front_camera/image_fx'
BOTTOM_FX = '/duburi/sim/bottom_camera/image_fx'
FRONT_RAW = '/duburi/sim/front_camera/image_raw'
BOTTOM_RAW = '/duburi/sim/bottom_camera/image_raw'
GROUND_TRUTH = '/duburi/sim/ground_truth'
STATE_TOPIC = '/duburi/state'


def _msg_to_jpeg(msg: Image, quality: int = 82) -> Optional[bytes]:
    enc = msg.encoding
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    expected = msg.height * msg.step
    if raw.size < expected:
        return None
    ch = 3 if enc in ('rgb8', 'bgr8') else (4 if 'a8' in enc else 1)
    img = raw[:expected].reshape((msg.height, msg.step))[:, : msg.width * ch]
    img = img.reshape((msg.height, msg.width, ch))
    if enc == 'rgb8':
        bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif enc == 'rgba8':
        bgr = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    elif enc == 'mono8':
        bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    else:
        bgr = img[:, :, :3]
    ok, buf = cv2.imencode('.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    return buf.tobytes() if ok else None


class LabRosNode(Node):
    def __init__(self) -> None:
        super().__init__('duburi_sim_lab')
        self._lock = threading.Lock()
        self._jpeg = {'front': None, 'bottom': None}
        # Preview defaults to raw for smoother MJPEG; record still uses FX checkbox.
        self._use_fx = False
        self._jpeg_seq = {'front': 0, 'bottom': 0}
        self._state = {
            'armed': False,
            'mode': 'UNKNOWN',
            'yaw_deg': 0.0,
            'depth_m': 0.0,
            'battery_voltage': 0.0,
            'have_state': False,
        }
        self._gt = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'have': False}
        self._fx_params = {
            'turbidity': 0.45,
            'backscatter': 0.55,
            'blur_sigma': 0.8,
            'noise': 0.012,
            'vignette': 0.25,
            'enabled': True,
        }

        self._sub_front = None
        self._sub_bottom = None
        self._rewire_cameras(False)

        if DuburiState is not None:
            self.create_subscription(DuburiState, STATE_TOPIC, self._on_state, 10)
        self.create_subscription(Odometry, GROUND_TRUTH, self._on_gt, 10)

        self._set_cli = self.create_client(SetParameters, '/underwater_fx/set_parameters')
        self._get_cli = self.create_client(GetParameters, '/underwater_fx/get_parameters')

    def _rewire_cameras(self, use_fx: bool) -> None:
        self._use_fx = use_fx
        if self._sub_front is not None:
            self.destroy_subscription(self._sub_front)
            self.destroy_subscription(self._sub_bottom)
        front = FRONT_FX if use_fx else FRONT_RAW
        bottom = BOTTOM_FX if use_fx else BOTTOM_RAW
        self._sub_front = self.create_subscription(
            Image, front, lambda m: self._on_image('front', m), qos_profile_sensor_data
        )
        self._sub_bottom = self.create_subscription(
            Image, bottom, lambda m: self._on_image('bottom', m), qos_profile_sensor_data
        )

    def _on_image(self, cam: str, msg: Image) -> None:
        jpeg = _msg_to_jpeg(msg)
        if jpeg is None:
            return
        with self._lock:
            self._jpeg[cam] = jpeg
            self._jpeg_seq[cam] = self._jpeg_seq.get(cam, 0) + 1

    def _on_state(self, msg) -> None:
        with self._lock:
            self._state = {
                'armed': bool(msg.armed),
                'mode': str(msg.mode),
                'yaw_deg': float(msg.yaw_deg),
                'depth_m': float(msg.depth_m),
                'battery_voltage': float(msg.battery_voltage),
                'have_state': True,
            }

    def _on_gt(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        with self._lock:
            self._gt = {'x': p.x, 'y': p.y, 'z': p.z, 'have': True}

    def latest_jpeg(self, cam: str) -> Optional[bytes]:
        with self._lock:
            return self._jpeg.get(cam)

    def snapshot(self) -> dict:
        with self._lock:
            return {
                'state': dict(self._state),
                'ground_truth': dict(self._gt),
                'use_fx': self._use_fx,
                'fx': dict(self._fx_params),
                'cameras': {
                    'front': self._jpeg['front'] is not None,
                    'bottom': self._jpeg['bottom'] is not None,
                },
            }

    def set_fx_params(self, updates: dict) -> dict:
        if not self._set_cli.wait_for_service(timeout_sec=1.0):
            # Keep local cache for UI even if node not up yet.
            self._fx_params.update({k: updates[k] for k in updates if k in self._fx_params})
            return self._fx_params

        req = SetParameters.Request()
        for key, value in updates.items():
            if key not in self._fx_params and key != 'enabled':
                continue
            p = Parameter()
            p.name = key
            if isinstance(value, bool):
                p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
            else:
                p.value = ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)
                )
            req.parameters.append(p)
            self._fx_params[key] = value if not isinstance(value, float) else float(value)

        fut = self._set_cli.call_async(req)
        # Don't block the executor thread forever; spin briefly from caller side.
        return self._fx_params, fut

    def refresh_fx_params(self) -> dict:
        if not self._get_cli.wait_for_service(timeout_sec=0.5):
            return self._fx_params
        req = GetParameters.Request()
        req.names = list(self._fx_params.keys())
        fut = self._get_cli.call_async(req)
        return fut


class RosWorker:
    """Owns the rclpy context + executor on a daemon thread."""

    def __init__(self) -> None:
        self.node: Optional[LabRosNode] = None
        self._executor: Optional[SingleThreadedExecutor] = None
        self._thread: Optional[threading.Thread] = None
        self._ready = threading.Event()

    def start(self) -> None:
        if self._thread is not None:
            return

        def _run():
            if not rclpy.ok():
                rclpy.init()
            self.node = LabRosNode()
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self.node)
            self._ready.set()
            self._executor.spin()

        self._thread = threading.Thread(target=_run, name='lab-ros', daemon=True)
        self._thread.start()
        self._ready.wait(timeout=10.0)

    def stop(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
        if self.node is not None:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

#!/usr/bin/env python3

"""Underwater image degradation for vision / OD training.

Subscribes to the drop-in raw camera topics (unchanged contract) and publishes
degraded copies on image_fx. Effects are driven by ROS parameters so the web
lab can change turbidity live without restarting Gazebo.

Algorithm (lightweight UUV-style attenuation):
  - depth-aware RGB channel attenuation (red falls off fastest)
  - turbidity-scaled backscatter haze
  - mild Gaussian blur + additive noise
  - optional vignette
"""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

FRONT_RAW = '/duburi/sim/front_camera/image_raw'
BOTTOM_RAW = '/duburi/sim/bottom_camera/image_raw'
FRONT_FX = '/duburi/sim/front_camera/image_fx'
BOTTOM_FX = '/duburi/sim/bottom_camera/image_fx'
GROUND_TRUTH = '/duburi/sim/ground_truth'

# Water attenuation coefficients (1/m), roughly Jerlov coastal water.
ATTEN_RGB = np.array([0.45, 0.18, 0.12], dtype=np.float32)
HAZE_RGB = np.array([0.05, 0.22, 0.28], dtype=np.float32)


def _msg_to_bgr(msg: Image) -> np.ndarray:
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(
        msg.encoding
    )
    if channels is None:
        raise ValueError(f'unsupported encoding {msg.encoding}')
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    expected = msg.height * msg.step
    img = raw[:expected].reshape((msg.height, msg.step))[:, : msg.width * channels]
    img = img.reshape((msg.height, msg.width, channels))
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if msg.encoding == 'rgba8':
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    if msg.encoding == 'bgra8':
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    if msg.encoding == 'mono8':
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img


def _bgr_to_msg(bgr: np.ndarray, stamp, frame_id: str) -> Image:
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = rgb.shape[:2]
    msg.encoding = 'rgb8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = rgb.tobytes()
    return msg


def apply_underwater_fx(
    bgr: np.ndarray,
    depth_m: float,
    turbidity: float,
    backscatter: float,
    blur_sigma: float,
    noise: float,
    vignette: float,
    atten_scale: float = 1.0,
) -> np.ndarray:
    """Return a degraded BGR image. depth_m is negative below the surface."""
    depth = max(0.05, abs(float(depth_m)))
    t = max(0.0, float(turbidity))

    img = bgr.astype(np.float32) / 255.0
    # Effective optical path grows with turbidity.
    path = depth * (0.35 + 0.65 * t)
    atten = np.exp(-ATTEN_RGB * path * atten_scale).reshape(1, 1, 3)
    haze = HAZE_RGB.reshape(1, 1, 3)
    # Beer-Lambert blend toward haze colour.
    scatter = np.clip(backscatter * t * (1.0 - np.exp(-0.4 * path)), 0.0, 1.0)
    out = img * atten * (1.0 - scatter) + haze * scatter

    # Contrast collapse with turbidity.
    mean = out.mean(axis=(0, 1), keepdims=True)
    out = mean + (out - mean) * (1.0 - 0.35 * t)

    out = np.clip(out, 0.0, 1.0)
    u8 = (out * 255.0).astype(np.uint8)

    sigma = blur_sigma * (0.4 + 1.2 * t)
    if sigma > 0.15:
        k = max(3, int(round(sigma * 2) * 2 + 1))
        u8 = cv2.GaussianBlur(u8, (k, k), sigma)

    if noise > 1e-4:
        n = np.random.normal(0.0, noise * 255.0 * (0.5 + t), u8.shape)
        u8 = np.clip(u8.astype(np.float32) + n, 0, 255).astype(np.uint8)

    if vignette > 1e-4:
        h, w = u8.shape[:2]
        yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
        cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
        r = np.sqrt(((xx - cx) / cx) ** 2 + ((yy - cy) / cy) ** 2)
        mask = 1.0 - vignette * np.clip(r, 0.0, 1.0) ** 2
        u8 = np.clip(u8.astype(np.float32) * mask[..., None], 0, 255).astype(np.uint8)

    return u8


class UnderwaterFx(Node):
    def __init__(self) -> None:
        super().__init__('underwater_fx')

        self.declare_parameter('turbidity', 0.45)
        self.declare_parameter('backscatter', 0.55)
        self.declare_parameter('blur_sigma', 0.8)
        self.declare_parameter('noise', 0.012)
        self.declare_parameter('vignette', 0.25)
        self.declare_parameter('atten_scale', 1.0)
        self.declare_parameter('randomize_on_start', False)
        self.declare_parameter('enabled', True)

        self._depth = -0.8
        self._reload_params()
        if self.get_parameter('randomize_on_start').value:
            self._randomize()

        self.create_subscription(Odometry, GROUND_TRUTH, self._on_odom, 10)
        self._pub_front = self.create_publisher(Image, FRONT_FX, qos_profile_sensor_data)
        self._pub_bottom = self.create_publisher(Image, BOTTOM_FX, qos_profile_sensor_data)
        self.create_subscription(
            Image, FRONT_RAW, lambda m: self._on_image(m, self._pub_front), qos_profile_sensor_data
        )
        self.create_subscription(
            Image, BOTTOM_RAW, lambda m: self._on_image(m, self._pub_bottom), qos_profile_sensor_data
        )
        self.add_on_set_parameters_callback(self._on_params)
        self.get_logger().info(
            f'underwater_fx ready  turbidity={self._turbidity:.2f}  '
            f'publishing {FRONT_FX} / {BOTTOM_FX}'
        )

    def _reload_params(self) -> None:
        self._turbidity = float(self.get_parameter('turbidity').value)
        self._backscatter = float(self.get_parameter('backscatter').value)
        self._blur_sigma = float(self.get_parameter('blur_sigma').value)
        self._noise = float(self.get_parameter('noise').value)
        self._vignette = float(self.get_parameter('vignette').value)
        self._atten_scale = float(self.get_parameter('atten_scale').value)
        self._enabled = bool(self.get_parameter('enabled').value)

    def _randomize(self) -> None:
        rng = np.random.default_rng()
        self.set_parameters(
            [
                rclpy.parameter.Parameter('turbidity', value=float(rng.uniform(0.2, 0.85))),
                rclpy.parameter.Parameter('backscatter', value=float(rng.uniform(0.3, 0.8))),
                rclpy.parameter.Parameter('blur_sigma', value=float(rng.uniform(0.3, 1.4))),
                rclpy.parameter.Parameter('noise', value=float(rng.uniform(0.005, 0.025))),
            ]
        )
        self._reload_params()

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name == 'turbidity':
                self._turbidity = float(p.value)
            elif p.name == 'backscatter':
                self._backscatter = float(p.value)
            elif p.name == 'blur_sigma':
                self._blur_sigma = float(p.value)
            elif p.name == 'noise':
                self._noise = float(p.value)
            elif p.name == 'vignette':
                self._vignette = float(p.value)
            elif p.name == 'atten_scale':
                self._atten_scale = float(p.value)
            elif p.name == 'enabled':
                self._enabled = bool(p.value)
        return SetParametersResult(successful=True)

    def _on_odom(self, msg: Odometry) -> None:
        self._depth = float(msg.pose.pose.position.z)

    def _on_image(self, msg: Image, pub) -> None:
        try:
            bgr = _msg_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return
        if self._enabled:
            bgr = apply_underwater_fx(
                bgr,
                self._depth,
                self._turbidity,
                self._backscatter,
                self._blur_sigma,
                self._noise,
                self._vignette,
                self._atten_scale,
            )
        pub.publish(_bgr_to_msg(bgr, msg.header.stamp, msg.header.frame_id))

    def snapshot_params(self) -> dict:
        return {
            'turbidity': self._turbidity,
            'backscatter': self._backscatter,
            'blur_sigma': self._blur_sigma,
            'noise': self._noise,
            'vignette': self._vignette,
            'atten_scale': self._atten_scale,
            'enabled': self._enabled,
            'depth_m': self._depth,
        }


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = UnderwaterFx()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

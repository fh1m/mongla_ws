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

from functools import lru_cache

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
FRONT_RANGE = '/duburi/sim/front_camera/range'
BOTTOM_RANGE = '/duburi/sim/bottom_camera/range'
FRONT_FX = '/duburi/sim/front_camera/image_fx'
BOTTOM_FX = '/duburi/sim/bottom_camera/image_fx'
GROUND_TRUTH = '/duburi/sim/ground_truth'

# Water attenuation coefficients (1/m), roughly Jerlov coastal water.
ATTEN_RGB = np.array([0.45, 0.18, 0.12], dtype=np.float32)
HAZE_RGB = np.array([0.05, 0.22, 0.28], dtype=np.float32)

# Pool water is far clearer than the open ocean ATTEN_RGB describes. See the
# per-pixel block in apply_underwater_fx.
RANGE_ATTEN = 0.22
RANGE_FLOOR = 0.30


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


_NOISE_BANK: dict = {}
_NOISE_TILES = 8


def _noise_tile(shape: tuple) -> np.ndarray:
    """A unit-variance noise field, drawn from a small precomputed bank.

    `np.random.normal` over a 640x480x3 frame measured 22.0 ms -- on its own
    most of a 30 Hz budget, and doubled because one node degrades both cameras.
    It was the single reason underwater_fx pinned a core, and that CPU comes
    straight out of Gazebo's render loop: the camera published 2.9 Hz with
    0.44 s of jitter, which is what the operator sees as laggy teleop video and
    juddery dataset playback.

    Eight fields are drawn once and one is picked per frame with a random
    circular shift, so successive frames do not share a pattern. For sensor
    noise this is not an approximation worth apologising for -- real sensor
    noise is spatially correlated anyway, and nothing downstream does
    statistics on it. Costs ~0.5 ms.
    """
    bank = _NOISE_BANK.get(shape)
    if bank is None:
        rng = np.random.default_rng(0xD00B)
        bank = [rng.standard_normal(shape, dtype=np.float32)
                for _ in range(_NOISE_TILES)]
        _NOISE_BANK[shape] = bank
    tile = bank[np.random.randint(_NOISE_TILES)]
    return np.roll(tile, (np.random.randint(shape[0]), np.random.randint(shape[1])),
                   axis=(0, 1))


@lru_cache(maxsize=8)
def _vignette_mask(shape: tuple, vignette: float) -> np.ndarray:
    """Radial falloff mask, cached on (shape, strength).

    This used to be rebuilt per frame: an `np.mgrid` pair, two float32
    480x640 arrays, a sqrt and a power, thirty times a second. Measured
    2026-08-28 it was a large part of underwater_fx pinning a full core at
    106 %, which starves Gazebo's render loop -- the camera published 2.9 Hz
    with a 0.44 s standard deviation, and that jitter is what the operator
    sees as laggy teleop video and juddery dataset playback.

    Nothing in the mask depends on the image, only on its size and the
    strength, so there are at most a couple of distinct masks in a run.
    """
    h, w = shape[0], shape[1]
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    r = np.sqrt(((xx - cx) / cx) ** 2 + ((yy - cy) / cy) ** 2)
    mask = (1.0 - vignette * np.clip(r, 0.0, 1.0) ** 2).astype(np.float32)
    return np.ascontiguousarray(np.repeat(mask[:, :, None], shape[2], axis=2))


def apply_underwater_fx(
    bgr: np.ndarray,
    depth_m: float,
    turbidity: float,
    backscatter: float,
    blur_sigma: float,
    noise: float,
    vignette: float,
    atten_scale: float = 1.0,
    # Per-pixel path length, metres, same shape as the frame. None ->
    # uniform attenuation by the vehicle's own depth, the old behaviour.
    range_m=None,
) -> np.ndarray:
    """Return a degraded BGR image. depth_m is negative below the surface."""
    depth = max(0.05, abs(float(depth_m)))
    t = max(0.0, float(turbidity))

    # ONE LOOKUP TABLE for attenuation, haze and contrast.
    #
    # All three are per-channel AFFINE functions of the uint8 input, and the
    # composition of affine maps is affine -- so the whole chain is exactly a
    # 256-entry table per channel, with no approximation anywhere.
    #
    # Worth doing because the obvious numpy spelling is the slow one:
    # `out *= gain` with gain shaped (1,1,3) measured 3.09 ms, because
    # broadcasting drops numpy off its vectorised inner loop onto the strided
    # path. Materialising or table-ising the operand is ~10x faster than
    # letting it broadcast. cv2.LUT with a float32 table returns float32, so
    # blur/noise/vignette still get a float frame.
    #
    #   v      = x / 255
    #   v      = v * gain + bias                    (attenuate + haze)
    #   v      = v * k + mean_post * (1 - k)        (contrast collapse)
    #   =>  x * (gain * k / 255)  +  (bias * k + mean_post * (1 - k))
    #
    # mean_post is the mean AFTER gain/bias, but mean is linear so it follows
    # from the raw mean without touching the full frame. That raw mean is taken
    # on every 4th pixel in each axis: it is only a contrast anchor, 1/16 the
    # work, and the difference is far below a JPEG quantisation step. Do NOT
    # subsample anything whose per-pixel value is consumed.
    path = depth * (0.35 + 0.65 * t)          # optical path grows with turbidity
    atten = np.exp(-ATTEN_RGB * path * atten_scale)
    scatter = float(np.clip(backscatter * t * (1.0 - np.exp(-0.4 * path)), 0.0, 1.0))
    gain = atten * (1.0 - scatter)
    bias = HAZE_RGB * scatter

    k = 1.0 - 0.35 * t
    mean_raw = bgr[::4, ::4].reshape(-1, 3).mean(axis=0) / 255.0
    mean_post = mean_raw * gain + bias

    ramp = np.arange(256, dtype=np.float32)
    lut = np.empty((1, 256, 3), dtype=np.float32)
    for c in range(3):
        lut[0, :, c] = ramp * (gain[c] * k / 255.0) + (bias[c] * k + mean_post[c] * (1.0 - k))
    out = cv2.LUT(bgr, lut)

    if range_m is not None and range_m.shape[:2] == bgr.shape[:2]:
        # The LUT above applied ONE path length to every pixel. Correct each by
        # the ratio between its true path and that one, so near pixels get less
        # attenuation and far pixels more. Doing it as a correction keeps the
        # LUT -- which is what makes this fast -- for one exponential per frame.
        r = np.nan_to_num(range_m, nan=depth, posinf=depth, neginf=depth)
        r = np.clip(r, 0.05, 40.0).astype(np.float32)
        # RANGE_ATTEN scales the along-path coefficient down from the open-ocean
        # figures in ATTEN_RGB. Those are right for the sea and much too strong
        # for a chlorinated pool: applied raw, the far wall of a 25 m pool went
        # essentially black (red down to 0.004 of its value at 20 m) while a
        # real pool photo shows the far end clearly, just blue and low-contrast.
        #
        # The floor keeps the far field visible-but-degraded, which is the point
        # -- a detector must find props at range through worse imagery, not be
        # handed a black frame it cannot possibly work with.
        rel = (r * (0.35 + 0.65 * t) - path)[:, :, None]
        att = np.exp(-ATTEN_RGB.reshape(1, 1, 3) * rel * atten_scale * RANGE_ATTEN)
        out *= np.maximum(att, RANGE_FLOOR)
        # Haze accumulates along the path too: a distant surface is not merely
        # dimmer, it is washed toward the colour of the water in between.
        far = np.clip(rel * 0.02 * t, 0.0, 0.55)
        out = out * (1.0 - far) + HAZE_RGB.reshape(1, 1, 3) * far

    # ONE uint8 conversion, at the very end.
    #
    # Blur, noise and vignette each used to round-trip the whole frame
    # uint8 -> float32 -> clip -> uint8. Measured, each of those round trips is
    # ~5.6 ms on a 640x480x3 frame -- more than the effect it was applying.
    # Staying in float32 [0,1] and converting once cuts the function roughly in
    # half. cv2.GaussianBlur takes float32 directly, so nothing is given up.
    sigma = blur_sigma * (0.4 + 1.2 * t)
    if sigma > 0.15:
        k = max(3, int(round(sigma * 2) * 2 + 1))
        out = cv2.GaussianBlur(out, (k, k), sigma)

    if noise > 1e-4:
        out += _noise_tile(out.shape) * (noise * (0.5 + t))

    if vignette > 1e-4:
        # Full (h, w, 3), not (h, w, 1) broadcast -- same 3.09 ms -> 0.6 ms
        # reason as the gain above.
        out *= _vignette_mask(out.shape, vignette)

    out *= np.float32(255.0)
    np.clip(out, 0.0, 255.0, out=out)
    return out.astype(np.uint8)


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
        # Latest range image per camera, for per-pixel attenuation. None until
        # one arrives, and the filter falls back to uniform if it never does --
        # so a missing depth sensor degrades to the old behaviour rather than
        # to a crash or a black frame.
        self._range = {'front': None, 'bottom': None}
        self._reload_params()
        if self.get_parameter('randomize_on_start').value:
            self._randomize()

        self.create_subscription(Odometry, GROUND_TRUTH, self._on_odom, 10)
        self._pub_front = self.create_publisher(Image, FRONT_FX, qos_profile_sensor_data)
        self._pub_bottom = self.create_publisher(Image, BOTTOM_FX, qos_profile_sensor_data)
        self.create_subscription(
            Image, FRONT_RANGE, lambda m: self._on_range('front', m),
            qos_profile_sensor_data
        )
        self.create_subscription(
            Image, BOTTOM_RANGE, lambda m: self._on_range('bottom', m),
            qos_profile_sensor_data
        )
        self.create_subscription(
            Image, FRONT_RAW, lambda m: self._on_image(m, self._pub_front, 'front'), qos_profile_sensor_data
        )
        self.create_subscription(
            Image, BOTTOM_RAW, lambda m: self._on_image(m, self._pub_bottom, 'bottom'), qos_profile_sensor_data
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

    def _on_range(self, cam: str, msg: Image) -> None:
        # 32FC1 from Gazebo's depth_camera: metres along the optical axis.
        try:
            a = np.frombuffer(msg.data, dtype=np.float32)
            self._range[cam] = a[:msg.height * msg.width].reshape(
                msg.height, msg.width)
        except Exception:
            self._range[cam] = None

    def _on_image(self, msg: Image, pub, cam: str = 'front') -> None:
        # NOTHING unless somebody is watching this camera.
        #
        # This node is the single biggest drag on simulator smoothness, and the
        # cost is not the filter -- that is ~4 ms/frame -- it is moving two
        # extra ~1 MB image streams per camera through DDS and re-encoding
        # them. Measured 2026-08-28 on the SAUVC final course:
        #
        #     fx off :  8.21 Hz,  jitter (stdev) 5 ms
        #     fx on  :  2.83 Hz,  jitter        440 ms
        #
        # That jitter is exactly what the operator reports as laggy teleop
        # video and juddery dataset playback. Almost always only one camera is
        # actually being looked at, so gating on subscriber count gives the
        # other one back for free -- and with no viewer at all (a headless
        # mission run) the node costs nothing.
        #
        # Cheap and exact: rmw already tracks this, and image_fx is a
        # debug/dataset topic, so there is no late-joiner to miss a frame that
        # mattered. Do NOT copy this to a control-path publisher.
        if pub.get_subscription_count() == 0:
            return
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
                range_m=self._range.get(cam),
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

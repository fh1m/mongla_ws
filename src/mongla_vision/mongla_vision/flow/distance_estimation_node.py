#!/usr/bin/env python3
"""distance_estimation_node -- downward-camera optical-flow distance (DVL-free).

Accumulates metric distance travelled along ONE latched axis from sparse
Lucas-Kanade optical flow on the downward camera, rotation-compensated by Pixhawk
gyro rates and metric-scaled by Bar30 depth + a configured pool_depth_m. Bounded
by calc_distance('start'/'stop') via the latched distance_control topic.

Topics
  in    /mongla/vision/<cam>/image_raw     sensor_msgs/Image       (downward stream)
  in    /mongla/imu_rates                  geometry_msgs/Vector3Stamped (x=pitch,y=roll,z=yaw rad/s)
  in    /mongla/state                      mongla_interfaces/MonglaState (depth_m + yaw_deg)
  in    /mongla/vision/<cam>/distance_control    std_msgs/String  (LATCHED: 'start_axial'|'start_lateral'|'stop')
  out   /mongla/vision/<cam>/distance_traveled  std_msgs/Float32   (running metres; continuous)
  out   /mongla/vision/<cam>/distance_debug     std_msgs/Float32MultiArray [dist,height,n_tracks,active]

Mirrors depth_estimation_node: light subscriber, sparse LK on CPU (does NOT fight
the YOLO detector for GPU -- distance mode runs with the detectors paused).
"""

import sys
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray, String
from geometry_msgs.msg import Vector3Stamped
from rclpy.qos import QoSDurabilityPolicy

from mongla_interfaces.msg import MonglaState
from mongla_vision.flow.flow_math import (
    DistanceAccumulator, height_above_floor, rotation_flow_px, robust_flow,
    interp_rate,
)

# Shi-Tomasi corner + LK params (CPU, real-time on Jetson at 640x480).
_FEATURE_PARAMS = dict(maxCorners=120, qualityLevel=0.01, minDistance=8, blockSize=7)
_LK_PARAMS      = dict(winSize=(21, 21), maxLevel=3,
                       criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
_MIN_TRACKS     = 6      # below this -> re-seed corners, hold distance this frame
# B10. How long a height reading stays usable after the last depth sample.
# Depth arrives with /mongla/state (>=1 Hz heartbeat, faster on change), so 2.0 s
# is several missed updates -- long enough not to gate on ordinary jitter, short
# enough that a dead depth source stops scaling flow rather than scaling it wrong.
HEIGHT_STALE_S  = 2.0
_RESEED_EVERY   = 10     # frames between forced corner re-detections (tracks decay)


class DistanceEstimationNode(Node):
    def __init__(self):
        super().__init__('mongla_distance_estimator')

        self.declare_parameter('camera',           'downward')
        self.declare_parameter('pool_depth_m',      4.0)     # total water column (surface->floor)
        self.declare_parameter('camera_focal_px', 500.0)     # f_px from intrinsics calibration
        # Mount calibration (static-tilt test): sign of each rotation-comp axis
        # and whether image x/y map to roll/pitch or are swapped.
        self.declare_parameter('rot_sign_x',       1.0)
        self.declare_parameter('rot_sign_y',       1.0)
        self.declare_parameter('flow_smooth_alpha', 0.4)     # EMA on per-frame translational flow

        cam = str(self.get_parameter('camera').value or 'downward').strip()
        self._pool_depth = float(self.get_parameter('pool_depth_m').value)
        self._f_px       = float(self.get_parameter('camera_focal_px').value)
        self._rsx        = float(self.get_parameter('rot_sign_x').value)
        self._rsy        = float(self.get_parameter('rot_sign_y').value)
        self._alpha      = float(self.get_parameter('flow_smooth_alpha').value)
        self._cam        = cam

        ns = f'/mongla/vision/{cam}'
        img_qos = QoSProfile(depth=2, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, f'{ns}/image_raw', self._on_image, img_qos)
        self.create_subscription(Vector3Stamped, '/mongla/imu_rates', self._on_rates, 50)
        self.create_subscription(MonglaState, '/mongla/state', self._on_state, 10)

        self._pub_dist  = self.create_publisher(Float32, f'{ns}/distance_traveled', 10)
        self._pub_debug = self.create_publisher(Float32MultiArray, f'{ns}/distance_debug', 10)
        # Control is a LATCHED topic (not a service): fire-and-forget from the
        # manager, so no synchronous service round-trip is spun on the already-
        # spinning manager node. Values: 'start_axial' | 'start_lateral' | 'stop'.
        ctrl_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, f'{ns}/distance_control',
                                 self._on_control, ctrl_qos)

        self._bridge = CvBridge()
        self._acc    = DistanceAccumulator()
        # timestamped (stamp_s, pitch_rate, roll_rate) ring for interpolation.
        self._rate_buf: deque = deque(maxlen=64)
        self._depth_m: float | None = None
        self._yaw_deg: float | None = None
        self._last_height: float | None = None
        # B10: the height latch needs a CLOCK. Without one it is unbounded --
        # if depth stops arriving, flow keeps being scaled by the last known
        # height for ever, and metres-per-pixel is directly proportional to it,
        # so a hull that has since changed altitude integrates at the wrong
        # scale with a plausible number and no warning. Mirrors the pattern
        # bno085._fresh_raw_yaw already uses: past the bound, return None.
        self._last_height_t: float | None = None

        # LK state.
        self._prev_gray = None
        self._prev_pts  = None
        self._prev_t    = None
        self._frame_i   = 0
        self._flow_ema  = (0.0, 0.0)

        self._n_tracks  = 0
        self._last_log  = time.monotonic()

        self.get_logger().info(
            f'[DIST ] camera={cam!r} pool_depth={self._pool_depth:.2f}m '
            f'f_px={self._f_px:.0f} -- IDLE (calc_distance start to arm)')

    # ── control topic ('start_axial' | 'start_lateral' | 'stop') ────────────
    def _on_control(self, msg: String) -> None:
        cmd = str(msg.data or '').strip().lower()
        if cmd.startswith('start'):
            lateral = cmd.endswith('lateral')
            yaw = self._yaw_deg if self._yaw_deg is not None else 0.0
            self._acc.start(np.radians(yaw), lateral)
            self._reset_lk()
            self.get_logger().info(
                f'[DIST ] ACTIVE axis_yaw={yaw:.1f} '
                f'{"lateral" if lateral else "axial"}')
        elif cmd == 'stop':
            d = self._acc.stop()
            # Publish the frozen total immediately so a stopper reading the
            # cached distance_traveled sees the final value without a race.
            self._publish(d, self._last_height, self._n_tracks)
            self.get_logger().info(f'[DIST ] STOPPED total={d:.3f}m')

    # ── inputs ──────────────────────────────────────────────────────────────
    def _on_rates(self, msg: Vector3Stamped) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # x=pitch_rate, y=roll_rate (see manager _imu_rates_tick).
        self._rate_buf.append((t, float(msg.vector.x), float(msg.vector.y)))

    def _on_state(self, msg: MonglaState) -> None:
        if not np.isnan(msg.depth_m):
            self._depth_m = float(msg.depth_m)
        if not np.isnan(msg.yaw_deg):
            self._yaw_deg = float(msg.yaw_deg)

    # ── LK flow ─────────────────────────────────────────────────────────────
    def _reset_lk(self) -> None:
        self._prev_gray = None
        self._prev_pts  = None
        self._prev_t    = None
        self._flow_ema  = (0.0, 0.0)

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'[DIST ] cv_bridge decode failed: {exc!r}')
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        t    = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t <= 0.0:
            t = time.monotonic()
        self._frame_i += 1

        if self._prev_gray is None or self._prev_pts is None \
                or len(self._prev_pts) < _MIN_TRACKS \
                or (self._frame_i % _RESEED_EVERY == 0):
            self._prev_pts  = cv2.goodFeaturesToTrack(gray, mask=None, **_FEATURE_PARAMS)
            self._prev_gray = gray
            self._prev_t    = t
            self._publish(self._acc.distance_m, self._last_height, self._n_tracks)
            return

        next_pts, status, _err = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, self._prev_pts, None, **_LK_PARAMS)
        dt = max(t - (self._prev_t or t), 1e-3)

        flow = robust_flow(self._prev_pts, next_pts, status, min_tracks=_MIN_TRACKS)
        # advance LK anchor to this frame (re-seed if flow starved).
        if flow is None or next_pts is None:
            self._prev_pts  = cv2.goodFeaturesToTrack(gray, mask=None, **_FEATURE_PARAMS)
            self._n_tracks  = 0
        else:
            st = np.asarray(status).reshape(-1).astype(bool)
            self._prev_pts = next_pts[st].reshape(-1, 1, 2)
            self._n_tracks = int(st.sum())
        self._prev_gray = gray
        self._prev_t    = t

        if flow is not None and self._acc.active:
            self._fold_flow(flow, dt, t)

        self._publish(self._acc.distance_m, self._last_height, self._n_tracks)

    def _fold_flow(self, flow_px, dt: float, t: float) -> None:
        """Rotation-comp + metric-scale one frame's flow into the accumulator."""
        rates = interp_rate(self._rate_buf, t - dt * 0.5)   # frame-pair midpoint
        if rates is not None:
            pitch_rate, roll_rate = rates
            rdx, rdy = rotation_flow_px(self._f_px, pitch_rate, roll_rate, dt)
            trans = (flow_px[0] - self._rsx * rdx, flow_px[1] - self._rsy * rdy)
        else:
            trans = flow_px   # no rate yet -> uncompensated (still integrates)

        # EMA-smooth the translational flow (glint/turbidity leaves residual jitter).
        a = self._alpha
        self._flow_ema = (a * trans[0] + (1 - a) * self._flow_ema[0],
                          a * trans[1] + (1 - a) * self._flow_ema[1])

        height = None
        if self._depth_m is not None:
            height = height_above_floor(self._pool_depth, self._depth_m)
        if height is not None:
            self._last_height = height
            self._last_height_t = t
        # B05: pass the height THROUGH, never `or 0.0`.
        # `DistanceAccumulator.add` refuses a frame on `height_m is None`, on
        # purpose. `or 0.0` made that test unreachable -- a None became 0.0, the
        # guard passed, and execution reached `distance_m += proj * 0.0 / f_px`,
        # which adds exactly zero. Those are NOT the same: the guard skips the
        # frame ("we could not tell"), the coercion records it ("it did not
        # move"). Real motion in that window was silently under-reported.
        self._acc.add(self._flow_ema, self._fresh_height(t), self._f_px)

    def _fresh_height(self, t: float) -> float | None:
        """Last height if it is still trustworthy, else None (B10).

        None is the honest answer and the accumulator is built to receive it --
        it refuses the frame rather than integrating a wrong scale.
        """
        if self._last_height is None or self._last_height_t is None:
            return None
        if (t - self._last_height_t) > HEIGHT_STALE_S:
            return None
        return self._last_height

    # ── outputs ─────────────────────────────────────────────────────────────
    def _publish(self, dist: float, height, n_tracks: int) -> None:
        m = Float32(); m.data = float(dist)
        self._pub_dist.publish(m)
        dbg = Float32MultiArray()
        # NaN, not 0.0: an absent height is not a height of zero. float('nan')
        # survives the Float32MultiArray and every consumer that plots it shows
        # a gap instead of a floor-level reading that never happened.
        dbg.data = [float(dist),
                    float(height) if height is not None else float('nan'),
                    float(n_tracks),
                    1.0 if self._acc.active else 0.0]
        self._pub_debug.publish(dbg)
        now = time.monotonic()
        if now - self._last_log >= 1.0 and self._acc.active:
            self._last_log = now
            self.get_logger().info(
                f'[DIST ] d={dist:+.3f}m  '
                f'height={f"{height:.2f}m" if height is not None else "--"}  '
                f'tracks={n_tracks}')


def main(args=None):
    rclpy.init(args=args)
    node = DistanceEstimationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
    sys.exit(0)

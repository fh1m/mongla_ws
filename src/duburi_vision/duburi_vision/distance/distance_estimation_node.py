#!/usr/bin/env python3
"""distance_estimation_node -- downward-camera optical-flow distance (DVL-free).

Accumulates metric distance travelled along ONE latched axis from sparse
Lucas-Kanade optical flow on the downward camera, rotation-compensated by Pixhawk
gyro rates and metric-scaled by Bar30 depth + a configured pool_depth_m. Bounded
by calc_distance('start'/'stop') via the distance_control service.

Topics
  in    /duburi/vision/<cam>/image_raw     sensor_msgs/Image       (downward stream)
  in    /duburi/imu_rates                  geometry_msgs/Vector3Stamped (x=pitch,y=roll,z=yaw rad/s)
  in    /duburi/state                      duburi_interfaces/DuburiState (depth_m + yaw_deg)
  out   /duburi/vision/<cam>/distance_traveled  std_msgs/Float32   (running metres; continuous)
  out   /duburi/vision/<cam>/distance_debug     std_msgs/Float32MultiArray [dist,height,n_tracks,active]
Service
  /duburi/vision/<cam>/distance_control    std_srvs/SetBool  (true=start/latch/reset, false=stop)

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
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Vector3Stamped
from std_srvs.srv import SetBool

from duburi_interfaces.msg import DuburiState
from duburi_vision.distance.flow_math import (
    DistanceAccumulator, height_above_floor, rotation_flow_px, robust_flow,
    interp_rate,
)

# Shi-Tomasi corner + LK params (CPU, real-time on Jetson at 640x480).
_FEATURE_PARAMS = dict(maxCorners=120, qualityLevel=0.01, minDistance=8, blockSize=7)
_LK_PARAMS      = dict(winSize=(21, 21), maxLevel=3,
                       criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
_MIN_TRACKS     = 6      # below this -> re-seed corners, hold distance this frame
_RESEED_EVERY   = 10     # frames between forced corner re-detections (tracks decay)


class DistanceEstimationNode(Node):
    def __init__(self):
        super().__init__('duburi_distance_estimator')

        self.declare_parameter('camera',           'downward')
        self.declare_parameter('pool_depth_m',      4.0)     # total water column (surface->floor)
        self.declare_parameter('camera_focal_px', 500.0)     # f_px from intrinsics calibration
        # Mount calibration (static-tilt test): sign of each rotation-comp axis
        # and whether image x/y map to roll/pitch or are swapped.
        self.declare_parameter('rot_sign_x',       1.0)
        self.declare_parameter('rot_sign_y',       1.0)
        self.declare_parameter('flow_smooth_alpha', 0.4)     # EMA on per-frame translational flow
        # Projection axis kind: False=axial (along heading, fore/aft move),
        # True=lateral (heading+90, left/right move). The manager bridge sets
        # this (SetParameters) from the current move's axis flag BEFORE start.
        self.declare_parameter('distance_lateral', False)

        cam = str(self.get_parameter('camera').value or 'downward').strip()
        self._pool_depth = float(self.get_parameter('pool_depth_m').value)
        self._f_px       = float(self.get_parameter('camera_focal_px').value)
        self._rsx        = float(self.get_parameter('rot_sign_x').value)
        self._rsy        = float(self.get_parameter('rot_sign_y').value)
        self._alpha      = float(self.get_parameter('flow_smooth_alpha').value)
        self._cam        = cam

        ns = f'/duburi/vision/{cam}'
        img_qos = QoSProfile(depth=2, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, f'{ns}/image_raw', self._on_image, img_qos)
        self.create_subscription(Vector3Stamped, '/duburi/imu_rates', self._on_rates, 50)
        self.create_subscription(DuburiState, '/duburi/state', self._on_state, 10)

        self._pub_dist  = self.create_publisher(Float32, f'{ns}/distance_traveled', 10)
        self._pub_debug = self.create_publisher(Float32MultiArray, f'{ns}/distance_debug', 10)
        self.create_service(SetBool, f'{ns}/distance_control', self._on_control)

        self._bridge = CvBridge()
        self._acc    = DistanceAccumulator()
        # timestamped (stamp_s, pitch_rate, roll_rate) ring for interpolation.
        self._rate_buf: deque = deque(maxlen=64)
        self._depth_m: float | None = None
        self._yaw_deg: float | None = None
        self._last_height: float | None = None

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

    # ── control service ─────────────────────────────────────────────────────
    def _on_control(self, req, resp):
        if req.data:
            # start: latch axis from current yaw. axis kind (axial/lateral) is
            # carried in the request message field via the manager bridge -- here
            # SetBool only toggles; the axis-kind default is axial. (The manager
            # passes lateral by pre-setting a param; see _lateral below.)
            yaw = self._yaw_deg if self._yaw_deg is not None else 0.0
            lateral = bool(self.get_parameter('distance_lateral').value)
            self._acc.start(np.radians(yaw), lateral)
            self._reset_lk()
            resp.message = (f'distance ACTIVE axis_yaw={yaw:.1f} '
                            f'{"lateral" if lateral else "axial"}')
            self.get_logger().info(f'[DIST ] {resp.message}')
        else:
            d = self._acc.stop()
            resp.message = f'distance STOPPED total={d:.3f}m'
            self.get_logger().info(f'[DIST ] {resp.message}')
        resp.success = True
        return resp

    # ── inputs ──────────────────────────────────────────────────────────────
    def _on_rates(self, msg: Vector3Stamped) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # x=pitch_rate, y=roll_rate (see manager _imu_rates_tick).
        self._rate_buf.append((t, float(msg.vector.x), float(msg.vector.y)))

    def _on_state(self, msg: DuburiState) -> None:
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
        self._acc.add(self._flow_ema, self._last_height or 0.0, self._f_px)

    # ── outputs ─────────────────────────────────────────────────────────────
    def _publish(self, dist: float, height, n_tracks: int) -> None:
        m = Float32(); m.data = float(dist)
        self._pub_dist.publish(m)
        dbg = Float32MultiArray()
        dbg.data = [float(dist), float(height or 0.0), float(n_tracks),
                    1.0 if self._acc.active else 0.0]
        self._pub_debug.publish(dbg)
        now = time.monotonic()
        if now - self._last_log >= 1.0 and self._acc.active:
            self._last_log = now
            self.get_logger().info(
                f'[DIST ] d={dist:+.3f}m  height={height or 0.0:.2f}m  '
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

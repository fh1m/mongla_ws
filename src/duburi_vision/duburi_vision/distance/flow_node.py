#!/usr/bin/env python3
"""flow_node -- the bottom camera AS A VELOCITY SENSOR. Our DVL substitute.

This replaces `distance_estimation_node` and folds the two flow paths that had
grown apart: the one that was WIRED but had no refusals (`distance_traveled`,
a scalar integral) and the one that had every refusal and NO CONSUMERS
(`flow_velocity`). Neither was usable alone.

⛔ WHY VELOCITY IS THE PRIMARY OUTPUT AND DISTANCE IS DERIVED FROM IT.
An accumulated distance grows whether the hull tracks its heading or drifts
sideways, so it cannot answer the question every station-keeping task actually
asks -- "am I moving, which way, and do I still know where I am". It is also an
INTEGRAL: its error grows without bound and a filter cannot consume it, because
fusion needs a quantity with a stationary error rather than a running sum of
one. Velocity is that quantity. `distance_traveled` survives here for the
existing `calc_distance` consumers, but it is now integrated from GATED
velocity -- so it inherits every refusal instead of quietly accumulating noise.

THE GEOMETRY, and the asymmetry the whole design turns on:

    TRANSLATION   dx_px = f * (v / h) * dt      <- scales with range h
    ROTATION      dx_px = f * omega * dt        <- INDEPENDENT of range

Rotation must therefore be removed BEFORE scaling by `h`; scaling a rotation
term by range is meaningless. And the two are separable only because `omega` is
measured independently -- the gyro is not a refinement here, it is the thing
that makes translation recoverable at all.

    v = h * (flow_px/dt - f*omega) / f

⛔ WHAT WE PUBLISH WHEN WE DO NOT KNOW. `quality` follows the MAVLink
`OPTICAL_FLOW_RAD` convention where **0 means no valid flow**, and a refused
interval publishes quality 0 and NO velocity. It never publishes 0 m/s for a
measurement it could not make: a confident zero is a measurement, and an
estimator cannot tell it from a real standstill.
"""

import math
import sys
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray, String, UInt8
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped

from duburi_interfaces.msg import DuburiState
from duburi_vision.distance.flow_math import (
    DistanceAccumulator, flow_dispersion, height_above_floor, interp_rate,
    robust_flow,
)
from duburi_vision.distance.flow_velocity import flow_velocity

# Shi-Tomasi + LK. Bucketing is applied on top of goodFeaturesToTrack so the
# corners are spread across the frame rather than clustered on the one bright
# patch of grout -- a clustered set makes a rotation and a translation look
# alike, and a pool floor is exactly the repetitive lattice where that bites.
_FEATURE_PARAMS = dict(maxCorners=160, qualityLevel=0.01, minDistance=8,
                       blockSize=7)
_LK_PARAMS = dict(winSize=(21, 21), maxLevel=3,
                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                            30, 0.01))
_MIN_TRACKS = 6
_RESEED_EVERY = 10

# The FOCAL LENGTHS ARE MEASURED, and there are two of them because a flat port
# is a lens. Round 25: 63.8 deg in air / 46.7 deg in water, +-0.7, 25 views,
# calibrateCameraRO, held-out validated with an external tape check. At 640 px
# that is 514 and 741 -- a ratio of 1.44, where the flat-port literature says
# 25-33 %. The discrepancy is expected (port thickness and geometry) and the
# MEASUREMENT wins; the literature only explains why the two differ.
#
# Using the air number underwater is the single most dangerous mistake
# available here: a clean 44 % multiplier on every velocity, invisible in every
# plot, integrated by everything downstream. So `medium` is an explicit
# parameter with no safe default -- it is printed at startup.
_F_AIR_PX = 513.94
_F_WATER_PX = 741.0

# ⛔ THE DE-ROTATION GAINS DEFAULT TO THE CANONICAL -1.0, DELIBERATELY.
#
# The bench rig measured -1.150 (dx<-gx) and -1.095 (dy<-gy), cond(MtM) 27.7,
# and cut false velocity under pure rotation by 90 %. But rotation about the
# lens gives EXACTLY f*omega*dt, so a 12 % excess needs an owner and does not
# have one: a pivot lever arm was tried and its sign came out WRONG, an f of
# 577 contradicts a stronger calibration, and a dt bias would produce the same
# number. measured-bars.md section 12 says it plainly -- "the axis mapping and
# the method carry over; THIS GAIN DOES NOT, whatever its cause. Re-derive it
# on the hull."
#
# So the canonical 1.0 is the defensible default and the bench values are
# available as parameters. An unexplained 12 % correction applied to the wrong
# rig is a bad correction, and a bad correction is worse than none -- this
# session already watched de-rotation with an unvalidated mapping destroy
# 35.7 cm of real travel.
_GYRO_GAIN_DEFAULT = -1.0


class FlowVelocityNode(Node):
    def __init__(self):
        super().__init__('duburi_flow_velocity')

        self.declare_parameter('camera', 'downward')
        # NaN, not 4.0. A default pool depth is a scale factor nobody sets
        # deliberately multiplying every output -- the same shape as the
        # 50-vs-72 cm height error that produced an 83 % scale gap on the
        # bench. Absence must refuse, not guess.
        self.declare_parameter('pool_depth_m', float('nan'))
        self.declare_parameter('medium', 'water')          # 'air' | 'water'
        self.declare_parameter('focal_air_px', _F_AIR_PX)
        self.declare_parameter('focal_water_px', _F_WATER_PX)
        self.declare_parameter('gyro_gain_x', _GYRO_GAIN_DEFAULT)
        self.declare_parameter('gyro_gain_y', _GYRO_GAIN_DEFAULT)
        self.declare_parameter('rot_fraction_max', 0.80)
        self.declare_parameter('min_net_flow_px', 0.5)
        self.declare_parameter('max_dispersion_ratio', 5.0)
        self.declare_parameter('grid_buckets', 4)

        cam = str(self.get_parameter('camera').value or 'downward').strip()
        self._cam = cam
        self._pool_depth = float(self.get_parameter('pool_depth_m').value)
        self._medium = str(self.get_parameter('medium').value or 'water').lower()
        f_air = float(self.get_parameter('focal_air_px').value)
        f_water = float(self.get_parameter('focal_water_px').value)
        self._f_px = f_water if self._medium == 'water' else f_air
        self._gx = float(self.get_parameter('gyro_gain_x').value)
        self._gy = float(self.get_parameter('gyro_gain_y').value)
        self._rot_max = float(self.get_parameter('rot_fraction_max').value)
        self._min_flow = float(self.get_parameter('min_net_flow_px').value)
        self._max_disp = float(self.get_parameter('max_dispersion_ratio').value)
        self._buckets = max(1, int(self.get_parameter('grid_buckets').value))

        ns = f'/duburi/vision/{cam}'
        img_qos = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, f'{ns}/image_raw', self._on_image,
                                 img_qos)
        self.create_subscription(Vector3Stamped, '/duburi/imu_rates',
                                 self._on_rates, 50)
        self.create_subscription(DuburiState, '/duburi/state', self._on_state,
                                 10)
        ctrl_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, f'{ns}/distance_control',
                                 self._on_control, ctrl_qos)

        self._pub_vel = self.create_publisher(
            TwistWithCovarianceStamped, f'{ns}/velocity', 10)
        self._pub_quality = self.create_publisher(UInt8, f'{ns}/flow_quality', 10)
        self._pub_dist = self.create_publisher(Float32, f'{ns}/distance_traveled', 10)
        self._pub_debug = self.create_publisher(Float32MultiArray,
                                                f'{ns}/distance_debug', 10)

        self._bridge = CvBridge()
        self._acc = DistanceAccumulator()
        self._rate_buf: deque = deque(maxlen=128)
        self._depth_m = None
        self._yaw_deg = None
        self._last_height = None

        self._prev_gray = None
        self._prev_pts = None
        self._prev_t = None
        self._frame_i = 0
        self._n_tracks = 0
        self._last_quality = 0
        self._last_reason = 'no frames yet'
        self._last_log = time.monotonic()
        self._n_ok = 0
        self._n_refused = 0

        # THE FRAME PATH IS A ONE-DEEP MAILBOX AND A WORKER THREAD, not the
        # executor. The bottom camera runs at 211 Hz and one LK interval costs
        # ~8 ms on one core, so the node cannot and should not process every
        # frame -- it must process the NEWEST one and drop the rest. A queue
        # would hand the worker a fossil record; this hands it the latest
        # frame and reports how many it skipped.
        #
        # Threading is worth it because the work genuinely overlaps: OpenCV
        # releases the GIL for the duration of goodFeaturesToTrack and
        # calcOpticalFlowPyrLK, so LK runs on another core while this process
        # keeps servicing callbacks.
        self._slot = None                       # (gray, capture_t, seq)
        self._slot_lock = threading.Lock()
        self._slot_evt = threading.Event()
        self._dropped = 0
        self._seq = 0
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._flow_loop, daemon=True,
                                        name='flow')
        self._worker.start()

        ok, why = self._scale_ready()
        self.get_logger().info(
            f'[FLOW ] camera={cam!r} medium={self._medium!r} '
            f'f={self._f_px:.1f}px (air {f_air:.1f} / water {f_water:.1f})  '
            f'pool_depth={self._pool_depth:.2f}m  '
            f'gyro_gain=({self._gx:+.3f},{self._gy:+.3f})')
        if not ok:
            # Loud, and it refuses rather than running: the two scale terms are
            # clean multipliers on every number this node emits.
            self.get_logger().error(
                f'[FLOW ] VELOCITY PATH DISABLED -- {why}. Set it explicitly '
                f'(-p pool_depth_m:=<metres> -p medium:=air|water). A default '
                f'here would turn an unknown SCALE into a confident wrong '
                f'speed, which is invisible in every plot downstream.')

    # ── scale sanity ────────────────────────────────────────────────────────
    def _scale_ready(self):
        """Both scale terms known? Returns (ok, why_not)."""
        if not math.isfinite(self._pool_depth) or self._pool_depth <= 0.0:
            return False, 'pool_depth_m was never set'
        if self._medium not in ('air', 'water'):
            return False, f'medium={self._medium!r} is not air|water'
        if self._f_px <= 0.0:
            return False, 'focal length is not positive'
        return True, ''

    # ── inputs ──────────────────────────────────────────────────────────────
    def _on_rates(self, msg: Vector3Stamped) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._rate_buf.append((t, float(msg.vector.x), float(msg.vector.y)))

    def _on_state(self, msg: DuburiState) -> None:
        if not np.isnan(msg.depth_m):
            self._depth_m = float(msg.depth_m)
        if not np.isnan(msg.yaw_deg):
            self._yaw_deg = float(msg.yaw_deg)

    def _on_control(self, msg: String) -> None:
        cmd = str(msg.data or '').strip().lower()
        if cmd.startswith('start'):
            lateral = cmd.endswith('lateral')
            yaw = self._yaw_deg if self._yaw_deg is not None else 0.0
            self._acc.start(math.radians(yaw), lateral)
            self._reset_lk()
            self.get_logger().info(
                f'[FLOW ] ACTIVE axis_yaw={yaw:.1f} '
                f'{"lateral" if lateral else "axial"}')
        elif cmd == 'stop':
            d = self._acc.stop()
            self._publish_distance(d)
            self.get_logger().info(
                f'[FLOW ] STOPPED total={d:+.3f}m  '
                f'({self._n_ok} intervals used, {self._n_refused} refused)')

    def _reset_lk(self) -> None:
        self._prev_gray = None
        self._prev_pts = None
        self._prev_t = None
        self._n_ok = 0
        self._n_refused = 0

    # ── capture -> mailbox (callback thread, must never block) ──────────────
    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'[FLOW ] cv_bridge decode failed: {exc!r}')
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # THE CAPTURE STAMP, never `now()`. dt is the denominator of every
        # velocity here, and re-stamping on arrival makes it the scheduler's
        # jitter rather than the shutter interval. This stack has shipped that
        # bug three times.
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t <= 0.0:
            t = time.monotonic()
        with self._slot_lock:
            if self._slot is not None:
                self._dropped += 1
            self._seq += 1
            self._slot = (gray, t, self._seq)
        self._slot_evt.set()

    # ── worker ──────────────────────────────────────────────────────────────
    def _flow_loop(self) -> None:
        while not self._stop.is_set():
            if not self._slot_evt.wait(timeout=0.25):
                continue
            with self._slot_lock:
                item = self._slot
                self._slot = None
                self._slot_evt.clear()
            if item is None:
                continue
            try:
                self._process(*item)
            except Exception as exc:            # never let the thread die
                self.get_logger().error(f'[FLOW ] worker: {exc!r}')

    def _bucketed_corners(self, gray):
        """Shi-Tomasi corners spread over a grid, not clustered.

        A pool floor is a repetitive lattice, and corners that all land on one
        patch of it make a rotation and a translation look alike -- the two are
        separated by how the flow field VARIES across the frame, so a clustered
        set throws that away. Documented to improve ego-motion accuracy and to
        help precisely with repetitive patterns.
        """
        h, w = gray.shape[:2]
        n = self._buckets
        if n <= 1:
            return cv2.goodFeaturesToTrack(gray, mask=None, **_FEATURE_PARAMS)
        per = max(4, _FEATURE_PARAMS['maxCorners'] // (n * n))
        params = dict(_FEATURE_PARAMS, maxCorners=per)
        out = []
        for iy in range(n):
            for ix in range(n):
                y0, y1 = iy * h // n, (iy + 1) * h // n
                x0, x1 = ix * w // n, (ix + 1) * w // n
                sub = gray[y0:y1, x0:x1]
                if sub.size == 0:
                    continue
                pts = cv2.goodFeaturesToTrack(sub, mask=None, **params)
                if pts is None:
                    continue
                pts = pts.reshape(-1, 2) + np.array([x0, y0], dtype=np.float32)
                out.append(pts)
        if not out:
            return None
        return np.concatenate(out).reshape(-1, 1, 2).astype(np.float32)

    def _process(self, gray, t, seq) -> None:
        self._frame_i += 1
        if (self._prev_gray is None or self._prev_pts is None
                or len(self._prev_pts) < _MIN_TRACKS
                or self._frame_i % _RESEED_EVERY == 0):
            self._prev_pts = self._bucketed_corners(gray)
            self._prev_gray = gray
            self._prev_t = t
            return

        next_pts, status, _err = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, self._prev_pts, None, **_LK_PARAMS)
        dt = t - (self._prev_t if self._prev_t is not None else t)

        flow = robust_flow(self._prev_pts, next_pts, status,
                           min_tracks=_MIN_TRACKS)
        disp = flow_dispersion(self._prev_pts, next_pts, status)
        n_used = 0
        if next_pts is not None and status is not None:
            st = np.asarray(status).reshape(-1).astype(bool)
            n_used = int(st.sum())
            self._prev_pts = (next_pts[st].reshape(-1, 1, 2)
                              if n_used >= _MIN_TRACKS
                              else self._bucketed_corners(gray))
        else:
            self._prev_pts = self._bucketed_corners(gray)
        self._n_tracks = n_used
        self._prev_gray = gray
        self._prev_t = t

        if flow is None or dt <= 0.0:
            self._refuse('LK produced no usable flow' if flow is None
                         else 'non-positive dt')
            return
        self._evaluate(flow, disp, dt, t, n_used)

    def _evaluate(self, flow, disp, dt, t, n_used) -> None:
        ok, why = self._scale_ready()
        if not ok:
            self._refuse(why)
            return

        height = None
        if self._depth_m is not None:
            height = height_above_floor(self._pool_depth, self._depth_m)
        if height is not None:
            self._last_height = height
        h = self._last_height
        if h is None:
            self._refuse('no depth yet, so no height above the floor')
            return

        rates = interp_rate(self._rate_buf, t - dt * 0.5)
        pitch_rate = roll_rate = 0.0
        if rates is not None:
            pitch_rate, roll_rate = rates
        else:
            # Uncompensated flow is not "slightly worse", it is a different
            # quantity: at 0.6 rad/s the rotational term alone is ~200 px.
            self._refuse('no gyro sample for this interval')
            return

        # The gains carry the sign, so the residual is measured minus fitted.
        v = flow_velocity(
            flow[0], flow[1], dt,
            f_px=self._f_px, height_m=h,
            pitch_rate=-self._gy * pitch_rate,
            roll_rate=-self._gx * roll_rate,
            dispersion_px=disp,
            rot_fraction_max=self._rot_max,
            min_net_flow_px=self._min_flow,
            max_dispersion_ratio=self._max_disp)

        if not v.ok:
            self._refuse(v.reason)
            return

        self._n_ok += 1
        quality = self._quality(v, n_used, disp)
        self._last_quality = quality
        self._last_reason = 'ok'
        self._publish_velocity(v, t, n_used, disp, h, dt, quality)

        # Distance is now the INTEGRAL OF GATED VELOCITY. Every refusal above
        # is displacement this never sees -- which is the point: the old path
        # accumulated whatever LK returned, including a covered lens.
        if self._acc.active:
            yaw = math.radians(self._yaw_deg if self._yaw_deg is not None
                               else 0.0)
            self._acc.add_body_velocity(v.vx, v.vy, yaw, dt)
        self._publish_distance(self._acc.distance_m)

    def _quality(self, v, n_used, disp) -> int:
        """0-255, MAVLink OPTICAL_FLOW_RAD convention: 0 = NO VALID FLOW.

        Built from the three things that actually degrade a flow fix, so a
        consumer can gate on one number instead of re-deriving them:
        how many points survived, how much they disagreed, and how much of the
        flow was rotation (the regime where translation is a small difference
        of two large numbers).
        """
        n_term = min(1.0, n_used / 40.0)
        d_term = 1.0 - min(1.0, (disp or 0.0) / max(v.net_flow_px, 1e-6)
                           / max(self._max_disp, 1e-6))
        r_term = 1.0 - min(1.0, v.rot_fraction / max(self._rot_max, 1e-6))
        q = int(round(255.0 * max(0.0, n_term * 0.4 + d_term * 0.3
                                  + r_term * 0.3)))
        # 0 is reserved for "no valid flow"; a poor but real fix floors at 1.
        return max(1, min(255, q))

    def _refuse(self, reason: str) -> None:
        self._n_refused += 1
        self._last_quality = 0
        self._last_reason = reason
        q = UInt8()
        q.data = 0
        self._pub_quality.publish(q)
        self._publish_distance(self._acc.distance_m)
        self._maybe_log()

    # ── outputs ─────────────────────────────────────────────────────────────
    def _publish_velocity(self, v, t, n_used, disp, h, dt, quality) -> None:
        m = TwistWithCovarianceStamped()
        m.header.stamp.sec = int(t)
        m.header.stamp.nanosec = int((t - int(t)) * 1e9)
        m.header.frame_id = f'{self._cam}_cam'
        m.twist.twist.linear.x = float(v.vx)
        m.twist.twist.linear.y = float(v.vy)

        # COVARIANCE FROM THE DATA, not a constant. velocity = scale * mean
        # flow, so var(v) = scale^2 * var(flow) / N -- the standard error of
        # the mean, which is the honest statement of how well N noisy points
        # pinned one displacement. A constant here would tell a filter the
        # estimate is equally good on a textured floor and a bare one.
        scale = h / (self._f_px * dt)
        sigma_px = (disp if disp is not None else 1.0) / math.sqrt(
            max(n_used, 1))
        var = (scale * sigma_px) ** 2
        m.twist.covariance[0] = var          # vx
        m.twist.covariance[7] = var          # vy
        for i in (14, 21, 28, 35):           # z / rpy unobserved here
            m.twist.covariance[i] = -1.0
        self._pub_vel.publish(m)

        qm = UInt8()
        qm.data = int(quality)
        self._pub_quality.publish(qm)
        self._maybe_log(v, n_used, h)

    def _publish_distance(self, dist: float) -> None:
        m = Float32()
        m.data = float(dist)
        self._pub_dist.publish(m)
        dbg = Float32MultiArray()
        dbg.data = [float(dist), float(self._last_height or 0.0),
                    float(self._n_tracks), 1.0 if self._acc.active else 0.0,
                    float(self._last_quality)]
        self._pub_debug.publish(dbg)

    def _maybe_log(self, v=None, n_used=0, h=None) -> None:
        now = time.monotonic()
        if now - self._last_log < 1.0:
            return
        self._last_log = now
        with self._slot_lock:
            dropped, self._dropped = self._dropped, 0
        if v is not None and v.ok:
            self.get_logger().info(
                f'[FLOW ] v=({v.vx:+.3f},{v.vy:+.3f})m/s  q={self._last_quality} '
                f'h={h:.2f}m tracks={n_used} rot={v.rot_fraction:.2f}  '
                f'd={self._acc.distance_m:+.3f}m  skipped={dropped}')
        else:
            self.get_logger().info(
                f'[FLOW ] REFUSING: {self._last_reason}  '
                f'(q=0, {self._n_refused} refused / {self._n_ok} used)  '
                f'skipped={dropped}')

    def destroy_node(self):
        self._stop.set()
        self._slot_evt.set()
        try:
            self._worker.join(timeout=1.0)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FlowVelocityNode()
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

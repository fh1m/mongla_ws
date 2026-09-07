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

from typing import Optional

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
from duburi_vision.distance.flow_timing import (
    TimeOffset, exposure_offset_s, interval_midpoint,
)
from duburi_vision.distance.flow_math import (
    DistanceAccumulator, HeightFromDivergence, Intrinsics, detect_corners,
    flow_dispersion, forward_backward_error,
    RefractiveRectifier, height_above_floor, integrate_rate, interp_rate,
    robust_flow,
    solve_planar_motion,
)
from duburi_vision.distance.flow_velocity import flow_velocity

# Shi-Tomasi + LK. Bucketing is applied on top of goodFeaturesToTrack so the
# corners are spread across the frame rather than clustered on the one bright
# patch of grout -- a clustered set makes a rotation and a translation look
# alike, and a pool floor is exactly the repetitive lattice where that bites.
_FEATURE_PARAMS = dict(maxCorners=160, qualityLevel=0.01, minDistance=8,
                       blockSize=7)
# WINDOW 31, MEASURED. On this camera over a real ~16-frame baseline, with
# 172 candidate corners:
#
#     window   fb<=1px   RANSAC inliers   residual   ms
#       15       128          127          0.54     3.7
#       21       146          140          0.40     5.0
#       31       153          150          0.40     6.8
#       41       164          165          0.29     8.8
#
# A bigger window carries more texture, which is what makes a match unique on
# a blurry or low-contrast floor -- and blur is what we measured this camera to
# have (sharpness 70 against 321 and 1180 on archived competition clips). 41
# is the best of the four and 31 is taken instead only because LK runs on
# every arriving frame for the ripeness check: at ~90 fps that is 0.6 of a core
# at 31 and 0.8 at 41. Revisit if the frame path ever gets cheaper.
#
# maxLevel 4 measured IDENTICAL to 3 on every window, so it stays at 3.
_LK_PARAMS = dict(winSize=(31, 31), maxLevel=3,
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
        # Correct the FLAT PORT instead of averaging over it. Default ON in
        # water: a single f_water is exact only at the radius it was fitted
        # at, and the residual is a 1-2.8 % ANISOTROPIC scale error -- larger
        # than the whole error budget for a pool leg. Verified to recover the
        # true velocity to 0.000 % against forward-simulated port physics.
        self.declare_parameter('refractive_rectify', True)
        self.declare_parameter('water_refractive_index', 1.333)
        self.declare_parameter('min_net_flow_px', 0.5)
        self.declare_parameter('max_dispersion_ratio', 5.0)
        self.declare_parameter('grid_buckets', 4)
        self.declare_parameter('want_points', 80)
        # ADAPTIVE KEYFRAME BASELINE. Emit a velocity once this much image
        # displacement has accumulated against the anchor, rather than once
        # per frame. See `_track` for why, and measured-bars for the numbers.
        self.declare_parameter('target_px', 8.0)
        self.declare_parameter('max_px', 25.0)
        self.declare_parameter('max_baseline_s', 0.75)
        # RE-ANCHOR ON ACCUMULATED ROTATION, not only on displacement and
        # time. Measured on this camera's own floor texture (see below).
        self.declare_parameter('max_rotation_deg', 6.0)
        # PLANAR RIGID FIT. A downward camera on a flat floor sees ONE body
        # move, so all the points measure the same four numbers. Measured
        # against a median, truth exact: with 1.5 deg of rotation in an
        # interval the median reports 4.79 px of translation that did not
        # happen, the fit 0.055 px.
        self.declare_parameter('use_planar_fit', True)
        self.declare_parameter('ransac_px', 2.0)
        # Forward-backward rejection. LK reports convergence, not correctness:
        # over a tiled pool floor it converges confidently one tile off.
        self.declare_parameter('fb_reject_px', 2.0)
        # Sign relating IMAGE rotation to the vehicle's yaw. MOUNT-SPECIFIC:
        # it depends which way the camera is clocked in the hull, so it is
        # calibrated once against the gyro and then monitored, never assumed.
        self.declare_parameter('yaw_image_sign', -1.0)
        self.declare_parameter('yaw_cross_check_tol', 0.25)
        # THE CALIBRATION, not a focal length. fx != fy, the principal point is
        # not the frame centre, and the lens distorts -- and all three are
        # axis-dependent, which is why they showed up as a 3 % asymmetry
        # between forward/back and lateral on real 30 cm slides.
        self.declare_parameter('calibration', '')
        self.declare_parameter('undistort', True)
        # TIMING. Each of these is larger than the residual the estimator
        # removes, so they are corrected deterministically first.
        self.declare_parameter('exposure_us', 0.0)      # V4L2 100us units
        self.declare_parameter('stamp_at_midpoint', True)
        self.declare_parameter('estimate_time_offset', True)
        self.declare_parameter('time_offset_s', 0.0)
        # A BOUND ON td, because it is no longer only a diagnostic. Once td
        # shifts every velocity stamp, a spurious correlation peak actively
        # CORRUPTS the output instead of logging a warning -- and the slew
        # makes a wrong value persist across windows. The plausible range is
        # bounded by physics we have measured: transport jitter p2p 35 ms,
        # half-exposure 7.85 ms, and one adaptive baseline 750 ms. Anything
        # past 150 ms is not a link delay, it is a bad peak.
        self.declare_parameter('time_offset_max_s', 0.15)
        self.declare_parameter('time_offset_min_quality', 0.5)

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
        self._want_pts = int(self.get_parameter('want_points').value)
        self._target_px = float(self.get_parameter('target_px').value)
        self._max_px = float(self.get_parameter('max_px').value)
        self._max_baseline = float(self.get_parameter('max_baseline_s').value)
        self._max_rot_rad = math.radians(
            float(self.get_parameter('max_rotation_deg').value))
        self._n_rot_anchor = 0
        self._use_planar = bool(self.get_parameter('use_planar_fit').value)
        self._ransac_px = float(self.get_parameter('ransac_px').value)
        self._fb_px = float(self.get_parameter('fb_reject_px').value)
        self._yaw_sign = float(self.get_parameter('yaw_image_sign').value)
        self._n_water = float(
            self.get_parameter('water_refractive_index').value)
        self._want_refract = bool(
            self.get_parameter('refractive_rectify').value)
        self._refract = None
        # The focal length the GYRO GUESS uses. It stays in RAW pixel space
        # because that is where LK runs, so it must NOT switch to f_ref: the
        # guess is an initial condition, refined by LK, and at the frame edge
        # -- where the predicted shift is largest and the guess matters most
        # -- the raw water focal length is the better approximation.
        self._f_guess_px = self._f_px
        self._yaw_tol = float(self.get_parameter('yaw_cross_check_tol').value)
        self._undistort = bool(self.get_parameter('undistort').value)
        self._intr = None
        self._exposure_units = float(self.get_parameter('exposure_us').value)
        self._mid = bool(self.get_parameter('stamp_at_midpoint').value)
        self._td_estimate = bool(self.get_parameter('estimate_time_offset').value)
        self._td = float(self.get_parameter('time_offset_s').value)
        self._td_est = TimeOffset(max_lag_s=0.20)
        self._td_last_fit = 0.0
        self._td_n = 0
        self._td_max = abs(float(self.get_parameter('time_offset_max_s').value))
        self._td_min_q = float(
            self.get_parameter('time_offset_min_quality').value)
        self._td_rejected = 0
        cal = str(self.get_parameter('calibration').value or '').strip()
        if cal:
            try:
                self._intr = Intrinsics.from_json(cal, 640, 360)
                self._f_guess_px = self._f_px
                if self._medium != 'water':
                    self._f_px = self._intr.fx
                elif self._want_refract:
                    # ⛔ THE INTRINSICS STAY IN AIR, DELIBERATELY. The lens and
                    # its distortion coefficients are AIR-SIDE properties,
                    # calibrated with the camera dry; the water is in front of
                    # the port, not inside the lens. Scaling K to f_water
                    # before undistortPoints -- which the previous code did --
                    # divides by an fx 1.44x too large, so the radial model is
                    # evaluated at 1/1.44 of the true normalised radius and
                    # applies only **45 % of the needed correction**, leaving
                    # up to 4.16 px at the frame edge. That is half an
                    # adaptive baseline of pure error, and it exists ONLY in
                    # water mode, which has never run.
                    #
                    # Correct order: undistort the LENS in air, rectify the
                    # PORT, then measure with f_ref.
                    self._refract = RefractiveRectifier(
                        self._intr.fx, self._intr.fy,
                        self._intr.cx, self._intr.cy, n=self._n_water)
                    self._f_px = self._refract.f_ref
                    self._f_guess_px = f_water
                else:
                    # Rectification off: keep the old single-focal-length
                    # behaviour so the two paths can be A/B'd, and say what it
                    # costs rather than leaving it to be discovered.
                    k = f_water / self._intr.fx
                    self._intr.fx *= k
                    self._intr.fy *= k
                    self._f_px = self._intr.fx
                    self._f_guess_px = self._f_px
                    self.get_logger().warning(
                        '[FLOW ] refractive_rectify=false in WATER: a single '
                        'focal length is exact only at the radius it was '
                        'fitted at (1-2.8 % anisotropic scale error), and '
                        'undistortion runs with a water-scaled K that applies '
                        '~45 % of the lens correction. Both measured.')
            except Exception as exc:
                self.get_logger().error(
                    f'[FLOW ] calibration {cal!r} unreadable ({exc}); falling '
                    f'back to a single focal length and the frame centre, '
                    f'which measured a 3 % axis asymmetry on the bench')
        self._n_median_fallback = 0
        self._n_yaw_disagree = 0
        self._last_yaw_img = 0.0
        self._last_scale_rate = 0.0
        # Height measured by the CAMERA, checked against the barometer path.
        self._h_optical = HeightFromDivergence()
        self._depth_hist = deque(maxlen=16)
        self._n_h_warn = 0

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
        self._yaw_rate_buf: deque = deque(maxlen=128)
        self._depth_m = None
        self._yaw_deg = None
        self._last_height = None

        # The ANCHOR, not the previous frame: flow is measured frame-to-
        # anchor and the anchor is replaced only when a measurement is emitted.
        self._anchor_gray = None
        self._anchor_pts = None
        self._anchor_t = None
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
        if self._intr is not None:
            self.get_logger().info(f'[FLOW ] {self._intr}')
        self.get_logger().info(
            f'[FLOW ] camera={cam!r} medium={self._medium!r} '
            f'f={self._f_px:.1f}px (air {f_air:.1f} / water {f_water:.1f})  '
            f'port={"RECTIFIED" if self._refract is not None else "single-f"}  '
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
        # x=pitch, y=roll, z=yaw (rad/s). The yaw channel was published and
        # read by nothing; it is what makes the image/gyro cross-check below
        # possible without adding a sensor.
        self._rate_buf.append((t, float(msg.vector.x), float(msg.vector.y)))
        self._yaw_rate_buf.append((t, float(msg.vector.z)))

    def _on_state(self, msg: DuburiState) -> None:
        if not np.isnan(msg.depth_m):
            self._depth_m = float(msg.depth_m)
            self._depth_hist.append((time.monotonic(), self._depth_m))
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
        self._anchor_gray = None
        self._anchor_pts = None
        self._anchor_t = None
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
        """Corners for the anchor: spread over a grid, quality bar adaptive.

        See `flow_math.detect_corners`. A fixed Shi-Tomasi threshold is a fixed
        assumption about the floor, and it fails quietly on the smooth ones --
        measured 150 points on a textured surface and 19 on a plain one, with
        no refusal from either, because 19 still clears the fit's 8-point
        minimum while making its residual far less meaningful.
        """
        return detect_corners(gray, want=self._want_pts,
                              max_corners=_FEATURE_PARAMS['maxCorners'],
                              min_distance=_FEATURE_PARAMS['minDistance'],
                              block=_FEATURE_PARAMS['blockSize'],
                              buckets=self._buckets)

    def _is_ripe(self, mag: float, n_used: int, dt: float,
                 rot_rad: float) -> bool:
        """Has this baseline accumulated enough to be worth estimating from?

        Five ways an interval becomes ripe, and the last one is new:
          - `target_px` of displacement: the signal we actually want
          - `max_px`: re-anchor before LK is asked to cross its own window
          - too few tracks: the anchor is dying, take what is left
          - `max_baseline_s`: a hard ceiling so a still hull still reports
          - **accumulated rotation**: see below

        ⛔ ROTATION IS NOT COVERED BY THE OTHERS, and assuming it was is the
        mistake this method exists to prevent. The tempting argument is that
        rotation inflates the median so displacement-ripeness fires anyway --
        round 38 measured a median inventing 4.79 px under 1.5 deg. **That
        does not hold here**: with this node's grid bucketing the corners are
        spread symmetrically about the principal point, and the
        component-wise median of a pure rotation is then ~0 -- measured
        **2.28 px at 6 deg**, against an 8 px floor. The protection that
        remains is REACTIVE: tracks die, `n_used` falls under `_MIN_TRACKS`,
        and ripeness fires having already spent the interval.

        Measured A/B over a simulated station-keep-while-yawing on real floor
        texture, 3 s runs: distance recovered is unchanged (98.7 -> 98.6 %,
        102.9 -> 102.1 %) and the MEASUREMENT RATE rises where it matters --
        19 -> 23 and 41 -> 45 emitted intervals at 0.638 and 1.128 rad/s,
        and exactly unchanged (7 -> 7, 34 -> 34) when not yawing. So this
        buys measurements during a yawing hold, not accuracy, and it is
        written down that way rather than sold as an accuracy fix.
        """
        if mag >= self._target_px or mag >= self._max_px:
            return True
        if n_used < _MIN_TRACKS or dt >= self._max_baseline:
            return True
        if rot_rad >= self._max_rot_rad:
            self._n_rot_anchor += 1
            return True
        return False

    def _rotation_since_anchor(self, t_now: float) -> float:
        """|image rotation| accumulated since the anchor, from the gyro.

        Yaw about the optical axis is what rotates a DOWNWARD image, so the
        yaw channel is the one that matters here -- the same channel the
        image/gyro cross-check already uses, and it was published and read by
        nothing before that.

        Returns 0.0 when there is no gyro for the interval. That is
        deliberate and is NOT an absence-is-zero mistake: with no rotation
        estimate the other ripeness criteria still apply, so the worst case is
        the behaviour we had before this criterion existed. Refusing here
        instead would turn a missing IMU sample into a stalled anchor.
        """
        if not self._yaw_rate_buf:
            return 0.0
        dt = t_now - self._anchor_t
        if dt <= 0.0:
            return 0.0
        gz = interp_rate([(a, b, 0.0) for (a, b) in self._yaw_rate_buf],
                         t_now - dt * 0.5 - self._td)
        if gz is None:
            return 0.0
        return abs(gz[0]) * dt

    def _accept_td(self, got, quality):
        """Decide whether an estimated camera<->gyro offset may be used.

        td stopped being a diagnostic the moment it began shifting every
        velocity stamp: a spurious correlation peak now CORRUPTS the output
        instead of logging a warning, and the slew makes a wrong value
        persist across windows. Two things must hold.

        Quality, because the peak must actually be a peak. And a bound,
        because the plausible range is bounded by physics we have measured:
        transport jitter p2p 35 ms, half-exposure 7.85 ms, one adaptive
        baseline 750 ms. Past 150 ms it is a correlation artefact, not a
        link delay -- and we have no in-water measurement of the true value
        to sanity-check a large one against.

        Returns the offset to slew toward, or None to hold the current td.
        """
        if got is None:
            return None
        if abs(got) > self._td_max:
            self._td_rejected += 1
            if self._td_rejected % 10 == 1:
                self.get_logger().warning(
                    f'[FLOW ] REFUSING a time offset of {got * 1000:+.1f} ms '
                    f'-- beyond the {self._td_max * 1000:.0f} ms bound, so it '
                    f'is a correlation artefact, not a link delay. td stays '
                    f'at {self._td * 1000:+.2f} ms.')
            return None
        if quality < self._td_min_q:
            return None
        return got

    def _process(self, gray, t, seq) -> None:
        """Track against the ANCHOR; emit a velocity when it is worth one.

        ⛔ THE DEFECT THIS REPLACES, measured on this camera at h=0.72 m.
        `MIN_NET_FLOW_PX` is a per-interval PIXEL floor, so the SPEED it
        refuses scales with the frame rate: `min_px * h / (f * dt)`. Running
        flow at the camera's native 210 Hz therefore refuses everything below
        **0.147 m/s** -- and it refuses by publishing NO measurement, which
        every consumer reads as "not moving".

        A synthetic 30 cm translation over real texture, truth exact:

            speed     210 Hz      70 Hz      30 Hz      ADAPTIVE
            0.02      0.0 %       0.0 %      11.1 %     100.1 %
            0.05      0.0 %      65.7 %     100.0 %     100.2 %
            0.10      0.0 %     100.0 %     100.0 %     100.2 %
            0.80    100.3 %      99.0 %      97.8 %     100.5 %

        Zero of 1260 intervals survived a 30 cm move at 5 cm/s, reported as
        0.0 cm travelled. Station-keeping is the AUV's slowest and most common
        state, so a fixed high rate blinds the sensor in exactly the regime it
        exists for.

        LOWERING THE CONSTANT IS NOT THE FIX -- that trades a blind spot for
        integrated noise, and both are symptoms of measuring over a fixed TIME
        when the thing that matters is DISPLACEMENT. So the baseline stretches
        until there is something to measure: every emitted interval carries
        ~`target_px` of signal against ~0.06 px of noise at ANY speed, and the
        measurement RATE follows distance travelled rather than the clock.

        Two further properties fall out. Within a baseline the flow is
        frame-to-ANCHOR, so it does not chain per-frame noise the way
        frame-to-frame integration does. And `max_px` re-anchors before LK is
        asked to match across more displacement than its window can follow.
        """
        self._frame_i += 1
        if (self._anchor_gray is None or self._anchor_pts is None
                or len(self._anchor_pts) < _MIN_TRACKS):
            self._anchor(gray, t)
            return

        nxt, status, _err = cv2.calcOpticalFlowPyrLK(
            self._anchor_gray, gray, self._anchor_pts, None, **_LK_PARAMS)

        flow = robust_flow(self._anchor_pts, nxt, status,
                           min_tracks=_MIN_TRACKS)
        if flow is None:
            # The anchor is unusable; re-seed here rather than reporting a
            # refusal for every frame until something changes.
            self._anchor(gray, t)
            self._refuse('LK lost the anchor')
            return

        n_used = int(np.asarray(status).reshape(-1).astype(bool).sum())
        mag = math.hypot(flow[0], flow[1])
        dt = t - self._anchor_t
        # ⛔ ROTATION IS A RIPENESS CRITERION, and its absence was a hole.
        # Ripeness asked about DISPLACEMENT, track count and TIME -- never
        # rotation. The baseline stretches when the hull moves SLOWLY, which
        # is station-keeping, our most common state; so a hull holding
        # position while yawing accumulates the whole rotation inside ONE
        # interval. At 1.128 rad/s over the 0.75 s cap that is 48.5 deg, and
        # measured on this camera's own floor texture it leaves **0 of 192
        # points**. The node then refuses -- honest, and a blind sensor
        # exactly where the vehicle spends most of its time.
        #
        # Measured survival and translation error vs accumulated rotation
        # (real frames, forward-backward at 2 px applied):
        #     3 deg  82.5 %  0.014 px      10 deg  51.1 %  0.076 px
        #     6 deg  74.7 %  0.040 px      12 deg  37.4 %  0.191 px
        #     8 deg  60.5 %  0.091 px      20 deg  12.4 %  1.344 px
        # Graceful to ~10 deg, then a knee. 6 deg sits inside the graceful
        # region with margin, and re-anchoring costs one corner detection.
        #
        # SEEDING LK WITH THE GYRO WAS TRIED FIRST AND MEASURED TO DO NOTHING:
        # at our rates the displacement is already inside LK's basin (identical
        # results to three decimals), and at large rotation the extra points a
        # seed recovers FAIL forward-backward -- 0 -> 5 of 192 at 48 deg. The
        # points it wins back are not correct matches. Re-anchoring earlier is
        # the fix; a better initial guess is not.
        rot = self._rotation_since_anchor(t)
        ripe = self._is_ripe(mag, n_used, dt, rot)
        if not ripe:
            self._n_tracks = n_used
            return

        # FORWARD-BACKWARD, and ONLY NOW. It is a second full LK pass, so
        # running it on every arriving frame doubles the cost of the frame
        # path to decide something the ripeness check does not need -- the
        # ripeness check only asks "has enough displacement accumulated", and
        # a mistracked point cannot fake that at the scale that matters.
        # Culling belongs immediately before the estimate, which is here.
        #
        # It is the defence against the failure a pool invites: over a
        # repetitive tile lattice LK converges confidently one tile away and
        # reports status=1, returning a clean multiple of the tile pitch --
        # indistinguishable from a correct match by residual or status alone.
        if self._fb_px > 0.0 and nxt is not None and status is not None:
            fb = forward_backward_error(self._anchor_gray, gray,
                                        self._anchor_pts, nxt, _LK_PARAMS)
            if fb is not None:
                st = np.asarray(status).reshape(-1).astype(bool)
                st &= (fb <= self._fb_px)
                status = st.astype(np.uint8).reshape(-1, 1)
                n_used = int(st.sum())

        disp = flow_dispersion(self._anchor_pts, nxt, status)

        # THE RIGID FIT, with the median kept as a VISIBLE fallback. A floor
        # too bare to give 8 agreeing points cannot support a 4-DOF fit, and
        # in that regime a median of what little there is beats refusing
        # outright -- but the fallback is counted and logged, because a sensor
        # that silently degrades to its weaker estimator is one that reports
        # phantom translation under rotation without ever saying so.
        if self._use_planar:
            h_, w_ = gray.shape[:2]
            a_pts, n_pts = self._anchor_pts, nxt
            if self._intr is not None and self._undistort:
                # Remove the lens BEFORE fitting. Distortion scales a
                # displacement by a factor that depends on RADIUS, and a
                # 640x360 frame is 1.8x wider than tall -- so the horizontal
                # axis samples a radial range the vertical one never reaches.
                # Measured shift at the frame edge: 3.37 px in x, 1.62 px in y.
                a_pts = self._intr.undistort_points(self._anchor_pts)
                n_pts = self._intr.undistort_points(nxt)
            cx_ = self._intr.cx if self._intr is not None else w_ / 2.0
            cy_ = self._intr.cy if self._intr is not None else h_ / 2.0
            if self._refract is not None:
                # Rectify AFTER undistortion and BEFORE the fit. Order is
                # forced: undistort removes the LENS (an air-side property),
                # rectify removes the PORT (a water-side one), and the port
                # sees rays the lens has already been accounted for. The
                # principal point is a fixed point of the rectification, so
                # cx_/cy_ carry through unchanged.
                a_pts = self._refract.rectify(a_pts)
                n_pts = self._refract.rectify(n_pts)
            pm = solve_planar_motion(a_pts, n_pts, status, dt,
                                     cx=cx_, cy=cy_,
                                     ransac_px=self._ransac_px)
            if pm.ok:
                flow = (pm.dx_px, pm.dy_px)
                n_used = pm.n_inliers
                disp = pm.residual_px
                self._last_yaw_img = self._yaw_sign * pm.yaw_rate
                self._last_scale_rate = pm.scale_rate
            else:
                self._n_median_fallback += 1

        self._n_tracks = n_used
        self._anchor(gray, t)
        if dt > 0.0:
            self._evaluate(flow, disp, dt, t, n_used)
        else:
            self._refuse('non-positive baseline')

    def _anchor(self, gray, t) -> None:
        self._anchor_gray = gray
        self._anchor_pts = self._bucketed_corners(gray)
        self._anchor_t = t

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

        # The MEAN rate over the baseline, not a midpoint sample: with an
        # adaptive baseline dt reaches ~0.5 s at 2 cm/s, and de-rotation
        # subtracts `f * omega * dt`, so a midpoint that lands on the peak of
        # a swing scales that error by the whole interval.
        # ⛔ THE INTERVAL IS SHIFTED BY td BEFORE THE GYRO IS READ. td > 0
        # means the image timestamps are LATE, so an image stamped t shows the
        # world at t - td, and the gyro that belongs with it is the gyro from
        # then. Getting this sign wrong steers de-rotation the wrong way and
        # DOUBLES the residual rather than removing it, while returning an
        # entirely plausible number.
        rates = integrate_rate(self._rate_buf, t - dt - self._td, t - self._td)
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
            f_px=self._f_px,
            # After rectification BOTH focal lengths live in the rectified
            # frame. Passing the air-side fy against a rectified fx would
            # scale the two image axes by different models -- 516.93 vs
            # 685.08 -- which is a 25 % axis error, not a subtle one.
            fy_px=(self._refract.f_ref_y if self._refract is not None
                   else (self._intr.fy if self._intr is not None else None)),
            height_m=h,
            pitch_rate=-self._gy * pitch_rate,
            roll_rate=-self._gx * roll_rate,
            dispersion_px=disp,
            rot_fraction_max=self._rot_max,
            min_net_flow_px=self._min_flow,
            max_dispersion_ratio=self._max_disp)

        if not v.ok:
            self._refuse(v.reason)
            return

        self._cross_check_yaw(t, dt)
        self._cross_check_height()
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

    def _vz_down(self) -> Optional[float]:
        """Vertical speed, POSITIVE DOWNWARD, from the depth series.

        Depth is negative below the surface here, so descending makes it more
        negative and `-d(depth)/dt` is positive going down -- which is the sign
        that makes a descent GROW the image, matching the divergence.
        """
        if len(self._depth_hist) < 4:
            return None
        (t0, d0), (t1, d1) = self._depth_hist[0], self._depth_hist[-1]
        span = t1 - t0
        if span < 0.2:
            return None
        return -(d1 - d0) / span

    def _cross_check_height(self) -> None:
        """Height from the image, against the height from a typed pool depth.

        The divergence of the flow field is `-vz / h`, and the barometer gives
        `vz` independently -- so the camera can measure its own altitude, with
        no `pool_depth_m` in it. That matters because height is a clean
        multiplier on every velocity this node emits and `pool_depth_m` is the
        one input nobody measures carefully.

        Reports only. A disagreement does not say WHICH is wrong, and acting on
        it would be guessing.
        """
        vz = self._vz_down()
        if vz is None or abs(self._last_scale_rate) < 1e-9:
            return
        self._h_optical.add(self._last_scale_rate, vz)
        d = self._h_optical.disagreement(self._last_height)
        if d is not None and d > 0.20:
            self._n_h_warn += 1
            if self._n_h_warn % 20 == 1:
                self.get_logger().warning(
                    f'[FLOW ] HEIGHT DISAGREES by {d:.0%}: the image says '
                    f'{self._h_optical.height_m:.2f} m (from {self._h_optical.n_samples} '
                    f'samples of flow divergence vs barometer rate), the '
                    f'pool_depth path says {self._last_height:.2f} m. Height '
                    f'multiplies EVERY velocity here -- check pool_depth_m '
                    f'before trusting any distance.')

    def _cross_check_yaw(self, t: float, dt: float) -> None:
        """Image-derived yaw against the gyro's. Free, and always on.

        ⛔ WHY THIS IS WORTH ITS OWN METHOD. The planar fit measures the
        rotation of the floor in the image, and for a downward camera that IS
        the vehicle's yaw. The gyro measures the same quantity through
        completely different physics. Two independent sensors reading one
        number is the cheapest integrity monitor available to us, and it costs
        nothing extra: both values are already computed.

        It catches precisely the class of defect that cost this session four
        bench runs and four wrong answers -- an axis swapped, a sign flipped, a
        gain 12 % out. Every one of those produced a plausible number and no
        error, and every one would show here as a standing disagreement
        between two things that must agree.

        It only reports. A disagreement does not know WHICH source is wrong,
        so acting on it would be guessing; the honest response is to say so
        loudly and let the operator or a later gate decide.
        """
        if not self._yaw_rate_buf or dt <= 0.0:
            return
        gy = interp_rate([(a, b, 0.0) for (a, b) in self._yaw_rate_buf],
                         t - dt * 0.5)
        if gy is None:
            return
        gyro_yaw = gy[0]
        img_yaw = self._last_yaw_img
        # Only meaningful when something is actually rotating: below the gyro's
        # own noise the ratio of two near-zero numbers is noise, the same trap
        # that made a motionless bench read "rotation-dominated".
        # Feed the TIME-OFFSET estimator from the same two series. They are
        # already computed, already known to measure one quantity, and trace
        # correlation wants exactly this -- so td costs nothing extra.
        if self._td_estimate:
            self._td_est.add_image_yaw(t - dt * 0.5, img_yaw)
            self._td_est.add_gyro_yaw(t - dt * 0.5, gyro_yaw)
            now = time.monotonic()
            if now - self._td_last_fit >= 5.0:
                self._td_last_fit = now
                got = self._accept_td(
                    self._td_est.estimate(), self._td_est.quality)
                if got is not None:
                    self._td_n += 1
                    # Slew rather than jump: the offset is a property of the
                    # link, not of one window, and a step would move every
                    # subsequent velocity's stamp discontinuously.
                    self._td = 0.7 * self._td + 0.3 * got
                    if self._td_n % 6 == 1:
                        self.get_logger().info(
                            f'[FLOW ] camera<->gyro time offset '
                            f'{self._td * 1000:+.2f} ms (this window '
                            f'{got * 1000:+.2f}, quality '
                            f'{self._td_est.quality:.2f})')

        if abs(gyro_yaw) < 0.05 and abs(img_yaw) < 0.05:
            return
        denom = max(abs(gyro_yaw), abs(img_yaw), 1e-6)
        if abs(img_yaw - gyro_yaw) / denom > self._yaw_tol:
            self._n_yaw_disagree += 1
            if self._n_yaw_disagree % 20 == 1:
                self.get_logger().warning(
                    f'[FLOW ] yaw DISAGREES: image {img_yaw:+.3f} vs gyro '
                    f'{gyro_yaw:+.3f} rad/s ({self._n_yaw_disagree} times). '
                    f'Two independent measurements of one quantity should not '
                    f'differ by {self._yaw_tol:.0%}. Suspect the camera<->body '
                    f'axis mapping, yaw_image_sign, or a gyro scale -- not the '
                    f'flow itself.')

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
    def _stamp_for(self, t_end: float, dt: float) -> float:
        """When this velocity actually happened, in host time.

        ⛔ THREE CORRECTIONS, and the first is the biggest error in the whole
        pipeline:

        1. THE MIDPOINT. Flow measures DISPLACEMENT over an interval, so
           dividing by dt gives the AVERAGE velocity across it -- which belongs
           at the middle. PX4 defines its own flow delay parameter exactly so:
           "to the middle of the optical flow integration interval". Our
           baseline is ADAPTIVE and stretches to 0.75 s when the hull is slow,
           so stamping at the END is wrong by up to 375 ms, and worst precisely
           during station-keeping.

        2. HALF THE EXPOSURE. The standard image timestamp is mid-exposure.
           Ours reads 15.7 ms on AUTO exposure -- 7.85 ms, larger on its own
           than the 6 ms tolerance, and it MOVES with the light.

        3. td. Whatever fixed lag remains, from `TimeOffset`.
        """
        t = interval_midpoint(t_end - dt, t_end) if self._mid else t_end
        t -= exposure_offset_s(self._exposure_units)
        t -= self._td
        return t

    def _publish_velocity(self, v, t, n_used, disp, h, dt, quality) -> None:
        m = TwistWithCovarianceStamped()
        t = self._stamp_for(t, dt)
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
                f'h={h:.2f}m pts={n_used} rot={v.rot_fraction:.2f} '
                f'yaw_img={self._last_yaw_img:+.3f} '
                f'd={self._acc.distance_m:+.3f}m  skipped={dropped} '
                f'median_fallback={self._n_median_fallback}')
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

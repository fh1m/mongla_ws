#!/usr/bin/env python3
"""flow_node -- the bottom camera AS A VELOCITY SENSOR. Our DVL substitute.

Started by `flow:=true` in every vision launch (`vision_pi`, `vision`,
`vision_dual`). It replaced `distance_estimation_node` (in git history) and
folds the two flow paths that had grown apart: the one that was WIRED but had no refusals (`distance_traveled`,
a scalar integral) and the one that had every refusal and NO CONSUMERS
(`flow_velocity`). Neither was usable alone.

Why velocity is the primary output and distance is derived from it.
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

What we publish when we do not know. `quality` follows the MAVLink
`OPTICAL_FLOW_RAD` convention where **0 means no valid flow**, and a refused
interval publishes quality 0 and NO velocity. It never publishes 0 m/s for a
measurement it could not make: a confident zero is a measurement, and an
estimator cannot tell it from a real standstill.

THE FILES. `FlowVelocityNode` is one class in six files (plus its constants),
so each part can be read alone:

    flow_node.py       the node: its state, the flow loop, shutdown
    flow_frame.py      the per-frame pipeline: ripeness, tracking, evaluate
    flow_setup.py      parameters, intrinsics, the input callbacks
    flow_checks.py     every check that may refuse a frame
    flow_floor.py      what the floor says: tile-grid heading, lane lines
    flow_publish.py    every topic written, and the periodic log
    flow_constants.py  the measured constants, each in measured-bars.md
"""

import math
import sys
import threading
import time
from collections import deque

from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Float32, Float32MultiArray, String, UInt8
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped

from mongla_interfaces.msg import FloorHeading, MonglaState
from mongla_vision.flow.flow_timing import TimeOffset
from mongla_vision.flow.flow_math import (
    DistanceAccumulator,
    HeightFromDivergence,
)
from .flow_setup import FlowSetupMixin
from .flow_checks import FlowChecksMixin
from .flow_floor import FlowFloorMixin
from .flow_publish import FlowPublishMixin
from .flow_frame import FlowFrameMixin


class FlowVelocityNode(FlowFrameMixin, FlowSetupMixin, FlowChecksMixin, FlowFloorMixin,
                       FlowPublishMixin, Node):
    def __init__(self):
        super().__init__('mongla_flow_velocity')

        self._declare_parameters()

        cam = str(self.get_parameter('camera').value or 'downward').strip()
        self._cam = cam
        self._pool_depth = float(self.get_parameter('pool_depth_m').value)
        lo = float(self.get_parameter('pool_depth_min_m').value)
        hi = float(self.get_parameter('pool_depth_max_m').value)
        # The RMS of what the ASSUMED depth gets wrong, for a floor anywhere
        # in [lo, hi]: with a = lo - d and b = hi - d, uniform gives
        # sqrt((a^2 + ab + b^2) / 3). Not the range's own spread -- the typed
        # depth is often measured at one end (the dock), not the middle. Not
        # a measurement of this venue either: the honest size of "somewhere
        # in the range the venue states".
        a, b = lo - self._pool_depth, hi - self._pool_depth
        self._floor_sigma_m = (math.sqrt(max(0.0, (a * a + a * b + b * b) / 3.0))
                               if all(map(math.isfinite, (a, b))) else 0.0)
        # Lens below the baro port, body FRD metres, or None = not measured
        # (issue #27; see `flow_math.height_above_floor`).
        from mongla_localization import frames as _frames
        self._lens_below_baro = _frames.load().baro_to_downward_cam_z()
        self._medium = str(self.get_parameter('medium').value or 'water').lower()
        f_air = float(self.get_parameter('focal_air_px').value)
        f_water = float(self.get_parameter('focal_water_px').value)
        self._f_px = f_water if self._medium == 'water' else f_air
        # The gyro's only consumer is de-rotation inside flow_velocity
        # (`pitch_rate`/`roll_rate` below): these gains shape the rotational
        # term subtracted afterwards, never tracking. Seeding LK's initial
        # guess from the gyro was measured and rejected (nothing survived the
        # 2 px forward-backward gate).
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
        from mongla_vision import optics
        self._n_water = float(optics.N_WATER)   # read now, not at import
        self._want_refract = bool(
            self.get_parameter('refractive_rectify').value)
        self._refract = None
        self._yaw_tol = float(self.get_parameter('yaw_cross_check_tol').value)
        self._undistort = bool(self.get_parameter('undistort').value)
        self._intr = None
        self._exposure_units = float(self.get_parameter('exposure_us').value)
        self._stamp_src = str(self.get_parameter('stamp_source').value).lower()
        if self._stamp_src not in ('soe', 'eof'):
            self.get_logger().warn(
                f'[FLOW ] stamp_source={self._stamp_src!r} is neither soe nor '
                f'eof: NO exposure correction is applied, because a guess is '
                f'a whole exposure wrong in one direction or the other.')
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
        # Issue #32: Scaled to the frames that arrive, not to a constant.
        # This loaded the calibration for a hard-coded 640x360 whatever the
        # camera sent: at 1280x720 f was half its true value and EVERY velocity
        # and distance read 2x, with no error -- `from_json`'s own comment calls
        # it "a clean 2x error nobody sees". The first frame's shape now decides
        # (`_on_image`), and a frame of any other size later is refused.
        self._cal_path = str(self.get_parameter('calibration').value or '').strip()
        self._profile = str(self.get_parameter('profile').value or '').strip()
        if not self._cal_path and self._profile:
            from mongla_vision.calibration.binding import calibration_for_profile
            self._cal_path = calibration_for_profile(self._profile)
            if self._cal_path:
                self.get_logger().info(
                    f'[FLOW ] calibration auto-bound from profile '
                    f'{self._profile!r}: {self._cal_path}')
        self._allow_uncal = bool(self.get_parameter('allow_uncalibrated').value)
        self._cal_problem = self._calibration_problem()
        self._publish_calibration_state()
        self._f_water = f_water
        self._intr_wh = None
        self._n_wrong_size = 0
        self._n_median_fallback = 0
        self._n_yaw_disagree = 0
        self._last_yaw_img = 0.0
        self._last_scale_rate = 0.0
        # Height measured by the CAMERA, checked against the barometer path.
        self._h_optical = HeightFromDivergence()
        self._depth_hist = deque(maxlen=16)
        self._state_stamp_warned = False
        self._n_h_warn = 0

        ns = f'/mongla/vision/{cam}'
        img_qos = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, f'{ns}/image_raw', self._on_image,
                                 img_qos)
        self.create_subscription(Vector3Stamped, '/mongla/imu_rates',
                                 self._on_rates, 50)
        self.create_subscription(MonglaState, '/mongla/state', self._on_state,
                                 10)
        ctrl_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, f'{ns}/distance_control',
                                 self._on_control, ctrl_qos)

        self._pub_vel = self.create_publisher(
            TwistWithCovarianceStamped, f'{ns}/velocity', 10)
        self._pub_quality = self.create_publisher(UInt8, f'{ns}/flow_quality', 10)
        # The floor's grid orientation, absolute in the POOL frame modulo 90
        # degrees. Its own topic rather than folded into the velocity message,
        # because it is a different quantity measured at a different rate and a
        # consumer wanting one rarely wants the other.
        self._pub_grid = self.create_publisher(FloorHeading, f'{ns}/floor_grid_deg', 10)
        # Height above the floor, when the floor itself says so. It was measured
        # here to scale velocity and then thrown away, while every floor task
        # (a bin drop, a floor search, a sloped SAUVC floor) wants exactly this
        # rather than depth below the surface. Published ONLY from the tile
        # grating: the pool_depth path is depth minus a typed constant, which
        # is not a measurement of the floor at all. Absent means unknown.
        self._pub_floor_h = self.create_publisher(Range, f'{ns}/floor_height', 10)
        self._pub_lane = self.create_publisher(FloorHeading, f'{ns}/lane_heading_deg', 10)
        self._pub_dist = self.create_publisher(Float32, f'{ns}/distance_traveled', 10)
        self._pub_debug = self.create_publisher(Float32MultiArray,
                                                f'{ns}/distance_debug', 10)
        # Whether the velocity is on the calibration or on the focal
        # parameters (#80). Latched: it changes at most once, at the first
        # frame, and localization must know it whenever it starts.
        self._pub_cal = self.create_publisher(String, f'{ns}/flow_calibration',
                                              ctrl_qos)

        self._acc = DistanceAccumulator()
        self._rate_buf: deque = deque(maxlen=128)
        self._yaw_rate_buf: deque = deque(maxlen=128)
        self._depth_m = None
        self._depth_t = None
        self._yaw_deg = None
        self._last_height = None
        self._tile_m = float(self.get_parameter('tile_m').value)
        self._tile_period_s = float(self.get_parameter('tile_period_s').value)
        self._tile_decimate = int(self.get_parameter('tile_decimate').value)
        self._tile_next_t = 0.0
        self._tile_height = None          # last height read off the floor
        self._tile_angle = None           # grid orientation, absolute mod 90
        self._tile_warned = False
        # The grating height against the barometric one, over a window rather
        # than per frame. See `_read_the_floor`.
        from .scale_check import ScaleCrossCheck
        self._scale_x = ScaleCrossCheck()
        self._lane_lines = bool(self.get_parameter('lane_lines').value)
        self._lane_angle = None           # lane heading, pool frame mod 180

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

        # The frame path is a one-deep mailbox and a worker thread, not the
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
        elif self._cal_problem:
            self.get_logger().warning(
                f'[FLOW ] UNCALIBRATED by request -- {self._cal_problem}. '
                f'Velocity runs on one focal length and the frame centre '
                f'(3.08 % axis asymmetry measured in this state); the sigma '
                f'and {self._pub_cal.topic_name} say so.')

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

    def destroy_node(self):
        self._stop.set()
        self._slot_evt.set()
        try:
            self._worker.join(timeout=1.0)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    from mongla_localization.orphan import die_with_parent
    die_with_parent()        # an orphan holds devices and duplicates topics
    rclpy.init(args=args)
    node = FlowVelocityNode()
    from mongla_localization.params_file import refuse_unknown
    refuse_unknown(node)       # bringup.yaml `nodes:` typos stop here
    try:
        from mongla_localization.spin import spin
        spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # SIGTERM / SIGINT delivered through rcl rather than to Python. On
        # ROS 2 Jazzy this is the NORMAL path when the launch or an operator
        # stops the node, and letting it propagate prints a ten-frame stack
        # trace on an ordinary Ctrl+C -- which on a pool deck at night reads
        # as a crash in the sensor you are about to trust.
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

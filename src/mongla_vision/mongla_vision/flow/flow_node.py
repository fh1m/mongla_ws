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

THE FILES. `FlowVelocityNode` is one class in five files (plus its constants),
so each part can be read alone:

    flow_node.py       the per-frame pipeline: ripeness, tracking, evaluate
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

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Float32, Float32MultiArray, String, UInt8
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped

from mongla_interfaces.msg import MonglaState
from mongla_vision.flow.flow_timing import TimeOffset
from mongla_vision.flow.flow_math import (
    CAUSTIC_TOPHAT_MAX,
    DistanceAccumulator,
    HeightFromDivergence,
    caustic_score,
    detect_corners,
    suppress_caustics,
    flow_dispersion,
    height_above_floor,
    integrate_rate,
    interp_rate,
    robust_flow,
)
from mongla_vision.flow.flow_velocity import flow_velocity
from .flow_constants import _LK_PARAMS, HEIGHT_STALE_S
from .flow_setup import FlowSetupMixin
from .flow_checks import FlowChecksMixin
from .flow_floor import FlowFloorMixin
from .flow_publish import FlowPublishMixin

# Shi-Tomasi + LK. Bucketing is applied on top of goodFeaturesToTrack so the
# corners are spread across the frame rather than clustered on the one bright
# patch of grout -- a clustered set makes a rotation and a translation look
# alike, and a pool floor is exactly the repetitive lattice where that bites.
_FEATURE_PARAMS = dict(maxCorners=160, qualityLevel=0.01, minDistance=8,
                       blockSize=7)
_MIN_TRACKS = 6
_RESEED_EVERY = 10


class FlowVelocityNode(FlowSetupMixin, FlowChecksMixin, FlowFloorMixin, FlowPublishMixin, Node):
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
        self._pub_grid = self.create_publisher(Float32, f'{ns}/floor_grid_deg', 10)
        # Height above the floor, when the floor itself says so. It was measured
        # here to scale velocity and then thrown away, while every floor task
        # (a bin drop, a floor search, a sloped SAUVC floor) wants exactly this
        # rather than depth below the surface. Published ONLY from the tile
        # grating: the pool_depth path is depth minus a typed constant, which
        # is not a measurement of the floor at all. Absent means unknown.
        self._pub_floor_h = self.create_publisher(Range, f'{ns}/floor_height', 10)
        self._pub_lane = self.create_publisher(Float32, f'{ns}/lane_heading_deg', 10)
        self._pub_dist = self.create_publisher(Float32, f'{ns}/distance_traveled', 10)
        self._pub_debug = self.create_publisher(Float32MultiArray,
                                                f'{ns}/distance_debug', 10)

        self._bridge = CvBridge()
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

        Rotation is not covered by the others, and assuming it was is the
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

        The defect this replaces, measured on this camera at h=0.72 m.
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

        Lowering the constant is not the fix -- that trades a blind spot for
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

        # The regime is the ANCHOR's: both images of a pair go through the same
        # filter, or LK compares an eroded patch against a raw one.
        track = suppress_caustics(gray) if getattr(self, '_anchor_caustic', False) else gray
        nxt, status, _err = cv2.calcOpticalFlowPyrLK(
            self._anchor_gray, track, self._anchor_pts, None, **_LK_PARAMS)

        flow = robust_flow(self._anchor_pts, nxt, status,
                           min_tracks=_MIN_TRACKS)
        if flow is None:
            # The anchor is unusable; re-seed here rather than reporting a
            # refusal for every frame until something changes.
            #
            # Say which failure it is. "LK lost the anchor" reads as a
            # tracking bug and sends an operator after the algorithm; on a
            # dark or textureless floor -- the normal case in murky water --
            # the truth is that there was nothing to track. The two are
            # distinguishable for free by SURVIVAL FRACTION, with no
            # threshold to calibrate: losing a few tracks is tracking,
            # losing essentially all of them at once is the scene.
            #
            # Verified against a genuinely black camera (room dark, mean
            # pixel 0.64): 174 "corners" were detected on sensor noise and
            # 0 of 174 survived. The node refused, which is the right answer
            # -- shot noise is not stable frame to frame, so it cannot
            # produce the confident ZERO that a static fixed-pattern would.
            n_in = len(self._anchor_pts) if self._anchor_pts is not None else 0
            n_st = (int(np.asarray(status).reshape(-1).astype(bool).sum())
                    if status is not None else 0)
            self._anchor(gray, t)
            if n_in >= _MIN_TRACKS and n_st <= max(1, n_in // 20):
                self._refuse(
                    f'no trackable texture ({n_st}/{n_in} points survived) '
                    f'-- dark or featureless floor, not a tracking fault')
            else:
                self._refuse(f'LK lost the anchor ({n_st}/{n_in} survived)')
            return

        n_used = int(np.asarray(status).reshape(-1).astype(bool).sum())
        mag = math.hypot(flow[0], flow[1])
        dt = t - self._anchor_t
        # Rotation is a ripeness criterion, and its absence was a hole.
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
        # Seeding LK with the gyro was tried first and measured to do nothing:
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

        status, n_used = self._drop_inconsistent_tracks(track, nxt, status, n_used)
        if self._caustics_fooled_us(gray, t, track, nxt, status):
            return

        disp = flow_dispersion(self._anchor_pts, nxt, status)

        flow, n_used, disp = self._planar_refit(gray, nxt, status, dt,
                                                flow, n_used, disp)

        self._n_tracks = n_used
        # The anchor advances unconditionally, so a refusal discards the
        # Travel it covered -- and that is DELIBERATE, having been changed
        # and changed back on measurement.
        #
        # The change looked obviously right: `_evaluate` can refuse an
        # interval (rotation-dominated, no gyro, flow under the noise floor)
        # and with the anchor already moved on, that displacement is gone
        # rather than deferred. Worse, the loss is not a uniform sample --
        # refusals CLUSTER IN MOTION, since a rotation-dominated interval and
        # a lost LK anchor both happen in the fast part of a move, which is
        # where the distance is. So it was put behind a parameter.
        #
        # Then it was measured, offline, against a chosen truth of 0.5604 m
        # with forced rotation-dominated refusals:
        #
        #     refusals    advance    hold      verdict
        #     none         0.5604   0.5604     equal
        #     1 in 8       0.4437   0.1868     HOLDING IS WORSE
        #     1 in 5       0.3736   0.0934     HOLDING IS WORSE
        #     1 in 3       0.2102   0.0467     HOLDING IS WORSE
        #
        # Holding also RAISES the refusal count (5 -> 7, 8 -> 10), which is
        # the mechanism: holding widens `dt`, the widened window still
        # contains the disturbance that caused the refusal, so the next
        # interval refuses too and it cascades. Deferring a measurement only
        # pays if the next one is accepted, and a persistent cause guarantees
        # it is not.
        #
        # So travel lost to a refusal is the PRICE of refusing, and the lever
        # that matters is refusing less often -- not holding a stale anchor
        # across it. Pinned by TestRefusalTravelIsThePriceOfRefusing.
        # Read the floor BEFORE the height is consumed, not after, or the
        # tile height is always one cycle stale against the frame that used it.
        self._read_the_floor(gray, t)
        self._anchor(gray, t)
        if dt > 0.0:
            self._evaluate(flow, disp, dt, t, n_used)
        else:
            self._refuse('non-positive baseline')

    # ── the checks inside one `_process` interval ────────────────────────────
    # Each was a span of `_process` until 2026-11-07; code and comments moved
    # verbatim.

    def _anchor(self, gray, t) -> None:
        # Sun caustics: decided once per anchor (see flow_math.CAUSTIC_*).
        score = caustic_score(gray)
        caustic = score > CAUSTIC_TOPHAT_MAX and bool(
            self.get_parameter('caustic_suppression').value)
        if caustic != getattr(self, '_anchor_caustic', False):
            self.get_logger().info(
                f'[FLOW ] sun caustics {"ON" if caustic else "OFF"} '
                f'(score {score:.3f} vs {CAUSTIC_TOPHAT_MAX}): tracking the '
                f'{"ERODED" if caustic else "raw"} floor')
        self._anchor_caustic = caustic
        self._caustic_score = score
        if caustic:
            gray = suppress_caustics(gray)
        self._anchor_gray = gray
        self._anchor_pts = self._bucketed_corners(gray)
        self._anchor_t = t

    def _fresh_height(self, now: float):
        """Height above the floor from a depth measured no more than
        HEIGHT_STALE_S before `now` (host monotonic), else None. None is the
        honest answer: the caller refuses the frame rather than scaling it by
        a height from before the link stalled."""
        if self._depth_m is None or self._depth_t is None:
            return None
        if now - self._depth_t > HEIGHT_STALE_S:
            return None
        return height_above_floor(self._pool_depth, self._depth_m,
                                  self._lens_below_baro)

    def _evaluate(self, flow, disp, dt, t, n_used) -> None:
        ok, why = self._scale_ready()
        if not ok:
            self._refuse(why)
            return

        # Recomputed every frame from a FRESH depth, never held: when depth
        # stops arriving the height goes with it (B10).
        self._last_height = self._fresh_height(time.monotonic())
        h = self._last_height
        # The floor's own answer, when it has one. Preferred over the
        # pool-depth path because it is a direct measurement of the quantity we
        # actually need, with no `pool_depth_m` in it -- and because when the
        # two disagree the one derived from an unmeasured constant is the one
        # to doubt. Falls back silently; a floor without tiles is the common
        # case and not an error.
        if self._tile_height is not None:
            h = self._tile_height
        if h is None:
            self._refuse('no fresh depth, so no height above the floor')
            return

        # The MEAN rate over the baseline, not a midpoint sample: with an
        # adaptive baseline dt reaches ~0.5 s at 2 cm/s, and de-rotation
        # subtracts `f * omega * dt`, so a midpoint that lands on the peak of
        # a swing scales that error by the whole interval.
        # The interval is shifted by td BEFORE THE GYRO IS READ. td > 0
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
        self._last_rates = (pitch_rate, roll_rate)
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
        rclpy.spin(node)
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

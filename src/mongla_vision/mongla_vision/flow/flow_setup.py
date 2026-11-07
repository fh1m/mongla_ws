"""FlowVelocityNode, part 1 of 5: parameters, camera intrinsics, and the input callbacks.

Everything here runs before or beside the per-frame pipeline: it decides what the
node knows about its camera and the hull, never what the floor is doing.
"""
import math
import time
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped
from mongla_interfaces.msg import MonglaState
from mongla_vision.flow.flow_math import Intrinsics, RefractiveRectifier
from mongla_vision.stamps import capture_monotonic
from .flow_constants import (  # noqa: F401
    FLOW_TD_MEASURED_S,
    FOCAL_PARAMS_WIDTH,
    _F_AIR_PX,
    _F_WATER_PX,
    _GYRO_GAIN_X_DEFAULT,
    _GYRO_GAIN_Y_DEFAULT,
)


class FlowSetupMixin:
    """Methods of `FlowVelocityNode`; see `flow_node.py` for the node."""

    def _declare_parameters(self) -> None:
        """Every ROS parameter this node reads, with its default.

        One place to look for a knob; values are read in __init__.
        """
        self.declare_parameter('camera', 'downward')
        # NaN, not 4.0. A default pool depth is a scale factor nobody sets
        # deliberately multiplying every output -- the same shape as the
        # 50-vs-72 cm height error that produced an 83 % scale gap on the
        # bench. Absence must refuse, not guess.
        self.declare_parameter('pool_depth_m', float('nan'))
        # The floor is not one number (issue #35). `pool_depth_m` multiplies
        # every velocity, and real venues slope (TRANSDEC ~1.2 -> 5 m) or step.
        # State the venue's shallowest and deepest floor and the published
        # sigma carries the scale error that range implies; left at NaN, the
        # floor is declared flat at `pool_depth_m` -- an explicit claim, not a
        # silent one. Tile height, when it is seen, replaces both.
        self.declare_parameter('pool_depth_min_m', float('nan'))
        self.declare_parameter('pool_depth_max_m', float('nan'))
        self.declare_parameter('medium', 'water')          # 'air' | 'water'
        self.declare_parameter('focal_air_px', _F_AIR_PX)
        self.declare_parameter('focal_water_px', _F_WATER_PX)
        self.declare_parameter('gyro_gain_x', _GYRO_GAIN_X_DEFAULT)
        self.declare_parameter('gyro_gain_y', _GYRO_GAIN_Y_DEFAULT)
        self.declare_parameter('rot_fraction_max', 0.80)
        # Correct the FLAT PORT instead of averaging over it. Default ON in
        # water: a single f_water is exact only at the radius it was fitted
        # at, and the residual is a 1-2.8 % ANISOTROPIC scale error -- larger
        # than the whole error budget for a pool leg. Verified to recover the
        # true velocity to 0.000 % against forward-simulated port physics.
        self.declare_parameter('refractive_rectify', True)
        # No `water_refractive_index` parameter here (issue #37, B22's rule).
        # Flow had its own; PnP, the lock and the DSL read `optics.N_WATER`.
        # Overriding it at the pool for salt water made flow and pose disagree
        # about the same port. One constant, `optics.N_WATER`, read by all.
        self.declare_parameter('min_net_flow_px', 0.5)
        self.declare_parameter('max_dispersion_ratio', 5.0)
        self.declare_parameter('grid_buckets', 4)
        self.declare_parameter('want_points', 80)
        # Adaptive keyframe baseline. Emit a velocity once this much image
        # displacement has accumulated against the anchor, rather than once
        # per frame. See `_track` for why, and measured-bars for the numbers.
        self.declare_parameter('target_px', 8.0)
        self.declare_parameter('max_px', 25.0)
        self.declare_parameter('max_baseline_s', 0.75)
        # Re-anchor on accumulated rotation, not only on displacement and
        # time. Measured on this camera's own floor texture (see below).
        self.declare_parameter('max_rotation_deg', 6.0)
        # Planar rigid fit. A downward camera on a flat floor sees ONE body
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
        # Which instant the camera driver stamps (V4L2 TSTAMP_SRC). 'soe' is
        # what `uvcvideo` sets -- both vehicle cameras -- and the camera node
        # logs what the driver actually reports at stream start (issue #34).
        self.declare_parameter('stamp_source', 'soe')
        self.declare_parameter('stamp_at_midpoint', True)
        self.declare_parameter('estimate_time_offset', True)
        self.declare_parameter('time_offset_s', FLOW_TD_MEASURED_S)
        # A bound on td, because it is no longer only a diagnostic. Once td
        # shifts every velocity stamp, a spurious correlation peak actively
        # CORRUPTS the output instead of logging a warning -- and the slew
        # makes a wrong value persist across windows. The plausible range is
        # bounded by physics we have measured: transport jitter p2p 35 ms,
        # half-exposure 7.85 ms, and one adaptive baseline 750 ms. Anything
        # past 150 ms is not a link delay, it is a bad peak.
        self.declare_parameter('time_offset_max_s', 0.15)
        self.declare_parameter('time_offset_min_quality', 0.5)
        # The floor's own tile, as a third height source. `pool_depth_m` is,
        # by this node's own admission below, "the one input nobody measures
        # carefully" -- and height is a clean multiplier on every velocity we
        # emit. A tiled floor carries its own scale: `h = f * tile_m / pitch_px`
        # needs no pool depth, no barometer and no vz estimate.
        #
        # `tile_m` is 0.0 = OFF, because a tile size is a VENUE constant. Our
        # pool is not the competition's, and a wrong tile size does not fail --
        # it scales every height by the ratio of the two, silently. Measure it
        # on deck and set it; unset means this stays quiet.
        self.declare_parameter('tile_m', 0.0)
        self.declare_parameter('tile_period_s', 0.5)      # 2 Hz; it moves slowly
        self.declare_parameter('tile_decimate', 3)
        # Lane-line heading (mod 180) for the yaw drift bound. OFF: a path
        # marker is also a dark band. See `_read_the_lane`.
        self.declare_parameter('lane_lines', False)
        # Sun-caustic handling: erode the floor when the top-hat score says the
        # sun is painting it, and refuse a bare floor under caustics. ON by
        # default; OFF tracks the raw floor always (an A/B switch, and the right
        # setting in an indoor pool where the detector has nothing to find).
        self.declare_parameter('caustic_suppression', True)

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

    # ── capture -> mailbox (callback thread, must never block) ──────────────
    def _load_intrinsics(self, width: int, height: int) -> None:
        """Scale the calibration -- or the focal parameters -- to WxH (#32)."""
        cal, f_water = self._cal_path, self._f_water
        if not cal:
            self._scale_focal_params(width, height, 'no calibration')
            return
        if cal:
            try:
                self._intr = Intrinsics.from_json(cal, width, height)
                if self._medium != 'water':
                    self._f_px = self._intr.fx
                elif self._want_refract:
                    # The intrinsics stay in air, deliberately. The lens and
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
                else:
                    # Rectification off: keep the old single-focal-length
                    # behaviour so the two paths can be A/B'd, and say what it
                    # costs rather than leaving it to be discovered.
                    k = f_water / self._intr.fx
                    self._intr.fx *= k
                    self._intr.fy *= k
                    self._f_px = self._intr.fx
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
                # The fallback is the no-calibration path, all of it. It used
                # to leave whatever had been built before the failure -- an
                # `_intr` with no rectifier behind it -- and an `_f_px` still at
                # FOCAL_PARAMS_WIDTH, so a 640-wide stream read a focal length
                # for some other width and every velocity came out scaled by
                # the ratio. (Cloud audit, 2026-11-07.)
                self._intr = None
                self._refract = None
                self._scale_focal_params(width, height, 'calibration unreadable')
        if self._intr is not None:
            self.get_logger().info(f'[FLOW ] intrinsics scaled to the first '
                                   f'frame, {width}x{height}: {self._intr}')

    def _scale_focal_params(self, width: int, height: int, why: str) -> None:
        """No usable calibration: the focal parameters are for FOCAL_PARAMS_WIDTH."""
        k = float(width) / FOCAL_PARAMS_WIDTH
        self._f_px *= k
        self.get_logger().warning(
            f'[FLOW ] {why}: focal parameters scaled x{k:.3f} '
            f'from {FOCAL_PARAMS_WIDTH:.0f} px wide to the {width}x{height} '
            f'frames actually arriving')

    def _admit_frame_size(self, wh) -> bool:
        """First frame: scale the intrinsics to it. Later: refuse any other size.

        Refused, not rescaled mid-run: the anchor, the baseline and every
        buffered point are in the OLD pixel units (issue #32).
        """
        if self._intr_wh is None:
            self._load_intrinsics(*wh)
            self._intr_wh = tuple(wh)
            return True
        if tuple(wh) == self._intr_wh:
            return True
        if self._n_wrong_size % 100 == 0:
            self.get_logger().error(
                f'[FLOW ] frame {wh[0]}x{wh[1]} != the {self._intr_wh[0]}x'
                f'{self._intr_wh[1]} the intrinsics were scaled for -- REFUSING '
                f'it (a mid-run resolution change). Restart flow.')
        self._n_wrong_size += 1
        return False

    def _reset_lk(self) -> None:
        self._anchor_gray = None
        self._anchor_pts = None
        self._anchor_t = None
        self._n_ok = 0
        self._n_refused = 0

    # ── inputs ──────────────────────────────────────────────────────────────
    def _on_rates(self, msg: Vector3Stamped) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # x=pitch, y=roll, z=yaw (rad/s). The yaw channel was published and
        # read by nothing; it is what makes the image/gyro cross-check below
        # possible without adding a sensor.
        self._rate_buf.append((t, float(msg.vector.x), float(msg.vector.y)))
        self._yaw_rate_buf.append((t, float(msg.vector.z)))

    def _on_state(self, msg: MonglaState) -> None:
        """Depth, stamped when it was MEASURED rather than when it arrived.

        The sixth instance, in the package that owns the fix. `stamps.py`
        records four prior cases of "the clock was read at the wrong place";
        `flow_position` integrating on arrival time was the fifth. This is the
        same defect on the depth series: `_vz_down` differences these instants,
        so the host's transport jitter (sd 6.67 ms, p2p 35.12 ms, all of it
        USB/UART scheduling) went straight into a vertical speed.

        The manager now stamps `/mongla/state` from the board's own clock, so
        the honest instant is on the message. `capture_monotonic` maps it into
        this process's monotonic domain and says so when it cannot.
        """
        t, why = capture_monotonic(msg.header)
        if why and not self._state_stamp_warned:
            self._state_stamp_warned = True
            self.get_logger().warning(
                f'[FLOW ] depth stamps fall back to arrival time ({why}). '
                f'vz is then differenced over arrival instants and carries the '
                f'transport jitter, which reads as vertical motion that is not '
                f'happening.')
        if not np.isnan(msg.depth_m):
            self._depth_m = float(msg.depth_m)
            # The measurement's own instant, already on this host's monotonic
            # clock: a depth's age is now minus when the board took it.
            self._depth_t = t
            self._depth_hist.append((t, self._depth_m))
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

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'[FLOW ] cv_bridge decode failed: {exc!r}')
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if not self._admit_frame_size((int(gray.shape[1]), int(gray.shape[0]))):
            return
        # The capture stamp, never `now()`. dt is the denominator of every
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

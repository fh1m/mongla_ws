"""The vehicle's own answer to "where am I", fused from every sensor it has.

⛔ WHY THIS NODE EXISTS AT ALL. `inekf.py` landed as a tested library with no
caller -- 19 unit tests, a converging filter, and nothing on the vehicle ever
constructing one. That is the failure the bumblebee doctrine names directly:
effort is not points, and a module nothing calls scores zero however good it
is. This file is the wiring, and it is deliberately thin. Every decision worth
arguing about lives in `inekf.py` and is tested without ROS.

THE SPLIT, which is the point. The SROT board runs a 500 Hz attitude loop with
the BNO085 in hand and owns every inner loop. It is not trying to know where it
is in the pool, and it has neither the cameras nor the memory to. This node is
the outer half: it takes the board's inertial stream, the downward camera's
optical flow, the barometer, and pool-frame position fixes resected off props,
and produces one pose. Board fuses what is fast; Pi fuses what is wide.

TIME. Every board-sourced sample is stamped through `flow_timing.ClockMap` in
the manager, not on arrival -- measured on this vehicle, the board's ATTITUDE
interval has sd 0.00 ms where host arrival has sd 6.67 ms and p2p 35.12. This
node reads `header.stamp` and never `now()`, or it would put all of that
jitter back into dt.

NO IMU, NO PREDICT. There is deliberately no synthetic propagation when the
inertial stream stops. A filter that holds its state and keeps taking depth,
yaw and flow updates is honest about having no new inertial information; one
that invents `a = 0` to keep a timer fed is dead-reckoning on an assumption
and says nothing about it. Measured 2026-09-11: after a bad boot the board
streamed ATTITUDE and SCALED_IMU2 as exact 0.0 in every field while reporting
3D_GYRO and 3D_ACCEL unhealthy -- so "zeros arrived" and "the hull is still"
are the same bytes, and only the health bit separates them. `SrotFC.get_imu`
returns None in that state, no message is published, and this node coasts.
"""
from __future__ import annotations

import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from geometry_msgs.msg import (PointStamped, TwistWithCovarianceStamped,
                               Vector3Stamped)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String

from duburi_interfaces.msg import DuburiState

from duburi_localization.command_velocity import (BLOCKED, CommandVelocityModel,
                                                  MotionCheck)
from duburi_localization.inekf import RIEKF
from duburi_localization.tile_grating import snap_to_grid

# A dt longer than this is a GAP, not a long step. Integrating one 2 s
# interval as a single Euler step is not the same estimate as 100 steps of
# 20 ms, and the error is worst exactly when it matters -- after a dropout.
# 0.25 s is 12 missed samples at the board's 50 Hz.
MAX_PREDICT_DT_S = 0.25

# Below this the step is numerically pointless and only amplifies stamp noise.
MIN_PREDICT_DT_S = 1e-4

# ZUPT stationarity, measured rather than assumed. The board's gyro reads
# 9, -11, -3 mrad/s on a still bench (measured 2026-09-11), so 0.02 rad/s is
# roughly twice the observed noise floor -- tight enough to exclude a slow
# yaw, loose enough not to be tripped by the sensor itself.
STILL_GYRO_RAD_S = 0.02
# Peak-to-peak of |specific force| over the window. Gravity is included in the
# reading, so a tilted hull shows a large CONSTANT; only variation is motion.
STILL_ACCEL_SPREAD = 0.25          # m/s^2
STILL_WINDOW = 50                  # 1.0 s at the board's 50 Hz
# Flow newer than this means the camera is already reporting velocity, so a
# ZUPT would add nothing and could only conflict with it.
FLOW_FRESH_S = 1.0


def _stamp_s(header) -> float:
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


class LocalizationNode(Node):
    def __init__(self, **kw):
        super().__init__('duburi_localization', **kw)

        self._filter = RIEKF()
        self._last_imu_t: float | None = None
        self._imu_gap_warned = False
        # Counters, published in the diagnostic line. Which channel is feeding
        # the filter is the first question when a pose looks wrong, and a rate
        # of zero on one input is invisible in the pose itself.
        self._n = {'imu': 0, 'att': 0, 'depth': 0, 'yaw': 0, 'flow': 0,
                   'fix': 0, 'zupt': 0, 'grid': 0, 'grid_refused': 0,
                   'lane': 0, 'lane_refused': 0, 'model': 0, 'gap': 0}
        self._still: deque = deque(maxlen=STILL_WINDOW)
        self._last_flow_t = 0.0
        self._attitude_seeded = False
        self._anchored = False
        self._last_input_t = 0.0
        self._aided_at_last_diag = -1

        cam = str(self.declare_parameter('flow_camera', 'downward').value)
        self._flow_sigma = float(self.declare_parameter('flow_sigma', 0.05).value)
        self._depth_sigma = float(self.declare_parameter('depth_sigma', 0.02).value)
        self._yaw_sigma_deg = float(self.declare_parameter('yaw_sigma_deg', 2.0).value)
        self._fix_sigma = float(self.declare_parameter('fix_sigma', 0.5).value)
        # The BNO's datasheet drift, not a tuning guess: under 0.01 deg/min at
        # rest, measured on this board over 8 minutes. Loose enough that a
        # manoeuvre's transient does not fight the filter, tight enough that
        # attitude is effectively pinned to the board.
        self._attitude_sigma_deg = float(
            self.declare_parameter('attitude_sigma_deg', 0.5).value)
        self._zupt_sigma = float(self.declare_parameter('zupt_sigma', 0.01).value)
        self._zupt_enabled = bool(self.declare_parameter('zupt', True).value)
        # The grid is geometry, not a compass: when it applies it is very
        # precise (measured +/-1.0 deg against known truth), so a tight sigma
        # is honest. The correction bound is the safety, not the sigma.
        self._grid_sigma_deg = float(
            self.declare_parameter('grid_sigma_deg', 1.0).value)
        self._grid_max_corr_deg = float(
            self.declare_parameter('grid_max_correction_deg', 20.0).value)
        # Off by default. The board's yaw is already a fused 500 Hz solution and
        # feeding it back in as a measurement makes this filter agree with it by
        # construction -- which looks like convergence and measures nothing.
        # Turn it on when the anchor has made heading absolute, not before.
        self._use_yaw = bool(self.declare_parameter('use_yaw', False).value)
        # Velocity from commanded demand, learned against flow. ON by default
        # because it is silent until it has LEARNED: an unready model aids
        # with nothing, so the default cannot inject a guessed gain.
        self._model = CommandVelocityModel(
            tau_s=float(self.declare_parameter('demand_tau_s', 1.0).value))
        self._model_aid = bool(self.declare_parameter('demand_aid', True).value)
        self._last_demand_t = None
        self._motion = MotionCheck()
        self._motion_state = None

        sensor_qos = QoSProfile(depth=20,
                                reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Imu, '/duburi/imu', self._on_imu, sensor_qos)
        self.create_subscription(DuburiState, '/duburi/state', self._on_state, 10)
        self.create_subscription(
            TwistWithCovarianceStamped,
            f'/duburi/vision/{cam}/velocity', self._on_flow, sensor_qos)
        # The pool-frame fix from `duburi.fix_position()` -- prop resection.
        # This is the only channel that bounds horizontal drift, so it is the
        # one whose absence is worth noticing in the diagnostic line.
        self.create_subscription(
            PointStamped, '/duburi/localization/fix', self._on_fix, 10)
        # The anchored world heading, latched by `anchor_on()`. Until one
        # arrives the filter's attitude is the BOARD's, which is boot-relative
        # or magnetic -- a perfectly good attitude in a frame that is not the
        # pool's. Receiving this is what makes the output frame honest.
        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Float32, '/duburi/localization/heading',
                                 self._on_heading, latched)
        self.create_subscription(Vector3Stamped, '/duburi/demand',
                                 self._on_demand, sensor_qos)
        # The floor's grid, which BOUNDS yaw drift without a magnetometer.
        self.create_subscription(
            Float32, f'/duburi/vision/{cam}/floor_grid_deg',
            self._on_floor_grid, 10)
        # The lane line: the same drift bound with HALF the aliasing (a line
        # is identical from two directions, a grid from four). Shares the
        # grid's sigma and correction bound -- one floor-geometry pair, not two.
        self.create_subscription(
            Float32, f'/duburi/vision/{cam}/lane_heading_deg',
            self._on_lane_line, 10)

        self._pub = self.create_publisher(Odometry, '/duburi/odom', 10)
        # 'ok' | 'blocked' | 'unknown', latched and published on change. Read
        # by `duburi.motion()`: a timed move into a prop reports success, and
        # this is the only thing that can tell the mission it went nowhere.
        self._pub_motion = self.create_publisher(String, '/duburi/localization/motion', latched)
        self.create_timer(0.1, self._publish)
        self.create_timer(5.0, self._diagnose)
        self.get_logger().info(
            f'[LOCAL] invariant filter up: imu=/duburi/imu '
            f'flow=/duburi/vision/{cam}/velocity use_yaw={self._use_yaw}')

    # ---- inputs ---------------------------------------------------------

    def _on_imu(self, msg: Imu) -> None:
        t = _stamp_s(msg.header)
        prev, self._last_imu_t = self._last_imu_t, t
        if prev is None:
            return
        dt = t - prev
        if dt <= MIN_PREDICT_DT_S:
            # Also catches a non-monotonic stamp, which a refitted ClockMap can
            # produce: the mapping moves, so two samples can arrive out of
            # order in host time. Dropping the step is right -- a negative dt
            # integrates the state BACKWARDS and nothing downstream would show
            # it as anything but drift.
            return
        if dt > MAX_PREDICT_DT_S:
            self._n['gap'] += 1
            if not self._imu_gap_warned:
                self._imu_gap_warned = True
                self.get_logger().warning(
                    f'[LOCAL] inertial gap {dt:.2f} s > {MAX_PREDICT_DT_S} s: '
                    f'skipping the step rather than integrating it as one. '
                    f'Position is coasting on flow and depth alone.')
            return
        self._filter.predict(
            (msg.angular_velocity.x, msg.angular_velocity.y,
             msg.angular_velocity.z),
            (msg.linear_acceleration.x, msg.linear_acceleration.y,
             msg.linear_acceleration.z),
            dt)
        self._n['imu'] += 1
        self._last_input_t = t

        # ⛔ THE SPLIT, ENFORCED HERE. The board owns attitude; without this
        # line the companion propagates its own and DIVERGES -- measured on
        # the vehicle before it was added: 7.1e6 m of position in 35 s. An
        # unaided inertial attitude error grows through the gravity coupling
        # (A[3:6,0:3] = skew(GRAVITY)), the accelerometer's gravity component
        # then leaks into horizontal acceleration, and the depth update's gain
        # pumps the result into x and y through the cross terms. Nothing in
        # the filter is wrong; it was simply being asked to estimate something
        # it had no information about.
        #
        # `orientation_covariance[0] < 0` is the ROS "no data" convention, and
        # a backend that cannot supply attitude must not be given a fabricated
        # one -- it would be better to diverge visibly than to converge to a
        # number nobody measured.
        if msg.orientation_covariance[0] >= 0.0:
            R_meas = _R_from_quat(msg.orientation.w, msg.orientation.x,
                                  msg.orientation.y, msg.orientation.z)
            if not self._attitude_seeded:
                # ⛔ THE FIRST ATTITUDE IS AN INITIALISATION, NOT A CORRECTION.
                # The filter starts at identity; the board starts wherever the
                # hull is pointing. Measured on the vehicle at -168.3 deg of
                # yaw, correcting into that from identity produced 345
                # rejected measurements, one gate-lockout break, and 0.9 m of
                # position error laid down during the transient -- error that
                # never goes away, because nothing observes horizontal
                # position until a fix arrives.
                #
                # An estimator with no prior information should ADOPT the
                # first measurement, not argue with it.
                self._attitude_seeded = True
                self._filter.X.R = R_meas
                self.get_logger().info(
                    f'[LOCAL] seeded attitude from the board: '
                    f'yaw {self._filter.X.yaw_deg():+.1f} deg')
            self._filter.update_attitude(
                R_meas, sigma_deg=self._attitude_sigma_deg)
            self._n['att'] += 1

        # Stationarity evidence for the ZUPT, kept per sample. |a| rather than
        # the vector, because the gravity component is what makes the raw axes
        # attitude-dependent while its magnitude is not.
        a = msg.linear_acceleration
        w = msg.angular_velocity
        self._still.append((
            max(abs(w.x), abs(w.y), abs(w.z)),
            math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)))
        if self._zupt_enabled:
            self._maybe_zupt()

    def _on_state(self, msg: DuburiState) -> None:
        depth = float(msg.depth_m)
        if not math.isnan(depth):
            self._filter.update_depth(depth, sigma=self._depth_sigma)
            self._n['depth'] += 1
        yaw = float(msg.yaw_deg)
        # NaN is the documented absence sentinel, and on srot it is now what a
        # board with an unhealthy BNO actually publishes. Feeding NaN into the
        # update would poison every state through the gain, silently.
        if self._use_yaw and not math.isnan(yaw):
            self._filter.update_yaw(yaw, sigma_deg=self._yaw_sigma_deg)
            self._n['yaw'] += 1

    def _on_flow(self, msg: TwistWithCovarianceStamped) -> None:
        """Downward optical flow: the DVL we do not have.

        ⛔ X AND Y ONLY. A bottom-looking camera cannot see vertical velocity,
        and the flow node says so by marking `covariance[14]` as -1.0 -- the
        ROS convention for an unobserved component. Its `linear.z` is 0.0
        because nothing ever set it, and feeding that to a 3-D update asserts
        at full confidence that the hull is not moving vertically, which
        fights the depth channel on every dive.

        The variances are the flow node's own, computed per sample from the
        standard error of the mean over its RANSAC inliers. A textured floor
        and a bare one do not deserve equal weight, and the constant sigma
        this used to pass threw that distinction away.
        """
        v = msg.twist.twist.linear
        if math.isnan(v.x) or math.isnan(v.y):
            return
        var_x = float(msg.twist.covariance[0])
        var_y = float(msg.twist.covariance[7])
        # A non-positive variance is the "no data" marker, not a confident
        # zero. Falling back to the parameter keeps a publisher that does not
        # fill covariance usable instead of silently un-weighting it.
        floor = self._flow_sigma ** 2
        if not (var_x > 0.0) or not math.isfinite(var_x):
            var_x = floor
        if not (var_y > 0.0) or not math.isfinite(var_y):
            var_y = floor
        self._filter.update_body_velocity_xy(v.x, v.y,
                                             max(var_x, 1e-6),
                                             max(var_y, 1e-6))
        self._n['flow'] += 1
        now = time.monotonic()
        dt = min(now - self._last_flow_t, 0.25) if self._last_flow_t > 0.0 else 0.0
        self._last_flow_t = now
        state = self._motion.observe(self._model, v.x, v.y, dt)
        # Never learn from a hull that is not moving as told: a pinned hull
        # would teach the model that thrust makes no speed.
        if not self._motion.suspect and state != BLOCKED:
            self._model.learn(v.x, v.y)      # the easy regime teaches the hard one
        self._report_motion(state)

    def _report_motion(self, state: str) -> None:
        if state == self._motion_state:
            return
        self._motion_state = state
        if state == BLOCKED:
            self.get_logger().warning(
                f'[LOCAL] BLOCKED on body {self._motion.axis}: thrust is '
                f'commanded and the floor is not moving. Against a prop, '
                f'snagged, or a thruster is dead.')
        pub = getattr(self, '_pub_motion', None)
        if pub is not None:
            pub.publish(String(data=state))

    def _on_demand(self, msg) -> None:
        """The thruster demand in force (NaN = unknown). Advances the model's lag.

        Arrival time on `monotonic()`, not the header: the lag integrates a
        host-side quantity the manager sampled on its own timer, and the gap is
        capped so a stalled publisher cannot become one enormous step.
        """
        now = time.monotonic()
        dt = 0.0 if self._last_demand_t is None else min(now - self._last_demand_t, 0.25)
        self._last_demand_t = now
        x, y = float(msg.vector.x), float(msg.vector.y)
        if math.isfinite(x) and math.isfinite(y):
            self._model.step(x, y, dt)
        else:
            self._model.step(None, None, dt)

    def _maybe_model_aid(self) -> None:
        """When flow has gone quiet, aid velocity from the learned demand model.

        ⛔ ONLY WHILE FLOW IS STALE, on the same `FLOW_FRESH_S` the ZUPT uses, so
        there is one notion of "flow is dead". With flow live the model would
        be fed back the numbers it was fitted to -- agreement by construction.
        """
        if not self._model_aid:
            return
        if time.monotonic() - self._last_flow_t < FLOW_FRESH_S:
            return
        if self._last_demand_t is None or time.monotonic() - self._last_demand_t > 0.5:
            return
        p = self._model.predict()
        if p is None:
            return
        self._filter.update_body_velocity_xy(p[0], p[1], p[2], p[3])
        self._n['model'] += 1

    def _maybe_zupt(self) -> None:
        """Stand still and the filter learns from it -- but only if it IS still.

        ZUPT turns a stationary interval into a direct observation of the
        velocity error that has accumulated, which keeps inertial growth
        linear rather than quadratic. It matters exactly when flow is absent:
        with flow running, a still hull already measures zero and this adds
        nothing.

        ⛔ THE CALLER OWNS THE STATIONARITY TEST, and getting it wrong is how a
        ZUPT ruins a filter -- declaring "still" during a slow constant-speed
        transit deletes real motion. So stillness is MEASURED, never assumed
        and never taken from mission intent: the hull station-keeping against
        a current is commanded still and is not. The board reports its own
        gyro and accelerometer at 50 Hz, which is the same reasoning that let
        us measure bench stillness rather than assume it.
        """
        if len(self._still) < self._still.maxlen:
            return
        if time.monotonic() - self._last_flow_t < FLOW_FRESH_S:
            return                      # flow is live; it already says zero
        gyro = [g for g, _ in self._still]
        acc = [a for _, a in self._still]
        if max(gyro) > STILL_GYRO_RAD_S:
            return
        # The SPREAD of specific force, not its magnitude -- gravity is in
        # there and a tilted hull reads a large constant. Variation is motion.
        if (max(acc) - min(acc)) > STILL_ACCEL_SPREAD:
            return
        self._filter.update_zero_velocity(sigma=self._zupt_sigma)
        self._n['zupt'] += 1
        self._still.clear()

    def _on_heading(self, msg: Float32) -> None:
        """An absolute world heading from the landmark anchor.

        ⛔ THIS IS WHAT MAKES `frame_id` TRUE. The board's yaw is excellent and
        is not a pool bearing: it is relative to boot, or to magnetic north
        through a reference that may not have locked (`MAGACC` 0.0, `YAW_REF`
        0.0, `COMP_SEEN` 0.0 on this hull, measured). Until an anchor arrives
        the estimate is in the board's frame and the output says `odom`; after
        it, the frame really is the course's and the output says `pool`.

        Publishing `pool` before that point would not be a labelling nicety:
        flow integrates into position through the SAME rotation, so every
        dead-reckoned metre would walk off along an offset nobody measured.
        """
        self._filter.update_yaw(float(msg.data), sigma_deg=self._yaw_sigma_deg)
        self._n['yaw'] += 1
        if not self._anchored:
            self._anchored = True
            self.get_logger().info(
                f'[LOCAL] heading anchored at {float(msg.data):+.1f} deg: the '
                f'output frame is the POOL from here on.')

    def _on_floor_grid(self, msg: Float32) -> None:
        """The floor's tile grid: a DRIFT BOUND on yaw, not a heading source.

        ⛔ WHAT THIS CAN AND CANNOT DO. A square grid is identical from four
        directions, so it can never say which way the hull faces. What it can
        say is that the residual between our heading and the grid should be
        CONSTANT -- so any movement of that residual is accumulated gyro drift,
        and removing it bounds the drift without a magnetometer. Our hull
        deliberately never fuses one, because the thrusters sit beside it.

        Gated hard: a correction larger than `grid_max_correction_deg` means
        the estimate and the floor disagree about which grid line is which, and
        applying it would snap the hull 90 degrees onto the wrong branch. A
        drifting heading is still roughly right; a confidently wrong one is
        not. So a large disagreement is REFUSED and counted, never applied.

        Only while anchored. Before the anchor our yaw is in the board's boot
        frame, and pulling a boot-frame heading onto a pool-frame grid would
        combine two unrelated angles into a confident wrong one.
        """
        self._bound_yaw(float(msg.data), 90.0, 'grid')

    def _on_lane_line(self, msg: Float32) -> None:
        """The lane line's heading, modulo 180: the grid's drift bound, but
        a line is identical from two ends rather than four, so a wrong branch
        is 180 degrees away instead of 90 -- the bound is far harder to alias.
        """
        self._bound_yaw(float(msg.data), 180.0, 'lane')

    def _bound_yaw(self, angle_deg: float, period_deg: float, key: str) -> None:
        """Pull yaw onto a floor feature of symmetry `period_deg`, or refuse."""
        if not self._anchored:
            return
        snapped = snap_to_grid(self._filter.X.yaw_deg(), angle_deg,
                               max_correction_deg=self._grid_max_corr_deg,
                               period_deg=period_deg)
        if snapped is None:
            self._n[f'{key}_refused'] += 1
            return
        self._filter.update_yaw(snapped, sigma_deg=self._grid_sigma_deg)
        self._n[key] += 1

    def _on_fix(self, msg: PointStamped) -> None:
        """A pool-frame position, from `fix_position()` or `fix_from_prop()`.

        `point.z` carries the sigma the producer derived (range-dependent for
        a single-prop fix, because pose error from a planar target grows with
        the square of range). <= 0 means the producer had none, and the node's
        parameter stands in -- absence, not a silent zero, which here would
        mean infinite confidence.
        """
        sigma = float(msg.point.z)
        if not (sigma > 0.0) or not math.isfinite(sigma):
            sigma = self._fix_sigma
        self._filter.update_position((msg.point.x, msg.point.y), sigma=sigma)
        self._n['fix'] += 1

    # ---- output ---------------------------------------------------------

    def _publish(self) -> None:
        self._maybe_model_aid()          # 10 Hz: one aid per output, not per IMU
        st = self._filter.X
        m = Odometry()
        # ⛔ THE STAMP IS THE LATEST INPUT'S, NOT `now()`. Every input to this
        # filter is stamped on the BOARD's clock through `ClockMap`, and
        # stamping the output on the host wall clock reintroduces exactly the
        # transport jitter that mapping exists to remove -- measured at 6.67 ms
        # sd, 35.12 ms p2p. This stack has now made the wrong-clock mistake in
        # six places; this was nearly the seventh, on the output of the one
        # node that was careful at all four inputs.
        if self._last_input_t > 0.0:
            m.header.stamp.sec = int(self._last_input_t)
            m.header.stamp.nanosec = int(
                (self._last_input_t - int(self._last_input_t)) * 1e9)
        else:
            m.header.stamp = self.get_clock().now().to_msg()
        # `pool` is a CLAIM, and it is only true once the heading is anchored.
        m.header.frame_id = 'pool' if self._anchored else 'odom'
        # Both frames are NED (z down: position.z is +depth) with an FRD child.
        m.child_frame_id = 'duburi'
        m.pose.pose.position.x = float(st.p[0])
        m.pose.pose.position.y = float(st.p[1])
        m.pose.pose.position.z = float(st.p[2])
        qw, qx, qy, qz = _quat_from_R(st.R)
        m.pose.pose.orientation.w = qw
        m.pose.pose.orientation.x = qx
        m.pose.pose.orientation.y = qy
        m.pose.pose.orientation.z = qz
        # Velocity in the BODY frame, which is what `child_frame_id` means in
        # nav_msgs/Odometry and what every consumer of it assumes. The filter
        # carries world velocity (it has to, or the error dynamics stop being
        # log-linear), so rotate on the way out rather than storing it rotated.
        v_body = st.R.T @ st.v
        m.twist.twist.linear.x = float(v_body[0])
        m.twist.twist.linear.y = float(v_body[1])
        m.twist.twist.linear.z = float(v_body[2])
        _fill_covariance(m, self._filter)
        self._pub.publish(m)

    def _diagnose(self) -> None:
        n = self._n
        # ⛔ THE ONE DEGRADATION AN OPERATOR MUST BE TOLD ABOUT. With attitude
        # and depth alone, horizontal velocity is COMPLETELY unobserved --
        # depth constrains z, attitude constrains R, nothing constrains vx/vy
        # -- so bias and attitude residual integrate twice without bound.
        # Measured on this vehicle with ZUPT disabled and no flow: 635 m of
        # position in 95 s, while the filter published a pose the whole time
        # and looked entirely healthy.
        #
        # Flow in water or ZUPT when still. Neither is optional, and having
        # neither is not a degraded estimate, it is not an estimate.
        aided = n['flow'] + n['zupt']
        if aided == self._aided_at_last_diag:
            self.get_logger().warning(
                '[LOCAL] NO VELOCITY AIDING in the last window: no optical '
                'flow, no ZUPT. Horizontal position is unobserved and will '
                'run away -- do not act on it. (Downward camera seeing the '
                'floor? Hull genuinely moving?)')
        self._aided_at_last_diag = aided
        self.get_logger().info(
            f"[LOCAL] imu={n['imu']} att={n['att']} flow={n['flow']} "
            f"zupt={n['zupt']} depth={n['depth']} "
            f"yaw={n['yaw']} grid={n['grid']}/{n['grid'] + n['grid_refused']} "
            f"lane={n['lane']}/{n['lane'] + n['lane_refused']} "
            f"model={n['model']} fix={n['fix']} gaps={n['gap']} "
            f"rej={self._filter.rejected} brk={self._filter.lockout_breaks} | "
            f"yaw={self._filter.X.yaw_deg():+.1f} deg "
            f"p=({self._filter.X.p[0]:+.2f},{self._filter.X.p[1]:+.2f},"
            f"{self._filter.X.p[2]:+.2f}) m")


def _R_from_quat(w, x, y, z):
    """Rotation matrix from (w, x, y, z), normalised on the way in.

    The normalisation is not defensive tidiness: a quaternion that arrives
    slightly off unit length produces a matrix with determinant != 1, and
    `so3_log` of that is not a rotation vector. The filter would take the
    resulting garbage as a real innovation.
    """
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n == 0.0:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _quat_from_R(R):
    """(w, x, y, z) from a rotation matrix, via the largest-diagonal branch.

    The naive `w = sqrt(1+trace)/2` form divides by `w`, so it loses all
    precision near a 180 degree rotation and can take a sqrt of a small
    negative from rounding. Branching on the largest diagonal element keeps the
    divisor bounded away from zero for every input.
    """
    m00, m11, m22 = R[0, 0], R[1, 1], R[2, 2]
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return (0.25 * s, (R[2, 1] - R[1, 2]) / s,
                (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return ((R[2, 1] - R[1, 2]) / s, 0.25 * s,
                (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s)
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                0.25 * s, (R[1, 2] + R[2, 1]) / s)
    s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
            (R[1, 2] + R[2, 1]) / s, 0.25 * s)


def _fill_covariance(msg: Odometry, filt: RIEKF) -> None:
    """Copy the filter's own position and attitude blocks into the message.

    BumblebeeAS moved four nodes from `PoseWithCovarianceStamped` to
    `PoseStamped` in 2026 -- they threw the covariance away. Publishing a
    correctly-scaled one is us going past them, and it costs nothing here
    because the filter already maintains it. The state order is
    [theta | v | p | bg | ba], so attitude is block 0:3 and position 6:9.
    """
    P = filt.P
    # The twist is published in the BODY frame, so its covariance must be too.
    # The filter's velocity block is WORLD-frame; copying it unrotated swaps
    # the x and y variances on a 90 degree heading, and every consumer that
    # weights by axis (the board's distance leg, flow position) gets the
    # other axis's confidence.
    R = filt.X.R
    Pv_body = R.T @ P[3:6, 3:6] @ R
    for r in range(3):
        for c in range(3):
            msg.pose.covariance[r * 6 + c] = float(P[6 + r, 6 + c])
            msg.pose.covariance[(r + 3) * 6 + (c + 3)] = float(P[r, c])
            msg.twist.covariance[r * 6 + c] = float(Pv_body[r, c])


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

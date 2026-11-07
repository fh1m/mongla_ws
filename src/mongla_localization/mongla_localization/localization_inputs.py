"""What the localization node does with each measurement as it arrives.

One mixin of `LocalizationNode` (`localization_node.py`): every subscription
callback, and the stillness and motion-model checks they feed. Each callback
turns a message into one filter update, applied at the instant it describes.
"""

from __future__ import annotations

import math
import time

import numpy as np

from geometry_msgs.msg import (PointStamped, TwistWithCovarianceStamped)
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String

from mongla_interfaces.msg import FloorHeading, MonglaState

from mongla_localization.command_velocity import (BLOCKED)
from mongla_localization import frames
from mongla_localization.inekf import _wrap180
from mongla_localization.tile_grating import snap_to_grid

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
# Stillness needs a command too (issue #25). The IMU test above cannot tell
# "still" from "cruising at constant velocity": heading held, no acceleration,
# |a| = g. Flow drops out mid-transit (bare floor, turbidity), and the ZUPT then
# deleted real forward velocity at sigma 1 cm/s, once a second, in exactly the
# regime where dead reckoning was all that was left. The thruster demand is a
# NECESSARY condition -- a hull asked to move is not still -- never a
# sufficient one, since a hull commanded still drifts on a current.
#
# Centred is the board's own test (`actuation_model.CENTRE_EPS`, mixer.cpp:102):
# below it the mixer outputs neutral. Restated, not imported -- this package
# does not depend on mongla_control -- and compared by a test.
DEMAND_CENTRE_EPS = 0.005
# How long a centred demand must hold before the hull can have stopped: the
# thrusters spin down and the hull coasts, so several of the command model's
# time constants, and never less than the IMU window.
DEMAND_SETTLE_TAUS = 3.0
# A demand older than this is UNKNOWN, the same bar the model aid uses.
DEMAND_FRESH_S = 0.5
# Flow newer than this means the camera is already reporting velocity, so a
# ZUPT would add nothing and could only conflict with it.
FLOW_FRESH_S = 1.0


def _stamp_s(header) -> float:
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


def _opt_stamp(msg):
    """A message's stamp in seconds, or None when it has no usable one."""
    h = getattr(msg, 'header', None)
    if h is None:
        return None
    t = _stamp_s(h)
    return t if t > 0.0 else None


def _now_or(t_newest, fallback: float) -> float:
    return t_newest if t_newest is not None else fallback



def _Rz(deg: float) -> np.ndarray:
    """Yaw about world z (NED down): positive turns north toward east."""
    a = math.radians(float(deg))
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


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



class LocalizationInputsMixin:
    # ---- inputs ---------------------------------------------------------

    def _apply(self, t, fn, *, span: float = 0.0, make=None) -> bool:
        """Run `fn(filter)` as of time `t` (retrodicted), or now (default).

        `t=None` means the input carries no stamp: it is applied at the newest
        event time, which is what on-arrival meant all along.
        """
        retro = getattr(self, '_retro', None)
        if retro is None:
            fn(self._filter)
            return True
        if t is None:
            t = _now_or(retro.newest_t(), self._last_imu_t or 0.0)
        return retro.run(float(t), fn, span=span, make=make)

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
                    f'not integrating it as one IMU step -- coasting at '
                    f'constant velocity and growing the covariance for it.')
            # Coast, do not drop (issue #37). Dropping froze position for
            # the gap (flow corrects velocity, not position) and added no
            # uncertainty for time nobody measured.
            self._apply(t, lambda f, d=dt: f.coast(d), span=dt,
                        make=lambda d: (lambda f: f.coast(d)))
            return
        gyro = (msg.angular_velocity.x, msg.angular_velocity.y,
                msg.angular_velocity.z)
        accel = (msg.linear_acceleration.x, msg.linear_acceleration.y,
                 msg.linear_acceleration.z)
        # `make` rebuilds this step over any duration, so a late measurement
        # can split it at its own instant (retro.py, issue #31).
        self._apply(t, lambda f, g=gyro, a=accel, d=dt: f.predict(g, a, d),
                    span=dt,
                    make=lambda d, g=gyro, a=accel: (
                        lambda f: f.predict(g, a, d)))
        self._last_accel = np.asarray(accel, float)
        self._n['imu'] += 1
        self._last_input_t = t

        # The split, enforced here. The board owns attitude; without this
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
            self._board_R = _R_from_quat(msg.orientation.w, msg.orientation.x,
                                         msg.orientation.y, msg.orientation.z)
            # Into the pool frame before the filter sees it. The board's yaw
            # is boot-relative; once anchored, handing it over raw pins the
            # filter back to boot axes at 50 Hz while `frame_id` says `pool`.
            # Rz on the LEFT changes yaw only: R = Rz(yaw) Ry(pitch) Rx(roll)
            # in the board's Z-Y-X convention, so Rz(off) R keeps roll/pitch.
            R_meas = _Rz(self._yaw_offset_deg) @ self._board_R
            if not self._attitude_seeded:
                # The first attitude is an initialisation, not a correction.
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
                # An EVENT, so a replay from before the seed re-applies it.
                self._apply(t, lambda f, R=R_meas: setattr(f.X, 'R', R.copy()))
                self.get_logger().info(
                    f'[LOCAL] seeded attitude from the board: '
                    f'yaw {self._filter.X.yaw_deg():+.1f} deg')
            self._apply(t, lambda f, R=R_meas, s=self._attitude_sigma_deg:
                        f.update_attitude(R, sigma_deg=s))
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

    def _on_state(self, msg: MonglaState) -> None:
        t = _opt_stamp(msg)
        depth = float(msg.depth_m)
        if not math.isnan(depth):
            ok = self._apply(t, lambda f, d=depth, s=self._depth_sigma:
                             f.update_depth(d, sigma=s))
            self._n['depth' if ok else 'depth_refused'] += 1
        yaw = float(msg.yaw_deg)
        # NaN is the documented absence sentinel, and on srot it is now what a
        # board with an unhealthy BNO actually publishes. Feeding NaN into the
        # update would poison every state through the gain, silently.
        if self._use_yaw and not math.isnan(yaw):
            # The same board yaw as the attitude stream, so the same offset --
            # raw, it would drag an anchored filter back to boot axes.
            yaw = _wrap180(yaw + self._yaw_offset_deg)
            ok = self._apply(t, lambda f, y=yaw, s=self._yaw_sigma_deg:
                             f.update_yaw(y, sigma_deg=s))
            self._n['yaw' if ok else 'yaw_refused'] += 1

    def _flow_now(self, t_meas, vx: float, vy: float, var_x: float,
                  var_y: float):
        """Carry a flow sample to NOW, when nothing else accounts for its age.

        With retrodiction off (the default) A STAMP WAS IGNORED (issue #31).
        Flow is stamped at the middle of its baseline -- up to 375 ms back --
        and was compared with the CURRENT state at its own, overconfident
        variance. During a turn on a current, or any acceleration, the motion
        in between became innovation the filter believed: measured in
        `test_a_stale_flow_sample_no_longer_bends_velocity_on_a_turn` at
        4.5 cm/s, exactly |v| |w| age.

        Widening the variance alone was tried and does not work: flow is
        the only horizontal velocity observer, so a bias at any variance still
        wins in the end (4.50 -> 4.50 cm/s). The sample is instead MOVED to
        now (`RIEKF.body_velocity_now`) and keeps half the move as 1-sigma for
        the rate and acceleration it assumed held. With retrodiction on, it is
        applied at its own instant and none of this runs.
        """
        if (getattr(self, '_retro', None) is not None or t_meas is None
                or self._last_imu_t is None
                or getattr(self, '_last_accel', None) is None):
            return vx, vy, var_x, var_y
        now, d = self._filter.body_velocity_now(
            vx, vy, self._last_accel, self._last_imu_t - t_meas)
        return (float(now[0]), float(now[1]),
                var_x + (0.5 * float(d[0])) ** 2,
                var_y + (0.5 * float(d[1])) ** 2)

    def _on_flow(self, msg: TwistWithCovarianceStamped) -> None:
        """Downward optical flow: the DVL we do not have.

        X AND Y ONLY. A bottom-looking camera cannot see vertical velocity,
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
        t_meas = _opt_stamp(msg)
        vx_now, vy_now, var_x, var_y = self._flow_now(t_meas, v.x, v.y,
                                                      var_x, var_y)
        if not self._apply(t_meas,
                           lambda f, vx=vx_now, vy=vy_now, a=max(var_x, 1e-6),
                           b=max(var_y, 1e-6), r=self._flow_lever_arm:
                           f.update_body_velocity_xy(vx, vy, a, b, lever_arm=r)):
            # Older than the replay horizon: NOT fused, so not counted as
            # fused -- the aiding diagnostic read refused samples as aid.
            self._n['flow_refused'] += 1
            return
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

    def _report_aiding(self) -> None:
        """Publish whether horizontal position is observable, on change.

        Called from the publish timer rather than from an input handler,
        because the thing being reported is the ABSENCE of an input -- there is
        no callback for flow that did not arrive.
        """
        observed = True
        probe = getattr(self._filter, '_velocity_is_observed', None)
        if probe is not None:
            try:
                observed = bool(probe())
            except Exception:                              # noqa: BLE001
                observed = True      # an unreadable gate must not stop the node
        state = 'aided' if observed else 'unaided'
        if state == self._aiding_state:
            return
        self._aiding_state = state
        if state == 'unaided':
            self.get_logger().warning(
                '[LOCAL] UNAIDED: velocity is unobserved, so depth and yaw '
                'updates are being REFUSED (B-56) and horizontal position is '
                'dead reckoning. The pose still publishes and still looks '
                'healthy -- do not act on absolute position.')
        else:
            self.get_logger().info('[LOCAL] aiding restored: position observable')
        pub = getattr(self, '_pub_aiding', None)
        if pub is not None:
            pub.publish(String(data=state))

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
        centred = (math.isfinite(x) and math.isfinite(y)
                   and max(abs(x), abs(y)) < DEMAND_CENTRE_EPS)
        if not centred:
            self._centred_since = None
        elif getattr(self, '_centred_since', None) is None:
            self._centred_since = now

    def _maybe_model_aid(self) -> None:
        """When flow has gone quiet, aid velocity from the learned demand model.

        Only while flow is stale, on the same `FLOW_FRESH_S` the ZUPT uses, so
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
        self._apply(None, lambda f, p=p: f.update_body_velocity_xy(p[0], p[1], p[2], p[3]))
        self._n['model'] += 1

    def _maybe_zupt(self) -> None:
        """Stand still and the filter learns from it -- but only if it IS still.

        ZUPT turns a stationary interval into a direct observation of the
        velocity error that has accumulated, which keeps inertial growth
        linear rather than quadratic. It matters exactly when flow is absent:
        with flow running, a still hull already measures zero and this adds
        nothing.

        The caller owns the stationarity test, and getting it wrong is how a
        ZUPT ruins a filter -- declaring "still" during a slow constant-speed
        transit deletes real motion. So stillness is MEASURED, never assumed
        and never taken from mission intent: the hull station-keeping against
        a current is commanded still and is not. The board reports its own
        gyro and accelerometer at 50 Hz, which is the same reasoning that let
        us measure bench stillness rather than assume it.
        """
        if len(self._still) < self._still.maxlen:
            return
        now = time.monotonic()
        if now - self._last_flow_t < FLOW_FRESH_S:
            return                      # flow is live; it already says zero
        # The command veto: known, centred, and long enough to have stopped.
        # Unknown demand is NOT stillness -- on srot it is unknown precisely
        # while the board runs a move.
        since = getattr(self, '_centred_since', None)
        if (since is None or self._last_demand_t is None
                or now - self._last_demand_t > DEMAND_FRESH_S):
            return
        settle = max(STILL_WINDOW / 50.0,
                     DEMAND_SETTLE_TAUS * float(self._model.tau_s))
        if now - since < settle:
            return
        gyro = [g for g, _ in self._still]
        acc = [a for _, a in self._still]
        if max(gyro) > STILL_GYRO_RAD_S:
            return
        # The SPREAD of specific force, not its magnitude -- gravity is in
        # there and a tilted hull reads a large constant. Variation is motion.
        if (max(acc) - min(acc)) > STILL_ACCEL_SPREAD:
            return
        self._apply(None, lambda f, s=self._zupt_sigma: f.update_zero_velocity(sigma=s))
        self._n['zupt'] += 1
        self._still.clear()

    def _on_heading(self, msg: Float32) -> None:
        """An absolute world heading from the landmark anchor.

        This is what makes `frame_id` true. The board's yaw is excellent and
        is not a pool bearing: it is relative to boot, or to magnetic north
        through a reference that may not have locked (`MAGACC` 0.0, `YAW_REF`
        0.0, `COMP_SEEN` 0.0 on this hull, measured). Until an anchor arrives
        the estimate is in the board's frame and the output says `odom`; after
        it, the frame really is the course's and the output says `pool`.

        Publishing `pool` before that point would not be a labelling nicety:
        flow integrates into position through the SAME rotation, so every
        dead-reckoned metre would walk off along an offset nobody measured.

        An offset, not an update -- the defect this replaced. This used to be
        one `update_yaw` against a filter the board pins at 0.5 deg, 50 times a
        second. Any anchor more than ~5 deg from boot yaw was chi-square
        REJECTED; one that got in was pulled back to boot axes by the next
        attitude samples. And `_anchored` flipped either way, so `/mongla/odom`
        said `pool` while every number in it was still boot-relative.

        So the anchor is turned into what it actually is -- the angle between
        the board's axes and the pool's -- and kept: offset = anchor - the
        board's CURRENT raw yaw, every later board attitude is rotated by it
        (`_on_imu`), and the filter's state is rotated by the CHANGE in offset
        as an EVENT, so a retrodicted replay re-applies it at the same place in
        the sequence, exactly as the seed is. A later anchor replaces the
        offset and rotates by the difference.

        The pairing is by arrival. The message carries the hull's absolute
        heading at the moment the mission computed it, and it is paired here
        with the newest board sample. Mid-turn, the latency between the two is
        heading error. Carrying the mission's own `offset_deg` would remove
        that; this keeps the wire as it is.
        """
        heading = float(msg.data)
        if not math.isfinite(heading):
            return
        if self._board_R is None:
            # Refused, not held. An absolute heading means nothing without
            # the board yaw it was paired with, and a latched anchor delivered
            # to a node that has not yet heard the board may be minutes old --
            # the hull has turned since. A wrong heading zero is worse than
            # none, because nothing downstream can tell it is lost.
            self.get_logger().warning(
                f'[LOCAL] heading anchor {heading:+.1f} deg REFUSED: no board '
                f'attitude yet to pair it with. The output stays in `odom_ned`.')
            return
        board_yaw = math.degrees(math.atan2(self._board_R[1, 0],
                                            self._board_R[0, 0]))
        offset = _wrap180(heading - board_yaw)
        delta = _wrap180(offset - self._yaw_offset_deg)
        if not self._apply(None, lambda f, d=delta: f.rotate_world_yaw(d)):
            return
        self._yaw_offset_deg = offset
        self._n['yaw'] += 1
        if not self._anchored:
            self._anchored = True
            # Connect `pool` to the hull, once. The same claim the odometry
            # header has always made at this moment: pool AXES. The origin is
            # still where the filter started until a position fix moves it
            # (see `RIEKF.rotate_world_yaw`), so this is not a pool position.
            st = getattr(self, '_tf_static', None)
            if st is not None:
                st.sendTransform(frames.to_msg(frames.anchored_edge(),
                                               self.get_clock().now().to_msg()))
            self.get_logger().info(
                f'[LOCAL] heading anchored at {heading:+.1f} deg (board '
                f'{board_yaw:+.1f}, offset {offset:+.1f}): the output frame is '
                f'the POOL from here on.')

    def _on_floor_grid(self, msg: FloorHeading) -> None:
        """The floor's tile grid: a DRIFT BOUND on yaw, not a heading source.

        What this can and cannot do. A square grid is identical from four
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
        self._bound_yaw(float(msg.heading_deg), 90.0, 'grid', _opt_stamp(msg))

    def _on_lane_line(self, msg: FloorHeading) -> None:
        """The lane line's heading, modulo 180: the grid's drift bound, but
        a line is identical from two ends rather than four, so a wrong branch
        is 180 degrees away instead of 90 -- the bound is far harder to alias.
        """
        self._bound_yaw(float(msg.heading_deg), 180.0, 'lane', _opt_stamp(msg))

    def _bound_yaw(self, angle_deg: float, period_deg: float, key: str,
                   t=None) -> None:
        """Pull yaw onto a floor feature of symmetry `period_deg`, or refuse.

        `t` is the camera frame the angle was read from. The snap runs INSIDE
        the replayed update, against the yaw the filter had at that instant:
        during a turn the heading of now is somewhere else, and comparing the
        two pulled yaw toward where the hull had been (BUGS B-91).
        """
        if not self._anchored:
            return
        refused = []

        def bound(f, a=angle_deg, s=self._grid_sigma_deg):
            snapped = snap_to_grid(f.X.yaw_deg(), a,
                                   max_correction_deg=self._grid_max_corr_deg,
                                   period_deg=period_deg)
            if snapped is None:
                refused.append(True)
                return
            f.update_yaw(snapped, sigma_deg=s)

        before = self._filter.X.yaw_deg()
        if not self._apply(t, bound) or refused:
            # Older than the replay horizon, or a disagreement too large to be
            # drift: the wrong grid line, never applied.
            self._n[f'{key}_refused'] += 1
            return
        # What the floor moved goes into the offset, or it is undone. The
        # next board attitude, 20 ms later, would pull yaw straight back to
        # board + offset: a drift bound that bounds nothing. Folding the
        # accepted correction (the filter's own gain, so a 1 deg floor against
        # a 0.2 deg board moves a few percent a sample) into the offset makes
        # the board agree with it from then on. `moved` is measured on the
        # yaw of NOW, after the replay, which is what the offset is added to.
        moved = _wrap180(self._filter.X.yaw_deg() - before)
        self._yaw_offset_deg = _wrap180(self._yaw_offset_deg + moved)
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
        if not self._apply(_opt_stamp(msg),
                           lambda f, xy=(msg.point.x, msg.point.y), s=sigma:
                           f.update_position(xy, sigma=s)):
            return                      # older than the replay horizon: refused
        self._n['fix'] += 1

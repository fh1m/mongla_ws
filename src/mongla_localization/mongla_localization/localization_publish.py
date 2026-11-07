"""What the localization node publishes: the odometry and its diagnostic line.

One mixin of `LocalizationNode` (`localization_node.py`).
"""

from __future__ import annotations

import math

import numpy as np

from nav_msgs.msg import Odometry


from mongla_localization import frames
from mongla_localization.inekf import RIEKF, skew


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
    # P IS THE COVARIANCE OF THE RIGHT-INVARIANT ERROR, NOT OF POSITION
    # (issue #30). True = exp(xi) * estimate, so to first order
    #     p_true = p + dtheta x p + dp     ->  dp_pos = dp - skew(p) dtheta
    # and copying the dp block alone left out what the attitude uncertainty
    # does to a hull away from the origin: 1 deg of sigma at 10 m is 17 cm of
    # position sigma nobody reported. The orientation rows are the world-frame
    # dtheta itself (ROS: rotations about the parent's fixed axes), and the
    # position-orientation cross terms are filled, not dropped.
    J = np.zeros((6, 9))
    J[0:3, 0:3] = -skew(filt.X.p)
    J[0:3, 6:9] = np.eye(3)
    J[3:6, 0:3] = np.eye(3)
    C = J @ P[0:9, 0:9] @ J.T
    # The twist is published in the BODY frame, so its covariance must be too.
    # The filter's velocity block is WORLD-frame; copying it unrotated swaps
    # the x and y variances on a 90 degree heading, and every consumer that
    # weights by axis (the board's distance leg, flow position) gets the
    # other axis's confidence.
    # And unlike position it needs no attitude term: v_body_true =
    # R^T exp(-dtheta) (exp(dtheta) v + dv) = R^T (v + dv) to first order --
    # the dtheta parts cancel, so R^T P_vv R IS the body-velocity covariance.
    R = filt.X.R
    Pv_body = R.T @ P[3:6, 3:6] @ R
    for r in range(6):
        for c in range(6):
            msg.pose.covariance[r * 6 + c] = float(C[r, c])
    for r in range(3):
        for c in range(3):
            msg.twist.covariance[r * 6 + c] = float(Pv_body[r, c])



class LocalizationPublishMixin:
    # ---- output ---------------------------------------------------------

    def _publish(self) -> None:
        self._maybe_model_aid()          # 10 Hz: one aid per output, not per IMU
        # Report observability alongside the pose, so the two can never
        # disagree about which tick they describe.
        self._report_aiding()
        st = self._filter.X
        m = Odometry()
        # The stamp is the latest input's, not `now()`. Every input to this
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
        m.header.frame_id = frames.POOL if self._anchored else frames.ODOM
        # Both frames are NED (z down: position.z is +depth) with an FRD child.
        # `odom_ned`, not `odom`: REP-103 reserves `odom` for ENU, and a tool
        # reading this under `odom` drew the hull CLIMBING as it dived (#19).
        m.child_frame_id = frames.BODY
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
        # The same pose as the tree's one moving edge. ALWAYS from odom_ned:
        # after the anchor the filter is in pool axes, and `map -> odom` (sent
        # once, at the anchor) is what makes `pool` reachable -- see frames.py.
        tf = getattr(self, '_tf', None)
        if tf is not None:
            t = frames.to_msg(frames.Edge(
                frames.ODOM, frames.BODY,
                (float(st.p[0]), float(st.p[1]), float(st.p[2])),
                (qx, qy, qz, qw)), m.header.stamp)
            tf.sendTransform(t)

    def _diagnose(self) -> None:
        n = self._n
        # The one degradation an operator must be told about. With attitude
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
            f"gated={self._filter.gated} "
            f"rej={self._filter.rejected} brk={self._filter.lockout_breaks} "
            + (f"late={self._retro.late} replay={self._retro.replayed} "
               f"too_old={self._retro.refused} " if getattr(self, '_retro', None) else '')
            + f"| "
            f"yaw={self._filter.X.yaw_deg():+.1f} deg "
            f"p=({self._filter.X.p[0]:+.2f},{self._filter.X.p[1]:+.2f},"
            f"{self._filter.X.p[2]:+.2f}) m")

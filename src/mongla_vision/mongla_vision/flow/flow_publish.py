"""FlowVelocityNode, part 4 of 5: every topic the node writes, and its periodic log.
"""
import math
import time
from std_msgs.msg import Float32, Float32MultiArray, UInt8
from geometry_msgs.msg import TwistWithCovarianceStamped
from mongla_vision.flow.flow_timing import exposure_correction_s, interval_midpoint


class FlowPublishMixin:
    """Methods of `FlowVelocityNode`; see `flow_node.py` for the node."""

    # ── outputs ─────────────────────────────────────────────────────────────
    def _stamp_for(self, t_end: float, dt: float) -> float:
        """When this velocity actually happened, in host time.

        THREE CORRECTIONS, and the first is the biggest error in the whole
        pipeline:

        1. The midpoint. Flow measures DISPLACEMENT over an interval, so
           dividing by dt gives the AVERAGE velocity across it -- which belongs
           at the middle. PX4 defines its own flow delay parameter exactly so:
           "to the middle of the optical flow integration interval". Our
           baseline is ADAPTIVE and stretches to 0.75 s when the hull is slow,
           so stamping at the END is wrong by up to 375 ms, and worst precisely
           during station-keeping.

        2. Half the exposure. The standard image timestamp is mid-exposure.
           Ours reads 15.7 ms on AUTO exposure -- 7.85 ms, larger on its own
           than the 6 ms tolerance, and it MOVES with the light. Its SIGN is
           the driver's stamp source: uvcvideo stamps the START, so this ADDS
           (issue #34; it used to subtract, a full exposure wrong).

        3. td. Whatever fixed lag remains, from `TimeOffset`.
        """
        t = interval_midpoint(t_end - dt, t_end) if self._mid else t_end
        t += exposure_correction_s(self._exposure_units, self._stamp_src)
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

        # Covariance from the data, not a constant. velocity = scale * mean
        # flow, so var(v) = scale^2 * var(flow) / N -- the standard error of
        # the mean, which is the honest statement of how well N noisy points
        # pinned one displacement. A constant here would tell a filter the
        # estimate is equally good on a textured floor and a bare one.
        scale = h / (self._f_px * dt)
        sigma_px = (disp if disp is not None else 1.0) / math.sqrt(
            max(n_used, 1))
        var = (scale * sigma_px) ** 2
        var += self._model_variance(v, h)
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
                f'median_fallback={self._n_median_fallback} '
                # Both were counted and never surfaced. rot_anchor is how
                # often rotation ripeness fired -- the ONLY way to tell at the
                # pool whether the cap is doing anything (it buys intervals,
                # not accuracy, so it is invisible in the distance). td_rej is
                # how often a time-offset fit was refused; a rising count with
                # a flat td means the estimator is running and never trusted.
                f'rot_anchor={self._n_rot_anchor} '
                f'td_rej={self._td_rejected}')
        else:
            self.get_logger().info(
                f'[FLOW ] REFUSING: {self._last_reason}  '
                f'(q=0, {self._n_refused} refused / {self._n_ok} used)  '
                f'skipped={dropped} rot_anchor={self._n_rot_anchor} '
                f'td_rej={self._td_rejected}')

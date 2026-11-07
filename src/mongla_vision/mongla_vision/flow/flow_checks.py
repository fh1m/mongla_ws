"""FlowVelocityNode, part 2 of 5: the checks that decide a measurement may be published.

Each one can refuse a frame. A refusal publishes nothing, never a confident zero.
"""
import math
import time
import numpy as np
from std_msgs.msg import UInt8
from mongla_vision.flow.flow_math import (
    CAUSTIC_NCC_MIN,
    patch_ncc_median,
    forward_backward_error,
    interp_rate,
    solve_planar_motion,
)
from .flow_constants import (  # noqa: F401
    FLOW_ROT_RESIDUAL_CALIBRATED,
    FLOW_SCALE_SIGMA_FRAC,
    _LK_PARAMS,
)


class FlowChecksMixin:
    """Methods of `FlowVelocityNode`; see `flow_node.py` for the node."""

    def _drop_inconsistent_tracks(self, track, nxt, status, n_used):
        """Tracks that do not come back to where they started are not counted.

        -> (status, n_used)"""
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
            fb = forward_backward_error(self._anchor_gray, track,
                                        self._anchor_pts, nxt, _LK_PARAMS)
            if fb is not None:
                st = np.asarray(status).reshape(-1).astype(bool)
                st &= (fb <= self._fb_px)
                status = st.astype(np.uint8).reshape(-1, 1)
                n_used = int(st.sum())
        return status, n_used

    def _caustics_fooled_us(self, gray, t, track, nxt, status) -> bool:
        """True when the motion is the light on the floor, not the hull.

        The interval is then refused and the anchor retaken."""
        # ⛔ SUN ON A BARE FLOOR. Erosion strips the filaments, but on a floor
        # with no dark texture of its own what is left are the caustic CELLS,
        # which boil between frames and still move with the waves. A rigid
        # floor keeps its patches; cells do not. Refuse rather than publish
        # the waves' speed as the hull's.
        if getattr(self, '_anchor_caustic', False) and nxt is not None:
            keep = np.asarray(status).reshape(-1).astype(bool)
            ncc = patch_ncc_median(self._anchor_gray, track,
                                   np.asarray(self._anchor_pts).reshape(-1, 2)[keep],
                                   np.asarray(nxt).reshape(-1, 2)[keep])
            if ncc < CAUSTIC_NCC_MIN:
                self._anchor(gray, t)
                self._refuse(f'sun caustics over a floor with no texture of its '
                             f'own (patch NCC {ncc:.2f} < {CAUSTIC_NCC_MIN}): the '
                             f'motion is the waves, not the hull')
                return True
        return False

    def _planar_refit(self, gray, nxt, status, dt, flow, n_used, disp):
        """Translation, yaw and scale fitted together, when enabled.

        -> (flow, n_used, disp)"""
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
        return flow, n_used, disp

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
                            f'{self._td_est.quality:.2f}, peak '
                            f'{self._td_est.peak_correlation:.2f})')
                        # ⭐ THE PEAK, NOT ONLY THE QUALITY. Quality is height x
                        # prominence, so a low value means EITHER the two yaw
                        # series barely correlate OR they correlate broadly and
                        # the lag is poorly determined. Those want opposite
                        # responses -- distrust the sensor, or lengthen the
                        # window -- and the log could not tell them apart. The
                        # peak height was already computed and exposed by a
                        # property nothing read.

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

    def _model_variance(self, v, h) -> float:
        """Variance the fit cannot see: scale and un-removed rotation (#23)."""
        speed = math.hypot(float(v.vx), float(v.vy))
        var = (FLOW_SCALE_SIGMA_FRAC * speed) ** 2
        # The floor's own spread, while the height comes from the typed
        # constant rather than read off the floor (#35): v scales with h.
        if getattr(self, '_tile_height', None) is None and float(h) > 0.0:
            var += (speed * getattr(self, '_floor_sigma_m', 0.0) / float(h)) ** 2
        pr, rr = getattr(self, '_last_rates', (0.0, 0.0))
        for rate, gain in ((pr, self._gy), (rr, self._gx)):
            residual = 1.0 if gain == 0.0 else FLOW_ROT_RESIDUAL_CALIBRATED
            var += (residual * float(h) * float(rate)) ** 2
        return var

"""LockNode's inputs besides images and detections: attitude, odometry,
floor height and camera info, each with the freshness rule it is read under.
"""
import math
import time
import numpy as np
try:
    from mongla_interfaces.msg import MonglaState
except ImportError:                                              # noqa: BLE001
    MonglaState = None
from geometry_msgs.msg import PointStamped
from mongla_vision.anchor import loop_closure as _lc
from mongla_vision.anchor import health as _health


class LockInputsMixin:
    """Methods of `LockNode`; see `lock_node.py` for the node."""

    def _on_imu(self, msg) -> None:
        """Hull roll from the board's fused BNO085.

        The -1.0 convention is load-bearing. `auv_manager_node` sets
        `orientation_covariance[0] = -1.0` when the board cannot supply
        attitude -- the ROS "no data" marker -- and publishes a
        structurally-valid message anyway. Reading the quaternion without
        checking it would take (0,0,0,1) as level, which is a confident wrong
        answer on a rolled hull and exactly the failure that convention
        exists to prevent.
        """
        try:
            if float(msg.orientation_covariance[0]) < 0.0:
                self._roll_rad = float('nan')
                return
            q = msg.orientation
            x, y, z, w = (float(q.x), float(q.y), float(q.z), float(q.w))
            self._roll_rad = math.atan2(2.0 * (w * x + y * z),
                                        1.0 - 2.0 * (x * x + y * y))
            self._imu_t = time.monotonic()
        except Exception:                                        # noqa: BLE001
            self._roll_rad = float('nan')

    def _roll_fresh(self, max_age_s: float = 1.0) -> float:
        """Roll, or NaN when stale or never received -- same discipline as
        `_yaw_fresh`. A second-old roll on a rolling hull is a different
        attitude."""
        r = getattr(self, '_roll_rad', float('nan'))
        if not math.isfinite(r):
            return float('nan')
        if (time.monotonic() - getattr(self, '_imu_t', 0.0)) > max_age_s:
            return float('nan')
        return r

    def _on_state(self, msg) -> None:
        """The hull's heading, depth and armed state.

        NaN IS THE ABSENT VALUE AND IS KEPT AS ONE. The board suppresses a
        yaw it cannot stand behind, and `MonglaState` carries that through as
        NaN. Coercing it to 0.0 here would hand every downstream rung a
        confident heading of due north -- the repo's recurring defect, a
        plausible number standing in for an absent measurement.
        """
        y = float(msg.yaw_deg)
        self._yaw_deg = y if math.isfinite(y) else float('nan')
        d = float(msg.depth_m)
        self._depth_m = d if math.isfinite(d) else float('nan')
        self._armed = bool(msg.armed)
        self._state_t = time.monotonic()

    def _yaw_fresh(self, max_age_s: float = 1.0) -> float:
        """Yaw, or NaN when it is stale or was never received.

        Freshness matters more here than for most signals: a remembered world
        position is built from yaw AT THE MOMENT OF THE SIGHTING, and a yaw
        one second old on a turning hull is a different direction entirely.
        """
        if not math.isfinite(getattr(self, '_yaw_deg', float('nan'))):
            return float('nan')
        if (time.monotonic() - getattr(self, '_state_t', 0.0)) > max_age_s:
            return float('nan')
        return self._yaw_deg

    def _on_odom(self, msg) -> None:
        """The filter's own estimate: where it thinks it is, and how sure.

        Sigma is read, not assumed. It gates the bank search radius and it
        is half of every closure's own sigma, so a zero covariance here (the
        ROS default for "not filled in") would claim perfect knowledge. Absent
        is NaN, which every gate downstream refuses.
        """
        pp = msg.pose.pose.position
        self._odom_xy = (float(pp.x), float(pp.y))
        self._odom_z = float(pp.z)
        cov = msg.pose.covariance
        vx, vy = float(cov[0]), float(cov[7])
        self._odom_sigma = (math.sqrt(vx + vy)
                            if (vx > 0.0 and vy > 0.0) else float('nan'))
        self._odom_t = time.monotonic()

    def _note_anchor_health(self) -> None:
        """Say WHICH failure this is, when the bank stops matching.

        Reported, not acted on. The thresholds in `anchor/health.py` are
        DECLARED -- no port has been fouled on purpose -- so this logs a
        verdict for an operator and changes no control path. Wiring it to
        behaviour needs a measured bar first.
        """
        try:
            cur, best = self._anchor.last_lookup_scores()
        except Exception:                      # noqa: BLE001 -- never fatal
            return
        v = _health.assess(cur, best)
        if v.state == self._last_health:
            return
        self._last_health = v.state
        if v.state == _health.CAMERA:
            self.get_logger().warning(
                f'[LOCK ] the CAMERA looks degraded, not the scene: '
                f'{v.reason}. Searching for a better view cannot help.')
        elif v.state == _health.WORLD:
            self.get_logger().info(
                f'[LOCK ] the VIEW changed, the camera is fine: {v.reason}')

    def _on_floor_height(self, msg) -> None:
        """Measured height above the floor, from the tile grating."""
        r = float(msg.range)
        if r > 0.05 and math.isfinite(r):
            self._floor_h = r
            self._floor_h_t = time.monotonic()

    def _altitude_m(self) -> float:
        """Height above the floor. NaN when it cannot be known.

        The measured floor first. `flow_node` publishes `floor_height` from
        the tile grating, and refuses to publish the pool_depth arithmetic
        under that name because `pool_depth_m - |depth|` is a typed constant
        minus a depth, not a measurement of the floor. This agrees: the
        grating wins, and it must be FRESH -- a stale height is a different
        altitude wearing the right units.

        The pool_depth fallback stays, because an operator who has measured
        their pool may legitimately supply it. It says what it is, once.

        And there is no third option. Pixels become metres only with this
        number; a default would put a plausible value where a measurement is
        missing, the recurring defect in this codebase. The bench barometer
        read "not initialised" (section 37), so with neither source this
        returns NaN and every closure is refused.
        """
        if (self._floor_h == self._floor_h
                and (time.monotonic() - self._floor_h_t)
                <= self._FLOOR_H_MAX_AGE_S):
            return self._floor_h
        if not (self._pool_depth > 0.0):
            return float('nan')
        if self._odom_z != self._odom_z:
            return float('nan')
        alt = self._pool_depth - abs(self._odom_z)
        if alt <= 0.05:
            return float('nan')
        if not self._pool_depth_warned:
            self._pool_depth_warned = True
            self.get_logger().warn(
                f'[LOCK ] no floor_height; using pool_depth_m '
                f'({self._pool_depth:.2f} m) minus depth. That is a TYPED '
                f'CONSTANT minus a measurement, not a measured altitude, and '
                f'every closure offset is scaled by it.')
        return alt

    def _place_tick(self, gray, header, now: float) -> None:
        """Remember where we are, and recognise where we have been.

        Recognition FIRST: enrolling before asking would add the current frame
        to the bank and then find it, which is the self-closure this whole
        path is built to avoid.
        """
        if not self._closer_on or gray is None or self._anchor is None:
            return
        xy = self._odom_xy

        # ---- recognise ---------------------------------------------------
        if self._anchor.size:
            # The filter's own uncertainty is the search radius. A confident
            # filter refuses to consider a place 20 m away; a drifting one
            # widens until it gates on nothing, which is exactly right -- the
            # prior should stop helping precisely when it stops being one.
            radius = (3.0 * self._odom_sigma
                      if self._odom_sigma == self._odom_sigma else 0.0)
            bp = self._anchor.locate(gray, label=_lc.PLACE_LABEL,
                                     near=xy if radius > 0.0 else None,
                                     radius_m=radius)
            c = _lc.consider(
                bp,
                ref_age_s=(now - (self._places._last_t or now)),
                travel_m=self._places.travel_since(xy),
                ref_sigma_m=(self._odom_sigma
                             if self._odom_sigma == self._odom_sigma else 1.0),
                m_per_px=self._m_per_px(),
                bank_size=self._anchor.size,
                min_inliers=int(self._closure_cfg['min_inliers']),
                min_age_s=float(self._closure_cfg['min_age_s']),
                min_travel_m=float(self._closure_cfg['min_travel_m']),
                max_offset_m=float(self._closure_cfg['max_offset_m']))
            if c.ok:
                self._closures += 1
                m = PointStamped()
                # The matched frame's header, on the board clock -- not now().
                # A fix stamped on arrival is replayed at the wrong instant by
                # the filter's retrodiction, which is the one thing that makes
                # a late measurement safe.
                m.header = header
                m.point.x, m.point.y = float(c.xy[0]), float(c.xy[1])
                m.point.z = float(c.sigma)      # the producer's own sigma
                self._fix_pub.publish(m)
                self.get_logger().info(
                    f'[LOCK ] loop closure {self._closures}: ref {c.index}, '
                    f'{c.reason}, sigma {c.sigma:.2f} m')
            else:
                # Counted, not dropped. A closure path that silently never
                # fires is indistinguishable from one that was never wired up.
                key = c.reason.split(' -- ')[0].split('(')[0].strip()
                self._closure_refusals[key] += 1
                n = sum(self._closure_refusals.values())
                if n % 300 == 0:
                    top = ', '.join(f'{k} x{v}' for k, v in
                                    self._closure_refusals.most_common(3))
                    self.get_logger().info(
                        f'[LOCK ] no closure in {n} looks: {top}')

        # ---- remember ----------------------------------------------------
        if self._places.should_enrol(now, xy):
            # WHOLE FRAME, no ROI. The ~100-inlier bar was derived on
            # whole-frame downward references (section 24); cropping would
            # repeat section 25, where a bar measured on one configuration was
            # shipped against another and silently never fired.
            r = self._anchor.enrol(gray, roi=None, det_conf=1.0,
                                   label=_lc.PLACE_LABEL, position=xy)
            if r.accepted:
                self._places.note(now, xy)
                self.get_logger().info(
                    f'[LOCK ] place {self._anchor.size}: {r.keypoints} kp '
                    f'at ({xy[0]:.2f}, {xy[1]:.2f})')

    def _m_per_px(self) -> float:
        """Metres per BACKEND pixel on the floor. NaN without an altitude.

        THE RECTIFIED K, NOT THE AIR ONE. `_on_info` already scales the
        intrinsics to the backend's own grid -- which is the grid the
        homography is defined on -- and `rectifier_for` returns the K that
        matches the flat-port correction. Using the air focal length underwater
        leaves a clean 1/n scale error: every offset 33 % short, with a
        plausible number and no warning. That is the exact defect
        `optics.py` exists to prevent, and a loop closure is precisely where a
        silent 33 % would be believed.
        """
        alt = self._altitude_m()
        if alt != alt:
            return float('nan')
        K = self._K_rect if getattr(self, '_K_rect', None) is not None \
            else self._K
        if K is None:
            return float('nan')
        f = float(K[0, 0])
        return alt / f if f > 0.0 else float('nan')

    def _on_info(self, msg):
        """K, scaled to the anchor backend's resolution.

        The homography and the correspondences live in backend pixels, so a
        full-resolution K would scale every recovered angle -- silently, with
        no error and a plausible number. Same trap as the anchor's ROI.
        """
        k = list(msg.k)
        if len(k) < 9 or k[0] <= 0.0 or self._anchor is None:
            return
        sx = self._anchor._be.w / float(msg.width or 1)
        sy = self._anchor._be.h / float(msg.height or 1)
        K = np.array([[k[0] * sx, 0.0, k[2] * sx],
                      [0.0, k[4] * sy, k[5] * sy],
                      [0.0, 0.0, 1.0]], np.float64)
        # CameraInfo arrives with EVERY FRAME, so rebuilding here would
        # reallocate the rectifier and re-log at camera rate. Only a genuine
        # change of intrinsics is an event; anything else is the same K again.
        # D travels with K (issue #24): the lens comes off before the port.
        # It acts on normalised coordinates, so it needs no rescaling here.
        D = tuple(float(v) for v in (msg.d or ()))
        if (self._K is not None and np.array_equal(K, self._K)
                and D == getattr(self, '_D', None)):
            return
        self._K = K
        self._D = D

        # The rectified points need the rectified K. `rectify` re-projects
        # each ray through `f_ref`, which defaults to `fx * n` -- so a point at
        # water angle tw lands at radius `f_ref * tan(tw)`, a true pinhole of
        # focal `f_ref`. Feeding those points to PnP with the AIR K would leave
        # a clean 1/n scale error: every range 33 % short, with a plausible
        # number and no warning. Same class as the fx!=fy aspect defect the
        # rectifier's own docstring records.
        # One place builds the (rectifier, matching K) pair -- see
        # `optics.rectifier_for`. `pnp_node` reads the same helper, so the two
        # cannot disagree about what optics a point set is in.
        from mongla_vision.optics import rectifier_for
        self._rect, self._K_rect, note = rectifier_for(self._K, self._medium, self._D)
        self.get_logger().info(f'[LOCK ] {note}')

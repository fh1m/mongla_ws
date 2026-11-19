"""LockNode's inputs besides images and detections: odometry, floor
height and camera info, each with the freshness rule it is read under.
"""
import math
import time
import numpy as np
from geometry_msgs.msg import PointStamped
from mongla_vision.anchor import loop_closure as _lc
from mongla_vision.anchor import health as _health

# /mongla/odom comes from the localiser's 10 Hz timer: worst gap 105.76 ms on
# Kutum under the whole vision graph (measured-bars section 91), and a live
# filter's stamp moves with every one. Five periods without a new stamp means
# the localiser or its inputs stopped, not jitter.
ODOM_MAX_AGE_S = 0.5

class LockInputsMixin:
    """Methods of `LockNode`; see `lock_node.py` for the node."""

    # A floor height older than this is not this vehicle's altitude any more.
    _FLOOR_H_MAX_AGE_S = 2.0
    _ODOM_MAX_AGE_S = ODOM_MAX_AGE_S

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
        # The localiser publishes on a timer whether or not its inputs still
        # arrive, stamping each message with its newest input (50 Hz IMU). So
        # the age that matters is since the STAMP last moved: on-time messages
        # with a frozen stamp are a position that stopped updating.
        st = msg.header.stamp
        stamp = (int(st.sec), int(st.nanosec))
        if stamp != self._odom_stamp:
            self._odom_stamp = stamp
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
        if now - self._odom_t > self._ODOM_MAX_AGE_S:
            # A position the localiser stopped sending would be searched
            # around and stored as where a place is: refused, and counted
            # with the other refusals so it is seen.
            self._count_refusal('stale odometry')
            return
        xy = self._odom_xy
        self._places.step(xy)

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
            # Age, travel and sigma of the place the bank MATCHED, as it was
            # when that place was taken: a closure is only as good as the
            # position the reference carries, and the self-closure guards ask
            # whether the vehicle has been away from THAT place.
            ref_age, ref_travel, ref_sigma = self._places.since(
                getattr(bp, 'ref_position', None), now)
            c = _lc.consider(
                bp,
                ref_age_s=ref_age,
                travel_m=ref_travel,
                ref_sigma_m=ref_sigma,
                m_per_px=self._m_per_px(),
                bank_size=self._anchor.size,
                centre=self._anchor.grid_centre)
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
                self._count_refusal(c.reason.split(' -- ')[0].split('(')[0].strip())

        # ---- remember ----------------------------------------------------
        # A place whose position the filter cannot put a sigma on would make
        # every closure onto it unsound, so it is not taken.
        if (self._odom_sigma == self._odom_sigma
                and self._places.should_enrol(now, xy)):
            # WHOLE FRAME, no ROI. The ~100-inlier bar was derived on
            # whole-frame downward references (section 24); cropping would
            # repeat section 25, where a bar measured on one configuration was
            # shipped against another and silently never fired.
            r = self._anchor.enrol(gray, roi=None, det_conf=1.0,
                                   label=_lc.PLACE_LABEL, position=xy)
            if r.accepted:
                self._places.note(now, xy, self._odom_sigma)
                self.get_logger().info(
                    f'[LOCK ] place {self._anchor.size}: {r.keypoints} kp '
                    f'at ({xy[0]:.2f}, {xy[1]:.2f})')

    def _count_refusal(self, key: str) -> None:
        """Counted, not dropped, and summarised every 300 looks."""
        self._closure_refusals[key] += 1
        n = sum(self._closure_refusals.values())
        if n % 300 == 0:
            top = ', '.join(f'{k} x{v}' for k, v in
                            self._closure_refusals.most_common(3))
            self.get_logger().info(f'[LOCK ] no closure in {n} looks: {top}')

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
        if len(k) < 9 or k[0] <= 0.0:
            return
        # Without a bank (off, or it failed to load) K stays on the camera's
        # own grid: the frame-edge warning needs it whether or not any
        # reference was stored.
        if self._anchor is not None:
            grid = (float(self._anchor._be.w), float(self._anchor._be.h))
        else:
            grid = (float(msg.width or 1), float(msg.height or 1))
        self._K_grid = grid
        sx = grid[0] / float(msg.width or 1)
        sy = grid[1] / float(msg.height or 1)
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

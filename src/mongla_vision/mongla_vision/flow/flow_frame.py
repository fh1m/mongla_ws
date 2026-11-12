"""The flow node's per-frame work: pick the points, wait for the motion, judge it.

`FlowFrameMixin` is the part of `FlowVelocityNode` that runs once per camera
frame. `_process` tracks the floor from the last anchor frame; `_is_ripe` decides
whether enough motion has built up to measure; `_evaluate` turns the motion
into a velocity, or refuses it. The node's wiring lives in `flow_node.py`.
"""
import math
import time


import cv2
import numpy as np

from mongla_vision.flow.flow_math import (
    flow_dispersion,
    height_above_floor,
    integrate_rate,
    interp_rate,
    robust_flow,
)
from mongla_vision.flow.flow_features import (
    CAUSTIC_TOPHAT_MAX,
    caustic_score,
    detect_corners,
    suppress_caustics,
)
from mongla_vision.flow.flow_velocity import flow_velocity
from .flow_constants import _FEATURE_PARAMS, _LK_PARAMS, _MIN_TRACKS, HEIGHT_STALE_S


class FlowFrameMixin:
    """Per-frame tracking and evaluation; mixed into `FlowVelocityNode`."""

    def _bucketed_corners(self, gray):
        """Corners for the anchor: spread over a grid, quality bar adaptive.

        See `flow_features.detect_corners`. A fixed Shi-Tomasi threshold is a fixed
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
        self._publish_tracks(nxt, status, t)
        self._anchor(gray, t)
        if dt > 0.0:
            self._evaluate(flow, disp, dt, t, n_used)
        else:
            self._refuse('non-positive baseline')

    # ── the checks inside one `_process` interval ────────────────────────────
    # Each was a span of `_process` until 2026-11-07; code and comments moved
    # verbatim.

    def _anchor(self, gray, t) -> None:
        # Sun caustics: decided once per anchor (see flow_features.CAUSTIC_*).
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

"""The vision side of the node: the vision state the verbs read, and the uplinks to the board.

Mixed into `AUVManagerNode` (auv_manager_node.py), which owns the board,
the publishers and the state these methods read.
"""
from __future__ import annotations

import math
import time

from mongla_control.bearing import bearing_from_normalised
from mongla_control.fc.srot_protocol import uplink_class_num as srot_uplink_class_num
from mongla_vision import wait_vision_state_ready

from .vision_state import VisionState


class VisionLinkMixin:
    def _vision_state_for(self, camera: str):
        """Return (and build on first call) the VisionState for `camera`.

        The control loop reads ``/detections`` directly (the topic the HUD
        shows); the tracker keeps running for display only. Subscriptions
        stay alive for the rest of the process lifetime so repeat vision_*
        goals don't pay the preflight wait twice. The FIRST build for a
        camera requires a live detection (``require_detection``) so a goal
        fails fast with a clear "no boxes" rather than silently chasing an
        empty cache.
        """
        cache_key = camera

        with self._vision_lock:
            cached = self._vision_states.get(cache_key)
            if cached is not None:
                return cached
            self.get_logger().info(
                f'[VST  ] building VisionState for camera={camera!r}')
            vstate = VisionState(self, camera=camera,
                                 logger=self.get_logger())

        # Preflight outside the lock — it just polls VisionState's diags.
        # The FIRST build waits (up to 10 s) for a live detection so the
        # pipeline has time to warm up before the loop starts. The loop
        # itself tolerates no-detection (grace -> LOST -> DSL fallback), so
        # we cache the state either way and never pay this wait twice.
        try:
            wait_vision_state_ready(
                vstate, timeout=10.0, require_detection=True,
                log=self.get_logger())
        except Exception as exc:
            self.get_logger().warning(
                f'[VST  ] preflight for {camera!r} did not pass within '
                f'10s: {exc!r}; first goal starts cold (loop will search '
                f'via its fallback)')

        with self._vision_lock:
            # Check again under lock in case a concurrent goal built the same state.
            if cache_key not in self._vision_states:
                self._vision_states[cache_key] = vstate
            winner = self._vision_states[cache_key]
        # ⛔ THE LOSER IS CLOSED, NOT DROPPED (issue #21). Two concurrent first
        # goals for one camera each built a VisionState; the second was simply
        # not inserted, and its subscriptions stayed alive for the life of the
        # process.
        if winner is not vstate:
            try:
                vstate.close()
            except Exception as exc:          # noqa: BLE001 -- best effort
                self.get_logger().warning(
                    f'[VST  ] duplicate VisionState for {camera!r} not '
                    f'closed: {exc!r}')
        return winner

    def _live_vision_state(self):
        """A VisionState from the pool, or None before any vision goal has run.

        ⛔ THIS EXISTS BECAUSE `self.vision` NEVER DID. `_detection_rate_hz`
        read `getattr(self, 'vision', None)` and nothing on this class ever
        assigns `self.vision` -- VisionStates live in the `_vision_states` pool,
        keyed by camera. So the read always missed, the rate was always None,
        and the `detector` reporter -- which IS registered -- could only ever
        say UNKNOWN. The D16 signal it was built to carry (an aborted detector
        keeps every topic alive; only the RATE changes) never fired once.

        The defensive `getattr` is what hid it: a missing attribute returned a
        plausible None instead of raising, so the reporter degraded silently
        into a reporter that cannot report.

        Preference order: the pool is usually one entry; when a mission has
        touched both cameras, prefer 'forward' as the mission camera and fall
        back to whatever is there. Absence stays None -- the reporters render
        that as UNKNOWN, which is the honest answer before any goal has run.
        """
        pool = getattr(self, '_vision_states', None) or {}
        if not pool:
            return None
        return pool.get('forward') or next(iter(pool.values()))

    def _detection_rate_hz(self):
        """Detections per second, or None if we are not subscribed at all.

        THE SIGNAL D16 NEEDED. A detector that aborts stays alive with every
        topic present; the rate is the only thing that changes, and nothing was
        watching it.
        """
        vs = self._live_vision_state()
        st = getattr(vs, 'stats', None) if vs else None
        if st is None:
            return None
        try:
            return float(st().get('det_hz'))
        except Exception:                       # noqa: BLE001
            return None

    def _publish_flare_order(self) -> None:
        """Republish the board's latched flare order, if it has one.

        Nothing is published until an order exists; a mission reads silence as "no
        order", never a default. latched_at is host wall-clock, derived from the board's
        own FLARE_AGE, so a consumer can refuse an order from before its mission started.
        """
        if self.flare_order_publisher is None:
            return
        order = self.fc.flare_order()
        if order is None:
            return
        import json
        latched_at = time.time() - order['age_s']
        msg = self._String()
        msg.data = json.dumps({'colours': list(order['colours']),
                               'nonce': order['nonce'], 'latched_at': latched_at})
        self.flare_order_publisher.publish(msg)
        key = (order['colours'], order['nonce'])
        if key != self._flare_key:
            self._flare_key = key
            self.get_logger().info(
                f"[ORDER] flare order {'-'.join(order['colours'])} "
                f"nonce {order['nonce']} (latched {order['age_s']:.1f} s ago)")

    def _vision_uplink_tick(self) -> None:
        """Send ONE LANDING_TARGET for the currently selected target, or nothing.

        SENDING NOTHING IS THE LOSS SIGNAL. `VISION_API.md` §1 has no "lost"
        flag -- a detector that sees nothing simply stops sending, and the board
        ages the last bearing out. So this must not re-send a stale sample to
        "hold" a target: that is indistinguishable from a live one on the wire
        and defeats the board's staleness timer, which is the whole safety
        mechanism on this path. Hence the early returns rather than a cached
        last-good value.
        """
        cam = str(self.get_parameter('vision_uplink_camera').value).strip()
        # Say so ON CHANGE. Without this the operator sets the parameter, gets
        # a success, and has nothing telling them whether anything started --
        # which is indistinguishable from the startup-only bug this replaced.
        if cam != getattr(self, '_uplink_cam_logged', None):
            self._uplink_cam_logged = cam
            self.get_logger().info(
                f'[VIS  ] uplink camera -> {cam!r}' if cam else
                '[VIS  ] uplink idle (vision_uplink_camera cleared)')
        if not cam:
            return
        vstate = self._vision_state_for(cam)
        if vstate is None:
            return
        want = str(self.get_parameter('vision_uplink_class').value).strip()
        sample = vstate.bbox_error(want)
        if sample is None:
            return
        w, h = vstate.image_size()
        K, D = vstate.calibration()
        # ⛔ The index comes from `mongla_vision.optics`, which exists to be
        # "the ONE place the water refractive index lives" (B22). `bearing.py`
        # lives in mongla_control, which must NOT depend on mongla_vision -- so
        # the caller supplies it rather than the library holding a second copy.
        b = bearing_from_normalised(
            sample.ex, sample.ey, sample.w_frac, sample.h_frac,
            width=w, height=h, K=K, D=D, n_medium=self._uplink_n())
        if b is None:
            # No calibration and no FOV: refuse rather than invent a bearing.
            # An uncalibrated guess on this wire is a confident wrong heading.
            if not getattr(self, '_uplink_warned', False):
                self._uplink_warned = True
                self.get_logger().warn(
                    f'[VIS  ] uplink idle: {cam} has no usable CameraInfo.k and no '
                    f'FOV, so a pixel offset cannot become a bearing. Set the '
                    f'camera_node `calibration` param.')
            return
        if not b.calibrated and not getattr(self, '_uplink_fov_warned', False):
            self._uplink_fov_warned = True
            self.get_logger().warn(
                '[VIS  ] uplink using the FOV fallback, not the calibration: '
                'bearings carry the linear-approximation error (up to ~2.5 deg '
                'on our measured lens, and ~1.3 deg at frame centre from the '
                'off-axis principal point).')
        try:
            # The frozen class map, not the detector's own index: `class_id` is
            # a property of whichever model is loaded and means something
            # different for every one. Our two senders disagreed about this
            # field -- this tick sent a hardcoded 0 while the uplink check sent
            # `d.class_id` -- so neither was a wire contract.
            self.fc.send_landing_target(
                b,
                target_num=srot_uplink_class_num(
                    getattr(sample, 'class_name', '') or want),
                coasted=bool(getattr(sample, 'coasted', False)),
                gap_age_s=float(getattr(sample, 'age_s', 0.0) or 0.0),
                board_capture_s=self._board_capture_s(sample))
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn(f'[VIS  ] landing_target send failed: {exc}')

    def _on_odom_uplink(self, msg) -> None:
        """Forward one RIEKF body velocity to the board, stamped on ITS clock.

        The odom stamp is the filter's latest INPUT instant on this host's wall
        clock (localization maps every input through the same board<->host
        fit), so the inverse lands it on the board's `time_boot_ms` base.
        Nothing is sent until that fit is good: an unstamped velocity would be
        integrated at the wrong instant.
        """
        if not getattr(self, '_imu_clock_ok', False):
            return
        host_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if host_s <= 0.0:
            return
        if bool(self.get_parameter('position_uplink').value):
            self._send_position_uplink(msg, self._imu_clock.to_board(host_s))
        if not bool(self.get_parameter('velocity_uplink').value):
            return
        cov = msg.twist.covariance
        try:
            sent = self.fc.send_speed_estimate(
                float(msg.twist.twist.linear.x), float(msg.twist.twist.linear.y),
                float(cov[0]), float(cov[7]),
                self._imu_clock.to_board(host_s))
        except Exception as exc:                            # noqa: BLE001
            self.get_logger().warn(f'[VEL  ] speed_estimate send failed: {exc}')
            return
        if sent:
            self._vel_uplink_n += 1
            if self._vel_uplink_n == 1:
                self.get_logger().info(
                    '[VEL  ] velocity uplink live: RIEKF body velocity -> board '
                    '(VISION_SPEED_ESTIMATE, board-clock stamped)')

    def _send_position_uplink(self, msg, board_s) -> None:
        """One RIEKF pose to the board, with a reset counter that is honest.

        The counter increments when the frame changes (odom -> pool, when the
        heading anchor lands) and when the position jumps further than any
        motion could explain between two 10 Hz estimates (a prop fix).
        """
        from mongla_control.fc import srot_protocol as sp
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        frame = str(getattr(msg.header, 'frame_id', ''))
        last = getattr(self, '_pos_last', None)
        if last is not None:
            jump = math.hypot(p.x - last[1], p.y - last[2])
            if frame != last[0] or jump > sp.POS_RESET_JUMP_M:
                self._pos_reset = getattr(self, '_pos_reset', 0) + 1
        self._pos_last = (frame, p.x, p.y)
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c = msg.pose.covariance
        cov6 = [[c[r * 6 + k] for k in range(6)] for r in range(6)]
        try:
            sent = self.fc.send_position_estimate(p.x, p.y, p.z, yaw, cov6, board_s,
                                                  getattr(self, '_pos_reset', 0))
        except Exception as exc:                            # noqa: BLE001
            self.get_logger().warn(f'[POS  ] position_estimate send failed: {exc}')
            return
        if sent:
            self._pos_uplink_n = getattr(self, '_pos_uplink_n', 0) + 1
            if self._pos_uplink_n == 1:
                self.get_logger().info(
                    f'[POS  ] position uplink live: RIEKF pose -> board '
                    f'(VISION_POSITION_ESTIMATE, frame {frame!r}, board-clock stamped)')

    def _board_capture_s(self, sample):
        """The frame's capture instant on the BOARD's clock, or None.

        `sample.age_s` runs from capture on this host's clock; `ClockMap` holds
        the board<->host fit built from ATTITUDE.time_boot_ms, so the inverse
        puts the capture where the board's gyro history can find it. None until
        that fit is announced good, and None for a COASTED box, whose bearing
        is a prediction for now rather than an observation from the past --
        de-rotating it would count the same turn twice.
        """
        if getattr(sample, 'coasted', False) or not getattr(self, '_imu_clock_ok', False):
            return None
        age = float(getattr(sample, 'age_s', 0.0) or 0.0)
        if not (math.isfinite(age) and age >= 0.0):
            return None
        try:
            board_s = self._imu_clock.to_board(time.time() - age)
        except Exception:                                   # noqa: BLE001
            return None
        return board_s if math.isfinite(board_s) and board_s > 0.0 else None

    def _uplink_n(self) -> float:
        """Refractive index for the uplink bearing, from the ONE source of it.

        Refuses an unknown medium rather than defaulting: silently picking a
        medium here is picking a 33 % bearing error, and the failure would look
        like a mis-tuned gain rather than a units bug.
        """
        med = str(self.get_parameter('vision_uplink_medium').value or '').strip().lower()
        if med == 'air':
            return 1.0
        if med != 'water':
            if not getattr(self, '_uplink_medium_warned', False):
                self._uplink_medium_warned = True
                self.get_logger().error(
                    f'[VIS  ] vision_uplink_medium={med!r} is not water|air -- '
                    f'refusing to guess. Falling back to WATER, which is where '
                    f'the vehicle is; fix the parameter.')
        from mongla_vision.optics import N_WATER
        return float(N_WATER)

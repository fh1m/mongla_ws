#!/usr/bin/env python3
"""Vision verbs for the Duburi facade -- exactly two, pixel-native.

``vision_align`` centres the target on the selected axes (lat / yaw /
depth) each at a signed pixel offset; ``vision_move`` drives forward
until the target's bbox fills the frame to a ratio. Both delegate to the
two loops in :mod:`motion_vision` and ALWAYS return ``success=True`` --
the align/move outcome rides in ``Move.Result.final_value`` as an
integer code so the mission DSL branches on it and the action client
never raises on a miss.

Mixed into ``Duburi`` via multiple inheritance; uses only the base
facade helpers (``_command_scope``, ``_writers``, ``_resolve_vision_state``,
``_send_neutral_and_settle``, ``_ensure_alt_hold``, ``_suspend_heading_lock``,
``_retarget_heading_lock``, ``_current_heading``, ``_make_result``,
``_lock_active``, ``_abort_fn``) -- nothing rclpy-aware.
"""

from contextlib import nullcontext

from .motion_vision import (
    align_loop, move_loop, anchor_align_loop,
    KP_LAT_DEFAULT, KP_YAW_DEFAULT, KP_DEPTH_DEFAULT, KP_FORWARD_DEFAULT,
    KP_ANCHOR_LAT_DEFAULT, KP_ANCHOR_YAW_DEFAULT, KP_ANCHOR_DEPTH_DEFAULT,
    VISION_BRAKE_GAIN,
)


def _parse_channels(csv: str):
    """``'1,2'`` -> ``[1, 2]``. Empty / malformed -> ``[]`` (no fire)."""
    out = []
    for tok in (csv or '').split(','):
        tok = tok.strip()
        if not tok:
            continue
        try:
            out.append(int(float(tok)))
        except ValueError:
            continue
    return out


def _parse_axes(csv: str):
    """``'lat,yaw'`` -> ``{'lat','yaw'}``. Whitespace + case tolerant."""
    out = set()
    for token in (csv or '').split(','):
        name = token.strip().lower()
        if name:
            out.add(name)
    return out


class VisionVerbs:
    """Camera-driven verbs for the Duburi facade. Never instantiated alone."""

    # ================================================================== #
    #  vision_align -- centre on lat / yaw / depth at a pixel offset      #
    # ================================================================== #
    def vision_align(self, camera, target_class, axes,
                     offset_lat=0.0, offset_yaw=0.0, offset_depth=0.0,
                     err_px=40.0, duration=20.0, gain=30.0,
                     gain_lat=0.0, gain_yaw=0.0, gain_depth=0.0,
                     brake_off=False, brake_gain=0.0, hold_s=0.0,
                     hold_through_loss=False,
                     kp_lat=0.0, kp_yaw=0.0, kp_depth=0.0,
                     lost_grace_s=0.0, align_stable_frames=0.0):
        """Hold ``target_class`` at the requested pixel offset on each axis.

        ``axes`` is a CSV subset of ``lat,yaw,depth``; each active axis
        uses its matching ``offset_*`` (signed px, 0 = centre). Returns a
        Move.Result with ``success=True`` and the outcome code in
        ``final_value``.
        """
        axis_set = _parse_axes(axes) & {'lat', 'yaw', 'depth'}
        if not axis_set:
            return self._make_result(
                True, "vision_align: no active axis (axes empty)",
                final_value=2.0, error_value=0.0)   # TIMEOUT-ish no-op

        offsets = {}
        if 'lat' in axis_set:
            offsets['lat'] = float(offset_lat)
        if 'yaw' in axis_set:
            offsets['yaw'] = float(offset_yaw)
        if 'depth' in axis_set:
            offsets['depth'] = float(offset_depth)

        with self._command_scope('vision_align'):
            self._send_neutral_and_settle()
            vstate = self._resolve_vision_state(camera)
            is_downward   = camera in ('downward', 'sim_bottom')
            depth_sign    = -1 if is_downward else +1
            touches_yaw   = 'yaw' in axis_set
            touches_depth = 'depth' in axis_set
            if touches_depth:
                self._ensure_alt_hold('vision_align')

            stable = int(align_stable_frames) or 3
            self.log.info(
                f'[CMD  ] vision_align camera={camera!r} class={target_class!r} '
                f'axes={sorted(axis_set)} err={float(err_px):.0f}px '
                f'gain={float(gain):.0f}% dur={float(duration):.0f}s '
                f'hold={float(hold_s):.0f}s')

            # Ch4 arbitration: when yaw IS an align axis we suspend the lock
            # and the loop drives Ch4 itself. When yaw is NOT an axis but a
            # lock is live, the lock owns Ch4 -- tell the loop to write
            # lateral via send_rc_translation so it never clobbers the lock's
            # yaw stream (the lat/depth-only-align-fights-lock bug).
            release_yaw = self._lock_active() and not touches_yaw
            with self._suspend_heading_lock() if touches_yaw else nullcontext():
                outcome = align_loop(
                    pixhawk=self.pixhawk, vision_state=vstate,
                    target_class=target_class, axes=axis_set, offsets=offsets,
                    err_px=float(err_px), duration=float(duration),
                    gain=float(gain),
                    gain_lat=float(gain_lat) or float(gain),
                    gain_yaw=float(gain_yaw) or float(gain),
                    gain_depth=float(gain_depth) or float(gain),
                    brake=not bool(brake_off),
                    brake_gain=float(brake_gain) or VISION_BRAKE_GAIN,
                    hold_s=float(hold_s),
                    kp_lat=float(kp_lat) or KP_LAT_DEFAULT,
                    kp_yaw=float(kp_yaw) or KP_YAW_DEFAULT,
                    kp_depth=float(kp_depth) or KP_DEPTH_DEFAULT,
                    lost_grace_s=float(lost_grace_s) or 1.0,
                    hold_through_loss=bool(hold_through_loss),
                    align_stable_frames=stable,
                    depth_sign=depth_sign,
                    release_yaw=release_yaw,
                    writers=self._writers(), log=self.log,
                    abort_fn=self._abort_fn)
            if touches_yaw:
                self._retarget_heading_lock(self._current_heading())
            self._send_neutral_and_settle()
            return self._make_result(
                True, f'vision_align: {outcome.reason}',
                final_value=float(outcome.code),
                error_value=float(outcome.last_err_px))

    # ================================================================== #
    #  vision_move -- drive forward to a bbox fill ratio                  #
    # ================================================================== #
    def vision_move(self, camera, target_class, fwd_fill=95.0, mode='area',
                    maintain_px=0.0, maintain_on=False, hold_s=0.0,
                    err_px=40.0, duration=20.0, gain=30.0, gain_lat=0.0,
                    brake_off=False, brake_gain=0.0,
                    hold_through_loss=False,
                    kp_forward=0.0, kp_lat=0.0, lost_grace_s=0.0):
        """Drive forward until ``target_class`` fills ``fwd_fill`` % of the frame.

        ``mode`` is the fill metric (area/width/height). ``maintain_on``
        holds a lateral pixel offset (``maintain_px``) while driving;
        depth/yaw are never commanded. A ``fwd_fill`` <= 0 selects
        PASS-THROUGH: drive forward until the target is seen and then
        leaves the frame, plus a commit overshoot (used to go *through* a
        gate). Returns a Move.Result with ``success=True`` and the
        outcome code in ``final_value``.
        """
        # fwd_fill <= 0 is the pass-through sentinel (DSL move(fwd=None) /
        # CLI --fwd_fill -1). Anything > 0 is a real fill-% stop target.
        passthrough = float(fwd_fill) <= 0.0
        with self._command_scope('vision_move'):
            self._send_neutral_and_settle()
            vstate = self._resolve_vision_state(camera)
            # move_loop drives forward but never commands depth -- it relies
            # on ArduSub's onboard depth hold. Ensure ALT_HOLD so the approach
            # holds depth even when a mission jumps straight to vision_move
            # without a prior set_depth (from MANUAL the setpoint is dropped).
            self._ensure_alt_hold('vision_move')
            self.log.info(
                f'[CMD  ] vision_move camera={camera!r} class={target_class!r} '
                f'{"PASS-THROUGH" if passthrough else "fwd_fill=%.0f%%" % float(fwd_fill)} '
                f'mode={mode} '
                f'maintain={"%+.0fpx" % maintain_px if maintain_on else "off"} '
                f'hold={float(hold_s):.0f}s gain={float(gain):.0f}%')
            outcome = move_loop(
                pixhawk=self.pixhawk, vision_state=vstate,
                target_class=target_class,
                fwd_fill=float(fwd_fill) / 100.0,
                mode=str(mode) or 'area',
                passthrough=passthrough,
                maintain_px=float(maintain_px),
                maintain_on=bool(maintain_on),
                hold_s=float(hold_s),
                err_px=float(err_px), duration=float(duration),
                gain=float(gain),
                gain_lat=float(gain_lat) or float(gain),
                brake=not bool(brake_off),
                brake_gain=float(brake_gain) or VISION_BRAKE_GAIN,
                kp_forward=float(kp_forward) or KP_FORWARD_DEFAULT,
                kp_lat=float(kp_lat) or KP_LAT_DEFAULT,
                lost_grace_s=float(lost_grace_s) or 1.0,
                hold_through_loss=bool(hold_through_loss),
                release_yaw=self._lock_active(),
                writers=self._writers(), log=self.log,
                abort_fn=self._abort_fn)
            self._send_neutral_and_settle()
            return self._make_result(
                True, f'vision_move: {outcome.reason}',
                final_value=float(outcome.code),
                error_value=float(outcome.fill))

    # ================================================================== #
    #  anchor verbs -- XFeat geometric superglue lock                     #
    # ================================================================== #
    def vision_anchor_snap(self, camera):
        """Capture the current view as the anchor reference (non-blocking).

        Calls the manager-wired snap service on anchor_node; the node stores
        the NEXT frame as its reference. Returns success + final_value 1/0.
        """
        fn = getattr(self, 'anchor_snap_fn', None)
        if fn is None:
            return self._make_result(
                False, 'anchor_snap: no service wired (launch anchor:=true)',
                final_value=0.0, error_value=0.0)
        ok = bool(fn(camera))
        self.log.info(f'[CMD  ] vision_anchor_snap camera={camera!r} -> {ok}')
        return self._make_result(
            ok, 'anchor reference captured' if ok else 'anchor snap failed',
            final_value=1.0 if ok else 0.0, error_value=0.0)

    def vision_anchor_clear(self, camera):
        """Drop the anchor reference so the next snap starts fresh."""
        fn = getattr(self, 'anchor_clear_fn', None)
        if fn is None:
            return self._make_result(
                False, 'anchor_clear: no service wired', final_value=0.0)
        ok = bool(fn(camera))
        self.log.info(f'[CMD  ] vision_anchor_clear camera={camera!r} -> {ok}')
        return self._make_result(
            ok, 'anchor cleared' if ok else 'anchor clear failed',
            final_value=1.0 if ok else 0.0, error_value=0.0)

    def vision_anchor_align(self, camera, err_px=20.0, theta_thresh=0.05,
                            duration=30.0, gain=30.0,
                            gain_lat=0.0, gain_yaw=0.0, gain_depth=0.0,
                            brake_off=False, brake_gain=0.0, hold_s=0.0,
                            fire_channels='', min_inliers=0.0,
                            kp_lat=0.0, kp_yaw=0.0, kp_depth=0.0,
                            lost_grace_s=0.0, align_stable_frames=0.0):
        """Superglue the hull to the snapped reference (lat/yaw/depth).

        Geometric lock: drives lat from tx, yaw from theta, depth from ty until
        the live view re-superimposes on the reference within err_px / theta,
        holds for hold_s, optionally firing fire_channels once at first lock.
        Always returns success=True with the outcome code in final_value.
        """
        provider = getattr(self, 'anchor_state_provider', None)
        if provider is None:
            return self._make_result(
                True, 'vision_anchor_align: no anchor_state wired',
                final_value=3.0, error_value=0.0)   # NO_CAMERA-ish

        channels = _parse_channels(fire_channels)

        with self._command_scope('vision_anchor_align'):
            self._send_neutral_and_settle()
            astate = provider(camera)
            self._ensure_alt_hold('vision_anchor_align')   # ty -> depth nudge needs ALT_HOLD
            is_downward = camera in ('downward', 'sim_bottom')
            depth_sign  = -1 if is_downward else +1
            stable      = int(align_stable_frames) or 3

            self.log.info(
                f'[CMD  ] vision_anchor_align camera={camera!r} '
                f'err={float(err_px):.0f}px theta={float(theta_thresh):.3f} '
                f'gain={float(gain):.0f}% dur={float(duration):.0f}s '
                f'hold={float(hold_s):.0f}s fire={channels or "off"}')

            # Anchor always drives Ch4 (yaw from theta) -> always suspend the
            # heading lock for the duration, like align with a yaw axis.
            on_locked = (lambda: [self.fire(c) for c in channels]) if channels else None
            with self._suspend_heading_lock():
                outcome = anchor_align_loop(
                    pixhawk=self.pixhawk, anchor_state=astate,
                    err_px=float(err_px), theta_thresh=float(theta_thresh),
                    duration=float(duration), gain=float(gain),
                    gain_lat=float(gain_lat) or float(gain),
                    gain_yaw=float(gain_yaw) or float(gain),
                    gain_depth=float(gain_depth) or float(gain),
                    brake=not bool(brake_off),
                    brake_gain=float(brake_gain) or VISION_BRAKE_GAIN,
                    hold_s=float(hold_s),
                    match=(float(min_inliers) if float(min_inliers) > 0.0 else None),
                    kp_lat=float(kp_lat) or KP_ANCHOR_LAT_DEFAULT,
                    kp_yaw=float(kp_yaw) or KP_ANCHOR_YAW_DEFAULT,
                    kp_depth=float(kp_depth) or KP_ANCHOR_DEPTH_DEFAULT,
                    lost_grace_s=float(lost_grace_s) or 1.0,
                    anchor_stable_frames=stable,
                    depth_sign=depth_sign,
                    release_yaw=False,
                    on_locked=on_locked,
                    writers=self._writers(), log=self.log,
                    abort_fn=self._abort_fn)
            self._retarget_heading_lock(self._current_heading())
            self._send_neutral_and_settle()
            return self._make_result(
                True, f'vision_anchor_align: {outcome.reason}',
                final_value=float(outcome.code),
                error_value=float(outcome.last_err_px))

    # ---- vision helper (private) ------------------------------------- #
    def _resolve_vision_state(self, camera):
        if self.vision_state_provider is None:
            raise RuntimeError(
                'vision verbs require a vision_state_provider; '
                'launch via auv_manager_node so the manager can wire '
                'VisionState into Duburi.')
        vstate = self.vision_state_provider(camera)
        if vstate is None:
            raise RuntimeError(
                f"vision_state_provider({camera!r}) returned None; "
                f"check the camera name matches a running detector_node "
                f"(e.g. 'forward', 'downward', 'sim_front').")
        return vstate

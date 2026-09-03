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
``_lock_active``, ``_abort_fn``, ``_fire_payload``, ``report_vision``) --
nothing rclpy-aware.
"""

import threading
import time
from contextlib import nullcontext

from .errors import MovementError
from .fc import srot_protocol as sp

from .motion_vision import (
    align_loop, move_loop,
    KP_LAT_DEFAULT, KP_YAW_DEFAULT, KP_DEPTH_DEFAULT, KP_FORWARD_DEFAULT,
    VISION_BRAKE_GAIN, _MAX_DEPTH_NUDGE,
)


def _parse_axes(csv: str):
    """``'lat,yaw'`` -> ``{'lat','yaw'}``. Whitespace + case tolerant."""
    out = set()
    for token in (csv or '').split(','):
        name = token.strip().lower()
        if name:
            out.add(name)
    return out


def _parse_channels(csv: str):
    """``'9,10'`` -> ``[9, 10]``. Whitespace tolerant; ignores junk.

    These are BOARD channels (1..16) -- `DO_SET_SERVO param1`, the same n as
    `SERVO{n}_ROLE` -- not host-side indices. Preserves order so a mission fires
    them one-by-one in the order given.

    ⚠ The range was 1-4 while a host-side fire map existed. Left at 1-4 after the
    map was removed it would silently DROP every real payload channel on the
    default role layout (switches are 9-16 there), i.e. `fire=[9,10]` would parse
    to `[]` and the mission would sail past the target having fired nothing, with
    no error anywhere.
    """
    out = []
    for token in (csv or '').split(','):
        token = token.strip()
        if not token:
            continue
        try:
            ch = int(float(token))
        except ValueError:
            continue
        if 1 <= ch <= sp.PCA9685_NUM_CH:
            out.append(ch)
    return out


def _srot_backend(fc) -> bool:
    """True when actuation goes to the srot board rather than ArduSub."""
    return getattr(fc, 'name', '') == 'srot'


# Modes in which a streamed MANUAL_CONTROL actually reaches the thrusters.
#
# STABILIZE is the intended one: the board holds attitude and heading at 500 Hz
# and lat/yaw/fwd servo on top. MANUAL works too -- raw passthrough, no
# stabilisation -- and is allowed rather than forced away from, because an
# operator who deliberately chose it should not be overridden mid-verb.
# STABILIZE ONLY -- and MANUAL is deliberately NOT here.
#
# MANUAL passes translation through, which is why the first version of this
# check accepted it. That reasoning was wrong, and the firmware's own contract
# says so plainly: "MANUAL (mode 19) is raw passthrough with no stabilization
# at all -- no heading hold, no attitude hold. It is the escape hatch, not a
# driving mode. Fly STABILIZE." (JETSON_COMMS.md §6.)
#
# For a vision loop that is worse than it sounds. `MANUAL_CONTROL` carries no
# roll or pitch field, so in MANUAL those demands sit at zero and NOTHING
# corrects an attitude disturbance -- the hull is free to drift off level and
# stay there. The bounding box then moves for reasons that have nothing to do
# with the vehicle's position, and the loop chases them. Every gain in
# `precision-alignment.md` assumes the board is holding attitude underneath.
_SROT_VISION_MODES = ('STABILIZE',)

# The mode that made this check necessary. SURFACE is a FAILSAFE DESTINATION,
# and the firmware deliberately zeroes translation and yaw in it:
#
#   "TRANSLATION AND YAW ARE ZEROED. SURFACE is a failsafe destination --
#    reached on leak, low thruster battery, or GCS loss ... A vehicle that has
#    lost its operator should not still be driving somewhere."
#       -- srot task_control_loop.cpp, FlightMode::SURFACE
#
# That is correct firmware behaviour. The bug was ours: `vision_align` asserted
# "STABILIZE is the mode here" in a COMMENT and never set it, so an align in
# SURFACE ran the whole loop, streamed MANUAL_CONTROL at 50 Hz, and reported
# success while the board discarded every frame. Same silent-success shape as
# the pre-rev-13 disarmed SROT_MOVE, which the firmware team fixed precisely
# because a consumer would advance a mission on a dead hull.


def _require_srot_vision_mode(fc, log, verb: str) -> None:
    """Put the board in a mode where MANUAL_CONTROL actually moves it.

    Sets STABILIZE if it is not already in an acceptable mode, then VERIFIES
    the change took. Verification is the point: `set_mode` is best-effort on
    this wire (the firmware's `onSetMode` discards its own return value and
    sends no ACK), so a request that is silently refused looks identical to one
    that worked.
    """
    mode = (fc.get_mode() or '').upper()
    if mode in _SROT_VISION_MODES:
        return
    if log is not None:
        log.info(f'[CMD  ] {verb}: board is in {mode or "?"} -- '
                 f'switching to STABILIZE so MANUAL_CONTROL reaches the thrusters')
    fc.set_mode('STABILIZE')
    mode = (fc.get_mode() or '').upper()
    if mode not in _SROT_VISION_MODES:
        raise MovementError(
            f'{verb}: the board is in {mode or "an unknown mode"} and would not '
            f'accept STABILIZE. In SURFACE the firmware zeroes translation and '
            f'yaw, so this verb would run, report success, and move nothing. '
            f'Refusing. (A board in SURFACE is usually there because a failsafe '
            f'put it there -- check LEAK, thruster battery, and the GCS link.)')


class VisionVerbs:
    """Camera-driven verbs for the Duburi facade. Never instantiated alone."""

    # ================================================================== #
    #  vision_align -- centre on lat / yaw / depth at a pixel offset      #
    # ================================================================== #
    def vision_align(self, camera, target_class, axes,
                     offset_lat=0.0, offset_yaw=0.0, offset_depth=0.0,
                     err_px=40.0, duration=20.0, gain=30.0,
                     gain_lat=0.0, gain_yaw=0.0,
                     brake_off=False, brake_gain=0.0, hold_s=0.0,
                     hold_through_loss=False,
                     fire_channels='', fire_t=0.0,
                     kp_lat=0.0, kp_yaw=0.0, kp_depth=0.0,
                     lost_grace_s=0.0, align_stable_frames=0.0,
                     lock_target=False, ctrl_conf=0.0,
                     range_gain_floor=0.0, ki_lat=0.0, coast_s=0.0,
                     fwd_fill=0.0, mode='area', kp_forward=0.0,
                     settle_px=0.0, depth_step=0.0, fire_pass_enabled=False,
                     hold_heading=False, surge_sign=0.0, max_depth_m=0.0,
                     depth_ceiling_m=0.0, fire_gap=0.0):
        """Hold ``target_class`` at the requested pixel offset on each axis.

        ``axes`` is a CSV subset of ``lat,yaw,depth``; each active axis
        uses its matching ``offset_*`` (signed px, 0 = centre). Returns a
        Move.Result with ``success=True`` and the outcome code in
        ``final_value``.

        ``fire_channels`` (CSV, e.g. ``'1,2'``) fires those payload channels
        ONCE, on the first stably-aligned tick at or after ``fire_t`` s into the
        hold window, on a BACKGROUND THREAD so the 20 Hz correction loop never
        stalls on the payload write (the CH340 reconnect path can sleep ~2 s).
        The shot leaves while the loop is still gluing the hull to the target --
        no align-then-fire drift. The fire is GATED on alignment: if the lock is
        never held during the hold, the shot is NOT fired (never off-target).
        ``fire_t`` is clamped to 0 when >= ``hold_s`` (or hold_s<=0) so a held
        lock always fires mid-hold rather than on the drifting exit tick.

        ``fwd_fill`` (> 0) adds a forward range-hold axis: align ALSO drives
        forward until the bbox fills ``fwd_fill`` %% of the frame (``mode`` =
        area/width/height), then holds that standoff -- so ONE verb does
        forward-standoff + lat/depth centering + station-keep + mid-hold fire (the
        unified torpedo standoff shot). The shot is gated on the standoff range too
        (forward joins the in-band check). ``fwd_fill`` = 0 (default) -> no forward
        axis, unchanged. ``kp_forward`` overrides the fill P-gain (0 = default).
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

        # Mid-hold payload fire. Clamp fire_t < hold_s so the shot always leaves
        # WHILE the loop still corrects (firing on the drifting exit tick is the
        # wide-shot we're eliminating); warn loudly on a misconfig.
        channels = _parse_channels(fire_channels)
        eff_fire_t = float(fire_t)
        if channels and (float(hold_s) <= 0.0 or eff_fire_t >= float(hold_s)):
            self.log.warning(
                f'[CMD  ] vision_align fire_t={eff_fire_t:.1f}s >= hold_s='
                f'{float(hold_s):.1f}s -- clamping fire_t to 0 (fire at hold '
                f'start). Set hold_s > fire_t for a delayed mid-hold shot.')
            eff_fire_t = 0.0
        # Inter-channel delay when firing MULTIPLE payloads (e.g. fire=[1,4]): the
        # solenoid launcher misfires if two go together, so space them fire_gap s
        # apart (0 = back-to-back; single-channel fires are unaffected).
        gap_s = float(fire_gap) if float(fire_gap) > 0.0 else 0.0
        on_locked = (lambda: self._fire_async(channels, gap_s)) if channels else None

        with self._command_scope('vision_align'):
            self._send_neutral_and_settle()
            # Backend precondition FIRST, before resolving the camera. A board
            # in SURFACE will discard everything this verb sends, so there is
            # no point subscribing to a vision state (which can block waiting
            # for the first CameraInfo) only to refuse afterwards.
            if _srot_backend(self.pixhawk):
                _require_srot_vision_mode(self.pixhawk, self.log, 'vision_align')
            vstate = self._resolve_vision_state(camera)
            is_downward   = camera in ('downward', 'sim_bottom')
            depth_sign    = -1 if is_downward else +1
            touches_yaw   = 'yaw' in axis_set
            touches_depth = 'depth' in axis_set
            # ALT_HOLD needed when we command the depth setpoint (forward 'depth'
            # axis or downward fill->depth), AND on any downward align -- there the
            # 'depth' axis drives Ch5 surge while ArduSub must still hold the mission
            # depth on Ch3 (or we'd sink/surface uncommanded).
            # SROT: ALT_HOLD is an ArduSub mode this board does not have, and
            # the depth axis needs `set_target_depth` (SET_POSITION_TARGET),
            # which SrotFC does not implement. REFUSE rather than run an align
            # whose depth axis silently does nothing -- a mission that believes
            # it is descending onto a bin and is not is the dangerous version.
            # STABILIZE is the mode here: the board holds attitude and heading
            # at 500 Hz and lat/yaw/fwd servo on top of it.
            if _srot_backend(self.pixhawk):
                if touches_depth or is_downward:
                    raise MovementError(
                        "vision_align: the 'depth' axis (and any downward align) "
                        "is not supported on the SROT backend -- it needs a "
                        "streamed depth setpoint, which this board does not take. "
                        "Use lat/yaw/fwd, or drive depth with a separate "
                        "set_depth once the depth loop is water-verified.")
            elif touches_depth or is_downward or float(fwd_fill) > 0.0:
                self._ensure_alt_hold('vision_align')

            stable = int(align_stable_frames) or 3
            fire_note = (f' fire={channels}@{eff_fire_t:.1f}s' if channels else '')
            fwd_note = (f' fwd>={float(fwd_fill):.0f}%({mode})'
                        if float(fwd_fill) > 0.0 else '')
            self.log.info(
                f'[CMD  ] vision_align camera={camera!r} class={target_class!r} '
                f'axes={sorted(axis_set)}{fwd_note} err={float(err_px):.0f}px '
                f'gain={float(gain):.0f}% dur={float(duration):.0f}s '
                f'hold={float(hold_s):.0f}s{fire_note}')

            # Ch4 arbitration, gated on the YAW AXIS (not on lock-state): the
            # verb writes Ch4 ONLY when yaw is a requested align axis. When yaw
            # is an axis we suspend the lock and the loop drives Ch4 itself.
            # When yaw is NOT an axis we ALWAYS release Ch4 (send_rc_translation,
            # lateral-only) so the verb never commands yaw the operator didn't
            # ask for, and never clobbers a live lock's Ch4 stream (the
            # lat/depth-align-fights-lock jitter). With a lock active the lock
            # holds heading on BNO; with no lock, Ch4 falls to the heartbeat /
            # ArduSub -- either way the verb stays off the yaw channel.
            release_yaw = not touches_yaw
            # Fire-window quiet mode: only meaningful when yaw is released (the
            # background heading lock owns Ch4). Widen the lock deadband so it holds
            # a steady launcher heading instead of chasing sub-deg noise while the
            # torpedo fires; always restored in the finally. No-op if no lock active.
            hold_lock = release_yaw and bool(hold_heading)
            if hold_lock:
                self._set_lock_hold(True)
            try:
                with self._suspend_heading_lock() if touches_yaw else nullcontext():
                    outcome = align_loop(
                        pixhawk=self.pixhawk, vision_state=vstate,
                        target_class=target_class, axes=axis_set, offsets=offsets,
                        err_px=float(err_px), duration=float(duration),
                        gain=float(gain),
                        gain_lat=float(gain_lat) or float(gain),
                        gain_yaw=float(gain_yaw) or float(gain),
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
                        lock_on=bool(lock_target),
                        ctrl_conf=float(ctrl_conf),
                        range_gain_floor=float(range_gain_floor) or 1.0,
                        ki_lat=float(ki_lat),
                        coast_s=float(coast_s),
                        fwd_fill=float(fwd_fill) / 100.0,   # % -> fraction (like move)
                        fwd_mode=str(mode) or 'area',
                        kp_forward=float(kp_forward) or KP_FORWARD_DEFAULT,
                        settle_px=float(settle_px),
                        depth_step=float(depth_step) or _MAX_DEPTH_NUDGE,
                        downward=is_downward,
                        # SIGN-ONLY: coerce to exactly +1/-1 (rosidl-0 -> +1) so it can
                        # never scale Ch5 past the gain cap -- it only flips fore/aft.
                        surge_sign=(-1 if float(surge_sign) < 0.0 else +1),
                        max_depth_m=float(max_depth_m),      # deep floor for fill->depth (0=off)
                        depth_ceiling_m=float(depth_ceiling_m),  # shallow surface guard (0=default)
                        on_locked=on_locked,
                        fire_t=eff_fire_t,
                        fire_pass=bool(fire_pass_enabled),
                        report_fn=self.report_vision,
                        writers=self._writers(), log=self.log,
                        abort_fn=self._abort_fn)
            finally:
                if hold_lock:
                    self._set_lock_hold(False)
            if touches_yaw:
                self._retarget_heading_lock(self._current_heading())
            self._send_neutral_and_settle()
            return self._make_result(
                True, f'vision_align: {outcome.reason}',
                final_value=float(outcome.code),
                error_value=float(outcome.last_err_px),
                end_x_px=outcome.end_x_px, end_y_px=outcome.end_y_px,
                fill_frac=0.0, elapsed_s=outcome.elapsed_s)

    def _fire_async(self, channels, gap_s: float = 0.0):
        """Fire payload ``channels`` one-by-one on a daemon thread (non-blocking).

        Called from inside the align hold loop via ``on_locked``; returns
        immediately so the 20 Hz station-keep keeps correcting while the payload
        actuates (``payload.fire`` can sleep ~2 s on a CH340 reconnect). Each
        channel re-checks the cooperative abort right before its write, so a
        torpedo never leaves after an emergency stop. The shared payload serial
        is serialised inside ``PayloadDriver.fire`` (a lock), so overlapping a
        later standalone ``fire()`` goal is safe.

        ``gap_s`` > 0 spaces MULTIPLE channels apart (fire=[1,4]): the solenoid
        launcher misfires when two fire together, so we wait gap_s BETWEEN shots
        (never before the first, and never after the last). The wait is abort-
        interruptible in gap_s slices so an emergency stop still cancels promptly.
        """
        abort_fn = self._abort_fn

        # ponytail: CTRL-14 residual -- this thread reads the SHARED _abort_event,
        # which the NEXT command clears. In the narrow window where an emergency
        # abort is set during the inter-shot gap and a new command clears it before
        # this thread's next 0.1 s poll, a queued shot could still leave. A full fix
        # needs a per-fire cancel token, but the fire is deliberately non-blocking so
        # a delayed shot OUTLIVES the align scope (payload.fire can sleep ~2 s) -- so
        # "abort when the scope exits" would wrongly cancel a legit delayed shot.
        # Deferred rather than risk the fire path pre-competition; the gap is 0.1 s.
        def _run():
            for i, ch in enumerate(channels):
                if abort_fn is not None and abort_fn():
                    self.log.warning(
                        f'[FIRE ] abort signalled -- skipping ch={ch} '
                        f'(remaining {channels[i:]} cancelled)')
                    return
                # Space multi-channel shots apart (not before the first).
                if i > 0 and gap_s > 0.0:
                    self.log.info(f'[FIRE ] waiting {gap_s:.1f}s before ch={ch} '
                                  f'(solenoid needs the gap)')
                    waited = 0.0
                    while waited < gap_s:
                        if abort_fn is not None and abort_fn():
                            self.log.warning('[FIRE ] abort during inter-shot gap '
                                             f'-- {channels[i:]} cancelled')
                            return
                        time.sleep(min(0.1, gap_s - waited))
                        waited += 0.1
                try:
                    res = self._fire_payload(ch)
                    # The shot is the whole point of the hold, so a refusal must be
                    # loud. This used to be discarded: a channel the board calls the
                    # ARM was refused deep in the driver and the mission sailed on
                    # believing it had fired.
                    if getattr(res, 'ok', bool(res)):
                        self.log.info(f'[FIRE ] ch={ch} {res.code_name}: {res.reason}')
                    else:
                        self.log.error(f'[FIRE ] ch={ch} NOT FIRED -- '
                                       f'{res.code_name}: {res.reason}')
                except Exception as exc:   # noqa: BLE001 -- thread must not crash silently
                    self.log.error(f'[FIRE ] ch={ch} raised {exc!r}')

        threading.Thread(target=_run, name='vision_align_fire',
                         daemon=True).start()

    # ================================================================== #
    #  vision_move -- drive forward to a bbox fill ratio                  #
    # ================================================================== #
    def vision_move(self, camera, target_class, fwd_fill=95.0, mode='area',
                    maintain_px=0.0, maintain_on=False, hold_s=0.0,
                    err_px=40.0, duration=20.0, gain=30.0, gain_lat=0.0,
                    brake_off=False, brake_gain=0.0,
                    hold_through_loss=False,
                    kp_forward=0.0, kp_lat=0.0, lost_grace_s=0.0,
                    range_gain_floor=0.0, coast_s=0.0):
        """Drive forward until ``target_class`` fills ``fwd_fill`` % of the frame.

        ``mode`` is the fill metric (area/width/height). ``maintain_on``
        holds a lateral pixel offset (``maintain_px``) while driving;
        depth/yaw are never commanded. A ``fwd_fill`` <= 0 selects
        PASS-THROUGH: drive forward until the target is seen and then
        leaves the frame, plus a commit overshoot (used to go *through* a
        gate). Returns a Move.Result with ``success=True`` and the
        outcome code in ``final_value``.
        """
        # Guard: move() is meaningless on a DOWNWARD camera -- surging Ch5 does
        # not grow a downward target's bbox fill (you descend to approach, you
        # don't drive into it), so a fill-stop move would drive forever. The bin
        # task centres with align(camera='downward') and descends via its fill
        # axis; there is no move() phase. Reject explicitly rather than misbehave.
        if camera in ('downward', 'sim_bottom'):
            return self._make_result(
                True, "vision_move: not supported on a downward camera "
                      "(use vision_align -- surge doesn't grow downward fill)",
                final_value=2.0, error_value=0.0)   # TIMEOUT-ish no-op
        # fwd_fill <= 0 is the pass-through sentinel (DSL move(fwd=None) /
        # CLI --fwd_fill -1). Anything > 0 is a real fill-% stop target.
        passthrough = float(fwd_fill) <= 0.0
        with self._command_scope('vision_move'):
            self._send_neutral_and_settle()
            if _srot_backend(self.pixhawk):
                _require_srot_vision_mode(self.pixhawk, self.log, 'vision_move')
            vstate = self._resolve_vision_state(camera)
            # move_loop drives forward but never commands depth -- it relies
            # on ArduSub's onboard depth hold. Ensure ALT_HOLD so the approach
            # holds depth even when a mission jumps straight to vision_move
            # without a prior set_depth (from MANUAL the setpoint is dropped).
            # SROT holds depth on-board (or not at all, in STABILIZE); there
            # is no ALT_HOLD to ensure and no host depth stream to start.
            if not _srot_backend(self.pixhawk):
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
                # move never computes a yaw command, so it must ALWAYS leave Ch4
                # alone (lock owns it on BNO, else heartbeat/ArduSub hold) --
                # never write Ch4=1500 against a live lock.
                release_yaw=True,
                range_gain_floor=float(range_gain_floor) or 1.0,
                coast_s=float(coast_s),
                report_fn=self.report_vision,
                writers=self._writers(), log=self.log,
                abort_fn=self._abort_fn)
            self._send_neutral_and_settle()
            return self._make_result(
                True, f'vision_move: {outcome.reason}',
                final_value=float(outcome.code),
                error_value=float(outcome.last_err_px),
                end_x_px=outcome.end_x_px, end_y_px=outcome.end_y_px,
                fill_frac=float(outcome.fill), elapsed_s=outcome.elapsed_s)

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

#!/usr/bin/env python3
"""Vision verbs for the Mongla facade -- exactly two, pixel-native.

``vision_align`` centres the target on the selected axes (lat / yaw /
depth) each at a signed pixel offset; ``vision_move`` drives forward
until the target's bbox fills the frame to a ratio. Both delegate to the
two loops in :mod:`motion_vision` and ALWAYS return ``success=True`` --
the align/move outcome rides in ``Move.Result.final_value`` as an
integer code so the mission DSL branches on it and the action client
never raises on a miss.

Both stream MANUAL_CONTROL to the board in STABILIZE: the board holds
attitude, heading and depth at 500 Hz and these verbs add lat / yaw / forward
on top. Neither moves the depth setpoint -- the board takes none over this
wire -- so a forward ``depth`` axis and a downward fill-driven descent are
REFUSED with the reason, never run as a verb that silently holds depth.

Mixed into ``Mongla``; uses only the facade helpers (``_command_scope``,
``_writers``, ``_resolve_vision_state``, ``_send_neutral_and_settle``,
``_make_result``, ``_abort_fn``, ``_fire_payload``, ``report_vision``) --
nothing rclpy-aware.
"""

import threading
import time

from .errors import MovementError

from .motion_vision import (
    align_loop, move_loop,
    KP_LAT_DEFAULT, KP_YAW_DEFAULT, KP_FORWARD_DEFAULT,
    VISION_BRAKE_GAIN,
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

    The range was 1-4 while a host-side fire map existed. Left at 1-4 after the
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
        # Keep out-of-range numbers -- above all 0, which is "NOT ASSIGNED"
        # in `competition_config` (B53). Dropping them here turned an unassigned
        # torpedo into an align that fired nothing, with no error anywhere -- the
        # same silent-drop this docstring records for the old 1-4 range. Passed
        # through, `fire()` refuses each one WITH a reason, and that reason lands
        # in the verb's payload outcome and the log.
        out.append(ch)
    return out


# An ATTITUDE older than this is not "the rate now". Two 50 Hz periods.
_DEROTATE_RATE_MAX_AGE_S = 0.04


def _derotation_inputs(fc, vstate):
    """`(yaw_rate rad/s, px_per_rad)` for `align_loop`, or None.

    Absence is not zero. `get_angular_rates()` already returns None for an
    unhealthy BNO (whose rates read exactly 0.0), and a stale sample is the
    same claim with an extra delay -- so both mean "no correction".
    """
    rates = fc.get_angular_rates() if hasattr(fc, 'get_angular_rates') else None
    if not rates or rates.get('age_s', 1.0) > _DEROTATE_RATE_MAX_AGE_S:
        return None
    ppr = vstate.px_per_rad()
    if ppr is None:
        return None
    return float(rates['yaw_rate']), float(ppr)


# Modes in which a streamed MANUAL_CONTROL actually reaches the thrusters.
#
# STABILIZE ONLY: the board holds attitude and heading at 500 Hz and lat/yaw/fwd
# servo on top. MANUAL is deliberately NOT here.
#
# (The two sentences above used to be preceded by their own contradiction -- the
# pre-correction text "MANUAL works too ... and is allowed" was left in place
# when the rule was reversed, so the first thing a reader met was the opposite
# of the code directly below. Deleted rather than annotated: a comment block
# that argues with itself is worse than either version alone.)
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
#   "translation and yaw are zeroed. Surface is a failsafe destination --
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

    Surface is never switched out of -- only refused. It used to be treated
    like any other wrong mode and "corrected" to STABILIZE, which let a vision
    verb pull the hull out of a leak / low-battery / GCS-loss failsafe, or out
    of the operator's own `surface`, and drive it back down. The board reached
    SURFACE for a reason this verb cannot see; leaving it is the operator's
    call (an explicit `set_mode`), never a side-effect of an align. Refused
    with the same MovementError as a refused STABILIZE, so the manager turns
    it into a failed Result exactly as before.
    """
    mode = (fc.get_mode() or '').upper()
    if mode in _SROT_VISION_MODES:
        return
    if mode == 'SURFACE':
        raise MovementError(
            f'{verb}: the board is in SURFACE and a vision verb will not take it '
            f'out. SURFACE is a failsafe destination (leak, thruster battery, GCS '
            f'loss) or an operator surface; the firmware zeroes translation and '
            f'yaw there, so this verb would move nothing. Refusing. Check the '
            f'failsafe, then set_mode STABILIZE explicitly if it is safe to dive.')
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



# How long a verb waits, after its loop ends, for an in-flight fire to report.
# `payload.fire` can sleep ~2 s on a CH340 reconnect; longer than that is a
# fire the result reports as `pending` rather than a verb that hangs.
_FIRE_REPORT_WAIT_S = 2.5


def _fired_suffix(channels, fire_state) -> str:
    """` fired=ch1:FIRED,...` | ` fired=none` | ` fired=pending` | '' (no fire asked).

    Carried in the result MESSAGE so no action field changes; the DSL parses it
    into `VisionResult.fired`.
    """
    if not channels:
        return ''
    th = fire_state.get('thread')
    if th is None:
        return ' fired=none'
    th.join(timeout=_FIRE_REPORT_WAIT_S)
    if th.is_alive() and not fire_state['outcomes']:
        return ' fired=pending'
    return ' fired=' + ','.join(fire_state['outcomes'] or ['none'])

class VisionVerbs:
    """Camera-driven verbs for the Mongla facade. Never instantiated alone."""

    # ================================================================== #
    #  vision_align -- centre on lat / yaw / depth at a pixel offset      #
    # ================================================================== #
    _tool_warned = None

    def _warn_tool_geometry(self, tool, vstate):
        """One line, once per tool, when the aim correction cannot be trusted.

        Three distinguishable states, and collapsing them would hide the one
        that matters:
          * no tool named        -> silent. Aiming the camera is a legitimate
                                    choice and the historical default.
          * tool UNMEASURED      -> WARN. It reads as zero and will miss by the
                                    real offset, invisibly.
          * measured but no pose -> WARN. The correction is range-dependent and
                                    there is no range, so it cannot be applied.
        """
        if not tool:
            return
        if self._tool_warned is None:
            self._tool_warned = set()
        if tool in self._tool_warned:
            return
        self._tool_warned.add(tool)
        try:
            from mongla_vision.tool_geometry import is_measured, known_tools
        except ImportError:
            return
        if tool not in known_tools():
            self.log.error(
                f'[VIS  ] tool {tool!r} is not in tool_geometry.yaml '
                f'(known: {known_tools()}). Aiming the CAMERA.')
            return
        if not is_measured(tool):
            self.log.warning(
                f'[VIS  ] tool {tool!r} is UNMEASURED -- its offset reads as '
                f'zero, so this aims the CAMERA. The round leaves a parallel '
                f'axis, so the miss equals the real offset AT EVERY RANGE and '
                f'closing in does not help. Measure it into '
                f'config/tool_geometry.yaml.')
            return
        if getattr(vstate, 'tool_offset_px', None) and vstate.tool_offset_px(tool) is None:
            self.log.warning(
                f'[VIS  ] tool {tool!r} is measured but there is no live '
                f'target_pose, so the range-dependent aim correction cannot be '
                f'computed. Aiming the CAMERA. (lock_node + a calibration + a '
                f'committed target width are what produce a pose.)')

    def vision_align(self, camera, target_class, axes,
                     offset_lat=0.0, offset_yaw=0.0, offset_depth=0.0,
                     err_px=40.0, duration=20.0, gain=30.0,
                     gain_lat=0.0, gain_yaw=0.0,
                     brake_off=False, brake_gain=0.0, hold_s=0.0,
                     hold_through_loss=False,
                     fire_channels='', fire_t=0.0,
                     kp_lat=0.0, kp_yaw=0.0,
                     lost_grace_s=0.0, align_stable_frames=0.0,
                     lock_target=False, ctrl_conf=0.0,
                     range_gain_floor=0.0, ki_lat=0.0, coast_s=0.0,
                     lock_s=0.0,
                     standoff_max_tilt_deg=0.0,
                     tool='',
                     fwd_fill=0.0, mode='area', kp_forward=0.0,
                     settle_px=0.0, verify_off=False,
                     fire_pass_enabled=False,
                     surge_sign=0.0, fire_gap=0.0,
                     fire_max_tilt_deg=0.0):
        """Hold ``target_class`` at the requested pixel offset on each axis.

        ``axes`` is a CSV subset of ``lat,yaw,depth``; each active axis
        uses its matching ``offset_*`` (signed px, 0 = centre). ``depth`` is
        the DOWNWARD camera's image-Y, driven as SURGE fore/aft; on the forward
        camera it would move the depth setpoint and is refused. Returns a
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
        # The fire's own outcome, so the RESULT can say whether a shot left
        # (it used to exist only as a log line from a daemon thread).
        fire_state = {'thread': None, 'outcomes': []}

        def _fire_and_keep():
            fire_state['thread'] = self._fire_async(channels, gap_s,
                                                    outcomes=fire_state['outcomes'])
        on_locked = _fire_and_keep if channels else None

        is_downward = camera == 'downward'
        if 'depth' in axis_set and not is_downward:
            raise MovementError(
                "vision_align: the forward 'depth' axis moves the depth setpoint, "
                "which the board does not take over this wire. Use lat/yaw (and "
                "fwd_fill for range), with depth held by set_depth.")
        if is_downward and float(fwd_fill) > 0.0:
            raise MovementError(
                "vision_align: fwd_fill on the downward camera is a depth DESCENT, "
                "which needs a streamed depth setpoint the board does not take. "
                "Centre with lat + depth (surge) and set the height with set_depth.")

        with self._command_scope('vision_align'):
            self._send_neutral_and_settle()
            # Mode FIRST, before resolving the camera: a board in SURFACE
            # discards everything this verb sends, so there is no point waiting
            # for a CameraInfo only to refuse afterwards.
            _require_srot_vision_mode(self.fc, self.log, 'vision_align')
            vstate = self._resolve_vision_state(camera)
            stable = int(align_stable_frames) or 3
            fire_note = (f' fire={channels}@{eff_fire_t:.1f}s' if channels else '')
            fwd_note = (f' fwd>={float(fwd_fill):.0f}%({mode})'
                        if float(fwd_fill) > 0.0 else '')
            self.log.info(
                f'[CMD  ] vision_align camera={camera!r} class={target_class!r} '
                f'axes={sorted(axis_set)}{fwd_note} err={float(err_px):.0f}px '
                f'gain={float(gain):.0f}% dur={float(duration):.0f}s '
                f'hold={float(hold_s):.0f}s{fire_note}')

            # Say it before the shot, not after. An unmeasured tool reads as
            # ZERO offset -- byte-identical to aiming the camera, and therefore
            # silent. But it is exactly the case that misses: the round leaves
            # a parallel axis and the miss equals the offset at every range.
            self._warn_tool_geometry(tool, vstate)
            outcome = align_loop(
                fc=self.fc, vision_state=vstate,
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
                lost_grace_s=float(lost_grace_s) or 1.0,
                hold_through_loss=bool(hold_through_loss),
                align_stable_frames=stable,
                lock_on=bool(lock_target),
                ctrl_conf=float(ctrl_conf),
                range_gain_floor=float(range_gain_floor) or 1.0,
                ki_lat=float(ki_lat),
                coast_s=float(coast_s),
                mixer_aware=bool(getattr(self, 'vision_mixer_aware', True)),
                # Without this line the ladder is unreachable: `align_loop`
                # accepted `lock_s` for weeks while no caller passed it.
                lock_s=float(lock_s),
                fwd_fill=float(fwd_fill) / 100.0,   # % -> fraction (like move)
                fwd_mode=str(mode) or 'area',
                kp_forward=float(kp_forward) or KP_FORWARD_DEFAULT,
                settle_px=float(settle_px),
                verify=not bool(verify_off),
                fire_max_tilt_deg=float(fire_max_tilt_deg),
                # Bound to THIS camera's state: the pose is per-camera.
                tilt_gate_fn=getattr(vstate, 'square_within', None),
                # The APPROACH guard. Distinct from `square_within`, which
                # folds "no pose" into "not square" -- right for a shot,
                # wrong for an approach.
                standoff_max_tilt_deg=float(standoff_max_tilt_deg),
                obliquity_fn=getattr(vstate, 'obliquity_deg', None),
                # Aim the TOOL. Resolved by the manager, which is the layer
                # allowed to import mongla_vision.
                tool_offset_fn=((lambda: vstate.tool_offset_px(tool))
                                if tool and hasattr(vstate, 'tool_offset_px')
                                else None),
                # Steer on where the target is NOW: gyro rate x box age.
                derotate_fn=((lambda: _derotation_inputs(self.fc, vstate))
                             if hasattr(vstate, 'px_per_rad') else None),
                downward=is_downward,
                # SIGN-ONLY: exactly +1/-1 (rosidl 0 -> +1), so it can never
                # scale surge past the gain cap -- it only flips fore/aft.
                surge_sign=(-1 if float(surge_sign) < 0.0 else +1),
                on_locked=on_locked,
                fire_t=eff_fire_t,
                fire_pass=bool(fire_pass_enabled),
                report_fn=self.report_vision,
                writers=self._writers(), log=self.log,
                abort_fn=self._abort_fn)
            self._send_neutral_and_settle()
            return self._make_result(
                True, f'vision_align: {outcome.reason}'
                      + _fired_suffix(channels, fire_state),
                final_value=float(outcome.code),
                error_value=float(outcome.last_err_px),
                end_x_px=outcome.end_x_px, end_y_px=outcome.end_y_px,
                fill_frac=0.0, elapsed_s=outcome.elapsed_s)

    def _fire_async(self, channels, gap_s: float = 0.0, outcomes=None):
        """Fire payload ``channels`` one-by-one on a daemon thread (non-blocking).

        Called from inside the align hold loop via ``on_locked``; returns
        immediately so the 20 Hz station-keep keeps correcting while the payload
        actuates (``payload.fire`` can sleep ~2 s on a CH340 reconnect). Each
        channel re-checks the cooperative abort right before its write, so a
        torpedo never leaves after an emergency stop. Payload writes are
        serialised inside ``SrotPayload.fire`` (a lock), so overlapping a
        later standalone ``fire()`` goal is safe.

        ``gap_s`` > 0 spaces MULTIPLE channels apart (fire=[1,4]): the solenoid
        launcher misfires when two fire together, so we wait gap_s BETWEEN shots
        (never before the first, and never after the last). The wait is abort-
        interruptible in gap_s slices so an emergency stop still cancels promptly.
        """
        abort_fn = self._abort_fn
        # The shared abort event is cleared by the NEXT verb, so on its own it
        # let a shot queued in the gap leave after an emergency stop (issue
        # #73). The generation is never reset: once any abort, stop or surface
        # bumps it, this sequence is cancelled for good. A sequence nobody
        # aborted still outlives the align that scheduled it, by design.
        gen = self._abort_generation

        def cancelled():
            return ((abort_fn is not None and abort_fn())
                    or self._abort_generation != gen)

        def cancel_rest(rest, why):
            self.log.warning(f'[FIRE ] {why} -- {rest} cancelled, never sent')
            if outcomes is not None:
                outcomes.extend(f'ch{c}:CANCELLED' for c in rest)

        def _run():
            for i, ch in enumerate(channels):
                if cancelled():
                    cancel_rest(channels[i:], 'abort signalled')
                    return
                # Space multi-channel shots apart (not before the first).
                if i > 0 and gap_s > 0.0:
                    self.log.info(f'[FIRE ] waiting {gap_s:.1f}s before ch={ch} '
                                  f'(solenoid needs the gap)')
                    end = time.monotonic() + gap_s
                    while time.monotonic() < end:
                        if cancelled():
                            break
                        time.sleep(min(0.05, max(0.0, end - time.monotonic())))
                    # Checked again AFTER the wait: the last slice used to run
                    # straight into the write.
                    if cancelled():
                        cancel_rest(channels[i:], 'abort during inter-shot gap')
                        return
                try:
                    res = self._fire_payload(ch)
                    if outcomes is not None:
                        outcomes.append(f'ch{ch}:{getattr(res, "code_name", "DONE")}')
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
                    if outcomes is not None:
                        outcomes.append(f'ch{ch}:RAISED')

        th = threading.Thread(target=_run, name='vision_align_fire', daemon=True)
        th.start()
        return th

    # ================================================================== #
    #  vision_move -- drive forward to a bbox fill ratio                  #
    # ================================================================== #
    def vision_move(self, camera, target_class, fwd_fill=95.0, mode='area',
                    maintain_px=0.0, maintain_on=False, hold_s=0.0,
                    err_px=40.0, duration=20.0, gain=30.0, gain_lat=0.0,
                    brake_off=False, brake_gain=0.0,
                    hold_through_loss=False,
                    kp_forward=0.0, kp_lat=0.0, lost_grace_s=0.0,
                    range_gain_floor=0.0, coast_s=0.0, lock_s=0.0):
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
        if camera == 'downward':
            return self._make_result(
                True, "vision_move: not supported on a downward camera "
                      "(use vision_align -- surge doesn't grow downward fill)",
                final_value=2.0, error_value=0.0)   # TIMEOUT-ish no-op
        # fwd_fill <= 0 is the pass-through sentinel (DSL move(fwd=None) /
        # CLI --fwd_fill -1). Anything > 0 is a real fill-% stop target.
        passthrough = float(fwd_fill) <= 0.0
        with self._command_scope('vision_move'):
            self._send_neutral_and_settle()
            _require_srot_vision_mode(self.fc, self.log, 'vision_move')
            vstate = self._resolve_vision_state(camera)
            self.log.info(
                f'[CMD  ] vision_move camera={camera!r} class={target_class!r} '
                f'{"PASS-THROUGH" if passthrough else "fwd_fill=%.0f%%" % float(fwd_fill)} '
                f'mode={mode} '
                f'maintain={"%+.0fpx" % maintain_px if maintain_on else "off"} '
                f'hold={float(hold_s):.0f}s gain={float(gain):.0f}%')
            outcome = move_loop(
                fc=self.fc, vision_state=vstate,
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
                range_gain_floor=float(range_gain_floor) or 1.0,
                coast_s=float(coast_s),
                mixer_aware=bool(getattr(self, 'vision_mixer_aware', True)),
                # The ladder, same as align -- `move_loop` took `lock_s` and
                # nobody passed it either.
                lock_s=float(lock_s),
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
                'VisionState into Mongla.')
        vstate = self.vision_state_provider(camera)
        if vstate is None:
            raise RuntimeError(
                f"vision_state_provider({camera!r}) returned None; "
                f"check the camera name matches a running detector_node "
                f"('forward' or 'downward').")
        return vstate

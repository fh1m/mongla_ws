"""Motion: the SROT_MOVE verbs, their ACK relay, and MANUAL_CONTROL.

`move()` sends one SROT_MOVE and relays the board's progress and terminal ACKs;
`manual()` and `demand()` are the STABILIZE path the vision verbs use.

Mixed into `SrotFC`.
"""
from __future__ import annotations
import math
import time
from ._srot_base import _ARMED_FLAG, _POLL_S, _LINK_STALE_S, _rx_age, _finite
from .base import (MoveResult, SUCCEEDED, PREEMPTED, FAILED, DENIED, TIMEOUT, ABORTED)
from . import srot_protocol as sp
from .srot_moves import (  # noqa: F401
    MOVE_ACK_DRAIN_S,
    _ACK_TO_CODE,
    _POST_ACK_HB_WAIT_S,
    _STALL_HARD_MULT,
    _ack_budget_s,
    _build_params,
)


class SrotMotion:
    """Methods of `SrotFC`; see `srot_fc.py` for the node."""

    def manual(self, fwd: float, lat: float, up: float, yaw: float) -> None:
        """One MANUAL_CONTROL frame. x=fwd, y=lat(+starboard), z=heave(+up), r=yaw;
        all axes -1..1 (the board clamps; we clamp in srot_protocol). Buttons=0.

        A non-finite axis is coerced to neutral (0) rather than raising: this is the
        streamed 20 Hz servo primitive, so a single NaN from a vision loop must not
        crash the send -- it degrades to 'hold' for that tick.

        The mode guard is the point, not decoration (B28). In `AUTO` and `SURFACE`
        the firmware DISCARDS this frame -- every axis of it -- and says nothing:

            AUTO     fwd = md.fwd; lat = md.lat        task_control_loop.cpp:243
                     thr = depth::update(...)         :241   (sp_throttle ignored)
                     attitude::stabilize(0,0,md.yaw)  :239   (sp_yaw ignored)
            SURFACE  fwd = 0; lat = 0;                :205   and yaw/thr likewise

        and `AUTO` is where the board is LEFT after every `SROT_MOVE`, because --
        unlike STUNT, PATTERN and AUTOTUNE, which all restore STABILIZE when their
        state machine ends (:865, :874, :855) -- nothing resets the mode when a
        movement finishes. So "stream MANUAL_CONTROL after a move" is a silent
        no-op, and the only thing that used to stand against it was a comment.

        This costs NOTHING on the wire: the mode comes from the HEARTBEAT the link
        already carries. It warns rather than raises -- this is the 20 Hz servo
        path, and a hard failure here is worse than a wrong-mode frame.
        Callers that genuinely need to drive must reach a MANUAL_CONTROL-honouring
        mode first, as `vision_verbs._require_srot_vision_mode` does."""
        self._warn_if_mode_discards_manual()

        def _safe(v):
            return v if isinstance(v, (int, float)) and math.isfinite(v) else 0.0
        x = sp.unit_to_mc(_safe(fwd))
        y = sp.unit_to_mc(_safe(lat))
        r = sp.unit_to_mc(_safe(yaw))
        z = sp.unit_to_mc_z(_safe(up))
        with self._tx_lock:
            self.master.mav.manual_control_send(sp.VEHICLE_SYSID, x, y, z, r, 0)
        # What was SENT, after coercion and clamping -- the one funnel every
        # srot writer passes through. Read by `demand()`.
        #
        # A NaN axis is SENT as 0, so the demand in force on the board is 0 and is
        # recorded as such -- the board acts on the wire, not on the request.
        # (A cloud audit read this as "an unknown recorded as known"; it is not:
        # `test_demand_is_what_was_SENT_after_clamping` pins it, 2026-11-07.)
        self._demand = (x / sp.MC_AXIS_MAX, y / sp.MC_AXIS_MAX, time.monotonic())

    def demand(self):
        """(fwd, lat) in [-1, 1] in force on the board, or None when it is not known.

        Three sources, in order of how much they know:
          1. a fresh heartbeat saying DISARMED -> (0, 0), known;
          2. the board's echo (DEM_FWD / DEM_LAT), when both are fresh -- the
             demand the mixer was actually handed, in every mode, so an
             SROT_MOVE is no longer a blind spot;
          3. the host's own MANUAL_CONTROL record, younger than DEMAND_FRESH_S
             -- firmware without the echo. None after any SROT_MOVE (the board
             owns the demand), before the first frame, and once stale.

        The echo outranks the host's record because the record is what was
        SENT, and in AUTO the board discards MANUAL_CONTROL: the record would be
        a confident demand the thrusters never saw. Consumed by the
        command-velocity model, which must predict nothing from an unknown, and
        by the ZUPT gate.
        """
        # A disarmed board drives no thruster, so its demand is KNOWN: zero.
        # Without this the demand read "unknown" whenever no MANUAL_CONTROL
        # stream ran -- every idle moment -- and the localization ZUPT, which
        # now requires a known-zero demand (issue #25), could never fire on
        # deck. Only on a FRESH heartbeat: a stale link is unknown, not idle.
        hb = self._vehicle_hb()
        if (hb is not None and _rx_age(hb) <= _LINK_STALE_S
                and not (hb.base_mode & _ARMED_FLAG)):
            return 0.0, 0.0
        fwd = self._named_value('DEM_FWD', self.DEMAND_ECHO_FRESH_S)
        lat = self._named_value('DEM_LAT', self.DEMAND_ECHO_FRESH_S)
        if (fwd is not None and lat is not None
                and math.isfinite(fwd) and math.isfinite(lat)):
            return float(fwd), float(lat)
        d = getattr(self, '_demand', None)
        if d is None or time.monotonic() - d[2] > self.DEMAND_FRESH_S:
            return None
        return d[0], d[1]

    def _warn_if_mode_discards_manual(self) -> None:
        """Warn (rate-limited) when the board is in a mode that throws this frame away."""
        mode = self.get_mode()
        if mode not in self._MANUAL_DISCARDING_MODES:
            self._last_mc_mode_warn = 0.0
            return
        now = time.monotonic()
        if now - getattr(self, '_last_mc_mode_warn', 0.0) < self._MANUAL_WARN_PERIOD_S:
            return
        self._last_mc_mode_warn = now
        self._log_warn(
            f'[SROT ] !! MANUAL_CONTROL sent while the board is in {mode} -- the '
            f'firmware DISCARDS every axis of it and reports nothing (B28). '
            f'{"AUTO is where a completed SROT_MOVE leaves the board; " if mode == "AUTO" else ""}'
            f'reach STABILIZE first (vision_verbs._require_srot_vision_mode does).')

    def _cut_short_by_mode_change(self, t_ack: float):
        """Why a move ACKed ACCEPTED,100 did not really complete -- or None.

        The board reports a cancelled move as done. When a failsafe (leak,
        battery, companion loss), a baro loss or a disarm displaces AUTO, the
        running move is cancelled and then latched as finished, so the terminal
        ACK is ACCEPTED at 100 %. Taken at its word, the mission advances to its
        next leg -- which (without the SURFACE refusal in `move`) would pull the
        board straight back out of the failsafe. A move only completed if the
        board is still ARMED and in AUTO afterwards: a completed move never
        leaves AUTO (it never exits by itself).

        Judged on a HEARTBEAT newer than the ACK, waited for briefly; with no
        vehicle heartbeat at all there is nothing to judge and the ACK stands.

        An older heartbeat is not evidence. If none newer than the ACK lands
        inside the wait, the only heartbeat in hand was sent BEFORE the move
        ended -- ARMED and in AUTO by construction, whatever the failsafe did
        since. Judging on it reported every cut-short move as completed exactly
        when the link was struggling. The manager asks for HEARTBEAT at 10 Hz, so
        silence for _POST_ACK_HB_WAIT_S is itself the news: fail, unverified.
        (Cloud audit, 2026-11-07.)
        """
        hb = self._vehicle_hb()
        if hb is None:
            return None
        deadline = time.monotonic() + _POST_ACK_HB_WAIT_S
        while getattr(hb, '_timestamp', 0.0) < t_ack and time.monotonic() < deadline:
            time.sleep(_POLL_S)
            hb = self._vehicle_hb() or hb
        if getattr(hb, '_timestamp', 0.0) < t_ack:
            return (f'unverified -- no heartbeat after the ACK within '
                    f'{_POST_ACK_HB_WAIT_S:.1f} s, so whether a failsafe cut the '
                    f'move short cannot be told')
        mode = sp.mode_name(getattr(hb, 'custom_mode', None))
        base = getattr(hb, 'base_mode', None)
        if base is not None and not (base & _ARMED_FLAG):
            return 'cut short -- the board DISARMED during the move (ACK said 100 %)'
        if mode not in ('AUTO', 'UNKNOWN'):
            return (f'cut short -- the board left AUTO for {mode} during the move '
                    f'(ACK said 100 %); a failsafe or mode change cancelled it')
        return None

    def _drain_move_acks(self, wait_s: float = MOVE_ACK_DRAIN_S) -> int:
        """Consume the terminal SROT_MOVE ACKs a STOP sets in flight (issue #10).

        The next move used to read them as its own. A stop displaces the
        running sequence, and the firmware answers that displacement -- the old
        sequence's CANCELLED (or ACCEPTED, if it had just completed) -- on its
        stream task's next tick, followed by the STOP's own terminal ACK. Both
        ARRIVE AFTER the following command's send, so no timestamp filter can
        exclude them, and the move ACK carries no sequence id to bind to
        (`result_param2` is always 0 -- an upstream ask). Returning without
        consuming them let `yaw_right` report SUCCEEDED in ~20 ms off the STOP's
        ACCEPTED while the board had only just begun the turn.

        So the exit that CAUSES them waits for them: up to two terminal ACKs
        (displaced + STOP), bounded by `wait_s`, each cleared as it is seen.
        Returns how many were consumed.
        """
        seen, deadline = 0, time.monotonic() + float(wait_s)
        last = None
        while seen < 2 and time.monotonic() < deadline:
            ack = self._cache('COMMAND_ACK')
            if (ack is not None and ack is not last
                    and getattr(ack, 'command', None) == sp.CMD_SROT_MOVE
                    and getattr(ack, 'result', None) in sp.TERMINAL_ACKS):
                seen += 1
                last = ack
                self._clear_ack()
            time.sleep(_POLL_S)
        return seen

    def stop_motion(self) -> None:
        """Bring the vehicle to an actual halt.

        Just MOVE_STOP now. Firmware behaviour rev 2 (fw AUDIT.md R36) made the
        wire-reachable MOVE_STOP brake along the outgoing leg's axis, so the
        host-side reverse-leg brake this used to run is gone -- keeping it would
        kick the hull twice.

        Worth recording why our tripwire did not catch that: we asserted on the
        SHAPE of their C++ (`abort()` appearing in the `Type::STOP` case). They
        fixed it by restoring the axis and speed `start()` had zeroed before the
        switch, so the string never appeared and the test stayed green while the
        behaviour changed. Their own suggested fix would not have worked either,
        for the same reason -- `abort()` does not restore those fields. We now
        assert on `sp.FW_BEHAVIOUR_REV` instead.

        The board still resolves the displaced sequence as CANCELLED and settles
        at zero, which is what makes this usable as the ROS-cancel action.
        """
        self._last_leg = None
        # A STOP is a SROT_MOVE, and every SROT_MOVE puts the board in AUTO. In
        # SURFACE there is nothing to brake (leaving AUTO cancelled the move), so
        # sending it would only undo the surface. See `move`.
        if self.get_mode() == 'SURFACE':
            return
        self._command_long(sp.CMD_SROT_MOVE, p1=float(sp.MOVE_STOP))

    # ------------------------------------------------------------------ #
    #  move() -- the mongla-verb -> SROT_MOVE mapping + ACK state machine #
    # ------------------------------------------------------------------ #
    def move(self, verb: str, *, on_progress=None, abort_fn=None, **kw) -> MoveResult:
        """Run one collapse verb as a SROT_MOVE and relay its four-terminal ACK.

        The verb->wire-params mapping is `_build_params` (the single verb table).
        Never raises: a bad verb/param returns MoveResult(DENIED, ...) so the action
        server always resolves; preemption returns PREEMPTED; a stall TIMEOUT.
        """
        try:
            p1, p2, p3, p4, p5 = _build_params(verb, kw)
        except (KeyError, ValueError) as exc:
            return MoveResult(DENIED, f'{verb}: {exc}')
        if not _finite(p1, p2, p3, p4, p5):
            return MoveResult(DENIED, f'{verb}: non-finite parameter -- refused host-side')
        try:
            settle = float(kw.get('settle') or 0.0)
        except (TypeError, ValueError):
            settle = math.nan
        if not (math.isfinite(settle) and settle >= 0.0):
            return MoveResult(DENIED, f'{verb}: settle={kw.get("settle")!r} -- '
                              f'refused host-side (seconds, >= 0)')
        # An absolute turn is only a compass heading with a locked yaw
        # REFERENCE (#15). Checked HERE, not once at startup: YAW_REF is a live
        # state machine that restarts at SAMPLING after every board reboot.
        # Warned, not refused, deliberately: the search sweeps call
        # `turn(head() + step)`, a target in the board's OWN frame, which is
        # correct whether or not that frame is magnetic. Only a heading taken
        # from the course (GATE_HEADING_DEG ...) is wrong unlocked -- and the verb
        # cannot tell which it was given. The `heading_ref` health line reports
        # the state continuously; this puts it beside the command that needs it.
        if verb == 'turn':
            yr_ok, yr_why = self.check_yaw_reference()
            if not yr_ok:
                self._log_warn(f'[SROT ] turn {p1:+.1f} deg sent with {yr_why} -- '
                               f'correct for a head()-relative target, WRONG for a '
                               f'compass heading from the course')

        # Never send a SROT_MOVE into surface. The firmware's SROT_MOVE handler
        # sets `mode = AUTO` unconditionally (mav_commands.cpp), so the next leg of
        # a mission -- or a plain `stop` -- quietly undoes a leak/battery/GCS
        # failsafe SURFACE, or the operator's own `surface`, and the hull re-dives
        # to hold depth. Leaving SURFACE is a deliberate `set_mode`, never a side
        # effect of a motion verb.
        if self.get_mode() == 'SURFACE':
            if verb == 'stop':
                # Leaving AUTO already cancelled any move; there is nothing to
                # brake, and a STOP would re-enter AUTO.
                return MoveResult(SUCCEEDED, 'stop: board is in SURFACE -- no move '
                                  'can be running; STOP not sent (it would re-enter AUTO)')
            return MoveResult(DENIED, f'{verb}: refused -- the board is in SURFACE '
                              f'(failsafe or operator). A SROT_MOVE would switch it '
                              f'back to AUTO and re-dive. Leave SURFACE deliberately first.')

        # No host-side brake before a 'stop' any more: fw rev 2 brakes on-board.
        self._clear_ack()
        self._command_long(sp.CMD_SROT_MOVE, p1=p1, p2=p2, p3=p3, p4=p4, p5=p5)
        self._last_leg = None
        res = self._relay_move_ack(verb, on_progress, abort_fn,
                                   _ack_budget_s(verb, p1, p2, p5))
        return self._hold_after(verb, res, settle, abort_fn)

    def _hold_after(self, verb, res, settle, abort_fn) -> MoveResult:
        """`settle=N`: a board HOLD for N s after the move's ACK (issue #86).

        The board ends a turn or a dive on ONE sample inside its error band,
        not after the hull has stopped, so a mission that asked for a settle
        used to start its next leg mid-rotation: the keyword was accepted and
        dropped here. The HOLD is the board's own station-keep (the `pause`
        mapping), sent as a second SROT_MOVE through this same path, so an
        abort brakes it and SURFACE refuses it like any other move.

        It is time held, not a measured stop. The result says "held N s" and
        never "settled".
        """
        if settle <= 0.0 or verb in ('stop', 'pause') or not res.ok:
            return res
        if abort_fn is not None and abort_fn():
            return MoveResult(PREEMPTED, f'{res.reason} -- hold {settle:g} s '
                                         f'not started: aborted')
        hold = self.move('pause', duration=settle, abort_fn=abort_fn)
        if hold.ok:
            return MoveResult(res.code, f'{res.reason} -- held {settle:g} s')
        return MoveResult(hold.code, f'{res.reason} -- hold {settle:g} s did not '
                                     f'complete: {hold.reason}')

    def _relay_move_ack(self, verb, on_progress, abort_fn, budget) -> MoveResult:
        """Wait for the terminal ACK. A STALL IS SILENCE, not slowness (B39).

        The MAVLink command protocol is explicit about this: *"The GCS should have
        a much increased timeout after receiving an ACK with
        MAV_RESULT_IN_PROGRESS"* (https://mavlink.io/en/services/command.html).
        This used to arm ONE deadline before the loop and never extend it, so a
        move that was demonstrably alive -- reporting rising progress every tick --
        was declared stalled and BRAKED the moment it exceeded 2x its predicted
        time. Braking a healthy manoeuvre mid-leg is worse than waiting: the leg is
        lost and the mission carries on from somewhere unplanned.

        Now `deadline` is a SILENCE window, refreshed by genuine progress, under an
        absolute ceiling so a board that reports progress for ever still terminates.

        Refresh only on *new* progress. `_cache()` returns the same COMMAND_ACK
        object every poll until a new one lands, so refreshing on every sighting
        would let ONE stale IN_PROGRESS hold the deadline open indefinitely --
        turning the backstop into a hang. That is why the reset is inside the
        `prog != last_prog` branch, and why the branch no longer depends on
        `on_progress` being supplied.
        """
        started = time.monotonic()
        deadline = started + budget
        hard_deadline = started + budget * _STALL_HARD_MULT
        last_prog = -1.0
        while time.monotonic() < min(deadline, hard_deadline):
            if abort_fn is not None and abort_fn():
                self.stop_motion()       # real brake, then STOP -> board CANCELs the seq
                self._drain_move_acks()  # issue #10: leave nothing for the NEXT move
                return MoveResult(ABORTED, f'{verb}: aborted (braked to a stop)')
            ack = self._cache('COMMAND_ACK')
            if ack is not None and ack.command == sp.CMD_SROT_MOVE:
                if ack.result in sp.TERMINAL_ACKS:
                    code = _ACK_TO_CODE[ack.result]
                    if code == SUCCEEDED:
                        cut = self._cut_short_by_mode_change(
                            getattr(ack, '_timestamp', 0.0))
                        if cut is not None:
                            return MoveResult(FAILED, f'{verb}: {cut}')
                    if on_progress is not None and code == SUCCEEDED:
                        on_progress(1.0)
                    return MoveResult(
                        code, self._terminal_reason(verb, code, ack.result))
                if ack.result == sp.ACK_IN_PROGRESS:
                    prog = float(getattr(ack, 'progress', 0)) / 100.0
                    if prog != last_prog:
                        # Genuine forward progress: the board is alive and working.
                        # Refresh the silence window (capped by hard_deadline).
                        deadline = time.monotonic() + budget
                        last_prog = prog
                        if on_progress is not None:
                            on_progress(prog)
            time.sleep(_POLL_S)
        # No terminal ACK inside the budget -> stall. Brake to be safe.
        self.stop_motion()
        self._drain_move_acks()          # issue #10, as on abort
        # WARN HERE, not only in the returned string. A stall is the commonest real
        # failure on this link and it used to be reported ONLY as a MoveResult --
        # visible only if whoever received it chose to surface it, and the DSL
        # logged it at INFO alongside every success. An operator scrolling a pool
        # log had nothing to catch the eye at the exact moment the vehicle stopped
        # responding.
        self._log_warn(
            f'[SROT ] !! STALL: {verb} got no terminal ACK in {budget:.0f}s. The '
            f'board accepted the command and never reported a result -- the hull '
            f'has been braked. Two causes look IDENTICAL from here: a dead link, '
            f'and a board refusing every AUTO move because the Bar30 is unhealthy. '
            f'`ros2 run mongla_manager connect` tells them apart.')
        return MoveResult(TIMEOUT, f'{verb}: no terminal ACK within {budget:.0f}s (stall) -- the board took the '
                f'command but never reported a terminal result. Check the link is '
                f'alive (`ros2 run mongla_manager connect`) and that the board is '
                f'not stuck in a refusing mode; a DEPTH_HOLD/AUTO refusal from an '
                f'unhealthy Bar30 looks exactly like this')

    def _terminal_reason(self, verb, code, result=None) -> str:
        if code == SUCCEEDED:
            return f'{verb}: completed'
        if code == PREEMPTED:
            return f'{verb}: preempted by a newer move'
        st = self._statustext()
        if result == sp.ACK_TEMPORARILY_REJECTED:
            # Distinct from a plain FAILED: the board did not start the move. But
            # fw rev 13 gave this code a SECOND meaning -- SROT_MOVE now requires
            # ARMED and answers "SROT_MOVE refused: arm first" with this same
            # result. Retrying is right for a state-lock miss and useless for a
            # disarmed board, so the two must not print the same advice. The
            # board's own STATUSTEXT is what separates them; report it rather
            # than guessing from the code alone.
            if 'arm first' in st.lower():
                return (f'{verb}: refused -- the board is DISARMED '
                        f'(fw rev 13+ requires ARMED for SROT_MOVE). Arm, then retry.')
            return (f'{verb}: board busy (state lock) -- not started, safe to retry'
                    + (f' ({st})' if st else ''))
        if result == sp.ACK_UNSUPPORTED:
            # The opposite advice to the line above, which is why they must not share
            # a value: this firmware does not implement the verb and never will
            # without a flash. Retrying is pointless.
            return (f'{verb}: NOT SUPPORTED by this firmware -- do not retry; '
                    f'flash a build that implements it')
        if code == FAILED:
            return f'{verb}: could not start' + (f' ({st})' if st else '')
        return f'{verb}: denied' + (f' ({st})' if st else '')

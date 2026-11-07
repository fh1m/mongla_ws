"""SrotPayload: fire(N) on board channel N, over MAVLink. Split out of srot_fc.py."""
import threading
import time

from .base import (FireResult, FIRE_FIRED, FIRE_REJECTED_ARM, FIRE_DISABLED, FIRE_DENIED,
                   FIRE_NO_ACK, FIRE_NOT_READY, FIRE_BUSY)
from . import srot_protocol as sp

# ---------------------------------------------------------------------- #
#  SrotPayload -- payload over MAVLink (replaces the obsolete USB ESP32)  #
# ---------------------------------------------------------------------- #
# There is no host-side channel map, deliberately.
#
# `fire(N)` addresses BOARD channel N -- literally `DO_SET_SERVO param1`, 1..16,
# the same N that names `SERVO{N}_ROLE` and the same N printed by
# `ros2 run mongla_manager connect`. The previous design routed a "mongla channel"
# 1..4 through a `payload_fire_map` onto a PCA channel. That indirection bought
# nothing and cost the two things that actually go wrong:
#
#   * a second numbering to keep in sync with the harness by hand, silently wrong
#     the moment it drifts -- and "silently wrong" here means driving the
#     manipulator arm during a torpedo shot; and
#   * it invited a fixed "1-8 servo / 9-16 switch" folklore that is NOT a firmware
#     rule. That split is only the DEFAULT of `SERVO{n}_ROLE`
#     (fw `params.cpp:368-371`: `(c < 8) ? 1.0f : 2.0f`). Every channel is
#     independently re-rolable from Bondor, so any host table encoding the split
#     is wrong as soon as anyone uses the feature.
#
# Which channels are fireable is FIRMWARE state. So we ask the firmware
# (`SERVO{n}_ROLE`) instead of holding an opinion about it.
_FIRE_PULSE_S = 0.6   # > any PCA service tick; long enough for a servo to travel
_FIRE_ACK_S   = 0.35  # ACK budget. MUST stay < _FIRE_PULSE_S -- see fire().


class SrotPayload:
    """The payload: the board's PCA9685 expander, driven over the same MAVLink
    link as everything else. The facade calls `is_ready` and `fire`; there is
    no separate payload board and nothing to auto-detect.

    `names` is cosmetic only -- {board_channel: label} used in logs and preflight.
    It NEVER routes: `fire(N)` always addresses board channel N, so a stale label
    can mislabel a log line but can never send a shot to the wrong channel.
    """

    def __init__(self, fc, log=None, names=None):
        self._fc = fc
        self._log = log
        self._names = dict(names or {})
        # Board channel (1-based) -> role int, read from the board and cached.
        self._roles: dict = {}
        # Board channel -> SERVOn_FUNCTION int. Identity only; never gates fire().
        self._functions: dict = {}
        # fire() holds a channel energised for _FIRE_PULSE_S and de-energises in a
        # finally. Two overlapping fires would let B's ON land inside A's pulse and
        # A's finally then de-energise while B still believes it is firing -- a
        # truncated shot, from a race, with both callers reporting success. The
        # vision path already ASSUMED this lock existed (vision_verbs.py) when only
        # the legacy USB driver had one.
        self._fire_lock = threading.Lock()

    def _fireable_text(self) -> str:
        """The board's own answer to "which channel, then?" for a refusal.

        From the roles ALREADY read (`preflight_roles` reads all 16 at bring-up);
        never a fresh param round-trip inside a refusal. Says so when nothing has
        been read, rather than implying the board has no switch channels.
        """
        if not self._roles:
            return '(roles not read yet -- `ros2 run mongla_manager connect` lists them)'
        sw = sorted(c for c, r in self._roles.items() if r == sp.PCA_ROLE_SWITCH)
        return (f'(this board: SWITCH {sw})' if sw
                else '(this board has NO switch channel configured)')

    def channel_role(self, channel: int, refresh: bool = False):
        """The board's configured role for a channel, or None if unreadable.

        The role is FIRMWARE state (`SERVO{n}_ROLE`, set in Bondor), not ours. We read
        it rather than keeping a host-side wiring table because a host copy goes stale
        SILENTLY the moment someone re-roles a channel on the board -- and the failure
        mode of a stale copy is driving the manipulator arm during a payload drop.

        Cached, because a param round-trip costs ~50 ms and `fire()` is called at the
        moment a mission is glued to a target. `refresh=True` re-reads; `preflight_roles`
        does that once at bring-up.
        """
        ch = int(channel)
        if refresh or ch not in self._roles:
            val = self._fc.get_param(sp.PCA_ROLE_PARAM_FMT.format(ch))
            if val is None:
                return None
            self._roles[ch] = int(val)
        return self._roles.get(ch)

    def channel_function(self, channel: int, refresh: bool = False):
        """The board's configured FUNCTION for a channel, or None if unreadable.

        Identity, not authority -- see `srot_protocol.PCA_FUNC_*`. `fire()` never
        consults this: a channel is fireable because its ROLE is SWITCH, full stop.
        Mixing the two would let a mislabelled channel become an actuation decision,
        which is the whole thing this design avoids.
        """
        ch = int(channel)
        if refresh or ch not in self._functions:
            val = self._fc.get_param(sp.PCA_FUNC_PARAM_FMT.format(ch))
            if val is None:
                return None
            self._functions[ch] = int(val)
        return self._functions.get(ch)

    def label(self, channel: int) -> str:
        """Display name for a channel: the operator override if set, else whatever
        the BOARD says is wired there, else nothing.

        The override wins because the board's enum is a small fixed vocabulary and
        cannot express "torpedo_1 vs torpedo_2" -- two tubes share one function.
        """
        ch = int(channel)
        if ch in self._names:
            return self._names[ch]
        func = self._functions.get(ch)
        if func:                                  # 0 (NONE) is "unassigned", not a name
            return sp.PCA_FUNC_NAMES.get(func, f'function {func}')
        return ''

    def preflight_roles(self, channels=None):
        """Read + log every channel role at bring-up, so a mis-roled payload is found
        on the deck rather than at the moment a mission tries to drop a marker.

        Defaults to ALL 16 channels: with no host-side map there is no smaller set to
        consult, and the full read is the thing that retires the "1-8 servo / 9-16
        switch" folklore by printing what the board ACTUALLY has.

        Requires the reader thread to be running. `get_param` reads the pymavlink
        message cache and never calls `recv_match()` itself (the stack's threading
        rule), so with no reader every role comes back None and every channel reads
        UNREADABLE -- indistinguishable from a mis-roled board. `auv_manager_node`
        starts the reader BEFORE `_preflight_payload` for exactly this reason.
        """
        if channels is None:
            channels = range(1, sp.PCA9685_NUM_CH + 1)
        report = {}
        for ch in channels:
            # Retry once. This is 16 sequential param round-trips, and on a lossy
            # link a single dropped PARAM_VALUE reports a perfectly healthy channel
            # as UNREADABLE -- which reads exactly like a mis-roled board. MEASURED
            # over a lossy network bridge (~8% frame loss): one channel of the 16
            # came back unreadable on the first pass and fine on a retry. Only here:
            # `fire()` must not spend a second round-trip at the moment a mission is
            # glued to a target.
            role = self.channel_role(ch, refresh=True)
            if role is None:
                role = self.channel_role(ch, refresh=True)
            # FUNCTION rides the SAME loop and the same retry -- one traversal, one
            # policy. Measured over the bridge: 16 reads take ~0.7 s, so 32 is ~1.4 s
            # at bring-up only.
            #
            # Do NOT pipeline these two requests. `get_param` pops the single
            # PARAM_VALUE slot and polls it; two replies in flight means the second
            # overwrites the first, and the first then times out indistinguishably
            # from a dropped frame -- permanently, on a lossy link.
            func = self.channel_function(ch, refresh=True)
            if func is None:
                func = self.channel_function(ch, refresh=True)
            report[int(ch)] = role
        if self._log is not None:
            fireable = sorted(c for c, r in report.items() if r == sp.PCA_ROLE_SWITCH)
            arm = sorted(c for c, r in report.items() if r == sp.PCA_ROLE_SERVO)
            unread = sorted(c for c, r in report.items() if r is None)
            self._log.info(
                f'[PAYLOAD] board roles: FIREABLE (switch) {fireable or "none"} | '
                f'arm/PWM {arm or "none"} | unreadable {unread or "none"}')
            for ch in fireable:
                lbl = self.label(ch)
                self._log.info(f'[PAYLOAD]   ch {ch}: SWITCH -- fire({ch}) will actuate'
                               + (f" ({lbl})" if lbl else ''))
            # A host label AND a board function that disagree is the stale-host-copy
            # detector: someone re-wired the harness and told the board but not the
            # launch file (or the reverse). Neither value is trustworthy then, so say
            # both and say which one is being displayed.
            for ch, lbl in sorted(self._names.items()):
                func = self._functions.get(ch)
                if not func:
                    continue
                board = sp.PCA_FUNC_NAMES.get(func, f'function {func}')
                if board.lower() not in lbl.lower():
                    self._log.warn(
                        f'[PAYLOAD]   ch {ch}: payload_channels says {lbl!r} but the '
                        f'BOARD says {board!r}. Displaying {lbl!r} (the override wins). '
                        f'One of the two is stale -- and if it is the launch file, a '
                        f'mission may be aiming at the wrong device.')
            for ch, lbl in sorted(self._names.items()):
                if report.get(ch) != sp.PCA_ROLE_SWITCH:
                    role = report.get(ch)
                    name = sp.PCA_ROLE_NAMES.get(role, 'UNREADABLE')
                    self._log.error(
                        f'[PAYLOAD]   ch {ch} ({lbl}) is named in payload_channels but '
                        f'its board role is {name} -- fire({ch}) will REFUSE. '
                        f'Set SERVO{ch}_ROLE=2 in Bondor, or fix the name list.')
            if self._roles and not self._functions:
                # Roles came back but not one function. Without this line the
                # [PAYLOAD] block simply loses every name, which looks like an
                # unconfigured board rather than a read that failed.
                self._log.warn(
                    '[PAYLOAD] no SERVOn_FUNCTION could be read -- channel names '
                    'will be blank. The board may predate the payload-identity enum, '
                    'or the reads dropped; roles (which gate firing) are unaffected.')
            if unread:
                self._log.error(
                    f'[PAYLOAD] roles unreadable on {unread} -- fire() FAILS CLOSED on '
                    f'those. Is the reader thread up and the link healthy?')
        return report

    @property
    def is_ready(self) -> bool:
        # The payload rides the same MAVLink link as everything else.
        return bool(getattr(self._fc, 'link_alive', lambda: True)())

    def fire(self, channel: int) -> FireResult:
        """Activate BOARD channel `channel` (1..16) as a bounded pulse.

        `channel` is `DO_SET_SERVO param1` -- the same number as `SERVO{n}_ROLE`.
        There is no mapping step and nothing is looked up to decide WHERE to send.

        The only decision made here is WHETHER to send, and it is made from the
        board's own role config, not from a host opinion.
        """
        ch = int(channel)
        if ch == 0:
            return FireResult(
                FIRE_DENIED, ch,
                f'payload channel NOT ASSIGNED (0) -- set it in '
                f'missions/competition_config.py from a SWITCH channel '
                f'{self._fireable_text()} (B53)')
        if ch < 1 or ch > sp.PCA9685_NUM_CH:
            return FireResult(FIRE_DENIED, ch,
                              f'channel {ch} out of range 1..{sp.PCA9685_NUM_CH}')
        if not self.is_ready:
            return FireResult(FIRE_NOT_READY, ch, 'MAVLink link is down')

        # ---- ROLE GATE -- why the host still decides -------------------- #
        # The user-facing contract is "the board decides and tells us". The firmware
        # does NOT do that yet: DO_SET_SERVO on a role-1 channel WRITES servo_us and
        # returns ACCEPTED (fw `mav_commands.cpp:475-481`) -- i.e. it silently moves
        # the manipulator arm and reports success. Until the board gains a
        # role-enforcing payload command, refusing here is the only thing standing
        # between a mission `fire()` and the arm.
        #
        # Fails CLOSED on an unreadable role: "the param read timed out" is not
        # evidence that a channel is safe to drive.
        role = self.channel_role(ch)
        if role is None:
            return FireResult(FIRE_NOT_READY, ch,
                              f'SERVO{ch}_ROLE unreadable -- refusing to guess')
        if role == sp.PCA_ROLE_SERVO:
            return FireResult(
                FIRE_REJECTED_ARM, ch,
                f'channel {ch} is a PWM/SERVO channel (the on-board arm). '
                f'mongla_ws drives SWITCH channels only {self._fireable_text()}. '
                f'Set SERVO{ch}_ROLE=2 in Bondor if this really is a payload '
                f'channel.')
        if role != sp.PCA_ROLE_SWITCH:
            return FireResult(
                FIRE_DISABLED, ch,
                f'channel {ch} role is {role} (disabled) -- driving it would be a '
                f'silent no-op on the board {self._fireable_text()}.')

        # ---- ACTUATE ---------------------------------------------------- #
        # Non-blocking: a queued shot that outlives its align scope is worse than a
        # refused one, because it fires after the hull has moved off target.
        if not self._fire_lock.acquire(blocking=False):
            return FireResult(FIRE_BUSY, ch, 'another fire is mid-pulse')
        try:
            t0 = time.monotonic()
            ack = self._fc.set_servo_acked(ch, sp.PCA_SWITCH_ON_US, _FIRE_ACK_S)
            # The pulse is timed from the SEND, not from the ACK -- otherwise the ACK
            # wait is added to the time the solenoid coil is energised.
            remain = _FIRE_PULSE_S - (time.monotonic() - t0)
            if remain > 0:
                time.sleep(remain)
            return self._ack_to_result(ch, ack)
        finally:
            # Always de-energise. A MOSFET held on burns the solenoid coil, and NO
            # board-side failsafe clears it: leak, disarm and GCS-loss all leave an
            # energised channel energised (only a reboot clears AuxState).
            try:
                off = self._fc.set_servo_acked(ch, sp.PCA_SWITCH_OFF_US, _FIRE_ACK_S)
                if off is None and self._log:
                    self._log.error(
                        f'[PAYLOAD] ch {ch}: NO ACK for the OFF command. The board '
                        f'latches switch outputs and no failsafe clears them -- '
                        f'assume the channel may still be ENERGISED.')
            except Exception as exc:   # noqa: BLE001 -- best-effort de-energise
                if self._log:
                    self._log.error(f'[PAYLOAD] ch {ch}: de-energise raised {exc!r}')
            self._fire_lock.release()

    def _ack_to_result(self, ch: int, ack) -> FireResult:
        """Map the board's COMMAND_ACK for the ON command onto a FireResult."""
        lbl = self._names.get(ch)
        who = f'channel {ch}' + (f' ({lbl})' if lbl else '')
        if ack is None:
            return FireResult(FIRE_NO_ACK, ch,
                              f'{who}: no COMMAND_ACK within {_FIRE_ACK_S:.2f}s -- '
                              f'outcome UNKNOWN, assume it did not fire')
        if ack == sp.ACK_ACCEPTED:
            return FireResult(FIRE_FIRED, ch, f'{who}: board accepted the activation')
        if ack == sp.ACK_TEMPORARILY_REJECTED:
            # Board state lock was busy (fw `mav_commands.cpp:464`) -- not started,
            # and unlike the other failures this one is safe to retry immediately.
            return FireResult(FIRE_BUSY, ch,
                              f'{who}: board busy (state lock) -- not fired, safe to retry')
        name = sp.ACK_NAMES.get(ack, str(ack))
        st = self._fc._statustext() if hasattr(self._fc, '_statustext') else ''
        return FireResult(FIRE_DENIED, ch,
                          f'{who}: board answered {name}' + (f' ({st})' if st else ''))

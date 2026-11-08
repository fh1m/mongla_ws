#!/usr/bin/env python3
"""SrotFC -- the host side of the SROT board (firmware Hengla).

SROT speaks MAVLink 2 (sysid/compid 1/1) and owns the motion primitives on-board:
attitude + depth hold, thrust allocation, timed moves with braking, heading hold,
failsafes. So this backend is thin -- it sends **intent** and relays the board's
own progress/terminal ACKs. The wire constants all live in `srot_protocol.py`.

Threading rule: only the manager's reader thread calls `recv_match()`; every
read here is a non-blocking `master.messages` cache lookup. Every write goes
through `self._tx_lock` (pymavlink shares one seq counter + encode buffer across
threads).

What the board does NOT have, so nothing here pretends to:
  * no depth-setpoint stream -- depth is a mode (DEPTH_HOLD latches the current
    depth) or a DIVE move;
  * no RC override and no per-channel release -- MANUAL_CONTROL only, all four
    axes always sent;
  * no RC heartbeat -- only the >=1 Hz MAVLink HEARTBEAT matters (5 s of silence
    and the board surfaces).

The board's depth loop has never run closed in AUTO (AUDIT R1); it closed in
DEPTH_HOLD only in the firmware team's 2026-08-06/07 water test, on the old hull,
and is bench-unverified under this stack. The facade gates dive-dependent verbs; this layer just sends them.

Where each part lives. `SrotFC` is one class assembled from one file per concern,
so a bug report names a file:

  srot_fc.py         the link, arm/disarm, modes, heartbeats, the verb wrappers
  srot_motion.py     move(): one SROT_MOVE and its ACK relay; manual(), demand()
  srot_moves.py      the move envelope, the ACK budget, SROT_MOVE parameters
  srot_arming.py     the arming interlocks: firmware rev, thruster power, depth loop
  srot_telemetry.py  attitude, IMU, batteries, the telemetry snapshot
  srot_health.py     STATUSTEXT, the boot burst, reboots, ESC/thruster health, leak
  srot_params.py     PARAM read/set, stream rates, the preflight parameter checks
  srot_bench.py      motor test / detect, autotune, motor tune (props clear only)
  srot_uplink.py     vision target, speed and position sent TO the board
  srot_payload.py    `SrotPayload`: fire(N) on board channel N
  _srot_base.py      the shared names, and the MAVLink 2 import every part uses
"""

from __future__ import annotations

import collections
import threading
import time

from ._srot_base import mavutil, _ARMED_FLAG, _POLL_S, _LINK_STALE_S, _rx_age
from .srot_bench import SrotBench
from .srot_health import SrotHealth
from .srot_params import SrotParams
from .srot_telemetry import SrotTelemetry
from .srot_uplink import SrotUplink

from . import srot_protocol as sp
from .srot_arming import SrotArming
from .srot_motion import SrotMotion


_STATUSTEXT_RING = 64      # a boot burst is ~13 lines; this holds several


class SrotFC(SrotArming, SrotMotion, SrotHealth, SrotTelemetry, SrotBench, SrotUplink, SrotParams):
    """The board. Constructed with a pre-opened mavutil master + optional logger
    (the connection lives in the manager / connection_config)."""

    name = 'srot'

    def __init__(self, master, log=None):
        self.master = master
        self._log = log
        self._tx_lock = threading.Lock()
        self._param_table: dict = {}   # name -> (value, monotonic rx) (#17)
        self._param_fed = False
        self._boot_time = time.time()
        # Last HEARTBEAT from the VEHICLE (autopilot != INVALID). Our own GCS
        # heartbeat and any other GCS on the link (e.g. Bondor) send
        # MAV_AUTOPILOT_INVALID; the reader thread caches the latest HEARTBEAT
        # regardless of source, so mode/armed must prefer the vehicle's.
        self._last_vehicle_hb = None
        # (move_type, speed) of the last translation leg sent, for the abort brake.
        self._last_leg = None
        self._last_mc_mode_warn = 0.0
        # Highest ATTITUDE.time_boot_ms seen, for the unplanned-reboot detector.
        # None until the first sample. See `check_for_reboot()`.
        self._peak_boot_ms = None
        self._reboots = 0
        # Board's SROT_FW_BEHAVIOUR_REV once read: int, or None while unknown.
        self._behaviour_rev = None
        self._behaviour_rev_logged = False
        # Operator escape hatch (ROS param `allow_fw_behaviour_mismatch`). Off by
        # default: the failure this guards is silent, so opting into it must not be.
        self.allow_fw_behaviour_mismatch = False
        # Board's DEPTH_P, read once at preflight. `check_depth_loop_settled` converts
        # DEPTH_CMD back into metres of depth error through it, so a retune of the gain
        # cannot silently move the physical threshold. None -> the firmware default.
        self.depth_p = None
        # name -> (value, wall-clock stamp). Our own de-multiplexing of NAMED_VALUE_FLOAT,
        # because pymavlink keeps one message per msgid and SROT rides ~15 names on this one.
        self._named_cache = {}
        # STATUSTEXT ring. Same one-slot hazard as NAMED_VALUE_FLOAT: the board
        # bursts ~13 announcements at boot back to back, and pymavlink keeps one
        # message per msgid, so sampling the slot sees the LAST one and loses the
        # rest -- including "Params reset to build defaults", which silently
        # returns JS_GAIN_DEFAULT to 0.5 and LEAK_EN to 0.
        self._statustext_log = collections.deque(maxlen=_STATUSTEXT_RING)
        # The boot burst, kept OUT of the ring: a chatty board can push 64 lines
        # through before preflight reads it, and these are the ones that must
        # not be lost. None until the burst's first line ('RST: ...') arrives,
        # so "not seen" and "seen, nothing wrong" are different answers.
        self._boot_lines = None
        # Per-thruster telemetry presence, parsed from the board's own arm-time
        # announcements. None until the board has said something -- absence is
        # not "all healthy". See `esc_presence()`.
        self._esc_present = None
        self._esc_lost = set()
        self._last_nvf = None
        # battery id -> (voltage_v, current_a, stamp). SAME single-slot trap as
        # NAMED_VALUE_FLOAT, one layer down: the board sends TWO BATTERY_STATUS
        # instances (0 = PM1 electronics, 1 = PM2 thruster pack over ESP-NOW) and
        # pymavlink caches one message per MSGID, not per instance.
        self._battery_cache = {}
        self._battery_steps = {}       # id -> recent |dV| between samples
        self._battery_last = {}        # id -> previous voltage
        self._last_batt = None
        # Separate from allow_fw_behaviour_mismatch ON PURPOSE. That flag means "I
        # accept an unknown firmware revision"; this one means "I accept that the
        # depth loop is currently demanding full heave". They are different risks and
        # an operator who needs one must not be forced to silence the other.
        self.allow_saturated_depth_arm = False

    def _vehicle_hb(self):
        """Latest HEARTBEAT from the vehicle, ignoring GCS/loopback frames.

        The board reports MAV_AUTOPILOT_GENERIC; a GCS/companion reports
        MAV_AUTOPILOT_INVALID. Test doubles omit the field -> default 0 (GENERIC)
        so a scripted vehicle heartbeat is accepted."""
        msg = self._cache('HEARTBEAT')
        if msg is None:
            return self._last_vehicle_hb
        if getattr(msg, 'autopilot', 0) == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            return self._last_vehicle_hb        # GCS / our own loopback -- skip
        self._last_vehicle_hb = msg
        return msg

    # ------------------------------------------------------------------ #
    #  Low-level send / read helpers                                      #
    # ------------------------------------------------------------------ #
    def _command_long(self, command, p1=0.0, p2=0.0, p3=0.0, p4=0.0,
                      p5=0.0, p6=0.0, p7=0.0, confirmation=0):
        if command == sp.CMD_SROT_MOVE:
            # An on-board primitive ramps its own demand, so the host's record
            # of what it sent stops being true. Only the board's DEM_* echo
            # (firmware with #33) can say what the thrusters are asked now.
            self._demand = None
        with self._tx_lock:
            self.master.mav.command_long_send(
                sp.VEHICLE_SYSID, sp.VEHICLE_COMPID, command, confirmation,
                float(p1), float(p2), float(p3), float(p4),
                float(p5), float(p6), float(p7))

    def _cache(self, msgtype):
        return self.master.messages.get(msgtype)

    def _clear_ack(self):
        """Discard the previous command's replies before issuing the next one.

        STATUSTEXT goes with the ACK, and that is not tidiness. pymavlink keeps
        ONE message per msgid, and STATUSTEXT is the ONLY thing separating two
        opposite operator instructions: fw rev 13 answers both "board busy,
        safe to retry" and "refused -- arm first" with ACK_TEMPORARILY_REJECTED,
        and `_terminal_reason` tells them apart by reading the board's text.

        Left uncleared, an "arm first" from an earlier disarmed attempt sits in
        that slot indefinitely. The next genuine state-lock rejection then reads
        as "the board is DISARMED. Arm, then retry." on a board that is armed --
        so the operator acts on the wrong half of a fork whose whole purpose is
        to be unambiguous. Same one-slot hazard as PARAM_VALUE and
        NAMED_VALUE_FLOAT, but here it is load-bearing.
        """
        self.master.messages.pop('COMMAND_ACK', None)
        self.master.messages.pop('STATUSTEXT', None)

    def _named_value(self, name, max_age_s: float = 3.0):
        """Latest NAMED_VALUE_FLOAT for `name`, or None if it has not arrived recently.

        Why this is not a one-line cache read. SROT multiplexes MV_STATE / LEAK / WTEMP /
        Gain / curr / kill and a dozen stack counters onto NAMED_VALUE_FLOAT, and pymavlink
        keeps exactly ONE message per msgid. Reading `master.messages['NAMED_VALUE_FLOAT']`
        and comparing `.name` therefore succeeds only when the value you want happened to be
        the most recent to arrive -- roughly 1 call in 15. Against LEAK that is not a latency
        problem, it is a LOTTERY on whether we ever observe a flooding hull.

        The reader-side fix is to keep our OWN per-name table rather than sampling a slot
        fifteen other names are overwriting.

        (The board also publishes LEAK on the SYS_STATUS extended health bits now -- the
        structurally correct home for it, and verified on the wire. We cannot read it:
        pymavlink 2.4.49's SYS_STATUS schema has 13 fields and no extensions, so those bytes
        are parsed away before we see them. See srot_protocol.SYS_STATUS_HAS_EXTENDED_HEALTH.)

        `max_age_s` matters for the same reason absence matters everywhere else here: a value
        that stopped arriving must read None, not its value from four minutes ago.
        """
        self._drain_named()
        hit = self._named_cache.get(name)
        if hit is None:
            return None
        value, stamp = hit
        return value if (time.monotonic() - stamp) <= max_age_s else None

    def flare_order(self, max_age_s: float = 3.0):
        """The run order the board latched, or None.

        Returns {'colours': (...), 'nonce': int, 'age_s': float}, where age_s is the
        board's own count of seconds since the latch. None until all three FLARE_* values
        are fresh and FLARE_ORD decodes -- the board publishes nothing before an order
        arrives, so None is the honest "no order", never a default.
        """
        colours = sp.flare_order_from_digits(self._named_value('FLARE_ORD', max_age_s))
        nonce = self._named_value('FLARE_NON', max_age_s)
        age = self._named_value('FLARE_AGE', max_age_s)
        if colours is None or nonce is None or age is None:
            return None
        return {'colours': colours, 'nonce': int(nonce), 'age_s': float(age)}

    def _drain_named(self):
        """Fold the currently-cached NAMED_VALUE_FLOAT into our per-name table.

        Cheap and idempotent. It cannot recover names overwritten between calls -- only a
        reader-thread hook can (see `note_named_value`) -- but it means each name survives in
        our table for `max_age_s` instead of only until the next NAMED_VALUE_FLOAT of ANY
        name lands.

        Each message is folded once, and that identity check is load-bearing: pymavlink
        never clears its slot, so re-folding the same object would re-stamp it as fresh on
        every call and `max_age_s` could never fire -- a value from a dead link would read
        as current forever, which is the exact failure the freshness stamp exists to catch.
        """
        msg = self._cache('NAMED_VALUE_FLOAT')
        if msg is not None and msg is not self._last_nvf:
            self._last_nvf = msg
            self.note_named_value(msg)

    def note_named_value(self, msg):
        """Hook for the manager's MAVLink reader thread.

        The reader sees EVERY NAMED_VALUE_FLOAT; this object only ever sees whichever one
        last landed in pymavlink's single slot. Calling this from the reader makes the
        multiplexed telemetry lossless -- it is what turns LEAK from probable into
        deterministic. Optional: without it `_drain_named()` degrades to sampling.
        """
        mname = getattr(msg, 'name', '')
        mname = mname.decode() if isinstance(mname, bytes) else str(mname)
        mname = mname.strip('\x00').strip()
        if mname:
            self._named_cache[mname] = (float(msg.value), time.monotonic())

    def _log_info(self, msg):
        if self._log is not None:
            self._log.info(msg)

    def _log_warn(self, msg):
        if self._log is not None:
            self._log.warning(msg)

    # ------------------------------------------------------------------ #
    #  Liveness / heartbeat                                               #
    # ------------------------------------------------------------------ #
    def send_gcs_heartbeat(self) -> None:
        """>=1 Hz companion HEARTBEAT -- MANDATORY: 5 s of silence trips the board's
        GCS failsafe and SURFACEs the vehicle mid-mission.

        ONBOARD_CONTROLLER, not GCS: we are a companion computer, and now that we
        also identify as compid 191 (MAV_COMP_ID_ONBOARD_COMPUTER) declaring
        MAV_TYPE_GCS would contradict that on the same frame -- exactly the
        ambiguity the compid change exists to remove.

        Safe: nothing reads the field. The firmware's heartbeat handler branches on
        sysid/compid only (fw mav_commands.cpp:687), and our own `_vehicle_hb`
        filter keys on `autopilot == MAV_AUTOPILOT_INVALID`, which is unchanged --
        so our loopback frames are still correctly ignored.
        """
        with self._tx_lock:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def link_alive(self) -> bool:
        hb = self._vehicle_hb()
        if hb is None:
            return False
        age = _rx_age(hb)
        return age <= _LINK_STALE_S

    def is_armed(self) -> bool:
        hb = self._vehicle_hb()
        return bool(hb and (hb.base_mode & _ARMED_FLAG))

    # ------------------------------------------------------------------ #
    #  Arm / disarm / mode                                               #
    # ------------------------------------------------------------------ #
    def arm(self, timeout: float = 15.0, abort=None):
        """COMPONENT_ARM_DISARM p1=1. ACK carries a rejection (pre-arm), then poll
        HEARTBEAT for the armed bit. Honours ``abort`` (()->bool) mid-poll.

        Gated on the board's firmware behaviour revision: arming is the last point
        before anything can move, and every path to motion goes through it, so it is
        where a known-too-old board has to be turned away. See `check_behaviour_rev`.
        """
        ok, reason = self.check_behaviour_rev(for_arm=True)
        if not ok:
            return False, reason
        ok, reason = self.check_depth_loop_settled()
        if not ok:
            return False, reason
        ok, reason = self.check_thruster_power()
        if not ok:
            return False, reason
        return self._arm_disarm(True, timeout, abort)

    def disarm(self, timeout: float = 15.0):
        return self._arm_disarm(False, timeout, abort=None)

    def _arm_disarm(self, arm: bool, timeout: float, abort):
        want = 1.0 if arm else 0.0
        self._clear_ack()
        self._command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            p1=want)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if abort is not None and abort():
                # Cancelled mid-arm: disarm and VERIFY it took (a dropped frame must
                # not strand the hull armed while the caller believes it aborted).
                # Fail-closed: re-send a few times, require is_armed()==False to hold.
                for _ in range(6):
                    self._command_long(
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=0.0)
                    time.sleep(_POLL_S)
                    if not self.is_armed():
                        return False, 'ABORTED'
                return False, 'ABORTED_DISARM_UNCONFIRMED'
            ack = self._cache('COMMAND_ACK')
            if ack is not None and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if ack.result == sp.ACK_FAILED:
                    st = self._statustext()
                    return False, f'PREARM_REJECTED: {st}' if st else 'PREARM_REJECTED'
                if ack.result == sp.ACK_DENIED:
                    return False, 'DENIED'
            if self.is_armed() == arm:
                return True, 'ARMED' if arm else 'DISARMED'
            time.sleep(_POLL_S)
        return False, 'NO_STATE_CHANGE (timeout)'

    def set_mode(self, mode: str, timeout: float = 8.0):
        """DO_SET_MODE (176) with custom_mode. Refusal (e.g. DEPTH_HOLD/AUTO with no
        depth sensor -> board falls back to STABILIZE + STATUSTEXT) is reported."""
        target = sp.mode_int(mode)
        if target is None:
            return False, (f"unknown SROT mode '{mode}' -- valid: "
                       f"{', '.join(sorted(sp.MODE_INTS))}")
        sent = time.monotonic()
        self._command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            p1=float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            p2=float(target))
        deadline = sent + timeout
        while time.monotonic() < deadline:
            hb = self._vehicle_hb()
            # Only a heartbeat heard since the command. pymavlink keeps the last
            # one forever, so a cache that already showed the target -- or a link
            # that died minutes ago in that mode -- confirmed the change at once,
            # and the caller drove on a mode the board may not be in. The vision
            # verbs rely on this to know MANUAL_CONTROL will be honoured.
            # (Cloud audit, 2026-11-07.)
            if (hb is not None and int(hb.custom_mode) == target
                    and _rx_age(hb) < time.monotonic() - sent):
                return True, mode
            time.sleep(_POLL_S)
        cur = sp.mode_name(getattr(self._vehicle_hb(), 'custom_mode', -1))
        st = self._statustext()
        return False, f'mode stayed {cur}' + (f' ({st})' if st else '')

    # ------------------------------------------------------------------ #
    #  Manual (the streamed servo primitive)                             #
    # ------------------------------------------------------------------ #

    # Modes in which the firmware overwrites the pilot axes, so a MANUAL_CONTROL
    # frame sent in them is discarded with no error. Derived from the mode switch
    # in `task_control_loop.cpp:computeDemands` -- see `manual()` for the lines.
    # DEPTH_HOLD is deliberately ABSENT: it discards only heave (the depth PID owns
    # it) while surge/sway/yaw pass through, which is a legitimate driving mode.
    _MANUAL_DISCARDING_MODES = frozenset({'AUTO', 'SURFACE'})
    _MANUAL_WARN_PERIOD_S = 5.0   # 20 Hz stream: warn at most once per 100 frames

    # A MANUAL_CONTROL frame older than this is not a demand in force: the
    # board's own pilot-input timeout is longer, but a writer that stopped
    # streaming has stopped commanding, and a stale demand would be modelled
    # as a live one.
    DEMAND_FRESH_S = 0.25

    # The board's own echo of what its mixer was handed (DEM_FWD / DEM_LAT,
    # srot-control-board PR #33). It rides the NAMED_VALUE_FLOAT block, whose
    # default period is 500 ms, so two periods is the oldest echo still in force.
    DEMAND_ECHO_FRESH_S = 1.0

    # -- payload (PCA9685 on the board -- integrated, no separate USB ESP32) --- #
    def set_servo(self, channel_1based: int, us: int) -> None:
        """DO_SET_SERVO (183): drive PCA9685 servo `channel_1based` (1-based, so PCA
        ch 0 -> channel_1based=1) to `us` µs. Raw firmware match -- the exact command
        the SROT board dispatches to its servo expander.

        CLAMPED to SERVO_MIN_US..SERVO_MAX_US, and it says so when it clamps.
        Those constants existed and were unused, so the host sent whatever it was
        given and relied on the expander to cope -- the same shape as relying on
        the board to clamp `gain`. A limit enforced only at the far end is not a
        limit this side can reason about, and on a role-2 switch channel (>=1500
        = ON) an out-of-band value is indistinguishable from a deliberate one.
        """
        raw = int(us)
        clamped = max(sp.SERVO_MIN_US, min(sp.SERVO_MAX_US, raw))
        if clamped != raw:
            self._log_warn(
                f'[SROT ] set_servo ch{int(channel_1based)}: {raw} us is outside '
                f'{sp.SERVO_MIN_US}-{sp.SERVO_MAX_US}, clamped to {clamped}')
        self._command_long(sp.CMD_DO_SET_SERVO, p1=float(int(channel_1based)),
                            p2=float(clamped))

    def set_servo_acked(self, channel_1based: int, us: int, timeout: float):
        """`set_servo` + the board's COMMAND_ACK result, or None if it never answered.

        The payload path needs to know what the board DID with the command, and the
        only feedback the firmware offers is this ACK -- there is no actuator readback
        of any kind. Separate from `set_servo` because the plain form is a raw
        fire-and-forget wire wrapper and several callers want it that way.

        ON and OFF are the SAME command id (183) and pymavlink keeps one message
        per msgid, so the ACK slot MUST be cleared immediately before each send or
        this reads the previous pulse's answer. That is the same single-slot hazard
        documented at length in `_named_value`.

        Clearing the cache is NOT sufficient on its own, and this was MEASURED:
        probing the live board with back-to-back DO_SET_SERVO commands read the
        PREVIOUS command's DENIED as the next command's answer, because clearing
        pymavlink's cache does nothing about an ACK still sitting in the socket
        buffer -- the reader parses it in a moment later and it lands in the slot
        looking fresh. Filtering on the command id cannot separate them either:
        ON and OFF are both 183. So also require the ACK to be NEWER than our send.
        """
        self._clear_ack()
        sent = time.monotonic()
        self.set_servo(channel_1based, us)
        deadline = sent + timeout
        while time.monotonic() < deadline:
            ack = self._cache('COMMAND_ACK')
            # Heard since the send, on the reader's monotonic stamp (`_rx_age`).
            # It compared wall clocks, and an ACK with no stamp at all counted as
            # fresh -- the default was the send time itself. The Pi has no RTC,
            # so the first NTP sync at the pool can step the wall clock between
            # the send and the answer. (Cloud audit, 2026-11-07.)
            if (ack is not None and ack.command == sp.CMD_DO_SET_SERVO
                    and _rx_age(ack) < time.monotonic() - sent):
                return int(ack.result)
            time.sleep(_POLL_S)
        return None

    def send_heartbeat(self):
        """Alias: the manager's `_HeartbeatThread` calls this -- on SROT it IS the
        mandatory >=1 Hz GCS HEARTBEAT (2 Hz thread > the 1 Hz failsafe floor)."""
        self.send_gcs_heartbeat()

    def send_neutral(self):
        """Safe idle: zero MANUAL_CONTROL with heave neutral.

        NOT "STABILIZE holds", which is what this said until B28. After any
        SROT_MOVE the board is still in AUTO, where this frame is discarded
        outright -- benign here only because the frame is all zeros and AUTO's
        own idle demand is also zero. Only a defensive fallback on SROT;
        stop/pause route through move()."""
        self.manual(0.0, 0.0, 0.0, 0.0)


# The verbs the board runs as one SROT_MOVE. Every other verb is host-side:
# arm/disarm/set_mode, fire, motor_test and the two vision verbs (which stream
# MANUAL_CONTROL in STABILIZE, never entering AUTO).
MOVE_VERBS = frozenset({
    'move_forward', 'move_back', 'move_left', 'move_right',
    'yaw_left', 'yaw_right', 'turn', 'set_depth', 'stop', 'pause', 'style_roll',
})

#!/usr/bin/env python3
"""SrotFC -- the SROT/Hengla flight-controller backend behind the FlightController HAL.

SROT speaks MAVLink 2 (sysid/compid 1/1) and owns the motion primitives on-board:
attitude + depth hold, thrust allocation, timed moves with braking, heading hold,
failsafes. So this backend is thin -- it sends **intent** and relays the board's
own progress/terminal ACKs. The wire constants all live in `srot_protocol.py`.

THREADING RULE (inherited from `pixhawk.py`): only the manager's reader thread
calls `recv_match()`; every read here is a non-blocking `master.messages` cache
lookup. Every write goes through `self._tx_lock` (pymavlink shares one seq counter
+ encode buffer across threads).

What does NOT port from Pixhawk (see DUBURI_WS_INTEGRATION.md / the plan):
  * `SET_POSITION_TARGET_*` depth-setpoint stream -> UNSUPPORTED on SROT; depth is
    a mode (DEPTH_HOLD latches current) or a DIVE move.
  * `RC_CHANNELS_OVERRIDE` / per-channel release -> SROT uses MANUAL_CONTROL, all
    four axes always sent (no NO_OVERRIDE).
  * `SET_MESSAGE_INTERVAL` -> SROT rates are fixed on-board; the manager must skip it.
  * The 5 Hz neutral-RC "heartbeat" (heartbeat.py) is harmful here (fights an AUTO
    move); only the >=1 Hz MAVLink HEARTBEAT matters (5 s silence -> board surfaces).

⚠ The board's depth loop has never run closed (AUDIT R1) -- DIVE/DEPTH_HOLD are
bench-unverified. The facade gates dive-dependent verbs; this layer just sends them.
"""

from __future__ import annotations

import collections
import math
import os
import re
import struct
import threading
import time

# MUST precede the pymavlink import, exactly as `duburi_control.pixhawk` does.
#
# pymavlink binds ONE dialect module at import time, chosen from the environment, and
# the default is `dialects.v10.ardupilotmega` -- MAVLink *1*. ESC_TELEMETRY_1_TO_4
# (11030) and _5_TO_8 (11031) do not exist there, so `decode()` returns
# `UNKNOWN_11030` and `/duburi/esc_rpm` reads nothing. Silently: no error, no
# callback, just zero RPM that looks exactly like an ESC or wiring fault. This is the
# same trap that cost Bondor every RPM packet via ESC_STATUS.
#
# It happened to work only because `duburi_control/__init__` reaches `pixhawk.py`
# first and pixhawk.py sets this. Anything importing `fc.srot_fc` on its own --
# bringup_check, a test, a script -- got MAVLink 1 and lost the ESC telemetry. Setting
# it here too is idempotent and removes the import-order dependency.
#
# MEASURED on the board (COM19, in-vehicle): with the default dialect the board's own
# ESC frames arrive and are reported as UNKNOWN_291 / UNKNOWN_11030 / UNKNOWN_11031.
os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil                      # noqa: E402

from .base import (FlightController, Telemetry, MoveResult, FireResult,
                   SUCCEEDED, PREEMPTED, FAILED, DENIED, TIMEOUT, ABORTED,
                   FIRE_FIRED, FIRE_REJECTED_ARM, FIRE_DISABLED, FIRE_DENIED,
                   FIRE_NO_ACK, FIRE_NOT_READY, FIRE_BUSY)
from . import srot_protocol as sp


# COMMAND_ACK.result -> MoveResult code. Terminal set + values live in
# srot_protocol (pinned, since older pymavlink dialects lack CANCELLED=6).
# TEMPORARILY_REJECTED means the board missed a state mutex at dispatch, so the
# move never started -- same outcome as FAILED for a caller, different reason text.
_ACK_TO_CODE = {sp.ACK_ACCEPTED: SUCCEEDED, sp.ACK_CANCELLED: PREEMPTED,
                sp.ACK_FAILED: FAILED, sp.ACK_DENIED: DENIED,
                sp.ACK_TEMPORARILY_REJECTED: FAILED,
                # UNSUPPORTED is a DENIED, not a FAILED: the board will never run
                # this command, so "could not start" invites a retry that cannot
                # ever succeed. It is how an older board answers a newer verb.
                sp.ACK_UNSUPPORTED: DENIED}

_ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

_POLL_S = 0.05    # cache-poll granularity for ack/arm/mode loops

# SYS_STATUS wire layout, from the firmware's own vendored header
# (lib/mavlink/common/mavlink_msg_sys_status.h): MIN_LEN 31 is the base message,
# LEN 43 includes the three uint32 extended-health fields at 31 / 35 / 39.
# ESC_STATUS (291): removed from upstream `common`, so pymavlink has no entry
# for it and surfaces the frame as UNKNOWN_291 -- unvalidated. crc_extra from
# the message definition; LEN is the full payload before v2 zero-truncation.
_ESC_STATUS_CRC_EXTRA = 10
_ESC_STATUS_LEN = 57
_STATUSTEXT_RING = 64      # a boot burst is ~13 lines; this holds several
_SYS_STATUS_BASE_LEN = 31
_SYS_STATUS_EXT_END  = 43
_LINK_STALE_S = 3.0

_ACK_MARGIN_S     = 5.0    # slack over the expected leg time before calling it a stall
_ACK_MIN_BUDGET_S = 8.0    # floor, so a 0.5 s leg still tolerates a slow first ACK
_STYLE_ROLL_S     = 360.0 / 90.0   # MOVE_STYLE is always a roll at 90 deg/s


def _finite(*vals) -> bool:
    """True iff every value is a finite float (goal validation before send)."""
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals)


# A progressing move may refresh its silence window this many times over before
# the absolute ceiling stops it. Bounds the worst case at a number an operator can
# predict, without punishing a board that is merely slower than predicted.
_STALL_HARD_MULT = 4.0


def _ack_budget_s(verb: str, p1: float, p2: float, p5: float) -> float:
    """How long to wait for a terminal ACK before declaring a stall.

    Why this is not just `p5 + margin`: p5 is the board's own *safety* timeout and
    is 0 (-> the board's 60 s default) for every verb whose `commands.py` row has no
    `timeout` field -- move_forward/left/right, stop, pause. A 3 s move would then
    hold the action thread for 65 s before giving up.

    This used to be the ONLY terminator: a failsafe or operator mode change mid-leg
    froze `mv_active` true and the board streamed IN_PROGRESS forever, so nothing
    but this deadline could end the action.

    Firmware behaviour rev 2 (fw AUDIT.md R35) fixed that at the source -- the board
    now ends the move when AUTO is displaced and publishes `mv_*` unconditionally,
    so the falling edge that latches the terminal ACK is observable from outside
    AUTO. This is therefore a BACKSTOP now, for a dead link or a wedged board, not
    the primary path.

    Kept, and kept generous, for exactly that reason: the failure it still covers is
    "no ACKs at all", where a too-tight deadline would report a bogus stall on a
    slow-but-alive board. Budget from the expected leg time; p5 is only a floor.
    """
    move_type = int(p1)
    if move_type in (sp.MOVE_FORWARD, sp.MOVE_BACK, sp.MOVE_STRAFE_L,
                     sp.MOVE_STRAFE_R, sp.MOVE_ARC, sp.MOVE_HOLD):
        expected = abs(float(p2))                       # p2 is duration_s
    elif move_type == sp.MOVE_TURN:
        expected = abs(float(p2)) / sp.MOVE_YAW_RATE    # degrees at the default rate
    elif move_type == sp.MOVE_DIVE:
        expected = abs(float(p2)) / sp.MOVE_DEPTH_RATE  # metres at the ramp rate
    elif move_type == sp.MOVE_STYLE:
        expected = abs(float(p2)) * _STYLE_ROLL_S       # 360 deg at 90 deg/s
    else:                                               # MOVE_STOP and anything new
        expected = 0.0
    # x2 covers ramp + brake + a slow board; the floor keeps a tiny leg's deadline
    # sane, and p5 (when the caller set one) is honoured as a lower bound.
    return max(expected * 2.0 + _ACK_MARGIN_S, _ACK_MIN_BUDGET_S, float(p5 or 0.0) + _ACK_MARGIN_S)


def _decode_esc_status(buf):
    """(index, [rpm x4]) from a raw ESC_STATUS frame, or None if it fails a check.

    Layout from the firmware's own vendored definition: the payload is
    `<Q4i4f4fB` = time_usec, rpm[4] (int32, SIGNED), voltage[4], current[4],
    index. MAVLink v2 truncates trailing zero bytes, so the payload is padded
    back to full length before unpacking -- the same trap as SYS_STATUS's
    extended health, and here the trailing field is `index` itself.
    """
    try:
        if len(buf) < 12 or buf[0] != 0xFD:
            return None                            # not a v2 frame
        n = buf[1]
        payload = buf[10:10 + n]
        if len(payload) != n:
            return None
        # pymavlink hands over an UNKNOWN message WITHOUT checking the checksum,
        # so a corrupt frame would decode into plausible-looking RPM. Check it.
        crc = mavutil.x25crc(buf[1:10 + n])
        crc.accumulate_str(chr(_ESC_STATUS_CRC_EXTRA))
        want = struct.unpack_from('<H', buf, 10 + n)[0]
        if crc.crc != want:
            return None
        payload = payload.ljust(_ESC_STATUS_LEN, b'\x00')
        fields = struct.unpack('<Q4i4f4fB', payload)
        return int(fields[-1]), [int(v) for v in fields[1:5]]
    except Exception:                              # noqa: BLE001
        return None


def _param_id(pv) -> str:
    """PARAM_VALUE.param_id as a clean str (mavlink sends a padded bytes field)."""
    pid = getattr(pv, 'param_id', '')
    pid = pid.decode() if isinstance(pid, bytes) else str(pid)
    return pid.strip('\x00')


# Saturation is refused whatever its cause, because SATURATION ITSELF is the hazard:
# a pinned heave demand becomes full vertical thrust the instant the outputs go live
# (mixer throttle column is -1 on all four verticals, 0 on all four horizontals). But
# the cause changes what the operator should DO, and a large zero offset is far more
# common than a dead sensor -- so name the cheap fix first instead of sending someone
# to look for a hardware fault.
_DEPTH_ARM_TAIL = (
    'Arming would command FULL vertical thrust (the mixer throttle column is -1 on all '
    'four verticals) with the horizontals idle. If the barometer VARIANCE is healthy '
    'this is a large zero offset, not a dead sensor -- run `duburi calibrate_depth`, '
    'then re-check with `ros2 run duburi_manager connect`')


class SrotFC(FlightController):
    """SROT backend. Constructed with a pre-opened mavutil master + optional logger,
    exactly like `Pixhawk` (connection lives in the manager / connection_config)."""

    name = 'srot'

    def __init__(self, master, log=None):
        self.master = master
        self._log = log
        self._tx_lock = threading.Lock()
        self._boot_time = time.time()
        # Last HEARTBEAT from the VEHICLE (autopilot != INVALID). Our own GCS
        # heartbeat and any other GCS on the link (e.g. Bondor) send
        # MAV_AUTOPILOT_INVALID; the reader thread caches the latest HEARTBEAT
        # regardless of source, so mode/armed must prefer the vehicle's -- same
        # guard Pixhawk uses via _last_autopilot_hb.
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
        self._last_batt = None
        # Separate from allow_fw_behaviour_mismatch ON PURPOSE. That flag means "I
        # accept an unknown firmware revision"; this one means "I accept that the
        # depth loop is currently demanding full heave". They are different risks and
        # an operator who needs one must not be forced to silence the other.
        self.allow_saturated_depth_arm = False

    # ------------------------------------------------------------------ #
    #  Firmware behaviour revision -- the runtime interlock               #
    # ------------------------------------------------------------------ #
    def read_behaviour_rev(self, timeout: float = 2.0, retries: int = 3):
        """Board's `SROT_FW_BEHAVIOUR_REV`, or None if it never answered.

        Carried in `AUTOPILOT_VERSION.middleware_sw_version` (the board has no
        middleware, so the field was free), requested with MAV_CMD_REQUEST_MESSAGE.

        WHY THIS EXISTS AT RUNTIME. `test_firmware_behaviour_rev_is_new_enough`
        checks the same number, but it reads the firmware repo off disk and skips
        when that repo is not checked out beside the workspace -- which is exactly
        the situation on the vehicle. As a vehicle-side guarantee it is worth zero.
        The hazard it is supposed to guard is silent in both directions: pre-rev-2
        firmware COASTS on MOVE_STOP and we no longer carry a host brake, so a
        `stop` or an abort simply does not decelerate 20 kg of hull, with nothing
        in any log to say why. That has to be caught on the wire, before arming.

        A rev of 0 is NOT "unknown" -- it is what firmware older than 2026-08-01
        reports, because that build never populated the field. Fail closed on it.
        """
        if self._behaviour_rev is not None:
            return self._behaviour_rev
        for _ in range(max(1, retries)):
            self._command_long(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                               p1=float(sp.MSG_ID_AUTOPILOT_VERSION))
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                msg = self._cache('AUTOPILOT_VERSION')
                if msg is not None:
                    self._behaviour_rev = int(getattr(msg, 'middleware_sw_version', 0))
                    return self._behaviour_rev
                time.sleep(_POLL_S)
        return None

    def check_behaviour_rev(self):
        """(ok, reason). Fail-closed on a KNOWN-too-old board; loud but permissive
        when the board did not answer at all.

        The asymmetry is deliberate. "Board says rev 1" is a definite statement that
        `stop` will coast, and refusing is right. "Board said nothing" is far more
        likely a dropped frame or a firmware without REQUEST_MESSAGE than a genuine
        rev-1 board, and bricking a vehicle on a comms hiccup is its own hazard --
        so that case warns hard on every attempt and lets the operator proceed.
        """
        rev = self.read_behaviour_rev()
        need = sp.FW_BEHAVIOUR_REV_REQUIRED
        if rev is None:
            self._log_warn(
                f'[SROT ] !! could not read SROT_FW_BEHAVIOUR_REV from the board '
                f'(AUTOPILOT_VERSION unanswered). Host expects >= {need}. If this '
                f'firmware predates 2026-08-01, MOVE_STOP COASTS and there is no '
                f'host-side brake -- stop/abort will NOT decelerate. Proceeding.')
            return True, 'FW_BEHAVIOUR_REV_UNKNOWN'
        if rev >= need:
            if not self._behaviour_rev_logged:
                self._log_info(f'[SROT ] firmware behaviour rev {rev} (>= {need} required)')
                self._behaviour_rev_logged = True
            return True, f'FW_BEHAVIOUR_REV={rev}'
        detail = (f'board reports SROT_FW_BEHAVIOUR_REV={rev}, host requires >= {need}. '
                  f'Rev < 2 COASTS on MOVE_STOP and this host no longer carries the '
                  f'reverse-leg brake, so stop/abort would not decelerate the hull. '
                  f'Flash rev >= {need}, or set allow_fw_behaviour_mismatch:=true if '
                  f'you accept an un-braked stop.')
        if self.allow_fw_behaviour_mismatch:
            self._log_warn(f'[SROT ] !! OVERRIDDEN: {detail}')
            return True, f'FW_BEHAVIOUR_REV_OVERRIDDEN={rev}'
        self._log_warn(f'[SROT ] !! REFUSING TO ARM: {detail}')
        return False, f'FW_BEHAVIOUR_REV_TOO_OLD: {detail}'

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
            # An on-board primitive ramps its own demand and reports none, so
            # from here the host no longer knows what the thrusters are asked.
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

        WHY THIS IS NOT A ONE-LINE CACHE READ. SROT multiplexes MV_STATE / LEAK / WTEMP /
        GAIN / CURR / KILL and a dozen stack counters onto NAMED_VALUE_FLOAT, and pymavlink
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
        return value if (time.time() - stamp) <= max_age_s else None

    def _drain_named(self):
        """Fold the currently-cached NAMED_VALUE_FLOAT into our per-name table.

        Cheap and idempotent. It cannot recover names overwritten between calls -- only a
        reader-thread hook can (see `note_named_value`) -- but it means each name survives in
        our table for `max_age_s` instead of only until the next NAMED_VALUE_FLOAT of ANY
        name lands.

        EACH MESSAGE IS FOLDED ONCE, and that identity check is load-bearing: pymavlink
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
            self._named_cache[mname] = (float(msg.value), time.time())

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
        ambiguity the compid change exists to remove. `pixhawk.py` has always sent
        ONBOARD_CONTROLLER; this backend was the inconsistent one.

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
        age = time.time() - getattr(hb, '_timestamp', 0.0)
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
        ok, reason = self.check_behaviour_rev()
        if not ok:
            return False, reason
        ok, reason = self.check_depth_loop_settled()
        if not ok:
            return False, reason
        ok, reason = self.check_thruster_power()
        if not ok:
            return False, reason
        return self._arm_disarm(True, timeout, abort)

    def check_thruster_power(self):
        """(ok, reason). Refuse to arm when thruster power is KNOWN to be cut.

        The thruster pack is switched by a rotary knob on the SECOND board (an
        AS5600 magnetic encoder driving a MOSFET). Outside its two ON windows the
        propulsion battery is physically disconnected -- the ESCs are unpowered
        and the hull cannot move at all.

        Nothing refuses this today, on either side: the firmware's own `canArm()`
        checks IMU, leak and pack voltage and never looks at the kill state. So
        arming succeeds, every motion verb runs its full profile against dead
        thrusters, the payload fires, and the mission advances to completion on a
        motionless vehicle -- the silent-success shape that has already cost this
        project a round in SURFACE mode and a disarmed `SROT_MOVE`.

        ⛔ REFUSES ONLY ON KNOWN-ENGAGED, and that asymmetry is deliberate.
        `kill_switch is None` means the second board is not talking (unpowered,
        out of range, `ESPNOW_EN = 0`, or simply not built), which is a perfectly
        ordinary bench configuration. Refusing on unknown would make `arm()`
        unreachable on every vehicle without a second board -- the same trap as
        refusing a torpedo on UNKNOWN thruster health.
        """
        try:
            kill = self.telemetry().kill_switch
        except Exception as exc:                  # a probe fault is not a hull fault
            return True, f'thruster power unreadable ({exc}) -- allowing'
        if kill is True:
            return False, ('thruster power is CUT at the 2nd-board rotary switch -- '
                           'arming would run the whole mission against dead thrusters')
        return True, ''

    def check_depth_loop_settled(self):
        """(ok, reason). Refuse to arm while the depth controller is already saturated.

        OBSERVED ON THE VEHICLE, 2026-08-02, disarmed and stationary on a bench:
        `DEPTH_OUT = -1.00` (full scale) with `DEPTH_ERR = -3.1 m`, because the Bar30
        was reporting a phantom depth. Arming into that state is not a subtle risk --
        the mixer's throttle column is **-1 on all four vertical thrusters and 0 on all
        four horizontals** (fw `mixer.cpp`), so a -1.0 heave demand becomes +1.0 on
        every vertical the instant the outputs go live. That is exactly the reported
        blocker: verticals at ~3000 RPM, horizontals idling, nothing commanded.

        In water it is a vehicle that dives or surfaces the moment it arms.

        ⚠ THIS READS `DEPTH_CMD`, NOT `DEPTH_OUT`, AND THAT CHANGE IS LOAD-BEARING.
        It used to read `DEPTH_OUT` (the real controller's last output). From fw rev 8
        that field is SUPPRESSED while the controller is not running -- which is the
        honest-absence fix we asked for, and it made this guard STRUCTURALLY DEAD:
        `arm()` is the only caller, `arm()` only runs while disarmed, and disarmed the
        loop never runs, so `DEPTH_OUT` is now ALWAYS absent here and the old code took
        the `None` -> "not reported" -> PASS branch every single time. A guard that
        cannot fail is worse than no guard, because the preflight still prints a line
        that reads like a check.

        `DEPTH_CMD` is the replacement the firmware team pointed at, and it is a better
        signal for this specific question, not merely an available one:
          * It is `depth::preview(depth, 0.10)` -- computed ON DEMAND against a fixed
            0.10 m target, so it is live while disarmed by construction.
          * Same +/-1.0 clamp, so the 2026-08-02 phantom-baro case (depth -3..-6.7 m,
            `3.0 * -3.1` = -9.3) still pins at -1.00 exactly as `DEPTH_OUT` did.
          * PROPORTIONAL-ONLY, no integrator -- it reflects the CURRENT baro sample
            rather than accumulated windup. For "is the barometer lying right now",
            which is what this guard actually asks, that is the property we want.

        The threshold is expressed in METRES OF DEPTH ERROR and converted through
        `DEPTH_P`, because `DEPTH_CMD = clamp(DEPTH_P * (depth - 0.10))` -- a hardcoded
        0.90 silently means a different physical depth the moment anyone retunes the
        gain. At the default `DEPTH_P = 3.0`, `|DEPTH_CMD| >= 0.90` <=> 0.30 m of error;
        a healthy surface reading (~0.03 m) gives ~-0.22 and is comfortably clear.

        It fails closed and quotes the numbers, because a refusal that does not say what
        it saw cannot be told apart from "not configured" -- the lesson the firmware team
        paid for four times over on the mag reference.

        Bypass with the ROS param `allow_saturated_depth_arm` (deliberately its own
        flag, not the firmware-revision one -- they are different risks).
        """
        cmd = self._named_value('DEPTH_CMD')
        if cmd is None:
            # DEPTH_CMD is gated on `depth_ok`, so absence means the board has declared
            # the barometer unhealthy/stale -- NOT that the check is unavailable. The
            # board then refuses DEPTH_HOLD/AUTO itself, and since SROT_MOVE enters AUTO
            # every move verb is denied, so this fails closed downstream without us
            # blocking the arm. Say which it is; do not report it as "settled".
            return True, 'depth preview absent (barometer unhealthy -- AUTO will refuse)'
        depth_p = self.depth_p or sp.DEPTH_P_DEFAULT
        # SATURATION FIRST. While clamped the true depth is beyond the clamp, so the
        # recovery below would report a harmless-looking value for the very case this
        # guard exists to catch -- the 2026-08-02 phantom baro pinned DEPTH_CMD at
        # -1.00, which back-converts to a benign-looking -0.23 m.
        if abs(cmd) >= sp.DEPTH_CMD_SATURATED:
            msg = (f'BAROMETER IMPLAUSIBLE: DEPTH_CMD={cmd:+.2f} is SATURATED, so the '
                   f'board is seeing a depth it cannot even express. '
                   + _DEPTH_ARM_TAIL)
        else:
            # Unsaturated: recover the board's own depth exactly. The preview's 0.10 m
            # target is baked into DEPTH_CMD, so subtracting it is what turns "error
            # against a target" into "how far the barometer is from zero" -- the thing
            # actually being judged.
            depth_m = cmd / depth_p + sp.DEPTH_PREVIEW_TARGET_M
            if abs(depth_m) < sp.DEPTH_ERR_ARM_LIMIT_M:
                return True, (f'barometer sane ({depth_m:+.2f} m at the surface, '
                              f'DEPTH_CMD={cmd:+.2f})')
            msg = (f'BAROMETER IMPLAUSIBLE: the board reads {depth_m:+.2f} m of depth '
                   f'at the surface (limit {sp.DEPTH_ERR_ARM_LIMIT_M:.2f} m, '
                   f'DEPTH_CMD={cmd:+.2f}, DEPTH_P={depth_p:g}). ' + _DEPTH_ARM_TAIL)
        if self.allow_saturated_depth_arm:
            self._log_warn(f'[SROT ] {msg} -- OVERRIDDEN, arming anyway')
            return True, f'OVERRIDDEN: {msg}'
        return False, msg

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
        self._command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            p1=float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            p2=float(target))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            hb = self._vehicle_hb()
            if hb is not None and int(hb.custom_mode) == target:
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

    def _warn_if_mode_discards_manual(self) -> None:
        """Warn (rate-limited) when the board is in a mode that throws this frame away."""
        mode = self.get_mode()
        if mode not in self._MANUAL_DISCARDING_MODES:
            self._last_mc_mode_warn = 0.0
            return
        now = time.time()
        if now - getattr(self, '_last_mc_mode_warn', 0.0) < self._MANUAL_WARN_PERIOD_S:
            return
        self._last_mc_mode_warn = now
        self._log_warn(
            f'[SROT ] !! MANUAL_CONTROL sent while the board is in {mode} -- the '
            f'firmware DISCARDS every axis of it and reports nothing (B28). '
            f'{"AUTO is where a completed SROT_MOVE leaves the board; " if mode == "AUTO" else ""}'
            f'reach STABILIZE first (vision_verbs._require_srot_vision_mode does).')

    def manual(self, fwd: float, lat: float, up: float, yaw: float) -> None:
        """One MANUAL_CONTROL frame. x=fwd, y=lat(+starboard), z=heave(+up), r=yaw;
        all axes -1..1 (the board clamps; we clamp in srot_protocol). Buttons=0.

        A non-finite axis is coerced to neutral (0) rather than raising: this is the
        streamed 20 Hz servo primitive, so a single NaN from a vision loop must not
        crash the send -- it degrades to 'hold' for that tick.

        THE MODE GUARD IS THE POINT, NOT DECORATION (B28). In `AUTO` and `SURFACE`
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
        self._demand = (x / sp.MC_AXIS_MAX, y / sp.MC_AXIS_MAX, time.monotonic())

    # A MANUAL_CONTROL frame older than this is not a demand in force: the
    # board's own pilot-input timeout is longer, but a writer that stopped
    # streaming has stopped commanding, and a stale demand would be modelled
    # as a live one.
    DEMAND_FRESH_S = 0.25

    def demand(self):
        """(fwd, lat) last sent in [-1, 1], or None when it is not known.

        None after any SROT_MOVE (the board owns the demand), before the first
        frame, and once the stream is older than DEMAND_FRESH_S. Consumed by the
        command-velocity model, which must predict nothing from an unknown.
        """
        d = getattr(self, '_demand', None)
        if d is None or time.monotonic() - d[2] > self.DEMAND_FRESH_S:
            return None
        return d[0], d[1]

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
        self._command_long(sp.CMD_SROT_MOVE, p1=float(sp.MOVE_STOP))

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

    def set_relay(self, instance_0based: int, on: bool) -> None:
        """DO_SET_RELAY (181): switch PCA9685 MOSFET `instance_0based` (0-based ->
        PCA ch PCA_RELAY_BASE_CH + instance) on/off. Raw firmware match.

        NOT used by the payload path: this is instance-addressed rather than
        channel-addressed, it shares its mapping with the joystick relay buttons,
        and its `PCA_RELAY_BASE_CH + param1` arithmetic is only bounds-checked on
        the RESULT, so a negative instance silently reaches channels 1-8 (fw
        `mav_commands.cpp:487-488`). `DO_SET_SERVO` addresses every channel by its
        own number whatever its role, which is the honest primitive. Kept because
        it is a truthful wrapper of a real command.
        """
        self._command_long(sp.CMD_DO_SET_RELAY, p1=float(int(instance_0based)),
                            p2=(1.0 if on else 0.0))

    def set_servo_acked(self, channel_1based: int, us: int, timeout: float):
        """`set_servo` + the board's COMMAND_ACK result, or None if it never answered.

        The payload path needs to know what the board DID with the command, and the
        only feedback the firmware offers is this ACK -- there is no actuator readback
        of any kind. Separate from `set_servo` because the plain form is a raw
        fire-and-forget wire wrapper and several callers want it that way.

        ⚠ ON and OFF are the SAME command id (183) and pymavlink keeps one message
        per msgid, so the ACK slot MUST be cleared immediately before each send or
        this reads the previous pulse's answer. That is the same single-slot hazard
        documented at length in `_named_value`.

        ⚠ Clearing the cache is NOT sufficient on its own, and this was MEASURED:
        probing the live board with back-to-back DO_SET_SERVO commands read the
        PREVIOUS command's DENIED as the next command's answer, because clearing
        pymavlink's cache does nothing about an ACK still sitting in the socket
        buffer -- the reader parses it in a moment later and it lands in the slot
        looking fresh. Filtering on the command id cannot separate them either:
        ON and OFF are both 183. So also require the ACK to be NEWER than our send.
        """
        self._clear_ack()
        t_send = time.time()          # wall clock: pymavlink stamps _timestamp with time.time()
        self.set_servo(channel_1based, us)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ack = self._cache('COMMAND_ACK')
            if (ack is not None and ack.command == sp.CMD_DO_SET_SERVO
                    and getattr(ack, '_timestamp', t_send) >= t_send):
                return int(ack.result)
            time.sleep(_POLL_S)
        return None

    # ------------------------------------------------------------------ #
    #  move() -- the duburi-verb -> SROT_MOVE mapping + ACK state machine #
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

        # No host-side brake before a 'stop' any more: fw rev 2 brakes on-board.
        self._clear_ack()
        self._command_long(sp.CMD_SROT_MOVE, p1=p1, p2=p2, p3=p3, p4=p4, p5=p5)
        self._last_leg = None
        return self._relay_move_ack(verb, on_progress, abort_fn,
                                    _ack_budget_s(verb, p1, p2, p5))

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

        ⛔ REFRESH ONLY ON *NEW* PROGRESS. `_cache()` returns the same COMMAND_ACK
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
                return MoveResult(ABORTED, f'{verb}: aborted (braked to a stop)')
            ack = self._cache('COMMAND_ACK')
            if ack is not None and ack.command == sp.CMD_SROT_MOVE:
                if ack.result in sp.TERMINAL_ACKS:
                    code = _ACK_TO_CODE[ack.result]
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
            f'`ros2 run duburi_manager connect` tells them apart.')
        return MoveResult(TIMEOUT, f'{verb}: no terminal ACK within {budget:.0f}s (stall) -- the board took the '
                f'command but never reported a terminal result. Check the link is '
                f'alive (`ros2 run duburi_manager connect`) and that the board is '
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

    # ------------------------------------------------------------------ #
    #  Telemetry                                                          #
    # ------------------------------------------------------------------ #
    def note_statustext(self, msg) -> None:
        """Hook for the manager's MAVLink reader thread, like `note_named_value`.

        The reader sees EVERY STATUSTEXT; this object only ever sees whichever
        one last landed in pymavlink's single slot. That matters more here than
        it looks, because the board sends its announcements as a BURST at boot
        (task_mavlink.cpp): the reset reason, a defaults reset, an NVS reformat,
        four migration notices, the calibration state and the config banner --
        13 messages back to back, of which a poller sees one.

        Two of those change how the vehicle behaves and are silent otherwise:
        "Params reset to build defaults" puts JS_GAIN_DEFAULT back to 0.5 and
        LEAK_EN back to 0, and "NVS reformatted" additionally loses the sensor
        calibration.

        Also parses the per-thruster presence lines, which are the only place
        `esc_present` reaches the wire at all -- the board holds the bitmask in
        state and publishes it exclusively as English.
        """
        text = getattr(msg, 'text', '')
        text = text.decode(errors='replace') if isinstance(text, bytes) else str(text)
        text = text.strip('\x00').strip()
        if not text:
            return
        sev = int(getattr(msg, 'severity', 6))
        self._statustext_log.append((time.time(), sev, text))
        self._note_esc_line(text)

    def _note_esc_line(self, text: str) -> None:
        """Per-thruster presence, from the board's own announcements.

        The board decides presence from bidirectional-DShot telemetry and holds
        it in `thrusters.esc_present`, but that bitmask NEVER reaches the wire
        as data -- only as these two sentences, once each:

            "Thrusters wired: all 8"            (INFO,  at the first arm)
            "Thrusters wired: 1,2,4 (absent: 3)"(WARN,  at the first arm)
            "Thruster 3 LOST telemetry"         (ERROR, on a regression only)

        The summary is sent ONCE per boot and the LOST lines are edge-triggered,
        so this has to be captured as it goes past. There is no way to ask again.
        """
        m = re.match(r'Thrusters wired: all (\d+)', text)
        if m:
            self._esc_present = set(range(1, int(m.group(1)) + 1))
            return
        m = re.match(r'Thrusters wired: ([\d,]*)', text)
        if m:
            wired = m.group(1)
            self._esc_present = {int(n) for n in wired.split(',') if n.strip().isdigit()}
            return
        m = re.match(r'Thruster (\d+) LOST telemetry', text)
        if m:
            self._esc_lost.add(int(m.group(1)))

    def statustext_log(self, since: float = 0.0):
        """(timestamp, severity, text) newest last, optionally since a time."""
        return [r for r in self._statustext_log if r[0] >= since]

    def boot_warnings(self):
        """The startup announcements that change how the vehicle behaves.

        Returned as (severity, text) so a caller can grade them. These are FAILs,
        not notices: a defaults reset silently reverts the pilot gain to half
        authority and disables the leak failsafe, and the operator's only clue is
        one line in a burst of thirteen.
        """
        keys = ('Params reset to build defaults',
                'NVS reformatted',
                'CAL DEFAULTS',
                'MAG cal MISSING',
                'SAVE FAILED',
                'set but NOT saved')
        return [(sev, txt) for _, sev, txt in self._statustext_log
                if any(k in txt for k in keys)]

    def esc_presence(self):
        """(present:set|None, lost:set) -- per-thruster telemetry presence.

        `present is None` means the board has not announced yet, which is NOT
        the same as "all healthy" and must never be graded as one. The summary
        is emitted at the FIRST ARM, so a disarmed bench session legitimately
        has nothing to report.
        """
        if self._esc_present is None:
            return None, set(self._esc_lost)
        return set(self._esc_present) - self._esc_lost, set(self._esc_lost)

    def _statustext(self) -> str:
        msg = self._cache('STATUSTEXT')
        if msg is None:
            return ''
        text = getattr(msg, 'text', '')
        return text.decode() if isinstance(text, bytes) else str(text)

    # Board uptime can wobble by a sample without meaning anything -- messages
    # are timestamped when built and can be reordered slightly by the tx queue.
    # A REBOOT drops uptime to near zero, so only a large backwards step counts.
    _REBOOT_DROP_MS = 3000

    def check_for_reboot(self) -> bool:
        """True exactly once per unplanned flight-controller restart.

        Opening the serial port reboots this board (see `fc/port_guard.py`), so an
        FC restart mid-session is a REAL and reachable state, not a theoretical
        one: any second process that touches the device causes it. After a reboot
        the board is DISARMED, back in its boot mode, with every setpoint cleared
        and its stream rates reset to the compiled defaults -- so a mission that
        keeps issuing verbs is commanding a vehicle that is no longer the one it
        configured.

        Detected on `ATTITUDE.time_boot_ms` going sharply backwards. That field is
        the board's own clock, so this needs no host timing and survives a stalled
        reader thread.
        """
        att = self._cache('ATTITUDE')
        if att is None:
            return False
        now_ms = int(getattr(att, 'time_boot_ms', 0))
        prev = self._peak_boot_ms
        if prev is None:
            self._peak_boot_ms = now_ms
            return False
        if now_ms < prev - self._REBOOT_DROP_MS:
            self._reboots += 1
            self._peak_boot_ms = now_ms
            if self._log is not None:
                self._log.error(
                    f'[SROT ] FLIGHT CONTROLLER REBOOTED -- uptime fell '
                    f'{prev} -> {now_ms} ms (restart #{self._reboots}). The board is '
                    f'now DISARMED with default stream rates; anything this session '
                    f'configured is gone. Most likely cause: a second process opened '
                    f'the serial port.')
            return True
        self._peak_boot_ms = max(prev, now_ms)
        return False

    def esc_status_rpm(self):
        """SIGNED per-thruster RPM from ESC_STATUS (291), or None.

        Both repos record that pymavlink "silently discards" msgid 291 because
        upstream removed it from `common`. It does not. `MAVLink.decode()` ends
        with:

            if mapkey not in mavlink_map:
                return MAVLink_unknown(msgId, msgbuf)

        -- a message object carrying the WHOLE FRAME, surfaced as UNKNOWN_291.
        Verified against two recorded BENCH sessions: 958 of 958 frames decode,
        CRC-valid, across both index blocks.

        ⛔ THAT IS DECODABILITY, NOT DATA. Every one of those 958 frames carries
        rpm EXACTLY 0 in all eight slots, because no ESC was attached -- the
        board fills the whole block regardless. Read as "the telemetry is real"
        this line would justify a health gate that reports OK on a hull with no
        thrusters. No non-zero RPM has yet crossed this wire on this vehicle.

        Worth the trouble because ESC_STATUS.rpm is `int32` and SIGNED, while
        ESC_TELEMETRY_*.rpm is `uint16` magnitude only -- the firmware says so
        itself ("a reversing thruster reports magnitude; the sign lives in the
        commanded direction, which the companion already knows"). We do know the
        commanded direction, but only for a thruster we commanded: it cannot
        tell a prop spun backwards by the wash from one driven backwards, and it
        says nothing at all when the vehicle is idle. Signed RPM can.

        ⚠ WE MUST CHECK THE CRC OURSELVES. That early return happens BEFORE
        pymavlink validates the checksum, so an unknown message is handed over
        unvalidated -- a corrupted frame would otherwise read as plausible RPM.
        `crc_extra` for ESC_STATUS is 10.

        Returns a tuple of 8 signed ints, or None if nothing valid has arrived.
        Absence, never zeros: a stopped thruster and a missing message are not
        the same fact.
        """
        out = [None] * 8
        got = False
        for key in ('UNKNOWN_291', 'ESC_STATUS'):
            msg = self._cache(key)
            if msg is None:
                continue
            try:
                buf = bytes(msg.get_msgbuf())
            except Exception:                      # noqa: BLE001
                continue
            block = _decode_esc_status(buf)
            if block is None:
                continue
            index, rpm = block
            for i, v in enumerate(rpm):
                if 0 <= index + i < 8:
                    out[index + i] = v
                    got = True
        return tuple(out) if got else None

    def motor_test(self, motor_1based: int, throttle_pct: float,
                   seconds: float = 2.0, *, abort_fn=None) -> tuple:
        """Spin ONE thruster, holding the board's keep-alive for `seconds`.

        `MAV_CMD_DO_MOTOR_TEST` (209) has been fully implemented on the board
        the whole time and had no driver method here, so the one procedure that
        can identify a thruster by making it turn was reachable only from
        Bondor.

        THE KEEP-ALIVE IS THE SAFETY MECHANISM, WHICH IS WHY THIS OWNS IT.
        The board expires the test when the resends stop, and expiry
        auto-disarms -- so a caller that spins a motor and then wedges must not
        be able to hold it spinning. This blocks for the duration and pumps from
        inside, and any exit path (a raise, an abort, the deadline) stops
        resending. The window is clamped to 600..3000 ms by the firmware
        whatever we ask, so silence for one clamped window is the longest a
        thruster can run past us.

        Returns (ok, reason). ARMED is required and refused messages come back
        as the board's own text.

        ⚠ PROPS OFF, OR THE VEHICLE RESTRAINED. This turns a thruster.
        """
        n = int(motor_1based)
        if not 1 <= n <= 8:
            return False, f'motor {n} out of range 1..8'
        pct = float(throttle_pct)
        if not math.isfinite(pct) or not -100.0 <= pct <= 100.0:
            return False, f'throttle {throttle_pct} must be finite and -100..100'
        if not self.is_armed():
            # The board says this too, but saying it here means the caller does
            # not have to spin a motor to find out.
            return False, ('motor test requires ARMED (the board refuses with '
                           '"Arm motors before testing motors.")')

        window_s = max(sp.MOTOR_TEST_WINDOW_MIN_MS,
                       min(sp.MOTOR_TEST_WINDOW_MAX_MS, 1000.0)) / 1000.0
        period = 1.0 / sp.MOTOR_TEST_KEEPALIVE_HZ
        deadline = time.monotonic() + max(0.0, float(seconds))
        sent = 0
        try:
            while time.monotonic() < deadline:
                if abort_fn is not None and abort_fn():
                    return False, f'motor test aborted after {sent} keep-alives'
                self._clear_ack()
                self._command_long(sp.CMD_DO_MOTOR_TEST,
                                   p1=float(n),
                                   p2=float(sp.MOTOR_TEST_THROTTLE_PERCENT),
                                   p3=pct,
                                   p4=window_s)
                sent += 1
                time.sleep(period)
                # Check for a refusal AFTER the sleep, not by blocking on one.
                # A refusal is a property of the request rather than of any
                # single keep-alive, so it only has to be noticed once -- and
                # blocking here would make the CADENCE depend on the link,
                # which is precisely what the keep-alive exists to survive. It
                # would also eat the caller's requested duration: waiting a
                # second for an ACK that never comes turned a 1 s test into one
                # keep-alive, which the board expires.
                ack = self._cache('COMMAND_ACK')
                if ack is not None and int(getattr(ack, 'command', 0)) == \
                        sp.CMD_DO_MOTOR_TEST:
                    res = int(ack.result)
                    if res != sp.ACK_ACCEPTED:
                        st = self._statustext()
                        return False, (f'motor test refused: '
                                       f'{sp.ACK_NAMES.get(res, res)}'
                                       + (f' ({st})' if st else ''))
        finally:
            # Stop resending, deliberately and on EVERY path. The board expires
            # the test within one clamped window and disarms, which is the
            # designed end state -- there is no "stop" command to send.
            pass
        return True, (f'motor {n} tested at {pct:.0f}% for {seconds:.1f}s '
                      f'({sent} keep-alives); the board expires and disarms '
                      f'within {sp.MOTOR_TEST_WINDOW_MAX_MS} ms of the last one')

    # ------------------------------------------------------------------ #
    #  MOTOR_DETECT -- GATE 0, behind an operator confirmation            #
    # ------------------------------------------------------------------ #
    def motor_detect_briefing(self, timeout: float = 2.0) -> str:
        """What this hull's direction configuration is, RIGHT NOW, read live.

        Three values decide what a detect run does, they are stored in three
        different places, and no display anywhere shows the product that
        actually reaches the mixer:

          CAL_MDIRn        what MOTOR_DETECT writes. It COMPOSES (fw rev 6:
                           `c' = c * agree`), so re-running is idempotent and a
                           wrong thruster converges in one pass -- but only
                           because it multiplies, which is the next line.
          MOT_n_DIRECTION  a separate parameter that MULTIPLIES with CAL_MDIRn.
                           On this hull M1 and M8 are -1, and they are set
                           runtime-only and unsaved. Flipping one for a motor
                           detect already reversed cancels the fix.
          FRAME_REVERSE    negates all six axis demands AFTER the mix. It is 1
                           here. A detect run that corrects the per-motor signs
                           does not know about it, so a successful detect leaves
                           this wrong and the whole vehicle inverted.

        Read live rather than assumed: a params-reset boot silently restores
        defaults, and the operator's only clue is one line in a burst of
        thirteen.
        """
        def _p(name):
            v = self.get_param(name, timeout=timeout)
            return None if v is None else int(round(v))

        cal = [_p(n) for n in sp.CAL_MDIR_PARAMS]
        mot = [_p(n) for n in sp.MOT_DIRECTION_PARAMS]
        fr = _p('FRAME_REVERSE')

        def _row(vals):
            return ' '.join('?' if v is None else ('-' if v < 0 else '+')
                            for v in vals)

        eff = []
        for c, m in zip(cal, mot):
            eff.append(None if (c is None or m is None) else c * m)
        # FRAME_REVERSE is applied to the AXIS demand, not per motor, so it is
        # reported beside the product rather than folded into it -- folding it
        # in would suggest a per-motor fix for a whole-vehicle inversion.
        lines = [
            'MOTOR DETECT -- this hull, read live:',
            f'  CAL_MDIR1..8     {_row(cal)}   (what detect writes)',
            f'  MOT_n_DIRECTION  {_row(mot)}   (multiplies with the above)',
            f'  effective mix    {_row(eff)}',
            f'  FRAME_REVERSE    {"?" if fr is None else fr}'
            + ('   <-- negates ALL six axes after the mix; a successful '
               'detect leaves this wrong' if fr else ''),
            '',
            'It pulses all eight thrusters at 30 % and then rewrites the signs',
            'the attitude controllers use. It needs WATER and a hull free to',
            'rotate: in air the gyro response is below the 0.05 rad/s gate, the',
            'run reports FAIL and writes nothing (fw rev 6 -- before that it',
            'reset every thruster to +1 and reported SUCCESS).',
            '',
            f'To run it: motor_detect({sp.MOTOR_DETECT_TOKEN!r})',
        ]
        return '\n'.join(lines)

    def motor_detect(self, confirm=None, *,
                     timeout: float = sp.MOTOR_DETECT_TIMEOUT_S,
                     abort_fn=None) -> tuple:
        """Run the board's MOTOR_DETECT (mode 20). Returns (ok, reason).

        CONFIRMATION IS THE POINT OF THIS METHOD. Called without the token it
        does not run -- it reads the three live values and returns them as the
        refusal text, so the only way to reach the token is to have been shown
        what it will change. A bool `force=True` would have made an accidental
        full-thruster run one keystroke away.

        WE DO NOT CHECK FOR WATER, AND SAY SO RATHER THAN PRETENDING TO. There
        is no wet sensor on this vehicle; the depth reading at the surface is
        the same whether the hull is in a pool or on a bench. The refusal text
        states the requirement instead of implying it was verified.

        The end state is MANUAL and DISARMED, and that is the DESIGNED one, not
        a fault (fw task_control_loop.cpp:383-406): detect has just rewritten
        the direction signs, and handing an armed hull straight to a closed-loop
        controller after an unverified sign change flipped the vehicle in water
        on 2026-08-07. The operator re-arms to verify.
        """
        if confirm != sp.MOTOR_DETECT_TOKEN:
            return False, ('motor detect NOT run -- confirmation required.\n\n'
                           + self.motor_detect_briefing())
        if not self.is_armed():
            # The board says this too ("Motor detect: arm first (needs water,
            # free to rotate)") and bounces back to STABILIZE, so without this
            # the caller watches a mode that silently reverted.
            return False, ('motor detect requires ARMED -- the board refuses '
                           'and falls back to STABILIZE')

        before = [self.get_param(n) for n in sp.CAL_MDIR_PARAMS]
        mark = time.time()
        ok, why = self.set_mode('MOTOR_DETECT')
        if not ok:
            return False, f'could not enter MOTOR_DETECT: {why}'

        deadline = time.monotonic() + max(1.0, float(timeout))
        while time.monotonic() < deadline:
            if abort_fn is not None and abort_fn():
                # Leaving the mode is what stops it driving motors
                # (task_control_loop.cpp:453). Disarm too, because a mode change
                # alone leaves thrusters live.
                self.set_mode('MANUAL')
                self.disarm()
                return False, 'motor detect aborted -- left the mode and disarmed'
            hb = self._vehicle_hb()
            mode = sp.mode_name(getattr(hb, 'custom_mode', -1))
            if mode == 'MANUAL' and not self.is_armed():
                break                       # the board's own completion signal
            time.sleep(_POLL_S)
        else:
            return False, (f'motor detect did not finish in {timeout:.0f}s '
                           f'(mode {sp.mode_name(getattr(self._vehicle_hb(), "custom_mode", -1))}, '
                           f'armed {self.is_armed()})'
                           + self._detect_texts(mark))

        after = [self.get_param(n) for n in sp.CAL_MDIR_PARAMS]
        changed = [i + 1 for i, (a, b) in enumerate(zip(before, after))
                   if a is not None and b is not None
                   and (a < 0) != (b < 0)]
        # A detect that changed NOTHING is ambiguous on purpose: it is either
        # idempotent success (already correct -- agree is +1 for every motor) or
        # a FAIL that correctly declined to write. The board's own text is what
        # separates them, so it is carried through rather than summarised away.
        return True, ('motor detect finished -- DISARMED in MANUAL (designed; '
                      're-arm to verify axes). CAL_MDIR changed on: '
                      + (', '.join(f'M{n}' for n in changed) if changed
                         else 'nothing -- already correct, or the run was '
                              'INCONCLUSIVE and declined to write')
                      + self._detect_texts(mark))

    def _detect_texts(self, since: float) -> str:
        """The board's own MotorDetect lines, verbatim.

        It prints DETECTED and EFFECTIVE on two lines because both together
        exceed the 50-char STATUSTEXT budget and a truncated direction list is
        actively misleading. Relaying them unedited is the point -- our summary
        above is derived, theirs is the source.
        """
        lines = [t for _, _, t in self.statustext_log(since=since)
                 if 'otorDetect' in t or 'otor detect' in t]
        return ('\n  ' + '\n  '.join(lines)) if lines else ''

    # ------------------------------------------------------------------ #
    #  AUTOTUNE (21) and MOTOR_TUNE (22)                                   #
    # ------------------------------------------------------------------ #

    def tune_progress(self):
        """Live tune telemetry: (percent, detail) -- or (None, '') when silent.

        The board publishes `STUNT_PRG` (reused for the autotune percentage,
        fw task_control_loop.cpp:847) and, during an autotune, the live
        limit-cycle measurement `AT_N` / `AT_AMP` / `AT_TU` / `AT_OKPCT`. They
        added those for a stated reason:

            "so a run can be watched from the GCS instead of only explained
             after it aborts"                      (fw control/autotune.h:34)

        Nothing on our side read any of it. The first version of `autotune()`
        polled mode and armed for up to 150 s in TOTAL SILENCE -- exactly the
        failure that comment exists to prevent, on a routine that is driving all
        eight thrusters at full authority. `AT_TU` and `AT_OKPCT` read 0 while
        collecting and settle when a phase converges, so "gathering" and "stuck"
        are distinguishable here and nowhere else.
        """
        pct = self._named_value('STUNT_PRG')
        bits = []
        for name, fmt in (('AT_N', '{:.0f} half-cycles'), ('AT_AMP', 'amp {:.3f}'),
                          ('AT_TU', 'Tu {:.2f}s'), ('AT_OKPCT', 'consensus {:.0f}%')):
            v = self._named_value(name)
            if v is not None:
                bits.append(fmt.format(v))
        return pct, ', '.join(bits)

    def _wait_for_tune_end(self, label, timeout, mark, abort_fn):
        """Poll until the board's own completion signal, or refuse.

        BOTH tunes finish DISARMED in STABILIZE -- which is NOT the end state
        MOTOR_DETECT uses (MANUAL, disarmed). Modelling these on detect and
        watching for MANUAL would wait out the full timeout on a run that
        succeeded, then report failure.
          MOTOR_TUNE  fw task_control_loop.cpp:830-833
          AUTOTUNE    fw task_control_loop.cpp:846-853
        """
        deadline = time.monotonic() + max(1.0, float(timeout))
        last_report = 0.0
        while time.monotonic() < deadline:
            # Say what the tune is doing instead of sitting silent for minutes.
            # Throttled: this loop polls at _POLL_S and the values move slowly.
            now_m = time.monotonic()
            if now_m - last_report >= 5.0:
                last_report = now_m
                pct, detail = self.tune_progress()
                if pct is not None or detail:
                    shown = '--' if pct is None else f'{pct:.0f}%'
                    self._log_info(f'[TUNE ] {label} {shown}'
                                   + (f' | {detail}' if detail else ''))
            if abort_fn is not None and abort_fn():
                # Leaving the mode is what stops the tuner -- the firmware calls
                # abort() on the falling edge (task_control_loop.cpp:532,540).
                # Disarm too: a mode change alone leaves thrusters live.
                self.set_mode('STABILIZE')
                self.disarm()
                return False, f'{label} aborted -- left the mode and disarmed'
            hb = self._vehicle_hb()
            mode = sp.mode_name(getattr(hb, 'custom_mode', -1))
            if mode == 'STABILIZE' and not self.is_armed():
                return True, ''
            time.sleep(_POLL_S)
        return False, (f'{label} did not finish in {timeout:.0f}s '
                       f'(mode {sp.mode_name(getattr(self._vehicle_hb(), "custom_mode", -1))}, '
                       f'armed {self.is_armed()})' + self._tune_texts(mark))

    def _tune_texts(self, since: float) -> str:
        """The board's own tune lines, verbatim. Ours is derived; theirs is source."""
        want = ('utotune', 'otor tune', 'MTune', 'Disarmed:')
        lines = [t for _, _, t in self.statustext_log(since=since)
                 if any(w in t for w in want)]
        return ('\n  ' + '\n  '.join(lines)) if lines else ''

    def autotune(self, confirm=None, *, timeout: float = sp.AUTOTUNE_TIMEOUT_S,
                 abort_fn=None) -> tuple:
        """Run the board's relay AUTOTUNE (mode 21). Returns (ok, reason).

        Astrom-Hagglund relay tuning of the rate loops, then the angle-P loops,
        then depth-hold -- and it WRITES AND PERSISTS EVERY ONE OF THOSE PIDs.
        The firmware's own warning is 'AUTOTUNE started - thrusters WILL drive'.

        ⛔ WE ENTER BY SET_MODE, DELIBERATELY, AND NOT BY THE `ATUNE` PARAM.
        There are three trigger paths -- SET_MODE 21, `MAV_CMD_USER_5`, and
        `PARAM_SET ATUNE >= 1` -- and the latter two set `autotune_active`
        with no reference to arming. It LATCHES: the firmware comment at
        task_control_loop.cpp:513-519 records that setting ATUNE while disarmed
        left it set, and 'the next arm started a full-authority relay tune in
        whatever mode the pilot happened to be in, with the GCS and OLED still
        showing MANUAL'. They mitigated it by reflecting the trigger into the
        mode; we avoid arming the latch at all.

        WE DO NOT CHECK FOR WATER, AND SAY SO RATHER THAN PRETENDING TO. There
        is no wet sensor. Run in air and the relay never establishes a limit
        cycle -- the hull cannot rotate freely -- so it consumes its phases and
        writes gains fitted to nothing.

        The end state is DISARMED in STABILIZE and that is DESIGNED, not a
        fault (fw task_control_loop.cpp:846-853).
        """
        if confirm != sp.AUTOTUNE_TOKEN:
            return False, ('autotune NOT run -- confirmation required.\n\n'
                           + self.autotune_briefing())
        if not self.is_armed():
            # The firmware gate is `(in.autotune || mode == AUTOTUNE) && in.armed`
            # (task_control_loop.cpp:508). Disarmed, the mode is accepted and the
            # tuner never starts -- a silent no-op.
            return False, ('autotune requires ARMED -- the board accepts the mode '
                           'and never starts the tuner while disarmed')

        before = {n: self.get_param(n) for n in sp.AUTOTUNE_PID_PARAMS}
        mark = time.time()
        ok, why = self.set_mode('AUTOTUNE')
        if not ok:
            return False, f'could not enter AUTOTUNE: {why}'

        ok, why = self._wait_for_tune_end('autotune', timeout, mark, abort_fn)
        if not ok:
            return False, why

        after = {n: self.get_param(n) for n in sp.AUTOTUNE_PID_PARAMS}
        moved = [n for n in sp.AUTOTUNE_PID_PARAMS
                 if before.get(n) is not None and after.get(n) is not None
                 and abs(after[n] - before[n]) > 1e-6]
        # A tune that moved NOTHING is not success. The depth phase is SKIPPED
        # without a healthy depth sensor (autotune.h:20-23), and a relay that
        # never established a limit cycle writes nothing either -- so an empty
        # list is reported as the ambiguity it is, with the board's own lines.
        return True, ('autotune finished -- DISARMED in STABILIZE (designed). '
                      + (f'PIDs changed: {", ".join(moved)}' if moved else
                         'NO PID CHANGED -- either the depth phase was skipped '
                         '(no healthy baro) or no phase established a limit '
                         'cycle. Read the board lines below before trusting it.')
                      + self._tune_texts(mark))

    def autotune_briefing(self, timeout: float = 2.0) -> str:
        """What an autotune run would overwrite on this hull, read LIVE."""
        rows = []
        for n in sp.AUTOTUNE_PID_PARAMS:
            v = self.get_param(n, timeout=timeout)
            rows.append(f'  {n:<12} {"--" if v is None else f"{v:.4f}"}')
        return ('AUTOTUNE will drive all thrusters at full authority for up to\n'
                f'{sp.AUTOTUNE_TIMEOUT_S:.0f}s and then OVERWRITE and PERSIST these:\n'
                + '\n'.join(rows)
                + '\n\nRequires: ARMED, IN WATER, free to rotate. There is no wet\n'
                  'sensor on this vehicle -- nothing verified that for you.\n'
                  'Without a healthy barometer the depth phase is SKIPPED.\n'
                  f'\nTo proceed pass confirm={sp.AUTOTUNE_TOKEN!r}')

    def motor_tune(self, confirm=None, *, timeout: float = sp.MOTOR_TUNE_TIMEOUT_S,
                   abort_fn=None) -> tuple:
        """Run the board's MOTOR_TUNE (mode 22). Returns (ok, reason).

        Per-thruster RPM-controller tuning: ramp to find idle, hold levels to fit
        the feedforward slope FF_A, then relay-tune the PI on the throttle->RPM
        plant. Averaged across motors, written to RPM_KP/KI/FF_A/IDLE +
        MOT_SPIN_MIN, persisted, and pushed to the Pico.

        ⛔ TWO PRECONDITIONS THAT EACH FAIL SILENTLY.

        1. `MTUNE_EN` must be > 0.5 and it DEFAULTS TO ZERO (fw config.h:588
           `DEF_MTUNE_EN = 0.0f`). The gate is
           `(mode == MOTOR_TUNE) && (mtune_en > 0.5f) && armed`
           (task_control_loop.cpp:509) -- so SET_MODE alone enters the mode and
           the tuner never runs. This method reads the param and REFUSES rather
           than sitting in a mode that does nothing. It does not set it for you:
           enabling a motor-spinning mode is an operator decision.

        2. It fits on PER-MOTOR RPM TELEMETRY, which on this vehicle has never
           been non-zero -- 958 recorded ESC_STATUS frames, every rpm exactly 0,
           because telemetry needs Bluejay-flashed ESCs with bidirectional
           DShot. With no RPM the plant fit has no plant. We WARN rather than
           refuse on UNKNOWN (the same asymmetry as the pre-fire gate: `None`
           is this hull's permanent state and refusing on it would make the
           method unreachable forever), and REFUSE on a known-bad thruster.

        The end state is DISARMED in STABILIZE and that is DESIGNED
        (fw task_control_loop.cpp:830-833).
        """
        if confirm != sp.MOTOR_TUNE_TOKEN:
            return False, ('motor tune NOT run -- confirmation required.\n\n'
                           + self.motor_tune_briefing())

        en = self.get_param(sp.MOTOR_TUNE_ENABLE_PARAM)
        if en is None:
            return False, (f'could not read {sp.MOTOR_TUNE_ENABLE_PARAM} -- refusing '
                           'rather than entering a mode that may silently do nothing')
        if en <= 0.5:
            return False, (f'{sp.MOTOR_TUNE_ENABLE_PARAM} = {en:.1f}: MOTOR_TUNE is '
                           'DISABLED on this board, and the mode would be accepted '
                           'while the tuner never started. Set it to 1 deliberately '
                           '-- it is the interlock on a mode that spins motors.')
        if not self.is_armed():
            return False, ('motor tune requires ARMED -- the board accepts the mode '
                           'and never starts the tuner while disarmed')

        health, why = self.thruster_health()
        if health is False:
            return False, f'motor tune refused -- {why}'
        warn = ''
        if health is None:
            warn = ('\n  ⚠ thruster telemetry UNKNOWN -- this tune FITS on per-motor '
                    'RPM. If the ESCs are not Bluejay-flashed it will spin motors '
                    'and fit nothing.')

        before = {n: self.get_param(n) for n in sp.MOTOR_TUNE_PARAMS}
        mark = time.time()
        ok, why = self.set_mode('MOTOR_TUNE')
        if not ok:
            return False, f'could not enter MOTOR_TUNE: {why}'

        ok, why = self._wait_for_tune_end('motor tune', timeout, mark, abort_fn)
        if not ok:
            return False, why + warn

        after = {n: self.get_param(n) for n in sp.MOTOR_TUNE_PARAMS}
        moved = [n for n in sp.MOTOR_TUNE_PARAMS
                 if before.get(n) is not None and after.get(n) is not None
                 and abs(after[n] - before[n]) > 1e-6]
        return True, ('motor tune finished -- DISARMED in STABILIZE (designed). '
                      + (f'changed: {", ".join(moved)}' if moved else
                         'NOTHING CHANGED -- with zero RPM telemetry that is the '
                         'expected outcome, not a success')
                      + warn + self._tune_texts(mark))

    def motor_tune_briefing(self, timeout: float = 2.0) -> str:
        """The two silent preconditions and the live RPM, quoted back."""
        en = self.get_param(sp.MOTOR_TUNE_ENABLE_PARAM, timeout=timeout)
        rpm = self.esc_status_rpm()
        health, hwhy = self.thruster_health()
        rows = []
        for n in sp.MOTOR_TUNE_PARAMS:
            v = self.get_param(n, timeout=timeout)
            rows.append(f'  {n:<14} {"--" if v is None else f"{v:.4f}"}')
        return ('MOTOR_TUNE spins ONE THRUSTER AT A TIME through a throttle ramp\n'
                'and then OVERWRITES and PERSISTS these:\n'
                + '\n'.join(rows)
                + f'\n\n  {sp.MOTOR_TUNE_ENABLE_PARAM:<14} '
                + ('--  (unreadable)' if en is None else
                   f'{en:.1f}  ' + ('OK' if en > 0.5 else
                                    '⛔ DISABLED -- the mode would do NOTHING'))
                + '\n  live signed RPM  '
                + ('-- (no ESC telemetry at all)' if rpm is None else str(list(rpm)))
                + f'\n  thruster health  {hwhy}'
                + ('\n  ⚠ this tune FITS ON RPM. All-zero means it has no plant.'
                   if health is not True else '')
                + '\n\nRequires: ARMED, IN WATER, props on. There is no wet sensor\n'
                  'on this vehicle -- nothing verified that for you.\n'
                  f'\nTo proceed pass confirm={sp.MOTOR_TUNE_TOKEN!r}')

    def thruster_health(self):
        """(ok, reason) -- is every thruster reporting and turning as commanded?

        `ok is None` means UNKNOWN, and that is the important return value. The
        board announces presence only at the FIRST ARM and only as English, and
        ESC telemetry exists only on ESCs flashed with Bluejay -- stock BLHeli_S
        has no bidirectional DShot at all, so an un-flashed ESC is silent while
        the motor spins perfectly. A gate that answered "healthy" for a vehicle
        nothing has reported on would be worse than no gate, because it reads as
        a check that passed.

        Callers must treat None as "do not know", not as a refusal and not as an
        approval. `duburi_ws` uses it to refuse a PAYLOAD run (where a dead
        thruster means a missed shot and a wasted round) while allowing a
        transit, which is the asymmetry that matters.
        """
        present, lost = self.esc_presence()
        if lost:
            return False, ('thruster(s) '
                           + ', '.join(str(n) for n in sorted(lost))
                           + ' LOST telemetry mid-session')
        if present is None:
            return None, ('thruster telemetry UNKNOWN -- the board announces '
                          'presence at the first arm only, and an ESC without '
                          'Bluejay never reports at all')
        missing = sorted(set(range(1, 9)) - present)
        if missing:
            return False, ('thruster(s) ' + ', '.join(str(n) for n in missing)
                           + ' report no telemetry (unflashed ESC, or unwired)')
        return True, f'all {len(present)} thrusters reporting'

    def thruster_stalled(self, commanded=None, min_rpm: int = 50):
        """Thrusters commanded to turn that are not turning, from SIGNED RPM.

        `commanded` maps 1-based thruster -> commanded sign (-1/0/+1). Without
        it this only reports zero-RPM thrusters, which on an idle hull is every
        thruster -- so it returns None rather than a list nobody can act on.

        The signed reading is what makes this worth having: a thruster turning
        the WRONG WAY is a wiring or direction-calibration fault that unsigned
        magnitude cannot see, and it is exactly the fault that made every axis
        respond backwards on this hull in August.
        """
        rpm = self.esc_status_rpm()
        if rpm is None or commanded is None:
            return None
        bad = []
        for n, want in commanded.items():
            if not want:
                continue
            v = rpm[n - 1] if 1 <= n <= 8 else None
            if v is None:
                continue
            if abs(v) < min_rpm:
                bad.append((n, v, 'not turning'))
            elif (v > 0) != (want > 0):
                bad.append((n, v, 'turning the WRONG WAY'))
        return bad

    def _sys_status_ext_bits(self):
        """`(present, enabled, health)` from SYS_STATUS's extended bitfields, or None.

        THE PADDING IS LOAD-BEARING and this is the only copy of it. MAVLink v2
        truncates trailing zero bytes, so the payload is not its declared 43:
        measured off the live board it arrives at 40, with `health_extended` cut
        to a SINGLE byte. Unpacking four bytes at offset 39 without padding reads
        past the end. A truncated field is zero by definition, which is what the
        `ljust` restores.

        Extracted so `sys_status_leak()` and `leak_state()` cannot drift apart --
        a second hand-written copy of an offset table is how the ESC_STATUS(291)
        trap happened.
        """
        msg = self._cache('SYS_STATUS')
        if msg is None:
            return None
        try:
            buf = bytes(msg.get_msgbuf())
            payload = buf[10:10 + buf[1]]          # v2 header is 10 bytes
            if len(payload) <= _SYS_STATUS_BASE_LEN:
                return None                        # no extension bytes at all
            payload = payload.ljust(_SYS_STATUS_EXT_END, b'\x00')
            return struct.unpack_from('<III', payload, _SYS_STATUS_BASE_LEN)
        except Exception:                          # noqa: BLE001
            return None

    def sys_status_leak(self):
        """Leak from the SYS_STATUS extended health bits, or None if unreadable.

        We ASKED for this (TASKS_FROM_DUBURI_WS.md §3): LEAK rides the multiplexed
        NAMED_VALUE_FLOAT, pymavlink keeps one message per msgid, so observing a
        flooding hull was a lottery on arrival order. The firmware delivered it in
        rev 3 as `MAV_SYS_STATUS_SENSOR_LEAK` on the extended health bitfield --
        a LATCHED state that a temperature reading cannot overwrite -- and we
        never adopted it, because pymavlink 2.4.49's SYS_STATUS schema has 13
        fields and no extensions (verified: it does not parse them).

        It parses them away; it does not throw them away. The bytes are in the
        payload, at the offsets the firmware's own accessors use:

            31  onboard_control_sensors_present_extended
            35  onboard_control_sensors_enabled_extended
            39  onboard_control_sensors_health_extended

        THE PADDING IS LOAD-BEARING. MAVLink v2 truncates trailing zero bytes, so
        the payload is not its declared 43: measured off the live board it arrives
        at 40, with `health_extended` cut to a SINGLE byte. Unpacking four bytes at
        39 without padding reads past the end. A truncated field is zero by
        definition, which is what the pad restores.

        Returns None when the sensor is not present or not enabled -- absence, not
        False. `LEAK_EN = 0` disables the board's own leak failsafe AND its pre-arm
        refusal, and reporting "no leak" for a vehicle that is not looking is the
        exact confusion this whole mechanism exists to remove.
        """
        bits = self._sys_status_ext_bits()
        if bits is None:
            return None
        present, enabled, health = bits
        if not (present & sp.SYS_STATUS_SENSOR_LEAK):
            return None                            # board is not reporting a leak sensor
        if not (enabled & sp.SYS_STATUS_SENSOR_LEAK):
            return None                            # LEAK_EN = 0: nothing is watching
        # Health bit SET means healthy, i.e. dry. Clear means leak.
        return not bool(health & sp.SYS_STATUS_SENSOR_LEAK)

    def leak_state(self):
        """`(enabled, leaking)` -- the two leak facts, kept APART.

        `sys_status_leak()` above answers one question ("are we taking on water?")
        and correctly returns None for every reason it cannot: no SYS_STATUS, no
        sensor, or `LEAK_EN = 0`. That is right for its callers and useless for a
        health verdict, because it collapses "dry" and "nothing is watching" into
        the same absence -- which is the exact confusion its own docstring says
        the mechanism exists to remove.

        `health_reporters.leak_sensor()` was written to render that distinction
        (`LEAK_EN=0 -> FAILED, NOTHING IS WATCHING`) and needs the two signals
        separately. It could not be wired to anything until this existed, which
        is why it sat unregistered.

        Returns:
            (None, None)   -- no SYS_STATUS, or no leak sensor on the board
            (False, None)  -- sensor present but LEAK_EN = 0: nothing is watching
            (True,  bool)  -- enabled; True = leaking, False = dry
        """
        bits = self._sys_status_ext_bits()
        if bits is None:
            return (None, None)
        present, enabled, health = bits
        if not (present & sp.SYS_STATUS_SENSOR_LEAK):
            return (None, None)                    # board reports no leak sensor
        if not (enabled & sp.SYS_STATUS_SENSOR_LEAK):
            return (False, None)                   # LEAK_EN = 0 -- the trap
        return (True, not bool(health & sp.SYS_STATUS_SENSOR_LEAK))

    def telemetry(self) -> Telemetry:
        t = Telemetry()
        hb = self._vehicle_hb()
        if hb is not None:
            t.armed = bool(hb.base_mode & _ARMED_FLAG)
            t.mode = sp.mode_name(hb.custom_mode)
            t.link_alive = (time.time() - getattr(hb, '_timestamp', 0.0)) <= _LINK_STALE_S
        att = self._cache('ATTITUDE')
        if att is not None:
            t.yaw_deg = math.degrees(att.yaw) % 360.0
            t.roll_deg = math.degrees(att.roll)
            t.pitch_deg = math.degrees(att.pitch)
        # Depth: NEGATIVE below the surface, same as Pixhawk/AHRS2 and as
        # DuburiState.msg documents. VFR_HUD.alt is already in that convention
        # (fw sends alt = -depth), so pass it through -- see get_attitude().
        # Stays NaN until VFR_HUD arrives (5 Hz).
        #
        # ⚠ GATED ON BARO HEALTH, and that gate is not optional. Unlike
        # SCALED_PRESSURE2 and WTEMP -- which the firmware SUPPRESSES when the
        # barometer is unhealthy or stale -- VFR_HUD is sent unconditionally
        # (fw mav_stream.cpp:258). So a board with NO Bar30 fitted at all still
        # streams `alt = -0.000` forever, and passing that through publishes a
        # confident 0.00 m on /duburi/state, in [STATE], and into every depth guard
        # that compares against a negative constant. Read the health bit instead;
        # the board already refuses DEPTH_HOLD/AUTO on the same condition
        # (fw task_control_loop.cpp:107 and :348), so this stays consistent with
        # what the vehicle will actually let you do.
        vhud = self._cache('VFR_HUD')
        if vhud is not None and self._baro_healthy() is not False:
            t.depth_m = float(vhud.alt)
        # Read through the per-instance table, NOT `_cache('BATTERY_STATUS')`. The raw
        # slot holds whichever instance landed last, so this line used to claim
        # "id 0 = electronics pack" while actually reporting the THRUSTER pack about
        # half the time -- caught by a live smoke test showing battery_voltage and
        # thruster_voltage identical at 14.63 V when PM1 reads ~1.35 V.
        batteries = self.get_batteries()
        main = batteries.get(sp.BATTERY_ID_MAIN)
        if main is not None:
            t.battery_voltage = main['voltage']
        # Per-thruster RPM (Bluejay bidirectional DShot).
        #
        # ⚠ THIS IS ALWAYS EMPTY on pymavlink 2.4.49: upstream MAVLink removed the
        # WIP messages 290/291 from `common`, so ESC_STATUS (291) is in NO dialect we
        # ship (checked common / ardupilotmega / all / development). pymavlink drops
        # any msgid missing from its CRC-extra table SILENTLY -- no error, no callback
        # -- so the board can be reporting RPM perfectly while we read nothing. Bondor
        # lost every RPM packet to exactly this and it looked like an ESC fault.
        #
        # RESOLVED as of fw behaviour rev 2, exactly as this comment proposed: the board
        # now ALSO emits ESC_TELEMETRY_1_TO_4 (11030) + ESC_TELEMETRY_5_TO_8 (11031),
        # which ARE in the ardupilotmega dialect and carry 4 ESCs each -- an exact fit
        # for our 8 thrusters. `/duburi/esc_rpm` is built on that fallback below (see
        # `_publish_srot_telemetry`), NOT on the ESC_STATUS branch, which stays only so
        # the read is free if pymavlink ever regains 291.
        esc = self._cache('ESC_STATUS')
        if esc is not None:
            t.rpm = tuple(int(r) for r in getattr(esc, 'rpm', ()) or ())
        elif t.rpm == ():
            for name in ('ESC_TELEMETRY_1_TO_4', 'ESC_TELEMETRY_5_TO_8'):
                block = self._cache(name)
                if block is not None:
                    t.rpm = t.rpm + tuple(int(r) for r in getattr(block, 'rpm', ()) or ())
        # The latched health bit FIRST, the multiplexed scalar only as a fallback.
        # NAMED_VALUE_FLOAT('LEAK') is deprecated as of rev 3 and is observed
        # roughly one call in fifteen; the bit is deterministic.
        leak_bit = self.sys_status_leak()
        if leak_bit is not None:
            t.leak = leak_bit
        else:
            leak = self._named_value('LEAK')
            if leak is not None:
                t.leak = leak >= 0.5
        wtemp = self._named_value('WTEMP')
        if wtemp is not None:
            t.water_temp_c = wtemp
        # NOT nested under WTEMP. The board SUPPRESSES WTEMP when the barometer is
        # unhealthy, so gating ESC temperatures on it would make thruster temps vanish
        # in exactly the situation where you most want them -- a sick vehicle.
        for name in ('ESC_TELEMETRY_1_TO_4', 'ESC_TELEMETRY_5_TO_8'):
            block = self._cache(name)
            if block is not None:
                t.esc_temp_c = t.esc_temp_c + tuple(
                    int(x) for x in getattr(block, 'temperature', ()) or ())

        # Everything below is SROT-only and has no Pixhawk equivalent. Each stays NaN
        # (not 0.0) when the board has not said it -- the board SUPPRESSES values it
        # cannot stand behind, so absence is a distinct, meaningful state.
        thr = self.get_batteries().get(sp.BATTERY_ID_THRUSTER)
        if thr is not None:
            t.thruster_voltage = thr['voltage']
        for attr, name in (('depth_err_m', 'DEPTH_ERR'), ('depth_out', 'DEPTH_OUT'),
                           ('mag_accuracy', 'MAGACC')):
            val = self._named_value(name)
            if val is not None:
                setattr(t, attr, val)
        # BARO_HEALTH is truncated to 'BARO_HEALT' on the wire: NAMED_VALUE_FLOAT's
        # name field is 10 chars and the board does not shorten it itself. Reading the
        # untruncated name finds nothing, silently -- which is exactly how this value
        # was once read as a health SCORE rather than a fault CODE (3 = not initialised,
        # not "very healthy").
        baro_h = self._named_value(sp.NAME_BARO_HEALTH)
        if baro_h is not None:
            t.baro_health = baro_h
        # Tri-state: absent stays None, so "the board never said" is not "never seen".
        comp = self._named_value('COMP_SEEN')
        if comp is not None:
            t.companion_seen = comp >= 0.5
        # ⛔ KILL = 0 IS NOT "CLEAR". The kill switch is a rotary knob on the
        # SECOND board and its state crosses to this one over ESP-NOW; on link
        # loss the firmware reports kill=false on purpose --
        # `kill = f ? s_kill : false;  // link lost -> don't assert kill
        # (display-only)` (espnow_link.cpp:49). But it is NOT display-only: it
        # is packed into NAMED_VALUE_FLOAT "KILL" and read here.
        #
        # So a bare `KILL >= 0.5` reads "power is live" for a vehicle whose
        # second board is unpowered, out of range, or simply not built -- and
        # the operator's status line says `KILL clear` while nothing on the
        # vehicle can see the switch at all.
        #
        # BATTERY_STATUS instance 1 is the disambiguator and costs nothing: the
        # board SUPPRESSES it rather than zeroing it when the ESP-NOW link is
        # stale (`if (s.pm2_present) sendBattery(1, ...)`, and pm2_present is
        # gated on ESPNOW_STALE_MS). Its presence therefore IS the link
        # liveness, already demuxed by id in `get_batteries()`.
        if thr is None:
            t.kill_switch = None            # no 2nd-board link -> nobody knows
        else:
            kill = self._named_value('KILL')
            t.kill_switch = None if kill is None else (kill >= 0.5)
        return t

    # ------------------------------------------------------------------ #
    #  Pixhawk-compatible surface                                         #
    # ------------------------------------------------------------------ #
    # auv_manager_node reads the vehicle through the historical Pixhawk method
    # names (get_attitude/get_mode/is_armed/...). Exposing the same names here
    # -- reading SROT telemetry, no-op'ing the ArduSub-only writes -- lets the
    # manager treat `self.fc` as a drop-in on the srot backend with only a few
    # guarded changes (skip the neutral-RC heartbeat + the BNO->EKF mocap tick).

    def get_attitude(self):
        """{'yaw'(deg 0..360),'roll'(deg),'pitch'(deg),'depth'(m)} or None.

        Same shape as Pixhawk.get_attitude so the manager's telemetry/state path
        is unchanged. yaw in degrees (ATTITUDE is radians).

        DEPTH SIGN: this stack's convention -- DuburiState.msg, set_depth's input,
        STYLE_ROLL_SURFACE_GUARD_M, motion_vision's _MIN_DEPTH_M / max_depth_m /
        depth_ceiling_m -- is NEGATIVE below the surface, matching Pixhawk's
        AHRS2.altitude. The board is positive-DOWN internally but already negates
        on the wire (`VFR_HUD.alt = -depth`, fw mav_stream.cpp:210-218), so alt is
        ALREADY in our convention. Pass it straight through: negating here too
        made /duburi/state.depth_m positive when submerged, which silently
        inverted every one of those guards (they all compare against a negative
        constant, so each just stopped firing)."""
        att = self._cache('ATTITUDE')
        if att is None:
            return None
        # Same baro-health gate as telemetry() -- see the long note there. VFR_HUD is
        # NOT suppressed when the barometer is unhealthy, so an ungated read publishes a
        # confident 0.00 m off a board with no Bar30 fitted, and every depth guard here
        # compares against a negative constant (they would simply stop firing).
        depth = math.nan
        vhud = self._cache('VFR_HUD')
        if vhud is not None and self._baro_healthy() is not False:
            depth = float(vhud.alt)
        # The SAME gate as depth above, on the other sensor. NaN, not a dropped
        # dict: depth is independent of the IMU and is frequently healthy while
        # the BNO is not, so blanking the whole reading would trade one silent
        # failure for another. `DuburiState.msg` documents NaN as the absence
        # sentinel for exactly this.
        if self._ahrs_healthy() is False:
            return {'yaw': math.nan, 'roll': math.nan,
                    'pitch': math.nan, 'depth': depth}
        return {
            'yaw':   math.degrees(att.yaw) % 360.0,
            'roll':  math.degrees(att.roll),
            'pitch': math.degrees(att.pitch),
            'depth': depth,
        }

    def get_attitude_age(self):
        att = self._cache('ATTITUDE')
        return None if att is None else (time.time() - getattr(att, '_timestamp', 0.0))

    def get_mode(self):
        hb = self._vehicle_hb()
        return sp.mode_name(hb.custom_mode) if hb is not None else 'UNKNOWN'

    def note_battery(self, msg):
        """Hook for the manager's MAVLink reader thread -- de-multiplex by instance id.

        MEASURED ON THE VEHICLE (2026-08-02): the board streams BATTERY_STATUS id 0
        (PM1, electronics) and id 1 (PM2, thruster pack) at 2 Hz EACH. pymavlink keys
        its cache by msgid, so `master.messages['BATTERY_STATUS']` alternates between
        them -- a live sample showed the slot flipping between **1.35 V and 14.74 V**.
        Reading that slot is therefore a coin flip on WHICH BATTERY you are reporting,
        and the two differ by an order of magnitude, so it cannot even be averaged away.

        Same failure as NAMED_VALUE_FLOAT, one layer down, and the same fix: key by the
        field that distinguishes them and feed it from the only loop that sees them all.
        """
        bid = int(getattr(msg, 'id', 0))
        volts = list(getattr(msg, 'voltages', []) or [])
        raw_mv = volts[0] if volts else 0xFFFF
        voltage = math.nan if raw_mv in (0, 0xFFFF) else raw_mv / 1000.0
        cur = getattr(msg, 'current_battery', -1)
        current = math.nan if cur == -1 else cur / 100.0
        self._battery_cache[bid] = (voltage, current, time.time())

    def _baro_healthy(self):
        """True / False / None (never reported) from SYS_STATUS's health bitfield.

        Tri-state on purpose: "the board says the barometer is bad" and "we have not
        heard a SYS_STATUS yet" are different, and only the first should suppress a
        depth reading. Treating the second as unhealthy would blank depth for the first
        half-second of every connection.
        """
        msg = self._cache('SYS_STATUS')
        if msg is None:
            return None
        bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
        return bool(int(getattr(msg, 'onboard_control_sensors_health', 0)) & bit)

    def _ahrs_healthy(self):
        """True / False / None (never reported) for the BNO085 behind ATTITUDE.

        The exact tri-state shape as `_baro_healthy`, and for the same reason: a
        board that has not sent SYS_STATUS yet has not said the IMU is bad.

        ⛔ MEASURED ON THE VEHICLE 2026-09-11, which is why this exists. After a
        bad boot the board reported `present=0x80203c0b health=0x2408` -- 3D_GYRO
        (0x01) and 3D_ACCEL (0x02) present and NOT healthy -- with MAGACC 0.0,
        COMP_SEEN 0.0, YAW_REF 0.0, and it kept streaming ATTITUDE at 10 Hz with
        roll/pitch/yaw AND all three rates EXACTLY 0.0. `SCALED_IMU2` was zero in
        every field including accel, which the firmware packs as `s.lx + s.grx`
        (mav_stream.cpp:230) -- so at rest a live sensor owes ~1000 mG on one
        axis and zero is the sensor, not the encoding.

        A confident 0.0 is the worst possible failure here: `_effective_yaw_deg`
        would publish heading 0.0, `heading_lock` would close its 50 Hz Ch4 loop
        on it, and `turn` would believe it was already pointing north. Nothing
        anywhere logged a fault, because nothing asked.

        This is the barometer gate's twin. That half was fixed when a failing
        Bar30 saturated the depth PID; the attitude half was left open, in this
        same file, against the same bitfield.
        """
        msg = self._cache('SYS_STATUS')
        if msg is None:
            return None
        health = int(getattr(msg, 'onboard_control_sensors_health', 0))
        bits = (mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO
                | mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL)
        # BOTH must be healthy. The attitude solution is a fusion of the two, so
        # either one failing makes the quaternion untrustworthy.
        return (health & bits) == bits

    def _drain_battery(self):
        """Fold the currently-cached BATTERY_STATUS in, once per message object.

        The identity guard is load-bearing for the same reason as `_drain_named`'s:
        pymavlink never clears its slot, so re-folding would re-stamp a dead link's
        last reading as fresh forever.
        """
        msg = self._cache('BATTERY_STATUS')
        if msg is not None and msg is not self._last_batt:
            self._last_batt = msg
            self.note_battery(msg)

    def get_batteries(self, max_age_s: float = 5.0):
        """{id: {'voltage','current'}} for every battery seen recently.

        id 0 = PM1 (electronics rail), id 1 = PM2 (thruster pack, via ESP-NOW from the
        2nd board). An id absent here means it has not been heard -- NOT that it reads
        zero. On this vehicle PM1 reads ~1.3 V because nothing is wired to GPIO36.
        """
        self._drain_battery()
        now = time.time()
        return {bid: {'voltage': v, 'current': c}
                for bid, (v, c, stamp) in self._battery_cache.items()
                if (now - stamp) <= max_age_s}

    def get_battery(self):
        """{'voltage','current'} (V/A) for the MAIN battery (id 0), or None.

        Matches Pixhawk's dict contract (the manager reads battery['voltage']), so
        /duburi/state is unchanged. Pinned to id 0 rather than 'whatever arrived
        last' -- see note_battery for why that distinction is not cosmetic.
        """
        self._drain_battery()
        hit = self._battery_cache.get(sp.BATTERY_ID_MAIN)
        if hit is None:
            return None
        voltage, current, _ = hit
        return {'voltage': voltage, 'current': current}

    def get_rc_channels(self):
        return None            # SROT has no RC-channel readback (MANUAL_CONTROL only)

    def get_statustext(self):
        st = self._statustext()
        return st or None

    def get_angular_rates(self):
        """Body-frame angular rates (rad/s) from ATTITUDE, matching Pixhawk's contract:
        {'roll_rate','pitch_rate','yaw_rate','age_s'} or None. The manager's
        _imu_rates_tick reads the *_rate keys, so they MUST match exactly.

        Also returns `board_ms` -- ATTITUDE.time_boot_ms, THE BOARD'S OWN
        CAPTURE TIME.

        ⛔ WHY THAT FIELD MATTERS AND WHY IT WAS BEING WASTED. Measured on this
        vehicle at 50 Hz: the BOARD's interval is 20.00 ms with sd 0.00, and
        the HOST's arrival interval is 20.00 ms with sd 6.67 and p2p 35.12.
        The board's clock is exact; every bit of that jitter is transport
        (ESP32 UART FIFO thresholding plus USB-serial scheduling). Anything
        that stamps on arrival inherits all of it, and de-rotation subtracts
        `f*omega*dt`, so 5 ms costs 1.6 px at 0.64 rad/s.

        This field was already parsed and read by exactly one caller -- the
        unplanned-reboot detector -- and thrown away everywhere else. It is the
        jitter-free time base the flow pipeline needs; `flow_timing.ClockMap`
        maps it onto host time (the board does not implement MAVLink TIMESYNC:
        0 of 12 requests answered, measured).

        `board_ms` is None when the field is absent, never 0 -- absence is not
        the boot instant.
        """
        att = self._cache('ATTITUDE')
        if att is None:
            return None
        # ABSENCE, not zeros. A dead BNO streams exact 0.0 rates (measured), and
        # the flow node de-rotates by `f*omega*dt` -- so a zero rate is not a
        # harmless neutral, it is a claim that the hull did not turn.
        if self._ahrs_healthy() is False:
            return None
        age = time.time() - getattr(att, '_timestamp', 0.0) if getattr(att, '_timestamp', 0.0) else 0.0
        boot = getattr(att, 'time_boot_ms', None)
        return {'roll_rate': float(getattr(att, 'rollspeed', 0.0)),
                'pitch_rate': float(getattr(att, 'pitchspeed', 0.0)),
                'yaw_rate': float(getattr(att, 'yawspeed', 0.0)),
                'age_s': age,
                'board_ms': None if boot is None else int(boot),
                'host_recv_s': float(getattr(att, '_timestamp', 0.0)) or None}

    def get_imu(self, imu_msg=None, att_msg=None):
        """Full 6-DoF inertial sample from SCALED_IMU2, or None.

        `imu_msg` / `att_msg`: a specific SCALED_IMU2 and its ATTITUDE, as the
        manager's reader thread queued them. Omitted, the one-slot cache is read
        -- which a 50 Hz poller of the board's ~44 Hz stream aliases (measured: 384
        of 2501 publishes were repeats, and 42.4 of 44.0 Hz of samples got out).

        {'gyro': (x,y,z) rad/s, 'accel': (x,y,z) m/s^2 INCLUDING gravity,
         'board_ms': int|None, 'host_recv_s': float|None}

        ⛔ THIS DATA WAS ALREADY ON THE WIRE AND ENTIRELY THROWN AWAY. The board
        streams SCALED_IMU2 at 50 Hz (60 messages in 6 s, measured) and this
        stack read exactly ONE field off it -- `temperature`, for the water-temp
        readout. Accel and gyro were decoded by pymavlink and discarded every
        single frame. Same shape as the ESC RPM finding: the sensor is paid for
        in bandwidth and never spent.

        The firmware packs accel as `(s.lx + s.grx) / 9.80665 * 1000` in mg and
        gyro as `s.gx * 1000` in mrad/s (mav_stream.cpp:230-240), so accel
        carries gravity and a level hull at rest owes ~1000 mg on one axis.
        Exact zero on all six is the dead-sensor signature, not a neutral
        reading -- which is why this shares `_ahrs_healthy` with the attitude
        path rather than trusting the numbers.

        `board_ms` is the board's own capture time, the jitter-free base
        `flow_timing.ClockMap` maps onto host time. None when absent, never 0.
        """
        imu = self._cache('SCALED_IMU2') if imu_msg is None else imu_msg
        if imu is None:
            return None
        if self._ahrs_healthy() is False:
            return None
        boot = getattr(imu, 'time_boot_ms', None)
        g = 9.80665 / 1000.0          # mg -> m/s^2
        # The board's OWN fused attitude, carried alongside the raw axes.
        # ATTITUDE and SCALED_IMU2 are packed from one `Snap` in one firmware
        # tick, so this is the same instant and needs no second stamp.
        att = self._cache('ATTITUDE') if imu_msg is None else att_msg
        rpy = None if att is None else (float(att.roll), float(att.pitch),
                                        float(att.yaw))
        raw = (float(imu.xacc) * g, float(imu.yacc) * g, float(imu.zacc) * g)
        return {
            'rpy': rpy,
            'gyro': (float(imu.xgyro) * 1e-3,
                     float(imu.ygyro) * 1e-3,
                     float(imu.zgyro) * 1e-3),
            # VEHICLE frame (FRD), or None until the frame is proven -- see
            # `_accel_in_vehicle_frame`. None, never the raw axes: a consumer
            # handed the wrong frame integrates gravity sideways.
            'accel': self._accel_in_vehicle_frame(raw, rpy),
            'accel_frame': getattr(self, '_accel_frame', None),
            'board_ms': None if boot is None else int(boot),
            'host_recv_s': float(getattr(imu, '_timestamp', 0.0)) or None,
        }

    def _accel_in_vehicle_frame(self, raw, rpy):
        """SCALED_IMU2 accel, rotated into the frame its own gyro and ATTITUDE use.

        ⛔ THE BOARD SENDS THEM IN TWO DIFFERENT FRAMES. `bno085.cpp` (under
        `BNO_SWAP_ROLL_PITCH`) maps the attitude and the gyro from the sensor's
        axes into the vehicle's FRD frame with (x, y, z) -> (y, x, -z), a proper
        180 deg rotation about x=y. The gravity and linear-acceleration reports
        get NO remap, and `mav_stream.cpp` packs `lx + grx` straight into
        `xacc`. So the accel alone is still in the sensor's axes.

        Measured on the vehicle 2026-09-15, still on the bench:
          ATTITUDE roll +1.57 deg, pitch -15.10 deg
          predicted FRD specific force  (-2.54, -0.26, -9.47) m/s^2
          SCALED_IMU2 accel             (-0.41, -2.56, +9.38)
          ... mapped (y, x, -z)         (-2.56, -0.41, -9.38)
        Fed unmapped to the localization filter, 50 s of that board at rest
        ran to 583 m with 39 of 40 ZUPTs rejected; mapped, 0.036 m and 0 of 40.

        ⛔ NOT HARD-CODED, PROVEN AT RUNTIME. If the firmware ever remaps the
        accel too, applying the swap again would put gravity UP -- 180 deg off,
        and exactly as silent. So both hypotheses are scored against the
        gravity direction the board's own attitude predicts, and the frame is
        latched only after `_ACCEL_FRAME_VOTES` consecutive samples agree
        unambiguously. The two differ by the sign of z, so they are separable
        at any tilt. Until latched this returns None and the filter coasts on
        attitude, depth and flow -- absence, not a guess.
        """
        # Voting never stops. The manager builds one SrotFC per process, so a
        # board reflashed mid-session would otherwise keep a stale latch.
        frame = getattr(self, '_accel_frame', None)
        proven = self._vote_accel_frame(raw, rpy)
        if proven is not None and proven != frame:
            log = getattr(self, '_log', None)
            if log is not None:
                try:
                    (log.info if frame is None else log.warning)(
                        f'[SROT ] SCALED_IMU2 accel frame '
                        f'{"proven" if frame is None else "CHANGED from " + repr(frame) + " to"}: '
                        f'{proven!r} ({self._ACCEL_FRAME_VOTES} consecutive samples '
                        f'against the attitude-predicted gravity)')
                except Exception:   # noqa: BLE001 -- logging must not break telemetry
                    pass
            self._accel_frame = frame = proven
        if frame is None:
            return None
        if frame == 'sensor':
            return (raw[1], raw[0], -raw[2])
        return raw

    _ACCEL_FRAME_VOTES = 25          # 0.5 s at the board's 50 Hz
    _ACCEL_FRAME_AGREE_DEG = 10.0    # level-trim offsets measured at ~1 deg
    _ACCEL_FRAME_REJECT_DEG = 60.0

    def _vote_accel_frame(self, raw, rpy):
        if rpy is None or any(math.isnan(c) for c in rpy):
            return None
        mag = math.sqrt(sum(c * c for c in raw))
        # Only a hull near rest says where gravity is; a hard manoeuvre does not.
        if not (0.8 * 9.80665 < mag < 1.2 * 9.80665):
            self._accel_votes = (None, 0)
            return None
        r, p, _ = rpy
        g = 9.80665
        # Specific force at rest in FRD for aerospace Z-Y-X Euler angles.
        exp = (g * math.sin(p), -g * math.sin(r) * math.cos(p),
               -g * math.cos(r) * math.cos(p))

        def ang(v):
            c = sum(a * b for a, b in zip(v, exp)) / (mag * g)
            return math.degrees(math.acos(max(-1.0, min(1.0, c))))

        e_sensor = ang((raw[1], raw[0], -raw[2]))
        e_vehicle = ang(raw)
        if e_sensor < self._ACCEL_FRAME_AGREE_DEG and e_vehicle > self._ACCEL_FRAME_REJECT_DEG:
            vote = 'sensor'
        elif e_vehicle < self._ACCEL_FRAME_AGREE_DEG and e_sensor > self._ACCEL_FRAME_REJECT_DEG:
            vote = 'vehicle'
        else:
            self._accel_votes = (None, 0)
            return None
        last, n = getattr(self, '_accel_votes', (None, 0))
        n = n + 1 if vote == last else 1
        self._accel_votes = (vote, n)
        return vote if n >= self._ACCEL_FRAME_VOTES else None

    def heartbeat_age(self):
        hb = self._vehicle_hb()
        return None if hb is None else (time.time() - getattr(hb, '_timestamp', 0.0))

    def send_heartbeat(self):
        """Alias: the manager's heartbeat_tick calls this -- on SROT it IS the
        mandatory >=1 Hz GCS HEARTBEAT (2 Hz tick > the 1 Hz failsafe floor)."""
        self.send_gcs_heartbeat()

    def send_neutral(self):
        """Safe idle: zero MANUAL_CONTROL with heave neutral.

        NOT "STABILIZE holds", which is what this said until B28. After any
        SROT_MOVE the board is still in AUTO, where this frame is discarded
        outright -- benign here only because the frame is all zeros and AUTO's
        own idle demand is also zero. Only a defensive fallback on SROT;
        stop/pause route through move()."""
        self.manual(0.0, 0.0, 0.0, 0.0)

    def send_att_pos_mocap(self, yaw_deg):
        """No-op: SROT fuses the BNO on-board; there is no external EKF to feed."""
        return None

    # LANDING_TARGET (149) -- the vision uplink `VISION_API.md` specifies.
    #
    # THE BOARD DOES NOT CONSUME THIS YET. Verified in the firmware source:
    # `mav_commands::handle()` has no MAVLINK_MSG_ID_LANDING_TARGET case, so the
    # message is parsed and silently dropped, and `MAV_CMD_SROT_VISION` (31001)
    # returns MAV_RESULT_UNSUPPORTED. Their spec says so itself: "Status:
    # specification, not yet implemented."
    #
    # It is built now anyway, and default-off, because the firmware side's named
    # blocker was a measured camera FOV -- which we now have. Shipping the
    # producer lets them implement against a stream they can actually watch,
    # rather than against prose.
    _LANDING_TARGET_TYPE_VISION_OTHER = 3
    _MAV_FRAME_BODY_FRD = 12

    def send_landing_target(self, bearing, *, target_num: int = 0,
                            distance_m: float = 0.0,
                            coasted: bool = False,
                            gap_age_s: float = 0.0,
                            board_capture_s: Optional[float] = None) -> None:
        """Publish one selected target as a body-frame bearing. Fire-and-forget.

        ⛔ `board_capture_s`: WHEN THE FRAME WAS EXPOSED, ON THE BOARD'S CLOCK.
        A bearing is already tens of ms old when it lands, and the link adds a
        variable delay on top (arrival jitter measured at 6.67 ms sd, 35 ms
        p2p). The board holds its gyro at 500 Hz, so given the capture instant
        in its OWN `time_boot_ms` base it can de-rotate the bearing exactly by
        the angle it turned since -- something no host-side estimate matches.
        The host knows that instant because `ClockMap` already maps board time
        to host time; this is the inverse. When given, `time_usec` carries it
        (microseconds since board boot) and `z` = `UPLINK_TIME_BOARD` says so,
        so a receiver can never mistake it for the old advisory host stamp.
        None (clock map not fitted) keeps the previous bytes exactly.

        `bearing` is a `duburi_control.bearing.Bearing`. One message per frame
        for the ONE currently-selected target: the board never sees candidate
        boxes and never does data association, which is deliberate -- that is
        the host's job and it is where the class filter, the coast and the
        continuity lock live.

        ABSENCE OF A MESSAGE IS THE ONLY LOSS SIGNAL. There is no "lost" flag in
        the spec: a detector that sees nothing simply stops sending, and the
        board ages the last one out. So a caller must NOT keep re-sending a
        stale bearing to "hold" a target -- that is indistinguishable from a
        live one and defeats the board's staleness timer, which is the entire
        safety mechanism on this path.

        A non-finite field is dropped rather than sent. NaN on this wire would
        reach a 500 Hz control loop.

        `coasted` and `gap_age_s` ride the message's spare x/y fields, which the
        spec leaves undefined and we were sending as literal 0.0. They matter
        because the board can only age a target from ITS OWN receipt time: it
        cannot see that what just arrived is a Kalman prediction of a track
        whose last real detection was 0.35 s ago. Without them a coasted target
        is DOUBLE-DECAYED -- once by our coast authority and again by the
        board's staleness -- roughly twice as fast, so a coast we still consider
        live at 0.6 s is long dead on their side.

        Costs nothing today: the board parses msgid 149 and drops it. It becomes
        a latent bug the moment staleness is implemented, which is exactly when
        nobody would think to look here. Proposed upstream in PR #2 §4.
        """
        vals = (bearing.angle_x, bearing.angle_y, bearing.size_x, bearing.size_y)
        if not all(math.isfinite(float(v)) for v in vals):
            self._log_info('[SROT ] landing_target: non-finite bearing, dropped')
            return
        on_board = (board_capture_s is not None
                    and math.isfinite(board_capture_s) and board_capture_s > 0.0)
        with self._tx_lock:
            self.master.mav.landing_target_send(
                (int(board_capture_s * 1e6) if on_board
                 else int(time.time() * 1e6)),   # advisory host stamp otherwise
                int(target_num) & 0xFF,
                self._MAV_FRAME_BODY_FRD,
                float(bearing.angle_x), float(bearing.angle_y),
                float(distance_m),               # 0 = unknown
                float(bearing.size_x), float(bearing.size_y),
                # x = coasted flag, y = seconds since the last REAL detection.
                # z = which clock time_usec is on (see the docstring).
                1.0 if coasted else 0.0,
                float(max(0.0, gap_age_s)) if math.isfinite(gap_age_s) else 0.0,
                sp.UPLINK_TIME_BOARD if on_board else 0.0, (0.0, 0.0, 0.0, 0.0),
                self._LANDING_TARGET_TYPE_VISION_OTHER,
                0)                               # position_valid = 0, angle-only

    def send_speed_estimate(self, vx: float, vy: float, var_x: float,
                            var_y: float, board_s: Optional[float]) -> bool:
        """Our filtered BODY velocity to the board, as VISION_SPEED_ESTIMATE (103).

        ⛔ THE PI MEASURES, THE BOARD CLOSES THE LOOP. A distance move is an
        integral of velocity along the leg. The board can integrate at 500 Hz
        against its own fused heading between our 10 Hz updates; the host
        cannot close that loop through a 115200-baud link with 35 ms p2p
        arrival jitter. So this sends the one thing only the Pi has -- velocity
        over ground, from flow + the command model through the RIEKF -- and
        nothing else.

        FRAME: body FRD (x forward, y right), NOT the local frame the MAVLink
        spec names for 103, stated in the upstream PR. z is unobserved by a
        downward camera and says so with an enormous variance, never a
        confident 0.

        `usec` is the estimate's instant on the BOARD clock. Without a fitted
        `ClockMap` there is no honest stamp, so nothing is sent: an unstamped
        velocity lets the board integrate it at the wrong instant, which is a
        distance error that grows with every update.

        Returns whether it was sent. Non-finite input is refused, not clamped.
        """
        vals = (vx, vy, var_x, var_y)
        if board_s is None or not (math.isfinite(board_s) and board_s > 0.0):
            return False
        if not all(math.isfinite(float(v)) for v in vals) or var_x <= 0.0 or var_y <= 0.0:
            return False
        cov = (float(var_x), 0.0, 0.0,
               0.0, float(var_y), 0.0,
               0.0, 0.0, sp.SPEED_Z_UNOBSERVED_VAR)
        with self._tx_lock:
            self.master.mav.vision_speed_estimate_send(
                int(board_s * 1e6), float(vx), float(vy), 0.0, cov, 0)
        return True

    def set_message_rate(self, message_id, hz):
        """MAV_CMD_SET_MESSAGE_INTERVAL (511). Fire-and-forget, like Pixhawk's.

        This was a no-op -- "SROT telemetry rates are fixed on-board" -- which made
        the board's 10 Hz ATTITUDE the hard ceiling on every host loop, un-raisable
        for a control loop and un-lowerable for a slow link. Firmware behaviour rev 2
        implements 511 (and 510), so it is a real call now.

        Board-side rules worth knowing: any interval is clamped to a 20 ms floor
        (50 Hz) so a companion cannot starve the PARAM_VALUE / COMMAND_ACK traffic
        missions depend on, and a request to DISABLE HEARTBEAT is refused with
        DENIED rather than accepted-and-ignored. `hz <= 0` restores the board's
        compiled default for that stream.

        VERIFIED ON HARDWARE 2026-09-03 (fw rev 14), and one result is a trap:

            baseline            10.17 Hz  (100.0 ms)   the compiled default
            ask 50 Hz           50.17 Hz  ( 20.0 ms)   ACK ACCEPTED
            ask 100 Hz          50.00 Hz  ( 20.0 ms)   ACK *ACCEPTED*, clamped
            restore (hz<=0)     10.25 Hz  (100.0 ms)   ACK ACCEPTED

        A BELOW-FLOOR REQUEST IS ACCEPTED, NOT DENIED. The ACK says nothing about
        the rate you actually got, so a host that asks for 100 Hz and believes the
        ACCEPTED will size its loop for 10 ms of feedback and get 20. The only way
        to know the delivered rate is to measure the arrivals. (DENIED is reserved
        for an unknown msgid and for disabling HEARTBEAT.)

        This is also the first time the 50 Hz in `SROT_MESSAGE_RATES` has been
        confirmed to take effect: the board had only ever been observed at its
        10 Hz default, because nothing had run the manager against it.
        """
        if hz is None:
            return None
        interval_us = 0.0 if hz <= 0 else float(1e6 / float(hz))
        self._command_long(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                           p1=float(message_id), p2=interval_us)
        return None

    def calibrate_barometer(self, timeout: float = 6.0):
        """Re-zero the depth reference: PREFLIGHT_CALIBRATION (241) p3=1 (baro)."""
        self._clear_ack()
        self._command_long(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                            p1=0.0, p2=0.0, p3=1.0)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ack = self._cache('COMMAND_ACK')
            if ack is not None and ack.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION:
                return ack.result == sp.ACK_ACCEPTED, 'baro'
            time.sleep(_POLL_S)
        return False, 'NO_ACK'

    def get_param(self, name, timeout: float = 2.0):
        self.master.messages.pop('PARAM_VALUE', None)
        with self._tx_lock:
            self.master.mav.param_request_read_send(
                sp.VEHICLE_SYSID, sp.VEHICLE_COMPID,
                name.encode() if isinstance(name, str) else name, -1)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pv = self._cache('PARAM_VALUE')
            if pv is not None and _param_id(pv) == str(name):
                return float(pv.param_value)
            time.sleep(_POLL_S)
        return None

    def set_param(self, name, value, timeout: float = 3.0):
        self.master.messages.pop('PARAM_VALUE', None)
        with self._tx_lock:
            self.master.mav.param_set_send(
                sp.VEHICLE_SYSID, sp.VEHICLE_COMPID,
                name.encode() if isinstance(name, str) else name,
                float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pv = self._cache('PARAM_VALUE')
            if pv is not None and _param_id(pv) == str(name):
                return abs(float(pv.param_value) - float(value)) < 1e-3
            time.sleep(_POLL_S)
        return False

    # ------------------------------------------------------------------ #
    #  Pilot gain -- MANUAL_CONTROL is halved until GAIN=1.0              #
    # ------------------------------------------------------------------ #
    def read_gain(self):
        """Live pilot gain (NAMED_VALUE_FLOAT 'GAIN', 0.1..1.0), or None if unseen."""
        return self._named_value('GAIN')

    # ------------------------------------------------------------------ #
    #  Thrust-allocator saturation (srot-control-board#20)                 #
    # ------------------------------------------------------------------ #
    #  1.0 = the group is delivering the demand. Below 1.0 the mixer scaled
    #  that group down and the axis is NOT achieving what the controller asked
    #  for. `mixer::mix()` computes this per group and drops it, so until #20
    #  merges these read None -- which is UNKNOWN, not OK. A saturating
    #  allocator and a silent one are indistinguishable from here, and that is
    #  exactly the state #20 exists to end.
    ALLOCATOR_SAT_NAMES = ('MIX_SAT_H', 'MIX_SAT_V')

    def allocator_saturation(self):
        """(horizontal, vertical) delivered fraction, each 0..1 or None."""
        return tuple(self._named_value(n) for n in self.ALLOCATOR_SAT_NAMES)

    def check_move_cruise_max(self, timeout: float = 3.0):
        """Compare the BOARD's speed cap with the host's own clamp (B34).

        `sanitize_speed` clamps every host-issued move to `sp.MOVE_CRUISE_MAX`, a
        hard-coded copy of the firmware DEFAULT. The board's is a runtime param.
        They agree out of the box, and the mismatch is silent in ONE direction:
        RAISE the board's cap for a faster transit and every autonomous move is
        still clamped by us, while a teleop move -- which does not pass through
        `sanitize_speed` -- is not. The operator changes a speed limit, watches
        nothing change, and has no log line to explain it.

        Returns (ok, message). Never raises: a param that will not read is a
        degraded report, not a failed bring-up.
        """
        board = self.get_param('MOVE_CRUISE_MAX', timeout=timeout)
        if board is None:
            return True, ('MOVE_CRUISE_MAX unread -- host clamp '
                          f'{sp.MOVE_CRUISE_MAX:.2f} is the binding one')
        board = float(board)
        if abs(board - sp.MOVE_CRUISE_MAX) < 1e-3:
            return True, f'MOVE_CRUISE_MAX {board:.2f} (host and board agree)'
        if board > sp.MOVE_CRUISE_MAX:
            return False, (
                f'MOVE_CRUISE_MAX: board {board:.2f} > host clamp '
                f'{sp.MOVE_CRUISE_MAX:.2f} -- autonomous moves stay capped at '
                f'{sp.MOVE_CRUISE_MAX:.2f}. Raising the board param does NOT make '
                f'them faster; edit srot_protocol.MOVE_CRUISE_MAX too.')
        return True, (f'MOVE_CRUISE_MAX: board {board:.2f} < host clamp '
                      f'{sp.MOVE_CRUISE_MAX:.2f} -- the board is the binding one')

    def read_depth_p(self, timeout: float = 3.0):
        """Cache the board's DEPTH_P for `check_depth_loop_settled`. None if unread.

        Best-effort by design: the guard falls back to the firmware default and names
        the gain it used, so a missed read degrades the message rather than the check.
        """
        val = self.get_param('DEPTH_P', timeout=timeout)
        if val is not None:
            self.depth_p = float(val)
        return self.depth_p

    def check_yaw_reference(self):
        """(is_absolute, reason) from YAW_REF (fw rev 9).

        Only `LOCKED` means `ATTITUDE.yaw` is a magnetic heading. Anything else and it
        is relative to wherever the BNO booted -- so an ABSOLUTE `turn` (MOVE_TURN p4=1)
        is aiming at a number that means nothing, and silently: the move completes
        normally, on the wrong heading.

        ⚠ Do NOT substitute MAGACC. The board's alignment is protected by the |B| band
        and a sample-agreement test, neither of which depends on the sensor's opinion of
        itself, and with a stored calibration the accuracy requirement drops to 0 -- so
        accuracy stops correlating with the outcome entirely. We read MAGACC 2 on a hull
        whose lock state we could not determine at all, which is what prompted the
        firmware to publish this (Round 8 §8.8).

        Absent on fw < 9. Reported as unknown rather than assumed either way.
        """
        raw = self._named_value('YAW_REF')
        if raw is None:
            return False, ('yaw reference UNKNOWN (no YAW_REF -- fw < 9). Absolute '
                           '`turn` may be aiming at a boot-relative heading; prefer '
                           'relative turns')
        state = int(raw)
        if state == sp.YAW_REF_LOCKED:
            return True, 'yaw reference LOCKED -- heading is absolute, `turn` is safe'
        return False, (f'yaw reference NOT LOCKED: {state} = '
                       f'{sp.YAW_REF_NAMES.get(state, "unrecognised")}. ATTITUDE.yaw is '
                       f'relative to boot, so an ABSOLUTE `turn` will aim at a '
                       f'meaningless heading -- prefer relative turns until this reads '
                       f'{sp.YAW_REF_LOCKED}')

    def set_default_gain(self, value: float = sp.GAIN_FOR_AUTONOMY,
                         timeout: float = 3.0) -> bool:
        """Persist JS_GAIN_DEFAULT so programmatic MANUAL_CONTROL is full-scale.

        GAIN is runtime-only and boots at JS_GAIN_DEFAULT (0.5), so setting the
        param is the durable way to make autonomous manual() commands full
        authority. Returns True on a PARAM_VALUE echo confirming the value.
        """
        self.master.messages.pop('PARAM_VALUE', None)
        with self._tx_lock:
            self.master.mav.param_set_send(
                sp.VEHICLE_SYSID, sp.VEHICLE_COMPID, b'JS_GAIN_DEFAULT',
                float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pv = self._cache('PARAM_VALUE')
            if pv is not None:
                pid = getattr(pv, 'param_id', '')
                pid = pid.decode() if isinstance(pid, bytes) else str(pid)
                if pid.strip('\x00') == 'JS_GAIN_DEFAULT':
                    return abs(float(pv.param_value) - float(value)) < 1e-3
            time.sleep(_POLL_S)
        return False


# ---------------------------------------------------------------------- #
#  The verb table -- duburi Move verb -> SROT_MOVE (p1..p5).             #
#  Kept module-level + pure so it unit-tests without a live master.      #
# ---------------------------------------------------------------------- #
def _speed_from_gain(kw) -> float:
    """Move.Goal.gain is a 0..100 % thrust cap -> SROT 0..1 speed (clamped to cruise).

    A NON-FINITE GAIN IS REFUSED, NOT COERCED. `sanitize_speed` maps NaN to 0.0,
    which is the correct fail-safe for the STREAMING path (`manual()` cannot raise
    once per 20 Hz tick, and zero means no motion). It is the wrong answer for a
    ONE-SHOT verb: the board would accept a perfectly valid "move at speed 0",
    the hull would not move, and the mission would carry on believing it had --
    absence rendered as a number, which is the defect this register keeps finding.
    A verb that cannot be executed should say so. (B35)
    """
    gain = float(kw.get('gain', 0.0) or 0.0)
    if not math.isfinite(gain):
        raise ValueError(
            f'non-finite gain: {gain} -- a NaN gain usually means it was computed '
            f'from a VisionResult that never saw its target (check saw_target)')
    return sp.sanitize_speed(gain / 100.0)


def _depth_to_dive(kw) -> float:
    """duburi set_depth target is NEGATIVE metres (below surface); SROT DIVE p2 is a
    POSITIVE depth. Refuse a positive target (would be above the surface).

    ⛔ THE NON-FINITE CHECK IS FIRST, AND IT IS NOT DECORATION (B35). `target > 0.0`
    is False for NaN -- every comparison against NaN is -- so the one guard written
    to reject a bad depth target used to ACCEPT NaN and emit it as p2. `-inf` got
    through the same way and became `-(-inf)` = **+inf**: a request to dive to
    infinite depth. Found by hammering the verb table, not by reading it.
    """
    target = float(kw.get('target', 0.0) or 0.0)
    if not math.isfinite(target):
        raise ValueError(f'non-finite set_depth target: {target}')
    if target > 0.0:
        raise ValueError(f'set_depth target must be <=0 (below surface), got {target}')
    return -target


def _finite_param(name: str, value: float) -> float:
    """A wire parameter, or a ValueError NAMING the field that was not finite.

    ⚠ THIS IS NOT WHAT STOPS NaN REACHING THE WIRE. `move()` already refuses the
    whole frame via the `_finite(*vals)` predicate above -- and the board refuses
    it again (fw `mav_commands.cpp:279-292`). Both of those say only
    "non-finite parameter"; this says WHICH ONE, at the point where the field
    still has a name. That is the entire value it adds, and B35's entry is
    written to keep that honest.
    """
    v = float(value)
    if not math.isfinite(v):
        # Keep the words "non-finite" -- `move()`'s catch-all uses them and
        # test_move_denied_on_nonfinite_param matches on them. This adds the
        # FIELD NAME without changing the contract callers already read.
        raise ValueError(
            f'non-finite {name}: {v} -- a NaN here almost always comes from a '
            f'VisionResult whose target was never seen (x_px/y_px are NaN then; '
            f'check saw_target before using them)')
    return v


def _build_params(verb: str, kw: dict):
    """duburi verb + Move.Goal-ish kwargs -> (p1, p2, p3, p4, p5). Raises KeyError
    for an unmapped verb, ValueError for a bad parameter. This is THE verb table.

    Every returned tuple is checked finite before it leaves (B35): a NaN reached
    the wire from `duration`, `target` or `gain`, and NaN has a live source --
    `VisionResult.x_px` is NaN when the target was never seen."""
    # Validated at the boundary, so no per-verb branch below can forget it.
    # `speed` needs no check: sanitize_speed maps NaN to 0.0 and clamps inf.
    dur   = _finite_param('duration', kw.get('duration', 0.0) or 0.0)
    tmo   = _finite_param('timeout',  kw.get('timeout', 0.0) or 0.0)
    speed = _speed_from_gain(kw)
    if verb == 'move_forward':
        return (sp.MOVE_FORWARD, dur, speed, 0.0, tmo)
    if verb == 'move_back':
        # MOVE_BACK=1 has always existed on the wire, and the old host brake already
        # commanded it on every forward abort -- the verb was refused only because
        # this branch was missing, so it fell through to the Pixhawk path and raised
        # AttributeError. Pure host-side gap, no firmware change involved.
        return (sp.MOVE_BACK, dur, speed, 0.0, tmo)
    if verb == 'move_left':
        return (sp.MOVE_STRAFE_L, dur, speed, 0.0, tmo)
    if verb == 'move_right':
        return (sp.MOVE_STRAFE_R, dur, speed, 0.0, tmo)
    # NOTE: 'arc' is deliberately NOT mapped. duburi's arc holds an ABSOLUTE target
    # heading (target_yaw, deg) while curving forward; SROT MOVE_ARC p4 is a signed
    # yaw RATE (deg/s) with no heading lock -- different primitives. Passing the
    # heading as a rate would spin the hull. Deferred until a host-side heading->rate
    # arc lands; arc is excluded from MOVE_VERBS so it never routes here on SROT.
    if verb == 'yaw_left':
        return (sp.MOVE_TURN, -abs(_finite_param('target', kw.get('target', 0.0) or 0.0)), 0.0,
                float(sp.TURN_RELATIVE), tmo)
    if verb == 'yaw_right':
        return (sp.MOVE_TURN, abs(_finite_param('target', kw.get('target', 0.0) or 0.0)), 0.0,
                float(sp.TURN_RELATIVE), tmo)
    if verb == 'turn':
        # duburi 'turn' is an ABSOLUTE heading -> needs MAG_YAW_REF=1 on the board.
        return (sp.MOVE_TURN, _finite_param('target', kw.get('target', 0.0) or 0.0), 0.0,
                float(sp.TURN_ABSOLUTE), tmo)
    if verb == 'set_depth':
        return (sp.MOVE_DIVE, _depth_to_dive(kw), 0.0, 0.0, tmo)
    if verb == 'stop':
        return (sp.MOVE_STOP, 0.0, 0.0, 0.0, tmo)
    if verb == 'pause':
        # No channel-release on SROT -> station-keep hold (type 7) until timeout.
        return (sp.MOVE_HOLD, dur, 0.0, 0.0, tmo)
    if verb == 'style_roll':
        return (sp.MOVE_STYLE, float(kw.get('flips', 1.0) or 1.0), 0.0, 0.0, tmo)
    raise KeyError(f"verb '{verb}' has no SROT_MOVE mapping")


# Verbs this backend can express as a single SROT_MOVE (the facade routes these
# through move(); everything else -- arm/disarm/set_mode, manual-streamed
# move_*_dist / vision, fire, dvl -- goes through the other methods or stays host-side).
MOVE_VERBS = frozenset({
    'move_forward', 'move_back', 'move_left', 'move_right',
    'yaw_left', 'yaw_right', 'turn', 'set_depth', 'stop', 'pause', 'style_roll',
})   # 'arc' excluded: heading-hold vs SROT's rate-arc mismatch (see _build_params)


# Verbs that must be REFUSED on this backend rather than dispatched.
#
# Everything here reaches Pixhawk-only primitives that SrotFC does not implement
# (`send_rc_override` / `send_rc_translation` / `send_rc_yaw_only` /
# `set_target_depth`) or gates on the ArduSub-only ALT_HOLD mode. Left to fall
# through, each fails in a way that is WORSE than an honest refusal:
#
#   lock_heading  returns success=True and holds nothing, while HeadingLock's
#                 daemon swallows an AttributeError 50x/second for the lock's
#                 300 s lifetime. A mission that believes its heading is held
#                 will dead-reckon straight off course -- the single most
#                 dangerous of these.
#   *_dist        DVL-driven distance moves; the streamed path is not ported.
#   vision_*      the 20 Hz vision loop writes RC channels directly.
#   arc/style_yaw die at the ALT_HOLD mode gate with a confusing ModeChangeError.
#
# The manager checks this BEFORE dispatch and returns a clean success=False, which
# is what `srot-integration.md` always claimed the behaviour was. Removing a verb
# from this set is how the port lands: implement it, then delete the line.
UNSUPPORTED_VERBS = frozenset({
    'lock_heading',
    'move_forward_dist', 'move_back_dist', 'move_lateral_dist',
    'arc', 'style_yaw',
})

# `vision_align` / `vision_move` CAME OUT of the set above (2026-09-03). They now
# actuate through `SrotFC.manual()` -> MANUAL_CONTROL, in STABILIZE, where the
# board holds attitude and heading at 500 Hz and the host servos lat/yaw/fwd on
# top. That path does NOT enter AUTO, so it does not touch the depth-loop gate
# that every SROT_MOVE goes through.
#
# WHAT UN-REFUSING COSTS, because it is not free. `UNSUPPORTED_VERBS` was the
# ONLY thing making the host motion stack unreachable on this backend --
# `heading_lock`, `motion_yaw`, `motion_depth` are all still present and
# imported by `duburi.py`. Removing an entry re-arms whatever that verb touches.
# For these two that is bounded and checked:
#   * the depth axis is REFUSED in `vision_verbs` (it needs `set_target_depth`,
#     which this class does not implement) rather than silently doing nothing;
#   * `_ensure_alt_hold` is skipped -- ALT_HOLD is an ArduSub mode this board
#     does not have, and hitting the facade's mode gate is what made `surface`
#     do nothing on this backend;
#   * `lock_heading` stays refused, so no host lock races the board's own hold.
# `test_no_facade_mode_gate_is_reachable_on_srot` pins that every verb is in
# exactly one of these buckets, so the next removal cannot be silent.


# ---------------------------------------------------------------------- #
#  SrotPayload -- payload over MAVLink (replaces the obsolete USB ESP32)  #
# ---------------------------------------------------------------------- #
# THERE IS NO HOST-SIDE CHANNEL MAP, DELIBERATELY.
#
# `fire(N)` addresses BOARD channel N -- literally `DO_SET_SERVO param1`, 1..16,
# the same N that names `SERVO{N}_ROLE` and the same N printed by
# `ros2 run duburi_manager connect`. The previous design routed a "duburi channel"
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
    """Payload driver for the SROT backend: the board's PCA9685 expander over
    MAVLink. Duck-types the USB `PayloadDriver` surface (`is_ready`, `fire`,
    `port_path`) so the Duburi facade is unchanged; there is NO separate USB
    ESP32 anymore, so the old CH340 auto-detect must not run on srot.

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

        ⚠ REQUIRES THE READER THREAD TO BE RUNNING. `get_param` reads the pymavlink
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
            # over the BlueOS/Bridget bridge (~8% frame loss): one channel of the 16
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
            # ⚠ Do NOT pipeline these two requests. `get_param` pops the single
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

    @property
    def port_path(self) -> str:
        return 'SROT MAVLink (PCA9685 DO_SET_SERVO)'

    def fire(self, channel: int) -> FireResult:
        """Activate BOARD channel `channel` (1..16) as a bounded pulse.

        `channel` is `DO_SET_SERVO param1` -- the same number as `SERVO{n}_ROLE`.
        There is no mapping step and nothing is looked up to decide WHERE to send.

        The only decision made here is WHETHER to send, and it is made from the
        board's own role config, not from a host opinion.
        """
        ch = int(channel)
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
                f'duburi_ws drives SWITCH channels only. Set SERVO{ch}_ROLE=2 in '
                f'Bondor if this really is a payload channel.')
        if role != sp.PCA_ROLE_SWITCH:
            return FireResult(
                FIRE_DISABLED, ch,
                f'channel {ch} role is {role} (disabled) -- driving it would be a '
                f'silent no-op on the board.')

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

    def disconnect(self) -> None:
        pass
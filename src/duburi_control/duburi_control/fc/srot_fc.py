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

import math
import os
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

from .base import (FlightController, Telemetry, MoveResult,
                   SUCCEEDED, PREEMPTED, FAILED, DENIED, TIMEOUT, ABORTED)
from . import srot_protocol as sp


# COMMAND_ACK.result -> MoveResult code. Terminal set + values live in
# srot_protocol (pinned, since older pymavlink dialects lack CANCELLED=6).
# TEMPORARILY_REJECTED means the board missed a state mutex at dispatch, so the
# move never started -- same outcome as FAILED for a caller, different reason text.
_ACK_TO_CODE = {sp.ACK_ACCEPTED: SUCCEEDED, sp.ACK_CANCELLED: PREEMPTED,
                sp.ACK_FAILED: FAILED, sp.ACK_DENIED: DENIED,
                sp.ACK_TEMPORARILY_REJECTED: FAILED}

_ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

_POLL_S = 0.05    # cache-poll granularity for ack/arm/mode loops
_LINK_STALE_S = 3.0

_ACK_MARGIN_S     = 5.0    # slack over the expected leg time before calling it a stall
_ACK_MIN_BUDGET_S = 8.0    # floor, so a 0.5 s leg still tolerates a slow first ACK
_STYLE_ROLL_S     = 360.0 / 90.0   # MOVE_STYLE is always a roll at 90 deg/s


def _finite(*vals) -> bool:
    """True iff every value is a finite float (goal validation before send)."""
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals)


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


def _param_id(pv) -> str:
    """PARAM_VALUE.param_id as a clean str (mavlink sends a padded bytes field)."""
    pid = getattr(pv, 'param_id', '')
    pid = pid.decode() if isinstance(pid, bytes) else str(pid)
    return pid.strip('\x00')


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
        # Board's SROT_FW_BEHAVIOUR_REV once read: int, or None while unknown.
        self._behaviour_rev = None
        self._behaviour_rev_logged = False
        # Operator escape hatch (ROS param `allow_fw_behaviour_mismatch`). Off by
        # default: the failure this guards is silent, so opting into it must not be.
        self.allow_fw_behaviour_mismatch = False

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
        with self._tx_lock:
            self.master.mav.command_long_send(
                sp.VEHICLE_SYSID, sp.VEHICLE_COMPID, command, confirmation,
                float(p1), float(p2), float(p3), float(p4),
                float(p5), float(p6), float(p7))

    def _cache(self, msgtype):
        return self.master.messages.get(msgtype)

    def _clear_ack(self):
        self.master.messages.pop('COMMAND_ACK', None)

    def _named_value(self, name):
        """Latest NAMED_VALUE_FLOAT for `name`, or None. SROT multiplexes several
        scalars (MV_STATE, LEAK, WTEMP, GAIN...) on this one message type, so the
        cache holds only the most-recent regardless of name -- match on .name."""
        msg = self._cache('NAMED_VALUE_FLOAT')
        if msg is None:
            return None
        mname = getattr(msg, 'name', '')
        mname = mname.decode() if isinstance(mname, bytes) else str(mname)
        return float(msg.value) if mname.strip('\x00').strip() == name else None

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
        GCS failsafe and SURFACEs the vehicle mid-mission."""
        with self._tx_lock:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
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
            return False, f"unknown SROT mode '{mode}'"
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
    def manual(self, fwd: float, lat: float, up: float, yaw: float) -> None:
        """One MANUAL_CONTROL frame. x=fwd, y=lat(+starboard), z=heave(+up), r=yaw;
        all axes -1..1 (the board clamps; we clamp in srot_protocol). Buttons=0.

        A non-finite axis is coerced to neutral (0) rather than raising: this is the
        streamed 20 Hz servo primitive, so a single NaN from a vision loop must not
        crash the send -- it degrades to 'hold' for that tick."""
        def _safe(v):
            return v if isinstance(v, (int, float)) and math.isfinite(v) else 0.0
        x = sp.unit_to_mc(_safe(fwd))
        y = sp.unit_to_mc(_safe(lat))
        r = sp.unit_to_mc(_safe(yaw))
        z = sp.unit_to_mc_z(_safe(up))
        with self._tx_lock:
            self.master.mav.manual_control_send(sp.VEHICLE_SYSID, x, y, z, r, 0)

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
        the SROT board dispatches to its servo expander."""
        self._command_long(sp.CMD_DO_SET_SERVO, p1=float(int(channel_1based)),
                            p2=float(int(us)))

    def set_relay(self, instance_0based: int, on: bool) -> None:
        """DO_SET_RELAY (181): switch PCA9685 MOSFET `instance_0based` (0-based ->
        PCA ch PCA_RELAY_BASE_CH + instance) on/off. Raw firmware match."""
        self._command_long(sp.CMD_DO_SET_RELAY, p1=float(int(instance_0based)),
                            p2=(1.0 if on else 0.0))

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
        deadline = time.monotonic() + budget
        last_prog = -1.0
        while time.monotonic() < deadline:
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
                if ack.result == sp.ACK_IN_PROGRESS and on_progress is not None:
                    prog = float(getattr(ack, 'progress', 0)) / 100.0
                    if prog != last_prog:
                        on_progress(prog)
                        last_prog = prog
            time.sleep(_POLL_S)
        # No terminal ACK inside the budget -> stall. Brake to be safe.
        self.stop_motion()
        return MoveResult(TIMEOUT, f'{verb}: no terminal ACK within {budget:.0f}s (stall)')

    def _terminal_reason(self, verb, code, result=None) -> str:
        if code == SUCCEEDED:
            return f'{verb}: completed'
        if code == PREEMPTED:
            return f'{verb}: preempted by a newer move'
        st = self._statustext()
        if result == sp.ACK_TEMPORARILY_REJECTED:
            # Distinct from a plain FAILED: the board was busy, not unable. Say so,
            # because "retry" is the right response here and not for the others.
            return f'{verb}: board busy (state lock) -- not started, safe to retry'
        if code == FAILED:
            return f'{verb}: could not start' + (f' ({st})' if st else '')
        return f'{verb}: denied' + (f' ({st})' if st else '')

    # ------------------------------------------------------------------ #
    #  Telemetry                                                          #
    # ------------------------------------------------------------------ #
    def _statustext(self) -> str:
        msg = self._cache('STATUSTEXT')
        if msg is None:
            return ''
        text = getattr(msg, 'text', '')
        return text.decode() if isinstance(text, bytes) else str(text)

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
        vhud = self._cache('VFR_HUD')
        if vhud is not None:
            t.depth_m = float(vhud.alt)
        batt = self._cache('BATTERY_STATUS')     # id 0 = electronics pack
        if batt is not None:
            volts = getattr(batt, 'voltages', [65535])
            mv = volts[0] if volts else 65535
            t.battery_voltage = (mv / 1000.0) if mv not in (0, 65535) else math.nan
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
        leak = self._named_value('LEAK')
        if leak is not None:
            t.leak = leak >= 0.5
        wtemp = self._named_value('WTEMP')
        if wtemp is not None:
            t.water_temp_c = wtemp
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
        depth = math.nan
        vhud = self._cache('VFR_HUD')
        if vhud is not None:
            depth = float(vhud.alt)
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

    def get_battery(self):
        """{'voltage','current'} (V/A) from BATTERY_STATUS id 0, or None -- matches
        Pixhawk's dict contract (the manager reads battery['voltage'])."""
        msg = self._cache('BATTERY_STATUS')
        if msg is None:
            return None
        volts = getattr(msg, 'voltages', [0xFFFF])
        raw_mv = volts[0] if volts else 0xFFFF
        voltage = math.nan if raw_mv in (0, 0xFFFF) else raw_mv / 1000.0
        cur = getattr(msg, 'current_battery', -1)
        current = math.nan if cur == -1 else cur / 100.0
        return {'voltage': voltage, 'current': current}

    def get_rc_channels(self):
        return None            # SROT has no RC-channel readback (MANUAL_CONTROL only)

    def get_statustext(self):
        st = self._statustext()
        return st or None

    def get_angular_rates(self):
        """Body-frame angular rates (rad/s) from ATTITUDE, matching Pixhawk's contract:
        {'roll_rate','pitch_rate','yaw_rate','age_s'} or None. The manager's
        _imu_rates_tick reads the *_rate keys, so they MUST match exactly."""
        att = self._cache('ATTITUDE')
        if att is None:
            return None
        age = time.time() - getattr(att, '_timestamp', 0.0) if getattr(att, '_timestamp', 0.0) else 0.0
        return {'roll_rate': float(getattr(att, 'rollspeed', 0.0)),
                'pitch_rate': float(getattr(att, 'pitchspeed', 0.0)),
                'yaw_rate': float(getattr(att, 'yawspeed', 0.0)),
                'age_s': age}

    def heartbeat_age(self):
        hb = self._vehicle_hb()
        return None if hb is None else (time.time() - getattr(hb, '_timestamp', 0.0))

    def send_heartbeat(self):
        """Alias: the manager's heartbeat_tick calls this -- on SROT it IS the
        mandatory >=1 Hz GCS HEARTBEAT (2 Hz tick > the 1 Hz failsafe floor)."""
        self.send_gcs_heartbeat()

    def send_neutral(self):
        """Safe idle: zero MANUAL_CONTROL with heave neutral (STABILIZE holds).
        Only a defensive fallback on SROT -- stop/pause route through move()."""
        self.manual(0.0, 0.0, 0.0, 0.0)

    def send_att_pos_mocap(self, yaw_deg):
        """No-op: SROT fuses the BNO on-board; there is no external EKF to feed."""
        return None

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
    """Move.Goal.gain is a 0..100 % thrust cap -> SROT 0..1 speed (clamped to cruise)."""
    return sp.sanitize_speed(float(kw.get('gain', 0.0) or 0.0) / 100.0)


def _depth_to_dive(kw) -> float:
    """duburi set_depth target is NEGATIVE metres (below surface); SROT DIVE p2 is a
    POSITIVE depth. Refuse a positive target (would be above the surface)."""
    target = float(kw.get('target', 0.0) or 0.0)
    if target > 0.0:
        raise ValueError(f'set_depth target must be <=0 (below surface), got {target}')
    return -target


def _build_params(verb: str, kw: dict):
    """duburi verb + Move.Goal-ish kwargs -> (p1, p2, p3, p4, p5). Raises KeyError
    for an unmapped verb, ValueError for a bad parameter. This is THE verb table."""
    dur   = float(kw.get('duration', 0.0) or 0.0)
    tmo   = float(kw.get('timeout', 0.0) or 0.0)
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
        return (sp.MOVE_TURN, -abs(float(kw.get('target', 0.0) or 0.0)), 0.0,
                float(sp.TURN_RELATIVE), tmo)
    if verb == 'yaw_right':
        return (sp.MOVE_TURN, abs(float(kw.get('target', 0.0) or 0.0)), 0.0,
                float(sp.TURN_RELATIVE), tmo)
    if verb == 'turn':
        # duburi 'turn' is an ABSOLUTE heading -> needs MAG_YAW_REF=1 on the board.
        return (sp.MOVE_TURN, float(kw.get('target', 0.0) or 0.0), 0.0,
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
    'vision_align', 'vision_move',
    'arc', 'style_yaw',
})


# ---------------------------------------------------------------------- #
#  SrotPayload -- payload over MAVLink (replaces the obsolete USB ESP32)  #
# ---------------------------------------------------------------------- #
# Duburi channels 1/2 = torpedo, 3/4 = dropper. On SROT each maps to a PCA9685
# action: a SERVO (pulse to a release µs, then back to rest) or a MOSFET/RELAY
# (energise for a pulse, then off). The exact per-channel wiring is a hardware
# fact -- set _FIRE_MAP from the operator's answer. Each entry is one of:
#   ('servo', pca_channel_1based, fire_us, rest_us)
#   ('relay', instance_0based)
# fire() runs a BOUNDED pulse in try/finally so a payload is never left energised
# (a MOSFET held on burns the solenoid coil; a servo held at end-stop stalls).
_FIRE_PULSE_S = 0.6   # > any PCA service tick; long enough for a servo to travel

# EMPTY BY DESIGN -- fire() refuses until the real wiring is configured.
#
# There is no safe default here. Which PCA channel each of torpedo_1/2 and
# dropper_1/2 is on, and whether each is a servo or a MOSFET, are hardware facts
# this code cannot infer, and guessing them means firing the wrong actuator on a
# live vehicle. A loud "not configured" beats a silent mis-actuation.
#
# Set it from the operator's wiring via `SrotFC`'s `fire_map` argument (the
# manager plumbs the `payload_fire_map` ROS param through), e.g.:
#     {1: ('relay', 0), 2: ('relay', 1), 3: ('servo', 3, 2000, 1000)}
#
# Round 3 (fw R15) made every channel reachable by its own number whatever its
# role: on a role-2 (MOSFET/switch) channel DO_SET_SERVO now treats the pulse
# width as a level, >=1500 us = ON. So a ('servo', ch, 2000, 1000) entry does
# drive a MOSFET correctly -- but only once SERVOn_ROLE is actually set to 2.
# Prefer ('relay', n) for channels 9-16, which map as PCA_RELAY_BASE_CH + n.
_FIRE_MAP: dict = {}


def parse_fire_map(spec, log=None) -> dict:
    """Parse the `payload_fire_map` ROS param string into the `_FIRE_MAP` shape.

    Grammar (comma-separated entries, whitespace ignored) -- the format the
    refusal message in `fire()` already tells the operator to use:

        <channel>:relay:<instance>
        <channel>:servo:<pca_ch>[:<fire_us>:<rest_us>]

    e.g. ``"1:relay:0, 2:relay:1, 3:servo:3, 4:servo:5:1900:1100"``

    `channel` is the duburi payload channel `fire()` is called with (1/2 torpedo,
    3/4 dropper). `instance` is the 0-based DO_SET_RELAY instance, which the board
    maps to PCA channel ``PCA_RELAY_BASE_CH + instance``. `pca_ch` is the 1-based
    DO_SET_SERVO channel. Servo µs default to SERVO_MAX_US / SERVO_MIN_US.

    A blank spec returns {} and `fire()` keeps refusing loudly -- correct when
    nobody has stated the wiring. A MALFORMED entry is skipped with an error
    rather than aborting the whole map: a typo in one channel must not silently
    disarm the other three.
    """
    out: dict = {}
    if not spec:
        return out

    def _bad(entry, why):
        if log:
            log.error(f'[PAYLOAD] payload_fire_map: ignoring {entry!r} -- {why}')

    for raw in str(spec).split(','):
        entry = raw.strip()
        if not entry:
            continue
        parts = [p.strip() for p in entry.split(':')]
        if len(parts) < 3:
            _bad(entry, 'expected <channel>:relay:<instance> or '
                        '<channel>:servo:<ch>[:fire_us:rest_us]')
            continue
        try:
            channel = int(parts[0])
            kind = parts[1].lower()
            target = int(parts[2])
        except ValueError:
            _bad(entry, 'channel / instance / pca_ch must be integers')
            continue
        if channel <= 0:
            _bad(entry, 'payload channel must be >= 1')
            continue

        if kind == 'relay':
            n_relay = sp.PCA9685_NUM_CH - sp.PCA_RELAY_BASE_CH
            if not 0 <= target < n_relay:
                _bad(entry, f'relay instance {target} out of range (0..{n_relay - 1})')
                continue
            out[channel] = ('relay', target)
        elif kind == 'servo':
            if not 1 <= target <= sp.PCA9685_NUM_CH:
                _bad(entry, f'servo channel {target} out of range (1..{sp.PCA9685_NUM_CH})')
                continue
            fire_us, rest_us = sp.SERVO_MAX_US, sp.SERVO_MIN_US
            if len(parts) >= 5:
                try:
                    fire_us, rest_us = int(parts[3]), int(parts[4])
                except ValueError:
                    _bad(entry, 'fire_us / rest_us must be integers')
                    continue
            if not all(sp.SERVO_MIN_US <= v <= sp.SERVO_MAX_US for v in (fire_us, rest_us)):
                _bad(entry, f'us values must be {sp.SERVO_MIN_US}..{sp.SERVO_MAX_US}')
                continue
            out[channel] = ('servo', target, fire_us, rest_us)
        else:
            _bad(entry, f"unknown kind {kind!r} -- expected 'relay' or 'servo'")
    return out


class SrotPayload:
    """Payload driver for the SROT backend: the board's PCA9685 expander over
    MAVLink. Duck-types the USB `PayloadDriver` surface (`is_ready`, `fire`,
    `port_path`) so the Duburi facade is unchanged; there is NO separate USB
    ESP32 anymore, so the old CH340 auto-detect must not run on srot."""

    def __init__(self, fc, log=None, fire_map=None):
        self._fc = fc
        self._log = log
        self._map = dict(fire_map if fire_map is not None else _FIRE_MAP)

    @property
    def is_ready(self) -> bool:
        # The payload rides the same MAVLink link as everything else.
        return bool(getattr(self._fc, 'link_alive', lambda: True)())

    @property
    def port_path(self) -> str:
        return 'SROT MAVLink (PCA9685 DO_SET_SERVO/RELAY)'

    def fire(self, channel: int) -> bool:
        """Actuate the payload for `channel` (1/2 torpedo, 3/4 dropper) as a bounded
        pulse. Returns True on a mapped, actuated channel; False (no-op) if unmapped."""
        spec = self._map.get(int(channel))
        if spec is None:
            if self._log:
                what = ('the payload fire map is EMPTY' if not self._map
                        else f'channel {channel} is not in the fire map')
                self._log.error(
                    f'[PAYLOAD] SROT: NOT FIRED -- {what}. Set the `payload_fire_map` '
                    f'ROS param from the real PCA9685 wiring, e.g. '
                    f'"1:relay:0, 2:relay:1, 3:servo:3". Refusing to guess.')
            return False
        kind = spec[0]
        try:
            if kind == 'servo':
                _, ch1, fire_us, rest_us = spec
                self._fc.set_servo(ch1, fire_us)
                time.sleep(_FIRE_PULSE_S)
                return True
            if kind == 'relay':
                _, inst = spec
                self._fc.set_relay(inst, True)
                time.sleep(_FIRE_PULSE_S)
                return True
            return False
        finally:
            # Always return to rest / de-energise -- never leave a solenoid on or a
            # servo stalled, even if the sleep is interrupted.
            try:
                if kind == 'servo':
                    self._fc.set_servo(spec[1], spec[3])
                elif kind == 'relay':
                    self._fc.set_relay(spec[1], False)
            except Exception as exc:   # noqa: BLE001 -- best-effort de-energise
                if self._log:
                    self._log.error(f'[PAYLOAD] SROT: reset raised {exc!r}')

    def disconnect(self) -> None:
        pass

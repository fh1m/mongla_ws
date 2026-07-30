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
import threading
import time

from pymavlink import mavutil

from .base import (FlightController, Telemetry, MoveResult,
                   SUCCEEDED, PREEMPTED, FAILED, DENIED, TIMEOUT, ABORTED)
from . import srot_protocol as sp


# COMMAND_ACK.result -> MoveResult code. Terminal set + values live in
# srot_protocol (pinned, since older pymavlink dialects lack CANCELLED=6).
_ACK_TO_CODE = {sp.ACK_ACCEPTED: SUCCEEDED, sp.ACK_CANCELLED: PREEMPTED,
                sp.ACK_FAILED: FAILED, sp.ACK_DENIED: DENIED}

_ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

_POLL_S = 0.05    # cache-poll granularity for ack/arm/mode loops
_LINK_STALE_S = 3.0


def _finite(*vals) -> bool:
    """True iff every value is a finite float (goal validation before send)."""
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals)


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
        HEARTBEAT for the armed bit. Honours ``abort`` (()->bool) mid-poll."""
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
        """Brake to a halt (SROT_MOVE type 6). Used for cooperative abort/cancel."""
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

        self._clear_ack()
        self._command_long(sp.CMD_SROT_MOVE, p1=p1, p2=p2, p3=p3, p4=p4, p5=p5)
        return self._relay_move_ack(verb, on_progress, abort_fn, p5)

    def _relay_move_ack(self, verb, on_progress, abort_fn, timeout_s) -> MoveResult:
        # Deadline: the board's own timeout (p5) plus margin; 60 s default when 0.
        budget = (float(timeout_s) if timeout_s and timeout_s > 0 else 60.0) + 5.0
        deadline = time.monotonic() + budget
        last_prog = -1.0
        while time.monotonic() < deadline:
            if abort_fn is not None and abort_fn():
                self.stop_motion()               # brake; board resolves CANCELLED
                return MoveResult(ABORTED, f'{verb}: aborted (stop sent)')
            ack = self._cache('COMMAND_ACK')
            if ack is not None and ack.command == sp.CMD_SROT_MOVE:
                if ack.result in sp.TERMINAL_ACKS:
                    code = _ACK_TO_CODE[ack.result]
                    if on_progress is not None and code == SUCCEEDED:
                        on_progress(1.0)
                    return MoveResult(code, self._terminal_reason(verb, code))
                if ack.result == sp.ACK_IN_PROGRESS and on_progress is not None:
                    prog = float(getattr(ack, 'progress', 0)) / 100.0
                    if prog != last_prog:
                        on_progress(prog)
                        last_prog = prog
            time.sleep(_POLL_S)
        # No terminal ACK inside the budget -> stall. Brake to be safe.
        self.stop_motion()
        return MoveResult(TIMEOUT, f'{verb}: no terminal ACK within {budget:.0f}s (stall)')

    def _terminal_reason(self, verb, code) -> str:
        if code == SUCCEEDED:
            return f'{verb}: completed'
        if code == PREEMPTED:
            return f'{verb}: preempted by a newer move'
        st = self._statustext()
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
        # Depth: VFR_HUD.alt = -depth (negative underwater) -> positive-down depth_m.
        # Stays NaN until VFR_HUD arrives (5 Hz).
        vhud = self._cache('VFR_HUD')
        if vhud is not None:
            t.depth_m = -float(vhud.alt)
        batt = self._cache('BATTERY_STATUS')     # id 0 = electronics pack
        if batt is not None:
            volts = getattr(batt, 'voltages', [65535])
            mv = volts[0] if volts else 65535
            t.battery_voltage = (mv / 1000.0) if mv not in (0, 65535) else math.nan
        esc = self._cache('ESC_STATUS')           # per-thruster RPM (Bluejay bidir DShot)
        if esc is not None:
            t.rpm = tuple(int(r) for r in getattr(esc, 'rpm', ()) or ())
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
        is unchanged. yaw in degrees (ATTITUDE is radians); depth from VFR_HUD
        (alt = -depth)."""
        att = self._cache('ATTITUDE')
        if att is None:
            return None
        depth = math.nan
        vhud = self._cache('VFR_HUD')
        if vhud is not None:
            depth = -float(vhud.alt)
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
        """No-op: SROT telemetry rates are fixed on-board (no SET_MESSAGE_INTERVAL)."""
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
    'move_forward', 'move_left', 'move_right',
    'yaw_left', 'yaw_right', 'turn', 'set_depth', 'stop', 'pause', 'style_roll',
})   # 'arc' excluded: heading-hold vs SROT's rate-arc mismatch (see _build_params)


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

# Default = the config.h payload servo on PCA ch 0 for channel 1, then the next
# three servo channels. VERIFY against the real wiring before trusting it.
_FIRE_MAP = {
    1: ('servo', 1, sp.SERVO_MAX_US, sp.SERVO_MIN_US),   # torpedo_1 -> PCA servo ch 0
    2: ('servo', 2, sp.SERVO_MAX_US, sp.SERVO_MIN_US),   # torpedo_2 -> PCA servo ch 1
    3: ('servo', 3, sp.SERVO_MAX_US, sp.SERVO_MIN_US),   # dropper_1 -> PCA servo ch 2
    4: ('servo', 4, sp.SERVO_MAX_US, sp.SERVO_MIN_US),   # dropper_2 -> PCA servo ch 3
}


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
                self._log.warning(
                    f'[PAYLOAD] SROT: channel {channel} not mapped -- set _FIRE_MAP '
                    f'from the real PCA9685 wiring; no actuation')
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

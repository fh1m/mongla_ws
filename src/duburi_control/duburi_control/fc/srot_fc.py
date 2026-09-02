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
        with self._tx_lock:
            self.master.mav.command_long_send(
                sp.VEHICLE_SYSID, sp.VEHICLE_COMPID, command, confirmation,
                float(p1), float(p2), float(p3), float(p4),
                float(p5), float(p6), float(p7))

    def _cache(self, msgtype):
        return self.master.messages.get(msgtype)

    def _clear_ack(self):
        self.master.messages.pop('COMMAND_ACK', None)

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
        return self._arm_disarm(True, timeout, abort)

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
        kill = self._named_value('KILL')
        if kill is not None:
            t.kill_switch = kill >= 0.5
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
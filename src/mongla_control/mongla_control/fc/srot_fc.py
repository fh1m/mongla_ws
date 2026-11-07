#!/usr/bin/env python3
"""SrotFC -- the host side of the SROT board (firmware Hengla).

SROT speaks MAVLink 2 (sysid/compid 1/1) and owns the motion primitives on-board:
attitude + depth hold, thrust allocation, timed moves with braking, heading hold,
failsafes. So this backend is thin -- it sends **intent** and relays the board's
own progress/terminal ACKs. The wire constants all live in `srot_protocol.py`.

THREADING RULE: only the manager's reader thread calls `recv_match()`; every
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

⚠ The board's depth loop has never run closed (AUDIT R1) -- DIVE/DEPTH_HOLD are
bench-unverified. The facade gates dive-dependent verbs; this layer just sends them.

WHERE EACH PART LIVES. `SrotFC` is one class assembled from one file per concern,
so a bug report names a file:

  srot_fc.py         the link, arm/disarm, modes, the SROT_MOVE verbs, manual()
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
import math
import threading
import time

from ._srot_base import mavutil, _ARMED_FLAG, _POLL_S, _LINK_STALE_S, _rx_age, _finite
from .srot_bench import SrotBench
from .srot_health import SrotHealth
from .srot_params import SrotParams
from .srot_telemetry import SrotTelemetry
from .srot_uplink import SrotUplink

from .base import (MoveResult, SUCCEEDED, PREEMPTED, FAILED, DENIED, TIMEOUT, ABORTED)
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
_STATUSTEXT_RING = 64      # a boot burst is ~13 lines; this holds several

# How long a SUCCEEDED move waits for a HEARTBEAT newer than its ACK before
# judging the board's mode. The manager requests HEARTBEAT at 10 Hz
# (SROT_MESSAGE_RATES), so one normally lands inside 0.1 s.
_POST_ACK_HB_WAIT_S = 0.3

# How long an aborted or stalled move waits to consume the ACKs its STOP sets in
# flight (issue #10). The stream task answers the displacement on its next tick
# and the STOP brakes in well under a second on the bench; 1.0 s bounds an abort
# that the board never answers, which is then logged, not hung on.
MOVE_ACK_DRAIN_S = 1.0

# ── the host-side motion envelope (issue #15) ────────────────────────────────
# Each bound catches a UNITS BUG or a TYPO, never a real request -- the reason is
# written beside it, and none is a tuning value.
#
# Deepest venue in the campaign is TRANSDEC at ~5 m; SAUVC is 1.2-1.6 m. 10 m
# restricts no real pool and refuses -50 (cm typed as m). The hull's own
# pressure rating is NOT measured, so this is not a crush-depth claim.
MAX_DEPTH_M = 10.0
# A turn of more than a full revolution is a units bug (3600 = deg x 10).
MAX_TURN_DEG = 360.0
# A single leg longer than ten minutes is a typo in a run that lasts ~15; a
# negative one (-30 for -3.0) has no meaning on the board, which is unverified.
MAX_DURATION_S = 600.0

_ACK_MARGIN_S     = 5.0    # slack over the expected leg time before calling it a stall
_ACK_MIN_BUDGET_S = 8.0    # floor, so a 0.5 s leg still tolerates a slow first ACK
_STYLE_ROLL_S     = 360.0 / 90.0   # MOVE_STYLE is always a roll at 90 deg/s


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


# Saturation is refused whatever its cause, because SATURATION ITSELF is the hazard:
# a pinned heave demand becomes full vertical thrust the instant the outputs go live
# (mixer throttle column is -1 on all four verticals, 0 on all four horizontals). But
# the cause changes what the operator should DO, and a large zero offset is far more
# common than a dead sensor -- so name the cheap fix first instead of sending someone
# to look for a hardware fault.
_DEPTH_ARM_TAIL = (
    'Arming would command FULL vertical thrust (the mixer throttle column is -1 on all '
    'four verticals) with the horizontals idle. If the barometer VARIANCE is healthy '
    'this is a large zero offset, not a dead sensor -- run `mongla calibrate_depth`, '
    'then re-check with `ros2 run mongla_manager connect`')


class SrotFC(SrotHealth, SrotTelemetry, SrotBench, SrotUplink, SrotParams):
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

    def check_behaviour_rev(self, for_arm: bool = False):
        """(ok, reason). Fail-closed on a KNOWN-too-old board, and -- when
        ``for_arm`` -- on a board that did not answer at all.

        At connect a silent board only warns: there "board said nothing" is more
        likely a dropped frame than a rev-1 board, and refusing to even talk to
        the vehicle on a comms hiccup is its own hazard.

        ⛔ AT ARM IT REFUSES. That asymmetry was decided when the floor was rev 2
        and the cost of an old board was a `stop` that coasts. The floor is now
        rev 10, because below it the board's yaw sense is INVERTED: every turn
        goes the wrong way and any loop closed on yaw -- the vision verbs --
        diverges, while roll and pitch read correctly so nothing looks wrong.
        `read_behaviour_rev` has already asked three times over six seconds, so
        "no answer" means firmware without REQUEST_MESSAGE (older than the floor)
        or a link too poor to arm on. `allow_fw_behaviour_mismatch:=true` still
        overrides, loudly. Reversed on the cloud audit's finding, 2026-11-07.
        """
        rev = self.read_behaviour_rev()
        need = sp.FW_BEHAVIOUR_REV_REQUIRED
        if rev is None:
            what = (f'could not read SROT_FW_BEHAVIOUR_REV from the board '
                    f'(AUTOPILOT_VERSION unanswered). Host expects >= {need}; below '
                    f'rev 10 the board turns every yaw backwards')
            if not for_arm:
                self._log_warn(f'[SROT ] !! {what}. Connecting anyway; arm will refuse.')
                return True, 'FW_BEHAVIOUR_REV_UNKNOWN'
            if self.allow_fw_behaviour_mismatch:
                self._log_warn(f'[SROT ] !! OVERRIDDEN: {what}.')
                return True, 'FW_BEHAVIOUR_REV_UNKNOWN_OVERRIDDEN'
            self._log_warn(f'[SROT ] !! REFUSING TO ARM: {what}. Set '
                           f'allow_fw_behaviour_mismatch:=true to arm anyway.')
            return False, f'FW_BEHAVIOUR_REV_UNKNOWN: {what}'
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
        sent = time.monotonic()
        self._command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            p1=float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            p2=float(target))
        deadline = sent + timeout
        while time.monotonic() < deadline:
            hb = self._vehicle_hb()
            # ⛔ ONLY A HEARTBEAT HEARD SINCE THE COMMAND. pymavlink keeps the last
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
        #
        # A NaN axis is SENT as 0, so the demand in force on the board is 0 and is
        # recorded as such -- the board acts on the wire, not on the request.
        # (A cloud audit read this as "an unknown recorded as known"; it is not:
        # `test_demand_is_what_was_SENT_after_clamping` pins it, 2026-11-07.)
        self._demand = (x / sp.MC_AXIS_MAX, y / sp.MC_AXIS_MAX, time.monotonic())

    # A MANUAL_CONTROL frame older than this is not a demand in force: the
    # board's own pilot-input timeout is longer, but a writer that stopped
    # streaming has stopped commanding, and a stale demand would be modelled
    # as a live one.
    DEMAND_FRESH_S = 0.25

    # The board's own echo of what its mixer was handed (DEM_FWD / DEM_LAT,
    # srot-control-board PR #33). It rides the NAMED_VALUE_FLOAT block, whose
    # default period is 500 ms, so two periods is the oldest echo still in force.
    DEMAND_ECHO_FRESH_S = 1.0

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
        # ⛔ A DISARMED BOARD DRIVES NO THRUSTER, so its demand is KNOWN: zero.
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

    def _cut_short_by_mode_change(self, t_ack: float):
        """Why a move ACKed ACCEPTED,100 did not really complete -- or None.

        ⛔ THE BOARD REPORTS A CANCELLED MOVE AS DONE. When a failsafe (leak,
        battery, companion loss), a baro loss or a disarm displaces AUTO, the
        running move is cancelled and then latched as finished, so the terminal
        ACK is ACCEPTED at 100 %. Taken at its word, the mission advances to its
        next leg -- which (without the SURFACE refusal in `move`) would pull the
        board straight back out of the failsafe. A move only completed if the
        board is still ARMED and in AUTO afterwards: a completed move never
        leaves AUTO (it never exits by itself).

        Judged on a HEARTBEAT newer than the ACK, waited for briefly; with no
        vehicle heartbeat at all there is nothing to judge and the ACK stands.

        ⛔ AN OLDER HEARTBEAT IS NOT EVIDENCE. If none newer than the ACK lands
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

        ⛔ THE NEXT MOVE USED TO READ THEM AS ITS OWN. A STOP displaces the
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
        # ⚠ AN ABSOLUTE TURN IS ONLY A COMPASS HEADING WITH A LOCKED YAW
        # REFERENCE (#15). Checked HERE, not once at startup: YAW_REF is a live
        # state machine that restarts at SAMPLING after every board reboot.
        # WARNED, NOT REFUSED, deliberately: the search sweeps call
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

        # ⛔ NEVER SEND A SROT_MOVE INTO SURFACE. The firmware's SROT_MOVE handler
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


# ---------------------------------------------------------------------- #
#  The verb table -- mongla Move verb -> SROT_MOVE (p1..p5).             #
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
    """mongla set_depth target is NEGATIVE metres (below surface); SROT DIVE p2 is a
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
    if target < -MAX_DEPTH_M:
        raise ValueError(f'set_depth target {target} m is deeper than the '
                         f'{MAX_DEPTH_M:.0f} m envelope -- a units bug (cm as m?), '
                         f'not a pool (#15)')
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


def _bounded_time(name: str, value) -> float:
    """Finite, not negative, not past MAX_DURATION_S (issue #15)."""
    v = _finite_param(name, value)
    if v < 0.0:
        raise ValueError(f'negative {name} {v} s -- a typo (-30 for -3.0?); the '
                         f'board was never verified on a negative {name} (#15)')
    if v > MAX_DURATION_S:
        raise ValueError(f'{name} {v:.0f} s is past the {MAX_DURATION_S:.0f} s '
                         f'envelope for one leg -- a typo, not a manoeuvre (#15)')
    return v


def _bounded_turn(kw) -> float:
    """A turn target within one revolution (issue #15)."""
    v = _finite_param('target', kw.get('target', 0.0) or 0.0)
    if abs(v) > MAX_TURN_DEG:
        raise ValueError(f'turn target {v} deg is more than one revolution -- '
                         f'a units bug (deg x 10?), not a heading (#15)')
    return v


def _build_params(verb: str, kw: dict):
    """mongla verb + Move.Goal-ish kwargs -> (p1, p2, p3, p4, p5). Raises KeyError
    for an unmapped verb, ValueError for a bad parameter. This is THE verb table.

    Every returned tuple is checked finite before it leaves (B35): a NaN reached
    the wire from `duration`, `target` or `gain`, and NaN has a live source --
    `VisionResult.x_px` is NaN when the target was never seen."""
    # Validated at the boundary, so no per-verb branch below can forget it.
    # `speed` needs no check: sanitize_speed maps NaN to 0.0 and clamps inf.
    dur   = _bounded_time('duration', kw.get('duration', 0.0) or 0.0)
    tmo   = _bounded_time('timeout',  kw.get('timeout', 0.0) or 0.0)
    speed = _speed_from_gain(kw)
    if verb == 'move_forward':
        return (sp.MOVE_FORWARD, dur, speed, 0.0, tmo)
    if verb == 'move_back':
        return (sp.MOVE_BACK, dur, speed, 0.0, tmo)
    if verb == 'move_left':
        return (sp.MOVE_STRAFE_L, dur, speed, 0.0, tmo)
    if verb == 'move_right':
        return (sp.MOVE_STRAFE_R, dur, speed, 0.0, tmo)
    if verb == 'yaw_left':
        return (sp.MOVE_TURN, -abs(_bounded_turn(kw)), 0.0,
                float(sp.TURN_RELATIVE), tmo)
    if verb == 'yaw_right':
        return (sp.MOVE_TURN, abs(_bounded_turn(kw)), 0.0,
                float(sp.TURN_RELATIVE), tmo)
    if verb == 'turn':
        # mongla 'turn' is an ABSOLUTE heading -> needs MAG_YAW_REF=1 on the board.
        return (sp.MOVE_TURN, _bounded_turn(kw), 0.0,
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


# The verbs the board runs as one SROT_MOVE. Every other verb is host-side:
# arm/disarm/set_mode, fire, motor_test and the two vision verbs (which stream
# MANUAL_CONTROL in STABILIZE, never entering AUTO).
MOVE_VERBS = frozenset({
    'move_forward', 'move_back', 'move_left', 'move_right',
    'yaw_left', 'yaw_right', 'turn', 'set_depth', 'stop', 'pause', 'style_roll',
})

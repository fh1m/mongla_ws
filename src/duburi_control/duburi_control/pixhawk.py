#!/usr/bin/env python3
"""Raw MAVLink layer to the Pixhawk autopilot.

This is the only file in the workspace that imports pymavlink. Everything
else in `duburi_control` and `duburi_manager` talks to the autopilot
through a `Pixhawk` instance.

THREADING RULE: Only the reader thread (in `auv_manager_node`) calls
`recv_match()`. Every method here reads from `master.messages` cache
only. This prevents message-consumption races between the reader
thread and the action / timer worker threads.

DEBUG-LEVEL MAVLINK TRACE: Every public ``send_*`` / ``set_*`` /
``arm`` / ``disarm`` method emits a single ``[MAV ] ...`` line at
DEBUG level when a logger is attached. Format is one line per
outbound MAVLink message, kept deliberately compact:

    [MAV <fn>]              <body>           # tracing off
    [MAV <fn> cmd=<verb>]   <body>           # tracing on (debug:=true)

``<fn>`` is the Pixhawk method that called ``_log_mavlink`` (one
Python frame above), so the operator immediately sees which
``send_rc_override`` / ``set_target_depth`` / ``arm`` produced the
frame -- no per-callsite bookkeeping. ``<body>`` skips channels
that are at neutral / released so a typical line stays short:

    [MAV send_rc_override cmd=lock_heading] yaw=1430
    [MAV send_rc_override cmd=stop]         all=neutral
    [MAV release_rc_override cmd=pause]     all=released
    [MAV set_target_depth cmd=set_depth]    depth=-0.40m

Activate with ``ros2 run duburi_manager start --ros-args -p
debug:=true`` or set the ``debug`` ROS-param at runtime. Tests using
FakePixhawk keep working because ``log`` defaults to None (debug
calls no-op).
"""

import os
import math
import sys
import threading
import time

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil                      # noqa: E402

from . import tracing                              # noqa: E402


# ArduSub channel layout (zero-indexed within the 18-slot RC override array).
CH_PITCH    = 0
CH_ROLL     = 1
CH_THROTTLE = 2
CH_YAW      = 3
CH_FORWARD  = 4
CH_LATERAL  = 5
NO_OVERRIDE = 65535        # MAVLink "ignore this channel" sentinel

# A cached autopilot HEARTBEAT older than this means the link is DEAD -- ArduSub
# streams HEARTBEAT at ~1 Hz, so 3 s = 3 missed frames (well past normal jitter).
# is_armed()/get_mode() report "no link" past this instead of a stale
# "armed/ALT_HOLD" that would let command preconditions pass on a dead link.
_LINK_STALE_S = 3.0

# Human-readable names for COMMAND_ACK.result values. Anything that isn't
# ACCEPTED is surfaced straight to the action server's result.message so
# the operator sees *why* a command failed, not just that it did.
MAV_RESULT = {
    0: 'ACCEPTED',
    1: 'TEMP_REJECTED',
    2: 'DENIED',
    3: 'UNSUPPORTED',
    4: 'FAILED',
    5: 'IN_PROGRESS',
    6: 'CANCELLED',
}


def _euler_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> list[float]:
    """ZYX Euler (NED, degrees) → quaternion [w, x, y, z] for ATT_POS_MOCAP."""
    r = math.radians(roll_deg) / 2
    p = math.radians(pitch_deg) / 2
    y = math.radians(yaw_deg) / 2
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        cr*cp*cy + sr*sp*sy,   # w
        sr*cp*cy - cr*sp*sy,   # x
        cr*sp*cy + sr*cp*sy,   # y
        cr*cp*sy - sr*sp*cy,   # z
    ]


class Pixhawk:
    """Thin, well-named wrapper around `pymavlink.mavutil`."""

    def __init__(self, master, log=None):
        self.master = master
        self._boot_time = time.time()
        # MAVLink WRITE serialization. pymavlink's MAVLink object shares a single
        # sequence counter + encode buffer across threads; five writers hit it
        # concurrently -- HeadingLock (50 Hz), Heartbeat (5 Hz), the action
        # thread, and the manager's _mocap_tick (20 Hz) + heartbeat_tick. Over
        # serial (desk mode) unserialized writes tear frames / duplicate seq
        # numbers. Every outbound send goes through _tx() under this lock (the
        # symmetric write rule to the existing "only the reader thread calls
        # recv_match"). It is a leaf lock -- held only around the pymavlink call,
        # never around anything that could re-enter -- so it cannot deadlock.
        self._tx_lock = threading.Lock()
        # Optional logger -- when provided, every outbound MAVLink frame
        # logs one DEBUG line via _log_mavlink() so missions are debuggable
        # frame-by-frame without a wrapper layer. Tests pass log=None
        # (the default) so FakePixhawk fixtures stay zero-config.
        self._log = log
        # Last HEARTBEAT we saw whose `autopilot` field was NOT
        # MAV_AUTOPILOT_INVALID -- i.e. the real autopilot, not our own
        # heartbeat looping back via BlueOS / mavproxy. get_mode() and
        # is_armed() prefer this so they never report the GCS's state
        # when the loop-back arrives between autopilot frames.
        self._last_autopilot_hb = None

    def _log_mavlink(self, body):
        """Emit one compact ``[MAV <fn>[ cmd=<verb>]] <body>`` DEBUG line.

        ``<fn>`` is the Pixhawk method that called us (one Python frame
        above), so the operator immediately sees which ``send_rc_override``
        / ``set_target_depth`` / ``arm`` produced the frame -- with zero
        per-callsite plumbing. The ``cmd=<verb>`` half comes from
        ``tracing.current_tag()`` (a contextvar the Duburi facade opens
        at the top of every public verb), so when ``debug:=true`` is set
        on the manager, every frame is attributed to the high-level
        command that caused it.

        We deliberately drop the source filename: this method is the
        ONLY caller of ``_log_mavlink`` outside its own file, and a
        bare ``send_rc_override`` is shorter than ``pixhawk.py:send_rc_override``
        in the terminal. If ``_log_mavlink`` ever moves out of pixhawk.py
        the function name alone still pinpoints the callsite via
        ``rg "def send_rc_override"``.

        Cheap no-op when ``self._log`` is None (FakePixhawk in tests
        and any non-manager caller). ``sys._getframe(1)`` is one C
        call per emit; the attribute-None branch above means we only
        pay it when DEBUG is actually wired up.
        """
        if self._log is None:
            return
        func = sys._getframe(1).f_code.co_name
        tag  = tracing.current_tag()
        prefix = f'[MAV {func} {tag}]' if tag else f'[MAV {func}]'
        self._log.debug(f'{prefix} {body}')

    @staticmethod
    def _summarise_rc(values):
        """Compact ``ch=PWM`` summary of an 18-slot RC override array.

        Skips channels that hold ``NO_OVERRIDE`` (released) or 1500
        (neutral) so a typical "yaw correction only" frame logs as
        ``yaw=1430`` instead of six redundant ``ch=1500`` tokens.
        Returns ``'all=released'`` when every driving channel is
        65535 and ``'all=neutral'`` when every driving channel is
        1500 -- both are common enough idle states that naming them
        explicitly is friendlier than an empty body.
        """
        labels = (
            ('pitch', CH_PITCH),
            ('roll',  CH_ROLL),
            ('thr',   CH_THROTTLE),
            ('yaw',   CH_YAW),
            ('fwd',   CH_FORWARD),
            ('lat',   CH_LATERAL),
        )
        parts = [f'{name}={values[idx]}' for name, idx in labels
                 if values[idx] not in (1500, NO_OVERRIDE)]
        if parts:
            return ' '.join(parts)
        if all(values[idx] == NO_OVERRIDE for _, idx in labels):
            return 'all=released'
        return 'all=neutral'

    # ------------------------------------------------------------------ #
    #  COMMAND_ACK — fast, explicit failure reporting                     #
    # ------------------------------------------------------------------ #

    def clear_ack(self):
        """Drop any cached COMMAND_ACK so wait_ack only sees fresh replies."""
        self.master.messages.pop('COMMAND_ACK', None)

    def wait_ack(self, command_id, timeout=3.0):
        """Poll for a COMMAND_ACK matching `command_id`.

        Returns `(accepted, reason_name)` — `accepted` is True only for
        MAV_RESULT_ACCEPTED (0). On timeout returns `(False, 'NO_ACK')`.
        Must be paired with `clear_ack()` before the command, otherwise
        a stale ACK from the previous command can be returned.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ack = self.master.messages.get('COMMAND_ACK')
            if ack is not None and ack.command == command_id:
                name = MAV_RESULT.get(ack.result, f'RESULT_{ack.result}')
                return ack.result == 0, name
            time.sleep(0.05)
        return False, 'NO_ACK'

    # ------------------------------------------------------------------ #
    #  Arm / Disarm — ACK for rejection, heartbeat poll for completion    #
    # ------------------------------------------------------------------ #

    # Max heartbeat age (s) for a post-abort disarm confirmation to be trusted.
    # Beyond this the link is stale and is_armed() is a cached value -> never
    # confirm 'disarmed' on it (fail-closed under link loss).
    _DISARM_CONFIRM_MAX_HB_AGE_S = 2.0

    def arm(self, timeout=15.0, abort=None):
        """Returns `(success, reason)`. Reason is a MAV_RESULT name,
        'NO_ACK', 'ABORTED', or 'NOT_ARMED_AFTER_ACK[: <pre-arm reason>]'.

        ArduSub ACKs the command before the arm actually completes
        (pre-arm checks run in parallel), so we still poll `is_armed()`.

        `abort` is an optional ``() -> bool`` cancel hook. The arm command is
        already in flight once sent, so on abort mid-poll we DISARM before
        returning -- a bare return could leave a hull that arms ~1 s after the
        caller already gave up (armed-hardware / believed-failed divergence).
        On timeout we attach ArduSub's latest STATUSTEXT (the pre-arm reason),
        so a rejected arm is a diagnosis instead of an opaque 'NOT_ARMED'.
        """
        cmd = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        self.clear_ack()
        # Snapshot the current STATUSTEXT so a timeout only reports a pre-arm
        # reason that actually arrived DURING this arm -- not a stale, unrelated
        # line left in the cache from an earlier mode change / EKF / battery event.
        prev_status = self.master.messages.get('STATUSTEXT')
        self._log_mavlink('COMPONENT_ARM_DISARM p1=1')
        with self._tx_lock:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                cmd, 0, 1, 0, 0, 0, 0, 0, 0)

        accepted, reason = self.wait_ack(cmd, timeout=3.0)
        if not accepted:
            return False, reason

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if abort is not None and abort():
                return self._disarm_after_abort()
            if self.is_armed():
                return True, 'ACCEPTED'
            time.sleep(0.1)
        why = self._fresh_statustext(prev_status)
        return False, 'NOT_ARMED_AFTER_ACK' + (f': {why}' if why else '')

    def _fresh_statustext(self, prev):
        """The STATUSTEXT only if a NEW one landed since `prev` was snapshotted --
        so an arm timeout never misattributes a stale/unrelated cached line as the
        pre-arm reason. Returns the fresh text, or None if nothing new arrived."""
        msg = self.master.messages.get('STATUSTEXT')
        if msg is None or msg is prev:
            return None
        return (msg.text or '').strip() or None

    def _disarm_after_abort(self, tries=6, settle=0.5):
        """Verified disarm after an abort mid-arm.

        The arm command is already in flight and ArduSub runs its pre-arm checks
        AFTER the ACK, so `is_armed()` can read False simply because the arm
        hasn't completed yet -- a single fire-and-forget disarm (or a lone
        is_armed()==False) would let the hull arm a beat later while the caller
        believes it aborted (fail-OPEN state drift). So we re-send DISARM across a
        short window and require the disarmed state to HOLD before reporting
        'ABORTED'. Fail-CLOSED: if we can't confirm disarmed, return a DISTINCT
        reason so no caller assumes 'safe' on an unverified state (the manager's
        cancel path + mission-runner _safe_shutdown then disarm again).
        """
        cmd = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        stable = 0
        for _ in range(tries):
            with self._tx_lock:
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    cmd, 0, 0, 0, 0, 0, 0, 0, 0)   # p1=0 -> disarm
            time.sleep(settle)
            age = self.heartbeat_age()
            fresh = age is not None and age <= self._DISARM_CONFIRM_MAX_HB_AGE_S
            if self.is_armed() or not fresh:
                # armed (possibly late), OR the link is stale so is_armed() is a
                # cached value we can't trust -> keep disarming, never confirm on
                # stale telemetry (that would be a fail-OPEN under link loss).
                stable = 0
            else:
                stable += 1
                if stable >= 2:            # two consecutive FRESH disarmed reads -> settled
                    return False, 'ABORTED'
        return False, 'ABORTED_DISARM_UNCONFIRMED'

    def disarm(self, timeout=15.0):
        """Swap to MANUAL and neutralise thrusters before disarming —
        what QGC does, avoids ArduSub's "still moving" disarm rejection.
        """
        self.set_mode('MANUAL')
        time.sleep(3)
        self.send_neutral()

        cmd = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        self.clear_ack()
        self._log_mavlink('COMPONENT_ARM_DISARM p1=0')
        with self._tx_lock:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                cmd, 0, 0, 0, 0, 0, 0, 0, 0)

        accepted, reason = self.wait_ack(cmd, timeout=3.0)
        if not accepted:
            return False, reason

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not self.is_armed():
                return True, 'ACCEPTED'
            time.sleep(0.1)
        return False, 'STILL_ARMED_AFTER_ACK'

    # ------------------------------------------------------------------ #
    #  Mode — SET_MODE is a legacy message with no COMMAND_ACK, so we     #
    #  retry the send and poll the heartbeat cache for the actual change. #
    # ------------------------------------------------------------------ #

    def set_mode(self, mode_name, timeout=8.0):
        """Returns `(success, reason)`. Reason is 'ACCEPTED' or 'MODE_NOT_REACHED'."""
        mode_id = self.master.mode_mapping().get(mode_name)
        if mode_id is None:
            return False, f'UNKNOWN_MODE:{mode_name}'
        self._log_mavlink(f'{mode_name} (id={mode_id})')
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._tx_lock:
                self.master.mav.set_mode_send(
                    self.master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id)
            time.sleep(0.3)
            if self.get_mode() == mode_name:
                return True, 'ACCEPTED'
        return False, 'MODE_NOT_REACHED'

    def get_param(self, name: str, timeout: float = 2.0) -> float | None:
        """Read a named ArduSub parameter. Returns None on timeout.

        Polls master.messages (filled by the dedicated reader thread at 200 Hz)
        instead of calling recv_match() — which is reserved for reader_loop only.
        Clears the PARAM_VALUE cache before the request to avoid stale hits.
        """
        self.master.messages.pop('PARAM_VALUE', None)
        self.master.param_fetch_one(name)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.master.messages.get('PARAM_VALUE')
            if msg and msg.param_id.rstrip('\x00') == name:
                return float(msg.param_value)
            time.sleep(0.05)   # 20 Hz poll; reader thread fills cache at 200 Hz
        return None

    def set_param(self, name: str, value: float, timeout: float = 3.0) -> bool:
        """Write a named ArduSub parameter via PARAM_SET and wait for value-echo ACK.

        Returns True when ArduSub echoes the new value back in PARAM_VALUE.
        Polls master.messages (reader thread) — never calls recv_match() directly.
        Used by style verbs to temporarily zero ACRO_BAL_ROLL / ACRO_TRAINER
        before a free-rotation maneuver and restore them after.
        """
        self.master.messages.pop('PARAM_VALUE', None)
        with self._tx_lock:
            self.master.param_set_send(name, float(value))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.master.messages.get('PARAM_VALUE')
            if (msg and msg.param_id.rstrip('\x00') == name
                    and abs(msg.param_value - value) < 0.01):  # value-echo ACK
                return True
            time.sleep(0.05)
        return False

    # ------------------------------------------------------------------ #
    #  RC Override                                                         #
    # ------------------------------------------------------------------ #

    def send_rc_override(self, pitch=1500, roll=1500, throttle=1500,
                         yaw=1500, forward=1500, lateral=1500):
        """Override the six driving channels with explicit PWM values."""
        values = [NO_OVERRIDE] * 18
        values[CH_PITCH]    = int(pitch)
        values[CH_ROLL]     = int(roll)
        values[CH_THROTTLE] = int(throttle)
        values[CH_YAW]      = int(yaw)
        values[CH_FORWARD]  = int(forward)
        values[CH_LATERAL]  = int(lateral)
        self._log_mavlink(self._summarise_rc(values))
        with self._tx_lock:
            self.master.mav.rc_channels_override_send(
                self.master.target_system, self.master.target_component, *values)

    def send_rc_translation(self, throttle=1500, forward=1500, lateral=1500):
        """Override translation channels only (Ch3 throttle, Ch5 forward, Ch6 lateral).

        Leaves pitch/roll/yaw RC channels released (65535) so the
        background HeadingLock thread keeps sole authority over Ch4
        (yaw rate). If Ch4 were overridden here -- even to neutral
        1500 us -- the lock thread's rate-override would arrive in
        the same slot a moment later and the two writers would race.
        """
        values = [NO_OVERRIDE] * 18
        values[CH_THROTTLE] = int(throttle)
        values[CH_FORWARD]  = int(forward)
        values[CH_LATERAL]  = int(lateral)
        self._log_mavlink(self._summarise_rc(values))
        with self._tx_lock:
            self.master.mav.rc_channels_override_send(
                self.master.target_system, self.master.target_component, *values)

    def send_rc_yaw_only(self, yaw: int) -> None:
        """Override Ch4 (yaw rate) only, leaving all other channels at NO_OVERRIDE.

        Used by HeadingLock so its 20 Hz Ch4 stream does NOT clobber
        Ch5/Ch6 when a concurrent translation command (move_forward_dist,
        move_lateral_dist, or any timed forward/lateral move) is running.
        Writing Ch5=1500 from the heading-lock tick would interrupt the
        DVL forward thrust every other frame -- this method avoids that
        by touching only the one channel it owns.
        """
        values = [NO_OVERRIDE] * 18
        values[CH_YAW] = int(yaw)
        self._log_mavlink(self._summarise_rc(values))
        with self._tx_lock:
            self.master.mav.rc_channels_override_send(
                self.master.target_system, self.master.target_component, *values)

    def send_neutral(self):
        """Active hold: send 1500 PWM to all six driving channels.

        The autopilot still sees us as "the pilot", so its onboard heading
        and depth holds latch at the current state.
        """
        self.send_rc_override(1500, 1500, 1500, 1500, 1500, 1500)

    def release_rc_override(self):
        """Release: send 65535 to every channel so the autopilot runs
        WITHOUT us on the loop.

        Useful for letting ALT_HOLD / POSHOLD take over fully between
        commands, or for A/B testing what ArduSub does on its own. This
        is the "pause" verb's MAVLink behaviour.
        """
        values = [NO_OVERRIDE] * 18
        self._log_mavlink('all=released')
        with self._tx_lock:
            self.master.mav.rc_channels_override_send(
                self.master.target_system, self.master.target_component, *values)

    # ------------------------------------------------------------------ #
    #  Heartbeat (mandatory >= 1 Hz)                                      #
    # ------------------------------------------------------------------ #

    def send_heartbeat(self):
        with self._tx_lock:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0)

    # ------------------------------------------------------------------ #
    #  Setpoints                                                           #
    # ------------------------------------------------------------------ #

    def set_target_depth(self, depth_m):
        """Command ArduSub's onboard depth controller to an absolute depth.

        `depth_m` is negative below the surface (matches AHRS2.altitude
        used everywhere else in the stack: -0.5 = 50 cm deep).

        Sends SET_POSITION_TARGET_GLOBAL_INT with every field except
        the altitude masked out, so ArduSub's onboard depth PID
        (running at 400 Hz) is the sole consumer. Requires
        ALT_HOLD / POSHOLD / GUIDED -- in MANUAL or STABILIZE the
        autopilot silently drops the setpoint. Used by
        ``motion_depth.hold_depth`` to drive to a new target; once
        reached, ALT_HOLD keeps holding without further refreshes.

        Reference: Blue Robotics pymavlink docs, "Set Target Depth/Attitude".
        """
        mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        self._log_mavlink(f'depth={float(depth_m):+.2f}m')
        with self._tx_lock:
            self.master.mav.set_position_target_global_int_send(
                int(1e3 * (time.time() - self._boot_time)),
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mask,
                0, 0,                       # lat_int, lon_int (ignored)
                float(depth_m),             # alt = target depth in metres
                0, 0, 0,                    # vx, vy, vz   (ignored)
                0, 0, 0,                    # afx, afy, afz (ignored)
                0, 0)                       # yaw, yaw_rate (ignored)

    def send_att_pos_mocap(self, yaw_deg: float) -> None:
        """Inject BNO085 yaw into ArduSub EKF3 via ATT_POS_MOCAP (MAVLink 138).

        ArduSub EKF3 only consumes yaw from this message (EK3_SRC1_YAW=6).
        Roll/pitch are zeroed — EKF3 always uses onboard IMU for those axes.
        covariance[0]=NaN signals ArduSub to ignore the position fields.
        Call at ~20 Hz when yaw_source is BNO-based.
        """
        q = _euler_to_quat(0.0, 0.0, yaw_deg)
        nan = float('nan')
        with self._tx_lock:
            self.master.mav.att_pos_mocap_send(
                time_usec=int(time.monotonic() * 1e6),
                q=q,
                x=0.0, y=0.0, z=0.0,
                covariance=[nan] * 21,
            )
        self._log_mavlink(f'att_pos_mocap  yaw={yaw_deg:.1f}°')

    # ------------------------------------------------------------------ #
    #  Stream rate control (MAV_CMD_SET_MESSAGE_INTERVAL)                  #
    # ------------------------------------------------------------------ #

    def set_message_rate(self, message_id, hz):
        """Pin the streaming rate for a given MAVLink message id.

        Without this, ArduSub picks a default rate (often 4 Hz for
        AHRS2) which silently caps how tight our control loops can be.
        Pass `hz=0` to stop the stream, `hz=-1` to reset to default.

        Reference: MAVLink common/MAV_CMD_SET_MESSAGE_INTERVAL (id 511).
        """
        interval_us = int(1_000_000 / hz) if hz > 0 else int(hz)
        self._log_mavlink(f'msg_id={message_id} hz={hz}')
        with self._tx_lock:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, message_id, interval_us, 0, 0, 0, 0, 0)

    # ------------------------------------------------------------------ #
    #  Barometer ground-pressure calibration (depth re-zero)              #
    # ------------------------------------------------------------------ #

    def calibrate_barometer(self, timeout=6.0):
        """Re-zero the barometer's ground pressure -- QGC's "Calibrate Pressure".

        Sends MAV_CMD_PREFLIGHT_CALIBRATION with param3=1 (ground pressure). On
        ArduSub this routes to `_handle_command_preflight_calibration_baro` ->
        `AP::baro().calibrate(true)`, which averages ~5 samples over ~1.5 s, resets
        the ground reference + alt offset to 0, and only then ACKs. So the ACK
        arrives AFTER the ~1.5 s blocking calibrate -- `timeout` must exceed it.
        Requires the vehicle DISARMED (ArduSub rejects it armed); the caller gates
        that. Returns `(ok, reason)` like `arm()`.

        Fixes the pre-dive depth drift: `depth` is `AHRS2.altitude` (baro-derived),
        so a stale ground reference makes the surface read non-zero.
        """
        cmd = mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION
        self.clear_ack()
        self._log_mavlink('PREFLIGHT_CALIBRATION p3=1 (baro ground pressure)')
        with self._tx_lock:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                cmd, 0,
                0, 0, 1, 0, 0, 0, 0)   # param3=1 = ground pressure / baro

        # IN_PROGRESS-tolerant wait: ArduSub ACKs ACCEPTED once the blocking
        # calibrate() returns, but treat an interim IN_PROGRESS (5) as keep-waiting
        # rather than a failure (wait_ack would return early on any non-zero result).
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ack = self.master.messages.get('COMMAND_ACK')
            if ack is not None and ack.command == cmd:
                if ack.result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                    time.sleep(0.1)
                    continue
                name = MAV_RESULT.get(ack.result, f'RESULT_{ack.result}')
                return ack.result == 0, name
            time.sleep(0.05)
        return False, 'NO_ACK'

    # ------------------------------------------------------------------ #
    #  Telemetry reads — master.messages cache only (non-blocking)        #
    # ------------------------------------------------------------------ #

    def get_attitude(self):
        msg = self.master.messages.get('AHRS2')
        if msg is None:
            return None
        yaw_deg = math.degrees(msg.yaw)
        if yaw_deg < 0:
            yaw_deg += 360.0
        return {
            'yaw':   yaw_deg,
            'roll':  math.degrees(msg.roll),
            'pitch': math.degrees(msg.pitch),
            'depth': msg.altitude,
        }

    def get_attitude_age(self):
        """Seconds since the last AHRS2 message, or None if none arrived.

        Pymavlink stamps every received message with `_timestamp` (wall
        time). Lets sensor sources gate on freshness (see
        `MavlinkAhrsSource` and the 250 ms staleness contract in
        `.claude/context/sensors-pipeline.md`). When the attribute is
        missing we return 0.0 so the sample is treated as fresh — the
        only known cause is an in-test mock without timestamps.
        """
        msg = self.master.messages.get('AHRS2')
        if msg is None:
            return None
        ts = getattr(msg, '_timestamp', None)
        if ts is None:
            return 0.0
        return max(0.0, time.time() - ts)

    def get_angular_rates(self):
        """Body-frame angular velocity from MAVLink ATTITUDE, or None if stale/absent.

        ATTITUDE (msg 30) carries rollspeed/pitchspeed/yawspeed in rad/s -- the
        EKF-filtered gyro rate, already on the MAVLink link (no BNO/serial). Used
        by the downward optical-flow distance estimator for rotation compensation
        (the trusted gyro *rate*; ArduSub compass *yaw* stays untrusted -> yaw
        still comes from yaw_source). Returns {'roll_rate','pitch_rate','yaw_rate'}
        rad/s + 'age_s' (seconds since the sample; 0.0 for a mock without
        _timestamp). None when no ATTITUDE has arrived. Pin the stream with
        MESSAGE_RATES[ATTITUDE] or the rates arrive at ArduSub's slow default.
        """
        msg = self.master.messages.get('ATTITUDE')
        if msg is None:
            return None
        ts  = getattr(msg, '_timestamp', None)
        age = 0.0 if ts is None else max(0.0, time.time() - ts)
        # `board_ms` is the AUTOPILOT'S OWN capture time (ATTITUDE.time_boot_ms).
        # Both backends now return it so the flow pipeline can time-base on the
        # sender's clock instead of on arrival -- measured on the srot board,
        # the sender's interval has sd 0.00 ms where arrival has sd 6.67 and
        # p2p 35.12. None when absent, never 0: absence is not the boot instant.
        boot = getattr(msg, 'time_boot_ms', None)
        return {
            'roll_rate':  float(msg.rollspeed),
            'pitch_rate': float(msg.pitchspeed),
            'yaw_rate':   float(msg.yawspeed),
            'age_s':      age,
            'board_ms':   None if boot is None else int(boot),
            'host_recv_s': float(ts) if ts is not None else None,
        }

    def get_battery(self):
        msg = self.master.messages.get('BATTERY_STATUS')
        if msg is None:
            return None
        # MAVLink "unknown" sentinels: voltage cell = 0xFFFF mV, current = -1.
        # Without this guard an unknown reading surfaces as 65.5 V / -0.01 A.
        raw_mv = msg.voltages[0]
        voltage = math.nan if raw_mv == 0xFFFF else raw_mv / 1000.0
        current = math.nan if msg.current_battery == -1 else msg.current_battery / 100.0
        return {'voltage': voltage, 'current': current}

    def get_rc_channels(self):
        msg = self.master.messages.get('RC_CHANNELS')
        if msg is None:
            return None
        return [
            msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
            msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw,
        ]

    def get_statustext(self):
        msg = self.master.messages.get('STATUSTEXT')
        return msg.text.strip() if msg else None

    def _autopilot_heartbeat(self):
        """Return the latest HEARTBEAT that came from the autopilot.

        Skips frames whose `autopilot` field is MAV_AUTOPILOT_INVALID
        (those are our own heartbeats, looped back through BlueOS or
        mavproxy). Falls back to the last good autopilot heartbeat we
        cached so get_mode/is_armed never briefly report 'UNKNOWN' or
        'disarmed' just because a loop-back arrived between autopilot
        frames.
        """
        msg = self.master.messages.get('HEARTBEAT')
        if msg is None:
            return self._last_autopilot_hb
        if getattr(msg, 'autopilot', None) == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            return self._last_autopilot_hb
        self._last_autopilot_hb = msg
        return msg

    def heartbeat_age(self):
        """Seconds since the last autopilot HEARTBEAT, or None if none seen.

        Mirrors ``get_attitude_age``: a message without ``_timestamp`` (test
        mocks) is treated as fresh (0.0). Lets the manager surface link health.
        """
        msg = self._autopilot_heartbeat()
        if msg is None:
            return None
        ts = getattr(msg, '_timestamp', None)
        if ts is None:
            return 0.0
        return max(0.0, time.time() - ts)

    def link_alive(self):
        """True iff an autopilot HEARTBEAT has arrived within ``_LINK_STALE_S``.

        False = the MAVLink link is (probably) dead -- the ArduSub failsafe owns
        the thrusters, but our side must stop trusting the cached mode/arm state.
        """
        age = self.heartbeat_age()
        return age is not None and age <= _LINK_STALE_S

    def get_mode(self):
        # Uses the CACHED heartbeat, NOT the freshness-gated one: arm/mode are
        # hard per-command preconditions (duburi.py `_command_scope` /
        # `_ensure_alt_hold`). Gating them on link freshness (CTRL-2's first cut)
        # turned normal 1 Hz-heartbeat jitter into spurious 'UNKNOWN' -> mode
        # re-engage / abort -> disarm. A truly dead link is owned by ArduSub's
        # own failsafe; RC into the void is harmless. Link health is surfaced
        # separately, advisory-only, via link_alive()/heartbeat_age().
        msg = self._autopilot_heartbeat()
        if msg is None:
            return 'UNKNOWN'
        mode_map = {v: k for k, v in self.master.mode_mapping().items()}
        return mode_map.get(msg.custom_mode, str(msg.custom_mode))

    def is_armed(self):
        # Cached heartbeat (see get_mode): the arm precondition must reflect the
        # last KNOWN arm state, which only changes on OUR arm()/disarm() -- never
        # flip it to False on a transient heartbeat gap, or every motion verb
        # raises NotArmedError mid-mission and the runner disarms. link_alive()
        # is the advisory link-health signal for the manager to log.
        msg = self._autopilot_heartbeat()
        if msg is None:
            return False
        return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    # ------------------------------------------------------------------ #
    #  Pure-math helpers                                                   #
    # ------------------------------------------------------------------ #

    @staticmethod
    def percent_to_pwm(percent):
        """Convert -100..100 percent to 1100..1900 us PWM. 0 -> 1500."""
        return max(1100, min(1900, int(1500 + (percent / 100.0) * 400)))

    @staticmethod
    def heading_error(target, current):
        """Shortest-path error on 0-360 deg circle. Returns -180..180."""
        return (target - current + 540) % 360 - 180

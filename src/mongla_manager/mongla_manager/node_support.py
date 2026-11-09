"""The manager node's shared constants: one home, read by every part of the node."""
from __future__ import annotations

import math
import threading
import time

from mongla_control.fc._srot_base import mavutil
from mongla_control.fc.srot_protocol import MSG_ID_ESC_STATUS as SROT_MSG_ID_ESC_STATUS


# 2 Hz against the board's 5 s GCS failsafe (sp.GCS_FAILSAFE_MS): ten beats per
# window, so a scheduler hiccup or a slow write still leaves real margin.
HEARTBEAT_PERIOD_S = 0.5

SEPARATOR = '=' * 52

# How much a value must change before the [STATE] log line reprints.
YAW_CHANGE_THRESH   = 5.0    # degrees
DEPTH_CHANGE_THRESH = 0.08   # metres
# |DEPTH_OUT| at/above this while disarmed => arming would command full heave.
_SROT_DEPTH_OUT_WARN = 0.90
BAT_CHANGE_THRESH   = 0.2    # volts
FORCE_PRINT_SECONDS = 30.0   # always reprint even if nothing changed

# The telemetry rates requested from the board at startup (SET_MESSAGE_INTERVAL):
#   * attitude is ATTITUDE, depth is VFR_HUD
#   * no RC input at all (there is no radio on the vehicle)
#   * ESC_TELEMETRY_1_TO_4/5_TO_8 carry the RPM (ESC_STATUS msgid 291 is absent from
#     every pymavlink dialect -- see SrotFC.telemetry)
#
# Available since firmware behaviour rev 2. The board clamps any request to a 20 ms
# floor so a companion cannot starve the PARAM_VALUE / COMMAND_ACK traffic missions
# depend on, and it refuses a request to disable HEARTBEAT.
SROT_MESSAGE_RATES = {
    # The board's MODE and ARMED state reach us only on HEARTBEAT, 1 Hz by
    # default. At 1 Hz a failsafe SURFACE is invisible for up to a second --
    # long enough for the next mission leg's SROT_MOVE to put the board back in
    # AUTO. 10 Hz costs ~170 B/s (~1.5 % of the 115200 link) and lets
    # `SrotFC.move` refuse that leg and see a cut-short move for what it is.
    mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT:      10,   # Hz -- mode/armed freshness
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE:       50,   # Hz -- the host-loop ceiling
    # RAW GYRO for flow de-rotation, and it was ABSENT from this table, so it
    # ran at the board's 10 Hz default while the camera ran at 30+. Measured on
    # the board: SET_MESSAGE_INTERVAL takes it 10.0 -> 51.6 Hz with no firmware
    # change. It matters because rotational flow is `f * omega * dt` and does
    # NOT depend on range, so it must be subtracted before the translation is
    # scaled -- with a gyro sample shared across three camera frames, that
    # subtraction is using the wrong omega for two of them.
    #
    # A 100 Hz request ACKs ACCEPTED and delivers 50: the firmware's
    # RATE_MIN_MS = 20 floor. The ACK is a claim; the arrival rate is the fact.
    mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2:    50,   # Hz -- gyro, de-rotation
    mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD:        10,   # depth
    mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS:  1,
    # 291 via srot_protocol, NOT mavutil.mavlink.MAVLINK_MSG_ID_ESC_STATUS -- that
    # symbol does not exist (291 was removed from the dialect) and naming it here
    # raises AttributeError at import. One rate on 291 paces ESC_TELEMETRY_* too.
    SROT_MSG_ID_ESC_STATUS:                         5,
}


def _parse_payload_channels(spec: str, log=None) -> dict:
    """"9:torpedo_1, 11:dropper_1" -> {9: 'torpedo_1', 11: 'dropper_1'}.

    Labels only. These never decide WHERE a fire goes -- `fire(N)` is always board
    channel N -- so a malformed or stale entry is a cosmetic problem, and the right
    response is to drop it with a warning rather than refuse to start. That is the
    opposite of how the routing map this replaces had to be treated, and it is the
    point of the redesign: nothing here can misdirect an actuation.
    """
    out: dict = {}
    for entry in (spec or '').split(','):
        entry = entry.strip()
        if not entry:
            continue
        ch, sep, name = entry.partition(':')
        try:
            if not sep or not name.strip():
                raise ValueError('expected <board_channel>:<name>')
            out[int(ch)] = name.strip()
        except ValueError as exc:
            if log is not None:
                log.warn(f'[PAYLOAD] payload_channels: ignoring {entry!r} ({exc}). '
                         f'Labels only -- fire() is unaffected.')
    return out


# How long bring-up waits for the board's first HEARTBEAT before refusing. The
# board reboots on port open (port_guard.py) and is talking within ~3 s on the
# bench; 20 s is generous enough for a slow boot and short enough to be noticed.
HEARTBEAT_WAIT_S = 20.0


# The B-72 trade-off, closed (issue #13). The heartbeat runs on its own thread
# so blocking callbacks cannot starve it -- which also meant a HUNG manager kept
# the board fed and its 5 s surface failsafe could never fire. A liveness timer
# in its own callback group stamps "the executor can still schedule work"; past
# this many seconds without a stamp the heartbeat is WITHHELD, and the board
# surfaces the vehicle on its own (3 + 5 = 8 s after the hang). Must clear an
# ordinary scheduling hiccup; it does not need to clear the 3-10 s blocking
# callbacks, because those run in OTHER groups on other executor threads.
EXECUTOR_STALE_S = 3.0


# A mission that ATTACHED (published /mongla/mission_alive) and then went silent
# this long, with the vehicle armed, is dead: abort, brake, SURFACE. The runner
# beats at 2 Hz, so this is six missed beats.
MISSION_SILENT_S = 3.0


def mission_watchdog(attached: bool, last_seen: float, now: float,
                     armed: bool, silent_s: float = MISSION_SILENT_S) -> str:
    """'surface' | 'detach' | 'none' -- pure, so the rule is tested without ROS.

    'detach' covers a mission that went silent on a DISARMED vehicle: there is
    nothing to surface, and acting would be noise.
    """
    if not attached or (now - last_seen) <= silent_s:
        return 'none'
    return 'surface' if armed else 'detach'



# A kill disarm that is not confirmed is retried this often. One attempt waits
# up to 2 s for the board's 1 Hz heartbeat; the gap keeps attempts from
# overlapping on the wire.
KILL_RETRY_S = 3.0
KILL_MAX_TRIES = 10


def kill_latch(latch, killed, armed, now: float,
               retry_s: float = KILL_RETRY_S, max_tries: int = KILL_MAX_TRIES):
    """(latch, action) for the kill switch; pure, so the rule is tested without ROS.

    `armed` is True / False / None, where None is "no fresh heartbeat" and is
    treated as possibly armed. `latch` is None, or a dict of attempts.
    Actions: 'disarm' (send one), 'resolved' (a fresh heartbeat says disarmed),
    'none'.

    Why the host acts at all: the switch cuts thruster power on the second
    board, and the control board treats it as display-only (firmware
    `arming.h`, issue #12): it stays armed and keeps running the move, so
    turning the knob back would resume it under a diver's hands. `None` for
    `killed` is "no second-board link", which says nothing about the switch.

    Issue #71. The first version disarmed once, on the edge, and forgot: a
    refused or unanswered disarm left the vehicle armed while the switch was
    engaged, and restoring power would resume the move. The latch holds until
    the board itself reports DISARMED, whatever the knob does in between.
    """
    if latch is None:
        if killed is True and armed is not False:
            return {'tries': 1, 'last': now}, 'disarm'
        return None, 'none'
    if armed is False:
        return None, 'resolved'
    if latch['tries'] < max_tries and now - latch['last'] >= retry_s:
        return {'tries': latch['tries'] + 1, 'last': now}, 'disarm'
    return latch, 'none'


# A SURFACE the board has not confirmed is re-requested this often, this many
# times; then the heartbeat is withheld so the board's own link-loss failsafe
# surfaces it (issue #72; owner decision 2026-11-09).
SURFACE_RETRY_S = 2.0
SURFACE_MAX_TRIES = 3


def mission_recovery(rec, now: float, armed, mode,
                     retry_s: float = SURFACE_RETRY_S,
                     max_tries: int = SURFACE_MAX_TRIES):
    """(rec, action) once a mission died on an armed vehicle; pure.

    `armed` and `mode` come from a heartbeat heard after the first SURFACE
    request (None when there is none). Actions: 'surface' (request again),
    'withhold' (stop heartbeating), 'resolved', 'none'.

    Issue #72. The first version detached the mission, sent one SURFACE and
    forgot. A refused or lost request left the vehicle armed at depth, and the
    manager's heartbeat kept the board's own failsafe from ever firing.
    """
    if armed is False or mode == 'SURFACE':
        return None, 'resolved'
    if now - rec['last'] < retry_s:
        return rec, 'none'
    if rec['tries'] < max_tries:
        return dict(rec, tries=rec['tries'] + 1, last=now), 'surface'
    if not rec.get('withholding'):
        return dict(rec, withholding=True), 'withhold'
    return rec, 'none'


class _HeartbeatThread:
    """The companion HEARTBEAT on its own daemon thread -- nothing may starve it.

    Why not a ROS timer. It used to be `create_timer(0.5, heartbeat_tick)` in
    the MutuallyExclusive `timer_group`, sharing one slot with callbacks that
    BLOCK: `_vision_uplink_tick` -> `_vision_state_for` waits up to 10 s for a
    first CameraInfo, and `_reapply_srot_config` -> `set_default_gain` busy-waits
    up to 3 s. While either ran, no heartbeat left -- and the board SURFACES the
    vehicle after 5 s of silence (CLAUDE.md safety rule 3). Worse, the timer
    only existed once `executor.spin()` started, so every second of bring-up
    after the port opened (preflight reads, payload roles -- many seconds)
    was already silent.

    So it is started the moment the MAVLink backend exists and runs on
    `time.monotonic` (a wall-clock step cannot stall or burst it). Thread-safe
    on the wire: `send_heartbeat` takes the FC's `_tx_lock`, the
    same lock every other writer (reader-thread replies, action threads) holds,
    because pymavlink shares one sequence counter and one port.
    """

    def __init__(self, send, period_s=HEARTBEAT_PERIOD_S, log=None,
                 alive_fn=None):
        self._send = send
        self._period = float(period_s)
        self._log = log
        # None = always beat. Otherwise beat only while it answers True: a hung
        # executor goes SILENT so the board's own failsafe fires (issue #13).
        self._alive_fn = alive_fn
        self._withholding = False
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name='mongla-heartbeat', daemon=True)

    def start(self):
        self._thread.start()
        return self

    def stop(self, timeout=1.0):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=timeout)

    def _run(self):
        fails = 0
        next_t = time.monotonic()
        while not self._stop.is_set():
            alive = True
            if self._alive_fn is not None:
                try:
                    alive = bool(self._alive_fn())
                except Exception:              # noqa: BLE001 -- unknown = keep beating
                    alive = True
            if not alive:
                if not self._withholding and self._log is not None:
                    self._log.error(
                        '[NET  ] !! heartbeat WITHHELD (executor stuck, or a '
                        'mission recovery the board has not confirmed) so the '
                        "board's GCS failsafe surfaces the vehicle (issues #13, #72).")
                self._withholding = True
            elif self._withholding:
                self._withholding = False
                if self._log is not None:
                    self._log.warning('[NET  ] executor alive again -- heartbeat resumed')
            try:
                if alive:
                    self._send()
                fails = 0
            except Exception as exc:          # noqa: BLE001 -- must never die
                fails += 1
                if self._log is not None and fails in (1, 10):
                    self._log.error(f'[NET  ] heartbeat send failed #{fails}: {exc!r}')
            next_t += self._period
            now = time.monotonic()
            if next_t < now:                  # overran: re-anchor, never burst
                next_t = now + self._period
            self._stop.wait(next_t - now)


def _load_sensor_offsets():
    """The frames file, or None if it cannot be read (reported as UNKNOWN)."""
    try:
        from mongla_localization import frames
        return frames.load()
    except Exception:                 # noqa: BLE001 -- health must not raise
        return None


def _kill_text(kill) -> str:
    """Three states, because the wire has three.

    `KILL clear` was printed whenever the value was not True -- including when
    the board had no ESP-NOW link to the second board and therefore could not
    see the switch at all. That is a FALSE SAFETY STATEMENT on the one line an
    operator reads before arming, and it was observed live: a session logged
    `thruster --` (no BATTERY_STATUS instance 1, so no link) next to
    `KILL clear` in the same row.
    """
    if kill is None:
        return 'UNKNOWN (no 2nd-board link)'
    return 'ENGAGED' if kill else 'clear'


def _quat_from_rpy(roll: float, pitch: float, yaw: float):
    """(w, x, y, z) from intrinsic Z-Y-X Euler angles in radians.

    ATTITUDE is MAVLink's aerospace convention and this is its standard
    conversion. Written out rather than pulled from tf_transformations, which
    is not a dependency of this package and would be a new one for nine lines
    of trigonometry.
    """
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)

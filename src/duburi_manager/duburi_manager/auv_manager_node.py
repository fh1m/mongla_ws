#!/usr/bin/env python3
"""AUV Manager Node -- Terminal 1.

* Owns MAVLink connection + reader thread (only thread calling recv_match).
* Exposes /duburi/move as a ROS2 ActionServer.
* Publishes /duburi/state (typed DuburiState message) on change.
* MultiThreadedExecutor: action callbacks + timers run in parallel threads.

Dispatch is registry-driven: every entry in `duburi_control.COMMANDS`
maps to a same-named method on `Duburi`. Adding a new command means a
row in commands.py and a method on Duburi -- this file does not need
to change.

The MAVLINK20 env var is owned by `duburi_control.pixhawk` so we don't
duplicate it here. The console format env is set in this file because
it must be in place BEFORE rclpy is imported.
"""

import os
import math
import signal
import sys
import threading
import time

# Drop ROS2's default `[INFO] [1776530611.533365998] [duburi_manager]:` prefix
# in favour of a compact `[INFO] <message>` so our [CMD  ]/[YAW  ]/[STATE] tags
# are the loudest thing on screen. Must be set BEFORE rclpy is imported.
os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}')

from pymavlink import mavutil                                            # noqa: E402

import rclpy                                                             # noqa: E402
from rclpy.node import Node                                              # noqa: E402
from rclpy.action import ActionServer, CancelResponse, GoalResponse      # noqa: E402
from rclpy.callback_groups import (                                      # noqa: E402
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor                        # noqa: E402

from duburi_interfaces.action import Move                                # noqa: E402
from duburi_interfaces.msg import DuburiState                            # noqa: E402

from duburi_control import (                                            # noqa: E402
    COMMANDS, Duburi, Heartbeat, Pixhawk, fields_for, tracing,
)
from duburi_control.fc import make_flight_controller
from duburi_control.fc.port_guard import PortGuard                     # noqa: E402
from duburi_control.bearing import bearing_from_normalised   # noqa: E402
from duburi_control.fc.srot_protocol import (                            # noqa: E402
    MSG_ID_ESC_STATUS as SROT_MSG_ID_ESC_STATUS,
    SOURCE_SYSID as SROT_SOURCE_SYSID,
    SOURCE_COMPID as SROT_SOURCE_COMPID,
    uplink_class_num as srot_uplink_class_num,
)
from duburi_control.fc.srot_fc import MOVE_VERBS as SROT_MOVE_VERBS       # noqa: E402
from duburi_control.fc.srot_fc import (                                   # noqa: E402
    UNSUPPORTED_VERBS as SROT_UNSUPPORTED_VERBS)
from duburi_control.duburi import _UNARM_SAFE as DUBURI_UNARM_SAFE        # noqa: E402
from duburi_control.tracing import command_scope                          # noqa: E402
from duburi_control.payload import PayloadDriver                         # noqa: E402
from duburi_sensors import make_yaw_source                               # noqa: E402
from duburi_vision  import wait_vision_state_ready                       # noqa: E402

from . import srot_format as _sfmt                                          # noqa: E402
from . import srot_changes as _schg                                        # noqa: E402
from . import health as _health
from . import health_reporters as _hr
from .connection_config import (                                             # noqa: E402
    DEFAULT_FLIGHT_CONTROLLER, DEFAULT_MODE, NETWORK, PROFILES, resolve_mode,
    resolve_profile, resolve_srot_profile,
)
from .dispatch_policy   import goal_acceptance                           # noqa: E402
from .vision_state     import VisionState                                # noqa: E402
from .vision_tunables  import (                                          # noqa: E402
    declare_vision_params,
    runtime_defaults_for_command,
    snapshot_from_node,
)


SEPARATOR = '=' * 52

# How much a value must change before the [STATE] log line reprints.
YAW_CHANGE_THRESH   = 5.0    # degrees
DEPTH_CHANGE_THRESH = 0.08   # metres
# |DEPTH_OUT| at/above this while disarmed => arming would command full heave.
_SROT_DEPTH_OUT_WARN = 0.90
BAT_CHANGE_THRESH   = 0.2    # volts
FORCE_PRINT_SECONDS = 30.0   # always reprint even if nothing changed

# Telemetry stream rates we explicitly request from ArduSub at startup
# via MAV_CMD_SET_MESSAGE_INTERVAL. Without this ArduSub picks defaults
# (typically 4 Hz for AHRS2) which silently caps how tight our control
# loops can be.
MESSAGE_RATES = {
    mavutil.mavlink.MAVLINK_MSG_ID_AHRS2:          50,   # Hz -- yaw/depth source
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE:       50,   # Hz -- body gyro rates (flow rotation-comp)
    mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS:  1,
    mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS:     5,
}

# SROT streams a different set, so it gets its own table rather than reusing the
# ArduSub one:
#   * no AHRS2 -- attitude is ATTITUDE, depth is VFR_HUD
#   * no RC input at all (there is no radio on the vehicle), so RC_CHANNELS is moot
#   * ESC_TELEMETRY_1_TO_4/5_TO_8 carry the RPM (ESC_STATUS msgid 291 is absent from
#     every pymavlink dialect -- see SrotFC.telemetry)
#
# Available since firmware behaviour rev 2. The board clamps any request to a 20 ms
# floor so a companion cannot starve the PARAM_VALUE / COMMAND_ACK traffic missions
# depend on, and it refuses a request to disable HEARTBEAT.
SROT_MESSAGE_RATES = {
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE:       50,   # Hz -- the host-loop ceiling
    # RAW GYRO for flow de-rotation, and it was ABSENT from this table, so it
    # ran at the board's 10 Hz default while the camera ran at 30+. Measured on
    # the board: SET_MESSAGE_INTERVAL takes it 10.0 -> 51.6 Hz with no firmware
    # change. It matters because rotational flow is `f * omega * dt` and does
    # NOT depend on range, so it must be subtracted before the translation is
    # scaled -- with a gyro sample shared across three camera frames, that
    # subtraction is using the wrong omega for two of them.
    #
    # ⛔ A 100 Hz request ACKs ACCEPTED and delivers 50: the firmware's
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


class FeedbackPump:
    """Stream Move.Feedback at ~2.5 Hz while a goal is executing.

    Used as a context manager so the worker thread is guaranteed to be
    joined no matter how the command exits (success, exception, or a
    cancel mid-loop).

    `yaw_provider` is an optional ``fn(attitude) -> (yaw_deg, label)``
    injected by the node so the feedback line reports the same yaw the
    control loops close on (BNO when configured, AHRS otherwise). When
    omitted we fall back to Pixhawk AHRS so the old call site still
    works.

    `vision_provider` is an optional ``fn() -> (x_px, y_px) | None`` -- the live
    signed target-from-centre px the active vision verb reports. When it returns
    a tuple the feedback carries `err_x_px`/`err_y_px` (else NaN) so an operator
    can watch a vision verb converge in real time via `ros2 topic echo`.
    """

    def __init__(self, pixhawk, goal_handle, yaw_provider=None,
                 vision_provider=None):
        self._pixhawk         = pixhawk
        self._goal_handle     = goal_handle
        self._yaw_provider    = yaw_provider
        self._vision_provider = vision_provider
        self._stop            = threading.Event()
        self._thread          = threading.Thread(target=self._run, daemon=True)

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *_):
        self._stop.set()
        self._thread.join(timeout=1.0)

    def _run(self):
        """Stream action feedback while a verb runs.

        GUARDED (B44 class). A bare Thread target: an exception here -- most
        plausibly `publish_feedback()` on a goal that has just been cancelled, or
        a vision provider raising mid-verb -- used to kill the pump outright. The
        verb kept running, but the operator's live `err_x_px` convergence view
        went dead with nothing said. Never fatal, so never allowed to be fatal.
        """
        fails = 0
        while not self._stop.is_set():
          try:
            attitude = self._pixhawk.get_attitude()
            if attitude is not None:
                if self._yaw_provider is not None:
                    yaw_deg, _ = self._yaw_provider(attitude)
                else:
                    yaw_deg = attitude['yaw']
                yaw_str = f'{yaw_deg:.1f}' if yaw_deg is not None else 'N/A'
                vis = (self._vision_provider()
                       if self._vision_provider is not None else None)
                feedback              = Move.Feedback()
                feedback.phase        = 'EXECUTING'
                feedback.current_value = float(attitude['depth'])
                feedback.error_value   = 0.0
                feedback.err_x_px      = float(vis[0]) if vis else math.nan
                feedback.err_y_px      = float(vis[1]) if vis else math.nan
                vis_str = f'  VIS:({vis[0]:+.0f},{vis[1]:+.0f})px' if vis else ''
                feedback.status_line   = (
                    f'YAW:{yaw_str}  DEPTH:{attitude["depth"]:+.2f}m{vis_str}')
                self._goal_handle.publish_feedback(feedback)
            fails = 0
          except Exception:                    # noqa: BLE001 -- see the docstring
            fails += 1
            if fails in (1, 25):               # once, then once more if persistent
                import traceback
                print(f'[FBK  ] feedback pump fault #{fails} (verb continues): '
                      f'{traceback.format_exc(limit=1).strip()}', file=sys.stderr)
          self._stop.wait(timeout=0.4)


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


class AUVManagerNode(Node):
    def __init__(self):
        super().__init__('duburi_manager')
        self._setup_parameters()
        self._setup_mavlink()
        self._setup_reader_and_warmup()
        self._preflight_payload()       # start payload connect in background (parallel to BNO)
        self._setup_yaw_source()        # BNO probe blocks here (≤3s with warm device)
        self._setup_vision_pool()
        self._setup_heartbeat_and_payload()
        self._setup_action_server()

    # ------------------------------------------------------------------ #
    #  Init helpers (called once from __init__, in order)                 #
    # ------------------------------------------------------------------ #

    def _setup_parameters(self) -> None:
        """Declare and read all ROS parameters; resolve mode/profile."""
        self.declare_parameter('mode',             DEFAULT_MODE)
        # Override the profile's connection string at the CLI:
        #   -p mav_device:=/dev/ttyACM0
        #   -p mav_device:=udpin:0.0.0.0:14560
        # Empty string (default) means use the resolved profile.
        self.declare_parameter('mav_device',       '')
        self.declare_parameter('smooth_yaw',       False)
        self.declare_parameter('smooth_translate', False)
        self.declare_parameter('yaw_source',           'mavlink_ahrs')
        # Position for the *_dist verbs. 'none' = no position source, which is
        # what a hull with no DVL has had until now: those verbs REFUSE.
        # 'flow' wraps the yaw source with the bottom camera's flow-derived
        # position. Default stays 'none' -- see _setup_yaw_source.
        self.declare_parameter('position_source',      'none')
        self.declare_parameter('bno085_port',          'auto')
        self.declare_parameter('bno085_baud',          115200)
        self.declare_parameter('payload_port',         'auto')
        # Pool default True. The sim launch sets it False: ArduSub SITL's
        # barometer ACKs PREFLIGHT_CALIBRATION and then stops tracking depth.
        self.declare_parameter('baro_calibration',     True)
        # LANDING_TARGET vision uplink (VISION_API.md). OFF by default: the
        # board parses msgid 149 and silently DROPS it today, so this is a
        # producer built ahead of its consumer. '' = disabled; set to a camera
        # name ('forward') to stream that camera's selected target.
        # Raw MAVLink replay log (.tlog). '' = off; a tag ('gate_am') names the
        # file, which lands beside the scorecards in DUBURI_RUN_DIR. Everything
        # the live console shows is derived from this stream, so the log is the
        # stream itself rather than a second copy of the derived numbers.
        self.declare_parameter('record', '')
        self.declare_parameter('vision_uplink_camera', '')
        self.declare_parameter('vision_uplink_class', '')
        self.declare_parameter('vision_uplink_hz', 25.0)
        # MEDIUM for the uplink bearing. A flat port refracts, so the pinhole
        # model recovers the angle INSIDE the housing and the vehicle needs the
        # angle in the water: `sin(air) = n * sin(water)`. Uncorrected, every
        # bearing on this wire is ~33 % too large off-axis and +10.14 deg at the
        # frame edge on our measured forward lens -- and this module exists to
        # give control gains UNITS, so wrong units defeat its whole purpose.
        # 'water' is the default because that is where the vehicle operates;
        # 'air' is the exact identity, for a bench run.
        self.declare_parameter('vision_uplink_medium', 'water')
        # VELOCITY uplink: the RIEKF's body velocity (/duburi/odom) to the
        # board as VISION_SPEED_ESTIMATE, at the rate localization publishes
        # it. The Pi measures; the board integrates at 500 Hz and closes the
        # distance loop. OFF by default for the same reason as the bearing
        # uplink: the board drops msgid 103 until the firmware PR lands, and
        # the link is 51.8 % full at idle.
        self.declare_parameter('velocity_uplink', False)
        # payload_channels: OPTIONAL per-instance labels, "<board_channel>:<name>",
        # e.g. "9:torpedo_1, 10:torpedo_2, 11:dropper_1".
        #
        # It does NOT route anything. `fire(N)` always addresses board channel N
        # (= DO_SET_SERVO param1 = the n in SERVO{n}_ROLE); these names only make
        # logs and the preflight readable. A stale label can mislabel a log line;
        # it cannot send a shot to the wrong channel, which is exactly why the
        # routing map it replaces is gone.
        #
        # ⚠ MOSTLY SUPERSEDED by SERVO{n}_FUNCTION on the board, which duburi_ws now
        # reads at bring-up (SrotPayload.preflight_roles). Prefer setting Function in
        # Bondor: it is stored in the board's NVS, so the payload map travels with the
        # hull instead of living in a launch file that goes stale on a re-wire.
        # This param survives only for names the board's fixed enum cannot express --
        # "torpedo_1" vs "torpedo_2", which share one FUNCTION. When both are present
        # and disagree, the override is used AND a warning names both.
        self.declare_parameter('payload_channels',     '')
        # payload_fire_map: REMOVED. Kept declared ONLY so a launch file still
        # setting it fails loudly -- see _preflight_payload. Silently ignoring it
        # would be the dangerous option, because its channel numbers (1..4, a
        # host-side index) now mean something completely different (1..16, the
        # board channel), and on the default role layout channel 1 is the ARM.
        self.declare_parameter('payload_fire_map',     '')
        self.declare_parameter('nucleus_dvl_host',     '192.168.2.201')
        self.declare_parameter('nucleus_dvl_port',     9000)
        self.declare_parameter('nucleus_dvl_password', 'nortek')
        # dvl_auto_connect: background retry at startup; eliminates manual dvl_connect
        self.declare_parameter('dvl_auto_connect',  True)
        self.declare_parameter('dvl_retry_s',       5.0)
        # debug:=true flips per-command MAVLink trace + raises logger to DEBUG
        self.declare_parameter('debug',            False)
        # flight_controller: which autopilot backend the HAL builds.
        #   'srot' (DEFAULT) -- the SROT board running Hengla, over direct USB
        #        Type-C serial. No Pixhawk, no Pi, no BlueOS, no UDP. This is
        #        the vehicle.
        #   'pixhawk' -- the ArduSub/BlueOS path. Still fully supported; pass
        #        flight_controller:=pixhawk for it (the sim does exactly that).
        #
        # ⛔ THE DEFAULT FLIPPED TO 'srot' ON 2026-09-08, AND THE REASON IT WAS
        # 'pixhawk' UNTIL THEN IS WORTH KEEPING. The srot->main merge landed
        # "without changing what main does", so the default stayed on the
        # configuration that placed 8th at RoboSub 2025. That tiebreaker has
        # expired: the hull now IS the SROT board, and a default naming a
        # backend the vehicle does not have is the stale claim, not the safe one.
        #
        # The 8th-place configuration is not gone -- it is preserved whole on the
        # `pixhawk` branch (b483722, the pre-merge tree), which is where to look
        # for it rather than reconstructing it from this parameter.
        #
        # Both directions still fail LOUDLY (no heartbeat) rather than silently.
        # Flipping it back is this one line, deliberately.
        self.declare_parameter('flight_controller', DEFAULT_FLIGHT_CONTROLLER)
        # allow_fw_behaviour_mismatch: proceed against firmware older than
        # srot_protocol.FW_BEHAVIOUR_REV_REQUIRED. OFF by default and it should stay
        # off. On pre-rev-2 firmware MOVE_STOP COASTS -- it applies zero braking thrust
        # -- and the host-side reverse-leg brake that used to cover that has been
        # removed, so `stop` and every abort would simply not decelerate 20 kg of hull,
        # with nothing in any log to say why. Setting this true is accepting that.
        self.declare_parameter('allow_fw_behaviour_mismatch', False)
        # Verbose SROT telemetry block period (s); 0 disables. Default 2.0 --
        # the first water test wants a continuous trace to correlate against
        # what the vehicle physically did.
        self.declare_parameter('srot_telemetry_period_s', 2.0)
        # Arm anyway when the depth controller is saturated. Its OWN flag rather
        # than reusing allow_fw_behaviour_mismatch: accepting an unknown firmware
        # revision and accepting full uncommanded heave are different decisions.
        self.declare_parameter('allow_saturated_depth_arm', False)
        declare_vision_params(self)

        requested_mode      = str(self.get_parameter('mode').value)
        mav_device          = str(self.get_parameter('mav_device').value).strip()
        self._smooth_yaw    = bool(self.get_parameter('smooth_yaw').value)
        self._smooth_tr     = bool(self.get_parameter('smooth_translate').value)
        self._yaw_src_name  = str(self.get_parameter('yaw_source').value)
        self._pos_src_name  = str(
            self.get_parameter('position_source').value).strip().lower()
        self._bno_port      = str(self.get_parameter('bno085_port').value)
        self._bno_baud      = int(self.get_parameter('bno085_baud').value)
        self._payload_port  = str(self.get_parameter('payload_port').value)
        self._payload_fire_map = str(self.get_parameter('payload_fire_map').value)
        self._payload_channels = str(self.get_parameter('payload_channels').value)
        self._dvl_host      = str(self.get_parameter('nucleus_dvl_host').value)
        self._dvl_port      = int(self.get_parameter('nucleus_dvl_port').value)
        self._dvl_passwd    = str(self.get_parameter('nucleus_dvl_password').value)
        self._dvl_auto      = bool(self.get_parameter('dvl_auto_connect').value)
        self._dvl_retry_s   = float(self.get_parameter('dvl_retry_s').value)
        self._debug         = bool(self.get_parameter('debug').value)
        self._fc_kind       = str(self.get_parameter('flight_controller').value).strip().lower()
        self._is_srot       = (self._fc_kind == 'srot')

        # Wire MAVLink tracing on as early as possible — mutates contextvar in
        # main thread; daemons spawned later still see the default (False).
        if self._debug:
            tracing.set_enabled(True)
            try:
                import rclpy.logging
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
            except Exception as exc:
                self.get_logger().warning(
                    f'debug:=true: could not raise logger level to DEBUG ({exc}); '
                    f'tag will still apply but [MAV ] lines may not print')

        self._mode_name = resolve_mode(requested_mode, logger=self.get_logger())
        if self._is_srot:
            # SROT connects over direct USB Type-C serial -- bypass the BlueOS/UDP
            # profile machinery entirely (mode still labels the yaw-source hints).
            self._profile = resolve_srot_profile(
                mav_device, logger=self.get_logger())
        else:
            self._profile = resolve_profile(
                self._mode_name, mav_device=mav_device, logger=self.get_logger())

    def _setup_mavlink(self) -> None:
        """Open MAVLink connection, wait for heartbeat, pin telemetry rates."""
        self.get_logger().info(
            f'Connecting ({self._mode_name}) -> {self._profile["conn"]} ...')
        baud_kw = {'baud': self._profile['baud']} if self._profile['baud'] else {}
        # Identify as an ONBOARD COMPUTER (191) on srot, not as "some GCS" (pymavlink's
        # default 190). The board's LoRa bridge synthesises its filler heartbeat as
        # 255/190 -- the same identity we were using -- so the firmware could not tell
        # the companion from the ground station. Consequence: a DEAD JETSON with Bondor
        # still connected holds the GCS failsafe open, and the vehicle station-keeps
        # when it should surface. A distinct compid is what lets the firmware key
        # FS_GCS_SYSID/FS_GCS_COMPID on us specifically (their JETSON_FEEDBACK §4).
        #
        # Safe to ship before that firmware lands: the board counts ANY heartbeat whose
        # id is not its own (`msg.compid != MAV_COMPONENT_ID || msg.sysid !=
        # MAV_SYSTEM_ID`, fw mav_commands.cpp:687), so 191 keeps the failsafe fed
        # exactly as 190 did.
        #
        # srot ONLY -- the pixhawk path keeps pymavlink's defaults so it stays
        # byte-identical to history, which is the whole promise of that backend.
        if self._is_srot:
            baud_kw['source_system'] = SROT_SOURCE_SYSID
            baud_kw['source_component'] = SROT_SOURCE_COMPID
        # Claim the port BEFORE opening it. On srot every open reboots the flight
        # controller (fc/port_guard.py has the measurements), so a second process
        # touching this device mid-mission is a silent disarm-and-reinit. The
        # kernel does not lock a tty; this does. A non-serial endpoint is a no-op.
        self._port_guard = PortGuard(self._profile['conn'], log=self.get_logger())
        self._port_guard.acquire()
        self.master  = mavutil.mavlink_connection(self._profile['conn'], **baud_kw)
        self.master.wait_heartbeat()
        # Build the backend behind the FlightController HAL. PixhawkFC is-a Pixhawk,
        # so `self.pixhawk` stays a valid alias for every existing direct call; SrotFC
        # exposes the same read surface. `flight_controller=pixhawk` is unchanged.
        self.fc = make_flight_controller(
            self._fc_kind, master=self.master, log=self.get_logger())
        self.pixhawk = self.fc
        self.get_logger().info(f'[NET  ] flight_controller = {self.fc.name}')
        if self._is_srot:
            self.fc.allow_saturated_depth_arm = bool(
                self.get_parameter('allow_saturated_depth_arm').value)
            self.fc.allow_fw_behaviour_mismatch = bool(
                self.get_parameter('allow_fw_behaviour_mismatch').value)
            # The two round-trip READS -- behaviour rev and JS_GAIN_DEFAULT -- used to
            # run here and could never succeed. Both land their reply in
            # `master.messages`, which only fills while SOMETHING DRAINS THE LINK, and
            # the reader thread does not start until `_setup_reader_and_warmup()`.
            # Nothing between `wait_heartbeat()` and that point calls recv_*, so the
            # replies were parsed by no one: `check_behaviour_rev` burned its 3x2 s of
            # retries and reported FW_BEHAVIOUR_REV_UNKNOWN on every single startup.
            #
            # That is worse than a missing read. Its warning says the firmware may be
            # pre-rev-2, i.e. that MOVE_STOP COASTS with no host brake -- so the one
            # bring-up check meant to catch an un-brakeable hull cried wolf every time,
            # and an operator who believed it would ground a perfectly good board.
            # (The ARM-time call in `SrotFC.arm()` was always fine: by then the reader
            # is running. Only the bring-up copy was broken -- on serial too, this is
            # not a UDP/BlueOS artefact.)
            #
            # They now run from `_srot_preflight_reads()`, straight after the reader
            # starts. Rate pinning stays here: `set_message_rate` is fire-and-forget
            # and reads no reply, so it works with nobody draining.
            # Pin rates. This used to be skipped: "SROT rates are fixed on-board (no
            # SET_MESSAGE_INTERVAL)". Firmware behaviour rev 2 implements 511 and 510,
            # so ATTITUDE is no longer stuck at the board's 10 Hz default -- which was
            # the ceiling on every host loop, and left _imu_rates_tick republishing a
            # 10 Hz stream at 50 Hz (5x oversampling into the flow rotation-comp).
            for msg_id, hz in SROT_MESSAGE_RATES.items():
                self.pixhawk.set_message_rate(msg_id, hz)
        else:
            # Pin rates so ArduSub streams at the rates we need (default ~4 Hz).
            for msg_id, hz in MESSAGE_RATES.items():
                self.pixhawk.set_message_rate(msg_id, hz)

    def _start_recorder(self) -> None:
        """Open the raw .tlog if `record:=<tag>` was given, else leave it None.

        Set before the reader thread starts, because the reader is the sink: a
        recorder attached later would silently miss the startup burst, which is
        where the board's banner, its behaviour revision and the first
        NAMED_VALUE_FLOAT sweep live -- exactly the part of a run you go back to
        the log for.

        Best-effort. A log that cannot be opened must not stop the vehicle from
        flying, so it degrades to no recording with a loud line rather than
        raising out of bring-up.
        """
        self._recorder = None
        tag = (self.get_parameter('record').value or '').strip()
        if not tag:
            return
        try:
            from duburi_manager.srot_recorder import SrotRecorder, default_path
            self._recorder = SrotRecorder(
                default_path(tag), log=self.get_logger()).start()
            # BOTH DIRECTIONS, or the log has no decisions in it.
            #
            # The reader thread only ever sees what ARRIVES, so a log fed from
            # it alone holds COMMAND_ACK and no COMMAND_LONG -- measured on the
            # first live capture: 5 acks, 0 commands. That is the same
            # impoverishment the board's own SD log has, and the reason this
            # log exists at all is to hold what the host DECIDED: the verbs it
            # issued, the MANUAL_CONTROL it streamed, the heartbeats that keep
            # the failsafe quiet.
            #
            # `send_callback` is pymavlink's own hook, called after every send
            # with the packed message, so this needs no wrapper around the
            # transport and no change at any of the seven send sites.
            self.master.mav.set_send_callback(
                lambda msg, *_a, **_k: self._recorder.write(msg))
        except Exception as exc:              # noqa: BLE001
            self._recorder = None
            self.get_logger().error(
                f'[REC  ] recording DISABLED -- could not open the log: {exc!r}')

    def _setup_reader_and_warmup(self) -> None:
        """Start the MAVLink reader thread, then wait for AHRS2 + autopilot HB.

        Reader must start BEFORE yaw_source init — BNO085 calibration calls
        pixhawk.get_attitude() and returns None if the reader isn't running.
        """
        self.last_statustext = ''
        self.prev_state      = {}
        self.last_print_time = 0.0
        self.prev_rc         = None
        # Fast HUD cache: armed/mode/battery reused by the 20 Hz instrument tick
        self._fast_armed  = False
        self._fast_mode   = ''
        self._fast_batt_v = math.nan
        self._start_recorder()          # BEFORE the reader -- it is the sink
        self.reader_thread = threading.Thread(
            target=self.reader_loop, daemon=True)
        self.reader_thread.start()
        # Only NOW can a round-trip read see its reply -- see _setup_mavlink.
        if self._is_srot:
            self._srot_preflight_reads()

        # Warmup: wait for both AHRS2 and a valid autopilot heartbeat.
        _deadline = time.monotonic() + 4.0
        while time.monotonic() < _deadline:
            if (self.pixhawk.get_attitude() is not None
                    and self.pixhawk.get_mode() != 'UNKNOWN'):
                break
            time.sleep(0.05)
        else:
            self.get_logger().warning(
                '[NET  ] AHRS2 or autopilot heartbeat not received within 4s. '
                'BNO085 calibration may still fail. '
                'Check MAVLink link and ArduSub telemetry rate config.')

    def _srot_preflight_reads(self) -> None:
        """SROT bring-up reads that need a reply. Call AFTER the reader thread starts.

        Both of these poll `master.messages`, which only fills while the reader is
        draining the link -- running them any earlier reports a failure that says
        more about our own startup order than about the board. See _setup_mavlink.

        Still before anything that could move: arming is an operator action and
        `SrotFC.arm()` re-checks the rev itself, so this is the early warning, not
        the gate.
        """
        fw_ok, fw_reason = self.fc.check_behaviour_rev()
        if not fw_ok:
            self.get_logger().error(f'[NET  ] {fw_reason}')
        # B34: the host clamps every move to its OWN copy of MOVE_CRUISE_MAX.
        # Say so when the board disagrees, or raising the board's cap silently
        # does nothing to autonomous moves.
        cm_ok, cm_reason = self.fc.check_move_cruise_max()
        (self.get_logger().info if cm_ok else self.get_logger().warning)(
            f'[SROT ] {cm_reason}')
        # Set the pilot gain to full so autonomous MANUAL_CONTROL isn't halved.
        if not self.fc.set_default_gain():
            self.get_logger().warning(
                '[NET  ] could not confirm JS_GAIN_DEFAULT=1.0 -- MANUAL_CONTROL '
                'may be scaled; check the board is reachable + not mid param-download')
        # DEPTH_P, so the arming guard can convert DEPTH_CMD back into metres of depth
        # error. Read once here rather than inside arm(): a param round-trip on the
        # arming path adds a failure mode to the one call that must not acquire new
        # ones. Absent -> the guard uses the firmware default and says so.
        self.fc.read_depth_p()
        # YAW_REF (fw rev 9). Only LOCKED means ATTITUDE.yaw is a magnetic heading;
        # anything else and an absolute `turn` is aiming at a boot-relative number.
        yr_ok, yr_reason = self.fc.check_yaw_reference()
        (self.get_logger().info if yr_ok else self.get_logger().warning)(
            f'[SROT ] {yr_reason}')

    # Heading sources whose yaw we trust enough to rotate a velocity by. The
    # hull compass is not one of them: `mavlink_ahrs` on the aluminium hull is
    # the source §6 of CLAUDE.md calls untrusted, and flow position rotates
    # body velocity into the latched frame, so heading error becomes CROSS-TRACK
    # position error directly -- 5 deg over 1 m is 8.7 cm, and the return-leg
    # measurements say cross-track drift already dominates.
    _POSITION_TRUSTED_YAW = ('bno085', 'bno085_dvl', 'dvl_bno', 'dvl',
                             'nucleus_dvl', 'bno085_sim_dvl', 'sim_dvl')

    def _wrap_position_source(self) -> None:
        """Optionally give the yaw source a position, from the bottom camera.

        `drive_forward_dist` / `drive_lateral_dist` duck-type on
        `get_position`/`reset_position`, which ONLY the Nortek sources provide
        -- and the Nortek is not fitted. So on this hull those verbs have been
        dead. `FlowPositionSource` supplies exactly that contract from
        `flow_node`, without the motion layer changing at all, which matters
        because those files carry the runaway guards added after measuring
        11.3 m of travel on a 1.0 m command.

        OFF BY DEFAULT. In-water accuracy is unvalidated (dry bench: 103.4 % of
        truth, +-7 % height uncertainty), so an operator opts in per run. The
        capability is that the verbs are runnable AND refuse honestly; it is
        not that they are trustworthy.
        """
        name = self._pos_src_name
        if name in ('', 'none', 'off', 'false'):
            return
        if name != 'flow':
            self.get_logger().error(
                f"[FLOWP] position_source={name!r} is unknown. Known: "
                f"'none' (default), 'flow'. Leaving the *_dist verbs without "
                f"a position source.")
            return
        if hasattr(self.yaw_source, 'get_position'):
            self.get_logger().info(
                f'[FLOWP] position_source:=flow ignored -- '
                f'{self.yaw_source.name!r} already supplies a position.')
            return
        # The displacement is rotated by the RIEKF's attitude, which is seeded
        # from `/duburi/imu`. On srot that is the board's fused BNO085 -- not a
        # hull compass -- so the warning is for the pixhawk path only.
        if not self._is_srot and self._yaw_src_name not in self._POSITION_TRUSTED_YAW:
            self.get_logger().warning(
                f'[FLOWP] position_source:=flow with yaw_source='
                f'{self._yaw_src_name!r}. Flow gives velocity in the BODY '
                f'frame and this rotates it by that heading, so heading error '
                f'lands directly in cross-track position (5 deg over 1 m is '
                f'8.7 cm). Prefer a BNO-backed heading for distance moves.')
        try:
            from duburi_manager.flow_position import FlowPositionSource
            self.yaw_source = FlowPositionSource(
                self, self.yaw_source, camera='downward')
        except Exception as exc:      # noqa: BLE001 -- never block startup
            self.get_logger().error(
                f'[FLOWP] could not attach flow position ({exc}); the *_dist '
                f'verbs stay refused.')

    def _setup_yaw_source(self) -> None:
        """Instantiate yaw source, print startup banner, start DVL auto-connect."""
        _DVL_SOURCES = {'dvl', 'nucleus_dvl', 'bno085_dvl', 'dvl_bno'}
        # The BNO085 moved onto the SROT board (I2C0) and the separate ESP32-C3 +
        # BNO085 USB board was removed from the hull. Selecting a source that reads
        # it will fail in make_yaw_source with a bare SerialException about a missing
        # port, which reads like a loose cable rather than "that board is gone". Say
        # the true thing first; the raise below still stops startup.
        if self._is_srot and self._yaw_src_name in ('bno085', 'bno085_dvl', 'dvl_bno'):
            self.get_logger().error(
                f'[SENS ] yaw_source={self._yaw_src_name!r} reads the USB ESP32-C3 + '
                f'BNO085 board, which is NOT FITTED on the srot vehicle -- the BNO085 '
                f'is on the control board now. Use yaw_source:=mavlink_ahrs (the '
                f'board\'s own fused ATTITUDE, same sensor, 500 Hz).')
        try:
            self.yaw_source = make_yaw_source(
                self._yaw_src_name,
                pixhawk=self.pixhawk,
                port=self._bno_port,
                baud=self._bno_baud,
                nucleus_dvl_host=self._dvl_host,
                nucleus_dvl_port=self._dvl_port,
                nucleus_dvl_password=self._dvl_passwd,
                logger=self.get_logger(),
            )
        except Exception as exc:
            self.get_logger().fatal(
                f'[SENS ] yaw_source={self._yaw_src_name!r} failed to init: {exc}')
            raise

        self._wrap_position_source()

        # Duck-typed: only BNO085Source has read_pitch/read_roll.
        self._bno_mocap_active: bool = hasattr(self.yaw_source, 'read_pitch')

        # Banner
        yaw_tag = 'glide' if self._smooth_yaw else 'snap(PID)'
        tr_tag  = 'eased' if self._smooth_tr  else 'constant'
        yaw_src_label = self.yaw_source.name
        if self._yaw_src_name == 'bno085':
            yaw_src_label = f'{yaw_src_label} ({self._bno_port} @ {self._bno_baud})'
            offset = getattr(self.yaw_source, 'offset_deg', None)
            if offset is not None:
                yaw_src_label += f'  Earth-ref offset: {offset:+.2f} deg'
        elif self._yaw_src_name in _DVL_SOURCES:
            connect_hint = ('auto-connecting...' if self._dvl_auto
                            else 'DISCONNECTED -- call dvl_connect')
            yaw_src_label = (f'{yaw_src_label} '
                             f'({self._dvl_host}:{self._dvl_port}  {connect_hint})')

        self.get_logger().info(SEPARATOR)
        self.get_logger().info(
            f' MONGLA · DUBURI AUV MANAGER  |  mode: {self._mode_name}')
        if self._is_srot:
            self.get_logger().info(
                ' Autopilot: SROT board  ·  firmware: Hengla  ·  link: USB serial')
        else:
            self.get_logger().info(
                ' Autopilot: Pixhawk / ArduSub  ·  link: BlueOS/UDP')
        self.get_logger().info(
            f' Connection: {self._profile["conn"]}'
            + (f' @ {self._profile["baud"]}' if self._profile.get('baud') else ''))
        if self._debug:
            self.get_logger().info(
                ' DEBUG TRACE: ON  -- per-command [MAV <fn> cmd=<verb>] '
                'lines will print on every outbound MAVLink frame')
        self.get_logger().info(
            f' MAVLink: sys={self.master.target_system} '
            f'comp={self.master.target_component}  (v2.0)')
        self.get_logger().info(f' Profiles: yaw={yaw_tag}  translate={tr_tag}')
        self.get_logger().info(f' Yaw source: {yaw_src_label}')
        if self._mode_name in ('pool', 'laptop'):
            self.get_logger().info(
                f' Expect BlueOS "{NETWORK["endpoint"]}" -> UDP Client '
                f'{NETWORK["jetson_ip"]}:{NETWORK["mav_port"]}')
        if self._yaw_src_name == 'bno085' and self._mode_name in ('sim', 'laptop', 'desk'):
            self.get_logger().info(
                ' [HINT ] BNO is the yaw source for ALL Python loops '
                '(yaw_*, lock_heading, translation heading-hold, [STATE], '
                'feedback). On a desk SITL the BNO chip does NOT move when '
                'ArduSub yaws in Gazebo -- so absolute-yaw verbs terminate '
                'only when you physically rotate the BNO board. Use '
                'yaw_source:=mavlink_ahrs for desk SITL smoke tests; keep '
                'bno085 for pool runs where the board moves with the AUV.')
        self.get_logger().info(SEPARATOR)

        if self._dvl_auto and self._yaw_src_name in _DVL_SOURCES:
            self._dvl_auto_retry_s = self._dvl_retry_s
            threading.Thread(
                target=self._dvl_auto_connect_loop,
                daemon=True, name='dvl_auto_connect').start()

    def _preflight_payload(self) -> None:
        """Prepare the payload driver.

        SROT: the payload is INTEGRATED into the board (PCA9685 over MAVLink,
        DO_SET_SERVO/DO_SET_RELAY) -- there is NO separate USB ESP32. Critically,
        that old board was a CH340 (1a86:7523), the SAME chip as the SROT board, so
        running the USB auto-detect here would grab the SROT serial port. Build the
        MAVLink-backed SrotPayload instead; no USB scan, no thread.

        Pixhawk: the historical USB PayloadDriver, connected in a background thread
        (concurrent with the BNO085 probe; joined in _setup_heartbeat_and_payload).
        """
        if self._is_srot:
            from duburi_control.fc.srot_fc import SrotPayload
            # A launch file still passing the removed routing param must STOP, not be
            # quietly ignored: its numbers were host-side indices 1..4, and the same
            # numbers now address board channels 1..4 -- which on the default role
            # layout are the on-board ARM. Failing closed here is the whole migration.
            if self._payload_fire_map.strip():
                self._payload = None
                self._payload_thread = None
                self.get_logger().error(
                    '[PAYLOAD] REFUSING TO ARM THE PAYLOAD: `payload_fire_map` is set '
                    f'({self._payload_fire_map!r}) but that parameter was REMOVED. '
                    'fire(N) now addresses BOARD channel N directly (N = DO_SET_SERVO '
                    'param1 = the n in SERVO{n}_ROLE), so the old 1..4 channel numbers '
                    'now point at completely different hardware -- on the default role '
                    'layout, at the manipulator arm. Delete payload_fire_map and pass '
                    'the real board channels to fire(); use payload_channels:="9:torpedo_1, '
                    '11:dropper_1" if you want names in the logs. Payload DISABLED.')
                return
            names = _parse_payload_channels(self._payload_channels, self.get_logger())
            self._payload = SrotPayload(self.fc, log=self.get_logger(), names=names)
            self._payload_thread = None
            self.get_logger().info(
                '[PAYLOAD] SROT: PCA9685 over MAVLink (DO_SET_SERVO), no separate USB '
                'board. fire(N) addresses BOARD channel N; which channels are fireable '
                'is read from the board (SERVO{n}_ROLE), never assumed.')
            # Read every channel role off the board NOW, so a mission aimed at the
            # arm is caught on the deck rather than mid-drop. This is also what
            # tells the operator which channels are actually fireable today.
            try:
                self._payload.preflight_roles()
            except Exception as exc:                # noqa: BLE001 -- advisory only
                self.get_logger().warn(
                    f'[PAYLOAD] could not read channel roles: {exc!r} -- fire() '
                    f'FAILS CLOSED on any channel whose role it cannot read')
            return
        self._payload = PayloadDriver()
        _pl_port = None if self._payload_port in ('auto', '') else self._payload_port
        self._payload_thread = threading.Thread(
            target=lambda: self._payload.connect(port=_pl_port, exclude=set()),
            daemon=True, name='payload-connect')
        self._payload_thread.start()

    def _setup_vision_pool(self) -> None:
        """Initialise the lazy per-camera VisionState pool."""
        self._vision_states: dict = {}
        self._vision_lock         = threading.Lock()

    def _setup_heartbeat_and_payload(self) -> None:
        """Start heartbeat, join payload connect thread, build Duburi facade."""
        self.heartbeat = Heartbeat(self.pixhawk, log=self.get_logger())
        # The Heartbeat streams NEUTRAL RC at 5 Hz (ArduSub FS_PILOT_INPUT guard).
        # On SROT that fights an on-board AUTO move, and the mandatory >=1 Hz MAVLink
        # HEARTBEAT is already sent by heartbeat_tick (2 Hz) -> do NOT stream it.
        if not self._is_srot:
            self.heartbeat.start()

        # Payload connect ran in parallel with BNO probe — join now (USB path only;
        # SROT's MAVLink payload has no thread).
        if self._payload_thread is not None:
            self._payload_thread.join(timeout=5.0)
            if self._payload_thread.is_alive():
                # Thread still running after timeout — treat as not connected.
                self.get_logger().warning(
                    '[PAYLOAD] connect timed out — fire() calls will log-stub only')
            elif self._payload.is_ready:
                self.get_logger().info(
                    f'[PAYLOAD] verified + connected on {self._payload.port_path}')
            else:
                self.get_logger().info(
                    '[PAYLOAD] not found — fire() calls will log-stub only')

        # Lazy DistanceState bridge for calc_distance (built on first use so
        # single-camera / no-distance runs never create the service clients).
        self._distance_state = None

        self.duburi = Duburi(
            self.pixhawk,
            log=self.get_logger(),
            smooth_yaw=self._smooth_yaw,
            smooth_translate=self._smooth_tr,
            yaw_source=self.yaw_source,
            vision_state_provider=self._vision_state_for,
            distance_provider=self._distance_state_for,
            heartbeat=self.heartbeat,
            payload=self._payload,
            # False only in simulation -- SITL's baro cannot be calibrated and
            # the attempt is destructive. See Duburi._run_baro_calibration.
            baro_calibration=self.get_parameter('baro_calibration').value,
        )

    def _distance_state_for(self):
        """Return (build on first call) the DistanceState bridge to the downward
        optical-flow estimator. Injected into Duburi as distance_provider."""
        if self._distance_state is None:
            from .distance_state import DistanceState
            self._distance_state = DistanceState(self, camera='downward')
            self.get_logger().info('[DIST ] DistanceState bridge built (downward)')
        return self._distance_state

    def _setup_action_server(self) -> None:
        """Create callback groups, action server, state publisher, and timers."""
        self.action_group = ReentrantCallbackGroup()
        self.timer_group  = MutuallyExclusiveCallbackGroup()

        self.command_active = False
        self.action_server  = ActionServer(
            self, Move, '/duburi/move',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_group,
        )

        self.state_publisher = self.create_publisher(
            DuburiState, '/duburi/state', 10)

        # Body-frame angular rates for the downward optical-flow distance
        # estimator (rotation compensation). Vector3Stamped: x=pitch_rate,
        # y=roll_rate, z=yaw_rate (rad/s); header.stamp = sample time so the
        # vision node can interpolate the rate to each flow frame-pair. Sourced
        # from Pixhawk ATTITUDE (pinned 50 Hz), NOT the BNO -- see get_angular_rates.
        from geometry_msgs.msg import Vector3Stamped
        self._Vector3Stamped = Vector3Stamped
        self.imu_rates_publisher = self.create_publisher(
            Vector3Stamped, '/duburi/imu_rates', 10)
        # The FULL inertial sample, for the invariant filter in
        # duburi_localization. A SEPARATE topic from `/duburi/imu_rates`
        # deliberately: that one is the flow node's de-rotation feed, shaped
        # (pitch, roll, yaw) for its interpolator, and re-shaping it would
        # break a working consumer to save one publisher. This one is standard
        # `sensor_msgs/Imu` in body axes, which is what every estimator and
        # every ROS tool already expects.
        from sensor_msgs.msg import Imu
        self._Imu = Imu
        self.imu_publisher = self.create_publisher(Imu, '/duburi/imu', 10)
        # The demand last SENT to the thrusters (srot only): x = forward,
        # y = lateral, in [-1, 1]; NaN = unknown (a board primitive owns it, or
        # no frame is in force). The command-velocity model in
        # duburi_localization learns speed from it while flow works and aids
        # the filter with it when flow refuses. Unknown must reach the consumer
        # as unknown -- silence would read as "the last value still holds".
        self.demand_publisher = self.create_publisher(
            Vector3Stamped, '/duburi/demand', 10)
        self._imu_rpy_warned = False
        # Board-clock -> host-clock mapping for the IMU stamp. See
        # _imu_rates_tick: the board's own interval has sd 0.00 ms where
        # arrival has sd 6.67, so the sender's clock is the better time base.
        from duburi_vision.flow.flow_timing import ClockMap
        self._imu_clock = ClockMap(window_s=20.0, min_pairs=40)
        self._imu_clock_fit_t = 0.0
        # (mapped board time, monotonic instant it was computed) or None.
        self._board_stamp = None
        self._imu_clock_ok = False
        self._imu_clock_warned = False

        # Per-thruster RPM, srot only. The bench runbook's step 1 asks for this and
        # it did not exist: SrotFC.telemetry() decoded RPM but had no production
        # caller, so `leak`, `water_temp_c` and `rpm` never reached ROS at all.
        #
        # It was also unreachable until firmware behaviour rev 2. ESC_STATUS (291)
        # was removed from upstream `common`, and pymavlink SILENTLY discards any
        # msgid missing from its CRC-extra table -- so the board could stream RPM
        # perfectly while the host read nothing, indistinguishable from an ESC
        # fault. Rev 2 emits ESC_TELEMETRY_1_TO_4 / _5_TO_8, which do decode, and
        # SrotFC.telemetry() already had the fallback written and waiting.
        #
        # int32 rather than uint16: the board sends magnitude (the ESC_TELEMETRY
        # field is unsigned) and direction lives in the commanded value, so a
        # consumer must not read these as signed velocity.
        self.esc_rpm_publisher = None
        self._leak_latched = False
        self._srot_block_last = 0.0
        self._srot_prev_fields = None
        self._srot_block_period = float(
            self.get_parameter('srot_telemetry_period_s').value)
        if self._is_srot:
            from std_msgs.msg import Int32MultiArray
            self._Int32MultiArray = Int32MultiArray
            self.esc_rpm_publisher = self.create_publisher(
                Int32MultiArray, '/duburi/esc_rpm', 10)

        # HEALTH. 1 Hz, and it only speaks when the vehicle's overall state
        # CHANGES -- a board that logs every second is a board nobody reads,
        # and the transition is the event worth seeing.
        self._health = _health.HealthBoard()
        self._register_health()
        self._health_last = None
        self.create_timer(1.0, self._health_tick, callback_group=self.timer_group)
        self.create_timer(0.5,  self.heartbeat_tick,   callback_group=self.timer_group)
        self.create_timer(0.5,  self.telemetry_tick,   callback_group=self.timer_group)
        # Fast tick: 20 Hz HUD compass + depth (AHRS2 pinned to 50 Hz).
        # Separate callback group so it can fire between telemetry ticks.
        self.fast_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(0.05, self._fast_state_tick, callback_group=self.fast_group)
        # IMU rates at 50 Hz (matches the ATTITUDE stream pin) -- the flow
        # rotation-comp is ~1:1 with the signal, so publish at full rate.
        self.create_timer(0.02, self._imu_rates_tick, callback_group=self.fast_group)

        # ⛔ THE TIMER IS UNCONDITIONAL, and that is the fix. It used to be
        # created only if `vision_uplink_camera` was already set AT STARTUP, so
        # `ros2 param set ... vision_uplink_camera downward` mid-run reported
        # success and did nothing -- the timer did not exist and never would.
        # Measured on the vehicle: the parameter took, and not one uplink tick
        # ever ran. That is the same shape as `vision.lock_s`, which was
        # declared, documented, mapped and never passed.
        #
        # The tick already early-returns on an empty camera, so an idle timer
        # costs one comparison at `vision_uplink_hz`. A mission can now aim the
        # uplink the way it aims everything else, without a relaunch.
        if self._is_srot:
            hz = max(1.0, float(self.get_parameter('vision_uplink_hz').value))
            self._uplink_cam_logged = None
            self.create_timer(1.0 / hz, self._vision_uplink_tick,
                              callback_group=self.timer_group)
            cam0 = str(self.get_parameter('vision_uplink_camera').value).strip()
            self.get_logger().info(
                f'[VIS  ] LANDING_TARGET uplink armed @ {hz:.0f} Hz, '
                f'camera={cam0 or "(none -- idle until set)"} '
                f'(the board does not consume msgid 149 yet -- producer only)')
            # Subscribed unconditionally, gated per message on the parameter, so
            # `ros2 param set ... velocity_uplink true` works mid-run -- the
            # startup-only-timer defect this file has already had once.
            from nav_msgs.msg import Odometry
            self._vel_uplink_n = 0
            self.create_subscription(Odometry, '/duburi/odom',
                                     self._on_odom_uplink, 10,
                                     callback_group=self.timer_group)

        # BNO->EKF3 mocap injection is an ArduSub/BlueOS feature; SROT fuses the
        # BNO on-board, so there is no external EKF to feed (skip on the srot path).
        if self._bno_mocap_active and not self._is_srot:
            self.create_timer(0.05, self._mocap_tick, callback_group=self.timer_group)
            self.get_logger().info('[SENS ] ATT_POS_MOCAP yaw injection active (20 Hz).')
            self._verify_extnav_params()

    # ================================================================== #
    #  Vision state pool -- lazily built per camera, preflighted once     #
    # ================================================================== #

    def _vision_state_for(self, camera: str):
        """Return (and build on first call) the VisionState for `camera`.

        The control loop reads ``/detections`` directly (the topic the HUD
        shows); the tracker keeps running for display only. Subscriptions
        stay alive for the rest of the process lifetime so repeat vision_*
        goals don't pay the preflight wait twice. The FIRST build for a
        camera requires a live detection (``require_detection``) so a goal
        fails fast with a clear "no boxes" rather than silently chasing an
        empty cache.
        """
        cache_key = camera

        with self._vision_lock:
            cached = self._vision_states.get(cache_key)
            if cached is not None:
                return cached
            self.get_logger().info(
                f'[VST  ] building VisionState for camera={camera!r}')
            vstate = VisionState(self, camera=camera,
                                 logger=self.get_logger())

        # Preflight outside the lock — it just polls VisionState's diags.
        # The FIRST build waits (up to 10 s) for a live detection so the
        # pipeline has time to warm up before the loop starts. The loop
        # itself tolerates no-detection (grace -> LOST -> DSL fallback), so
        # we cache the state either way and never pay this wait twice.
        try:
            wait_vision_state_ready(
                vstate, timeout=10.0, require_detection=True,
                log=self.get_logger())
        except Exception as exc:
            self.get_logger().warning(
                f'[VST  ] preflight for {camera!r} did not pass within '
                f'10s: {exc!r}; first goal starts cold (loop will search '
                f'via its fallback)')

        with self._vision_lock:
            # Check again under lock in case a concurrent goal built the same state.
            if cache_key not in self._vision_states:
                self._vision_states[cache_key] = vstate
            return self._vision_states[cache_key]

    # ================================================================== #
    #  DVL auto-connect background loop                                   #
    # ================================================================== #

    def _dvl_auto_connect_loop(self):
        """Background thread: try to connect the DVL, retry on failure.

        Runs until the DVL is connected successfully. After first
        success, sleeps forever (daemon thread dies with the process).
        This eliminates the manual `duburi dvl_connect` step for
        standard pool-day workflow.
        """
        src = self.yaw_source
        if src is None or not hasattr(src, 'connect'):
            return

        attempt = 0
        while True:
            attempt += 1
            try:
                src.connect()   # type: ignore[union-attr]
                self.get_logger().info(
                    f'[DVL  ] auto-connect succeeded (attempt {attempt})')
                return
            except Exception as exc:
                self.get_logger().warning(
                    f'[DVL  ] auto-connect attempt {attempt} failed: {exc}  '
                    f'-- retrying in {self._dvl_auto_retry_s:.0f}s')
                time.sleep(self._dvl_auto_retry_s)

    # ================================================================== #
    #  MAVLink reader -- only place recv_match() is called                #
    # ================================================================== #

    def reader_loop(self):
        # SROT multiplexes LEAK/WTEMP/STUNT_PRG/ATUNE/KILL/CURR/GAIN onto NAMED_VALUE_FLOAT
        # and sends all seven back-to-back in one 500 ms tick, while pymavlink keeps exactly
        # ONE message per msgid. So by the time anything reads the slot the burst has already
        # drained through it and only the last name -- GAIN -- is left, until the next burst.
        # Sampling the slot therefore does not miss LEAK occasionally; it misses it always.
        # This loop is the only place that sees the names in between, so it is the only place
        # the de-multiplexing can happen. Pixhawk has no such hook and is untouched.
        #
        # BATTERY_STATUS has the identical problem one layer down: the board sends
        # instance 0 (PM1 electronics) and instance 1 (PM2 thruster pack) at 2 Hz each,
        # and pymavlink keys its cache by MSGID, not instance -- so the slot alternates
        # between two voltages an order of magnitude apart (measured: 1.35 V / 14.74 V).
        note = getattr(self.fc, 'note_named_value', None)
        note_batt = getattr(self.fc, 'note_battery', None)
        # STATUSTEXT has the identical problem and is worse to miss: the board
        # sends ~13 announcements as a BURST at boot, so a poller sees the last
        # one and loses the rest -- including "Params reset to build defaults",
        # which silently puts the pilot gain back to half authority and disables
        # the leak failsafe. It is also the ONLY place per-thruster telemetry
        # presence reaches the wire, and that line is sent once, at first arm.
        note_text = getattr(self.fc, 'note_statustext', None)
        _DEMUX = {'NAMED_VALUE_FLOAT': note, 'BATTERY_STATUS': note_batt,
                  'STATUSTEXT': note_text}
        # ⛔ THE BODY IS GUARDED, AND THE THREAD'S DEATH IS OBSERVABLE (B40).
        #
        # This is a bare `threading.Thread` target and it is the ONLY thing
        # draining the link. An exception anywhere below -- a recorder write, a
        # demux callback, a malformed frame -- used to kill the thread outright.
        # Nothing logged it and nothing noticed the thread was gone; what the
        # operator saw was every reading going stale and `link_alive()` turning
        # False three seconds later.
        #
        # That is the WORST available misdiagnosis: a host-side software fault
        # presenting as a dead cable, at the pool, with the hull in the water.
        # Someone would pull the USB-C and re-seat the board while the actual
        # cause sat in this process.
        #
        # So: a fault here is logged, counted, and survivable, and the counters
        # are published so the health board can say "the READER is unwell" rather
        # than leaving "the LINK is dead" as the only available story.
        self._reader_faults = 0
        self._reader_last_fault = ''
        consec = 0
        while True:
          try:
            while True:
                msg = self.master.recv_match(blocking=False)
                if msg is None:
                    break
                # Record BEFORE the demux, and before any type filter: the
                # log's job is to hold what arrived, including messages nothing
                # on this side consumes. ESC_STATUS (291) is the live example --
                # pymavlink drops it, so it is absent from every decoded view
                # and present in every raw byte.
                #
                # `write()` queues and returns; the disk is another thread's
                # problem. This loop is the only thing draining the link and the
                # only place the NAMED_VALUE_FLOAT burst can be de-multiplexed,
                # so it must not wait on anything.
                if self._recorder is not None:
                    self._recorder.write(msg)
                fn = _DEMUX.get(msg.get_type())
                if fn is not None:
                    fn(msg)
            text = self.pixhawk.get_statustext()
            if text and text != self.last_statustext:
                self.last_statustext = text
                self.get_logger().info(f'[ARDUB] {text}')
            consec = 0
          except Exception as exc:                    # noqa: BLE001 -- see above
            self._reader_faults += 1
            consec += 1
            self._reader_last_fault = f'{type(exc).__name__}: {exc}'
            # Log the first, then back off: a persistent fault at 200 Hz would
            # bury the log it is trying to be found in.
            if consec == 1 or consec % 200 == 0:
                self.get_logger().error(
                    f'[NET  ] !! MAVLink reader fault #{self._reader_faults} '
                    f'({consec} consecutive): {self._reader_last_fault}. The link '
                    f'may be FINE -- this is a host-side fault in the reader, not '
                    f'a dead cable. Telemetry will look stale while it persists.')
            time.sleep(0.05)                          # do not spin on a hard fault
          time.sleep(0.005)   # 200 Hz drain

    # ================================================================== #
    #  Action Server callbacks                                            #
    # ================================================================== #

    def goal_callback(self, goal_request):
        accept, signal_abort = goal_acceptance(
            goal_request.cmd, self.command_active)
        if not accept:
            self.get_logger().warn(
                f'[ACT  ] Rejected {goal_request.cmd} -- command already active')
            return GoalResponse.REJECT
        if signal_abort:
            # Safety command accepted while busy: signal abort so the running
            # loop exits at its next tick, releasing the lock for us.
            self.get_logger().info(
                f'[ACT  ] {goal_request.cmd} (safety) accepted -- signalling abort')
            self.duburi.request_abort()
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('[ACT  ] Cancel requested -- stopping thrusters')
        # Signal abort FIRST so the running loop exits at its next tick (its
        # own finally neutralises lock-aware), and free the active gate so a
        # queued safety verb (disarm) gets through. Then stop the heading lock
        # (it must not outlive a cancelled goal) and send a backstop neutral.
        self.duburi.request_abort()
        self.command_active = False
        self.duburi.unlock_heading()
        self.pixhawk.send_neutral()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        request = goal_handle.request
        cmd     = request.cmd

        if cmd not in COMMANDS:
            result = Move.Result()
            result.success = False
            result.message = f'Unknown command: {cmd}'
            goal_handle.abort()
            return result

        self.command_active = True
        self.get_logger().info(f'[ACT  ] {cmd} -> EXECUTING')

        try:
            with FeedbackPump(self.pixhawk, goal_handle,
                              yaw_provider=self._effective_yaw_deg,
                              vision_provider=self.duburi.vision_telemetry):
                # Re-snapshot params for every goal so freshly-set
                # `vision.*` values land on the very next command.
                runtime = runtime_defaults_for_command(
                    cmd, snapshot_from_node(self))
                kwargs = fields_for(cmd, request, runtime_defaults=runtime)
                if self._is_srot and cmd in SROT_UNSUPPORTED_VERBS:
                    # Refuse BEFORE dispatch. Left to fall through, these reach
                    # Pixhawk-only primitives and fail in ways worse than a clean
                    # refusal -- lock_heading in particular would report success
                    # while holding nothing. See srot_fc.UNSUPPORTED_VERBS.
                    result = Move.Result()
                    result.success = False
                    result.message = (
                        f'{cmd}: not supported on the SROT backend yet '
                        f'(needs the MANUAL_CONTROL-streamed port)')
                    result.final_value = 0.0
                    result.error_value = 0.0
                elif self._is_srot and cmd in SROT_MOVE_VERBS:
                    # Collapse verb on the SROT backend: one on-board SROT_MOVE +
                    # its four-terminal ACK relay, instead of the host motion loop.
                    result = self._run_srot_move(cmd, kwargs, goal_handle)
                elif self._is_srot and cmd == 'surface':
                    result = self._run_srot_surface(kwargs)
                else:
                    method = getattr(self.duburi, cmd)
                    result = method(**kwargs)

            if result.success:
                goal_handle.succeed()
                self.get_logger().info(
                    f'[ACT  ] {cmd} -> DONE ({result.message})')
            else:
                goal_handle.abort()
                self.get_logger().error(
                    f'[ACT  ] {cmd} -> REJECTED ({result.message})')
            return result

        except Exception as exc:
            # Best-effort: surface current depth so the operator sees
            # *something*. Detail (target/current/error) is in
            # result.message.
            attitude = self.pixhawk.get_attitude()
            result = Move.Result()
            result.success     = False
            result.message     = f'{cmd}: exception -- {exc}'
            result.final_value = float(attitude['depth']) if attitude else 0.0
            result.error_value = 0.0
            goal_handle.abort()
            self.get_logger().error(f'[ACT  ] {cmd} FAILED: {exc}')

            # When a movement raises mid-loop, the per-axis cleanup
            # `stop()` is skipped, which can leave a stale Ch4 RC
            # override or SET_POSITION_TARGET setpoint active.
            # Neutralise explicitly so the next command starts from a
            # known state. Use the lock-aware writer so a still-active
            # heading lock keeps Ch4 (a raw send_neutral would clobber it
            # for one tick and the lock would just re-assert anyway).
            try:
                self.duburi._writers().neutral()
            except Exception as cleanup_exc:
                self.get_logger().warn(
                    f'[ACT  ] post-failure neutralise raised: {cleanup_exc}')
            return result

        finally:
            self.command_active = False

    def _run_srot_surface(self, kwargs):
        """Emergency surface on the SROT backend: engage the board's SURFACE mode.

        The facade's `surface` is `set_depth(0)`, which goes through
        `_ensure_alt_hold` -> ALT_HOLD, an ArduSub mode the board does not have.
        So on srot the verb raised ModeChangeError and did NOTHING -- a safety verb
        that bypasses the busy gate specifically so it can always run, then didn't.

        SURFACE (mode 9) is the board's own ascend-and-hold failsafe state: it
        drives depth::setTarget(0) through the depth PID, or an open-loop ascent
        (DEPTH_LOST_ASCENT) when there is no depth sensor -- which is exactly the
        behaviour wanted when things have gone wrong. It is also the ONE mode the
        board never blocks for a missing Bar30.

        Braked first: the board keeps running the active movement primitive until
        something displaces it, and SURFACE alone does not abort a move.
        """
        timeout = float(kwargs.get('timeout', 60.0) or 60.0)
        self.duburi._abort_event.clear()
        self.fc.stop_motion()                     # brake + cancel any running leg
        ok, reason = self.fc.set_mode('SURFACE')

        out = Move.Result()
        out.success = bool(ok)
        out.message = (f'surface: SURFACE engaged (ascending, <= {timeout:.0f}s)'
                       if ok else f'surface: could not engage SURFACE -- {reason}')
        att = self.fc.get_attitude()
        out.final_value = float(att['depth']) if att else 0.0
        out.error_value = 0.0
        if not ok:
            self.get_logger().error(f'[ACT  ] surface FAILED: {reason}')
        return out

    def _run_srot_move(self, cmd, kwargs, goal_handle):
        """Dispatch a collapse verb to the SROT board as one SROT_MOVE.

        Builds a Move.Result from the backend's MoveResult (never raises), streams
        ~3 Hz progress as action feedback, and passes the cooperative abort hook so
        a goal cancel brakes the board.

        Takes `duburi.lock` and applies the same disarmed gate as the facade's
        `_command_scope`. This path bypasses the facade entirely, so without these
        it was the ONLY dispatch route with no host-side arm check and no
        serialisation -- and the board's own pre-arm checks just IMU-healthy plus
        not-calibrating (fw arming.cpp:11-31), not depth, ESCs, leak or battery.
        The other two things `_command_scope` does are moot here: the neutral-RC
        heartbeat is never started on srot, and a deferred heading lock cannot
        exist because lock_heading is refused on this backend.
        """
        with self.duburi.lock, command_scope(cmd):
            self.duburi._abort_event.clear()
            if cmd not in DUBURI_UNARM_SAFE and not self.fc.is_armed():
                out = Move.Result()
                out.success = False
                out.message = f'{cmd}: AUV is disarmed -- call arm() first'
                out.final_value = 0.0
                out.error_value = 0.0
                return out

            def _on_progress(frac):
                fb = Move.Feedback()
                fb.phase         = cmd
                fb.current_value = float(frac)
                fb.status_line   = f'{cmd} {frac * 100:.0f}%'
                goal_handle.publish_feedback(fb)

            res = self.fc.move(cmd, on_progress=_on_progress,
                               abort_fn=self.duburi._abort_fn, **kwargs)
            out = Move.Result()
            out.success = res.ok
            out.message = res.reason
            att = self.fc.get_attitude()
            out.final_value = float(att['depth']) if att else 0.0
            out.error_value = 0.0
            return out

    # ================================================================== #
    #  Timers                                                             #
    # ================================================================== #

    def _register_health(self) -> None:
        """Translate what each subsystem already knows into one vocabulary.

        Every reporter is wrapped so a missing method is UNKNOWN rather than an
        AttributeError: the manager runs against two backends and a sim, and a
        health board that crashes the node it is watching is worse than no
        board at all.
        """
        fc = self.fc

        def named(name):
            fn = getattr(fc, '_named_value', None)
            return fn(name) if fn else None

        self._health.register('board_link', lambda: _hr.board_link(fc))
        # Registered right beside board_link ON PURPOSE (B40): when the reader
        # thread dies, board_link says "no heartbeat" and looks exactly like a
        # dead cable. This is the line that names the real side.
        self._health.register('mavlink_reader', lambda: _hr.mavlink_reader(
            lambda: bool(getattr(self, 'reader_thread', None) is not None
                         and self.reader_thread.is_alive()),
            lambda: int(getattr(self, '_reader_faults', 0)),
            lambda: str(getattr(self, '_reader_last_fault', ''))))
        self._health.register('barometer', lambda: _hr.barometer(named))
        self._health.register('heading_ref', lambda: _hr.heading_reference(named))
        # Thrust-allocator saturation. Reads UNKNOWN until srot-control-board#20
        # merges -- `mixer::mix()` computes the per-group scale-down and drops
        # it, so today a saturating allocator and a quiet one are the same from
        # here. Registered NOW so the day the names appear the line is already
        # watching, rather than the capability arriving with no consumer.
        self._health.register('allocator', lambda: _hr.allocator(
            getattr(fc, 'allocator_saturation', lambda: None)()))
        self._health.register('thrusters', lambda: _hr.thrusters(
            getattr(fc, 'thruster_health', lambda: None)()))
        self._health.register('thruster_power', lambda: _hr.thruster_power(
            getattr(fc.telemetry(), 'kill_switch', None)))
        self._health.register('detector', lambda: _hr.detector(
            self._detection_rate_hz()))
        # ⛔ LEAK. Nine reporters were defined here and six were registered; this
        # was one of the three that never ran, and it is the one written for a
        # flooding hull. It returns FAILED for `LEAK_EN = 0` -- the state this
        # board was MEASURED in -- because a disabled failsafe still reads dry,
        # so the vehicle looks safe precisely when nothing is watching.
        # It needed `fc.leak_state()`, which splits "enabled" from "leaking";
        # `sys_status_leak()` collapses both into None and cannot express the trap.
        self._health.register('leak', lambda: _hr.leak_sensor(
            *(getattr(fc, 'leak_state', lambda: (None, None))())))
        self._health.register('target_pose', lambda: _hr.target_pose(
            (lambda vs: vs.target_pose() if vs is not None else None)(
                self._live_vision_state())))
        # `target_lock` stays UNREGISTERED, deliberately. It wants the ladder's
        # decayed `authority` (0..1); `/lock` carries the rung name and a
        # CONFIDENCE in class_id/score, which is a different quantity. Deriving
        # one from the other would publish an invented number in a health line --
        # the exact failure this board exists to catch. It needs a real accessor
        # on VisionState first. Tracked as B01 in .claude/context/BUGS.md.

    def _live_vision_state(self):
        """A VisionState from the pool, or None before any vision goal has run.

        ⛔ THIS EXISTS BECAUSE `self.vision` NEVER DID. `_detection_rate_hz`
        read `getattr(self, 'vision', None)` and nothing on this class ever
        assigns `self.vision` -- VisionStates live in the `_vision_states` pool,
        keyed by camera. So the read always missed, the rate was always None,
        and the `detector` reporter -- which IS registered -- could only ever
        say UNKNOWN. The D16 signal it was built to carry (an aborted detector
        keeps every topic alive; only the RATE changes) never fired once.

        The defensive `getattr` is what hid it: a missing attribute returned a
        plausible None instead of raising, so the reporter degraded silently
        into a reporter that cannot report.

        Preference order: the pool is usually one entry; when a mission has
        touched both cameras, prefer 'forward' as the mission camera and fall
        back to whatever is there. Absence stays None -- the reporters render
        that as UNKNOWN, which is the honest answer before any goal has run.
        """
        pool = getattr(self, '_vision_states', None) or {}
        if not pool:
            return None
        return pool.get('forward') or next(iter(pool.values()))

    def _detection_rate_hz(self):
        """Detections per second, or None if we are not subscribed at all.

        THE SIGNAL D16 NEEDED. A detector that aborts stays alive with every
        topic present; the rate is the only thing that changes, and nothing was
        watching it.
        """
        vs = self._live_vision_state()
        st = getattr(vs, 'stats', None) if vs else None
        if st is None:
            return None
        try:
            return float(st().get('det_hz'))
        except Exception:                       # noqa: BLE001
            return None

    def _health_tick(self) -> None:
        self._health.poll()
        worst = self._health.worst()
        if worst is self._health_last:
            return
        self._health_last = worst
        if worst is _health.State.OK:
            self.get_logger().info('[HLTH ] all subsystems OK')
            return
        bad = '; '.join(str(h) for h in self._health.not_ok())
        log = (self.get_logger().warn
               if worst is _health.State.DEGRADED else self.get_logger().error)
        log(f'[HLTH ] {worst.name}: {bad}')

    def heartbeat_tick(self):
        self.pixhawk.send_heartbeat()

    def _verify_extnav_params(self) -> None:
        """Confirm ArduSub will actually fuse the ATT_POS_MOCAP yaw we stream.

        The feed is inert unless VISO_TYPE=1 (instantiates the MAVLink
        visual-odom backend; without it the message is dropped before the
        EKF) AND EK3_SRC1_YAW=6 (ExternalNav yaw source). Both live on the
        flight controller, not in this repo, so we read them back and WARN
        loudly on mismatch rather than silently streaming into a void. A
        None read means the param is absent -- on a Pixhawk 2.4.8 that's the
        1 MB (fmuv2) build, which strips visual-odom entirely.
        """
        log  = self.get_logger()
        # Short per-param timeout: bounds startup delay to ~2 s even when the
        # params are absent (e.g. a 1 MB build that lacks VISO_TYPE).
        viso = self.pixhawk.get_param('VISO_TYPE', timeout=1.0)
        yaw  = self.pixhawk.get_param('EK3_SRC1_YAW', timeout=1.0)

        if viso is None:
            log.warn('[SENS ] VISO_TYPE not readable -- BNO yaw will NOT be '
                     'fused. Likely a 1 MB (fmuv2) build with no visual-odom; '
                     'flash the 2 MB (fmuv3) ArduSub build.')
            return
        ok = True
        if int(round(viso)) != 1:
            ok = False
            log.warn(f'[SENS ] VISO_TYPE={viso:.0f} (need 1) -- ATT_POS_MOCAP '
                     'dropped before the EKF. Set VISO_TYPE=1.')
        if yaw is None or int(round(yaw)) != 6:
            ok = False
            shown = 'unreadable' if yaw is None else f'{yaw:.0f}'
            log.warn(f'[SENS ] EK3_SRC1_YAW={shown} (need 6=ExternalNav) -- '
                     'BNO yaw not fused. Set EK3_SRC1_YAW=6.')
        if ok:
            log.info('[SENS ] EKF external-nav yaw confirmed '
                     '(VISO_TYPE=1, EK3_SRC1_YAW=6).')

    def _vision_uplink_tick(self) -> None:
        """Send ONE LANDING_TARGET for the currently selected target, or nothing.

        SENDING NOTHING IS THE LOSS SIGNAL. `VISION_API.md` §1 has no "lost"
        flag -- a detector that sees nothing simply stops sending, and the board
        ages the last bearing out. So this must not re-send a stale sample to
        "hold" a target: that is indistinguishable from a live one on the wire
        and defeats the board's staleness timer, which is the whole safety
        mechanism on this path. Hence the early returns rather than a cached
        last-good value.
        """
        cam = str(self.get_parameter('vision_uplink_camera').value).strip()
        # Say so ON CHANGE. Without this the operator sets the parameter, gets
        # a success, and has nothing telling them whether anything started --
        # which is indistinguishable from the startup-only bug this replaced.
        if cam != getattr(self, '_uplink_cam_logged', None):
            self._uplink_cam_logged = cam
            self.get_logger().info(
                f'[VIS  ] uplink camera -> {cam!r}' if cam else
                '[VIS  ] uplink idle (vision_uplink_camera cleared)')
        if not cam:
            return
        vstate = self._vision_state_for(cam)
        if vstate is None:
            return
        want = str(self.get_parameter('vision_uplink_class').value).strip()
        sample = vstate.bbox_error(want)
        if sample is None:
            return
        w, h = vstate.image_size()
        K, D = vstate.calibration()
        # ⛔ The index comes from `duburi_vision.optics`, which exists to be
        # "the ONE place the water refractive index lives" (B22). `bearing.py`
        # lives in duburi_control, which must NOT depend on duburi_vision -- so
        # the caller supplies it rather than the library holding a second copy.
        b = bearing_from_normalised(
            sample.ex, sample.ey, sample.w_frac, sample.h_frac,
            width=w, height=h, K=K, D=D, n_medium=self._uplink_n())
        if b is None:
            # No calibration and no FOV: refuse rather than invent a bearing.
            # An uncalibrated guess on this wire is a confident wrong heading.
            if not getattr(self, '_uplink_warned', False):
                self._uplink_warned = True
                self.get_logger().warn(
                    f'[VIS  ] uplink idle: {cam} has no usable CameraInfo.k and no '
                    f'FOV, so a pixel offset cannot become a bearing. Set the '
                    f'camera_node `calibration` param.')
            return
        if not b.calibrated and not getattr(self, '_uplink_fov_warned', False):
            self._uplink_fov_warned = True
            self.get_logger().warn(
                '[VIS  ] uplink using the FOV fallback, not the calibration: '
                'bearings carry the linear-approximation error (up to ~2.5 deg '
                'on our measured lens, and ~1.3 deg at frame centre from the '
                'off-axis principal point).')
        try:
            # The frozen class map, not the detector's own index: `class_id` is
            # a property of whichever model is loaded and means something
            # different for every one. Our two senders disagreed about this
            # field -- this tick sent a hardcoded 0 while the uplink check sent
            # `d.class_id` -- so neither was a wire contract.
            self.fc.send_landing_target(
                b,
                target_num=srot_uplink_class_num(
                    getattr(sample, 'class_name', '') or want),
                coasted=bool(getattr(sample, 'coasted', False)),
                gap_age_s=float(getattr(sample, 'age_s', 0.0) or 0.0),
                board_capture_s=self._board_capture_s(sample))
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn(f'[VIS  ] landing_target send failed: {exc}')

    def _mocap_tick(self) -> None:
        """Stream BNO085 yaw to ArduSub EKF3 at 20 Hz via ATT_POS_MOCAP."""
        yaw = self.yaw_source.read_yaw()
        if yaw is None:
            return   # BNO stale — skip; EKF uses gyro integration until data resumes
        self.pixhawk.send_att_pos_mocap(yaw)

    def _fast_state_tick(self):
        """Publish heading + depth at 20 Hz for real-time HUD instruments.

        Reads fresh attitude from the Pixhawk cache (AHRS2 pinned to 50 Hz)
        and reuses the last-known armed/mode/battery from telemetry_tick.
        """
        self._publish_demand()
        attitude = self.pixhawk.get_attitude()
        if attitude is None:
            return
        yaw_deg, _ = self._effective_yaw_deg(attitude)
        msg = DuburiState()
        msg.header.stamp    = self._state_stamp()
        msg.header.frame_id = 'duburi'
        msg.armed           = self._fast_armed
        msg.mode            = self._fast_mode
        msg.yaw_deg         = float(yaw_deg) if yaw_deg is not None else math.nan
        msg.depth_m         = float(attitude['depth'])
        msg.battery_voltage = self._fast_batt_v
        self.state_publisher.publish(msg)

    def _publish_demand(self):
        """20 Hz: the thruster demand in force, or NaN when it is not known."""
        demand = getattr(self.fc, 'demand', None)
        if demand is None:
            return                       # backend has no demand funnel
        d = demand()
        m = self._Vector3Stamped()
        # ⛔ INFORMATIONAL ONLY. This is REALTIME, which can step (first NTP
        # sync on the Pi did, mid-run). The consumer times the lag on its own
        # arrival monotonic() and must never subtract these stamps.
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'duburi'
        m.vector.x, m.vector.y =(math.nan, math.nan) if d is None else d
        self.demand_publisher.publish(m)

    def _imu_rates_tick(self):
        """Publish body-frame angular rates (ATTITUDE) at 50 Hz.

        x=pitch_rate, y=roll_rate, z=yaw_rate (rad/s). The flow node buffers
        these and interpolates to each frame-pair for rotation compensation.
        Skips when ATTITUDE hasn't arrived (no stream / pre-connect).

        ⛔ THE STAMP IS THE SENDER'S CAPTURE TIME, NOT `now()`. This tick is a
        50 Hz timer polling a cache, so it is ASYNCHRONOUS to arrival: stamping
        `now()` added a uniform 0-20 ms of quantisation on top of transport
        delay, and de-rotation subtracts `f*omega*dt`, so that lands directly
        in the flow residual -- 5 ms is 1.6 px at 0.64 rad/s, and Qin & Shen
        put the whole tolerance at 6 ms.

        Measured on this vehicle: the BOARD's ATTITUDE interval is 20.00 ms
        with sd 0.00 while host ARRIVAL is 20.00 ms with sd 6.67 and p2p 35.12.
        All of the jitter is transport. `ClockMap` maps the board's own
        `time_boot_ms` onto host time from one-way pairs, fitted on the LOWER
        ENVELOPE because transport delay is strictly non-negative -- a fit
        through the middle of the cloud measures the mean delay, not the
        offset. The board does not implement MAVLink TIMESYNC (0 of 12
        requests answered, measured), so this is the available route.

        Falls back to arrival time, loudly, when the mapping is not yet
        established -- an unmapped board clock is not a host clock, and
        publishing it as one would be worse than the jitter it replaces.
        """
        rates = self.pixhawk.get_angular_rates()
        if rates is None:
            return

        stamp_s = None
        board_ms = rates.get('board_ms')
        recv_s = rates.get('host_recv_s')
        if board_ms is not None:
            board_s = board_ms * 1e-3
            if recv_s is not None:
                steps = self._imu_clock.steps
                self._imu_clock.add(board_s, recv_s)
                if self._imu_clock.steps != steps:
                    # The host clock stepped (NTP) or the board rebooted. The
                    # map dropped the other clock's pairs; stamps fall back to
                    # arrival until it refits, and `_imu_clock_ok` re-announces.
                    self._imu_clock_ok = False
                    self.get_logger().warning(
                        f'[SENS ] clock STEP detected (#{self._imu_clock.steps}): '
                        f'host or board time jumped. IMU stamps on arrival '
                        f'until the board clock is re-mapped (~2 s).')
            now = time.monotonic()
            if now - self._imu_clock_fit_t >= 2.0:
                self._imu_clock_fit_t = now
                self._imu_clock.fit()
            if self._imu_clock.ready:
                stamp_s = self._imu_clock.to_host(board_s)
                # Share it with `/duburi/state`, which carries the SAME board
                # sample (yaw, depth) and was stamped on host publish time.
                # ⛔ CACHED HERE, INSIDE the ready branch, and not after the
                # fallback below: past that point `stamp_s` may be the ARRIVAL
                # time, and caching that as a board stamp would relabel the
                # jitter instead of removing it -- the same lie in a better
                # disguise. Paired with its own monotonic instant so a stale
                # mapping cannot be applied to a fresh state message.
                self._board_stamp = (stamp_s, time.monotonic())
            elif not self._imu_clock_warned:
                self._imu_clock_warned = True
                self.get_logger().info(
                    '[SENS ] imu_rates: board clock not mapped yet, stamping '
                    'on arrival (adds ~6.7 ms sd of transport jitter). '
                    'Mapping needs a few seconds of ATTITUDE.')

        if stamp_s is None:
            stamp_s = recv_s if recv_s is not None else time.time()
        elif not self._imu_clock_ok:
            self._imu_clock_ok = True
            self.get_logger().info(
                f'[SENS ] imu_rates now stamped on the BOARD clock: '
                f'{self._imu_clock}')

        m = self._Vector3Stamped()
        m.header.stamp.sec = int(stamp_s)
        m.header.stamp.nanosec = int((stamp_s - int(stamp_s)) * 1e9)
        m.header.frame_id = 'duburi'
        m.vector.x = rates['pitch_rate']
        m.vector.y = rates['roll_rate']
        m.vector.z = rates['yaw_rate']
        self.imu_rates_publisher.publish(m)
        self._publish_imu(stamp_s)

    def _publish_imu(self, stamp_s: float) -> None:
        """Publish the 6-DoF sample on `/duburi/imu`, on the SAME mapped stamp.

        Shares `_imu_rates_tick`'s clock work rather than fitting a second
        `ClockMap`: ATTITUDE and SCALED_IMU2 are packed from one `Snap` in the
        same firmware tick (`mav_stream.cpp`), so they are the same instant and
        giving them two independently-fitted mappings would invent a lag
        between two halves of one sample.

        Silently absent when the backend has no `get_imu` (Pixhawk) or when the
        board says the IMU is unhealthy -- a filter that gets no message coasts,
        which is correct; one that gets zeros integrates a lie.
        """
        getter = getattr(self.pixhawk, 'get_imu', None)
        if getter is None:
            return
        imu = getter()
        if imu is None:
            return
        msg = self._Imu()
        msg.header.stamp.sec = int(stamp_s)
        msg.header.stamp.nanosec = int((stamp_s - int(stamp_s)) * 1e9)
        msg.header.frame_id = 'duburi'
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = imu['gyro']
        (msg.linear_acceleration.x, msg.linear_acceleration.y,
         msg.linear_acceleration.z) = imu['accel']
        # THE BOARD'S OWN ATTITUDE, and this is the whole point of the split.
        # It fuses the BNO085 at 500 Hz with the sensor on its own I2C bus and
        # holds heading to under 0.01 deg/min at rest (measured). Propagating
        # attitude again on the companion, from a gyro arriving at 50 Hz over a
        # serial link with nothing aiding it, is strictly worse -- and measured
        # on the vehicle it DIVERGED: an unaided inertial solution grows
        # through the gravity coupling, and the depth update's gain then pumps
        # that error into x and y. 7.1e6 m in 35 s.
        #
        # So the board owns attitude and the companion owns position. -1 in
        # element 0 stays the ROS "no data" convention for a board that cannot
        # supply it; the variance is the BNO's datasheet drift, not a guess
        # dressed as one.
        rpy = imu.get('rpy')
        if rpy is None:
            msg.orientation_covariance[0] = -1.0
            # SAY SO, ONCE. Without the board's attitude the filter propagates
            # its own from a 50 Hz gyro with nothing aiding it, and that was
            # MEASURED to diverge to 7.1e6 m in 35 s. Publishing a valid
            # message with the orientation quietly marked absent is exactly
            # the failure this stack keeps repeating: the consumer degrades
            # correctly and nobody is told the degraded path is live.
            if not self._imu_rpy_warned:
                self._imu_rpy_warned = True
                self.get_logger().warning(
                    '[SENS ] /duburi/imu carries NO orientation: the backend '
                    'gave no attitude with the inertial sample. Localization '
                    'will propagate attitude itself and WILL drift.')
        else:
            qw, qx, qy, qz = _quat_from_rpy(*rpy)
            msg.orientation.w, msg.orientation.x = qw, qx
            msg.orientation.y, msg.orientation.z = qy, qz
            var = math.radians(0.5) ** 2
            msg.orientation_covariance[0] = var
            msg.orientation_covariance[4] = var
            msg.orientation_covariance[8] = var
        self.imu_publisher.publish(msg)

    def _effective_yaw_deg(self, attitude):
        """Return ``(yaw_deg, label)`` -- the SAME yaw the control loops
        close on. Prefers ``yaw_source.read_yaw()`` when fresh, falls
        back to Pixhawk AHRS, degrades gracefully to ``(None, 'N/A')``.

        ``BNO085Source.read_yaw()`` already returns ``None`` when its
        stream goes stale (see ``_STALE_S`` in ``bno085.py``), so a
        yanked USB cable silently falls through to AHRS here rather
        than holding the last stale BNO value forever.
        """
        source = getattr(self, 'yaw_source', None)
        if source is not None:
            yaw = source.read_yaw()
            if yaw is not None:
                # Short-label for the [STATE] line. 'MAVLINK_AHRS' ->
                # 'AHRS' keeps the line tidy; custom sources (BNO085,
                # DVL, WITMOTION) render as-is.
                raw_name = getattr(source, 'name', 'SRC')
                label = 'AHRS' if raw_name == 'MAVLINK_AHRS' else raw_name
                return float(yaw), label
        if attitude is not None:
            # NaN IS ABSENCE HERE, and this branch used to pass it straight out.
            # `SrotFC.get_attitude` returns NaN yaw when the board reports the
            # BNO unhealthy (measured live after a bad boot); on srot there is
            # no second AHRS to fall through to, so without this check the
            # gate upstream would be a knob wired to nothing -- NaN would reach
            # `/duburi/state.yaw_deg` and `heading_lock` exactly where 0.0 used
            # to, and NaN comparisons are silently False in every guard.
            yaw = float(attitude['yaw'])
            if not math.isnan(yaw):
                return yaw, 'AHRS'
        return None, 'N/A'

    def telemetry_tick(self):
        attitude = self.pixhawk.get_attitude()
        battery  = self.pixhawk.get_battery()
        rc       = self.pixhawk.get_rc_channels()
        mode     = self.pixhawk.get_mode()
        armed    = self.pixhawk.is_armed()

        yaw_deg, yaw_label = self._effective_yaw_deg(attitude)

        self._fast_armed  = bool(armed)
        self._fast_mode   = mode or ''
        self._fast_batt_v = float(battery['voltage']) if battery else math.nan

        self._maybe_print_state(attitude, battery, mode, armed, yaw_deg, yaw_label)
        self._maybe_print_rc(rc)
        self._publish_state(attitude, battery, mode, armed, yaw_deg)
        self._publish_srot_telemetry()

    def _reapply_srot_config(self) -> None:
        """Re-push the board configuration that a reboot cleared (B43).

        Deliberately only the two things a reboot silently loses and that we set
        ourselves. Not a full re-run of `_srot_preflight_reads`: the behaviour-rev
        and yaw-reference CHECKS are diagnostics whose answers cannot change
        across a reboot of the same firmware, and re-running their round-trips
        from a 2 Hz telemetry tick would add link traffic for no information.
        """
        try:
            for msg_id, hz in SROT_MESSAGE_RATES.items():
                self.pixhawk.set_message_rate(msg_id, hz)
            gain_ok = self.fc.set_default_gain()
        except Exception as exc:                     # noqa: BLE001 -- best-effort
            self.get_logger().error(
                f'[SROT ] reconfigure after reboot FAILED: {exc!r}. The board is '
                f'running compiled defaults: MANUAL_CONTROL at half authority and '
                f'reduced stream rates.')
            return
        if gain_ok:
            self.get_logger().warning(
                '[SROT ] board restarted -- stream rates and JS_GAIN_DEFAULT '
                're-applied. Anything commanded between the reboot and now ran at '
                'half MANUAL_CONTROL authority.')
        else:
            self.get_logger().error(
                '[SROT ] board restarted and JS_GAIN_DEFAULT could NOT be '
                're-applied -- every MANUAL_CONTROL is at HALF authority until it '
                'is. Vision alignment and the arrival brake will under-correct.')

    def _publish_srot_telemetry(self):
        """Surface the parts of SrotFC.telemetry() that nothing else reads.

        `telemetry()` was fully implemented and had no production caller -- the
        manager reads the vehicle through the Pixhawk-compat surface
        (get_attitude/get_battery/...), which has no RPM or leak. So the decode
        work, including the ESC_TELEMETRY fallback, went nowhere.
        """
        # Gate on the BACKEND, not on the RPM publisher: the [SROT ] block and the
        # leak latch below have nothing to do with /duburi/esc_rpm, and tying them to
        # it meant a manager without that publisher silently lost all of them.
        if not self._is_srot:
            return

        # An unplanned FC restart, checked here for the same reason `telemetry()`
        # is: `check_for_reboot()` was fully implemented, documented and unit
        # tested, and called by NOTHING outside its own test. A detector nobody
        # runs is not a detector.
        #
        # It matters most exactly where it was missing. After a reboot the board
        # is DISARMED, in its boot mode, with every setpoint cleared and stream
        # rates back to compiled defaults -- while the mission carries on issuing
        # verbs to a vehicle that is no longer the one it configured. Each verb
        # then fails in its own way, none of them naming the cause.
        #
        # So the active command is ABORTED rather than merely logged: a mission
        # step that continues here is steering nothing, and the fail-safe default
        # is to stop and let the operator see why.
        if self.fc.check_for_reboot():          # logs the cause itself
            if self.command_active:
                self.get_logger().error(
                    '[ACT  ] aborting the active command -- the board restarted '
                    'under it, so it is disarmed and no longer configured')
                self.duburi.request_abort()
            # ...and RECONFIGURE it (B43). Aborting the running command was only
            # half the job: the very next verb ran on a board still holding its
            # COMPILED DEFAULTS, and both losses are silent.
            #
            #   JS_GAIN_DEFAULT reverts to 0.5, so every MANUAL_CONTROL -- i.e.
            #   every vision align and every brake -- runs at HALF authority. The
            #   hull just corrects more weakly; nothing reports it.
            #
            #   Stream rates revert to the compiled defaults, so ATTITUDE drops
            #   ~55 Hz -> ~11 Hz. The vision loop's freshness decay then bleeds
            #   translational authority away on a link that looks fine.
            #
            # Neither is visible in any log, and this comment's own predecessor
            # already said the board was "no longer configured" while nothing
            # acted on that.
            self._reapply_srot_config()

        try:
            tel = self.fc.telemetry()
        except Exception as exc:                      # noqa: BLE001 -- telemetry is best-effort
            self.get_logger().debug(f'[TELEM] srot telemetry read failed: {exc!r}')
            return

        # ⛔ ABSENCE, NOT ZEROS. `tel.rpm` is a tuple of eight, so `if tel.rpm`
        # is TRUE even when every slot is 0 -- and the board fills all eight
        # slots whether or not an ESC is attached (measured: 958 CRC-valid
        # frames, no ESCs, every rpm exactly 0). This topic was therefore
        # publishing eight fabricated zeros at 2 Hz on a hull with no thrusters.
        #
        # The discriminator is NOT "are the values zero" -- an idle armed hull
        # with real ESCs also reads zero, and that is genuine data. It is
        # whether the board has ANNOUNCED presence, which is the same source of
        # truth the thrusters health reporter grades. Unannounced publishes
        # nothing, so a consumer sees no message rather than a confident zero.
        present, _lost = getattr(self.fc, 'esc_presence', lambda: (None, set()))()
        if tel.rpm and present and self.esc_rpm_publisher is not None:
            msg = self._Int32MultiArray()
            msg.data = [int(r) for r in tel.rpm]
            self.esc_rpm_publisher.publish(msg)

        # LEAK is edge-latched, not spammed: the board streams it as a
        # NAMED_VALUE_FLOAT and this tick runs at 2 Hz, so an un-latched log would
        # repeat every 500 ms for the rest of the dive.
        #
        # Read it as ADVISORY. SROT multiplexes MV_STATE / LEAK / WTEMP / GAIN onto
        # one msgid and pymavlink's cache keeps only the newest message of a type,
        # so any single poll has roughly a 1-in-N chance of being the one we want --
        # a missed leak here is expected and is NOT a safety mechanism. The board's
        # own leak failsafe surfaces the vehicle regardless; this is for the
        # operator. (Raised upstream: LEAK wants its own message or a SYS_STATUS
        # sensor-health bit.)
        self._maybe_print_srot_block(tel)

        if tel.leak and not self._leak_latched:
            self._leak_latched = True
            self.get_logger().error(
                '[TELEM] LEAK reported by the board -- it surfaces on its own '
                'failsafe; abort the mission and recover the vehicle')
        elif not tel.leak:
            self._leak_latched = False

    @staticmethod
    def _tel(value, fmt='{:.2f}', suffix=''):
        """Render a telemetry numeric, or `--` when absent. NEVER renders absence as 0.

        Delegates to `srot_format` so this log block, `connect`, its dashboard and
        `--json` all format identically. They drifted once already -- the board shows a
        0..360 heading and one path was printing the raw signed value.
        """
        return _sfmt.fmt(value, fmt, suffix)

    def _log_srot_changes(self, tel):
        """One line per meaningful CHANGE, alongside the periodic block.

        The periodic block is a continuous trace you correlate against what the vehicle
        did; this is the opposite view -- what changed while nobody was watching. A mode
        flip or a sensor going absent is a single line here instead of something you have
        to spot by diffing two identical-looking blocks a minute apart.

        Absent <-> present transitions are included on purpose: the board SUPPRESSES
        values it cannot stand behind, so a barometer that stops being reported is the
        board telling you something, and a change log that only watches numbers move
        would never mention it.
        """
        fields = {
            'armed': tel.armed,
            'mode': tel.mode or None,
            'heading_deg': tel.yaw_deg if not math.isnan(tel.yaw_deg) else None,
            'depth_m': tel.depth_m if not math.isnan(tel.depth_m) else None,
            'battery_v': tel.battery_voltage,
            'thruster_v': tel.thruster_voltage,
            'water_temp_c': tel.water_temp_c,
            'depth_out': tel.depth_out,
            'depth_err_m': tel.depth_err_m,
            'mag_accuracy': tel.mag_accuracy,
            'leak': tel.leak,
            'kill': tel.kill_switch,
        }
        for lvl, _field, msg in _schg.diff(self._srot_prev_fields, fields):
            line = f'[SROT ] ~ {msg}'
            if lvl == 'CRIT':
                self.get_logger().error(line)
            elif lvl == 'WARN':
                self.get_logger().warn(line)
            else:
                self.get_logger().info(line)
        self._srot_prev_fields = fields

    def _maybe_print_srot_block(self, tel):
        """The verbose SROT telemetry block -- everything Pixhawk never had.

        Rate-limited by `srot_telemetry_period_s` (0 disables). This is deliberately
        periodic rather than on-change: for the first water test the operator wants a
        continuous trace they can correlate against what the vehicle physically did,
        and an on-change filter hides "nothing is changing", which for a depth loop is
        itself the interesting observation.
        """
        if self._srot_block_period <= 0:
            return
        now = time.time()
        if now - self._srot_block_last < self._srot_block_period:
            return
        self._srot_block_last = now

        self._log_srot_changes(tel)

        rpm = _sfmt.rpm_row(tel.rpm)
        etemp = _sfmt.esc_temp_row(tel.esc_temp_c)
        self.get_logger().info(
            f'[SROT ] BAT main {self._tel(tel.battery_voltage, "{:5.2f}", "V")} | '
            f'thruster {self._tel(tel.thruster_voltage, "{:5.2f}", "V")} | '
            f'DEPTH {self._tel(tel.depth_m, "{:+.2f}", "m")} '
            f'err {self._tel(tel.depth_err_m, "{:+.2f}", "m")} '
            f'out {self._tel(tel.depth_out, "{:+.2f}")} | '
            f'WTEMP {self._tel(tel.water_temp_c, "{:.1f}", "C")} | '
            f'MAGACC {self._tel(tel.mag_accuracy, "{:.0f}")} | '
            f'LEAK {"WET" if tel.leak else "dry"} | '
            f'KILL {_kill_text(tel.kill_switch)}')
        self.get_logger().info(f'[SROT ] RPM  {rpm}')
        if tel.esc_temp_c:
            self.get_logger().info(f'[SROT ] ESC°C{etemp}')

        # A depth loop saturated while DISARMED is the pre-arm tell that arming would
        # command full vertical thrust (mixer throttle column = -1 on all 4 verticals).
        if (not tel.armed and not math.isnan(tel.depth_out)
                and abs(tel.depth_out) >= _SROT_DEPTH_OUT_WARN):
            self.get_logger().error(
                f'[SROT ] DEPTH LOOP SATURATED while disarmed (out='
                f'{tel.depth_out:+.2f}, err={self._tel(tel.depth_err_m, "{:+.2f}", "m")}) '
                f'-- arming would command FULL vertical thrust. Check the barometer.')

    def _maybe_print_state(self, attitude, battery, mode, armed, yaw_deg, yaw_label):
        now  = time.time()
        prev = self.prev_state
        changed = (
            prev.get('arm')  != armed
            or prev.get('mode') != mode
            or abs(prev.get('yaw',   0)
                   - (yaw_deg if yaw_deg is not None else 0)) > YAW_CHANGE_THRESH
            or abs(prev.get('depth', 0)
                   - (attitude['depth'] if attitude else 0)) > DEPTH_CHANGE_THRESH
            or abs(prev.get('bat',   0)
                   - (battery['voltage'] if battery else 0)) > BAT_CHANGE_THRESH
            or (now - self.last_print_time) > FORCE_PRINT_SECONDS
        )
        if not changed:
            return

        arm_str   = 'ARM' if armed  else '---'
        # Show the yaw source label (BNO / AHRS) so the operator can
        # tell at a glance which sensor is actually driving the loops.
        if yaw_deg is not None:
            yaw_str = f'{yaw_deg:6.1f} ({yaw_label})'
        else:
            yaw_str = '   N/A'
        # Through the shared formatter: a NaN depth is ABSENT (the board suppresses
        # what it cannot stand behind), and `+nanm` on an operator's screen is neither
        # a reading nor a legible way to say "no barometer".
        depth_str = (_sfmt.fmt(attitude['depth'], '{:+6.2f}', 'm') if attitude
                     else '   N/A')
        bat_str   = (_sfmt.fmt(battery['voltage'], '{:5.1f}', 'V') if battery
                     else '  N/A')
        self.get_logger().info(
            f'[STATE] {arm_str} | {mode:<10} | '
            f'YAW:{yaw_str} | DEPTH:{depth_str} | BAT:{bat_str}')
        self.prev_state = {
            'arm':   armed, 'mode': mode,
            'yaw':   yaw_deg if yaw_deg is not None else 0,
            'depth': attitude['depth']  if attitude else 0,
            'bat':   battery['voltage'] if battery else 0,
        }
        self.last_print_time = now

    def _maybe_print_rc(self, rc):
        """Print the RC line only when an active channel actually
        changed -- otherwise the same line repeats every 0.5 s for the
        duration of a forward move and drowns the log."""
        if not rc or len(rc) < 6:
            return
        drive  = (rc[2], rc[3], rc[4], rc[5])
        active = any(abs(value - 1500) > 50 for value in drive)
        if active and drive != self.prev_rc:
            parts = []
            for label, value in zip(('Thr', 'Yaw', 'Fwd', 'Lat'), drive):
                if abs(value - 1500) > 50:
                    parts.append(f'{label}:{value}')
            self.get_logger().info('[RC   ] ' + '  '.join(parts))
            self.prev_rc = drive
        elif not active and self.prev_rc is not None:
            self.get_logger().info('[RC   ] all neutral')
            self.prev_rc = None

    # A board sample older than this is not the state we are publishing, so
    # fall back rather than stamp a fresh message with a stale capture time.
    # 0.2 s is 10 ATTITUDE periods at the pinned 50 Hz -- loose enough that an
    # ordinary scheduling hiccup does not flip the source back and forth.
    _BOARD_STAMP_MAX_AGE_S = 0.2

    def _state_stamp(self):
        """Board capture time for `/duburi/state`, or host time if unmapped.

        `/duburi/state` carries yaw and depth, which ORIGINATE ON THE BOARD and
        were stamped with the instant the host got round to publishing. The
        board's own interval has sd 0.00 ms where arrival has sd 6.67 and p2p
        35.12 -- so the stamp described the transport, not the measurement.

        `flow_node` reads depth from this topic and differences it over time
        (`_vz_down`), which put that jitter straight into a vertical speed.
        `ClockMap` is already fitted here for `/duburi/imu_rates`; this is the
        same mapping applied to the other stream that needs it.
        """
        bs = self._board_stamp
        if bs is not None and (time.monotonic() - bs[1]) <= self._BOARD_STAMP_MAX_AGE_S:
            sec = int(bs[0])
            stamp = self.get_clock().now().to_msg()
            stamp.sec = sec
            stamp.nanosec = int((bs[0] - sec) * 1e9)
            return stamp
        return self.get_clock().now().to_msg()

    def _on_odom_uplink(self, msg) -> None:
        """Forward one RIEKF body velocity to the board, stamped on ITS clock.

        The odom stamp is the filter's latest INPUT instant on this host's wall
        clock (localization maps every input through the same board<->host
        fit), so the inverse lands it on the board's `time_boot_ms` base.
        Nothing is sent until that fit is good: an unstamped velocity would be
        integrated at the wrong instant.
        """
        if not bool(self.get_parameter('velocity_uplink').value):
            return
        if not getattr(self, '_imu_clock_ok', False):
            return
        host_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if host_s <= 0.0:
            return
        cov = msg.twist.covariance
        try:
            sent = self.fc.send_speed_estimate(
                float(msg.twist.twist.linear.x), float(msg.twist.twist.linear.y),
                float(cov[0]), float(cov[7]),
                self._imu_clock.to_board(host_s))
        except Exception as exc:                            # noqa: BLE001
            self.get_logger().warn(f'[VEL  ] speed_estimate send failed: {exc}')
            return
        if sent:
            self._vel_uplink_n += 1
            if self._vel_uplink_n == 1:
                self.get_logger().info(
                    '[VEL  ] velocity uplink live: RIEKF body velocity -> board '
                    '(VISION_SPEED_ESTIMATE, board-clock stamped)')

    def _board_capture_s(self, sample):
        """The frame's capture instant on the BOARD's clock, or None.

        `sample.age_s` runs from capture on this host's clock; `ClockMap` holds
        the board<->host fit built from ATTITUDE.time_boot_ms, so the inverse
        puts the capture where the board's gyro history can find it. None until
        that fit is announced good, and None for a COASTED box, whose bearing
        is a prediction for now rather than an observation from the past --
        de-rotating it would count the same turn twice.
        """
        if getattr(sample, 'coasted', False) or not getattr(self, '_imu_clock_ok', False):
            return None
        age = float(getattr(sample, 'age_s', 0.0) or 0.0)
        if not (math.isfinite(age) and age >= 0.0):
            return None
        try:
            board_s = self._imu_clock.to_board(time.time() - age)
        except Exception:                                   # noqa: BLE001
            return None
        return board_s if math.isfinite(board_s) and board_s > 0.0 else None

    def _uplink_n(self) -> float:
        """Refractive index for the uplink bearing, from the ONE source of it.

        Refuses an unknown medium rather than defaulting: silently picking a
        medium here is picking a 33 % bearing error, and the failure would look
        like a mis-tuned gain rather than a units bug.
        """
        med = str(self.get_parameter('vision_uplink_medium').value or '').strip().lower()
        if med == 'air':
            return 1.0
        if med != 'water':
            if not getattr(self, '_uplink_medium_warned', False):
                self._uplink_medium_warned = True
                self.get_logger().error(
                    f'[VIS  ] vision_uplink_medium={med!r} is not water|air -- '
                    f'refusing to guess. Falling back to WATER, which is where '
                    f'the vehicle is; fix the parameter.')
        from duburi_vision.optics import N_WATER
        return float(N_WATER)

    def _publish_state(self, attitude, battery, mode, armed, yaw_deg):
        if attitude is None and battery is None:
            return
        msg = DuburiState()
        msg.header.stamp    = self._state_stamp()
        msg.header.frame_id = 'duburi'
        msg.armed           = bool(armed)
        msg.mode            = mode if mode else ''
        # Publish the yaw the control loops actually use (BNO when it's
        # the configured source, Pixhawk AHRS otherwise) so downstream
        # consumers of /duburi/state see the same number as [STATE].
        msg.yaw_deg         = float(yaw_deg) if yaw_deg is not None else math.nan
        msg.depth_m         = float(attitude['depth'])  if attitude else math.nan
        msg.battery_voltage = float(battery['voltage']) if battery  else math.nan
        self.state_publisher.publish(msg)


_KILL_BANNER = """\033[1;31m
╔══════════════════════════════════════════════════════════════╗
║              ██  MONGLA EMERGENCY STOP  ██                   ║
║                                                              ║
║   Signal received — stopping thrusters and disarming.        ║
╚══════════════════════════════════════════════════════════════╝\033[0m"""


def _emergency_stop(node) -> None:
    """Stop thrusters and disarm. Called from both Ctrl-C and SIGTERM paths."""
    print(_KILL_BANNER, file=sys.stderr)

    def _step(label: str, fn):
        try:
            result = fn()
            ok_sym = '\033[32m[OK]\033[0m'
            print(f'  {label:<22s} {ok_sym}', file=sys.stderr)
            return result
        except Exception as exc:
            fail_sym = '\033[33m[--]\033[0m'
            print(f'  {label:<22s} {fail_sym}  ({exc!r})', file=sys.stderr)
            return None

    _step('stop heading lock',  lambda: node.duburi._heading_lock.stop()
                                        if node.duburi._heading_lock else None)
    _step('stop heartbeat',     lambda: node.heartbeat.stop())

    # ⛔ BRAKE A RUNNING LEG FIRST, AND ON SROT `send_neutral` CANNOT DO IT (B42).
    #
    # Ctrl-C most often arrives DURING a move. A SROT_MOVE leaves the board in
    # AUTO, and in AUTO the firmware overwrites every pilot axis from the
    # movement primitive -- so the zero MANUAL_CONTROL frame `send_neutral()`
    # sends is DISCARDED (B28). The emergency stop's one "stop the thrusters"
    # step was therefore a no-op on the current backend for the exact case it
    # exists to cover, leaving `disarm()` as the only thing halting the hull:
    # motors off mid-leg rather than a commanded brake, and nothing at all if
    # the disarm is the step that fails.
    #
    # MOVE_STOP is the board's own brake, it is honoured in AUTO, and from fw
    # rev 2 it decelerates on-board. Pixhawk has no `stop_motion`, so this is
    # guarded rather than assumed.
    if hasattr(node.fc, 'stop_motion'):
        _step('brake running move', lambda: node.fc.stop_motion())
    _step('send neutral RC',    lambda: node.pixhawk.send_neutral())

    ok, reason = None, 'not attempted'
    try:
        ok, reason = node.pixhawk.disarm()
    except Exception as exc:
        reason = repr(exc)
    if ok:
        print(f'  {"disarm":<22s} \033[32m[OK]\033[0m', file=sys.stderr)
    else:
        print(f'  {"disarm":<22s} \033[33m[--]\033[0m  ({reason})', file=sys.stderr)

    # LAST of the vehicle steps, so the shutdown itself is in the log. How a run
    # ended -- whether the disarm was acknowledged, what the board said while it
    # happened -- is the part you go back to the log for, and a recorder closed
    # at the top of this function records everything except that.
    _step('close replay log',   lambda: node._recorder.stop()
                                        if getattr(node, '_recorder', None) else None)
    _step('close yaw source',   lambda: node.yaw_source.close())
    for cam, vstate in list(node._vision_states.items()):
        _step(f'close vision[{cam}]', lambda v=vstate: v.close())

    print(file=sys.stderr)


def main(args=None):
    rclpy.init(args=args)
    node = AUVManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # SIGTERM (kill command) triggers the same clean shutdown as Ctrl-C.
    def _sigterm_handler(sig, frame):
        executor.shutdown(timeout_sec=0)

    signal.signal(signal.SIGTERM, _sigterm_handler)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        _emergency_stop(node)
        # Drain executor threads before destroying the node.  Without this,
        # a timer callback (telemetry_tick / heartbeat_tick) can fire on a
        # background thread concurrently with node.destroy_node(), causing
        # "publisher's context is invalid" when the logger tries to publish
        # to /rosout after the context is torn down.
        executor.shutdown(timeout_sec=1)
        node.destroy_node()
        if rclpy.ok():          # Ctrl-C unwinds spin() which may already have shut down
            rclpy.shutdown()


if __name__ == '__main__':
    main()

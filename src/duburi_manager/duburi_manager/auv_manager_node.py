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
from duburi_control.payload import PayloadDriver                         # noqa: E402
from duburi_sensors import make_yaw_source                               # noqa: E402
from duburi_vision  import wait_vision_state_ready                       # noqa: E402

from .connection_config import (                                             # noqa: E402
    DEFAULT_MODE, NETWORK, PROFILES, resolve_mode, resolve_profile,
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
BAT_CHANGE_THRESH   = 0.2    # volts
FORCE_PRINT_SECONDS = 30.0   # always reprint even if nothing changed

# Telemetry stream rates we explicitly request from ArduSub at startup
# via MAV_CMD_SET_MESSAGE_INTERVAL. Without this ArduSub picks defaults
# (typically 4 Hz for AHRS2) which silently caps how tight our control
# loops can be.
MESSAGE_RATES = {
    mavutil.mavlink.MAVLINK_MSG_ID_AHRS2:          50,   # Hz -- yaw/depth source
    mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS:  1,
    mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS:     5,
}


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
    """

    def __init__(self, pixhawk, goal_handle, yaw_provider=None):
        self._pixhawk       = pixhawk
        self._goal_handle   = goal_handle
        self._yaw_provider  = yaw_provider
        self._stop          = threading.Event()
        self._thread        = threading.Thread(target=self._run, daemon=True)

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *_):
        self._stop.set()
        self._thread.join(timeout=1.0)

    def _run(self):
        while not self._stop.is_set():
            attitude = self._pixhawk.get_attitude()
            if attitude is not None:
                if self._yaw_provider is not None:
                    yaw_deg, _ = self._yaw_provider(attitude)
                else:
                    yaw_deg = attitude['yaw']
                yaw_str = f'{yaw_deg:.1f}' if yaw_deg is not None else 'N/A'
                feedback              = Move.Feedback()
                feedback.phase        = 'EXECUTING'
                feedback.current_value = float(attitude['depth'])
                feedback.error_value   = 0.0
                feedback.status_line   = (
                    f'YAW:{yaw_str}  DEPTH:{attitude["depth"]:+.2f}m')
                self._goal_handle.publish_feedback(feedback)
            self._stop.wait(timeout=0.4)


class AUVManagerNode(Node):
    def __init__(self):
        super().__init__('duburi_manager')
        self._setup_parameters()
        self._setup_mavlink()
        self._setup_reader_and_warmup()
        self._setup_yaw_source()
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
        self.declare_parameter('bno085_port',          'auto')
        self.declare_parameter('bno085_baud',          115200)
        self.declare_parameter('payload_port',         'auto')
        self.declare_parameter('nucleus_dvl_host',     '192.168.2.201')
        self.declare_parameter('nucleus_dvl_port',     9000)
        self.declare_parameter('nucleus_dvl_password', 'nortek')
        # dvl_auto_connect: background retry at startup; eliminates manual dvl_connect
        self.declare_parameter('dvl_auto_connect',  True)
        self.declare_parameter('dvl_retry_s',       5.0)
        # debug:=true flips per-command MAVLink trace + raises logger to DEBUG
        self.declare_parameter('debug',            False)
        # vision.use_tracks: subscribe /tracks instead of /detections (requires tracker_node)
        self.declare_parameter('vision.use_tracks', False)
        declare_vision_params(self)

        requested_mode      = str(self.get_parameter('mode').value)
        mav_device          = str(self.get_parameter('mav_device').value).strip()
        self._smooth_yaw    = bool(self.get_parameter('smooth_yaw').value)
        self._smooth_tr     = bool(self.get_parameter('smooth_translate').value)
        self._yaw_src_name  = str(self.get_parameter('yaw_source').value)
        self._bno_port      = str(self.get_parameter('bno085_port').value)
        self._bno_baud      = int(self.get_parameter('bno085_baud').value)
        self._payload_port  = str(self.get_parameter('payload_port').value)
        self._dvl_host      = str(self.get_parameter('nucleus_dvl_host').value)
        self._dvl_port      = int(self.get_parameter('nucleus_dvl_port').value)
        self._dvl_passwd    = str(self.get_parameter('nucleus_dvl_password').value)
        self._dvl_auto      = bool(self.get_parameter('dvl_auto_connect').value)
        self._dvl_retry_s   = float(self.get_parameter('dvl_retry_s').value)
        self._debug         = bool(self.get_parameter('debug').value)

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
        self._profile   = resolve_profile(
            self._mode_name, mav_device=mav_device, logger=self.get_logger())

    def _setup_mavlink(self) -> None:
        """Open MAVLink connection, wait for heartbeat, pin telemetry rates."""
        self.get_logger().info(
            f'Connecting ({self._mode_name}) -> {self._profile["conn"]} ...')
        baud_kw = {'baud': self._profile['baud']} if self._profile['baud'] else {}
        self.master  = mavutil.mavlink_connection(self._profile['conn'], **baud_kw)
        self.master.wait_heartbeat()
        self.pixhawk = Pixhawk(self.master, log=self.get_logger())
        # Pin rates so ArduSub streams at the rates we need (default ~4 Hz).
        for msg_id, hz in MESSAGE_RATES.items():
            self.pixhawk.set_message_rate(msg_id, hz)

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
        self.reader_thread = threading.Thread(
            target=self.reader_loop, daemon=True)
        self.reader_thread.start()

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

    def _setup_yaw_source(self) -> None:
        """Instantiate yaw source, print startup banner, start DVL auto-connect."""
        _DVL_SOURCES = {'dvl', 'nucleus_dvl', 'bno085_dvl', 'dvl_bno'}
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

    def _setup_vision_pool(self) -> None:
        """Initialise the lazy per-camera VisionState pool."""
        self._vision_states: dict = {}
        self._vision_lock         = threading.Lock()

    def _setup_heartbeat_and_payload(self) -> None:
        """Start heartbeat, connect payload driver, build Duburi facade."""
        self.heartbeat = Heartbeat(self.pixhawk, log=self.get_logger())
        self.heartbeat.start()

        # Exclude the actual port held by the BNO source (not the param 'auto').
        # BNO085Source._port_name / CompositeBNO._bno._port_name holds the real path.
        _bno_actual = (
            getattr(self.yaw_source, '_port_name', None)
            or getattr(getattr(self.yaw_source, '_bno', None), '_port_name', None)
        )
        _exclude: set[str] = {_bno_actual} if _bno_actual else set()
        if not _bno_actual and self._bno_port not in ('auto', ''):
            _exclude.add(self._bno_port)
        self._payload = PayloadDriver()
        _pl_port = None if self._payload_port in ('auto', '') else self._payload_port
        if self._payload.connect(port=_pl_port, exclude=_exclude):
            self.get_logger().info(
                f'[PAYLOAD] connected on {self._payload.port_path}')
        else:
            self.get_logger().info(
                '[PAYLOAD] not found — fire() calls will log-stub only')

        self.duburi = Duburi(
            self.pixhawk,
            log=self.get_logger(),
            smooth_yaw=self._smooth_yaw,
            smooth_translate=self._smooth_tr,
            yaw_source=self.yaw_source,
            vision_state_provider=self._vision_state_for,
            heartbeat=self.heartbeat,
            payload=self._payload,
        )

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

        self.create_timer(0.5,  self.heartbeat_tick,   callback_group=self.timer_group)
        self.create_timer(0.5,  self.telemetry_tick,   callback_group=self.timer_group)
        # Fast tick: 20 Hz HUD compass + depth (AHRS2 pinned to 50 Hz).
        # Separate callback group so it can fire between telemetry ticks.
        self.fast_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(0.05, self._fast_state_tick, callback_group=self.fast_group)

        if self._bno_mocap_active:
            self.create_timer(0.05, self._mocap_tick, callback_group=self.timer_group)
            self.get_logger().info('[SENS ] ATT_POS_MOCAP yaw injection active (20 Hz). '
                                   'Requires EK3_SRC1_YAW=6 in ArduSub params.')

    # ================================================================== #
    #  Vision state pool -- lazily built per camera, preflighted once     #
    # ================================================================== #

    def _vision_state_for(self, camera: str):
        """Return (and build on first call) the VisionState for `camera`.

        Cached by (camera, use_tracks) so a goal with tracking=True does not
        permanently poison the cache for subsequent goals that want tracking=False.
        Subscriptions stay alive for the rest of the process lifetime so
        repeat vision_* goals don't pay the preflight wait twice.
        """
        use_tracks = bool(self.get_parameter('vision.use_tracks').value)
        cache_key = (camera, use_tracks)

        with self._vision_lock:
            cached = self._vision_states.get(cache_key)
            if cached is not None:
                return cached
            self.get_logger().info(
                f'[VST  ] building VisionState for camera={camera!r} '
                f'use_tracks={use_tracks}')
            vstate = VisionState(self, camera=camera,
                                 use_tracks=use_tracks,
                                 logger=self.get_logger())
            # Do NOT cache until preflight passes — a cached-but-not-ready state
            # causes vision verbs to silently chase stale/empty detections
            # instead of raising a clear "camera not ready" failure.

        # Preflight outside the lock — it just polls VisionState's diags.
        try:
            wait_vision_state_ready(
                vstate, timeout=10.0, log=self.get_logger())
        except Exception as exc:
            self.get_logger().warning(
                f'[VST  ] preflight for {camera!r} did not pass within '
                f'10s: {exc!r}; vision verbs will fail until pipeline is up')
            # Return the (non-ready) state anyway so the goal can fail fast
            # with a detection error rather than blocking here indefinitely.
            return vstate

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
        while True:
            while self.master.recv_match(blocking=False) is not None:
                pass
            text = self.pixhawk.get_statustext()
            if text and text != self.last_statustext:
                self.last_statustext = text
                self.get_logger().info(f'[ARDUB] {text}')
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
        self.pixhawk.send_neutral()
        self.command_active = False  # allow disarm through before execute_callback exits
        self.duburi.request_abort()  # signal all motion loops to exit at next tick
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
                              yaw_provider=self._effective_yaw_deg):
                method = getattr(self.duburi, cmd)
                # Re-snapshot params for every goal so freshly-set
                # `vision.*` values land on the very next command.
                runtime = runtime_defaults_for_command(
                    cmd, snapshot_from_node(self))
                kwargs = fields_for(cmd, request, runtime_defaults=runtime)

                # Per-goal tracking override: if the goal sets tracking=True,
                # flip vision.use_tracks for this goal's VisionState build.
                # _vision_state_for picks it up on the next new-camera build
                # (cache key includes use_tracks). Snapshot + restore so the
                # flip is per-goal and does not poison later goals' default.
                tracking_flip   = kwargs.pop('tracking', False)
                prev_use_tracks = None
                if tracking_flip:
                    prev_use_tracks = self.get_parameter('vision.use_tracks').value
                    self.set_parameters([
                        rclpy.parameter.Parameter(
                            'vision.use_tracks',
                            rclpy.Parameter.Type.BOOL, True)])

                try:
                    result = method(**kwargs)
                finally:
                    if tracking_flip:
                        self.set_parameters([
                            rclpy.parameter.Parameter(
                                'vision.use_tracks',
                                rclpy.Parameter.Type.BOOL, bool(prev_use_tracks))])

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
            # known state.
            try:
                self.pixhawk.send_neutral()
            except Exception as cleanup_exc:
                self.get_logger().warn(
                    f'[ACT  ] post-failure neutralise raised: {cleanup_exc}')
            return result

        finally:
            self.command_active = False

    # ================================================================== #
    #  Timers                                                             #
    # ================================================================== #

    def heartbeat_tick(self):
        self.pixhawk.send_heartbeat()

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
        attitude = self.pixhawk.get_attitude()
        if attitude is None:
            return
        yaw_deg, _ = self._effective_yaw_deg(attitude)
        msg = DuburiState()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'duburi'
        msg.armed           = self._fast_armed
        msg.mode            = self._fast_mode
        msg.yaw_deg         = float(yaw_deg) if yaw_deg is not None else math.nan
        msg.depth_m         = float(attitude['depth'])
        msg.battery_voltage = self._fast_batt_v
        self.state_publisher.publish(msg)

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
            return float(attitude['yaw']), 'AHRS'
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
        depth_str = f'{attitude["depth"]:+6.2f}m' if attitude else '   N/A'
        bat_str   = f'{battery["voltage"]:5.1f}V'  if battery else '  N/A'
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

    def _publish_state(self, attitude, battery, mode, armed, yaw_deg):
        if attitude is None and battery is None:
            return
        msg = DuburiState()
        msg.header.stamp    = self.get_clock().now().to_msg()
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

    _step('close yaw source',   lambda: node.yaw_source.close())
    for (cam, _), vstate in list(node._vision_states.items()):
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

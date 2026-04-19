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
from duburi_sensors import make_yaw_source                               # noqa: E402
from duburi_vision  import wait_vision_state_ready                       # noqa: E402

from .connection_config import DEFAULT_MODE, NETWORK, PROFILES, resolve_mode  # noqa: E402
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

        # ---- ROS parameters --------------------------------------------
        self.declare_parameter('mode',             DEFAULT_MODE)
        self.declare_parameter('smooth_yaw',       False)
        self.declare_parameter('smooth_translate', False)
        self.declare_parameter('yaw_source',       'mavlink_ahrs')
        self.declare_parameter('bno085_port',      'auto')
        self.declare_parameter('bno085_baud',      115200)
        # debug:=true flips per-command MAVLink trace tags on (and the
        # logger to DEBUG so they actually print). Default off so
        # production runs stay quiet. See .claude/context/mavlink-reference.md
        # "MAVLink-trace via DEBUG logs" for the on-the-wire format.
        self.declare_parameter('debug',            False)
        # Live-tunable defaults for every vision_* command. Operators
        # tune with `ros2 param set /duburi_manager vision.kp_yaw 80.0`
        # and the next vision goal picks up the new value.
        declare_vision_params(self)

        requested_mode   = str(self.get_parameter('mode').value)
        smooth_yaw       = bool(self.get_parameter('smooth_yaw').value)
        smooth_translate = bool(self.get_parameter('smooth_translate').value)
        yaw_src_name     = str(self.get_parameter('yaw_source').value)
        bno085_port      = str(self.get_parameter('bno085_port').value)
        bno085_baud      = int(self.get_parameter('bno085_baud').value)
        debug_enabled    = bool(self.get_parameter('debug').value)

        # Wire MAVLink tracing on as early as possible -- we want every
        # frame from this point on to carry the cmd= tag. set_enabled
        # mutates a contextvar in the main thread; daemons spawned after
        # this point still see the default (False) because contextvars
        # don't propagate across threading.Thread, but their MAVLink
        # frames retain the file:func half so they are still traceable.
        if debug_enabled:
            tracing.set_enabled(True)
            try:
                import rclpy.logging
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
            except Exception as exc:
                self.get_logger().warning(
                    f'debug:=true: could not raise logger level to DEBUG ({exc}); '
                    f'tag will still apply but [MAV ] lines may not print')
        # 'auto' (the default) probes for BlueOS UDP / Pixhawk USB to
        # pick a profile; any explicit name short-circuits the probe.
        mode_name = resolve_mode(requested_mode, logger=self.get_logger())
        profile   = PROFILES[mode_name]

        # ---- MAVLink connection ----------------------------------------
        self.get_logger().info(f'Connecting ({mode_name}) -> {profile["conn"]} ...')
        baud_kw = {'baud': profile['baud']} if profile['baud'] else {}
        self.master = mavutil.mavlink_connection(profile['conn'], **baud_kw)
        self.master.wait_heartbeat()
        self.pixhawk = Pixhawk(self.master, log=self.get_logger())

        # Pin telemetry rates so ArduSub streams what we need at the
        # rates we need. Done early -- the reader thread starts below.
        for msg_id, hz in MESSAGE_RATES.items():
            self.pixhawk.set_message_rate(msg_id, hz)

        # ---- Yaw source -----------------------------------------------
        # Fail loudly if the requested source can't init -- the operator
        # picked it, the operator gets told. No silent fallback.
        try:
            self.yaw_source = make_yaw_source(
                yaw_src_name,
                pixhawk=self.pixhawk,
                port=bno085_port,
                baud=bno085_baud,
                logger=self.get_logger(),
            )
        except Exception as exc:
            self.get_logger().fatal(
                f'[SENS ] yaw_source={yaw_src_name!r} failed to init: {exc}')
            raise

        # ---- Banner ----------------------------------------------------
        yaw_tag = 'glide' if smooth_yaw       else 'snap'
        tr_tag  = 'eased' if smooth_translate else 'constant'
        yaw_src_label = self.yaw_source.name
        if yaw_src_name == 'bno085':
            yaw_src_label = f'{yaw_src_label} ({bno085_port} @ {bno085_baud})'
            offset = getattr(self.yaw_source, 'offset_deg', None)
            if offset is not None:
                yaw_src_label += f'  Earth-ref offset: {offset:+.2f} deg'

        self.get_logger().info(SEPARATOR)
        self.get_logger().info(f' DUBURI AUV MANAGER  |  mode: {mode_name}')
        if debug_enabled:
            self.get_logger().info(
                ' DEBUG TRACE: ON  -- per-command [MAV file:func cmd=verb] '
                'lines will print on every outbound MAVLink frame')
        self.get_logger().info(
            f' MAVLink: sys={self.master.target_system} '
            f'comp={self.master.target_component}  (v2.0)')
        self.get_logger().info(f' Profiles: yaw={yaw_tag}  translate={tr_tag}')
        self.get_logger().info(f' Yaw source: {yaw_src_label}')
        if mode_name in ('pool', 'laptop'):
            self.get_logger().info(
                f' Expect BlueOS "{NETWORK["endpoint"]}" -> UDP Client '
                f'{NETWORK["jetson_ip"]}:{NETWORK["mav_port"]}')
        if yaw_src_name == 'bno085' and mode_name in ('sim', 'laptop', 'desk'):
            # Operator-visible sanity hint. In the pool the BNO is bolted
            # inside the vehicle so it rotates with it and this note is
            # misleading -- suppress there. See ardusub-canon.md §4A.
            self.get_logger().info(
                ' [HINT ] BNO is the yaw source for ALL Python loops '
                '(yaw_*, lock_heading, translation heading-hold, [STATE], '
                'feedback). On a desk SITL the BNO chip does NOT move when '
                'ArduSub yaws in Gazebo -- so absolute-yaw verbs terminate '
                'only when you physically rotate the BNO board. Use '
                'yaw_source:=mavlink_ahrs for desk SITL smoke tests; keep '
                'bno085 for pool runs where the board moves with the AUV.')
        self.get_logger().info(SEPARATOR)

        # ---- VisionState pool (lazy per camera) -----------------------
        # The vision verbs ask for "the VisionState for camera X" and we
        # build it the first time it's requested. Holding the pool here
        # (one ROS2 node, one MAVLink owner) keeps subscriptions cheap
        # and shared across goals.
        self._vision_states: dict = {}
        self._vision_lock           = threading.Lock()

        # ---- Heartbeat (FS_PILOT_INPUT failsafe guard) ---------------
        # Streams an all-neutral RC override at 5 Hz whenever no other
        # writer is active so ArduSub never sees > 3 s of override
        # silence and triggers FS_PILOT_INPUT (default action: disarm).
        # The Duburi facade pauses/resumes it around every command and
        # for the lifetime of an active heading-lock.
        self.heartbeat = Heartbeat(self.pixhawk, log=self.get_logger())
        self.heartbeat.start()

        # ---- High-level facade ----------------------------------------
        self.duburi = Duburi(
            self.pixhawk,
            log=self.get_logger(),
            smooth_yaw=smooth_yaw,
            smooth_translate=smooth_translate,
            yaw_source=self.yaw_source,
            vision_state_provider=self._vision_state_for,
            heartbeat=self.heartbeat,
        )

        # ---- Callback groups ------------------------------------------
        # ReentrantCallbackGroup lets the action execute callback and the
        # timers run in parallel threads under MultiThreadedExecutor.
        self.action_group = ReentrantCallbackGroup()
        self.timer_group  = MutuallyExclusiveCallbackGroup()

        # ---- Action server --------------------------------------------
        self.command_active = False
        self.action_server  = ActionServer(
            self, Move, '/duburi/move',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_group,
        )

        # ---- Telemetry publisher --------------------------------------
        self.state_publisher = self.create_publisher(
            DuburiState, '/duburi/state', 10)

        # ---- Timers ---------------------------------------------------
        self.create_timer(0.5, self.heartbeat_tick,  callback_group=self.timer_group)
        self.create_timer(0.5, self.telemetry_tick,  callback_group=self.timer_group)

        # ---- Reader thread --------------------------------------------
        self.last_statustext = ''
        self.prev_state      = {}
        self.last_print_time = 0.0
        self.prev_rc         = None
        self.reader_thread   = threading.Thread(
            target=self.reader_loop, daemon=True)
        self.reader_thread.start()

    # ================================================================== #
    #  Vision state pool -- lazily built per camera, preflighted once     #
    # ================================================================== #

    def _vision_state_for(self, camera: str):
        """Return (and build on first call) the VisionState for `camera`.

        Subscriptions stay alive for the rest of the process lifetime so
        repeat vision_* goals don't pay the preflight wait twice.
        """
        with self._vision_lock:
            cached = self._vision_states.get(camera)
            if cached is not None:
                return cached
            self.get_logger().info(
                f'[VST  ] building VisionState for camera={camera!r}')
            vstate = VisionState(self, camera=camera, logger=self.get_logger())
            self._vision_states[camera] = vstate

        # Preflight outside the lock -- it just polls VisionState's diags.
        try:
            wait_vision_state_ready(
                vstate, timeout=10.0, log=self.get_logger())
        except Exception as exc:
            self.get_logger().warning(
                f'[VST  ] preflight for {camera!r} did not pass within '
                f'10s: {exc!r}; vision verbs will fail until pipeline is up')
        return vstate

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
        if self.command_active:
            self.get_logger().warn(
                f'[ACT  ] Rejected {goal_request.cmd} -- command already active')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('[ACT  ] Cancel requested -- stopping thrusters')
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
                              yaw_provider=self._effective_yaw_deg):
                method = getattr(self.duburi, cmd)
                # Re-snapshot params for every goal so freshly-set
                # `vision.*` values land on the very next command.
                runtime = runtime_defaults_for_command(
                    cmd, snapshot_from_node(self))
                kwargs = fields_for(cmd, request, runtime_defaults=runtime)
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


def main(args=None):
    rclpy.init(args=args)
    node = AUVManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Stop background writers BEFORE the final send_neutral so no
        # daemon thread races us by emitting another packet after the
        # shutdown frame. Order:
        #   heading-lock (Ch4 rate-override) -> heartbeat (all-neutral)
        #     -> one final send_neutral so the wire ends in a known state.
        try:
            if node.duburi._heading_lock is not None:
                node.duburi._heading_lock.stop()
        except Exception as exc:
            node.get_logger().debug(
                f'shutdown: heading_lock.stop() ignored: {exc!r}')
        try:
            node.heartbeat.stop()
        except Exception as exc:
            node.get_logger().debug(
                f'shutdown: heartbeat.stop() ignored: {exc!r}')
        node.pixhawk.send_neutral()
        try:
            node.yaw_source.close()
        except Exception as exc:
            node.get_logger().debug(
                f'shutdown: yaw_source.close() ignored: {exc!r}')
        for cam, vstate in list(node._vision_states.items()):
            try:
                vstate.close()
            except Exception as exc:
                node.get_logger().debug(
                    f'shutdown: vision_state[{cam}].close() ignored: {exc!r}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

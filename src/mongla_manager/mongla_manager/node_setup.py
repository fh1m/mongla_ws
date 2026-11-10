"""Bringing the node up: parameters, the board link, the recorder, the preflight reads.

Mixed into `AUVManagerNode` (auv_manager_node.py), which owns the board,
the publishers and the state these methods read.
"""
from __future__ import annotations

import math
import threading
import time

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from mongla_interfaces.msg import MonglaState

from mongla_control import Mongla
from mongla_control.fc._srot_base import mavutil
from mongla_control.fc.port_guard import PortGuard
from mongla_control.fc.srot_fc import SrotFC
from mongla_control.fc.srot_payload import SrotPayload
from mongla_control.fc.srot_protocol import SOURCE_COMPID as SROT_SOURCE_COMPID, SOURCE_SYSID as SROT_SOURCE_SYSID

from . import health as _health
from .connection_config import resolve_srot_profile
from .vision_tunables import declare_vision_params
from .yaw_source import BoardYaw
from .node_support import HEARTBEAT_WAIT_S, SEPARATOR, SROT_MESSAGE_RATES, _HeartbeatThread, _parse_payload_channels


class SetupMixin:
    def _setup_parameters(self) -> None:
        """Declare and read all ROS parameters; resolve the board's serial port."""
        # The board's serial device. '' (default) = find it (/dev/srot, then
        # the CH340 by-id link). Any pymavlink connection string also works:
        #   -p mav_device:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
        self.declare_parameter('mav_device',       '')
        # LANDING_TARGET vision uplink (VISION_API.md). OFF by default: the
        # board parses msgid 149 and silently DROPS it today, so this is a
        # producer built ahead of its consumer. '' = disabled; set to a camera
        # name ('forward') to stream that camera's selected target.
        # Raw MAVLink replay log (.tlog). '' = off; a tag ('gate_am') names the
        # file, which lands beside the scorecards in MONGLA_RUN_DIR. Everything
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
        # VELOCITY uplink: the RIEKF's body velocity (/mongla/odom) to the
        # board as VISION_SPEED_ESTIMATE, at the rate localization publishes
        # it. The Pi measures; the board integrates at 500 Hz and closes the
        # distance loop. OFF by default for the same reason as the bearing
        # uplink: the board drops msgid 103 until the firmware PR lands, and
        # the link is 51.8 % full at idle.
        self.declare_parameter('velocity_uplink', False)
        # POSITION uplink: the RIEKF pose as VISION_POSITION_ESTIMATE, same
        # clock and gating as velocity. OFF by default: no firmware consumes
        # msgid 102 yet. With `reset_counter` bumped on frame changes and fix
        # jumps, so a board integrating deltas never integrates a teleport.
        self.declare_parameter('position_uplink', False)
        # payload_channels: OPTIONAL per-instance labels, "<board_channel>:<name>",
        # e.g. "9:torpedo_1, 10:torpedo_2, 11:dropper_1".
        #
        # It does NOT route anything. `fire(N)` always addresses board channel N
        # (= DO_SET_SERVO param1 = the n in SERVO{n}_ROLE); these names only make
        # logs and the preflight readable. A stale label can mislabel a log line;
        # it cannot send a shot to the wrong channel, which is exactly why the
        # routing map it replaces is gone.
        #
        # MOSTLY SUPERSEDED by SERVO{n}_FUNCTION on the board, which mongla_ws now
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
        # debug:=true raises this node's logger to DEBUG
        self.declare_parameter('debug',            False)
        # allow_fw_behaviour_mismatch: proceed against firmware older than
        # srot_protocol.FW_BEHAVIOUR_REV_REQUIRED. OFF by default and it should stay
        # off. On pre-rev-2 firmware MOVE_STOP COASTS -- it applies zero braking thrust
        # -- and the host-side reverse-leg brake that used to cover that has been
        # removed, so `stop` and every abort would simply not decelerate ten-plus kilograms of hull,
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
        self.declare_parameter('allow_unknown_kill_switch', False)
        declare_vision_params(self)

        mav_device          = str(self.get_parameter('mav_device').value).strip()
        self._payload_fire_map = str(self.get_parameter('payload_fire_map').value)
        self._payload_channels = str(self.get_parameter('payload_channels').value)
        self._debug         = bool(self.get_parameter('debug').value)
        if self._debug:
            try:
                import rclpy.logging
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
            except Exception as exc:
                self.get_logger().warning(
                    f'debug:=true: could not raise logger level to DEBUG ({exc}); '
                    f'tag will still apply but [MAV ] lines may not print')

        self._profile = resolve_srot_profile(mav_device, logger=self.get_logger())

    def _setup_mavlink(self) -> None:
        """Open MAVLink connection, wait for heartbeat, pin telemetry rates."""
        self.get_logger().info(
            f'Connecting -> {self._profile["conn"]} ...')
        baud_kw = {'baud': self._profile['baud']} if self._profile['baud'] else {}
        # Identify as an ONBOARD COMPUTER (191) on srot, not as "some GCS" (pymavlink's
        # default 190). The board's LoRa bridge synthesises its filler heartbeat as
        # 255/190 -- the same identity we were using -- so the firmware could not tell
        # the companion from the ground station. Consequence: a DEAD COMPANION with Bondor
        # still connected holds the GCS failsafe open, and the vehicle station-keeps
        # when it should surface. A distinct compid is what lets the firmware key
        # FS_GCS_SYSID/FS_GCS_COMPID on us specifically (their JETSON_FEEDBACK §4).
        #
        # Safe to ship before that firmware lands: the board counts ANY heartbeat whose
        # id is not its own (`msg.compid != MAV_COMPONENT_ID || msg.sysid !=
        # MAV_SYSTEM_ID`, fw mav_commands.cpp:687), so 191 keeps the failsafe fed
        # exactly as 190 did.
        baud_kw['source_system'] = SROT_SOURCE_SYSID
        baud_kw['source_component'] = SROT_SOURCE_COMPID
        # Claim the port BEFORE opening it. On srot every open reboots the flight
        # controller (fc/port_guard.py has the measurements), so a second process
        # touching this device mid-mission is a silent disarm-and-reinit. The
        # kernel does not lock a tty; this does. A non-serial endpoint is a no-op.
        self._port_guard = PortGuard(self._profile['conn'], log=self.get_logger())
        self._port_guard.acquire()
        # A pulled cable used to be permanent (issue #12, item 1). The reader
        # caught the SerialException and retried the dead handle forever;
        # nothing reopened the port. pymavlink's `autoreconnect` reopens a serial
        # device IN PLACE (`mavserial.reset()`, same object, so every holder of
        # `self.master` stays valid) from its WRITE path -- and the heartbeat
        # thread writes at 2 Hz, so it is the reconnect driver. The by-id path
        # survives re-enumeration. Serial only: the UDP class has no such option.
        if str(self._profile['conn']).startswith('/dev/'):
            baud_kw['autoreconnect'] = True
        self.master  = mavutil.mavlink_connection(self._profile['conn'], **baud_kw)
        # Bounded, and said out loud (issue #12, item 3). wait_heartbeat() with
        # no timeout hung the node forever after "Connecting" on a powered-but-
        # silent board or the wrong device, with no further log line.
        _hb_deadline = time.monotonic() + HEARTBEAT_WAIT_S
        while self.master.wait_heartbeat(timeout=2.0) is None:
            if time.monotonic() >= _hb_deadline:
                raise RuntimeError(
                    f'no HEARTBEAT on {self._profile["conn"]} in '
                    f'{HEARTBEAT_WAIT_S:.0f}s -- wrong device, wrong baud, or a '
                    f'board that is powered but not running. Check `connect`.')
            self.get_logger().warning(
                f'[NET  ] still waiting for a HEARTBEAT on '
                f'{self._profile["conn"]} ...')
        self.fc = SrotFC(self.master, log=self.get_logger())
        # Heartbeat from here on, before a single slow bring-up step -- see
        # _HeartbeatThread for why it is not a timer.
        self._hb_thread = _HeartbeatThread(
            self.fc.send_heartbeat, log=self.get_logger(),
            alive_fn=self._heartbeat_allowed).start()
        self.fc.allow_saturated_depth_arm = bool(
            self.get_parameter('allow_saturated_depth_arm').value)
        self.fc.allow_fw_behaviour_mismatch = bool(
            self.get_parameter('allow_fw_behaviour_mismatch').value)
        self.fc.allow_unknown_kill_switch = bool(
            self.get_parameter('allow_unknown_kill_switch').value)
        # The round-trip READS (behaviour rev, JS_GAIN_DEFAULT, ...) cannot run
        # here: their replies land in `master.messages`, which only fills while
        # the reader thread drains the link, and it has not started yet. Run
        # here, `check_behaviour_rev` burned its retries and reported UNKNOWN on
        # every startup -- a bring-up check crying wolf about an un-brakeable
        # hull. They run from `_srot_preflight_reads()`, after the reader starts.
        #
        # Rate pinning does work here: `set_message_rate` is fire-and-forget.
        # Without it ATTITUDE sits at the board's 10 Hz default, the ceiling on
        # every host loop.
        for msg_id, hz in SROT_MESSAGE_RATES.items():
            self.fc.set_message_rate(msg_id, hz)

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
            from mongla_manager.srot_recorder import SrotRecorder, default_path
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
        """Start the MAVLink reader thread, then wait for the first attitude.

        The reader is the only thread that calls recv_match; every getter reads
        the cache it fills, so nothing can read the board before it starts.
        """
        self.last_statustext = ''
        self.prev_state      = {}
        self.last_print_time = 0.0
        # Fast HUD cache: armed/mode/battery reused by the 20 Hz instrument tick
        self._fast_armed  = False
        self._fast_mode   = ''
        self._fast_batt_v = math.nan
        self._start_recorder()          # BEFORE the reader -- it is the sink
        self.reader_thread = threading.Thread(
            target=self.reader_loop, daemon=True)
        self.reader_thread.start()
        # Only NOW can a round-trip read see its reply -- see _setup_mavlink.
        self._srot_preflight_reads()

        # Warmup: wait for the first ATTITUDE and a decoded mode.
        _deadline = time.monotonic() + 4.0
        while time.monotonic() < _deadline:
            if (self.fc.get_attitude() is not None
                    and self.fc.get_mode() != 'UNKNOWN'):
                break
            time.sleep(0.05)
        else:
            self.get_logger().warning(
                '[NET  ] no ATTITUDE or no decoded mode within 4s -- heading and '
                'depth read as absent until it arrives. Check `connect --watch`.')

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
        # A yaw demand the board ACCEPTS and the mixer turns into no thrust: the
        # dead band opens if PILOT_YAW_RATE falls below ~80 deg/s, which is what a
        # params reset does. Written 2026-09-23 and called by nothing until here.
        ya_ok, ya_reason = self.fc.check_yaw_authority()
        (self.get_logger().info if ya_ok else self.get_logger().warning)(
            f'[SROT ] {ya_reason}')
        # Boot announcements that change behaviour (a defaults reset halves the
        # pilot gain and disables the leak failsafe). The board sends them once,
        # in a burst at boot. On USB our own port open reboots it, so the burst
        # is normally here already; if not, ask for it again, and if it still
        # has not come, say that nothing is known rather than print nothing.
        self.fc.request_boot_announcements()
        boot = self.fc.boot_warnings()
        if boot is None:
            self.get_logger().warning(
                '[SROT ] boot announcements NOT SEEN: the board booted before the '
                'manager was listening, and this firmware does not repeat them. A '
                'params reset (half pilot gain, leak failsafe off) would not show. '
                'To read them: stop the manager, then `ros2 run mongla_manager '
                'connect --watch` -- opening the USB port reboots the board, and '
                'the burst prints.')
        for sev, txt in boot or []:
            self.get_logger().error(f'[SROT ] board said at boot (sev {sev}): {txt}')

    def _setup_yaw_source(self) -> None:
        """Heading comes from the board's own fused attitude (yaw_source.py)."""
        self.yaw_source = BoardYaw(self.fc)
        self.get_logger().info(SEPARATOR)
        self.get_logger().info(' MONGLA AUV MANAGER')
        self.get_logger().info(
            ' Autopilot: SROT board  ·  firmware: Hengla  ·  link: USB serial')
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
        self.get_logger().info(' Heading: the board\'s fused ATTITUDE')
        self.get_logger().info(SEPARATOR)

    def _preflight_payload(self) -> None:
        """Payload: board channels over MAVLink (DO_SET_SERVO), read off the board.

        There is no separate payload board. fire(N) addresses board channel N
        (= the n in SERVO{n}_ROLE); which channels are fireable is read from
        the board at bring-up, never assumed.
        """
        # A launch still passing the removed routing param must STOP, not be
        # quietly ignored: its numbers were host-side indices 1..4, and the same
        # numbers now address board channels 1..4 -- which on the default role
        # layout are the on-board ARM. Failing closed here is the whole migration.
        if self._payload_fire_map.strip():
            self._payload = None
            self.get_logger().error(
                '[PAYLOAD] REFUSING TO ARM THE PAYLOAD: `payload_fire_map` is set '
                f'({self._payload_fire_map!r}) but that parameter was REMOVED. '
                'fire(N) addresses BOARD channel N directly (N = DO_SET_SERVO '
                'param1 = the n in SERVO{n}_ROLE), so the old 1..4 channel numbers '
                'point at completely different hardware -- on the default role '
                'layout, at the manipulator arm. Delete payload_fire_map and pass '
                'the real board channels to fire(); use payload_channels:="9:torpedo_1, '
                '11:dropper_1" if you want names in the logs. Payload DISABLED.')
            return
        names = _parse_payload_channels(self._payload_channels, self.get_logger())
        self._payload = SrotPayload(self.fc, log=self.get_logger(), names=names)
        # Read every channel role off the board NOW, so a mission aimed at the
        # arm is caught on the deck rather than mid-drop.
        try:
            self._payload.preflight_roles()
        except Exception as exc:                # noqa: BLE001 -- advisory only
            self.get_logger().warning(
                f'[PAYLOAD] could not read channel roles: {exc!r} -- fire() '
                f'FAILS CLOSED on any channel whose role it cannot read')

    def _setup_vision_pool(self) -> None:
        """Initialise the lazy per-camera VisionState pool."""
        self._vision_states: dict = {}
        self._vision_lock         = threading.Lock()

    def _setup_heartbeat_and_payload(self) -> None:
        """Build the Mongla facade. The >=1 Hz MAVLink HEARTBEAT the board
        needs is sent by _hb_thread; nothing streams RC on this backend."""
        # Lazy DistanceState bridge for calc_distance (built on first use so
        # single-camera / no-distance runs never create the service clients).
        self._distance_state = None

        self.mongla = Mongla(
            self.fc,
            log=self.get_logger(),
            yaw_source=self.yaw_source,
            vision_state_provider=self._vision_state_for,
            distance_provider=self._distance_state_for,
            payload=self._payload,
        )

    def _distance_state_for(self):
        """Return (build on first call) the DistanceState bridge to the downward
        optical-flow estimator. Injected into Mongla as distance_provider."""
        if self._distance_state is None:
            from .distance_state import DistanceState
            self._distance_state = DistanceState(self, camera='downward')
            self.get_logger().info('[DIST ] DistanceState bridge built (downward)')
        return self._distance_state

    def _setup_publishers(self) -> None:
        """Every topic the node publishes about the vehicle, and the IMU clock map."""
        self.state_publisher = self.create_publisher(
            MonglaState, '/mongla/state', 10)

        # Body-frame angular rates for the downward optical-flow distance
        # estimator (rotation compensation). Vector3Stamped: x=pitch_rate,
        # y=roll_rate, z=yaw_rate (rad/s); header.stamp = sample time so the
        # vision node can interpolate the rate to each flow frame-pair. Sourced
        # from the board's ATTITUDE (pinned 50 Hz) -- see get_angular_rates.
        from geometry_msgs.msg import Vector3Stamped
        self._Vector3Stamped = Vector3Stamped
        self.imu_rates_publisher = self.create_publisher(
            Vector3Stamped, '/mongla/imu_rates', 10)
        # The FULL inertial sample, for the invariant filter in
        # mongla_localization. A SEPARATE topic from `/mongla/imu_rates`
        # deliberately: that one is the flow node's de-rotation feed, shaped
        # (pitch, roll, yaw) for its interpolator, and re-shaping it would
        # break a working consumer to save one publisher. This one is standard
        # `sensor_msgs/Imu` in body axes, which is what every estimator and
        # every ROS tool already expects.
        from sensor_msgs.msg import Imu
        self._Imu = Imu
        self.imu_publisher = self.create_publisher(Imu, '/mongla/imu', 10)
        # The demand last SENT to the thrusters (srot only): x = forward,
        # y = lateral, in [-1, 1]; NaN = unknown (a board primitive owns it, or
        # no frame is in force). The command-velocity model in
        # mongla_localization learns speed from it while flow works and aids
        # the filter with it when flow refuses. Unknown must reach the consumer
        # as unknown -- silence would read as "the last value still holds".
        self.demand_publisher = self.create_publisher(
            Vector3Stamped, '/mongla/demand', 10)
        self._imu_rpy_warned = False
        self._imu_last_ms = None
        # Board-clock -> host-clock mapping for the IMU stamp. See
        # _imu_rates_tick: the board's own interval has sd 0.00 ms where
        # arrival has sd 6.67, so the sender's clock is the better time base.
        from mongla_vision.flow.flow_timing import ClockMap
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
        self._leak_latched = False
        self._srot_block_last = 0.0
        self._srot_prev_fields = None
        self._srot_block_period = float(
            self.get_parameter('srot_telemetry_period_s').value)
        from std_msgs.msg import Int32MultiArray
        self._Int32MultiArray = Int32MultiArray
        self.esc_rpm_publisher = self.create_publisher(
            Int32MultiArray, '/mongla/esc_rpm', 10)
        # The flare order the operator sent over LoRa (fw comms/flare_order.h).
        # TRANSIENT_LOCAL so a mission that subscribes late still gets the last one;
        # the JSON carries latched_at, so a late reader can refuse an old order.
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, DurabilityPolicy
        self._String = String
        self.flare_order_publisher = self.create_publisher(
            String, '/mongla/flare_order',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._flare_key = None

    def _setup_health_and_watchdogs(self) -> None:
        """The health board, the executor liveness stamp and the mission keep-alive."""
        # HEALTH. 1 Hz, and it only speaks when the vehicle's overall state
        # CHANGES -- a board that logs every second is a board nobody reads,
        # and the transition is the event worth seeing.
        self._health = _health.HealthBoard()
        self._register_health()
        self._publish_static_frames()
        self._health_last = None
        self.create_timer(1.0, self._health_tick, callback_group=self.timer_group)
        # Issue #13: executor liveness (gates the heartbeat) and the mission
        # keep-alive watchdog. Each in its OWN group, so neither the blocking
        # timer_group callbacks nor a slow SURFACE mode change can stall the
        # liveness stamp.
        self._liveness_group = MutuallyExclusiveCallbackGroup()
        self._watchdog_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(0.2, self._stamp_executor_alive,
                          callback_group=self._liveness_group)
        self._mission = {'attached': False, 'name': '', 'seen': 0.0}
        # Undecided until the board's first heartbeat says armed or not (#98).
        self._boot_latch = 'pending'
        from std_msgs.msg import String as _String
        self.create_subscription(_String, '/mongla/mission_alive',
                                 self._on_mission_alive, 10,
                                 callback_group=self._watchdog_group)
        self.create_timer(0.5, self._mission_watchdog_tick,
                          callback_group=self._watchdog_group)
        # (No heartbeat timer: it runs on `_hb_thread`, started in _setup_mavlink,
        #  because this group also holds callbacks that block for seconds.)

    def _setup_telemetry_timers(self) -> None:
        """The 2 Hz telemetry tick, the 20 Hz state tick and the 50 Hz IMU tick."""
        self.create_timer(0.5,  self.telemetry_tick,   callback_group=self.timer_group)
        # Fast tick: 20 Hz HUD compass + depth (ATTITUDE pinned to 50 Hz).
        # Separate callback group so it can fire between telemetry ticks.
        self.fast_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(0.05, self._fast_state_tick, callback_group=self.fast_group)
        # IMU rates at 50 Hz (matches the ATTITUDE stream pin) -- the flow
        # rotation-comp is ~1:1 with the signal, so publish at full rate.
        self.create_timer(0.02, self._imu_rates_tick, callback_group=self.fast_group)

    def _setup_uplinks(self) -> None:
        """What the host tells the board: the vision target, the velocity and the position."""
        # The timer is unconditional, and that is the fix. It used to be
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
        self._pos_uplink_n = 0
        self._pos_reset = 0
        self._pos_last = None
        self.create_subscription(Odometry, '/mongla/odom',
                                 self._on_odom_uplink, 10,
                                 callback_group=self.timer_group)

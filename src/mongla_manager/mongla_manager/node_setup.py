"""Bringing the node up: parameters, the board link, the recorder, the preflight reads.

Mixed into `AUVManagerNode` (auv_manager_node.py), which owns the board,
the publishers and the state these methods read.
"""
from __future__ import annotations

import math
import threading
import time

from mongla_control import Mongla
from mongla_control.fc._srot_base import mavutil
from mongla_control.fc.port_guard import PortGuard
from mongla_control.fc.srot_fc import SrotFC
from mongla_control.fc.srot_payload import SrotPayload
from mongla_control.fc.srot_protocol import SOURCE_COMPID as SROT_SOURCE_COMPID, SOURCE_SYSID as SROT_SOURCE_SYSID

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
            alive_fn=self._executor_alive).start()
        self.fc.allow_saturated_depth_arm = bool(
            self.get_parameter('allow_saturated_depth_arm').value)
        self.fc.allow_fw_behaviour_mismatch = bool(
            self.get_parameter('allow_fw_behaviour_mismatch').value)
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

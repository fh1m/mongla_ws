#!/usr/bin/env python3
"""AUV Manager Node -- Terminal 1.

* Owns MAVLink connection + reader thread (only thread calling recv_match).
* Exposes /mongla/move as a ROS2 ActionServer.
* Publishes /mongla/state (typed MonglaState message) EVERY telemetry tick.
  ⚠ NOT "on change", which this line said and four other places repeated. The
  on-change logic (`_maybe_print_state`, with YAW/DEPTH/BAT_CHANGE_THRESH) gates
  the CONSOLE PRINT; `_publish_state` is called unconditionally from
  `telemetry_tick` and publishes at the timer rate on a depth-10 VOLATILE
  publisher. The distinction is a consumer contract: told "on change", a
  subscriber may reasonably treat each message as a transition, and every
  message here is periodic. (The measured "2939 of 2939 state-change
  publications were this pin" in `srot_fc.plausible_battery` counted LOG LINES,
  not publications -- the guard is still right, its evidence was misattributed.)
* MultiThreadedExecutor: action callbacks + timers run in parallel threads.

Dispatch is registry-driven: every entry in `mongla_control.COMMANDS`
maps to a same-named method on `Mongla`. Adding a new command means a
row in commands.py and a method on Mongla -- this file does not need
to change.

WHERE EACH PART LIVES. `AUVManagerNode` is one class in six files; the parts
are mixins, so `self` is the same node everywhere and nothing is passed
between them:

  auv_manager_node.py  the core: __init__, the reader thread, the action server,
                       dispatch, cancel/abort, shutdown, main()
  node_setup.py        bring-up: every declare_parameter, the board link, the
                       recorder, the preflight reads
  node_telemetry.py    what is published about the vehicle: /mongla/state, imu,
                       esc_rpm, demand, and the console blocks
  node_vision.py       the vision state the verbs read, and the velocity /
                       position / bearing uplinks to the board
  node_health.py       the health report and the two watchdogs
  node_support.py      the shared constants and the heartbeat thread

Two env vars are set here because each must be in place BEFORE the import it
governs: MAVLINK20 before pymavlink (the board speaks MAVLink 2 only), and the
console format before rclpy.
"""

import os
import math
import signal
import sys
import threading
import time
from collections import deque

# Drop ROS2's default `[INFO] [1776530611.533365998] [mongla_manager]:` prefix
# in favour of a compact `[INFO] <message>` so our [CMD  ]/[YAW  ]/[STATE] tags
# are the loudest thing on screen. Must be set BEFORE rclpy is imported.
os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}')
os.environ['MAVLINK20'] = '1'

import rclpy                                                             # noqa: E402
from rclpy.node import Node                                              # noqa: E402
from rclpy.action import ActionServer, CancelResponse, GoalResponse      # noqa: E402
from rclpy.callback_groups import (                                      # noqa: E402
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor                        # noqa: E402

from mongla_interfaces.action import Move                                # noqa: E402
from mongla_interfaces.msg import MonglaState                            # noqa: E402

from mongla_control import COMMANDS, fields_for                          # noqa: E402

from . import health as _health                                          # noqa: E402
from .dispatch_policy import goal_acceptance, _CommandGate               # noqa: E402
from .vision_tunables import runtime_defaults_for_command, snapshot_from_node  # noqa: E402
from .node_health import HealthMixin                                     # noqa: E402
from .node_setup import SetupMixin                                       # noqa: E402
from .node_telemetry import TelemetryMixin                               # noqa: E402
from .node_vision import VisionLinkMixin                                 # noqa: E402


# Board verbs relay the board's progress fraction as action feedback.
from mongla_control.fc.srot_fc import MOVE_VERBS as _BOARD_VERBS_WITH_PROGRESS  # noqa: E402





class FeedbackPump:
    """Stream Move.Feedback at ~2.5 Hz while a goal is executing.

    Used as a context manager so the worker thread is guaranteed to be
    joined no matter how the command exits (success, exception, or a
    cancel mid-loop).

    `yaw_provider` is an optional ``fn(attitude) -> (yaw_deg, label)``
    injected by the node so the feedback line reports the same yaw the
    verbs read. When omitted, the attitude's own yaw.

    `vision_provider` is an optional ``fn() -> (x_px, y_px) | None`` -- the live
    signed target-from-centre px the active vision verb reports. When it returns
    a tuple the feedback carries `err_x_px`/`err_y_px` (else NaN) so an operator
    can watch a vision verb converge in real time via `ros2 topic echo`.
    """

    def __init__(self, fc, goal_handle, yaw_provider=None,
                 vision_provider=None):
        self._fc              = fc
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
            attitude = self._fc.get_attitude()
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













def _is_dead_port(exc, master) -> bool:
    """A reader fault that is the CABLE, not the reader (issue #12).

    pymavlink sets `portdead` only from a failed WRITE, so the first failed
    READ after an unplug arrives as a bare SerialException ("device reports
    readiness to read but returned no data") while `portdead` is still False --
    measured on the bench, 2026-10-01. Either signal means the port is gone.
    """
    if getattr(master, 'portdead', False):
        return True
    try:
        import serial
        return isinstance(exc, (serial.SerialException, OSError))
    except ImportError:                          # pragma: no cover
        return isinstance(exc, OSError)






class AUVManagerNode(SetupMixin, HealthMixin, VisionLinkMixin, TelemetryMixin, Node):
    def __init__(self):
        super().__init__('mongla_manager')
        self._setup_parameters()
        self._setup_mavlink()
        self._setup_reader_and_warmup()
        self._preflight_payload()       # read channel roles off the board
        self._setup_yaw_source()
        self._setup_vision_pool()
        self._setup_heartbeat_and_payload()
        self._setup_action_server()

    # ------------------------------------------------------------------ #
    #  Init helpers (called once from __init__, in order)                 #
    # ------------------------------------------------------------------ #











    def _setup_action_server(self) -> None:
        """Create callback groups, action server, state publisher, and timers."""
        self.action_group = ReentrantCallbackGroup()
        self.timer_group  = MutuallyExclusiveCallbackGroup()

        # ⛔ THE GATE IS CLAIMED IN goal_callback, NOT IN execute_callback.
        #
        # It used to be a plain bool READ in `goal_callback` (the accept/reject
        # decision) and WRITTEN in `execute_callback` (the start of the motion
        # loop). Those are two different callbacks on a ReentrantCallbackGroup
        # under a MultiThreadedExecutor, and nothing held a lock across the gap --
        # so a second goal arriving in that window saw `command_active` still
        # False and was ACCEPTED. Two motion loops then drove the board at once,
        # sharing one abort flag, and whichever finished first cleared the gate
        # while the other was still commanding thrust.
        #
        # REPRODUCED: 40/40 trials at every hand-off delay from 20 ms down to
        # 1 ms (`test_command_gate.py::test_two_goals_cannot_both_claim`).
        # rclpy only sets the WIDTH of the window, not whether it exists.
        #
        # Claiming at the accept decision makes test-and-set one operation. The
        # cost is that an accepted goal which never reaches execute_callback would
        # leak the claim, so the claim carries a monotonic stamp and
        # `_reap_stale_claim()` clears it from the telemetry timer -- loudly,
        # because a leak means an assumption above is wrong.
        self._gate = _CommandGate()
        self.action_server  = ActionServer(
            self, Move, '/mongla/move',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_group,
        )

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
        self.esc_rpm_publisher = None
        self.flare_order_publisher = None
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
        from std_msgs.msg import String as _String
        self.create_subscription(_String, '/mongla/mission_alive',
                                 self._on_mission_alive, 10,
                                 callback_group=self._watchdog_group)
        self.create_timer(0.5, self._mission_watchdog_tick,
                          callback_group=self._watchdog_group)
        # (No heartbeat timer: it runs on `_hb_thread`, started in _setup_mavlink,
        #  because this group also holds callbacks that block for seconds.)
        self.create_timer(0.5,  self.telemetry_tick,   callback_group=self.timer_group)
        # Fast tick: 20 Hz HUD compass + depth (ATTITUDE pinned to 50 Hz).
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
        # the de-multiplexing can happen.
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
        note_param = getattr(self.fc, 'note_param_value', None)
        _DEMUX = {'NAMED_VALUE_FLOAT': note, 'BATTERY_STATUS': note_batt,
                  'STATUSTEXT': note_text, 'PARAM_VALUE': note_param}
        # SCALED_IMU2 + ATTITUDE: EVERY sample, not whichever one a 50 Hz timer
        # happens to find in a one-slot cache. Polling a 50 Hz stream at 50 Hz
        # aliases -- measured 2026-09-15: 384 of 2501 `/mongla/imu` messages
        # were repeats and 42.4 of the board's 44.0 Hz got out, with 40 ms holes.
        # Queued: 2160 of 2160 unique, 44.0 Hz, largest interval 30 ms (the
        # board's own 20/30 ms cadence).
        # ⛔ Created HERE, not in __init__: this thread starts before __init__
        # reaches the IMU publisher, and a later reassignment would orphan the
        # bound `append` captured below. Filled here (deque.append is atomic),
        # drained by the 50 Hz tick; maxlen bounds a stalled executor to ~1.3 s.
        self._imu_q = deque(maxlen=64)
        self._att_q = deque(maxlen=64)
        if getattr(self.fc, 'get_imu', None) is not None:
            _DEMUX['SCALED_IMU2'] = self._imu_q.append
            _DEMUX['ATTITUDE'] = self._att_q.append
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
                # Receipt instant on the MONOTONIC clock (issue #18): every age
                # SrotFC computes is measured from this, because pymavlink's own
                # `_timestamp` is wall time and the Pi's wall clock steps.
                msg._mono = time.monotonic()
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
            text = self.fc.get_statustext()
            if text and text != self.last_statustext:
                self.last_statustext = text
                self.get_logger().info(f'[ARDUB] {text}')
            if consec and getattr(self, '_port_was_dead', False) \
                    and not getattr(self.master, 'portdead', False):
                self._port_was_dead = False
                self.get_logger().warning(
                    '[NET  ] serial port REOPENED -- the board rebooted on reopen '
                    'and is DISARMED; message rates are re-pinned on its reboot.')
            consec = 0
          except Exception as exc:                    # noqa: BLE001 -- see above
            self._reader_faults += 1
            consec += 1
            self._reader_last_fault = f'{type(exc).__name__}: {exc}'
            if _is_dead_port(exc, self.master):
                # The CABLE, not the reader: a dead serial port. The heartbeat
                # thread's next write reopens it (autoreconnect, issue #12).
                if consec == 1 or consec % 200 == 0:
                    self.get_logger().error(
                        f'[NET  ] !! SERIAL PORT DEAD ({consec} reads): '
                        f'{self._reader_last_fault}. Reconnecting on the next '
                        f'heartbeat write; the board reboots on reopen and comes '
                        f'back DISARMED.')
                self._port_was_dead = True
                time.sleep(0.05)
                continue
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

    @property
    def command_active(self) -> bool:
        """Whether a command currently holds the gate.

        Read-only on purpose. Every write used to be a bare assignment from one of
        four places, which is what made the race invisible; the gate object now
        owns the transitions and the only way in is `claim()` / `release()`.
        """
        return self._gate.active

    def goal_callback(self, goal_request):
        # ONE atomic operation: decide and claim together. `goal_acceptance` stays
        # the pure policy (and stays unit-testable); the gate applies it under its
        # own lock so no second caller can observe the pre-claim state.
        accept, signal_abort = self._gate.claim(goal_request.cmd, goal_acceptance)
        if not accept:
            self.get_logger().warn(
                f'[ACT  ] Rejected {goal_request.cmd} -- command already active')
            return GoalResponse.REJECT
        if signal_abort:
            # Safety command accepted while busy: signal abort so the running
            # loop exits at its next tick, releasing the lock for us.
            self.get_logger().info(
                f'[ACT  ] {goal_request.cmd} (safety) accepted -- signalling abort')
            self.mongla.request_abort()
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('[ACT  ] Cancel requested -- stopping thrusters')
        # Remembered HERE, before the abort below: the running verb can exit
        # and reach `execute_callback`'s result before rclpy has moved the goal
        # to CANCELING, so `is_cancel_requested` alone loses the race.
        self._cancelled_goals = getattr(self, '_cancelled_goals', set())
        self._cancelled_goals.add(bytes(goal_handle.goal_id.uuid))
        # Signal abort FIRST so the running loop exits at its next tick (its
        # own finally neutralises lock-aware), and free the active gate so a
        # queued safety verb (disarm) gets through. Then a backstop neutral.
        self.mongla.request_abort()
        # ⚠ Releasing here is deliberate (a queued `disarm` must get through) and
        # it is ALSO how a cancel let an ordinary `move_*` in while the cancelled
        # loop was still winding down: the gate is freed for every verb, not just
        # the safety ones. Keep the release -- a stuck gate is worse -- but mark it
        # DRAINING, which `claim()` treats as busy for everything except a safety
        # verb. The cancelled loop clears it from its own `finally`.
        self._gate.begin_drain()
        self.fc.send_neutral()
        return CancelResponse.ACCEPT

    def _was_cancelled(self, goal_handle, wait_s: float = 0.5) -> bool:
        """True once the goal is CANCELING; waits briefly for rclpy to get there.

        `canceled()` is only a legal transition from CANCELING. The cancel
        callback has already fired the abort, so the verb may finish first;
        rclpy's own state change follows on another executor thread within
        milliseconds. Not cancelled at all: False at once, no wait.
        """
        uid = bytes(goal_handle.goal_id.uuid)
        if uid not in getattr(self, '_cancelled_goals', set()):
            return bool(goal_handle.is_cancel_requested)
        self._cancelled_goals.discard(uid)
        deadline = time.monotonic() + wait_s
        while not goal_handle.is_cancel_requested and time.monotonic() < deadline:
            time.sleep(0.005)
        return bool(goal_handle.is_cancel_requested)

    def execute_callback(self, goal_handle):
        request = goal_handle.request
        cmd     = request.cmd

        if cmd not in COMMANDS:
            result = Move.Result()
            result.success = False
            result.message = f'Unknown command: {cmd}'
            goal_handle.abort()
            return result

        # Already claimed in goal_callback. Confirm the hand-off arrived so a leak
        # shows up here rather than as a mystery rejection three verbs later.
        self._gate.confirm(cmd)
        try:
            gid = bytes(bytearray(goal_handle.goal_id.uuid)).hex()[:8]
        except Exception:                   # noqa: BLE001 -- a log field only
            gid = '?'
        self.get_logger().info(f'[ACT  ] {cmd} -> EXECUTING (goal {gid})')

        try:
            with FeedbackPump(self.fc, goal_handle,
                              yaw_provider=self._effective_yaw_deg,
                              vision_provider=self.mongla.vision_telemetry):
                # Re-snapshot params for every goal so freshly-set
                # `vision.*` values land on the very next command.
                snap = snapshot_from_node(self)
                runtime = runtime_defaults_for_command(cmd, snap)
                self.mongla.vision_mixer_aware = bool(
                    snap.get('vision.mixer_aware', True))
                kwargs = fields_for(cmd, request, runtime_defaults=runtime)
                method = getattr(self.mongla, cmd)
                if cmd in _BOARD_VERBS_WITH_PROGRESS:
                    kwargs['on_progress'] = self._progress_relay(cmd, goal_handle)
                result = method(**kwargs)

            if result.success:
                goal_handle.succeed()
                self.get_logger().info(
                    f'[ACT  ] {cmd} -> DONE ({result.message})')
            elif self._was_cancelled(goal_handle):
                # ⛔ A CANCELLED GOAL ENDS CANCELED, not ABORTED (issue #22's
                # executor test found it). The action contract has two terminal
                # failure states for a reason: "you asked me to stop" and "I
                # could not do it" are different facts, and a bag or a tool
                # reading the status could not tell an operator's cancel from a
                # failed verb.
                goal_handle.canceled()
                self.get_logger().warning(
                    f'[ACT  ] {cmd} -> CANCELED ({result.message})')
            else:
                goal_handle.abort()
                self.get_logger().error(
                    f'[ACT  ] {cmd} -> REJECTED ({result.message})')
            return result

        except Exception as exc:
            # Best-effort: surface current depth so the operator sees
            # *something*. Detail (target/current/error) is in
            # result.message.
            attitude = self.fc.get_attitude()
            result = Move.Result()
            result.success     = False
            result.message     = f'{cmd}: exception -- {exc}'
            result.final_value = float(attitude['depth']) if attitude else math.nan
            result.error_value = 0.0
            goal_handle.abort()
            self.get_logger().error(f'[ACT  ] {cmd} FAILED: {exc}')

            # A verb that raised mid-loop skipped its own neutral, so the
            # last MANUAL_CONTROL demand may still be in force. Zero it so
            # the next verb starts from a known state.
            try:
                self.fc.send_neutral()
            except Exception as cleanup_exc:
                self.get_logger().warn(
                    f'[ACT  ] post-failure neutralise raised: {cleanup_exc}')
            return result

        finally:
            self._gate.release()

    @staticmethod
    def _progress_relay(cmd, goal_handle):
        """The board's progress fraction, relayed as action feedback."""
        def relay(frac):
            fb = Move.Feedback()
            fb.phase         = cmd
            fb.current_value = float(frac)
            fb.status_line   = f'{cmd} {frac * 100:.0f}%'
            goal_handle.publish_feedback(fb)
        return relay

    # ================================================================== #
    #  Timers                                                             #
    # ================================================================== #






        # `target_lock` stays UNREGISTERED, deliberately. It wants the ladder's
        # decayed `authority` (0..1); `/lock` carries the rung name and a
        # CONFIDENCE in class_id/score, which is a different quantity. Deriving
        # one from the other would publish an invented number in a health line --
        # the exact failure this board exists to catch. It needs a real accessor
        # on VisionState first. Tracked in DEFERRED_FUNCTIONS
        # (mongla_vision/test/test_no_function_is_built_and_uncalled.py).

    # ================================================================== #
    #  Vision state pool -- lazily built per camera, preflighted once     #
    # ================================================================== #




















    # A board sample older than this is not the state we are publishing, so
    # fall back rather than stamp a fresh message with a stale capture time.
    # 0.2 s is 10 ATTITUDE periods at the pinned 50 Hz -- loose enough that an
    # ordinary scheduling hiccup does not flip the source back and forth.
    _BOARD_STAMP_MAX_AGE_S = 0.2








_KILL_BANNER = """\033[1;31m
╔══════════════════════════════════════════════════════════════╗
║              ██  MONGLA EMERGENCY STOP  ██                   ║
║                                                              ║
║   Signal received — stopping thrusters and disarming.        ║
╚══════════════════════════════════════════════════════════════╝\033[0m"""


# How long the Ctrl-C path waits for the board to confirm the disarm. Kept
# below the SIGTERM grace bringup.launch.py gives this process
# (`MANAGER_SIGTERM_TIMEOUT_S` there), with room for the brake before it.
EMERGENCY_DISARM_S = 8.0

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

    # ⛔ STEP 0: SIGNAL ABORT, BEFORE ANY HARDWARE STEP. Every host loop
    # (`vision_align`/`vision_move` at 20-50 Hz, `style_roll`, a queued
    # `_fire_async` shot) exits only on `_abort_event`, and this path never set
    # it: the thrusters were braked and disarmed while an action thread kept
    # streaming MANUAL_CONTROL and a delayed torpedo could still leave. It goes
    # first because it cannot fail on the wire and every later step can.
    _step('signal abort',       lambda: node.mongla.request_abort())

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
    # rev 2 it decelerates on-board.
    _step('brake running move', lambda: node.fc.stop_motion())
    _step('send neutral',       lambda: node.fc.send_neutral())

    ok, reason = None, 'not attempted'
    try:
        # ⛔ BOUNDED UNDER THE LAUNCH'S GRACE (issue #21). `disarm()` polls up
        # to 15 s by default, and `ros2 launch` escalates SIGINT to SIGTERM
        # after its grace -- 5 s unless set -- so a slow confirmation was
        # killed mid-poll and the operator never saw whether it took.
        # bringup gives the manager MANAGER_SIGTERM_TIMEOUT_S; this stays under.
        ok, reason = node.fc.disarm(timeout=EMERGENCY_DISARM_S)
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


_PR_SET_PDEATHSIG = 1


def _set_parent_death_signal(sig) -> bool:
    """Ask the kernel to send `sig` when our parent exits. Linux only."""
    try:
        import ctypes
        libc = ctypes.CDLL(None, use_errno=True)
        return libc.prctl(_PR_SET_PDEATHSIG, int(sig), 0, 0, 0) == 0
    except Exception:                       # noqa: BLE001 -- non-Linux: no-op
        return False


def _orphan_guard(node, executor, parent_pid: int):
    """Handler for the parent-death signal: stop if the parent really is gone.

    ⛔ AN ORPHANED MANAGER IS THE WORST PROCESS ON THE VEHICLE (2026-10-03, on
    the Pi). `ros2 launch` died on SIGTERM and left every child running: the
    manager kept the board's port AND kept its heartbeat going, so the board's
    companion-loss failsafe never fired, with nothing attached that could stop
    it. When the parent exits the kernel now signals us, and we take the same
    path as SIGTERM -- brake, neutral, disarm.

    The parent pid is compared because PR_SET_PDEATHSIG fires on the death of
    the parent THREAD that forked us; a stray signal with the same parent
    still alive is ignored rather than disarming a running vehicle.
    """
    def handler(sig, frame):
        if os.getppid() == parent_pid:
            return
        node.get_logger().error(
            '[KILL ] parent process died -- this manager is ORPHANED. '
            'Emergency stop: brake, neutral, disarm.')
        executor.shutdown(timeout_sec=0)
    return handler


def main(args=None):
    rclpy.init(args=args)
    parent_pid = os.getppid()
    node = AUVManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # SIGTERM (kill command) triggers the same clean shutdown as Ctrl-C.
    def _sigterm_handler(sig, frame):
        executor.shutdown(timeout_sec=0)

    signal.signal(signal.SIGTERM, _sigterm_handler)
    signal.signal(signal.SIGUSR1, _orphan_guard(node, executor, parent_pid))
    _set_parent_death_signal(signal.SIGUSR1)
    if os.getppid() != parent_pid:          # died while we were starting
        _orphan_guard(node, executor, parent_pid)(signal.SIGUSR1, None)

    try:
        # Inside the try, so a refusal still runs the teardown below: the
        # board link is already open by now.
        from mongla_localization.params_file import refuse_unknown
        refuse_unknown(node)   # bringup.yaml `nodes:` typos stop here
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        _emergency_stop(node)
        # Drain executor threads before destroying the node.  Without this,
        # a timer callback (telemetry_tick / _health_tick) can fire on a
        # background thread concurrently with node.destroy_node(), causing
        # "publisher's context is invalid" when the logger tries to publish
        # to /rosout after the context is torn down.
        executor.shutdown(timeout_sec=1)
        # Last, after the disarm above: the link must stay fed while it runs.
        # `getattr` because `_setup_mavlink` can raise before the thread exists,
        # and an AttributeError here would replace the real error with a
        # teardown one -- the same class of mistake as safety rule 6.
        hb = getattr(node, '_hb_thread', None)
        if hb is not None:
            hb.stop()
        node.destroy_node()
        if rclpy.ok():          # Ctrl-C unwinds spin() which may already have shut down
            rclpy.shutdown()


if __name__ == '__main__':
    main()

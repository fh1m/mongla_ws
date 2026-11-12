#!/usr/bin/env python3
"""AUV Manager Node -- Terminal 1.

* Owns MAVLink connection + reader thread (only thread calling recv_match).
* Exposes /mongla/move as a ROS2 ActionServer.
* Publishes /mongla/state (typed MonglaState message) EVERY telemetry tick.
  NOT "on change", which this line said and four other places repeated. The
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

Where each part lives. `AUVManagerNode` is one class in seven files; the
parts are mixins, so `self` is the same node everywhere and nothing is passed
between them:

  auv_manager_node.py  the core: __init__ (the bring-up order), the reader
                       thread, shutdown, main()
  node_action.py       the /mongla/move action server: accepting a goal,
                       cancelling it, dispatching the verb, its feedback
  node_setup.py        bring-up: every declare_parameter, the board link, the
                       recorder, the preflight reads, and every publisher,
                       timer and uplink subscription
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
import signal
import sys
import time
from collections import deque

# Drop ROS2's default `[INFO] [1776530611.533365998] [mongla_manager]:` prefix
# in favour of a compact `[INFO] <message>` so our [CMD  ]/[YAW  ]/[STATE] tags
# are the loudest thing on screen. Must be set BEFORE rclpy is imported.
os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}')
os.environ['MAVLINK20'] = '1'

import rclpy                                                             # noqa: E402
from rclpy.node import Node                                              # noqa: E402
from rclpy.executors import MultiThreadedExecutor                        # noqa: E402

from .node_health import HealthMixin                                     # noqa: E402
from .node_setup import SetupMixin                                       # noqa: E402
from .node_telemetry import TelemetryMixin                               # noqa: E402
from .node_vision import VisionLinkMixin                                 # noqa: E402
from .node_action import ActionMixin                                     # noqa: E402


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


class AUVManagerNode(ActionMixin, SetupMixin, HealthMixin, VisionLinkMixin, TelemetryMixin,
                     Node):
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
        self._setup_publishers()
        self._setup_health_and_watchdogs()
        self._setup_telemetry_timers()
        self._setup_uplinks()

    # ------------------------------------------------------------------ #
    #  Init helpers (called once from __init__, in order)                 #
    # ------------------------------------------------------------------ #

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
        # Created HERE, not in __init__: this thread starts before __init__
        # reaches the IMU publisher, and a later reassignment would orphan the
        # bound `append` captured below. Filled here (deque.append is atomic),
        # drained by the 50 Hz tick; maxlen bounds a stalled executor to ~1.3 s.
        self._imu_q = deque(maxlen=64)
        self._att_q = deque(maxlen=64)
        if getattr(self.fc, 'get_imu', None) is not None:
            _DEMUX['SCALED_IMU2'] = self._imu_q.append
            _DEMUX['ATTITUDE'] = self._att_q.append
        # The body is guarded, and the thread's death is observable (B40).
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
                self._link_meter.note(msg, msg._mono)
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

    # Step 0: Signal abort, before any hardware step. Every host loop
    # (`vision_align`/`vision_move` at 20-50 Hz, `style_roll`, a queued
    # `_fire_async` shot) exits only on `_abort_event`, and this path never set
    # it: the thrusters were braked and disarmed while an action thread kept
    # streaming MANUAL_CONTROL and a delayed torpedo could still leave. It goes
    # first because it cannot fail on the wire and every later step can.
    _step('signal abort',       lambda: node.mongla.request_abort())

    # Brake a running leg first, and on SROT `send_neutral` cannot do it (B42).
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
        # Bounded under the launch's grace (issue #21). `disarm()` polls up
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

    An orphaned manager is the worst process on the vehicle (2026-10-03, on
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
    # Before the node exists, so the reader, heartbeat and executor threads it
    # starts inherit the priority (measured-bars 76).
    from mongla_localization import realtime
    took = realtime.apply(rclpy.logging.get_logger('mongla_manager'))
    node = AUVManagerNode()
    node.get_logger().info(f'[RT   ] {took}')
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

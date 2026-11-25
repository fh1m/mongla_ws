"""The helm: the one way a browser drives the vehicle, and the log of who did.

    ros2 run mongla_batighor helm             # bringup starts it with batighor:=true

Serves `/mongla/batighor/helm` (`mongla_interfaces/srv/Helm`). An operator's
browser calls it through `foxglove_bridge`, which relays service calls and
cannot tell browsers apart, so each request carries the name its operator
typed. The rules of the token are `seat.py`; this node adds the vehicle:

  * a verb is checked against `mongla_control/commands.py` -- the verb must
    exist and every field must be one of that verb's own, of the right type --
    and only then sent to `/mongla/move` without waiting on it;
  * a mission is started as `mission <name>` in its own process group, so
    `abort` reaches it the way Ctrl-C would (it cancels, stops and disarms);
  * when the pilot goes silent, a motion verb they sent by hand that is still
    running is followed by `stop`, and whether the manager took it is logged.
    Another operator's `surface`, `stop` or `disarm` is left to finish. A mission is not stopped: a dropped laptop must not
    end a run the vehicle is flying by itself;
  * `tune` sets one of a detector's live settings (`seat.TUNABLE`) for any
    named operator while no mission runs, and `camera` makes one camera the
    live one for the pilot -- the same operation as the DSL's `use_camera`:
    the latched `/mongla/vision/active_camera`, the other detector paused.
    The bridge gives browsers no parameter access of their own
    (`launch/batighor.launch.py`, BRIDGE_LIMITS), so these are the only way;
  * `link` takes one browser's queueing lag and sets the cameras' viewer
    stream by `link.py`'s policy: halved while any browser lags, stepped back
    after a calm spell. Anyone, named or not: it drives nothing, and the
    pictures are what starve the vehicle's state on a slow link;
  * `calibrate` passes one op to a running calibration
    (`mongla_vision/calibration/calibrate_node.py`), by the same rule as
    `tune`: any named operator, never during a mission.

The terminal is trusted and does not come here (owner, 2026-11-12): `mongla`,
a mission started in a shell and the TUI send to `/mongla/move` directly.

Publishes `/mongla/batighor/helm_state` (latched JSON: pilot, asker, mission,
the live camera and each detector's settings, read once a second)
and `/mongla/batighor/log` (one JSON line per request and per change of
pilot), and appends the same lines to `~/.mongla/batighor/helm.log`.
"""
from __future__ import annotations

import json
import math
import os
import re
import signal
import threading
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import String

from mongla_batighor.link import StreamGovernor
from mongla_batighor.seat import ANYONE, CAMERAS, TUNABLE, Seat, Verdict
from mongla_interfaces.action import Move
from mongla_interfaces.srv import Helm

LOG_PATH = Path.home() / '.mongla' / 'batighor' / 'helm.log'
MISSION_ALIVE_S = 1.5     # the runner beats /mongla/mission_alive at 2 Hz
# Read, never set here: `paused` is the camera's, `models` the registry
# `active_model` picks from (the panel offers its keys).
POLLED = (*TUNABLE, 'paused', 'models')
ACTIVE_CAMERA = '/mongla/vision/active_camera'
CALIBRATION_OP = '/mongla/vision/calibration/op'


def param_value(v):
    """A ParameterValue as plain Python; None when the node has no such setting."""
    return {ParameterType.PARAMETER_DOUBLE: lambda: v.double_value,
            ParameterType.PARAMETER_INTEGER: lambda: v.integer_value,
            ParameterType.PARAMETER_STRING: lambda: v.string_value,
            ParameterType.PARAMETER_BOOL: lambda: v.bool_value}.get(v.type, lambda: None)()


def param_msg(name: str, value) -> ParamMsg:
    if isinstance(value, bool):
        pv = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    elif isinstance(value, float):
        pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
    else:
        pv = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=str(value))
    return ParamMsg(name=name, value=pv)


def tune_value(name: str, raw: str):
    """(value, '') for a setting typed as JSON, or (None, why not)."""
    try:
        value = json.loads(raw)
    except ValueError:
        return None, f'{raw!r} is not a value'
    if TUNABLE[name] is float:
        if isinstance(value, bool) or not isinstance(value, (int, float)) \
                or not math.isfinite(value):
            return None, f'{name} must be a number'
        return float(value), ''
    if not isinstance(value, str):
        return None, f'{name} must be text'
    return value, ''


TELEOP_TOPIC = '/mongla/batighor/teleop'
TELEOP_SAY_S = 3.0
# One press: a second of thrust at a gentle gain, or a fifteen-degree pivot.
# Short enough that a released key stops the vehicle within a step.
TELEOP_STEP_S, TELEOP_GAIN, TELEOP_YAW_DEG = 1.0, 40.0, 15.0


def teleop_step(linear_x: float, angular_z: float):
    """(cmd, fields_json) for the pad's press, or None when no key is held.
    Surge wins over yaw when both are held."""
    if linear_x > 0.5 or linear_x < -0.5:
        cmd = 'move_forward' if linear_x > 0 else 'move_back'
        return cmd, json.dumps({'duration': TELEOP_STEP_S, 'gain': TELEOP_GAIN})
    if angular_z > 0.5 or angular_z < -0.5:
        cmd = 'yaw_left' if angular_z > 0 else 'yaw_right'
        return cmd, json.dumps({'target': TELEOP_YAW_DEG})
    return None


def build_goal(cmd: str, fields_json: str):
    """A `Move.Goal` for one verb, or the reason there is none.

    Only the verb's own fields from `COMMANDS` are accepted, each converted to
    the type the action declares; `set_fields` lists them, so a 0.0 the
    operator typed is a zero and not "unset" (Move.action, issue #14).
    """
    from mongla_control.commands import COMMANDS

    if cmd not in COMMANDS:
        return None, f'unknown verb {cmd!r}'
    try:
        fields = json.loads(fields_json) if fields_json.strip() else {}
    except ValueError as exc:
        return None, f'fields are not JSON: {exc}'
    if not isinstance(fields, dict):
        return None, 'fields must be a JSON object'
    allowed = COMMANDS[cmd]['fields']
    # An empty box is not a zero: with no field listed the manager would read
    # the rosidl 0.0 as asked for (north, the surface, the pool origin).
    missing = [f for f in COMMANDS[cmd].get('required', ()) if f not in fields]
    if missing:
        return None, f'{cmd} needs {", ".join(missing)}: it has no default'
    types = Move.Goal.get_fields_and_field_types()
    goal = Move.Goal()
    goal.cmd = cmd
    given = []
    for name, value in fields.items():
        if name not in allowed:
            return None, f'{cmd} has no field {name!r} (it takes: {", ".join(allowed) or "none"})'
        kind = types[name]
        if kind in ('float', 'double'):
            if isinstance(value, bool) or not isinstance(value, (int, float)) \
                    or not math.isfinite(value):
                return None, f'{name} must be a number'
            value = float(value)
        elif kind == 'boolean':
            if not isinstance(value, bool):
                return None, f'{name} must be true or false'
        elif kind == 'string':
            if not isinstance(value, str):
                return None, f'{name} must be text'
        else:
            return None, f'{name} cannot be set from the helm ({kind})'
        setattr(goal, name, value)
        given.append(name)
    goal.set_fields = given
    return goal, ''


LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# A viewer opened after the events (every fresh browser) would show an empty log
# card while helm.log held days of them. foxglove_bridge hands a new subscriber
# ONE latched message, so the history travels as one: the last HISTORY_LINES
# lines as a JSON array on LOG_TAIL, re-latched on every entry.
HISTORY_LINES = 200
LOG_TAIL = '/mongla/batighor/log_tail'

class HelmNode(Node):
    def __init__(self, log_path=None):
        super().__init__('batighor_helm')
        self.log_path = Path(log_path) if log_path else LOG_PATH
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._last_state = ''
        self.seat = Seat()
        self.move = ActionClient(self, Move, '/mongla/move')
        self.hand_goals = {}            # goal id -> (operator, cmd)
        self.mission = None             # (name, operator, Popen)
        self.mission_heard = ('', 0.0)  # (name, monotonic)
        self.pub_state = self.create_publisher(
            String, '/mongla/batighor/helm_state', LATCHED)
        self.pub_log = self.create_publisher(String, '/mongla/batighor/log', 50)
        self.pub_tail = self.create_publisher(String, LOG_TAIL, LATCHED)
        self._record = threading.Lock()     # `_log` runs on the list thread as well
        self._tail = self._read_tail()
        self.pub_tail.publish(String(data=json.dumps(self._tail)))
        self.create_subscription(String, '/mongla/mission_alive',
                                 self._on_mission_alive, 10)
        self.create_service(Helm, '/mongla/batighor/helm', self._on_request)
        from geometry_msgs.msg import Twist
        self.create_subscription(Twist, TELEOP_TOPIC, self._on_teleop, 10)
        self._teleop_said = 0.0
        self.create_timer(0.2, self._tick)
        self.missions = []
        threading.Thread(target=self._list_missions, daemon=True).start()
        # The detectors: their settings, read once a second, and the camera.
        self.tuning = {}                # camera -> {setting: value}
        self.camera = ''
        self._get = {c: self.create_client(GetParameters, f'/mongla_detector_{c}/get_parameters')
                     for c in CAMERAS}
        self._set = {c: self.create_client(SetParameters, f'/mongla_detector_{c}/set_parameters')
                     for c in CAMERAS}
        self.pub_camera = self.create_publisher(String, ACTIVE_CAMERA, LATCHED)
        self.create_subscription(String, ACTIVE_CAMERA,
                                 lambda m: setattr(self, 'camera', m.data), LATCHED)
        self.create_timer(1.0, self._poll)
        self._calibration = self.create_client(Helm, CALIBRATION_OP)
        # The viewers' video (link.py): each camera node found by who
        # publishes its stream, its own rate read once as the top.
        self.link = StreamGovernor()
        self._stream = {}               # node path -> {'get', 'set', 'top'}

    # -- the record ---------------------------------------------------
    def _log(self, **entry):
        entry = {'t': round(time.time(), 3), **entry}
        line = json.dumps(entry, sort_keys=True)
        self.pub_log.publish(String(data=line))
        with open(self.log_path, 'a') as f:
            f.write(line + '\n')
        with self._record:
            self._tail = (self._tail + [line])[-HISTORY_LINES:]
            self.pub_tail.publish(String(data=json.dumps(self._tail)))

    def _read_tail(self):
        try:
            return self.log_path.read_text().splitlines()[-HISTORY_LINES:]
        except OSError:
            return []

    def _flush_seat(self):
        for what in self.seat.events:
            self._log(event=what)
        self.seat.events.clear()

    def _on_teleop(self, msg):
        """Lichtblick's pad: one short step per press, as the seated pilot.

        The pad cannot say who pressed it, so it drives only while a pilot
        holds the seat and is beating, only between the pilot's own verbs (a
        held key sends the next step when the last has ended) and never
        under a mission. Anything else is refused once every few seconds in
        the log, not silently."""
        step = teleop_step(msg.linear.x, msg.angular.z)
        if step is None:
            return
        cmd, fields = step
        pilot, now = self.seat.pilot, time.monotonic()
        why = ('nobody holds the helm' if not pilot
               else f'mission {self._mission_running()} runs' if self._mission_running()
               else '')
        if why:
            if now - self._teleop_said > TELEOP_SAY_S:
                self._teleop_said = now
                self._log(event=f'pad {cmd} refused: {why}', ok=False)
            return
        if any(who == pilot for who, _ in self.hand_goals.values()):
            return                      # the last step still runs
        v = self._command(pilot, cmd, fields, [])
        self._log(operator=pilot, event=f'pad: {cmd} {fields}: {v.message}', ok=v.ok)

    def _mission_running(self) -> str:
        name, heard = self.mission_heard
        if name and time.monotonic() - heard < MISSION_ALIVE_S:
            return name
        if self.mission and self.mission[2].poll() is None:
            return self.mission[0]
        return ''

    def _publish_state(self, force=False):
        state = json.dumps({
            'pilot': self.seat.pilot, 'asker': self.seat.requester,
            'answer_by': round(self.seat.request_deadline - time.monotonic(), 1)
            if self.seat.requester else None,
            'mission': self._mission_running(),
            'missions': self.missions,
            'camera': self.camera,
            'tuning': self.tuning,
            'video': self.link.scale,
            'video_by': self.link.by,
        }, sort_keys=True)
        if force or state != self._last_state:
            self._last_state = state
            self.pub_state.publish(String(data=state))

    # -- requests -----------------------------------------------------
    def _on_request(self, req, res):
        # Every request is answered. A raise here (no `ros2` on PATH, an
        # unwritable log folder, mongla_vision absent) left the browser with
        # no reply and could take the helm, and the pilot's seat, down with it.
        try:
            return self._handle(req, res)
        except Exception as exc:   # noqa: BLE001 -- the reply is the report
            res.ok, res.message = False, f'{req.op} failed in the helm: {type(exc).__name__}: {exc}'
            try:
                self._log(operator=req.who.strip(), op=req.op, cmd=req.cmd,
                          ok=False, message=res.message)
            except Exception:      # noqa: BLE001 -- the log itself may be what failed
                pass
            return res

    def _handle(self, req, res):
        now = time.monotonic()
        who, op = req.who.strip(), req.op
        if op == 'heartbeat':
            v = self.seat.heartbeat(who, now)
        elif op == 'request':
            v = self.seat.request(who, now)
        elif op == 'give':
            v = self.seat.give(who, now)
        elif op == 'release':
            v = self.seat.release(who, now)
        elif op == 'command':
            v = self._command(who, req.cmd, req.fields_json, list(req.confirms))
        elif op == 'mission':
            v = self._start_mission(who, req.mission)
        elif op == 'abort':
            v = self._abort_mission(who)
        elif op == 'tune':
            v = self._tune(who, req.cmd, req.fields_json)
        elif op == 'camera':
            v = self._camera(who, req.cmd)
        elif op == 'calibrate':
            v = self._calibrate(who, req.cmd, req.fields_json)
        elif op == 'link':
            v = self._link(req.fields_json)
        else:
            v = Verdict(False, f'unknown op {op!r}')
        res.ok, res.message = v.ok, v.message
        # A beat and a lag report come every second or two from every browser;
        # only what they change is logged.
        if op not in ('heartbeat', 'link'):
            self._log(operator=who, op=op, cmd=req.cmd, fields=req.fields_json,
                      mission=req.mission, confirms=list(req.confirms),
                      ok=v.ok, message=v.message)
        self._flush_seat()
        self._publish_state()
        return res

    def _command(self, who, cmd, fields_json, confirms):
        v = self.seat.may_send(who, cmd, confirms)
        if not v.ok:
            return v
        goal, why = build_goal(cmd, fields_json)
        if goal is None:
            return Verdict(False, why)
        if not self.move.server_is_ready():
            return Verdict(False, 'the manager is not answering /mongla/move')
        fut = self.move.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._accepted(f, who, cmd))
        return Verdict(True, f'{cmd} sent to the manager (it may still refuse it: see the log)')

    def _accepted(self, fut, who, cmd):
        handle = fut.result()
        if handle is None or not handle.accepted:
            self._log(operator=who, event=f'{cmd} was refused by the manager')
            return
        # Keyed by the goal's id: a ClientGoalHandle is not hashable, and
        # keyed by the handle this table stayed empty, so a silent pilot's
        # running verb was never followed by stop (test_helm_node).
        key = bytes(handle.goal_id.uuid)
        self.hand_goals[key] = (who, cmd)
        handle.get_result_async().add_done_callback(
            lambda r: self._finished(key, r))

    def _finished(self, key, fut):
        who, cmd = self.hand_goals.pop(key, ('?', '?'))
        result = fut.result().result
        self._log(operator=who, event=f'{cmd} ended', ok=bool(result.success),
                  message=result.message)

    def _list_missions(self):
        # The runner's own list, asked once. Not imported: mongla_planner
        # depends on mongla_manager, which launches this package, so an
        # import would be a cycle colcon refuses.
        try:
            run = subprocess.run(['ros2', 'run', 'mongla_planner', 'mission', '--list'],
                                 capture_output=True, text=True, timeout=30)
            if run.returncode:
                self._log(event=f'could not list the missions: exit {run.returncode}: '
                                f'{run.stderr.strip()[-200:]}')
                return
            self.missions = sorted(l.strip() for l in run.stdout.splitlines() if l.strip())
        except (OSError, subprocess.TimeoutExpired) as exc:
            self._log(event=f'could not list the missions: {exc!r}')

    def _start_mission(self, who, name):
        v = self.seat.may_send(who, 'mission')
        if not v.ok:
            return v
        if not re.fullmatch(r'[A-Za-z0-9_]+', name or ''):
            return Verdict(False, f'{name!r} is not a mission name')
        if self.missions and name not in self.missions:
            return Verdict(False, f'no mission {name!r}')
        running = self._mission_running()
        if running:
            return Verdict(False, f'mission {running} is already running')
        # The child keeps its own copy of the file; the helm's is closed here,
        # or every run would hold one descriptor open for the helm's life.
        with open(self.log_path.parent / f'mission_{time.time_ns()}_{name}.log', 'w') as out:
            proc = subprocess.Popen(['ros2', 'run', 'mongla_planner', 'mission', name],
                                    stdout=out, stderr=subprocess.STDOUT,
                                    start_new_session=True)
        self.mission = (name, who, proc)
        return Verdict(True, f'mission {name} started (pid {proc.pid})')

    def _abort_mission(self, who):
        if not self.mission or self.mission[2].poll() is not None:
            running = self._mission_running()
            if running:
                return Verdict(False, f'{running} was started from a terminal; '
                                      f'stop or disarm it from here instead')
            return Verdict(False, 'no mission is running')
        # Ctrl-C, exactly: the runner's abort path cancels, stops and disarms.
        try:
            os.killpg(self.mission[2].pid, signal.SIGINT)
        except ProcessLookupError:          # it exited since the poll above
            return Verdict(False, 'no mission is running')
        return Verdict(True, f'SIGINT sent to mission {self.mission[0]}')

    # -- the detectors -------------------------------------------------
    def _poll(self):
        # A camera that respawned, or joined, while the video is scaled down
        # comes up at its launch rate: say the scaled one again, quietly.
        # (A respawned node keeps the top read before, which is its own.)
        if self.link.scale < 1.0:
            self._apply_stream(quiet=True)
        for cam, cli in self._get.items():
            if not cli.service_is_ready():
                self.tuning.pop(cam, None)
                continue
            fut = cli.call_async(GetParameters.Request(names=list(POLLED)))
            fut.add_done_callback(lambda f, cam=cam: self._polled(cam, f))

    def _polled(self, cam, fut):
        res = fut.result()
        if res is not None and len(res.values) == len(POLLED):
            self.tuning[cam] = {n: param_value(v) for n, v in zip(POLLED, res.values)}
        else:
            self.tuning.pop(cam, None)      # no answer: do not show the old values as current

    def _set_param(self, who, cam, name, value):
        fut = self._set[cam].call_async(
            SetParameters.Request(parameters=[param_msg(name, value)]))
        fut.add_done_callback(lambda f: self._was_set(who, cam, name, value, f))

    def _was_set(self, who, cam, name, value, fut):
        res = fut.result()
        r = res.results[0] if res is not None and res.results else None
        if r is not None and r.successful:
            self._log(operator=who, event=f'{cam} {name} is now {value!r}')
            if name == 'paused' and value is False:
                self.pub_camera.publish(String(data=cam))
                self.camera = cam
        else:
            self._log(operator=who, event=f'{cam} {name} refused by the detector: '
                                          f'{r.reason if r is not None else "no answer"}')

    def _tune(self, who, target, raw):
        v = self.seat.may_tune(who, self._mission_running())
        if not v.ok:
            return v
        cam, _, name = (target or '').partition('.')
        if cam not in CAMERAS or name not in TUNABLE:
            return Verdict(False, f'{target!r} is not camera.setting; settings: '
                                  f'{", ".join(TUNABLE)}')
        value, why = tune_value(name, raw)
        if why:
            return Verdict(False, why)
        if not self._set[cam].service_is_ready():
            return Verdict(False, f'the {cam} detector is not running')
        self._set_param(who, cam, name, value)
        return Verdict(True, f'{cam} {name} -> {value!r} sent')

    def _camera(self, who, cam):
        v = self.seat.may_send(who, 'camera')
        if not v.ok:
            return v
        if cam not in CAMERAS:
            return Verdict(False, f'no camera {cam!r}; cameras: {", ".join(CAMERAS)}')
        if not self._set[cam].service_is_ready():
            return Verdict(False, f'the {cam} detector is not running')
        # What the DSL's use_camera does: the HUD follows, one detector infers.
        # The camera is the live one once its detector says it runs
        # (`_was_set`); until then the panel must not light it.
        for other in CAMERAS:
            if other != cam and self._set[other].service_is_ready():
                self._set_param(who, other, 'paused', True)
        self._set_param(who, cam, 'paused', False)
        return Verdict(True, f'asked {cam} to be the live camera')

    # -- the viewers' video -------------------------------------------
    def _link(self, fields_json):
        try:
            got = json.loads(fields_json or '{}')
            lag, screen = got.get('lag_s'), got.get('screen')
        except (ValueError, AttributeError):
            lag, screen = None, None
        # A page's own short id; cleaned, since anyone may report.
        screen = re.sub(r'[^0-9a-z]', '', str(screen or '').lower())[:12] or 'unnamed'
        if isinstance(lag, bool) or not isinstance(lag, (int, float)) \
                or not math.isfinite(lag) or lag < 0:
            return Verdict(False, 'link takes {"lag_s": seconds}')
        scale = self.link.report(float(lag), time.monotonic(), screen)
        if scale is not None:
            self._log(event=f'viewer video at x{scale:g} of each camera\'s rate: '
                            f'screen {screen} reported {lag:.1f} s of lag'
                            + (' (close it if it is not needed)' if self.link.by == screen and scale < 1 else ''))
            self._apply_stream()
        return Verdict(True, f'video x{self.link.scale:g}')

    def _stream_nodes(self):
        for cam in CAMERAS:
            for info in self.get_publishers_info_by_topic(f'/mongla/vision/{cam}/stream/compressed'):
                ns = info.node_namespace.rstrip('/')
                yield f'{ns}/{info.node_name}'

    def _apply_stream(self, quiet=False):
        for path in self._stream_nodes():
            s = self._stream.get(path)
            if s is None:
                s = self._stream[path] = {
                    'get': self.create_client(GetParameters, f'{path}/get_parameters'),
                    'set': self.create_client(SetParameters, f'{path}/set_parameters'),
                    'top': None}
            if s['top'] is None:
                fut = s['get'].call_async(GetParameters.Request(names=['stream_rate_hz']))
                fut.add_done_callback(lambda f, path=path: self._stream_top(path, f, quiet))
            else:
                self._stream_rate(path, quiet)

    def _stream_top(self, path, fut, quiet=False):
        res = fut.result()
        top = param_value(res.values[0]) if res is not None and res.values else None
        if not isinstance(top, (int, float)) or isinstance(top, bool) or top <= 0:
            self._log(event=f'{path} gave no usable stream_rate_hz (absent, not a positive number, or the read failed)')
            return
        self._stream[path]['top'] = float(top)
        self._stream_rate(path, quiet)

    def _stream_rate(self, path, quiet=False):
        s = self._stream[path]
        hz = round(self.link.rate(s['top']), 2)
        fut = s['set'].call_async(SetParameters.Request(parameters=[param_msg('stream_rate_hz', hz)]))
        if not quiet:
            fut.add_done_callback(lambda f: self._was_set('helm', path, 'stream_rate_hz', hz, f))

    # -- a calibration ------------------------------------------------
    def _calibrate(self, who, cal_op, fields_json):
        from mongla_vision.calibration.calibrate_node import OPS
        v = self.seat.may_tune(who, self._mission_running())
        if not v.ok:
            return v
        if cal_op not in OPS:
            return Verdict(False, f'{cal_op!r} is not a calibration op; ops: {", ".join(OPS)}')
        if not self._calibration.service_is_ready():
            return Verdict(False, 'no calibration is running (ros2 launch '
                                  'mongla_batighor calibrate.launch.py)')
        fut = self._calibration.call_async(
            Helm.Request(who=who, op=cal_op, fields_json=fields_json or '{}'))
        fut.add_done_callback(lambda f: self._calibrated(who, cal_op, f))
        return Verdict(True, f'calibration {cal_op} sent')

    def _calibrated(self, who, cal_op, fut):
        try:
            res = fut.result()
        except Exception:          # noqa: BLE001 -- reported below as no answer
            res = None
        self._log(operator=who, event=f'calibration {cal_op}: '
                                      f'{res.message if res is not None else "no answer"}',
                  ok=bool(res is not None and res.ok))

    def _stop_for_lost_pilot(self, pilot, running):
        what = ', '.join(running)
        if not self.move.server_is_ready():
            self._log(operator=pilot, event=f'pilot lost with {what} running: '
                                            'stop NOT sent, the manager is not answering',
                      ok=False)
            return
        goal, _ = build_goal('stop', '')
        self.move.send_goal_async(goal).add_done_callback(
            lambda f: self._log(operator=pilot,
                                event=f'pilot lost with {what} running: stop '
                                      + ('accepted' if f.result() is not None
                                         and f.result().accepted else 'REFUSED'),
                                ok=bool(f.result() is not None and f.result().accepted)))

    def _on_mission_alive(self, msg):
        self.mission_heard = (msg.data, time.monotonic())

    # -- the clock ----------------------------------------------------
    def _tick(self):
        if self.mission and self.mission[2].poll() is not None:
            name, who, proc = self.mission
            self.mission = None
            self.mission_heard = ('', 0.0)  # its last beat is not a mission running now
            self._log(operator=who, event=f'mission {name} ended, exit code {proc.returncode}')
        pilot = self.seat.pilot
        if self.seat.tick(time.monotonic()):
            # Only the lost pilot's own motion. A watcher's surface or disarm
            # must finish: a stop after surface takes the hull back out of
            # SURFACE and holds it at depth.
            running = [cmd for who, cmd in self.hand_goals.values()
                       if who == pilot and cmd not in ANYONE]
            if running and not self._mission_running():
                self._stop_for_lost_pilot(pilot, running)
        self._flush_seat()
        self._publish_state()



def main(args=None):
    from mongla_runtime.params_file import refuse_unknown
    from mongla_runtime.spin import spin
    rclpy.init(args=args)
    node = HelmNode()
    refuse_unknown(node)       # bringup.yaml `nodes:` typos stop here
    node._publish_state(force=True)
    try:
        # Every callback returns at once (service calls finish in their
        # done-callbacks), so the single-threaded events executor fits.
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

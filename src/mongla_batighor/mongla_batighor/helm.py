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
  * when the pilot goes silent, a verb they sent by hand that is still running
    is followed by `stop`. A mission is not stopped: a dropped laptop must not
    end a run the vehicle is flying by itself.

The terminal is trusted and does not come here (owner, 2026-11-12): `mongla`,
a mission started in a shell and the TUI send to `/mongla/move` directly.

Publishes `/mongla/batighor/helm_state` (latched JSON: pilot, asker, mission)
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
from std_msgs.msg import String

from mongla_batighor.seat import Seat, Verdict
from mongla_interfaces.action import Move
from mongla_interfaces.srv import Helm

LOG_PATH = Path.home() / '.mongla' / 'batighor' / 'helm.log'
MISSION_ALIVE_S = 1.5     # the runner beats /mongla/mission_alive at 2 Hz


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
        self.create_subscription(String, '/mongla/mission_alive',
                                 self._on_mission_alive, 10)
        self.create_service(Helm, '/mongla/batighor/helm', self._on_request)
        self.create_timer(0.2, self._tick)
        self.missions = []
        threading.Thread(target=self._list_missions, daemon=True).start()

    # -- the record ---------------------------------------------------
    def _log(self, **entry):
        entry = {'t': round(time.time(), 3), **entry}
        line = json.dumps(entry, sort_keys=True)
        self.pub_log.publish(String(data=line))
        with open(self.log_path, 'a') as f:
            f.write(line + '\n')

    def _flush_seat(self):
        for what in self.seat.events:
            self._log(event=what)
        self.seat.events.clear()

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
        }, sort_keys=True)
        if force or state != self._last_state:
            self._last_state = state
            self.pub_state.publish(String(data=state))

    # -- requests -----------------------------------------------------
    def _on_request(self, req, res):
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
        else:
            v = Verdict(False, f'unknown op {op!r}')
        res.ok, res.message = v.ok, v.message
        if op != 'heartbeat':
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
        return Verdict(True, f'{cmd} sent')

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
            out = subprocess.run(['ros2', 'run', 'mongla_planner', 'mission', '--list'],
                                 capture_output=True, text=True, timeout=30).stdout
            self.missions = sorted(l.strip() for l in out.splitlines() if l.strip())
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
        out = open(self.log_path.parent / f'mission_{int(time.time())}_{name}.log', 'w')
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
        os.killpg(self.mission[2].pid, signal.SIGINT)
        return Verdict(True, f'mission {self.mission[0]} interrupted')

    def _on_mission_alive(self, msg):
        self.mission_heard = (msg.data, time.monotonic())

    # -- the clock ----------------------------------------------------
    def _tick(self):
        if self.mission and self.mission[2].poll() is not None:
            name, who, proc = self.mission
            self.mission = None
            self._log(operator=who, event=f'mission {name} ended, exit code {proc.returncode}')
        if self.seat.tick(time.monotonic()):
            running = [cmd for _, cmd in self.hand_goals.values()]
            if running and not self._mission_running():
                goal, _ = build_goal('stop', '')
                self.move.send_goal_async(goal)
                self._log(event=f'pilot lost with {", ".join(running)} running: stop sent')
        self._flush_seat()
        self._publish_state()



def main(args=None):
    rclpy.init(args=args)
    node = HelmNode()
    node._publish_state(force=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

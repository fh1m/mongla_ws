"""The deck: Batighor in a terminal, for the person standing at the pool.

    ros2 run mongla_batighor deck            # on Kutum, inside mosh + tmux

One screen: the vehicle (armed, mode, depth, heading, battery), who holds the
browsers' helm and which mission runs, and the command log as it happens.
Keys act at once, because these are the ones that must never wait:

    s  stop        u  surface        d  disarm        q  quit
    :  any verb, typed as  move_forward duration=2 gain=40

The terminal is trusted (owner, 2026-11-12): the deck sends to /mongla/move
itself, as `mongla` does, and does not need the browsers' helm. What it sends
goes into the same command log, marked as the terminal's.

A terminal suits this job better than a browser: it runs over a link too weak
for video, reads in sunlight, and needs nothing installed on a laptop.
"""
from __future__ import annotations

import json
import math
import time

KEYS = {'s': 'stop', 'u': 'surface', 'd': 'disarm'}


def parse_line(line: str):
    """`move_forward duration=2 gain=40` -> ('move_forward', '{"duration": 2, ...}').

    A word that is not name=value raises ValueError, said in words.

    Values are read as JSON where they parse (numbers, true, false) and kept as
    text otherwise, so `set_mode target_name=DEPTH_HOLD` needs no quotes.
    `build_goal` then holds them to the verb's own fields and types.
    """
    words = line.strip().lstrip(':').split()
    if not words:
        return None, '{}'
    fields = {}
    for word in words[1:]:
        name, sep, raw = word.partition('=')
        if not sep or not name:
            raise ValueError(f'{word!r} is not name=value')
        try:
            fields[name] = json.loads(raw)
        except ValueError:
            fields[name] = raw
    return words[0], json.dumps(fields)


def _num(v, fmt):
    return '--' if v is None or (isinstance(v, float) and math.isnan(v)) else fmt % v


def render(width: int, vehicle, helm: dict, log_lines: list, prompt: str = '') -> list:
    """The screen as lines of at most `width` characters.

    Absence renders `--`, never 0 (CLAUDE.md section 2): a depth nobody
    measured must not read as the surface.
    """
    if vehicle is None:
        v = 'vehicle  --   no /mongla/state yet'
    else:
        v = (f"vehicle  {'ARMED' if vehicle.armed else 'disarmed'}  "
             f"{vehicle.mode or '--'}  depth {_num(vehicle.depth_m, '%.2f m')}  "
             f"heading {_num(vehicle.yaw_deg, '%.0f')}  "
             f"battery {_num(vehicle.battery_voltage, '%.1f V')}")
    pilot = helm.get('pilot') or 'nobody'
    h = f"helm     {pilot}"
    if helm.get('asker'):
        h += f"   ({helm['asker']} is asking)"
    h += f"   mission {helm.get('mission') or '--'}"
    lines = [v, h, '-' * width]
    for raw in log_lines:
        try:
            e = json.loads(raw)
        except ValueError:
            lines.append(raw)
            continue
        when = time.strftime('%H:%M:%S', time.localtime(e.get('t', 0)))
        who = e.get('operator') or ''
        if 'event' in e:
            text = e['event']
        else:
            text = ' '.join(x for x in (e.get('op', ''), e.get('cmd', ''),
                                        e.get('mission', '')) if x)
            text += '  ok' if e.get('ok') else f"  REFUSED: {e.get('message', '')}"
        lines.append(f"{when} {who:>8}  {text}")
    lines.append('-' * width)
    lines.append(prompt or 's stop   u surface   d disarm   : verb   q quit')
    return [l[:width] for l in lines]


def main(args=None):
    import curses
    import os
    import threading

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from std_msgs.msg import String

    from mongla_batighor.helm import LATCHED, LOG_PATH, build_goal
    from mongla_interfaces.action import Move
    from mongla_interfaces.msg import MonglaState

    who = f"term:{os.environ.get('USER', '?')}"

    class Deck(Node):
        def __init__(self):
            super().__init__('batighor_deck')
            self.vehicle, self.helm, self.log = None, {}, []
            self.move = ActionClient(self, Move, '/mongla/move')
            self.pub_log = self.create_publisher(String, '/mongla/batighor/log', 50)
            self.create_subscription(MonglaState, '/mongla/state',
                                     lambda m: setattr(self, 'vehicle', m), 10)
            self.create_subscription(String, '/mongla/batighor/helm_state',
                                     lambda m: setattr(self, 'helm', json.loads(m.data)),
                                     LATCHED)
            self.create_subscription(String, '/mongla/batighor/log',
                                     lambda m: self.log.append(m.data), 50)

        def record(self, **entry):
            line = json.dumps({'t': round(time.time(), 3), 'operator': who, **entry},
                              sort_keys=True)
            self.pub_log.publish(String(data=line))
            LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
            with open(LOG_PATH, 'a') as f:
                f.write(line + '\n')

        def send(self, cmd, fields_json='{}'):
            goal, why = build_goal(cmd, fields_json)
            if goal is None:
                self.record(op='command', cmd=cmd or '', ok=False, message=why)
                return
            if not self.move.server_is_ready():
                self.record(op='command', cmd=cmd, ok=False,
                            message='the manager is not answering /mongla/move')
                return
            self.move.send_goal_async(goal)
            self.record(op='command', cmd=cmd, fields=fields_json, ok=True,
                        message='sent from the terminal')

    from mongla_localization.params_file import refuse_unknown
    from mongla_localization.spin import spin
    rclpy.init(args=args)
    node = Deck()
    refuse_unknown(node)       # bringup.yaml `nodes:` typos stop here
    threading.Thread(target=spin, args=(node,), daemon=True).start()

    def loop(scr):
        curses.curs_set(0)
        scr.timeout(200)
        typing = None
        while True:
            h, w = scr.getmaxyx()
            room = max(h - 5, 1)
            prompt = f':{typing}' if typing is not None else ''
            scr.erase()
            for y, line in enumerate(render(w - 1, node.vehicle, node.helm,
                                            node.log[-room:], prompt)[:h]):
                scr.addstr(y, 0, line)
            scr.refresh()
            k = scr.getch()
            if k == -1:
                continue
            ch = chr(k) if 0 <= k < 256 else ''
            if typing is not None:
                if k in (10, 13):
                    try:
                        cmd, fields = parse_line(typing)
                    except ValueError as exc:
                        node.record(op='command', cmd=typing.split()[0] if typing.split() else '',
                                    ok=False, message=str(exc))
                        cmd = None
                    if cmd:
                        node.send(cmd, fields)
                    typing = None
                elif k == 27:
                    typing = None
                elif k in (8, 127, curses.KEY_BACKSPACE):
                    typing = typing[:-1]
                elif ch.isprintable():
                    typing += ch
            elif ch in KEYS:
                node.send(KEYS[ch])
            elif ch == ':':
                typing = ''
            elif ch == 'q':
                return

    try:
        curses.wrapper(loop)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

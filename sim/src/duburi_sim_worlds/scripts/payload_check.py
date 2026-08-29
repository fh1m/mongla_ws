#!/usr/bin/env python3

"""End-to-end check of the simulated payload: driver -> PTY -> Gazebo body.

Proves the thing `payload_sim` exists for -- that `duburi.fire()`, and with it
`align(fire=..., fire_t=...)`, runs in simulation through the SAME
`PayloadDriver` the vehicle flies, and that the shot is a physical body rather
than a log line.

    ros2 launch duburi_sim_bringup sim.launch.py course:=robosub26_full \
        ardusub:=false bridge:=false gui:=false
    python3 payload_check.py

Three assertions, each of which failed at some point during development:

1. The real driver connects to the PTY and its bytes arrive.
2. Every fire reaches the ROS topic.
3. A torpedo REACHES THE BOARD -- sampled at the moment of impact, not at rest.

That third one is the subtle one. The round rebounds off the board and then
sinks down its face for several seconds, so a check that reads the final
position sees a body on the floor short of the target and calls a clean strike
a miss. Take the maximum range the round achieved, not where it stopped.
"""

import os
import re
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', '..', '..', '..', 'src', 'duburi_control'))
from duburi_control.payload import PayloadDriver  # noqa: E402

WORLD = 'robosub26_full'
PORT = f'/tmp/duburi-{os.environ.get("USER", "user")}/payload'
# robosub26_full puts the board at x = 8, y = 3; its face is 0.6 m tall and
# centred 0.85 m above its base at z = -2.1.
BOARD_X, BOARD_Y, BOARD_Z = 8.0, 3.0, -1.25
STANDOFF = 1.6
NOSE = 0.09          # half the round's length
FIRED_TOPIC = '/duburi/sim/payload/fired'


def _pose(name):
    out = subprocess.run(['gz', 'model', '-m', name, '-p'],
                         capture_output=True, text=True, timeout=15).stdout
    m = re.search(r'\[([-\d.e]+)\s+([-\d.e]+)\s+([-\d.e]+)\]', out)
    return tuple(round(float(m.group(i)), 3) for i in (1, 2, 3)) if m else None


def _teleport(x, y, z):
    subprocess.run(
        ['gz', 'service', '-s', f'/world/{WORLD}/set_pose',
         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '3000', '--req',
         f'name: "duburi", position: {{x: {x}, y: {y}, z: {z}}}, '
         f'orientation: {{x: 0, y: 0, z: 0, w: 1}}'],
        check=False, capture_output=True)


def main() -> int:
    slot = int(sys.argv[1]) if len(sys.argv) > 1 else 0

    rclpy.init()
    node = Node('payload_check')
    fired = []
    node.create_subscription(Int32, FIRED_TOPIC,
                            lambda m: fired.append(m.data), 10)
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

    driver = PayloadDriver()
    if not driver.connect(port=PORT):
        print(f'FAIL: the real driver could not open {PORT}')
        return 1
    print(f'driver connected to the virtual board on {PORT}')

    _teleport(BOARD_X - STANDOFF, BOARD_Y, BOARD_Z)
    time.sleep(3.0)

    if not driver.fire(1):
        print('FAIL: fire(1) returned False')
        return 1

    name = f'payload_shot_{slot}'
    time.sleep(0.25)
    best = None
    for _ in range(16):
        p = _pose(name)
        if p and (best is None or p[0] > best[0]):
            best = p
        time.sleep(0.15)

    if best is None:
        print(f'FAIL: no projectile named {name} was spawned')
        return 1

    reach = best[0] + NOSE
    print(f'furthest reach x={reach:.3f} at depth {best[2]:.3f} '
          f'(board face x={BOARD_X}, spans z=-0.95..-1.55)')

    if not fired:
        print(f'FAIL: nothing published on {FIRED_TOPIC}')
        return 1
    print(f'{FIRED_TOPIC} saw channels {fired}')

    if best[2] < -1.95:
        print('FAIL: the round was on the floor before it got there')
        return 1
    if reach < BOARD_X - 0.15:
        print(f'FAIL: fell {BOARD_X - reach:.3f} m short of the board')
        return 1

    print('PASS: real driver -> PTY -> ROS topic -> a body that reaches '
          'the board at depth')
    return 0


if __name__ == '__main__':
    sys.exit(main())

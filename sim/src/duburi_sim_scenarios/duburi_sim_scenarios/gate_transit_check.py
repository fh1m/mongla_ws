#!/usr/bin/env python3

"""Score whether the vehicle actually flew through a gate.

This is the V0.1 acceptance test. It watches the simulator's ground truth,
works out where the gate is from the course YAML, and reports whether the
vehicle's track crossed the gate plane inside the opening and how far off
centre it was.

It only observes. Driving is left to whatever is being tested, deliberately:
the simulator must not depend on the autonomy stack it exists to grade, so
nothing here imports or links against duburi_ws. Run it in one terminal and the
autonomy stack in another.

    # terminal 1
    ros2 launch duburi_sim_bringup sim.launch.py course:=sauvc26_qualification

    # terminal 2, the real stack, unmodified
    ros2 launch duburi_manager bringup.launch.py mode:=sim \\
        yaw_source:=mavlink_ahrs flight_controller:=pixhawk \\
        dvl_auto_connect:=false

    # terminal 3
    ros2 run duburi_sim_scenarios gate_transit_check

    # terminal 4, once the check says it is watching
    ros2 run duburi_planner duburi arm
    ros2 run duburi_planner duburi set_depth --target -1.0
    ros2 run duburi_planner duburi lock_heading
    ros2 run duburi_planner duburi move_forward --duration 22 --gain 60

Depth is negative below the surface, so `--target -1.0` descends. Passing
`+1.0` asks the vehicle to fly, which it will decline to do for thirty seconds
and then report as a timeout.
"""

import argparse
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node

GROUND_TRUTH = '/duburi/sim/ground_truth'

# Course props name their model, and the arena spec keys its dimensions
# separately. This maps one to the other.
GATE_MODELS = {
    'sauvc_qual_gate': 'qualification_gate',
    'sauvc_final_gate': 'final_gate',
}


def load_gate(course: str, name: str = None):
    """Return (label, x, y, width) for the gate to be scored."""
    share = get_package_share_directory('duburi_sim_worlds')
    with open(os.path.join(share, 'courses', f'{course}.yaml')) as f:
        layout = yaml.safe_load(f)
    with open(os.path.join(share, 'spec', 'arena.yaml')) as f:
        arena = yaml.safe_load(f)

    gates = [p for p in layout.get('props', [])
             if p.get('model') in GATE_MODELS
             and (name is None or p.get('name') == name)]
    if not gates:
        raise SystemExit(
            f'course {course!r} has no gate' +
            (f' named {name!r}' if name else '') +
            f'. Gate models are {sorted(GATE_MODELS)}.')
    if len(gates) > 1:
        raise SystemExit(
            f'course {course!r} has several gates '
            f'({", ".join(g["name"] for g in gates)}); pass --gate to choose.')

    gate = gates[0]
    width = arena['props'][GATE_MODELS[gate['model']]]['width']
    x, y = gate['xy']
    return gate['name'], float(x), float(y), float(width)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--course', default='sauvc26_qualification',
                        help='Course YAML the simulator was launched with.')
    parser.add_argument('--gate', default=None,
                        help='Prop name, when the course has more than one gate.')
    parser.add_argument('--timeout', type=float, default=120.0,
                        help='Seconds to wait for the vehicle to reach the gate.')
    parser.add_argument('--margin', type=float, default=0.0,
                        help='Metres to shrink the opening by on each side, to '
                             'require clearance rather than a bare pass.')
    args = parser.parse_args()

    label, gate_x, gate_y, width = load_gate(args.course, args.gate)
    half = width / 2.0 - args.margin
    if half <= 0:
        raise SystemExit(f'--margin {args.margin} m leaves no opening in a '
                         f'{width} m gate')

    rclpy.init()
    node = Node('duburi_sim_gate_transit_check')
    track = []
    node.create_subscription(
        Odometry, GROUND_TRUTH,
        lambda m: track.append((m.pose.pose.position.x,
                                m.pose.pose.position.y,
                                m.pose.pose.position.z)), 10)

    node.get_logger().info(
        f'watching {GROUND_TRUTH} for a pass through {label} at '
        f'x={gate_x:+.2f}, opening {2 * half:.2f} m wide')

    crossing = None
    deadline = node.get_clock().now().nanoseconds * 1e-9 + args.timeout
    checked = 0
    while crossing is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds * 1e-9 > deadline:
            break
        # Only look at segments that have arrived since the last sweep.
        for i in range(max(checked, 1), len(track)):
            (x0, y0, z0), (x1, y1, z1) = track[i - 1], track[i]
            # Crossing in either direction; the return leg counts too.
            if (x0 - gate_x) * (x1 - gate_x) <= 0 and x0 != x1:
                f = (gate_x - x0) / (x1 - x0)
                crossing = (y0 + f * (y1 - y0), z0 + f * (z1 - z0))
                break
        checked = len(track)

    print()
    if not track:
        print(f'FAIL: nothing published on {GROUND_TRUTH}.\n'
              '  Is the simulator running, and was it launched with bridge:=true?')
        return 1

    travelled = track[-1][0] - track[0][0]
    print(f'travelled x {track[0][0]:+.2f} -> {track[-1][0]:+.2f} '
          f'({travelled:+.2f} m) over {len(track)} samples')

    if crossing is None:
        print(f'FAIL: never reached the gate plane at x={gate_x:+.2f}, '
              f'stopped {abs(gate_x - track[-1][0]):.2f} m short.')
        return 1

    y, z = crossing
    offset = abs(y - gate_y)
    print(f'crossed the gate plane at y={y:+.2f} m, z={z:+.2f} m')
    if offset <= half:
        print(f'\nPASS: through the {2 * half:.2f} m opening, '
              f'{offset:.2f} m off centre\n')
        return 0
    print(f'\nFAIL: missed the opening by {offset - half:.2f} m\n')
    return 1


if __name__ == '__main__':
    sys.exit(main())

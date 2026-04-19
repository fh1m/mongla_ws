#!/usr/bin/env python3
"""vision_thrust_check -- end-to-end smoke test: detection -> RC channel.

Sends a short `vision_align_yaw` goal via /duburi/move and reports what
came back. Use BEFORE pool tests so a wiring mistake doesn't waste pool
time.

Watch the manager log for `[VIS  ]` lines (loop is running) and
`[RC   ] Yaw:NNN` lines (thrust has actually moved). If both appear
the detection -> thrust chain is intact.

Examples
--------
ros2 run duburi_vision vision_thrust_check
ros2 run duburi_vision vision_thrust_check --camera laptop --duration 5

Requires both `duburi_manager` (action server) AND a vision pipeline
(camera_node + detector_node) to already be running. It does NOT arm
the vehicle; if you want thrust to physically spin, the operator
arms the vehicle separately first.

We deliberately use rclpy.ActionClient directly here (no DuburiClient
import) so duburi_vision stays free of a duburi_planner dependency
and the build order remains lean.
"""

from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node   import Node

from duburi_interfaces.action import Move


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Vision -> thrust smoke test (sends one vision_align_yaw goal).')
    parser.add_argument('--camera',          default='laptop')
    parser.add_argument('--target-class',    default='person')
    parser.add_argument('--duration',        type=float, default=5.0,
                        help='vision_align_yaw duration seconds (default: 5)')
    parser.add_argument('--deadband',        type=float, default=0.10)
    parser.add_argument('--kp-yaw',          type=float, default=60.0)
    parser.add_argument('--on-lost',         choices=('fail', 'hold'),
                        default='hold',
                        help="hold by default so a brief mis-detect doesn't abort")
    parser.add_argument('--stale-after',     type=float, default=0.8)
    parser.add_argument('--connect-timeout', type=float, default=15.0)
    args = parser.parse_args(argv)

    rclpy.init()
    node = Node('vision_thrust_check')
    client = ActionClient(node, Move, '/duburi/move')

    print(f'[VTHR] connecting to /duburi/move (timeout {args.connect_timeout:.0f}s)...')
    if not client.wait_for_server(timeout_sec=args.connect_timeout):
        print('[VTHR] FAIL: action server /duburi/move not available')
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    goal = Move.Goal()
    goal.cmd          = 'vision_align_yaw'
    goal.camera       = args.camera
    goal.target_class = args.target_class
    goal.duration     = float(args.duration)
    goal.deadband     = float(args.deadband)
    goal.kp_yaw       = float(args.kp_yaw)
    goal.on_lost      = args.on_lost
    goal.stale_after  = float(args.stale_after)

    print(f'[VTHR] sending vision_align_yaw  camera={args.camera}  '
          f'class={args.target_class}  duration={args.duration:.1f}s  '
          f'on_lost={args.on_lost}')
    print('[VTHR] watch the manager log for [VIS  ] lines and [RC   ] Yaw:NNN')

    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    handle = send_future.result()
    if not handle.accepted:
        print('[VTHR] FAIL: action server REJECTED the goal '
              '(another command running?)')
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result().result

    print()
    print(f'[VTHR] {"PASS" if result.success else "FAIL"}')
    print(f'  message            : {result.message}')
    print(f'  composite_error    : {result.final_value:.3f}')
    print(f'  last detection age : {result.error_value:.2f}s')
    print()
    print('[VTHR] If you saw [RC   ] Yaw:... in the manager log, the loop '
          'is closed end-to-end. If not, run `vision_check` first to confirm '
          'the detector is publishing.')

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if result.success else 1)


if __name__ == '__main__':
    main()

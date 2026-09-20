#!/usr/bin/env python3
"""vision_thrust_check -- end-to-end smoke test: detection -> RC channel.

Sends a short `vision_align` (yaw axis) goal via /mongla/move and reports
what came back. Use BEFORE pool tests so a wiring mistake doesn't waste
pool time.

Watch the manager log for `[VIS  ]` lines (loop is running) and
`[RC   ] Yaw:NNN` lines (thrust has actually moved). If both appear
the detection -> thrust chain is intact.

Examples
--------
ros2 run mongla_vision vision_thrust_check
ros2 run mongla_vision vision_thrust_check --camera laptop --duration 5

Requires both `mongla_manager` (action server) AND a vision pipeline
(camera_node + detector_node) to already be running. It does NOT arm
the vehicle; if you want thrust to physically spin, the operator
arms the vehicle separately first.

We deliberately use rclpy.ActionClient directly here (no MonglaClient
import) so mongla_vision stays free of a mongla_planner dependency
and the build order remains lean.
"""

from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node   import Node

from mongla_interfaces.action import Move


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Vision -> thrust smoke test (sends one vision_align yaw goal).')
    parser.add_argument('--camera',          default='laptop')
    parser.add_argument('--target-class',    default='person')
    parser.add_argument('--duration',        type=float, default=5.0,
                        help='vision_align duration seconds (default: 5)')
    parser.add_argument('--err-px',          type=float, default=40.0,
                        help='pixel tolerance for "aligned" (default: 40)')
    parser.add_argument('--gain',            type=float, default=30.0,
                        help='hard max-speed cap %% (default: 30)')
    parser.add_argument('--kp-yaw',          type=float, default=0.0,
                        help='override yaw P gain (0 = use vision.kp_yaw param)')
    parser.add_argument('--connect-timeout', type=float, default=15.0)
    args = parser.parse_args(argv)

    rclpy.init()
    node = Node('vision_thrust_check')
    client = ActionClient(node, Move, '/mongla/move')

    print(f'[VTHR ] connecting to /mongla/move (timeout {args.connect_timeout:.0f}s)...')
    if not client.wait_for_server(timeout_sec=args.connect_timeout):
        print('[VTHR ] FAIL: action server /mongla/move not available')
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    goal = Move.Goal()
    goal.cmd          = 'vision_align'
    goal.camera       = args.camera
    goal.target_class = args.target_class
    goal.axes         = 'yaw'
    goal.offset_yaw   = 0.0
    goal.err_px       = float(args.err_px)
    goal.duration     = float(args.duration)
    goal.gain         = float(args.gain)
    goal.kp_yaw       = float(args.kp_yaw)
    # No fallback from a raw goal -> hold through brief losses for the run.
    goal.hold_through_loss = True

    print(f'[VTHR ] sending vision_align (yaw)  camera={args.camera}  '
          f'class={args.target_class}  duration={args.duration:.1f}s  '
          f'err={args.err_px:.0f}px gain={args.gain:.0f}%')
    print('[VTHR ] watch the manager log for [VIS  ] lines and [RC   ] Yaw:NNN')

    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    handle = send_future.result()
    if not handle.accepted:
        print('[VTHR ] FAIL: action server REJECTED the goal '
              '(another command running?)')
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result().result

    _OUTCOME = {0: 'ALIGNED', 1: 'LOST', 2: 'TIMEOUT', 3: 'NO_CAMERA', 4: 'ABORTED'}
    code = int(round(result.final_value))
    aligned = code == 0
    print()
    print(f'[VTHR ] {"PASS (aligned)" if aligned else "ran (not aligned)"}')
    print(f'  message            : {result.message}')
    print(f'  outcome            : {_OUTCOME.get(code, code)}')
    print(f'  last pixel error   : {result.error_value:.1f}px')
    print()
    print('[VTHR ] If you saw [RC   ] Yaw:... in the manager log, the loop '
          'is closed end-to-end. If not, run `vision_check` first to confirm '
          'the detector is publishing.')

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if aligned else 1)


if __name__ == '__main__':
    main()

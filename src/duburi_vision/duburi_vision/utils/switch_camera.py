#!/usr/bin/env python3
"""switch_camera -- resume ONE detector, pause the others (CLI exclusivity).

The mission DSL's ``use_camera`` already keeps exactly one detector inferring
(pauses every other known detector, resumes the target). This gives the SAME
exclusivity from the command line -- so a manual switch can't leave two
detectors inferring at once, which is the concurrent-inference OOM on the 8 GB
Jetson. Prefer this over a bare ``ros2 param set <one detector> paused false``,
which resumes one WITHOUT pausing the other.

Examples
--------
ros2 run duburi_vision switch_camera downward     # downward infers, forward pauses
ros2 run duburi_vision switch_camera forward      # forward infers, downward pauses
ros2 run duburi_vision switch_camera --cameras forward,downward,side downward

Only detectors that are actually up are touched (fast graph check), so a
single-camera setup is a no-op on the absent ones. Requires the detector
node(s) to be running.
"""

from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


def _set_paused(node: Node, detector: str, paused: bool, timeout: float) -> bool:
    cli = node.create_client(SetParameters, f'{detector}/set_parameters')
    if not cli.wait_for_service(timeout_sec=timeout):
        return False
    req = SetParameters.Request(parameters=[Parameter(
        name='paused',
        value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=paused))])
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    resp = fut.result()
    return bool(resp and resp.results and resp.results[0].successful)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Resume one detector, pause the others (CLI camera exclusivity).')
    parser.add_argument('camera', help="camera to make live, e.g. 'forward' | 'downward'")
    parser.add_argument('--cameras', default='forward,downward',
                        help='known cameras to enforce exclusivity over (CSV)')
    parser.add_argument('--timeout', type=float, default=3.0,
                        help='per-detector service timeout seconds (default: 3)')
    args = parser.parse_args(argv)

    known = [c.strip() for c in args.cameras.split(',') if c.strip()]
    if args.camera not in known:
        known.append(args.camera)

    rclpy.init()
    node = Node('duburi_switch_camera')
    try:
        present = {n.lstrip('/') for n in node.get_node_names()}
        # Pause the others FIRST (drop inference before adding the new one), then
        # resume the target -- so the two-inferring window never opens.
        rc = 0
        for cam in known:
            if cam == args.camera:
                continue
            det = f'duburi_detector_{cam}'
            if det not in present:
                continue                       # not up -> nothing to pause
            ok = _set_paused(node, f'/{det}', True, args.timeout)
            print(f'  pause  {cam:<9s} {"[OK]" if ok else "[--]"}')
            rc = rc or (0 if ok else 1)

        target = f'duburi_detector_{args.camera}'
        if target not in present:
            print(f'  ERROR: {target} is not running', file=sys.stderr)
            return 2
        ok = _set_paused(node, f'/{target}', False, args.timeout)
        print(f'  resume {args.camera:<9s} {"[OK]" if ok else "[--]"}')
        return rc or (0 if ok else 1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

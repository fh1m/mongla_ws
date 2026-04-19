#!/usr/bin/env python3
"""Test Runner -- Terminal 2.

Executes the Duburi square-pattern test mission via the /duburi/move
ActionServer. Exits non-zero on any failure so wrappers / CI can
detect a bad mission.

Mission (square pattern at surface):
  arm -> MANUAL -> set_depth -0.0
    -> forward 5s -> yaw_right 90
    -> forward 5s -> yaw_right 90
    -> forward 5s -> yaw_right 90
    -> forward 5s -> disarm

Each `duburi.send(...)` call returns a `Move.Result`; we log the
`final_value` and `error_value` so the mission log shows axis-correct
exit conditions per command (yaw heading + heading error, depth +
depth error, etc).
"""

import sys

import rclpy
from rclpy.node import Node

from .client import DuburiClient


def _step(duburi, log, label, cmd, **fields):
    """Send one command and log its result. Failures propagate."""
    result = duburi.send(cmd, **fields)
    log.info(
        f'  -> {label}: final={result.final_value:+.3f}  '
        f'err={result.error_value:+.3f}  ({result.message})')


def _run_mission(duburi, log):
    duburi.wait_for_connection(timeout=15.0)
    log.info('=' * 52)
    log.info(' DUBURI TEST MISSION -- starting')
    log.info('=' * 52)

    _step(duburi, log, 'arm',         'arm')
    _step(duburi, log, 'MANUAL',      'set_mode', target_name='MANUAL')
    _step(duburi, log, 'depth 0.0',   'set_depth', target=-0.0, timeout=40.0)

    for leg in range(1, 5):
        _step(duburi, log, f'leg {leg} fwd', 'move_forward', duration=5.0, gain=60.0)
        if leg < 4:
            _step(duburi, log, f'leg {leg} turn', 'yaw_right', target=90.0)

    _step(duburi, log, 'disarm', 'disarm')

    log.info('=' * 52)
    log.info(' DUBURI TEST MISSION -- complete OK')
    log.info('=' * 52)


def main(args=None):
    rclpy.init(args=args)
    node   = Node('duburi_test_runner')
    log    = node.get_logger()
    duburi = DuburiClient(node)

    exit_code = 0
    try:
        _run_mission(duburi, log)
    except Exception as exc:
        log.error(f'MISSION FAILED: {exc}')
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()

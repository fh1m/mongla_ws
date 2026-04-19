#!/usr/bin/env python3
"""mission -- argparse runner that dispatches into missions/<name>.run.

Usage:
    ros2 run duburi_planner mission <name>            # run a mission
    ros2 run duburi_planner mission --list            # list known missions

Every mission module under `duburi_planner.missions` exposes a
`run(duburi, log)` function where `duburi` is the `DuburiMission` DSL
wrapper. Drop a new `.py` file in that folder, expose `run`, and the
runner picks it up automatically -- no registry edit required.

The wrapper falls through to the raw `DuburiClient` for any attribute
it doesn't define (`__getattr__`) and still logs the one-line outcome,
so older missions that called `client.send('move_forward', ...)` keep
working unchanged.

This runner owns the rclpy lifecycle: init, build the client + DSL,
hand them to the mission's `run()`, then shutdown -- regardless of
whether the mission succeeded, raised, or was Ctrl+C'd. Exit code is
non-zero iff the mission raised, so wrappers / CI can detect a bad run.
"""

import argparse
import sys

import rclpy
from rclpy.node import Node

from .client      import DuburiClient
from .duburi_dsl  import DuburiMission
from .missions    import discover


def _build_parser(known):
    parser = argparse.ArgumentParser(prog='mission')
    parser.add_argument(
        'name', nargs='?',
        help=f'mission to run. Known: {sorted(known)}')
    parser.add_argument(
        '--list', action='store_true', help='list known missions and exit')
    return parser


def main(args=None):
    missions = discover()
    parsed   = _build_parser(missions).parse_args(args)

    if parsed.list or not parsed.name:
        for name in sorted(missions):
            print(name)
        sys.exit(0 if parsed.list else 2)

    if parsed.name not in missions:
        print(f"Unknown mission '{parsed.name}'. Known: {sorted(missions)}",
              file=sys.stderr)
        sys.exit(2)

    rclpy.init()
    node   = Node('duburi_mission_runner')
    log    = node.get_logger()
    client = DuburiClient(node)
    duburi = DuburiMission(client, log)

    exit_code = 0
    try:
        client.wait_for_connection(timeout=15.0)
        log.info(f'=== mission "{parsed.name}" -- start ===')
        missions[parsed.name](duburi, log)
        log.info(f'=== mission "{parsed.name}" -- complete OK ===')
    except KeyboardInterrupt:
        log.warn(f'mission "{parsed.name}" interrupted by user')
        exit_code = 130
    except Exception as exc:
        log.error(f'mission "{parsed.name}" FAILED: {exc}')
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()

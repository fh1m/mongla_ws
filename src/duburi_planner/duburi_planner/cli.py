#!/usr/bin/env python3
"""duburi CLI -- argparse wrapper auto-built from `COMMANDS`.

Every subcommand and its flags come straight from the COMMANDS
registry: adding a row in `duburi_control/commands.py` makes a new
CLI verb appear the next time `colcon build`s.

Usage:
    ros2 run duburi_planner duburi <cmd> [--field value ...]

Examples:
    ros2 run duburi_planner duburi arm
    ros2 run duburi_planner duburi set_mode --target_name ALT_HOLD
    ros2 run duburi_planner duburi set_depth --target -1.5
    ros2 run duburi_planner duburi move_forward --duration 5 --gain 60
    ros2 run duburi_planner duburi yaw_left --target 90
    ros2 run duburi_planner duburi arc --duration 4 --gain 50 --yaw_rate_pct 30
    ros2 run duburi_planner duburi lock_heading --target 0 --timeout 120
    ros2 run duburi_planner duburi unlock_heading
    ros2 run duburi_planner duburi pause --duration 3
    ros2 run duburi_planner duburi stop
    ros2 run duburi_planner duburi disarm
"""

import argparse
import sys

import rclpy
from rclpy.node import Node

from duburi_control import COMMANDS
from duburi_control.commands import STRING_FIELDS

from .client import DuburiClient


def _build_parser():
    parser = argparse.ArgumentParser(prog='duburi')
    sub    = parser.add_subparsers(dest='cmd', required=True)
    for name, spec in COMMANDS.items():
        cmd_parser = sub.add_parser(name, help=spec['help'])
        for field in spec['fields']:
            is_string = field in STRING_FIELDS
            has_default = field in spec['defaults']
            cmd_parser.add_argument(
                f'--{field}',
                type=str if is_string else float,
                required=not has_default,
                default=spec['defaults'].get(field),
                help=f'(default: {spec["defaults"][field]})' if has_default
                     else '(required)',
            )
    return parser


def _fields_from_args(cmd, args):
    """Pick only the fields that belong to `cmd` out of the parsed args."""
    return {field: getattr(args, field) for field in COMMANDS[cmd]['fields']}


def main():
    args = _build_parser().parse_args()

    rclpy.init()
    node   = Node('duburi_cli')
    duburi = DuburiClient(node)

    exit_code = 0
    try:
        duburi.wait_for_connection()
        result = duburi.send(args.cmd, **_fields_from_args(args.cmd, args))
        node.get_logger().info(
            f'{args.cmd} -> OK  '
            f'final={result.final_value:.3f}  err={result.error_value:.3f}  '
            f'msg="{result.message}"')
    except Exception as exc:
        node.get_logger().error(f'{args.cmd} -> FAIL: {exc}')
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()

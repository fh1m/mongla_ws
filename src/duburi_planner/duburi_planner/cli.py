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
from duburi_control.commands import BOOL_FIELDS, STRING_FIELDS

from .client import DuburiClient


def _bool_arg(value):
    """Loose bool parser so `--visual_pid true` / `1` / `yes` all work."""
    if isinstance(value, bool):
        return value
    truthy = {'1', 'true',  't', 'yes', 'y', 'on'}
    falsy  = {'0', 'false', 'f', 'no',  'n', 'off'}
    lower  = str(value).strip().lower()
    if lower in truthy:
        return True
    if lower in falsy:
        return False
    raise argparse.ArgumentTypeError(
        f'expected bool-ish value, got {value!r}')


_HEAD_SENTINEL = 'head'

def _float_or_head(value):
    """Accept a float OR the keyword 'head'.

    'head' is resolved at send time to the live heading reading from
    the active yaw source -- use it anywhere a float target is accepted:

        duburi lock_heading --target head
        duburi yaw_left --target head
    """
    if str(value).strip().lower() == _HEAD_SENTINEL:
        return _HEAD_SENTINEL
    return float(value)


# Vision verbs let the manager fill defaults from `vision.*` ROS params
# at dispatch time, so the CLI must NOT pre-fill spec defaults for these
# commands; sending the rosidl zero is what triggers the live-param
# substitution inside `commands.fields_for`.
_LIVE_TUNED_COMMANDS = {
    'vision_align_3d', 'vision_align_yaw', 'vision_align_lat',
    'vision_align_depth', 'vision_hold_distance', 'vision_acquire',
}


def _build_parser():
    parser = argparse.ArgumentParser(prog='duburi')
    sub    = parser.add_subparsers(dest='cmd', required=True)
    for name, spec in COMMANDS.items():
        cmd_parser = sub.add_parser(name, help=spec['help'])
        live_tuned = name in _LIVE_TUNED_COMMANDS
        for field in spec['fields']:
            has_default = field in spec['defaults']
            if field in BOOL_FIELDS:
                arg_type = _bool_arg
            elif field in STRING_FIELDS:
                arg_type = str
            else:
                arg_type = _float_or_head
            # For live-tuned commands, leave optional fields at None so
            # the rosidl zero reaches the manager and `vision.*` ROS
            # params apply. For everything else, prefill the spec default.
            cli_default = None if live_tuned else spec['defaults'].get(field)
            help_default = (
                f'(default: vision.* ROS param, see vision_tunables.yaml)'
                if live_tuned and has_default
                else (f'(default: {spec["defaults"][field]})'
                      if has_default else '(required)'))
            cmd_parser.add_argument(
                f'--{field}',
                type=arg_type,
                required=not has_default,
                default=cli_default,
                help=help_default,
            )
    return parser


def _fields_from_args(cmd, args):
    """Pick only the fields that belong to `cmd` out of the parsed args.

    Drops kwargs the user didn't supply for live-tuned commands so the
    rosidl zero reaches the manager (= live ROS-param defaults apply).
    """
    fields = {}
    for field in COMMANDS[cmd]['fields']:
        value = getattr(args, field)
        if value is None:
            continue
        fields[field] = value
    return fields


def main():
    args = _build_parser().parse_args()

    rclpy.init()
    node   = Node('duburi_cli')
    duburi = DuburiClient(node)

    exit_code = 0
    try:
        duburi.wait_for_connection()
        fields = _fields_from_args(args.cmd, args)
        if any(v == _HEAD_SENTINEL for v in fields.values()):
            live = duburi.send('head').final_value
            fields = {k: (live if v == _HEAD_SENTINEL else v)
                      for k, v in fields.items()}
        result = duburi.send(args.cmd, **fields)
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

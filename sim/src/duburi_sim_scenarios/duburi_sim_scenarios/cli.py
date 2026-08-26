#!/usr/bin/env python3

"""Command-line front end for the runtime prop services.

Calling ros_gz_interfaces services by hand means writing nested YAML with
quaternions in it, which nobody does twice. This wraps them so that rearranging
a course is a one-liner.

    ros2 run duburi_sim_scenarios props list
    ros2 run duburi_sim_scenarios props add sauvc_drum_red drum_x 8.0 -1.6
    ros2 run duburi_sim_scenarios props move drum_x 9.0 -2.0 --yaw 0.5
    ros2 run duburi_sim_scenarios props remove drum_x

Positions are given as x y in the pool plane. z is filled in from the prop's
anchor - floor-standing props land on the floor, surface props hang from the
water surface - which matches how courses/*.yaml are written. Pass --z to
override.
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity, EntityFactory
from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity

from duburi_sim_scenarios.prop_catalog import (
    load_prop_library,
    pool_depth,
    prop_anchor,
    prop_names,
)

NS = '/duburi/sim/props'
CALL_TIMEOUT = 10.0


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class PropClient(Node):

    def __init__(self):
        super().__init__('duburi_sim_props_cli')
        self.spawn = self.create_client(SpawnEntity, f'{NS}/spawn')
        self.move = self.create_client(SetEntityPose, f'{NS}/move')
        self.delete = self.create_client(DeleteEntity, f'{NS}/delete')

    def wait(self, client) -> bool:
        if client.wait_for_service(timeout_sec=CALL_TIMEOUT):
            return True
        print(
            f'error: {client.srv_name} is not available. Start the prop manager:\n'
            '  ros2 run duburi_sim_scenarios prop_manager '
            '--ros-args -p world:=<course>',
            file=sys.stderr,
        )
        return False

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=CALL_TIMEOUT)
        return future.result()


def resolve_z(library, model: str, args) -> float:
    if args.z is not None:
        return args.z
    anchor = prop_anchor(library, model)
    depth = pool_depth(library)
    if anchor == 'floor' and depth is not None:
        return -depth + args.z_offset
    return args.z_offset


def cmd_list(_node, library, _args) -> int:
    names = prop_names(library)
    if not names:
        print('no prop library available; is duburi_sim_worlds installed?')
        return 1
    print(f'{len(names)} registered props:')
    for name in names:
        print(f'  {name:<28} anchored to {prop_anchor(library, name)}')
    return 0


def cmd_add(node, library, args) -> int:
    if not node.wait(node.spawn):
        return 1

    request = SpawnEntity.Request()
    factory = EntityFactory()
    factory.name = args.name
    factory.sdf_filename = args.model  # resolved by the manager
    factory.allow_renaming = False
    factory.pose.position.x = float(args.x)
    factory.pose.position.y = float(args.y)
    factory.pose.position.z = resolve_z(library, args.model, args)
    qx, qy, qz, qw = yaw_to_quat(args.yaw)
    factory.pose.orientation.x = qx
    factory.pose.orientation.y = qy
    factory.pose.orientation.z = qz
    factory.pose.orientation.w = qw
    request.entity_factory = factory

    result = node.call(node.spawn, request)
    if result is None:
        print('error: spawn call timed out', file=sys.stderr)
        return 1
    print(f'spawned {args.name}' if result.success else 'spawn rejected')
    return 0 if result.success else 1


def cmd_move(node, library, args) -> int:
    if not node.wait(node.move):
        return 1

    request = SetEntityPose.Request()
    request.entity = Entity(name=args.name, type=Entity.MODEL)
    request.pose.position.x = float(args.x)
    request.pose.position.y = float(args.y)
    # Without the model name we cannot know the anchor, so an explicit --z or
    # --z-offset is the only way to place it off the floor.
    request.pose.position.z = args.z if args.z is not None else (
        -(pool_depth(library) or 0.0) + args.z_offset
    )
    qx, qy, qz, qw = yaw_to_quat(args.yaw)
    request.pose.orientation.x = qx
    request.pose.orientation.y = qy
    request.pose.orientation.z = qz
    request.pose.orientation.w = qw

    result = node.call(node.move, request)
    if result is None:
        print('error: move call timed out', file=sys.stderr)
        return 1
    print(f'moved {args.name}' if result.success else 'move rejected')
    return 0 if result.success else 1


def cmd_remove(node, _library, args) -> int:
    if not node.wait(node.delete):
        return 1

    request = DeleteEntity.Request()
    request.entity = Entity(name=args.name, type=Entity.MODEL)

    result = node.call(node.delete, request)
    if result is None:
        print('error: delete call timed out', file=sys.stderr)
        return 1
    print(f'removed {args.name}' if result.success else 'remove rejected')
    return 0 if result.success else 1


def build_parser():
    parser = argparse.ArgumentParser(
        prog='props',
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest='command', required=True)

    sub.add_parser('list', help='List registered props.')

    def add_pose_args(p, with_model):
        if with_model:
            p.add_argument('model', help='Registered prop name, e.g. sauvc_drum_red.')
        p.add_argument('name', help='Instance name in the world.')
        p.add_argument('x', type=float)
        p.add_argument('y', type=float)
        p.add_argument('--yaw', type=float, default=0.0, help='Heading, radians.')
        p.add_argument('--z', type=float, default=None,
                       help='Explicit z, overriding the prop anchor.')
        p.add_argument('--z-offset', dest='z_offset', type=float, default=0.0,
                       help='Nudge relative to the anchor.')

    add_pose_args(sub.add_parser('add', help='Spawn a prop.'), with_model=True)
    add_pose_args(sub.add_parser('move', help='Move an existing prop.'),
                  with_model=False)

    remove = sub.add_parser('remove', help='Delete a prop.')
    remove.add_argument('name')

    return parser


COMMANDS = {
    'list': cmd_list,
    'add': cmd_add,
    'move': cmd_move,
    'remove': cmd_remove,
}


def main(argv=None):
    args = build_parser().parse_args(argv if argv is not None else sys.argv[1:])
    library = load_prop_library()

    rclpy.init(args=None)
    node = PropClient()
    try:
        code = COMMANDS[args.command](node, library, args)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())

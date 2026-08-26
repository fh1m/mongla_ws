#!/usr/bin/env python3

"""Add, move and remove simulator props at runtime, without restarting Gazebo.

This is what makes the environment tunable against a real pool. Competition prop
placement is only specified to within a zone, and our own pool differs from the
competition one, so being able to shove a gate two metres left and re-run is the
difference between the simulator being a demo and being a test rig.

Exposes three ROS services under /duburi/sim/props:

    spawn   ros_gz_interfaces/SpawnEntity
    move    ros_gz_interfaces/SetEntityPose
    delete  ros_gz_interfaces/DeleteEntity

Spawn accepts either a registered prop name from duburi_sim_worlds (in
`entity_factory.sdf_filename`, given as a bare name such as "sauvc_drum_red") or
raw SDF. Registered names are resolved through prop_library, so a prop spawned
here is byte-identical to the same prop baked into a world.

The Gazebo side is reached with the gz-transport Python bindings rather than by
shelling out to `gz service`, which keeps errors structured and avoids paying
gz-transport's several-second discovery cost on every single call.

Usage:
    ros2 run duburi_sim_scenarios prop_manager --ros-args -p world:=sauvc26_final
"""

import os
import sys

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity

# Must precede the gz.transport import; gz-transport fixes its interface when the
# library initialises. sim.launch.py sets the same value for Gazebo itself. On a
# host with several interfaces, leaving this to gz-transport's own choice makes
# discovery slow and partial, and a partially discovered service call just times
# out with nothing in any log to explain it.
os.environ.setdefault('GZ_IP', '127.0.0.1')

from gz.msgs10.boolean_pb2 import Boolean  # noqa: E402
from gz.msgs10.entity_factory_pb2 import EntityFactory  # noqa: E402
from gz.msgs10.entity_pb2 import Entity  # noqa: E402
from gz.msgs10.pose_pb2 import Pose  # noqa: E402
from gz.transport13 import Node as GzNode  # noqa: E402

from duburi_sim_scenarios.prop_catalog import (
    load_prop_library,
    prop_names,
    render_prop,
)

# gz-transport service calls are local, so a short timeout is right; a slow one
# means the world is not running, not that it is busy.
GZ_TIMEOUT_MS = 5000


class PropManager(Node):

    def __init__(self):
        super().__init__('duburi_sim_prop_manager')

        self.declare_parameter('world', 'sauvc26_qualification')
        self.world = self.get_parameter('world').value

        self.gz = GzNode()
        self.library = load_prop_library()

        ns = '/duburi/sim/props'
        self.create_service(SpawnEntity, f'{ns}/spawn', self.on_spawn)
        self.create_service(SetEntityPose, f'{ns}/move', self.on_move)
        self.create_service(DeleteEntity, f'{ns}/delete', self.on_delete)

        if self.library is None:
            self.get_logger().warning(
                'duburi_sim_worlds prop library not importable; spawn will only '
                'accept raw SDF, not registered prop names'
            )
        else:
            self.get_logger().info(
                f'{len(prop_names(self.library))} registered props available'
            )
        self.get_logger().info(f'prop manager ready on world "{self.world}"')

    # -- gz plumbing --------------------------------------------------------

    def _call(self, service: str, request, request_type):
        """Make a gz-transport service call, returning (ok, detail)."""
        endpoint = f'/world/{self.world}/{service}'
        ok, reply = self.gz.request(
            endpoint, request, request_type, Boolean, GZ_TIMEOUT_MS
        )
        if not ok:
            return False, (
                f'no response from {endpoint}. Is a world named "{self.world}" '
                'running? Set the `world` parameter to match.'
            )
        if not reply.data:
            return False, f'{endpoint} rejected the request'
        return True, ''

    @staticmethod
    def _fill_pose(target, pose) -> None:
        target.position.x = pose.position.x
        target.position.y = pose.position.y
        target.position.z = pose.position.z
        target.orientation.x = pose.orientation.x
        target.orientation.y = pose.orientation.y
        target.orientation.z = pose.orientation.z
        # A default-constructed geometry_msgs/Quaternion is all zeros, which is
        # not a rotation. Treat it as identity rather than handing Gazebo a
        # degenerate quaternion.
        w = pose.orientation.w
        target.orientation.w = w if any(
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, w)
        ) else 1.0

    # -- services -----------------------------------------------------------

    def on_spawn(self, request, response):
        factory = request.entity_factory
        sdf = factory.sdf

        if not sdf and factory.sdf_filename:
            name = factory.sdf_filename
            if os.path.sep in name or name.endswith('.sdf'):
                # A real path; hand it to Gazebo untouched.
                sdf = ''
            else:
                sdf = render_prop(self.library, name)
                if sdf is None:
                    self.get_logger().error(
                        f'unknown prop "{name}". Known props: '
                        f'{", ".join(prop_names(self.library))}'
                    )
                    response.success = False
                    return response

        req = EntityFactory()
        req.name = factory.name
        req.allow_renaming = factory.allow_renaming
        if sdf:
            req.sdf = sdf
        elif factory.sdf_filename:
            req.sdf_filename = factory.sdf_filename
        elif factory.clone_name:
            req.clone_name = factory.clone_name
        else:
            self.get_logger().error(
                'spawn needs one of sdf, sdf_filename or clone_name'
            )
            response.success = False
            return response
        self._fill_pose(req.pose, factory.pose)

        ok, detail = self._call('create', req, EntityFactory)
        if not ok:
            self.get_logger().error(f'spawn failed: {detail}')
        else:
            self.get_logger().info(f'spawned "{factory.name}"')
        response.success = ok
        return response

    def on_move(self, request, response):
        req = Pose()
        req.name = request.entity.name
        if request.entity.id:
            req.id = request.entity.id
        self._fill_pose(req, request.pose)

        ok, detail = self._call('set_pose', req, Pose)
        if not ok:
            self.get_logger().error(f'move failed: {detail}')
        else:
            self.get_logger().info(f'moved "{request.entity.name}"')
        response.success = ok
        return response

    def on_delete(self, request, response):
        req = Entity()
        if request.entity.name:
            req.name = request.entity.name
        if request.entity.id:
            req.id = request.entity.id
        # ros_gz_interfaces/Entity and gz.msgs.Entity use the same numbering, and
        # MODEL is the only sensible default for a prop.
        req.type = request.entity.type or Entity.MODEL

        ok, detail = self._call('remove', req, Entity)
        if not ok:
            self.get_logger().error(f'delete failed: {detail}')
        else:
            self.get_logger().info(f'deleted "{request.entity.name}"')
        response.success = ok
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PropManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

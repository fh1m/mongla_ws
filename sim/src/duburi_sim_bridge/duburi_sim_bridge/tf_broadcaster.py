#!/usr/bin/env python3
"""Publish TF, and publish the vehicle's BELIEF next to the truth.

Nothing in this stack published TF before this node, which is why RViz was not
usable: without `odom -> base_link` every display has nothing to draw against.

It publishes two things deliberately:

  odom -> base_link          Gazebo ground truth. What is actually happening.
  /duburi/sim/pose_truth     the same pose, as a PoseStamped
  /duburi/sim/pose_believed  where the STACK thinks it is

The second one is the reason this node is worth having rather than a bare
`static_transform_publisher`. The whole sim audit turns on the gap between
measured and believed -- depth is read from `AHRS2.altitude`, which sits ~0.33 m
off truth at the surface, and the DVL integrator drifts. Those are currently
numbers in a troubleshooting table. Side by side in RViz they are a thing you
can watch.

`pose_believed` takes x/y from the DVL integrator when it is running and z from
`/duburi/state`'s depth, i.e. exactly the quantities the control loops act on.
When the stack is down it simply is not published, and the absence is honest.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

try:
    from duburi_interfaces.msg import DuburiState
except ImportError:      # autonomy overlay not sourced; truth still works
    DuburiState = None


class SimTf(Node):
    def __init__(self):
        super().__init__('duburi_sim_tf')
        self.declare_parameter('odom_topic', '/duburi/sim/ground_truth')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf = TransformBroadcaster(self)
        self.pub_truth = self.create_publisher(
            PoseStamped, '/duburi/sim/pose_truth', 10)
        self.pub_belief = self.create_publisher(
            PoseStamped, '/duburi/sim/pose_believed', 10)

        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10)

        self._depth = None
        self._yaw = None
        if DuburiState is not None:
            self.create_subscription(
                DuburiState, '/duburi/state', self._on_state, 10)
        else:
            self.get_logger().warn(
                '[TF   ] duburi_interfaces not importable -- publishing truth '
                'only, no believed pose. Source the autonomy overlay first.')

        self.get_logger().info(
            f'[TF   ] {self.odom_frame} -> {self.base_frame} from ground truth')

    def _on_state(self, msg):
        self._depth = msg.depth_m
        self._yaw = msg.yaw_deg

    def _on_odom(self, msg):
        stamp = msg.header.stamp
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(t)

        truth = PoseStamped()
        truth.header.stamp = stamp
        truth.header.frame_id = self.odom_frame
        truth.pose = msg.pose.pose
        self.pub_truth.publish(truth)

        if self._depth is None or math.isnan(self._depth):
            return
        # Believed pose: truth's x/y (we have no independent position estimate
        # to draw) but the DEPTH the stack actually acts on. Drawn at the same
        # x/y on purpose, so any visible vertical separation between the two
        # markers is the AHRS2 offset and nothing else.
        belief = PoseStamped()
        belief.header.stamp = stamp
        belief.header.frame_id = self.odom_frame
        belief.pose.position.x = msg.pose.pose.position.x
        belief.pose.position.y = msg.pose.pose.position.y
        belief.pose.position.z = float(self._depth)
        belief.pose.orientation = msg.pose.pose.orientation
        self.pub_belief.publish(belief)


def main(args=None):
    rclpy.init(args=args)
    node = SimTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

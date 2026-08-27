#!/usr/bin/env python3
"""Republish the Gazebo DVL onto ROS.

`ros_gz_bridge` cannot carry this one. There is no `gz.msgs.DVLVelocityTracking`
conversion in ros_gz_bridge and no `DVLVelocityTracking.msg` in
ros_gz_interfaces (checked against the installed Humble tree), so the standard
`parameter_bridge` route does not exist. This node speaks gz-transport directly
and publishes plain ROS messages instead of patching ros_gz_bridge.

Publishes:
  <ns>/velocity   geometry_msgs/TwistWithCovarianceStamped  body-frame m/s
  <ns>/altitude   sensor_msgs/Range                         bottom-track range

VALIDITY IS NOT OPTIONAL. The gz sensor keeps publishing when it loses bottom
lock, and a stale or water-mass reading republished as a good velocity is
exactly the silent-wrong failure this simulator exists to catch (see
`.context/TROUBLESHOOTING.md` on AHRS2 depth). A sample is forwarded only when
it is a BOTTOM target with a positive range; anything else is dropped and
counted, and the counter is logged so the drop is visible rather than inferred.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Range

from gz.transport13 import Node as GzNode
from gz.msgs10.dvl_velocity_tracking_pb2 import DVLVelocityTracking

# gz.msgs10.dvl_tracking_target_pb2.DVLTrackingTarget.TargetType
_TARGET_BOTTOM = 1
# gz.msgs10.dvl_kinematic_estimate_pb2.DVLKinematicEstimate.ReferenceType
_REFERENCE_SHIP = 2

# Matches <arrangement> tilt in duburi_sim_description/models/duburi_heavy/model.sdf.in
_BEAM_TILT_DEG = 30.0


class DvlBridge(Node):
    def __init__(self):
        super().__init__('duburi_sim_dvl_bridge')
        self.declare_parameter('gz_topic', '/dvl/velocity')
        self.declare_parameter('frame_id', 'duburi/dvl_link')
        self.declare_parameter('ns', '/duburi/sim/dvl')
        gz_topic = self.get_parameter('gz_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        ns = self.get_parameter('ns').value

        self.pub_vel = self.create_publisher(
            TwistWithCovarianceStamped, f'{ns}/velocity', 10)
        self.pub_alt = self.create_publisher(Range, f'{ns}/altitude', 10)

        self._n_ok = 0
        self._n_dropped = 0

        self._gz = GzNode()
        if not self._gz.subscribe(DVLVelocityTracking, gz_topic, self._on_dvl):
            raise RuntimeError(
                f'could not subscribe to gz topic {gz_topic}. Is the sim up, and '
                f'does the world load gz::sim::systems::DopplerVelocityLogSystem? '
                f'Without that system the sensor loads and never publishes.')
        self.get_logger().info(f'[DVL  ] gz {gz_topic} -> ROS {ns}/velocity, {ns}/altitude')
        self.create_timer(10.0, self._report)

    def _report(self):
        total = self._n_ok + self._n_dropped
        if total:
            self.get_logger().info(
                f'[DVL  ] {self._n_ok} bottom-locked, {self._n_dropped} dropped '
                f'({100.0 * self._n_dropped / total:.0f}% no lock)')

    def _on_dvl(self, msg: DVLVelocityTracking) -> None:
        target_ok = (msg.target.type == _TARGET_BOTTOM
                     and msg.target.range.mean > 0.0)
        if not target_ok:
            self._n_dropped += 1
            return
        self._n_ok += 1

        stamp = self.get_clock().now().to_msg()

        tw = TwistWithCovarianceStamped()
        tw.header.stamp = stamp
        tw.header.frame_id = self.frame_id
        # DVL_REFERENCE_SHIP is body frame, which is what the consumer integrates.
        # Anything else would need a rotation we are not doing, so say so loudly
        # rather than publishing a number in the wrong frame.
        if msg.velocity.reference != _REFERENCE_SHIP:
            self.get_logger().warn(
                f'[DVL  ] velocity reference {msg.velocity.reference} is not '
                f'SHIP/body frame -- not republishing, the consumer integrates '
                f'body-frame velocity', throttle_duration_sec=10.0)
            return
        tw.twist.twist.linear.x = msg.velocity.mean.x
        tw.twist.twist.linear.y = msg.velocity.mean.y
        tw.twist.twist.linear.z = msg.velocity.mean.z
        cov = list(msg.velocity.covariance)
        if len(cov) == 9:                       # linear 3x3 into the 6x6 block
            for r in range(3):
                for c in range(3):
                    tw.twist.covariance[r * 6 + c] = cov[r * 3 + c]
        self.pub_vel.publish(tw)

        rng = Range()
        rng.header.stamp = stamp
        rng.header.frame_id = self.frame_id
        rng.radiation_type = Range.ULTRASOUND
        rng.field_of_view = math.radians(2.0 * _BEAM_TILT_DEG)
        rng.min_range = 0.1
        rng.max_range = 50.0
        rng.range = msg.target.range.mean
        self.pub_alt.publish(rng)


def main(args=None):
    rclpy.init(args=args)
    node = DvlBridge()
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

#!/usr/bin/env python3

"""Assert that the simulator satisfies the drop-in contract duburi_ws expects.

The simulator is only useful if the autonomy stack cannot tell it from the real
vehicle. That contract is small and precise, so it is worth checking mechanically
rather than by eye:

  * /duburi/sim/front_camera/image_raw   640x480, rgb8, publishing
  * /duburi/sim/bottom_camera/image_raw  640x480, rgb8, publishing
  * /duburi/sim/{front,bottom}_camera/camera_info  publishing, matching size
  * /duburi/sim/ground_truth             publishing

MAVLink cannot be checked from here; it is not a ROS topic. Run
`ros2 topic echo /duburi/state` against duburi_ws for that half.

Usage:
    ros2 run duburi_sim_bridge contract_check
    ros2 run duburi_sim_bridge contract_check --ros-args -p timeout:=30.0

Exits non-zero if any requirement is unmet, so it can gate CI.
"""

import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

EXPECTED_WIDTH = 640
EXPECTED_HEIGHT = 480

IMAGE_TOPICS = [
    '/duburi/sim/front_camera/image_raw',
    '/duburi/sim/bottom_camera/image_raw',
]
INFO_TOPICS = [
    '/duburi/sim/front_camera/camera_info',
    '/duburi/sim/bottom_camera/camera_info',
]


class ContractCheck(Node):

    def __init__(self):
        super().__init__('duburi_sim_contract_check')
        self.declare_parameter('timeout', 20.0)
        self.declare_parameter('min_messages', 5)

        self.timeout = self.get_parameter('timeout').value
        self.min_messages = self.get_parameter('min_messages').value

        self.counts = {}
        self.problems = {}

        for topic in IMAGE_TOPICS:
            self.counts[topic] = 0
            self.create_subscription(
                Image, topic, self._make_image_cb(topic), 10
            )
        for topic in INFO_TOPICS:
            self.counts[topic] = 0
            self.create_subscription(
                CameraInfo, topic, self._make_info_cb(topic), 10
            )

        # Ground truth is optional in the sense that duburi_ws never reads it,
        # but its absence means the odometry publisher or the bridge is
        # misconfigured, which is worth knowing.
        self.ground_truth_seen = False
        try:
            from nav_msgs.msg import Odometry
            self.create_subscription(
                Odometry,
                '/duburi/sim/ground_truth',
                self._ground_truth_cb,
                10,
            )
        except ImportError:  # pragma: no cover
            self.get_logger().warning('nav_msgs unavailable, skipping ground truth')

        self.get_logger().info(
            f'watching {len(IMAGE_TOPICS) + len(INFO_TOPICS)} topics '
            f'for up to {self.timeout:.0f}s'
        )

    def _make_image_cb(self, topic):
        def cb(msg):
            self.counts[topic] += 1
            if msg.width != EXPECTED_WIDTH or msg.height != EXPECTED_HEIGHT:
                self.problems[topic] = (
                    f'{msg.width}x{msg.height}, expected '
                    f'{EXPECTED_WIDTH}x{EXPECTED_HEIGHT}. The sim_front and '
                    'sim_bottom profiles in duburi_vision assume the latter.'
                )
            elif msg.encoding not in ('rgb8', 'bgr8'):
                self.problems[topic] = f'encoding {msg.encoding}, expected rgb8 or bgr8'
        return cb

    def _make_info_cb(self, topic):
        def cb(msg):
            self.counts[topic] += 1
            if msg.width != EXPECTED_WIDTH or msg.height != EXPECTED_HEIGHT:
                self.problems[topic] = (
                    f'{msg.width}x{msg.height}, expected '
                    f'{EXPECTED_WIDTH}x{EXPECTED_HEIGHT}'
                )
        return cb

    def _ground_truth_cb(self, _msg):
        self.ground_truth_seen = True

    def done(self) -> bool:
        return all(n >= self.min_messages for n in self.counts.values())

    def report(self) -> bool:
        """Print a summary. Returns True if the contract holds."""
        ok = True
        print('\nDuburi simulator contract check')
        print('-' * 62)
        for topic, count in self.counts.items():
            problem = self.problems.get(topic)
            if count < self.min_messages:
                status, ok = 'NO DATA', False
            elif problem:
                status, ok = 'BAD', False
            else:
                status = 'ok'
            print(f'  {status:<8} {topic}  ({count} msgs)')
            if problem:
                print(f'           {problem}')

        gt = 'ok' if self.ground_truth_seen else 'NO DATA'
        if not self.ground_truth_seen:
            ok = False
        print(f'  {gt:<8} /duburi/sim/ground_truth')

        print('-' * 62)
        if ok:
            print('contract satisfied: duburi_ws can run against this simulator '
                  'unmodified\n')
        else:
            print('contract NOT satisfied - duburi_ws will not see what it '
                  'expects\n')
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = ContractCheck()

    deadline = node.get_clock().now().nanoseconds + int(node.timeout * 1e9)
    try:
        while rclpy.ok() and not node.done():
            if node.get_clock().now().nanoseconds > deadline:
                break
            rclpy.spin_once(node, timeout_sec=0.2)
        ok = node.report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

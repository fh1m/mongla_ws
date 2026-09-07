#!/usr/bin/env python3
"""Measure a hand slide THROUGH THE LAUNCH. The operator's button is the gate.

⛔ WHY THIS EXISTS. Every number in `measured-bars.md` §13 -- including the
30 cm result -- came from `flow_console.py --calibration <path>`, a tool that
passed the calibration by hand. The launch was never in that loop, and when
it was finally looked at, it had the calibration wired to the WRONG CAMERA
(§17). A verification that passes a path by hand does not verify the launch.

So this drives ONLY the shipped surface: `distance_control` start/stop and
`distance_traveled` out, on the node the launch started, with the calibration
the launch chose. It has no calibration argument on purpose -- if it had one,
it would be the same mistake again.

It also supplies `/duburi/state`, because on a dry bench no manager is
running and the node REFUSES without a depth (height multiplies every
velocity it emits, so refusing is correct). `--height` is the tape measure
from the lens to the floor.

THE BUTTON IS THE DESIGN, not a convenience. Three separate captures last
session read a correct sensor as short -- 19.6 cm for a 30.3 cm slide, and
worse -- every one of them from an automatic start/stop heuristic treating
"I could not measure" as "it did not move". No window short enough to be
responsive can tell a slow hand from a still rig. Press, slide, press.

Terminal A:
    ros2 launch duburi_vision vision_pi.launch.py \
        flow:=true flow_medium:=air pool_depth_m:=<height_m> forward:=false
Terminal B:
    python3 tools/flow_launch_check.py --height <height_m> --truth-cm 30
"""
import argparse
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, UInt8

from duburi_interfaces.msg import DuburiState


class Check(Node):
    def __init__(self, cam, height, lateral):
        super().__init__('flow_launch_check')
        ns = f'/duburi/vision/{cam}'
        self._ctrl = self.create_publisher(String, f'{ns}/distance_control', 10)
        self._state = self.create_publisher(DuburiState, '/duburi/state', 10)
        self.create_subscription(Float32, f'{ns}/distance_traveled',
                                 self._on_dist, 10)
        self.create_subscription(UInt8, f'{ns}/flow_quality', self._on_q, 10)
        self._height = height
        self._lateral = lateral
        self.dist = None
        self.q_seen = 0
        self.q_zero = 0
        # depth 0 => height above floor == pool_depth_m == the tape measure.
        self.create_timer(0.1, self._tick)

    def _tick(self):
        m = DuburiState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.depth_m = 0.0
        m.yaw_deg = 0.0
        self._state.publish(m)

    def _on_dist(self, msg):
        self.dist = float(msg.data)

    def _on_q(self, msg):
        self.q_seen += 1
        if int(msg.data) == 0:
            self.q_zero += 1

    def send(self, what):
        m = String()
        m.data = what
        self._ctrl.publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera', default='downward')
    ap.add_argument('--height', type=float, required=True,
                    help='tape measure, lens to floor, metres. Must equal the '
                         'pool_depth_m the launch was given.')
    ap.add_argument('--truth-cm', type=float, default=30.0)
    ap.add_argument('--lateral', action='store_true')
    a = ap.parse_args()

    rclpy.init()
    n = Check(a.camera, a.height, a.lateral)
    t = threading.Thread(target=rclpy.spin, args=(n,), daemon=True)
    t.start()
    try:
        print(f'\nheight {a.height} m   truth {a.truth_cm} cm   '
              f'{"LATERAL" if a.lateral else "AXIAL"}')
        print('Waiting for the node to publish quality...')
        import time
        t0 = time.time()
        while n.q_seen == 0 and time.time() - t0 < 15:
            time.sleep(0.2)
        if n.q_seen == 0:
            print('\n  NO flow_quality in 15 s. The node is not running, or '
                  'is on another camera.\n  Check terminal A -- it prints a '
                  '[FLOW ] banner naming the camera and f.')
            return 1
        print(f'  node is alive ({n.q_seen} quality msgs, '
              f'{n.q_zero} of them zero)\n')

        input('Put the rig at the START mark, then press ENTER to arm > ')
        n.q_seen = n.q_zero = 0
        n.send('start lateral' if a.lateral else 'start')
        print('  ARMED. Slide to the END mark, then press ENTER.')
        input('  > ')
        n.send('stop')
        time.sleep(1.0)
        if n.dist is None:
            print('  no distance published -- see terminal A for refusals')
            return 1
        got = n.dist * 100.0
        err = got - a.truth_cm
        print(f'\n  truth {a.truth_cm:6.2f} cm    measured {got:7.2f} cm'
              f'    err {err:+6.2f} cm    {100*got/a.truth_cm:6.1f} %')
        print(f'  quality: {n.q_seen} intervals, {n.q_zero} refused '
              f'({100.0*n.q_zero/max(1,n.q_seen):.0f} %)')
        print('\n  A hand slide is itself good to about +/-1 cm, so an error '
              'inside\n  that is an UPPER BOUND on the sensor, not its error.')
        return 0
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

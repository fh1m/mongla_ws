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
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Float32, String, UInt8

from rcl_interfaces.msg import Log

from duburi_interfaces.msg import DuburiState


class Check(Node):
    def __init__(self, cam, height, lateral):
        super().__init__('flow_launch_check')
        ns = f'/duburi/vision/{cam}'
        # ⛔ MUST MATCH THE NODE'S SUBSCRIPTION, which is RELIABLE +
        # TRANSIENT_LOCAL. A plain depth-10 publisher is VOLATILE, and rclpy
        # answers a durability mismatch with ONE warning and then silence --
        # `start` is accepted by the publisher and delivered to nobody. That
        # is exactly the failure this tool exists to catch in the pipeline
        # (round 28: a RELIABLE subscriber against BEST_EFFORT publishers gave
        # a clean launch, healthy nodes and zero frames), reproduced in the
        # instrument itself on its first live run: 74 intervals, 100 %
        # refused, 0.00 cm, and the sensor was never asked to start.
        ctrl_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._ctrl = self.create_publisher(String, f'{ns}/distance_control',
                                           ctrl_qos)
        self._state = self.create_publisher(DuburiState, '/duburi/state', 10)
        self.create_subscription(Float32, f'{ns}/distance_traveled',
                                 self._on_dist, 10)
        self.create_subscription(UInt8, f'{ns}/flow_quality', self._on_q, 10)
        self._height = height
        self._lateral = lateral
        # The node's REFUSAL REASON is the most useful thing it emits and it
        # only went to the launch terminal, so a failed capture here showed
        # "0.00 cm, 100 % refused" with the explanation in another window.
        # /rosout carries it; read it and report it in place.
        self.create_subscription(Log, '/rosout', self._on_log,
                                 QoSProfile(depth=64,
                                            reliability=QoSReliabilityPolicy
                                            .RELIABLE))
        self.reasons = []
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

    def _on_log(self, msg):
        if 'flow' not in str(msg.name).lower():
            return
        t = str(msg.msg)
        if 'REFUSING' in t:
            self.reasons.append(t.split('REFUSING:', 1)[-1].split('(q=0')[0]
                                .strip())

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
        t0 = time.time()
        while n.q_seen == 0 and time.time() - t0 < 15:
            time.sleep(0.2)
        if n.q_seen == 0:
            print('\n  NO flow_quality in 15 s. The node is not running, or '
                  'is on another camera.\n  Check terminal A -- it prints a '
                  '[FLOW ] banner naming the camera and f.')
            return 1
        print(f'  node is alive ({n.q_seen} quality msgs, '
              f'{n.q_zero} of them zero)')
        # A publisher with nobody compatible listening is silent, not an
        # error. Check the match BEFORE the operator does any work.
        t0 = time.time()
        while n.count_subscribers(n._ctrl.topic_name) == 0 and \
                time.time() - t0 < 5:
            time.sleep(0.2)
        subs = n.count_subscribers(n._ctrl.topic_name)
        if subs == 0:
            print(f'\n  NOTHING is subscribed to {n._ctrl.topic_name} with a '
                  f'compatible QoS.\n  start/stop would be accepted and '
                  f'delivered to nobody -- refusing to\n  waste a slide. '
                  f'The node wants RELIABLE + TRANSIENT_LOCAL.')
            return 1
        print(f'  control link matched ({subs} subscriber)\n')

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
        if n.reasons:
            from collections import Counter
            print('  why it refused:')
            for why, k in Counter(n.reasons).most_common(3):
                print(f'    x{k:<4d} {why}')
        print('\n  A hand slide is itself good to about +/-1 cm, so an error '
              'inside\n  that is an UPPER BOUND on the sensor, not its error.')
        return 0
    finally:
        # Shut the executor down BEFORE destroying the node, or the spin
        # thread runs on into a destroyed node and aborts with
        # "terminate called without an active exception" -- which looks like
        # a crash in the measurement and is not.
        rclpy.shutdown()
        t.join(timeout=2.0)


if __name__ == '__main__':
    sys.exit(main())

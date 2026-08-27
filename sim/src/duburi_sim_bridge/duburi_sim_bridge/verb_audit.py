#!/usr/bin/env python3
"""Measure what each /duburi/move verb PHYSICALLY does, against ground truth.

The stack's own telemetry cannot referee itself -- that is how the AHRS2 depth
offset (0.33 m at the surface) and the `move_forward_dist` dead-reckoning
(2.361 m travelled on a 1.0 m command, reported "completed") both survived. Every
number here comes from /duburi/sim/ground_truth.

Two failure shapes are worth naming:

  Type A  false NEGATIVE -- the verb fails, the physics succeeded. `surface()`.
                            Visible, annoying.
  Type B  false POSITIVE -- the verb succeeds while the vehicle did something
                            else, or nothing. Invisible. This is the dangerous
                            one, and it is what this tool exists to find.

TWO METHOD RULES, both learned by getting them wrong:

1. ROS REP-103: body +y is PORT, not starboard. An earlier expectation table
   said starboard and flagged move_left/move_right as inverted. They are not:
   move_right emits Ch6=1720 (>1500 = starboard, the pool-verified hull
   polarity) and the hull moves to negative body-y, which IS rightward.

2. NEVER measure rotation from a before/after pair. The hull carries angular
   momentum through the settle and wraps past +/-180, so the same teleop input
   measured -27.7 deg once and +169.7 deg the next time. Yaw is SAMPLED
   CONTINUOUSLY here and accumulated unwrapped.

    ros2 run duburi_sim_bridge verb_audit            # everything safe to run
    ros2 run duburi_sim_bridge verb_audit --only turn,arc
"""
import argparse
import math
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def _yaw_of(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class GroundTruth(Node):
    """Continuous pose sampler. Yaw is accumulated UNWRAPPED (see rule 2)."""

    def __init__(self):
        super().__init__('duburi_verb_audit_gt')
        self._lock = threading.Lock()
        self.x = self.y = self.z = 0.0
        self.yaw = 0.0            # instantaneous, wrapped [-180, 180]
        self.yaw_cum = 0.0        # accumulated, unwrapped
        self._last_yaw = None
        self.have = False
        self.create_subscription(
            Odometry, '/duburi/sim/ground_truth', self._cb, 10)

    def _cb(self, msg):
        p = msg.pose.pose.position
        yaw = _yaw_of(msg.pose.pose.orientation)
        with self._lock:
            self.x, self.y, self.z = p.x, p.y, p.z
            if self._last_yaw is not None:
                d = yaw - self._last_yaw
                # unwrap the step, not the total
                d = (d + 180.0) % 360.0 - 180.0
                self.yaw_cum += d
            self._last_yaw = yaw
            self.yaw = yaw
            self.have = True

    def snap(self):
        with self._lock:
            return (self.x, self.y, self.z, self.yaw, self.yaw_cum)


def body_delta(p0, p1):
    """World displacement expressed in the body frame at p0. REP-103: +y PORT."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    r = math.radians(p0[3])
    fwd = dx * math.cos(r) + dy * math.sin(r)
    port = -dx * math.sin(r) + dy * math.cos(r)
    return fwd, port


def run_verb(verb, args='', timeout=180):
    cmd = (f'ros2 run duburi_planner duburi {verb} {args}')
    out = subprocess.run(['bash', '-lc' if False else '-c',
                          f'source /opt/ros/humble/setup.bash >/dev/null 2>&1; {cmd} 2>&1'],
                         capture_output=True, text=True, timeout=timeout + 30)
    txt = out.stdout
    ok = '-> OK' in txt
    msg = ''
    for line in txt.splitlines():
        if 'msg=' in line:
            msg = line.split('msg=')[-1].strip().strip('"')
    return ok, msg, txt


# (verb, args, axis, expected sign, note). axis: fwd|port|yaw|depth|none
# Field names come from duburi_control/commands.py -- guessing them produces a
# clean-looking "failed" row that is the harness's fault, not the verb's.
CASES = [
    # `turn` is ABSOLUTE, so the sign of the rotation depends on where the hull
    # already points. Checked by final heading, not by direction (axis 'turn').
    ('turn',            '--target 60 --timeout 60',    'turn',   0,
     'absolute heading; verified against the commanded target, not a direction'),
    ('head',            '',                            'none',   0, 'telemetry read'),
    ('arc',             '--duration 5 --gain 45 --target_yaw 40', 'yaw', 0,
     'curved path; the only motion verb claiming a real error_value'),
    ('style_yaw',       '--flips 1',                   'yaw',    0, 'cosmetic spin'),
    ('style_roll',      '--flips 1',                   'none',   0, 'cosmetic roll'),
    ('lock_heading',    '--target 0 --timeout 20',     'none',   0, 'see disturbance test'),
    ('unlock_heading',  '',                            'none',   0, ''),
    ('set_mode',        '--target_name ALT_HOLD',      'none',   0, ''),
    ('move_back_dist',  '--distance_m 1.0 --gain 55',  'fwd',   -1, 'needs DVL'),
    ('move_lateral_dist', '--distance_m 1.0 --gain 45', 'port', -1,
     'needs DVL; +distance_m = signed_dir +1 = starboard = NEGATIVE port (REP-103)'),
    ('fire',            '--fire_channel 1',            'none',   0, 'no payload hardware in sim'),
]


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', default='', help='comma list of verbs')
    ap.add_argument('--settle', type=float, default=4.0)
    a = ap.parse_args(argv)
    wanted = {v.strip() for v in a.only.split(',') if v.strip()}

    rclpy.init()
    gt = GroundTruth()
    spin = threading.Thread(target=rclpy.spin, args=(gt,), daemon=True)
    spin.start()

    t0 = time.time()
    while not gt.have and time.time() - t0 < 20:
        time.sleep(0.2)
    if not gt.have:
        print('no /duburi/sim/ground_truth -- is the sim up?', file=sys.stderr)
        return 2

    run_verb('arm')
    time.sleep(2)
    rows = []
    for verb, args, axis, sign, note in CASES:
        if wanted and verb not in wanted:
            continue
        # Re-arm before each case. style_roll and any mid-run failure can leave
        # the vehicle disarmed, and then every later verb reports "AUV is
        # disarmed" -- rows that look like verb faults but are harness state.
        run_verb('arm')
        time.sleep(1.5)
        p0 = gt.snap()
        ok, msg, _ = run_verb(verb, args)
        time.sleep(a.settle)
        p1 = gt.snap()
        fwd, port = body_delta(p0, p1)
        dyaw = p1[4] - p0[4]          # unwrapped, so a multi-turn spin is honest
        if axis == 'turn':
            # Absolute heading: the question is "did it ARRIVE", not "which way
            # did it go".
            #
            # FRAME RELATION, measured (residuals -0.0 / -0.2 / +0.2 deg over
            # targets 60 / 120 / 0):
            #
            #     gt_yaw = 90 - stack_yaw
            #
            # The stack is a compass heading -- CW-positive, zero at north.
            # Ground truth is ENU -- CCW-positive, zero at +x/east. So it is a
            # negation AND a 90 deg offset. Using the negation alone reports a
            # ~88 deg error on a turn that is actually accurate to 1.4 deg,
            # which is how a correct verb gets flagged Type B by a wrong
            # harness. Do not "simplify" this to -target.
            target = float(args.split('--target')[1].split()[0])
            err = ((90.0 - target) - p1[3] + 180.0) % 360.0 - 180.0
            moved, unit = err, 'deg err'
            typeb = ok and abs(err) > 10.0
        else:
            moved = {'fwd': fwd, 'port': port, 'yaw': dyaw,
                     'depth': p1[2] - p0[2], 'none': 0.0}[axis]
            unit = 'deg' if axis == 'yaw' else 'm'
            # Type B = verb said OK while the axis it claims to drive did not move
            typeb = ok and sign != 0 and (moved * sign <= 0 or abs(moved) < 0.1)
        rows.append((verb, ok, axis, moved, unit, typeb, msg, note))
        print(f'  {verb:20s} ok={str(ok):5s} {axis:5s}={moved:+8.2f}{unit}'
              f'{"  <-- TYPE B?" if typeb else ""}   {msg[:60]}')
        time.sleep(2)
    run_verb('disarm')

    print('\n=== summary ===')
    for verb, ok, axis, moved, unit, typeb, msg, note in rows:
        flag = 'TYPE-B' if typeb else ('ok' if ok else 'failed')
        print(f'  {flag:7s} {verb:20s} {axis:5s} {moved:+8.2f}{unit}  {note}')
    gt.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

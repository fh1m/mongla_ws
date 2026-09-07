#!/usr/bin/env python3
"""Three de-rotation settings, ONE slide, measured simultaneously.

⛔ WHY. Four launch-path slides of a 30 cm truth read -8.17, +3.98, +3.38 cm
-- short, and one with the sign wrong. The refusals name a suspect:
`rotation-dominated (1.43 of the flow)` means the de-rotation term the node
subtracted was LARGER than the total flow it measured. That is the recorded
failure mode, in the record's own words: "a bad correction is worse than none
-- de-rotation with an unvalidated mapping destroyed 35.7 cm of real travel."

The gyro gains were calibrated in `measured-bars.md` §12 on a rig where the
camera and the IMU were rigidly coupled in a known orientation. The IMU is
now the SROT BOARD. Nothing has re-checked that the board sits the same way
round relative to this camera, and a swapped or sign-flipped axis subtracts
real translation while looking like a working correction.

ONE SLIDE, THREE ARMS. `flow_node` reads its frames off a TOPIC, so extra
instances can watch the same camera at no cost and be driven by the same
`distance_control` message. Every arm therefore sees the IDENTICAL motion --
which is the only way to compare, since a hand slide is not repeatable to
better than about a centimetre and the effect being chased is larger than
that but the arms differ by less.

  A  shipped   gains (-1, -1)   the launch's own node, untouched
  B  none      gains ( 0,  0)   de-rotation disabled entirely
  C  flipped   gains (+1, +1)   both signs inverted

Read it as a discriminator, not as a calibration:
  * B >> A            -> de-rotation is destroying travel. Mapping is wrong.
  * C >> A            -> the SIGNS are wrong (§12's convention no longer holds).
  * A ~ B ~ C         -> rotation is not the cause; look elsewhere.
  * all three short   -> the loss is upstream of de-rotation.

Arm A is the launch's node and stays the reference. Arms B and C are
diagnostic instances and take the calibration explicitly -- read off the
RUNNING node rather than hardcoded, so they cannot disagree with it.

Prerequisites are the three processes `water-owed.md` §5c lists: the manager
(for /duburi/imu_rates at 50 Hz -- without it every interval refuses), the
vision launch with flow:=true, and this.
"""
import argparse
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Float32, String, UInt8

from duburi_interfaces.msg import DuburiState

ARMS = [('B_none', 0.0, 0.0), ('C_flipped', 1.0, 1.0)]


def running_param(node_name, param):
    """Read a parameter off the node the LAUNCH started, so the diagnostic
    arms cannot silently use a different calibration than the reference."""
    out = subprocess.run(['ros2', 'param', 'get', node_name, param],
                         capture_output=True, text=True, timeout=20)
    txt = (out.stdout or '').strip()
    if 'value is:' not in txt:
        return None
    return txt.split('value is:', 1)[1].strip()


class AB(Node):
    def __init__(self, cam, height):
        super().__init__('flow_derot_ab')
        ns = f'/duburi/vision/{cam}'
        ctrl_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._ctrl = self.create_publisher(String, f'{ns}/distance_control',
                                           ctrl_qos)
        self._state = self.create_publisher(DuburiState, '/duburi/state', 10)
        self.dist = {}
        # ⛔ COUNT THE INTERVALS PER ARM. An arm reading 0.00 cm is either
        # "measured, and it did not move" or "measured nothing at all", and
        # those are opposite conclusions that print identically. Absence is
        # not zero -- the rule that has already caught the barometer, the ESC
        # gate and a covered lens reporting 11 px of flow. Without this the
        # A/B could hand back a clean-looking table that means nothing.
        self.q_all = {}
        self.q_ok = {}
        self.create_subscription(
            Float32, f'{ns}/distance_traveled',
            lambda m: self.dist.__setitem__('A_shipped', float(m.data)), 10)
        self._count(f'{ns}/flow_quality', 'A_shipped')
        for name, _, _ in ARMS:
            self.create_subscription(
                Float32, f'/ab/{name}/distance_traveled',
                (lambda n: lambda m: self.dist.__setitem__(n, float(m.data)))(name),
                10)
            self._count(f'/ab/{name}/flow_quality', name)
        self._h = height
        self.create_timer(0.1, self._tick)

    def _count(self, topic, name):
        self.q_all[name] = 0
        self.q_ok[name] = 0

        def cb(m, n=name):
            self.q_all[n] += 1
            if int(m.data) > 0:
                self.q_ok[n] += 1
        self.create_subscription(UInt8, topic, cb, 10)

    def reset_counts(self):
        for k in self.q_all:
            self.q_all[k] = 0
            self.q_ok[k] = 0

    def _tick(self):
        m = DuburiState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.depth_m = 0.0
        # ⛔ YAW MUST BE NaN, NOT ZERO. The manager also publishes
        # /duburi/state, with the board's real heading (~180 deg here). The
        # node keeps whichever arrived last, and `DistanceAccumulator`
        # projects with `e = yaw - axis_yaw`: with two publishers disagreeing
        # by 180 deg, `cos(e)` FLIPS SIGN between intervals and the
        # contributions cancel. That is what made 30 cm slides read 0.6-5 cm
        # with an unstable sign, and it is entirely an artefact of this tool.
        # The node skips a NaN field, so NaN means "I have no opinion, keep
        # the board's" -- which is the only honest thing for a fake state
        # message to say about a quantity it does not measure.
        m.yaw_deg = float('nan')
        self._state.publish(m)

    def send(self, what):
        m = String()
        m.data = what
        self._ctrl.publish(m)


def spawn(name, gx, gy, cam, cal, height, medium):
    ns = f'/duburi/vision/{cam}'
    cmd = ['ros2', 'run', 'duburi_vision', 'flow_node', '--ros-args',
           '-r', f'__node:=duburi_flow_{name}',
           '-p', f'camera:={cam}',
           '-p', f'medium:={medium}',
           '-p', f'pool_depth_m:={height}',
           '-p', f'gyro_gain_x:={gx}',
           '-p', f'gyro_gain_y:={gy}',
           '-p', 'estimate_time_offset:=false']
    if cal:
        cmd += ['-p', f'calibration:={cal}']
    # Outputs remapped so the arms cannot overwrite each other's topics;
    # distance_control is left ALONE on purpose, so one start/stop drives
    # every arm and they are guaranteed to see the same interval.
    for topic in ('distance_traveled', 'velocity', 'flow_quality',
                  'distance_debug'):
        cmd += ['-r', f'{ns}/{topic}:=/ab/{name}/{topic}']
    # ⛔ OWN THE PROCESS GROUP. `ros2 run` is a launcher: it execs the node
    # as a CHILD, so Popen.terminate() kills the wrapper and leaves the node
    # running. Measured: after three invocations there were THREE live copies
    # of every arm, all publishing to the same remapped topic, and the tool
    # read whichever published last -- an A/B whose arms were each three
    # nodes deep. `start_new_session` puts the wrapper and its child in one
    # group so the whole group can be signalled. Same failure the sim's
    # scoring `gz topic -e` had, which outlived every stop.
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                            start_new_session=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera', default='downward')
    ap.add_argument('--height', type=float, required=True)
    ap.add_argument('--truth-cm', type=float, default=30.0)
    ap.add_argument('--medium', default='air')
    ap.add_argument('--node', default='/duburi_flow_velocity')
    a = ap.parse_args()

    cal = running_param(a.node, 'calibration')
    if cal:
        cal = cal.strip().strip("'\"")
    print(f'calibration read off {a.node}: {cal or "(none)"}')

    procs = [spawn(n, gx, gy, a.camera, cal, a.height, a.medium)
             for n, gx, gy in ARMS]
    rclpy.init()
    n = AB(a.camera, a.height)
    t = threading.Thread(target=rclpy.spin, args=(n,), daemon=True)
    t.start()
    try:
        print('waiting for the extra arms to come up...')
        time.sleep(9)
        want = 1 + len(ARMS)
        subs = n.count_subscribers(n._ctrl.topic_name)
        print(f'  {subs} nodes listening on distance_control (want {want})')
        # BOTH directions. Checking only for "too few" is how three duplicate
        # copies of every arm went unnoticed through a whole measurement:
        # extra nodes are not a harmless surplus, they publish to the same
        # topic and the reader takes whichever was last.
        if subs != want:
            print(f'  WRONG NUMBER OF ARMS ({subs} != {want}).')
            if subs > want:
                print('  Orphaned flow_node processes are still running and '
                      'publishing to\n  these same topics. Clear them first:'
                      '\n\n    pkill -f "duburi_flow_[BC]_" \n')
            else:
                print('  Not every arm came up -- check the launch is '
                      'running.')
            return 1

        input('\nRig at the START mark, ENTER to arm > ')
        n.dist.clear()
        n.reset_counts()
        n.send('start')
        # Sent twice: on a stationary trial arm A returned its PREVIOUS run's
        # total with zero intervals used, i.e. `stop` without a `start`
        # returns the stale accumulator. A second start a beat later costs
        # nothing and removes that race from the comparison.
        time.sleep(0.4)
        n.send('start')
        print('  ARMED -- slide to the END mark, then ENTER.')
        input('  > ')
        n.send('stop')
        time.sleep(1.5)

        print(f'\n  truth {a.truth_cm:.1f} cm, one slide, all arms\n')
        order = ['A_shipped'] + [x[0] for x in ARMS]
        for k in order:
            v = n.dist.get(k)
            if v is None:
                print(f'    {k:12s}  no distance published')
                continue
            got = v * 100.0
            used, tot = n.q_ok.get(k, 0), n.q_all.get(k, 0)
            if not used:
                # Print no number at all. A stale accumulator formatted as
                # centimetres is indistinguishable from a measurement, and
                # this arm produced none.
                print(f'    {k:12s}  --  NO USED INTERVALS ({tot} seen). '
                      f'Any distance here is STALE, not a measurement.')
                continue
            print(f'    {k:12s}  {got:+8.2f} cm   {100*got/a.truth_cm:+7.1f} %'
                  f'   err {got - a.truth_cm:+7.2f} cm'
                  f'   [{used}/{tot} intervals used]')
        print('\n  A hand slide is good to about +/-1 cm, so read the SPREAD '
              'between\n  arms, not any one arm\'s absolute error.')
        return 0
    finally:
        rclpy.shutdown()
        for p in procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception:
                pass
        for p in procs:
            try:
                p.wait(timeout=5)
            except Exception:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except Exception:
                    pass
        # Belt and braces, VERIFIED NECESSARY: killing the process group
        # still left one copy of each arm alive, so the node is reparented
        # somewhere the group signal does not reach. Sweep by node name and
        # then CHECK, because a cleanup that is merely attempted is how three
        # duplicates accumulated in the first place.
        subprocess.run(['pkill', '-f', 'duburi_flow_[BC]_'],
                       capture_output=True)
        time.sleep(1.0)
        left = subprocess.run(['pgrep', '-f', 'duburi_flow_[BC]_'],
                              capture_output=True, text=True)
        if (left.stdout or '').strip():
            print('\n  ⚠ ARMS STILL RUNNING after cleanup: '
                  f'{len((left.stdout or "").split())} process(es).\n'
                  '  They publish to the same topics, so the NEXT run would '
                  'read them.\n  Clear with:  pkill -9 -f "duburi_flow_[BC]_"')


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3

"""Check the vehicle's thrust allocation against ArduSub's vectored_6dof frame.

ArduSub does not ask the vehicle how its thrusters are arranged. It applies a
fixed mixing table, and if the model's geometry disagrees the vehicle still
flies, just wrongly: a forward command yaws, a yaw command climbs, and the
symptom surfaces much later as an autonomy bug. This measures the model's real
allocation matrix and compares its signs to the table ArduSub will actually use.

Each thruster is pulsed on its own from rest and the resulting body-frame
velocity is recorded, which gives the direction of that thruster's column.

The pulse is deliberately short, and that matters more than it looks. The
vehicle is free rather than restrained, and a single thruster fired alone
produces far more moment than force, so it spins up fast: a second of one
vertical rolls it most of the way onto its side. Once that happens the reading
falls apart in two ways at once. Velocity built up in an earlier attitude no
longer lines up with the body axes, so heave reads as sway; and a body tumbling
about two axes generates rotation about the third, so roll and pitch together
manufacture a yaw that no thruster asked for. Kept short, the response stays
dominated by the applied wrench. The cost is that these are directions and
rough relative magnitudes, not calibrated gains.

Run against `pool_empty` with ArduSub disabled:

    ros2 launch duburi_sim_bringup sim.launch.py \\
        course:=pool_empty ardusub:=false bridge:=false gui:=false

    scripts/thruster_survey.py

Note that thrusters 1-4 are *expected* to disagree on the sign of surge. That is
what makes the frame vectored: ArduSub's forward channel drives them -1,-1,+1,+1
and the sways and yaws cancel. Equal positive thrust on all four is not a
forward command, it is a null command.
"""

import argparse
import sys

from thruster_rig import (
    HORIZONTAL_THRUSTERS,
    THRUSTERS,
    VERTICAL_THRUSTERS,
    ThrusterRig,
)

AXES = ('surge', 'sway', 'heave', 'roll', 'pitch', 'yaw')

# ArduSub's table, from AP_Motors6DOF::setup_motors, SUB_FRAME_VECTORED_6DOF:
#
#   motor   roll  pitch   yaw  throttle  forward  lateral
#     1      0      0     +1      0        -1       +1
#     2      0      0     -1      0        -1       -1
#     3      0      0     -1      0        +1       +1
#     4      0      0     +1      0        +1       -1
#     5     +1     -1      0     -1         0        0
#     6     -1     -1      0     -1         0        0
#     7     +1     +1      0     -1         0        0
#     8     -1     +1      0     -1         0        0
#
# Those are in ArduPilot's body frame: x forward, y right, z down. Gazebo's is x
# forward, y left, z up, which is a 180 degree rotation about x. Under it a
# velocity or rate (x, y, z) becomes (x, -y, -z), so surge and roll keep their
# sign while sway, heave, pitch and yaw invert. `throttle` is positive-up in
# ArduPilot's mixer, so its -1 column means positive thrust drives the vehicle
# down, which is negative heave in either frame.
#
# The table below is the result of that conversion: the sign each axis should
# take in Gazebo's body frame for one unit of positive thrust on that thruster.
# 0 means the axis should not be meaningfully excited at all.
EXPECTED = {
    #        surge sway heave roll pitch yaw
    1:     (   -1,  -1,    0,   0,    0,  -1),
    2:     (   -1,  +1,    0,   0,    0,  +1),
    3:     (   +1,  -1,    0,   0,    0,  +1),
    4:     (   +1,  +1,    0,   0,    0,  -1),
    5:     (    0,   0,   -1,  +1,   +1,   0),
    6:     (    0,   0,   -1,  -1,   +1,   0),
    7:     (    0,   0,   -1,  +1,   -1,   0),
    8:     (    0,   0,   -1,  -1,   -1,   0),
}


def describe(twist, floor: float) -> str:
    """Name the axes this thruster meaningfully excites."""
    parts = [f'{name} {value:+.2f}'
             for name, value in zip(AXES, twist) if abs(value) >= floor]
    return ', '.join(parts) if parts else '(no response)'


def check(index: int, twist, floor: float, cross_fraction: float):
    """Compare one thruster's measured column against the ArduSub table."""
    problems = []
    # Translational and rotational responses are in different units and differ
    # by an order of magnitude, so cross-coupling is judged against the largest
    # response of the same kind rather than against an absolute threshold.
    scale = (max(abs(v) for v in twist[:3]), max(abs(v) for v in twist[3:]))

    for axis_index, (axis, measured, want) in enumerate(
            zip(AXES, twist, EXPECTED[index])):
        if want == 0:
            limit = max(floor, cross_fraction * scale[axis_index // 3])
            if abs(measured) > limit:
                problems.append(
                    f'{axis} should be uncoupled but reads {measured:+.2f}, '
                    f'over {limit:.2f}')
        elif abs(measured) < floor:
            problems.append(f'{axis} should be {want:+d} but reads nothing')
        elif (measured > 0) != (want > 0):
            problems.append(
                f'{axis} should be {want:+d} but reads {measured:+.2f}')
    return problems


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--vehicle', default='duburi')
    parser.add_argument('--model', default='duburi_heavy')
    parser.add_argument('--thrust', type=float, default=20.0,
                        help='Newtons on the thruster under test.')
    parser.add_argument('--pulse', type=float, default=0.3,
                        help='Seconds to hold each thruster on. Long enough to '
                             'read, short enough not to tumble.')
    parser.add_argument('--coast', type=float, default=45.0,
                        help='Seconds to allow for returning to rest between '
                             'thrusters. Waiting ends as soon as the vehicle '
                             'is actually still, so this is only a cap.')
    parser.add_argument('--floor', type=float, default=0.01,
                        help='Smallest response counted as present.')
    parser.add_argument('--cross-fraction', type=float, default=0.25,
                        help='Response on a supposedly uncoupled axis, as a '
                             'fraction of the largest axis, above which it '
                             'counts as real cross-coupling.')
    args = parser.parse_args()

    rig = ThrusterRig(args.vehicle, args.model)
    if not rig.wait_for_odometry():
        print(f'FAIL: no odometry on /model/{args.vehicle}/odometry.\n'
              '  Is the simulator running, and is the vehicle really named '
              f'"{args.vehicle}"?')
        return 1

    unreachable = rig.wait_for_thrusters()
    if unreachable:
        print(f'FAIL: thrusters {unreachable} have no subscriber on '
              'cmd_thrust.\n'
              '  Commanding them now would be dropped, and they would show up\n'
              '  below as a spurious "(no response)".')
        return 1

    print(f'pulsing each thruster at {args.thrust:.1f} N for '
          f'{args.pulse:.1f}s from rest, body-frame velocity in m/s and rad/s\n')

    results = {}
    for i in THRUSTERS:
        if not rig.wait_until_still(timeout=args.coast):
            print(f'  (thruster {i}: vehicle never fully settled, reading may '
                  'be noisy)')
        before = rig.linear + rig.angular
        rig.hold(args.pulse, {i: args.thrust})
        after = rig.linear + rig.angular
        results[i] = tuple(a - b for a, b in zip(after, before))
        print(f'  thruster {i}: {describe(results[i], args.floor)}')
    rig.hold(0.5, 0.0)

    print()
    failed = False
    for i in THRUSTERS:
        problems = check(i, results[i], args.floor, args.cross_fraction)
        if problems:
            failed = True
            print(f'FAIL: thruster {i} does not match ArduSub vectored_6dof:')
            for problem in problems:
                print(f'    {problem}')

    if failed:
        print('\nThe model\'s thruster geometry and ArduSub\'s mixing table have\n'
              'diverged. Fix the link poses in model.sdf.in, not the table.\n')
        return 1

    heave_leak = max(abs(results[i][2]) for i in HORIZONTAL_THRUSTERS)
    surge_leak = max(abs(results[i][0]) for i in VERTICAL_THRUSTERS)
    print(f'horizontal-to-heave leak: {heave_leak:.3f} m/s')
    print(f'vertical-to-surge leak:   {surge_leak:.3f} m/s')
    print('\nall eight thrusters match ArduSub vectored_6dof\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())

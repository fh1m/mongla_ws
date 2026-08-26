#!/usr/bin/env python3

"""Measure the vehicle's surge step response and check it against the model.

This is the empirical check on the hydrodynamic damping coefficients, and
specifically on their signs. A sign error there does not look like a small
tuning problem: drag turns into thrust and the vehicle accelerates without
bound. That shows up here immediately.

Drives the four vectored horizontal thrusters directly, bypassing ArduSub, and
reads ground-truth speed from the odometry publisher. Run against the
`pool_empty` course with ArduSub disabled:

    ros2 launch duburi_sim_bringup sim.launch.py \\
        course:=pool_empty ardusub:=false bridge:=false gui:=false

    scripts/step_response.py

The measured terminal velocity is compared with the analytic solution of
    F = |xU|*u + |xUabsU|*u*|u|
using the coefficients in the vehicle's own configs.yaml, so the test has an
independent expectation rather than just plotting a curve.

The vehicle is ballasted 100 g buoyant, so left alone it rises and the
measurement picks up partial emergence instead of drag. The verticals are
trimmed to cancel that for the duration of the run.
"""

import argparse
import math
import os
import sys

import yaml

from thruster_rig import VERTICAL_THRUSTERS, ThrusterRig

HERE = os.path.dirname(os.path.abspath(__file__))

DEFAULT_CONFIG = os.path.normpath(
    os.path.join(
        HERE, '..', '..', 'duburi_sim_description', 'models', 'duburi_heavy',
        'configs.yaml',
    )
)

# Each horizontal is mounted at 45 degrees, so cos(45) of its thrust goes into
# surge and the rest into sway.
VECTOR_ANGLE = math.radians(45.0)
GRAVITY = 9.81

# The forward column of ArduSub's vectored_6dof mixing table. Equal positive
# thrust on all four is *not* a forward command on this frame: thrusters 1 and 2
# point aft, so it cancels to nothing. thruster_survey.py checks the full table.
FORWARD_MIX = {1: -1.0, 2: -1.0, 3: +1.0, 4: +1.0}


def analytic_terminal(linear: float, quadratic: float, force: float) -> float:
    """Solve force = |linear|*u + |quadratic|*u^2 for u."""
    a, b, c = abs(quadratic), abs(linear), -force
    if a == 0:
        return force / b
    return (-b + math.sqrt(b * b - 4 * a * c)) / (2 * a)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--vehicle', default='duburi',
                        help='Vehicle instance name from the course YAML.')
    parser.add_argument('--model', default='duburi_heavy',
                        help='Model name, which is what namespaces the thruster '
                             'topics.')
    parser.add_argument('--config', default=DEFAULT_CONFIG)
    parser.add_argument('--thrust', type=float, default=25.0,
                        help='Newtons per thruster. Four thrusters at 45 deg.')
    parser.add_argument('--settle', type=float, default=25.0,
                        help='Seconds to let the speed converge.')
    parser.add_argument('--tolerance', type=float, default=0.15,
                        help='Allowed fractional error against the analytic value.')
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    xU = cfg['drag']['linear']['xU']
    xUabsU = cfg['drag']['quadratic']['xUabsU']

    if xU > 0 or xUabsU > 0:
        print('FAIL: surge drag coefficients are positive in configs.yaml.\n'
              '  The Hydrodynamics plugin applies these without negating them,\n'
              '  so a positive value drives the vehicle instead of damping it.\n'
              '  See HYDRODYNAMICS.md.')
        return 1

    # Cancel the deliberate 100 g of positive buoyancy, split over four
    # verticals, so the run measures drag rather than the vehicle surfacing.
    trim = -cfg['buoyancy_adjustment'] * GRAVITY / len(VERTICAL_THRUSTERS)

    force = len(FORWARD_MIX) * args.thrust * math.cos(VECTOR_ANGLE)
    expected = analytic_terminal(xU, xUabsU, force)

    mix = ', '.join(f'{i}:{sign * args.thrust:+.0f}N'
                    for i, sign in sorted(FORWARD_MIX.items()))
    print(f'commanding {mix} -> {force:.1f} N surge')
    print(f'trimming {trim:+.2f} N on each vertical to hold depth')
    print(f'expecting terminal surge {expected:.3f} m/s from configs.yaml\n')

    rig = ThrusterRig(args.vehicle, args.model)
    if not rig.wait_for_odometry():
        print(f'FAIL: no odometry on /model/{args.vehicle}/odometry.\n'
              '  Is the simulator running, and is the vehicle really named '
              f'"{args.vehicle}"?')
        return 1

    unreachable = rig.wait_for_thrusters()
    if unreachable:
        print(f'FAIL: thrusters {unreachable} never acknowledged a command.\n'
              '  Their cmd_thrust subscribers were not discovered, so any\n'
              '  measurement now would silently be missing their thrust.')
        return 1

    step = {i: sign * args.thrust for i, sign in FORWARD_MIX.items()}
    step.update({i: trim for i in VERTICAL_THRUSTERS})
    coast = {i: trim for i in VERTICAL_THRUSTERS}

    # Settle at zero surge first, so the step starts from a known depth.
    rig.hold(5.0, coast)
    print(f'  t=  0.0s  u = {rig.surge:+.3f} m/s  z = {rig.depth:+.2f} m '
          '(before step)')

    elapsed = [0.0]
    samples = []

    def report():
        elapsed[0] += 1.0
        samples.append((elapsed[0], rig.surge, rig.depth))
        print(f'  t={elapsed[0]:5.1f}s  u = {rig.surge:+.3f} m/s  '
              f'z = {rig.depth:+.2f} m')

    rig.hold(args.settle, step, on_sample=report)
    rig.hold(0.5, 0.0)

    # The vertical trim is open loop, so the vehicle drifts slowly in depth and
    # the surge reading rides that drift by a few percent. Average the second
    # half of the run rather than trusting whichever sample happened to be last.
    settled = samples[len(samples) // 2:]
    if not settled:
        print('FAIL: the run was too short to collect a settled sample')
        return 1
    speed = sum(s[1] for s in settled) / len(settled)
    depth = sum(s[2] for s in settled) / len(settled)

    print(f'\naveraging the last {len(settled)} samples, from '
          f't={settled[0][0]:.0f}s\n')

    # A runaway is the signature of a sign error, so name it explicitly rather
    # than reporting it as a generic mismatch.
    if abs(speed) > 5 * max(expected, 0.1):
        print(f'FAIL: surge ran away to {speed:+.3f} m/s against an expected '
              f'{expected:.3f} m/s.\n'
              '  This is what a sign error in the drag coefficients looks like.')
        return 1

    if depth > -0.2:
        print(f'WARN: vehicle finished at z = {depth:+.2f} m, close enough to '
              'the surface that\n  the reading may include partial emergence '
              'rather than pure drag.\n')

    error = abs(speed - expected) / expected if expected else float('inf')
    print(f'measured  {speed:+.3f} m/s')
    print(f'expected  {expected:+.3f} m/s')
    print(f'error     {error * 100:.1f}%')

    if error <= args.tolerance:
        print('\nstep response matches the model\n')
        return 0
    print(f'\nstep response is outside the {args.tolerance * 100:.0f}% '
          'tolerance. Added mass sets how fast it converges, not where it\n'
          'settles, so check the drag terms and the thruster geometry first;\n'
          'thruster_survey.py measures the geometry directly.\n')
    return 1


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3

"""Terminal velocity and settling time on every translational axis.

`step_response.py` measures SURGE and only surge. It has one recorded result --
0.650 m/s measured against 0.661 predicted, 1.6 % -- and that single number is
the entire validation of the hydrodynamic model. Sway, heave and every
rotational axis have never been checked, and their coefficients differ per axis:
sway quadratic drag is -217 against surge's -141, and HYDRODYNAMICS.md states
the rotational added-mass terms carry 30-100 % estimation error.

This measures each axis the same way step_response does, and adds the quantity
that reveals ADDED MASS. Terminal velocity is set by drag alone -- added mass
does not appear in it at all, which step_response says itself: "added mass sets
how fast it converges, not where it settles." The convergence RATE is therefore
the only observable of added mass, and nothing was measuring it.

    ros2 launch duburi_sim_bringup sim.launch.py course:=pool_empty \
        ardusub:=false bridge:=false gui:=false
    python3 drag_survey.py --axes surge,sway,heave

Results go to RESULTS.md next to configs.yaml, so a coefficient change is
diffed against measurements instead of re-derived from scratch.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from datetime import datetime, timezone

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from thruster_rig import VERTICAL_THRUSTERS, ThrusterRig  # noqa: E402

GRAVITY = 9.80665
VECTOR_ANGLE = math.radians(45.0)

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(
    HERE, '..', '..', 'duburi_sim_description', 'models', 'duburi_heavy',
    'configs.yaml')

# Thruster mixes per axis, in the same convention step_response.py uses for
# surge. The vectored horizontals sit at 45 degrees, so each contributes
# cos(45) of its thrust to surge and sin(45) to sway -- which is why a mix that
# is symmetric in surge cancels in sway and vice versa.
MIXES = {
    'surge': ({1: -1.0, 2: -1.0, 3: +1.0, 4: +1.0}, math.cos(VECTOR_ANGLE),
              'xU', 'xUabsU', 'surge'),
    'sway':  ({1: -1.0, 2: +1.0, 3: -1.0, 4: +1.0}, math.sin(VECTOR_ANGLE),
              'yV', 'yVabsV', 'sway'),
    # Heave uses the four verticals directly -- no 45-degree factor.
    'heave': ({5: +1.0, 6: +1.0, 7: +1.0, 8: +1.0}, 1.0,
              'zW', 'zWabsW', 'heave'),
}


def analytic_terminal(linear: float, quadratic: float, force: float) -> float:
    """Solve  F = |lin|*u + |quad|*u^2  for u."""
    a, b = abs(quadratic), abs(linear)
    if a < 1e-9:
        return force / b if b else float('inf')
    return (-b + math.sqrt(b * b + 4 * a * force)) / (2 * a)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--vehicle', default='duburi')
    ap.add_argument('--model', default='duburi_heavy')
    ap.add_argument('--config', default=DEFAULT_CONFIG)
    ap.add_argument('--axes', default='surge,sway,heave')
    ap.add_argument('--thrust', type=float, default=25.0,
                    help='Newtons per thruster.')
    ap.add_argument('--settle', type=float, default=25.0)
    ap.add_argument('--tolerance', type=float, default=0.15)
    ap.add_argument('--write', default='',
                    help='Append the table to this file (default: stdout only).')
    a = ap.parse_args()

    with open(a.config) as f:
        cfg = yaml.safe_load(f)
    trim = -cfg['buoyancy_adjustment'] * GRAVITY / len(VERTICAL_THRUSTERS)

    rig = ThrusterRig(a.vehicle, a.model)
    if not rig.wait_for_odometry():
        print(f'FAIL: no odometry on /model/{a.vehicle}/odometry')
        return 1
    unreachable = rig.wait_for_thrusters()
    if unreachable:
        print(f'FAIL: thrusters {unreachable} never acknowledged a command')
        return 1

    rows, bad = [], 0
    for axis in [x.strip() for x in a.axes.split(',') if x.strip()]:
        if axis not in MIXES:
            print(f'unknown axis {axis!r}; known: {", ".join(MIXES)}')
            return 2
        mix, factor, lin_key, quad_key, prop = MIXES[axis]
        lin = cfg['drag']['linear'][lin_key]
        quad = cfg['drag']['quadratic'][quad_key]
        if lin > 0 or quad > 0:
            print(f'FAIL: {axis} drag coefficients are positive in configs.yaml;'
                  ' the plugin does not negate them, so they would DRIVE the'
                  ' vehicle. See HYDRODYNAMICS.md.')
            return 1

        force = len(mix) * a.thrust * factor
        expect = analytic_terminal(lin, quad, force)

        step = {i: s * a.thrust for i, s in mix.items()}
        if axis != 'heave':
            # Hold depth while measuring a horizontal axis, or the vehicle
            # surfaces on its 100 g of positive buoyancy and the run measures
            # emergence rather than drag.
            step.update({i: trim for i in VERTICAL_THRUSTERS})
        coast = {i: trim for i in VERTICAL_THRUSTERS}

        # RUNWAY CHECK. pool_empty is 25 m long and the vehicle starts at the
        # middle, so there is 12.5 m of runway. A 25 s run at the predicted
        # 0.66 m/s covers 16.5 m: the vehicle hits the far wall, stops dead,
        # and the tail average reads a number that is neither terminal velocity
        # nor an error -- it is half of one. step_response.py's own 25 s
        # default does exactly this, which is why its recorded 1.6 % result
        # cannot be reproduced today.
        runway = 12.0 if axis != 'heave' else 1.0
        need = expect * a.settle
        if need > runway:
            safe = runway / expect
            print(f'\n=== {axis}: SKIPPED -- {a.settle:.0f} s at {expect:.2f} m/s '
                  f'needs {need:.1f} m and the pool gives {runway:.1f} m. '
                  f'Use --settle {safe:.0f} or less.')
            bad += 1
            continue

        print(f'\n=== {axis}: {force:.1f} N, expecting {expect:.3f} m/s')
        rig.hold(5.0, coast)
        start_pos = rig.pose

        # on_sample takes no argument (see thruster_rig.hold), so the clock
        # comes from here rather than the callback.
        import time as _time
        t_start = _time.time()
        samples = []
        rig.hold(a.settle, step,
                 on_sample=lambda: samples.append(
                     (_time.time() - t_start, getattr(rig, prop))))
        rig.command(coast)

        if not samples:
            print('  no samples'); bad += 1; continue
        tail = [abs(v) for _, v in samples[-max(3, len(samples) // 4):]]
        measured = sum(tail) / len(tail)

        # Settling time to 63 % of terminal -- the first-order time constant,
        # and the only thing in this measurement that added mass shows up in.
        t0 = 0.0
        tau = next((t - t0 for t, v in samples if abs(v) >= 0.632 * measured),
                   float('nan'))

        err = abs(measured - expect) / expect if expect else float('inf')
        ok = err <= a.tolerance
        bad += 0 if ok else 1
        moved = math.dist(rig.pose, start_pos) if start_pos else float('nan')
        print(f'  measured {measured:.3f} m/s vs {expect:.3f} predicted '
              f'({err * 100:.1f} %)  tau {tau:.2f} s  travelled {moved:.1f} m  '
              f'{"OK" if ok else "OUT OF TOLERANCE"}')
        if moved > runway * 0.92:
            print('  WARNING: ran nearly the length of the pool -- the tail '
                  'average may include a wall impact. Lower --settle.')
        rows.append({'axis': axis, 'force_N': round(force, 1),
                     'predicted': round(expect, 3),
                     'measured': round(measured, 3),
                     'error_pct': round(err * 100, 1),
                     'tau_s': None if tau != tau else round(tau, 2),
                     'ok': ok})

    if a.write and rows:
        stamp = datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M UTC')
        with open(a.write, 'a') as f:
            f.write(f'\n## {stamp}, {a.thrust:.0f} N per thruster\n\n')
            f.write('| axis | force | predicted | measured | error | tau |\n')
            f.write('|---|---|---|---|---|---|\n')
            for r in rows:
                f.write(f"| {r['axis']} | {r['force_N']} N | "
                        f"{r['predicted']} m/s | {r['measured']} m/s | "
                        f"{r['error_pct']} % | {r['tau_s']} s |\n")
        print(f'\nappended to {a.write}')

    print('\n' + json.dumps(rows, indent=1))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())

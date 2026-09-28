#!/usr/bin/env python3
"""Rank the four scale sources against a KNOWN-CONSTANT-SPEED approach.

⭐ WHY THIS TOOL EXISTS BEFORE THE RUN. `tau = s / (ds/dt)` is the only
model-free candidate of six that survived section 58/60/61, and it survived for
a structural reason: the others answer the wrong question or answer it with no
information, while tau answers the right one and is merely NOISY (section 62 --
p50 2.5 s with a frame-to-frame p90 swing of 27.2 s).

Noise is closed by ground truth, and the ground truth is almost free. Approach
a fixed prop at a CONSTANT SPEED from a KNOWN START RANGE and true tau is a
straight line:

    tau(t) = (R0 - v t) / v        slope exactly -1 s per second

That turns every open question into a fit rather than a taste:

  * WHICH of the four scale sources is least biased -- they can be RANKED, not
    merely compared against each other, which is the difference between a truth
    test and an agreement test;
  * HOW MUCH filtering the variance needs;
  * WHETHER the bias is reference-dependent -- run it twice from different
    start ranges and check the two agree. That two-snap control is what caught
    the plane-tilt estimator reporting a smooth, confident, WRONG number.

⛔ DO NOT SKIP THE CONTROL. `anchor/geometry.py` already paid this bill: a
31-frame rolling median took the tilt's p90 swing from 39 deg to 1.3, and two
independent snaps of the same board still disagreed by 17.5 deg at p90.
Smoothing bought smoothness, NOT accuracy. A filtered tau will look excellent
on a plot for exactly the same reason.

    # after the run
    python3 tools/tau_from_scale.py --bag <dir> --r0 4.0 --speed 0.3

⚠ `--r0` and `--speed` are MEASURED, not commanded. A commanded 0.3 m/s that
was really 0.26 makes every source look 15 % biased in the same direction,
which is the one error this design cannot detect from inside.
"""
from __future__ import annotations

import argparse
import math
import sys


def true_tau(t, r0, v):
    """Seconds to contact at time t, from the geometry of the run."""
    if v <= 0:
        return float('nan')
    remaining = r0 - v * t
    return remaining / v if remaining > 0 else float('nan')


def tau_from_series(t, s):
    """tau = s / (ds/dt), central differences, NaN where ds/dt ~ 0.

    ⚠ NO SMOOTHING HERE, deliberately. The point of the run is to measure how
    much smoothing is needed; applying some first would answer the question
    with its own assumption.
    """
    import numpy as np
    t = np.asarray(t, float)
    s = np.asarray(s, float)
    if t.size < 3:
        return np.full(t.shape, np.nan)
    ds = np.gradient(s, t)
    with np.errstate(divide='ignore', invalid='ignore'):
        out = np.where(np.abs(ds) < 1e-9, np.nan, s / ds)
    return out


def score(t, tau, r0, v):
    """Bias, spread and slope against the straight line the run guarantees."""
    import numpy as np
    t = np.asarray(t, float)
    tau = np.asarray(tau, float)
    truth = np.array([true_tau(x, r0, v) for x in t])
    m = np.isfinite(tau) & np.isfinite(truth) & (tau > 0)
    if m.sum() < 10:
        return None
    err = tau[m] - truth[m]
    # The slope is the sharpest test: true tau falls at exactly -1 s/s, so a
    # source with the right slope and a constant offset has a CALIBRATION
    # error, while a wrong slope means the scale itself is wrong.
    slope = float(np.polyfit(t[m], tau[m], 1)[0])
    return dict(n=int(m.sum()),
                bias=float(np.median(err)),
                spread=float(np.percentile(np.abs(err - np.median(err)), 90)),
                slope=slope,
                jitter=float(np.percentile(np.abs(np.diff(tau[m])), 90)))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', default='', help='recorded run (not yet parsed)')
    ap.add_argument('--r0', type=float, required=True,
                    help='MEASURED start range, metres')
    ap.add_argument('--speed', type=float, required=True,
                    help='MEASURED surge speed, m/s')
    ap.add_argument('--self-test', action='store_true',
                    help='run the scorer against synthetic sources')
    a = ap.parse_args()

    if not a.self_test and not a.bag:
        print('give --bag <dir> from the constant-speed run, or --self-test.\n'
              'The run: park at a measured range from a fixed prop, command a '
              'constant surge with no depth or yaw change, record with the '
              'detector, the follower and the flow all running. A few minutes '
              'of pool time; see .claude/context/perception/tau-plan.md')
        return 2

    if a.self_test:
        # ⭐ THE SCORER IS TESTED BEFORE THE POOL, against sources whose bias
        # we CHOSE -- so a bug in the scorer cannot be mistaken for a bias in a
        # sensor on the day, when re-running costs a pool session.
        import numpy as np
        t = np.linspace(0.0, 0.9 * a.r0 / a.speed, 300)
        truth = np.array([true_tau(x, a.r0, a.speed) for x in t])
        s_true = 1.0 / np.maximum(a.r0 - a.speed * t, 1e-3)   # size ~ 1/range
        rng = np.random.default_rng(0)
        cases = {
            'perfect':        s_true,
            'noisy 2 %':      s_true * (1 + 0.02 * rng.normal(size=t.size)),
            'biased +20 %':   s_true * 1.2,
            'wrong exponent': s_true ** 1.15,
        }
        print(f'\nself-test  r0={a.r0} m  v={a.speed} m/s  '
              f'true tau {truth[0]:.1f} -> {truth[-1]:.1f} s')
        print(f'{"source":<18}{"n":>5}{"bias s":>9}{"spread s":>10}'
              f'{"slope":>8}{"jitter s":>10}')
        for name, s in cases.items():
            r = score(t, tau_from_series(t, s), a.r0, a.speed)
            if r is None:
                print(f'{name:<18}   too few usable samples')
                continue
            print(f'{name:<18}{r["n"]:>5}{r["bias"]:>+9.2f}{r["spread"]:>10.2f}'
                  f'{r["slope"]:>+8.2f}{r["jitter"]:>10.2f}')
        print('\n⭐ A CONSTANT SCALE BIAS SCORES IDENTICAL TO PERFECT, and '
              'that is not a bug in the scorer -- it is the whole reason tau '
              'is the right quantity. tau is SCALE-INVARIANT, so a source that '
              'is 20 % off everywhere cancels exactly. No calibration is '
              'needed and none can help.')
        print('⭐⭐ SO ONLY TWO THINGS CAN GO WRONG, and each has its own '
              'column:')
        print('   NOISE  -- 2 % scale noise gives 23.3 s of tau jitter here, '
              'against the 27.2 s p90 measured on real footage (section 62). '
              'Our problem is therefore roughly 2 % scale noise, and the fix '
              'is averaging: the bar is how many frames.')
        print('   SLOPE  -- true tau falls at exactly -1.00 s/s. A wrong '
              'exponent (size not going as 1/range) reads -0.87 and NO amount '
              'of filtering fixes it, because the scale itself is wrong.')
        return 0

    print(f'bag parsing is not written yet -- the run has not happened.\n'
          f'The scorer is ready and self-tested: '
          f'python3 {sys.argv[0]} --self-test --r0 {a.r0} --speed {a.speed}')
    return 2


if __name__ == '__main__':
    raise SystemExit(main())

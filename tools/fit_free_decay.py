#!/usr/bin/env python3
"""Fit a free-decay rotation: the drag-to-inertia ratio the controller needs.

⛔ WHAT IT IDENTIFIES, AND WHAT IT DOES NOT. Spin the hull about an axis, cut
the command, and the rate decays under its own drag. For pure QUADRATIC damping
with no restoring term the decay has a closed form and it is a STRAIGHT LINE in
1/omega:

    I dw/dt = -q w|w|      =>      1/w(t) = 1/w0 + (q/I) t

so a least-squares line through 1/omega gives **q/I** directly.

⚠ THAT RATIO IS THE ANSWER, NOT q. Turning it into a drag coefficient needs I,
and ours is a uniform-body guess -- a real hull is a shell with its battery
placed somewhere deliberate. The good news is that q/I is exactly what the
closed-loop time constant depends on, so the controller needs the ratio and not
either term. Say "q/I" wherever this is quoted, and do not silently multiply by
a guessed inertia.

⭐ THE PROCEDURE IS REHEARSED BEFORE THE POOL. `tools/control_bench/test_plant.py
::test_free_decay_recovers_the_drag_we_put_in` runs this fit against a plant
whose drag we CHOSE and checks it comes back to 1 %. An identification that
cannot recover a known answer on a noiseless plant will return a plausible one
from a pool and nobody will know.

Usage:
    python3 tools/fit_free_decay.py decay.csv          # time_s, rate_rad_s
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass


@dataclass(frozen=True)
class DecayFit:
    q_over_i: float            # 1/rad, the slope -- THE RESULT
    omega_0: float
    n_points: int
    r_squared: float
    span_s: float

    def time_constant_at(self, omega: float) -> float:
        """Seconds for the rate to halve, starting from `omega`.

        Quadratic drag has no single time constant -- it decays faster when it
        is spinning faster -- so the honest readout is a half-life AT a stated
        rate rather than one number.
        """
        return 1.0 / (self.q_over_i * omega) if omega > 0 else float('inf')

    def __str__(self) -> str:
        return (f'q/I        {self.q_over_i:8.4f}  1/rad\n'
                f'omega_0    {self.omega_0:8.4f}  rad/s '
                f'({math.degrees(self.omega_0):.1f} deg/s)\n'
                f'half-life  {self.time_constant_at(self.omega_0):8.3f}  s '
                f'at omega_0\n'
                f'R^2        {self.r_squared:8.5f}  over {self.n_points} points, '
                f'{self.span_s:.2f} s')


def fit_decay(t, omega, *, min_rate_frac: float = 0.10) -> DecayFit:
    """Least squares on 1/omega against t. `omega` may be either sign.

    `min_rate_frac` drops the tail below that fraction of the starting rate:
    1/omega explodes as omega approaches zero, so the last few percent would
    dominate a straight-line fit while carrying the worst signal-to-noise.
    """
    if len(t) != len(omega) or len(t) < 20:
        raise ValueError('need at least 20 paired samples')
    sign = 1.0 if sum(omega) >= 0 else -1.0
    w = [sign * v for v in omega]
    w0 = max(w)
    if w0 <= 0.0:
        raise ValueError('no rotation to decay')

    floor = min_rate_frac * w0
    xs = [tt for tt, v in zip(t, w) if v > floor]
    ys = [1.0 / v for v in w if v > floor]
    if len(xs) < 10:
        raise ValueError(
            'fewer than 10 samples above the rate floor -- either the spin was '
            'too slow to start with, or the record was cut short. Let it decay '
            'to about a tenth of the starting rate.')

    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0.0:
        raise ValueError('all samples at one instant')
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    if slope <= 0.0:
        raise ValueError(
            'the rate is not decaying -- something is still driving the hull. '
            'Check the command was actually cut and the mode did not re-engage.')

    ss_tot = sum((y - my) ** 2 for y in ys)
    ss_res = sum((y - (my + slope * (x - mx))) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0

    return DecayFit(q_over_i=slope, omega_0=w0, n_points=n,
                    r_squared=r2, span_s=xs[-1] - xs[0])


def looks_linear_instead(t, omega) -> bool:
    """⚠ IS THE DAMPING ACTUALLY QUADRATIC? A LINEAR-damped decay is exponential,
    so `ln(omega)` is the straight line and `1/omega` is not.

    Returns True when the log fit is clearly better, which means the quadratic
    model is wrong for this hull and the fit above should not be quoted. This
    matters: at low rates, and with four open tunnels, linear damping can
    dominate -- so the check is not academic.
    """
    def r2(transform):
        xs, ys = [], []
        w0 = max(abs(v) for v in omega)
        for tt, v in zip(t, omega):
            a = abs(v)
            if a > 0.10 * w0:
                xs.append(tt)
                ys.append(transform(a))
        if len(xs) < 10:
            return 0.0
        n = len(xs)
        mx, my = sum(xs) / n, sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        sl = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
        tot = sum((y - my) ** 2 for y in ys)
        res = sum((y - (my + sl * (x - mx))) ** 2 for x, y in zip(xs, ys))
        return 1.0 - res / tot if tot > 0 else 0.0

    return r2(math.log) > r2(lambda v: 1.0 / v) + 0.02


def main(argv) -> int:
    if len(argv) != 2:
        print(__doc__)
        return 2
    t, w = [], []
    with open(argv[1]) as fh:
        for line in fh:
            line = line.strip()
            if not line or line[0] not in '-0123456789':
                continue
            parts = line.replace(',', ' ').split()
            t.append(float(parts[0]))
            w.append(float(parts[1]))
    print(fit_decay(t, w))
    if looks_linear_instead(t, w):
        print('\n⚠ A LOG FIT IS BETTER THAN 1/omega HERE. The damping looks '
              'LINEAR, not quadratic, so the q/I above is the wrong model for '
              'this trace. Re-spin faster, or fit a linear coefficient.')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))

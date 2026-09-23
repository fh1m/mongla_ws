#!/usr/bin/env python3
"""Fit a thruster's step response: dead time, time constant, steady thrust.

⛔ WHY THIS SHIPS WITH THE ASK. `pr-o-thruster-step-response.md` asks the
hardware team for a step-response test. An ask that says "measure the response
time" and leaves the analysis unstated gets back a number nobody can reproduce
and nobody can argue with. This is the analysis, written down, so the deliverable
is a CSV and the interpretation is not up for negotiation.

    thrust(t) = 0                                      t <  t_dead
    thrust(t) = T_ss * (1 - exp(-(t - t_dead) / tau))  t >= t_dead

⭐ VALIDATED AGAINST SYNTHETIC DATA WITH A KNOWN ANSWER, the same discipline the
free-decay fit gets: if it cannot recover a tau we chose, from a trace we
generated, it will not recover one from a load cell.

Usage:
    python3 tools/fit_thruster_response.py trace.csv
    # CSV: two or three columns -- time_s, thrust_n [, rpm]
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass


@dataclass(frozen=True)
class StepFit:
    t_dead_s: float
    tau_s: float
    thrust_ss_n: float
    rise_10_90_s: float
    n_samples: int
    rms_residual_n: float

    def total_lag_s(self) -> float:
        """⭐ THE NUMBER THE CONTROL LOOP ACTUALLY FEELS.

        Dead time and time constant are not interchangeable: a first-order lag
        can be compensated, pure dead time cannot. But for ranking one design
        against another, their sum is what the closed loop sees, and it is the
        single figure to compare between prototypes.
        """
        return self.t_dead_s + self.tau_s

    def __str__(self) -> str:
        return (f'dead time {self.t_dead_s*1000:6.1f} ms\n'
                f'tau       {self.tau_s*1000:6.1f} ms\n'
                f'total lag {self.total_lag_s()*1000:6.1f} ms\n'
                f'10-90%    {self.rise_10_90_s*1000:6.1f} ms\n'
                f'steady    {self.thrust_ss_n:6.2f} N\n'
                f'fit RMS   {self.rms_residual_n:6.3f} N over {self.n_samples} samples')


def fit_step(t, y, *, settle_frac: float = 0.8) -> StepFit:
    """Fit the model above to one step. `t` seconds, `y` thrust (either sign).

    ⚠ SIGN-AGNOSTIC ON PURPOSE. A reverse step is the same experiment with a
    negative plateau, and PR M section 2.2 needs both directions from the same
    rig. Working on |y| means one analysis covers both.
    """
    if len(t) != len(y) or len(t) < 20:
        raise ValueError('need at least 20 paired samples')
    sign = 1.0 if sum(y) >= 0 else -1.0
    mag = [sign * v for v in y]

    # Steady state from the settled tail, not from the last sample -- one noisy
    # final reading would otherwise set the asymptote the whole fit hangs on.
    tail_start = t[0] + settle_frac * (t[-1] - t[0])
    tail = [v for tt, v in zip(t, mag) if tt >= tail_start]
    if not tail:
        raise ValueError('no settled tail -- record for longer than ~5 tau')
    ss = sum(tail) / len(tail)
    if ss <= 0.0:
        raise ValueError('the plateau is not distinguishable from zero')

    # ⛔ DEAD TIME COMES OUT OF THE REGRESSION, NOT OUT OF A THRESHOLD.
    # The first version found t_dead as "first sample above a noise floor", and
    # estimated that floor from the opening 5 % of the record -- which, for a
    # step at 10 ms in a 1 s trace, is mostly POST-step data. The floor came out
    # huge and dead time was over-estimated by 4x while tau fitted perfectly.
    #
    # Linearising gives both for free and needs no threshold at all:
    #
    #     ln(1 - y/ss) = -(t - t_dead)/tau = -t/tau + t_dead/tau
    #
    # so the slope is -1/tau and the intercept is t_dead/tau. Points are chosen
    # by AMPLITUDE (5..90 % of the plateau), which needs no prior knowledge of
    # when the step happened.
    xs, ys = [], []
    for tt, v in zip(t, mag):
        frac = v / ss
        if 0.05 < frac < 0.90:
            xs.append(tt)
            ys.append(math.log(1.0 - frac))
    if len(xs) < 5:
        raise ValueError(
            'too few points between 5 % and 90 % of the plateau -- sample '
            'faster, or the step is not first-order')
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    denom = sum((x - mx) ** 2 for x in xs)
    if denom <= 0.0:
        raise ValueError('all samples at one instant')
    slope = sum((x - mx) * (y_ - my) for x, y_ in zip(xs, ys)) / denom
    if slope >= 0.0:
        raise ValueError('the trace does not rise toward a plateau')
    tau = -1.0 / slope
    t_dead = (my - slope * mx) * tau

    # ⛔ REFUSE A RECORD THAT NEVER REACHED THE PLATEAU. Stopping at 2 tau makes
    # "steady thrust" an extrapolation presented as a measurement, and every
    # derived number -- the thrust curve, REVERSE_EFFICIENCY, tau itself --
    # would inherit it silently.
    if (t[-1] - t_dead) < 4.0 * tau:
        raise ValueError(
            f'record is only {(t[-1]-t_dead)/tau:.1f} tau long after the step; '
            f'need at least 4 tau so the plateau is measured rather than '
            f'extrapolated. Record for {5*tau:.2f} s or more.')

    resid = [v - ss * (1.0 - math.exp(-(tt - t_dead) / tau))
             for tt, v in zip(t, mag) if tt >= t_dead]
    rms = math.sqrt(sum(r * r for r in resid) / len(resid)) if resid else 0.0

    return StepFit(t_dead_s=max(0.0, t_dead - t[0]), tau_s=tau, thrust_ss_n=sign * ss,
                   rise_10_90_s=tau * math.log(9.0),
                   n_samples=len(t), rms_residual_n=rms)


def _stdev(xs) -> float:
    if len(xs) < 2:
        return 0.0
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def break_away_cost(from_stopped: StepFit, from_running: StepFit) -> float:
    """⭐ THE NUMBER PR M SECTION 2.1 IS REALLY ASKING FOR.

    The extra lag a STOPPED propeller costs over one already turning. That
    difference is the break-away -- cogging, seal stiction, sensorless ESC
    startup -- and it is the whole justification for `MOT_SPIN_MIN`. If it is
    near zero the floor can go to zero, and the 16.2 % output quantum with it.
    """
    return from_stopped.total_lag_s() - from_running.total_lag_s()


def main(argv) -> int:
    if len(argv) != 2:
        print(__doc__)
        return 2
    t, y = [], []
    with open(argv[1]) as fh:
        for line in fh:
            line = line.strip()
            if not line or line[0] not in '-0123456789':
                continue
            parts = line.replace(',', ' ').split()
            t.append(float(parts[0]))
            y.append(float(parts[1]))
    print(fit_step(t, y))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))

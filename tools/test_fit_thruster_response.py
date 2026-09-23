"""The step fit, checked against traces whose answer we CHOSE.

⛔ SAME DISCIPLINE AS THE FREE-DECAY FIT. We are asking the hardware team to
spend an afternoon producing data. If the analysis cannot recover a known
answer from a synthetic trace, it will return a plausible number from a real one
and nobody will know.
"""
import math
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fit_thruster_response import (break_away_cost, fit_step)  # noqa: E402


def synth(*, tau, t_dead, ss, hz=1000.0, seconds=1.0, noise=0.0, seed=1):
    rnd = random.Random(seed)
    t, y = [], []
    for i in range(int(hz * seconds)):
        tt = i / hz
        v = 0.0 if tt < t_dead else ss * (1.0 - math.exp(-(tt - t_dead) / tau))
        t.append(tt)
        y.append(v + (rnd.gauss(0.0, noise) if noise else 0.0))
    return t, y


@pytest.mark.parametrize('tau', [0.020, 0.079, 0.200, 0.590])
def test_it_recovers_a_time_constant_we_chose(tau):
    f = fit_step(*synth(tau=tau, t_dead=0.010, ss=20.0, seconds=max(1.0, 8 * tau)))
    assert f.tau_s == pytest.approx(tau, rel=0.05)


def test_it_recovers_dead_time_separately_from_tau():
    """⚠ THE TWO MUST NOT BE CONFLATED. A first-order lag can be compensated by
    a controller; pure dead time cannot. A fit that folds one into the other
    would hide the difference that decides what is fixable."""
    f = fit_step(*synth(tau=0.060, t_dead=0.035, ss=15.0, seconds=1.0))
    assert f.t_dead_s == pytest.approx(0.035, abs=0.005)
    assert f.tau_s == pytest.approx(0.060, rel=0.06)


def test_it_survives_realistic_load_cell_noise():
    """A 1 % noise floor on a 20 N plateau -- better than most load cells."""
    f = fit_step(*synth(tau=0.080, t_dead=0.015, ss=20.0, noise=0.2, seed=7))
    assert f.tau_s == pytest.approx(0.080, rel=0.15)


def test_a_reverse_step_fits_the_same_way():
    """PR M section 2.2 needs both directions off one rig, so the analysis is
    sign-agnostic and the plateau simply comes back negative."""
    t, y = synth(tau=0.070, t_dead=0.012, ss=20.0)
    f = fit_step(t, [-v for v in y])
    assert f.thrust_ss_n < 0
    assert f.tau_s == pytest.approx(0.070, rel=0.06)
    assert abs(f.thrust_ss_n) == pytest.approx(20.0, rel=0.02)


def test_break_away_is_the_difference_between_two_steps():
    """⭐ What section 2.1 is really asking. A stopped propeller costs extra
    lag; that extra IS the break-away, and it is the justification for
    MOT_SPIN_MIN existing at all."""
    running = fit_step(*synth(tau=0.070, t_dead=0.005, ss=20.0))
    stopped = fit_step(*synth(tau=0.070, t_dead=0.055, ss=20.0))
    assert break_away_cost(stopped, running) == pytest.approx(0.050, abs=0.006)


def test_too_short_a_record_is_refused():
    """⛔ REFUSE, DO NOT EXTRAPOLATE. Stopping at 2 tau leaves the plateau
    unreached, so 'steady thrust' would be an extrapolation presented as a
    measurement -- and every derived number would inherit it."""
    with pytest.raises(ValueError):
        fit_step(*synth(tau=0.500, t_dead=0.0, ss=20.0, seconds=0.06))


def test_a_flat_trace_is_refused_rather_than_fitted():
    with pytest.raises(ValueError):
        fit_step([i / 1000 for i in range(500)], [0.0] * 500)


def test_the_fit_reports_its_own_residual():
    """A fit with no residual is an assertion. A real trace that fits badly --
    because the response is not first-order -- must be visible as a large RMS
    rather than as a confident tau."""
    good = fit_step(*synth(tau=0.080, t_dead=0.010, ss=20.0))
    assert good.rms_residual_n < 0.05

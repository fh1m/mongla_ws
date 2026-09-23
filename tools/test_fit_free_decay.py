"""The free-decay fit, against decays whose answer we CHOSE."""
import math
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fit_free_decay import DecayFit, fit_decay, looks_linear_instead  # noqa: E402


def quad_decay(*, q_over_i, w0, hz=100.0, seconds=30.0, noise=0.0, seed=3):
    """Exact solution of I dw/dt = -q w|w|, so the test has a true answer."""
    rnd = random.Random(seed)
    t, w = [], []
    for i in range(int(hz * seconds)):
        tt = i / hz
        v = 1.0 / (1.0 / w0 + q_over_i * tt)
        t.append(tt)
        w.append(v + (rnd.gauss(0, noise) if noise else 0.0))
    return t, w


@pytest.mark.parametrize('q_over_i', [0.2, 0.88, 2.5])
def test_it_recovers_a_ratio_we_chose(q_over_i):
    f = fit_decay(*quad_decay(q_over_i=q_over_i, w0=3.0))
    assert f.q_over_i == pytest.approx(q_over_i, rel=0.02)
    assert f.r_squared > 0.999


def test_it_survives_gyro_noise():
    """The board's gyro is good but not perfect; 0.01 rad/s of noise on a
    3 rad/s spin is pessimistic for a BNO085."""
    f = fit_decay(*quad_decay(q_over_i=0.88, w0=3.0, noise=0.01, seed=11))
    assert f.q_over_i == pytest.approx(0.88, rel=0.10)


def test_either_spin_direction_fits():
    t, w = quad_decay(q_over_i=0.88, w0=3.0)
    assert fit_decay(t, [-v for v in w]).q_over_i == pytest.approx(0.88, rel=0.02)


def test_a_hull_still_under_power_is_refused():
    """⛔ THE POOL-DAY MISTAKE THIS CATCHES. If the command was not really cut
    -- AUTO re-engaging, a heading hold still running -- the rate does not
    decay, and a fit would return a small positive number that looks like very
    low drag. Refuse instead."""
    t = [i / 100 for i in range(500)]
    with pytest.raises(ValueError):
        fit_decay(t, [2.0] * 500)              # constant rate: still driven
    with pytest.raises(ValueError):
        fit_decay(t, [2.0 + 0.01 * x for x in t])   # accelerating


def test_a_record_cut_short_is_refused():
    """Stopping while the hull is still spinning fast leaves the fit
    extrapolating, and q/I would inherit that silently."""
    with pytest.raises(ValueError):
        fit_decay(*quad_decay(q_over_i=0.2, w0=3.0, seconds=0.15))


def test_linear_damping_is_detected_rather_than_mis_fitted():
    """⚠ THE MODEL CHECK. A linear-damped decay is exponential, so ln(omega) is
    the straight line and 1/omega is not. Quoting q/I off an exponential decay
    would be a confident wrong answer -- and with four open tunnels at low
    rates, linear damping is a real possibility, not an academic one."""
    t = [i / 100 for i in range(3000)]
    w = [3.0 * math.exp(-0.25 * tt) for tt in t]
    assert looks_linear_instead(t, w) is True
    assert looks_linear_instead(*quad_decay(q_over_i=0.88, w0=3.0)) is False


def test_the_half_life_is_rate_dependent_and_says_so():
    """Quadratic drag has no single time constant -- it decays faster when it
    spins faster -- so the readout is a half-life AT a stated rate."""
    f = fit_decay(*quad_decay(q_over_i=0.88, w0=3.0))
    assert f.time_constant_at(3.0) < f.time_constant_at(1.0)


def test_the_fit_reports_its_own_r_squared():
    """A fit with no goodness measure is an assertion. A trace that does not
    fit must show as a poor R^2, not as a confident ratio."""
    clean = fit_decay(*quad_decay(q_over_i=0.88, w0=3.0))
    noisy = fit_decay(*quad_decay(q_over_i=0.88, w0=3.0, noise=0.05, seed=5))
    assert clean.r_squared > noisy.r_squared

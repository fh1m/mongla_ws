"""Velocity from commanded demand: learned against flow, used when flow dies.

Every test runs a simulated first-order hull (gain, current bias, lag) and
scores the model against the TRUE velocity, not against flow -- agreement with
the teacher is not truth.
"""
import math

import numpy as np
import pytest

from duburi_localization.command_velocity import (
    MIN_SAMPLES, UNREADY_VAR, CommandVelocityModel,
)

DT = 0.05
TAU = 1.0


def _demand(t):
    """Excites both axes at different rates, crosses zero, holds plateaus."""
    return (0.6 * math.copysign(1, math.sin(0.21 * t)) * (0.4 + 0.6 * abs(math.sin(0.05 * t))),
            0.5 * math.sin(0.13 * t + 1.0))


def _fly(model, seconds, *, g=(0.55, 0.40), b=(0.03, -0.05), flow_sd=0.02,
         teach=True, rng=None, start=0.0, tau=TAU):
    rng = rng or np.random.default_rng(0)
    v = [0.0, 0.0]
    errs = []
    t = start
    for _ in range(int(seconds / DT)):
        u = _demand(t)
        a = min(1.0, DT / tau)
        for i in range(2):
            v[i] += a * (g[i] * u[i] + b[i] - v[i])
        model.step(u[0], u[1], DT)
        if teach:
            model.learn(v[0] + rng.normal(0, flow_sd), v[1] + rng.normal(0, flow_sd))
        else:
            p = model.predict()
            if p is not None:
                errs.append((p[0] - v[0], p[1] - v[1]))
        t += DT
    return errs, t


def test_it_predicts_NOTHING_before_it_has_learned():
    m = CommandVelocityModel(tau_s=TAU)
    m.step(0.5, 0.5, DT)
    assert m.predict() is None


def test_learned_on_flow_it_tracks_TRUTH_through_a_flow_outage():
    m = CommandVelocityModel(tau_s=TAU)
    _, t = _fly(m, 120.0)
    assert m.x.ready and m.y.ready
    assert m.x.theta[0] == pytest.approx(0.55, abs=0.05)
    assert m.x.theta[1] == pytest.approx(0.03, abs=0.03)      # the current
    errs, _ = _fly(m, 60.0, teach=False, start=t)
    e = np.array(errs)
    rms = np.sqrt((e ** 2).mean(axis=0))
    assert len(errs) > 1000
    assert rms[0] < 0.03 and rms[1] < 0.03, rms


def test_a_flow_that_is_pure_NOISE_never_becomes_ready():
    """Negative control: a teacher uncorrelated with the demand must not
    produce a confident model."""
    m = CommandVelocityModel(tau_s=TAU)
    rng = np.random.default_rng(1)
    t = 0.0
    for _ in range(4000):
        m.step(*_demand(t), DT)
        m.learn(rng.normal(0, 0.3), rng.normal(0, 0.3))
        t += DT
    assert not m.x.ready and not m.y.ready
    assert m.predict() is None


def test_zero_demand_teaches_nothing():
    m = CommandVelocityModel(tau_s=TAU)
    for _ in range(1000):
        m.step(0.0, 0.0, DT)
        m.learn(0.2, 0.2)
    assert m.x.n == 0 and m.y.n == 0


def test_UNKNOWN_demand_silences_prediction_for_one_full_tau():
    """A board primitive ran: the host does not know what the hull did."""
    m = CommandVelocityModel(tau_s=TAU)
    _, t = _fly(m, 120.0)
    assert m.predict() is not None
    m.step(None, None, DT)
    assert m.predict() is None
    m.step(0.3, 0.0, 0.5 * TAU)
    assert m.predict() is None                      # still settling
    m.step(0.3, 0.0, 0.6 * TAU)
    assert m.predict() is not None


def test_an_unready_axis_is_handed_on_as_ignorable():
    m = CommandVelocityModel(tau_s=TAU)
    t = 0.0
    v = 0.0
    for _ in range(4000):                           # only forward is excited
        u = _demand(t)[0]
        v += (DT / TAU) * (0.5 * u - v)
        m.step(u, 0.0, DT)
        m.learn(v, 0.0)
        t += DT
    p = m.predict()
    assert p is not None and m.x.ready and not m.y.ready
    assert p[3] == UNREADY_VAR


def test_it_needs_MIN_SAMPLES_excited_samples():
    m = CommandVelocityModel(tau_s=TAU)
    m.step(0.5, 0.5, 2 * TAU)
    for _ in range(MIN_SAMPLES - 1):
        m.step(0.5, 0.5, DT)
        m.learn(0.25, 0.25)
    assert not m.x.ready
    m.step(0.5, 0.5, DT)
    m.learn(0.25, 0.25)
    assert m.x.ready


def test_bad_tau_is_refused():
    with pytest.raises(ValueError):
        CommandVelocityModel(tau_s=0.0)

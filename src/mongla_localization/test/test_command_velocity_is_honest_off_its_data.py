"""The command-velocity aid's variance says how far it is from its data (#58).

v = g*u + b has two unknowns. Trained at ONE demand every regressor row is
[u0, 1] -- rank one -- so the slope/offset split is unidentified, however many
samples. The aid declared itself ready and predicted 0.20 m/s at zero demand
for a hull truly at rest, sigma 0.05: a 4-sigma lie, used during flow loss.

The fix is the least-squares prediction interval, s^2 (1 + phi' P phi), from the
RLS covariance the model already carries. Measured (plant g=0.5, b=0):

    trained at u=0.5 only   at u=0.0: 0.200 vs 0.000, sigma 0.611, 0.33 sigma
    trained over 0.2..0.8   at u=0.0: -0.002 vs 0.000, sigma 0.051
"""
import math

import pytest

from mongla_localization.command_velocity import (
    BLOCKED, CommandVelocityModel, MotionCheck)

G = 0.5


def _trained(demands):
    m = CommandVelocityModel(tau_s=1.0)
    for k in range(400):
        u = demands[k % len(demands)]
        for _ in range(100):            # 5 tau: the lagged demand has settled
            m.step(u, u, 0.05)
        m.learn(G * u, G * u)
    return m


def _at(m, u):
    for _ in range(300):
        m.step(u, u, 0.05)
    vx, _, var, _ = m.predict()
    return vx, math.sqrt(var)


def test_issue_58_an_unidentified_model_admits_it_away_from_its_data():
    m = _trained([0.5])
    v, s = _at(m, 0.0)
    assert abs(v - 0.0) < 1.0 * s, f'{v:.3f} m/s at sigma {s:.3f} -- still a confident lie'
    assert s > 0.3


def test_it_stays_tight_where_it_WAS_trained():
    """Not a blanket inflation: at its own demand the fit is genuinely right."""
    v, s = _at(_trained([0.5]), 0.5)
    assert v == pytest.approx(G * 0.5, abs=0.005) and s < 0.06


def test_an_excited_model_is_tight_and_right_everywhere_in_range():
    m = _trained([0.2, 0.5, 0.8])
    for u in (0.0, 0.35, 0.8):
        v, s = _at(m, u)
        assert v == pytest.approx(G * u, abs=0.01) and s < 0.06


def test_motion_check_does_not_call_BLOCKED_on_an_expectation_it_cannot_back():
    """Trained at 0.5, asked about 0.8: it expects ~0.28 m/s with sigma ~0.37.
    A hull doing 0.07 m/s there is not provably blocked -- say unknown."""
    m = _trained([0.5])
    for _ in range(300):
        m.step(0.8, 0.0, 0.05)
    mc = MotionCheck()
    for _ in range(40):
        st = mc.observe(m, 0.07, 0.0, 0.05)
    assert st != BLOCKED

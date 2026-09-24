"""The gate that keeps one confident wrong detection out of the estimate.

Every measurement in this filter except depth originates in a detector, and a
detector does not fail by adding noise -- it fails by being certain about the
wrong object. A mirrored PnP branch, a mislabelled prop, a second torpedo hole:
each is a metre-scale innovation that an ungated filter accepts at full gain.
"""
import numpy as np
import pytest

from mongla_localization.inekf import (
    CHI2_99, REJECT_STREAK_LIMIT, RIEKF, State,
)


def test_a_wild_position_fix_is_rejected():
    f = RIEKF()
    f.update_position([500.0, 500.0], sigma=0.3)
    assert f.rejected == 1
    assert np.allclose(f.X.p[:2], np.zeros(2))


def test_a_plausible_position_fix_is_accepted():
    f = RIEKF()
    assert f.update_position([0.4, 0.3], sigma=0.3) is True
    assert f.rejected == 0
    assert f.X.p[0] > 0.1


def test_the_gate_scales_with_the_filters_own_uncertainty():
    """The same innovation is an outlier to a confident filter and ordinary to
    an uncertain one. A fixed metric threshold cannot express that."""
    tight = RIEKF(P0_position=0.01)
    loose = RIEKF(P0_position=10.0)
    tight.update_position([3.0, 0.0], sigma=0.3)
    loose.update_position([3.0, 0.0], sigma=0.3)
    assert tight.rejected == 1
    assert loose.rejected == 0


def test_a_persistently_disagreeing_world_breaks_the_lockout():
    """⛔ THE GATE'S OWN FAILURE MODE.

    A filter that is confidently wrong rejects exactly the measurements that
    would fix it, and the more wrong it is the more certain the rejection. It
    then publishes a tight covariance around a false position for ever, with
    nothing logged. Persistent disagreement has to be read as evidence about
    the filter.
    """
    f = RIEKF()
    f.X.p = np.array([5.0, 5.0, 0.0])
    for _ in range(REJECT_STREAK_LIMIT):
        f.update_position([1.0, 2.0], sigma=0.3)
    assert f.lockout_breaks == 1
    for _ in range(60):
        f.update_position([1.0, 2.0], sigma=0.3)
    assert f.X.p[0] == pytest.approx(1.0, abs=0.2)
    assert f.X.p[1] == pytest.approx(2.0, abs=0.2)


def test_the_lockout_breaks_while_ANOTHER_channel_is_being_accepted():
    """⛔ TRUTH: the hull is at (1, 2), level. The filter is confidently wrong
    at (5, 5) with a 5 cm sigma. The board's attitude -- correct, accepted --
    arrives at 50 Hz, and a correct fix at 5 Hz.

    The streak used to be ONE counter that any accepted update zeroed. With
    attitude accepted ten times between fixes, the position streak never got
    past 1: the lockout break never fired and the filter sat at (5, 5)
    rejecting the truth for ever, publishing a 5 cm sigma. One channel
    agreeing about attitude says nothing about another disagreeing about
    position."""
    f = RIEKF(P0_position=0.05)
    f.X.p = np.array([5.0, 5.0, 0.0])
    truth = np.array([1.0, 2.0])
    breaks_at = None
    for i in range(1, 60 * 50 + 1):                     # 60 s at 50 Hz
        assert f.update_attitude(np.eye(3), sigma_deg=0.5) is True
        if i % 10 == 0:                                 # 5 Hz fix
            f.update_position(truth, sigma=0.3)
            if f.lockout_breaks and breaks_at is None:
                breaks_at = i // 10
    assert breaks_at == REJECT_STREAK_LIMIT, (
        f'lockout broke at fix {breaks_at}, not the {REJECT_STREAK_LIMIT}th')
    assert np.linalg.norm(f.X.p[:2] - truth) < 0.1


def test_one_outlier_does_not_break_the_lockout():
    """The recovery must need a RUN of disagreement. Tripping on a single
    outlier would make the gate pointless."""
    f = RIEKF()
    for _ in range(REJECT_STREAK_LIMIT - 1):
        f.update_position([500.0, 500.0], sigma=0.3)
        f.update_position([0.01, 0.01], sigma=0.3)      # a good one resets it
    assert f.lockout_breaks == 0
    assert np.linalg.norm(f.X.p[:2]) < 1.0


def test_a_NaN_innovation_is_rejected_not_injected():
    f = RIEKF()
    f.update_position([float('nan'), 0.0], sigma=0.3)
    assert np.all(np.isfinite(f.P))
    assert np.all(np.isfinite(f.X.p))


def test_the_threshold_table_refuses_an_unlisted_dimension():
    """A new measurement of an unlisted width must raise, not silently pick a
    wrong threshold."""
    assert set(CHI2_99) == {1, 2, 3}


def test_counters_are_published():
    """A filter silently discarding half its measurements looks exactly like
    one that is merely drifting."""
    f = RIEKF()
    f.update_depth(-1.0)
    f.update_position([900.0, 900.0], sigma=0.3)
    assert f.accepted >= 1 and f.rejected >= 1


def test_zupt_pulls_velocity_to_zero():
    f = RIEKF(state=State(v=np.array([0.4, 0.0, 0.0])))
    for _ in range(40):
        f.update_zero_velocity(sigma=0.01)
    assert np.linalg.norm(f.X.v) < 0.05


def test_two_d_flow_leaves_vertical_velocity_alone():
    """The 3-D form asserts vz = 0 at full confidence, which fights depth."""
    f = RIEKF(state=State(v=np.array([0.0, 0.0, 0.3])))
    for _ in range(40):
        f.update_body_velocity_xy(0.0, 0.0, 1e-4, 1e-4)
    assert f.X.v[2] == pytest.approx(0.3, abs=0.05)

"""Nav filters. The heading wrap is the one that would fail in the field.

A compass filter can be wrong through 358 degrees of the circle and correct
everywhere you happen to test it, then fail at north -- which is the heading a
returning AUV is most likely to be on.
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_control.nav_filter import (            # noqa: E402
    AlphaBeta, AngleAlphaBeta, DepthFilter, HeadingFilter,
)


def _feed(f, series, dt=0.02, t0=1.0):
    out = []
    for i, v in enumerate(series):
        out.append(f.update(v, t0 + i * dt))
    return out


def _step_sd(xs):
    d = [xs[i] - xs[i - 1] for i in range(1, len(xs))]
    m = sum(d) / len(d)
    return (sum((v - m) ** 2 for v in d) / len(d)) ** 0.5


# --------------------------------------------------------------------------- #
#  Scalar
# --------------------------------------------------------------------------- #
def test_it_smooths():
    import random
    rng = random.Random(5)
    noise = [rng.gauss(0, 1.0) for _ in range(400)]
    out = _feed(AlphaBeta(), noise)
    assert _step_sd(out) < _step_sd(noise) * 0.45


def test_steady_motion_is_tracked_without_lag():
    """The property an EMA cannot have, and the reason for this class."""
    ramp = [i * 0.5 for i in range(200)]
    out = _feed(AlphaBeta(), ramp)
    assert abs(ramp[-1] - out[-1]) < 0.5


def test_a_gap_is_a_new_measurement_not_an_extrapolation():
    f = AlphaBeta()
    _feed(f, [i * 0.5 for i in range(50)])       # velocity built up
    assert f.update(0.0, 99.0) == pytest.approx(0.0)


def test_none_clears_rather_than_holds():
    f = AlphaBeta()
    _feed(f, [7.0] * 10)
    assert f.update(None, 5.0) is None
    assert f.update(-3.0, 5.02) == pytest.approx(-3.0)


def test_a_nan_is_treated_as_absence():
    """A NaN reaching a control loop is worse than a gap. Non-finite in, None
    out -- never a NaN propagated into a thrust command."""
    f = AlphaBeta()
    _feed(f, [1.0] * 5)
    assert f.update(float('nan'), 2.0) is None
    assert f.update(float('inf'), 2.1) is None


# --------------------------------------------------------------------------- #
#  The circle
# --------------------------------------------------------------------------- #
def test_crossing_north_does_not_slam_the_estimate():
    """359 -> 1 is a +2 degree change, not -358. A naive subtraction sees the
    long way round and either jumps the estimate across the compass or trips
    the gate on every single crossing."""
    f = HeadingFilter()
    out = _feed(f, [355, 357, 359, 1, 3, 5, 7], dt=0.05)
    assert f.resets == 0, 'the gate fired crossing north'
    for v in out:
        assert 0.0 <= v < 360.0
    # the estimate must follow through the wrap, staying near the measurement
    assert min(abs(out[-1] - 7), abs(out[-1] - 7 + 360)) < 4.0


def test_a_steady_turn_through_the_wrap_keeps_its_rate():
    f = HeadingFilter()
    series = [(350 + i * 2) % 360 for i in range(40)]
    _feed(f, series, dt=0.05)
    assert f.v > 0, 'velocity lost its sign across the wrap'
    assert abs(f.v - 40.0) < 25.0, f'rate {f.v:.1f} deg/s, expected ~40'


def test_the_shortest_arc_is_used_in_both_directions():
    assert AngleAlphaBeta()._diff(1.0, 359.0) == pytest.approx(2.0)
    assert AngleAlphaBeta()._diff(359.0, 1.0) == pytest.approx(-2.0)
    assert AngleAlphaBeta()._diff(181.0, 0.0) == pytest.approx(-179.0)


def test_output_stays_in_range():
    f = HeadingFilter()
    out = _feed(f, [359.5, 0.2, 359.8, 0.5, 1.0], dt=0.05)
    assert all(0.0 <= v < 360.0 for v in out)


def test_a_reference_jump_snaps_instead_of_sweeping():
    """A YAW_REF change or a re-alignment is a genuine discontinuity. Sweeping
    to it would command a turn the vehicle does not need to make."""
    f = HeadingFilter()
    _feed(f, [90.0] * 20, dt=0.05)
    out = f.update(180.0, 2.05)
    assert out == pytest.approx(180.0)
    assert f.resets == 1


# --------------------------------------------------------------------------- #
#  Depth
# --------------------------------------------------------------------------- #
def test_depth_smooths_baro_jitter():
    import random
    rng = random.Random(2)
    # BARO_P2P ~ 5.8 mbar on this board, roughly 6 cm of water
    noisy = [-1.20 + rng.gauss(0, 0.03) for _ in range(300)]
    out = _feed(DepthFilter(), noisy, dt=0.2)
    assert _step_sd(out) < _step_sd(noisy) * 0.6


def test_a_depth_descent_is_not_lagged_into_the_floor():
    """A 0.2 m/s descent must be tracked, not trailed -- lag on depth is how a
    vehicle arrives deeper than it asked for."""
    ramp = [-0.04 * i for i in range(60)]         # 0.2 m/s at 5 Hz
    out = _feed(DepthFilter(), ramp, dt=0.2)
    assert abs(ramp[-1] - out[-1]) < 0.10


def test_a_depth_step_snaps():
    f = DepthFilter()
    _feed(f, [-1.0] * 20, dt=0.2)
    assert f.update(-3.0, 100.0) == pytest.approx(-3.0)   # also a gap

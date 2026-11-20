"""A filter flown on flow with a scale error must still take the next fix.

Flow's sigma is per-sample noise, and its integral grows like a square root;
a scale error grows with distance. Without a term for it the filter, one fix
in, held 0.15 m across a transit that put it 0.5 m off, and refused the next
correct fix (measured-bars 99; `tools/studies/localization_candidates.py`).
"""
import math

import numpy as np
import pytest

from mongla_localization.inekf import GRAVITY, RIEKF
from mongla_localization.localization_node import FLOW_SCALE_SIGMA


def _transit(k, *, seconds=40.0, speed=0.4, scale=0.05):
    """Fix at the origin, then fly +x on flow reading `scale` long.

    Truth ends at speed * seconds; the filter believes (1 + scale) of that.
    """
    f = RIEKF(odometry_scale_sigma=k)
    f.update_position([0.0, 0.0], sigma=0.15)
    f.X.v = np.array([speed, 0.0, 0.0])
    dt = 0.02
    for _ in range(int(seconds / dt)):
        f.predict([0.0, 0.0, 0.0], -GRAVITY, dt)
        f.update_attitude(np.eye(3), sigma_deg=0.5)   # the board's, as the node feeds it
        f.update_body_velocity_xy(speed * (1 + scale), 0.0, 1e-4, 1e-4)
    return f, speed * seconds


@pytest.mark.parametrize('k, taken', [(FLOW_SCALE_SIGMA, True), (0.0, False)])
def test_a_correct_fix_after_a_scaled_transit_is_taken(k, taken):
    f, truth_x = _transit(k)
    off = f.X.p[0] - truth_x
    assert 0.6 < off < 1.0, 'the transit must have drifted for this to test anything'
    assert f.update_position([truth_x, 0.0], sigma=0.15) is taken
    if taken:
        assert abs(f.X.p[0] - truth_x) < 0.2


def test_the_floor_grows_with_distance_and_restarts_at_a_fix():
    f, d = _transit(FLOW_SCALE_SIGMA, seconds=25.0)
    assert math.sqrt(f.P[6, 6]) >= FLOW_SCALE_SIGMA * d * 0.99
    f.update_position([f.X.p[0], 0.0], sigma=0.1)
    assert f._odo_dist == 0.0
    assert math.sqrt(f.P[6, 6]) < 0.15


def test_off_by_default_so_the_filter_alone_is_unchanged():
    assert RIEKF().odometry_scale_sigma == 0.0

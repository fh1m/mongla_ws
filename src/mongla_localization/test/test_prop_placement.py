"""Placing a prop from sightings: truth tests, written before the code.

What would falsify it. The claim is not "the estimate is close" -- any average
is close on a good day. The claim is that the COVARIANCE it reports is honest
when the hull's own position is wrong, because that is the error that does not
average away: every sighting in a pass is taken from the same drifted pose.
So the scorer is coverage. Over many synthetic passes whose truth we chose, the
true prop must fall inside the reported 95 % ellipse about 95 % of the time. A
placement that divides the hull error by the number of sightings reports a
small ellipse and misses the truth most of the time -- the test builds that
naive estimate beside it and requires it to fail, so the scorer is shown to be
able to tell the two apart.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from mongla_localization.prop_placement import (CHI2_2DOF_95, CHI2_2DOF_99,
                                                Sighting, place)

TRUTH = (6.0, 2.0)


def _pass(rng, *, hull_sigma, yaw_sigma_deg, range_sigma, bearing_sigma_deg,
          n=8):
    """One pass: a common hull and heading error, then n noisy sightings.

    The hull drifts by ONE offset for the whole pass and the heading by ONE
    bias -- the reported pose is wrong the same way for every sighting.
    """
    hull_err = rng.normal(0.0, hull_sigma, 2)
    yaw_err = rng.normal(0.0, yaw_sigma_deg)
    out = []
    for k in range(n):
        true_hull = np.array([1.0 + 0.1 * k, 0.5 - 0.05 * k])
        dx, dy = np.subtract(TRUTH, true_hull)
        r = math.hypot(dx, dy)
        b = math.degrees(math.atan2(dy, dx))
        out.append(Sighting(
            hull_xy=tuple(true_hull + hull_err),
            range_m=r + rng.normal(0.0, range_sigma),
            range_sigma_m=range_sigma,
            bearing_deg=b + yaw_err + rng.normal(0.0, bearing_sigma_deg)))
    hull_cov = np.eye(2) * hull_sigma ** 2
    return out, hull_cov


def _inside(est_xy, cov, truth, chi2):
    d = np.subtract(truth, est_xy)
    return float(d @ np.linalg.solve(np.asarray(cov), d)) <= chi2


def test_the_reported_ellipse_contains_the_truth_as_often_as_it_claims():
    rng = np.random.default_rng(7)
    hits = naive_hits = 0
    trials = 2000
    for _ in range(trials):
        s, hull_cov = _pass(rng, hull_sigma=0.30, yaw_sigma_deg=1.5,
                            range_sigma=0.15, bearing_sigma_deg=1.0)
        p = place(s, hull_cov_xy=hull_cov, yaw_sigma_deg=1.5)
        assert p.ok, p.reason
        hits += _inside((p.x_m, p.y_m), p.cov, TRUTH, CHI2_2DOF_95)
        # The naive version: every sighting treated as independent, hull error
        # and all, so the whole covariance shrinks by n.
        naive = (np.asarray(p.cov_noise) + hull_cov) / len(s)
        naive_hits += _inside((p.x_m, p.y_m), naive, TRUTH, CHI2_2DOF_95)
    cover, naive_cover = hits / trials, naive_hits / trials
    assert 0.92 <= cover <= 0.995, cover
    # The scorer can see the difference: the naive ellipse misses badly.
    assert naive_cover < 0.60, naive_cover


def test_with_a_perfect_pose_the_sightings_do_average_down():
    """The other half of the claim: what IS independent must shrink with n."""
    rng = np.random.default_rng(3)
    s4, hc = _pass(rng, hull_sigma=0.0, yaw_sigma_deg=0.0, range_sigma=0.2,
                   bearing_sigma_deg=1.0, n=4)
    s16, _ = _pass(rng, hull_sigma=0.0, yaw_sigma_deg=0.0, range_sigma=0.2,
                   bearing_sigma_deg=1.0, n=16)
    a = place(s4, hull_cov_xy=hc, yaw_sigma_deg=0.0)
    b = place(s16, hull_cov_xy=hc, yaw_sigma_deg=0.0)
    assert np.trace(b.cov) < 0.5 * np.trace(a.cov)


def test_a_prior_one_metre_off_is_named():
    """The failure a comparable vehicle reported on the day: its course file
    put a prop one spacing away from where it stood. Placed from a good pose,
    that is a disagreement the placement must name, not average into."""
    rng = np.random.default_rng(11)
    s, hc = _pass(rng, hull_sigma=0.10, yaw_sigma_deg=0.5, range_sigma=0.10,
                  bearing_sigma_deg=0.5)
    off = place(s, hull_cov_xy=hc, yaw_sigma_deg=0.5,
                prior_xy=(TRUTH[0] + 1.0, TRUTH[1]))
    assert off.ok
    assert off.prior_offset_m == pytest.approx(1.0, abs=0.25)
    assert off.prior_disagrees
    same = place(s, hull_cov_xy=hc, yaw_sigma_deg=0.5, prior_xy=TRUTH)
    assert not same.prior_disagrees


def test_too_few_sightings_is_refused_not_guessed():
    rng = np.random.default_rng(1)
    s, hc = _pass(rng, hull_sigma=0.1, yaw_sigma_deg=0.5, range_sigma=0.1,
                  bearing_sigma_deg=0.5, n=2)
    p = place(s, hull_cov_xy=hc, yaw_sigma_deg=0.5)
    assert not p.ok and 'sightings' in p.reason
    assert math.isnan(p.x_m)


def test_a_non_finite_sighting_is_dropped_before_it_counts():
    rng = np.random.default_rng(2)
    s, hc = _pass(rng, hull_sigma=0.1, yaw_sigma_deg=0.5, range_sigma=0.1,
                  bearing_sigma_deg=0.5, n=3)
    s[0] = Sighting(hull_xy=s[0].hull_xy, range_m=float('nan'),
                    range_sigma_m=0.1, bearing_deg=s[0].bearing_deg)
    p = place(s, hull_cov_xy=hc, yaw_sigma_deg=0.5)
    assert not p.ok          # two good sightings left: below the floor


def test_the_chi_square_constants_are_the_table_values():
    # 2 degrees of freedom: the quantile is -2 ln(1 - p), exactly.
    assert CHI2_2DOF_95 == pytest.approx(-2.0 * math.log(0.05))
    assert CHI2_2DOF_99 == pytest.approx(-2.0 * math.log(0.01))


def test_two_props_placed_in_one_pass_have_a_better_SPACING_than_position():
    """The module's claim about spacing, measured: both placements share the
    pass's hull error, so it cancels in their difference."""
    rng = np.random.default_rng(5)
    other = (TRUTH[0], TRUTH[1] + 1.0)          # one spacing to starboard
    pos_err, gap_err = [], []
    for _ in range(500):
        hull_err = rng.normal(0.0, 0.30, 2)
        placed = []
        for target in (TRUTH, other):
            s = []
            for k in range(8):
                hull = np.array([1.0 + 0.1 * k, 0.5])
                dx, dy = np.subtract(target, hull)
                s.append(Sighting(
                    hull_xy=tuple(hull + hull_err),
                    range_m=math.hypot(dx, dy) + rng.normal(0.0, 0.1),
                    range_sigma_m=0.1,
                    bearing_deg=math.degrees(math.atan2(dy, dx))
                    + rng.normal(0.0, 0.5)))
            p = place(s, hull_cov_xy=np.eye(2) * 0.09, yaw_sigma_deg=0.0)
            placed.append(np.array([p.x_m, p.y_m]))
        pos_err.append(np.linalg.norm(placed[0] - TRUTH))
        gap_err.append(abs(np.linalg.norm(placed[1] - placed[0]) - 1.0))
    assert np.median(gap_err) < 0.33 * np.median(pos_err)

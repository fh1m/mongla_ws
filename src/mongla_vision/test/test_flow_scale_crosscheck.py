"""The flow-scale cross-check must REPORT a disagreement, never resolve one."""

import pytest

from mongla_vision.flow.scale_check import ScaleCrossCheck


# --------------------------------------------------------------------------- #
#  Round 6 -- the two heights
# --------------------------------------------------------------------------- #
def test_too_few_pairs_refuses():
    x = ScaleCrossCheck()
    for _ in range(3):
        x.add(1.0, 1.0, now=0.0)
    assert not x.check(now=0.0).ok


def test_agreeing_heights_report_agreement():
    x = ScaleCrossCheck()
    for i in range(10):
        x.add(1.00 + 0.01 * (i % 3), 1.00, now=float(i))
    r = x.check(now=10.0)
    assert r.ok and not r.disagree and r.ratio == pytest.approx(1.01, abs=0.02)


def test_a_disagreement_is_REPORTED_not_resolved():
    """It says the two differ, never which is right. pool_depth_m is the
    one nobody measures, and silently preferring either is how a stack
    acquires a number it cannot defend."""
    x = ScaleCrossCheck()
    for i in range(10):
        x.add(1.50, 1.00, now=float(i))          # 50 % apart
    r = x.check(now=10.0)
    assert r.ok and r.disagree
    assert 'DISAGREE' in r.reason and 'NOT' in r.reason


def test_one_bad_pair_does_not_move_the_answer():
    """A median, not a mean -- a single grating misread must not recalibrate
    every velocity the filter receives."""
    x = ScaleCrossCheck()
    for i in range(9):
        x.add(1.00, 1.00, now=float(i))
    x.add(50.0, 1.0, now=9.0)
    assert x.check(now=10.0).ratio == pytest.approx(1.0, abs=0.01)


def test_scatter_lowers_confidence():
    tight, loose = ScaleCrossCheck(), ScaleCrossCheck()
    for i in range(10):
        tight.add(1.0, 1.0, now=float(i))
        loose.add(1.0 + 0.4 * ((-1) ** i), 1.0, now=float(i))
    assert loose.check(now=10.0).confidence < tight.check(now=10.0).confidence


def test_stale_pairs_age_out():
    """A calibration must not outlive the conditions it was measured in."""
    x = ScaleCrossCheck(max_age_s=10.0)
    for i in range(10):
        x.add(1.0, 1.0, now=float(i))
    assert x.check(now=5.0).ok
    assert not x.check(now=500.0).ok


def test_half_a_pair_is_not_stored():
    x = ScaleCrossCheck()
    for i in range(10):
        x.add(1.0, 0.0, now=float(i))            # no barometric height
        x.add(0.0, 1.0, now=float(i))            # no grating height
    assert x.check(now=1.0).samples == 0

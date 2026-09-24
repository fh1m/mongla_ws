"""Round 6, the turbidity calibration, and the approach band.

Three small pure modules, each guarding a different way of being wrong:
  * the flow-scale cross-check must REPORT a disagreement, never resolve one;
  * the confidence calibration must never manufacture a detection;
  * the approach advisor must be able to say BACK OFF, which is the half a
    "get closer" controller cannot express and the half §23 measured.
"""
import math

import pytest

from mongla_vision.approach import (BACK_OFF, CLOSE, HOLD, Evidence, advise)
from mongla_vision.detection.confidence_calibration import (MAX_GAIN,
                                                            calibrate,
                                                            turbidity)
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
    """⛔ It says the two differ, never which is right. pool_depth_m is the
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


# --------------------------------------------------------------------------- #
#  Turbidity -> confidence
# --------------------------------------------------------------------------- #
def test_turbidity_is_zero_in_clear_water_and_one_in_the_worst():
    assert turbidity(500) == 0.0
    assert turbidity(5) == 1.0
    assert 0.0 < turbidity(100) < 1.0


def test_no_estimate_is_not_clear_water():
    """Guessing here applies the largest correction exactly when nothing is
    known."""
    assert math.isnan(turbidity(float('nan')))
    c = calibrate(0.4, float('nan'), enabled=True)
    assert c.value == 0.4 and c.gain == 0.0


def test_it_ships_off():
    assert calibrate(0.4, 30, enabled=False).gain == 0.0


def test_calibration_never_manufactures_a_detection():
    """⛔ THE WHOLE RISK. A calibration that can invent confidence would do it
    in exactly the conditions where the stack can least check it."""
    assert calibrate(0.0, 5, enabled=True).value == 0.0


def test_the_gain_is_bounded():
    for conf in (0.01, 0.2, 0.5, 0.9):
        c = calibrate(conf, 5, enabled=True)
        assert c.gain <= MAX_GAIN + 1e-9
        assert c.value <= 1.0


def test_murkier_water_raises_more_than_clear_water():
    murky = calibrate(0.4, 30, enabled=True)
    clear = calibrate(0.4, 500, enabled=True)
    assert murky.gain > clear.gain
    assert clear.gain == 0.0, 'clear water must be the identity'


# --------------------------------------------------------------------------- #
#  The approach band
# --------------------------------------------------------------------------- #
def test_far_away_it_closes():
    assert advise(40, 640).action == CLOSE


def test_inside_the_band_it_holds():
    assert advise(0.35 * 640, 640).action == HOLD


def test_too_close_it_BACKS_OFF():
    """⭐ The half §23 measured and a 'minimise range' controller cannot
    express: confidence rises 0.260 -> 0.521 and then FALLS."""
    a = advise(0.80 * 640, 640)
    assert a.action == BACK_OFF and 'FALLS' in a.reason


def test_no_size_holds_rather_than_guessing():
    assert advise(0, 640).action == HOLD
    assert advise(100, 0).action == HOLD


def test_evidence_ignores_a_repeated_view():
    """Fifty frames of a stationary vehicle are ONE observation."""
    e = Evidence()
    for _ in range(50):
        e.observe(0.9, bank_inliers=400)          # same view every time
    assert e.views == 0 and e.skipped == 50
    assert e.probability == pytest.approx(0.5), 'certainty from one look'


def test_evidence_accumulates_across_changed_views():
    e = Evidence()
    for _ in range(5):
        assert e.observe(0.8, bank_inliers=20)    # viewpoint genuinely moved
    assert e.views == 5 and e.probability > 0.95


def test_disagreeing_views_pull_the_estimate_down():
    e = Evidence()
    for _ in range(3):
        e.observe(0.8, bank_inliers=10)
    high = e.probability
    for _ in range(3):
        e.observe(0.2, bank_inliers=10)
    assert e.probability < high

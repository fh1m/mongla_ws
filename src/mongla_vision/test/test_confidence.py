"""Confidence-adaptive measurement noise, and why it is not raw NSA.

The claim under test: the filter should FOLLOW a confident box and REJECT a
doubtful one. A test that only checks "output is smoother" would pass for
infinite smoothing, which is the failure mode, so every test here is about
DISCRIMINATION between the two cases.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_vision.tracking.confidence import (          # noqa: E402
    ConfidenceModel, ConfidenceTrend, NSA_MIN_FACTOR, NSA_MAX_FACTOR)
from mongla_vision.tracking.kalman import TrackKalmanSmoother   # noqa: E402


# --------------------------------------------------------------------------- #
#  Why not the published formula
# --------------------------------------------------------------------------- #
def test_raw_NSA_would_be_nearly_a_noop_on_OUR_score_distribution():
    """The reason `normalise()` exists, asserted so nobody "simplifies" it back
    to the paper's `(1 - c)`.

    NSA assumes benchmark scores (p10 0.55 .. p90 0.95), where (1-c) spans
    0.45..0.05 -- a 9x swing in trust. Our measured underwater scores are
    p10 0.167 .. p90 0.439, where (1-c) spans 0.833..0.561: **1.48x**. The
    published formula barely adapts in exactly the regime it was added for."""
    raw_range = (1 - 0.167) / (1 - 0.439)
    assert raw_range < 1.6, raw_range

    m = ConfidenceModel()
    lo, hi = m.bounds()
    ours = m.nsa_factor(lo) / m.nsa_factor(hi)
    assert ours > 8.0, (
        f'normalised NSA dynamic range {ours:.1f}x -- it must be much wider '
        f'than raw NSA on this distribution ({raw_range:.2f}x) or it buys '
        f'nothing')


def test_the_factor_is_bounded_at_both_ends():
    """R is never scaled to zero: a score of 1.0 does not mean the box is
    exact, and a zero-noise measurement makes the filter discard its own
    prediction, so a single perfect-looking frame would teleport the track."""
    m = ConfidenceModel()
    for c in (0.0, 0.01, 0.5, 0.99, 1.0, 5.0):
        f = m.nsa_factor(c)
        assert NSA_MIN_FACTOR <= f <= NSA_MAX_FACTOR, (c, f)


def test_a_confident_box_is_trusted_MORE_than_a_doubtful_one():
    m = ConfidenceModel()
    assert m.nsa_factor(0.45) < m.nsa_factor(0.18)
    assert m.authority(0.45) > m.authority(0.18)


def test_authority_never_reaches_zero():
    """A weak detection is still the only information available. A vehicle
    that stops moving because the water got murky has failed the mission just
    as surely as one that drives off target."""
    m = ConfidenceModel()
    assert m.authority(0.0) > 0.0
    assert m.authority(0.0) == m.authority(0.0)      # not NaN


def test_the_model_adapts_to_the_DETECTOR_IN_FRONT_OF_IT():
    """The bounds are observed, not baked. A number fitted to one water was
    exactly the mistake `underwater.recommend()` made -- thresholds from two
    clips that called a third venue wrong.

    A detector scoring 0.7..0.9 must treat 0.75 as POOR; the same 0.75 from a
    detector scoring 0.15..0.45 is excellent."""
    strong = ConfidenceModel()
    for i in range(200):
        strong.observe(0.70 + 0.20 * (i % 11) / 10.0)
    weak = ConfidenceModel()
    for i in range(200):
        weak.observe(0.15 + 0.30 * (i % 11) / 10.0)
    assert strong.normalise(0.75) < 0.5, strong.normalise(0.75)
    assert weak.normalise(0.75) > 0.9, weak.normalise(0.75)


def test_a_detector_pinned_at_one_score_falls_back_rather_than_dividing_by_zero():
    m = ConfidenceModel()
    for _ in range(200):
        m.observe(0.42)
    assert m.bounds() == (0.167, 0.439)
    assert 0.0 <= m.normalise(0.42) <= 1.0


# --------------------------------------------------------------------------- #
#  The filter actually discriminates
# --------------------------------------------------------------------------- #
def _run(adaptive, confs, jump_at):
    """Feed a stationary target, then inject ONE displaced box, and report how
    far the filter moved. The displaced frame carries the confidence under
    test."""
    sm = TrackKalmanSmoother(adaptive_noise=adaptive,
                             measurement_noise=1.0, process_noise=0.01)
    x = y = 0.5
    out = 0.0
    for i, c in enumerate(confs):
        cx, cy = (x + 0.25, y) if i == jump_at else (x, y)
        hx, hy = sm.smooth(1, cx, cy, i * 0.033, False, conf=c)
        if i == jump_at:
            out = math.hypot(hx - x, hy - y)
    return out


def test_a_LOW_confidence_outlier_moves_the_filter_LESS_than_a_high_one():
    """THE claim. Same 0.25-wide jump, different confidence on the frame that
    jumps. The adaptive filter must move less for the doubtful one; the fixed
    filter cannot tell them apart at all."""
    n, jump = 60, 40
    good = [0.45] * n
    bad = [0.45] * n
    bad[jump] = 0.12

    a_good = _run(True, good, jump)
    a_bad = _run(True, bad, jump)
    assert a_bad < a_good, (a_bad, a_good)

    f_good = _run(False, good, jump)
    f_bad = _run(False, bad, jump)
    assert abs(f_bad - f_good) < 1e-9, (
        'fixed R moved differently for different confidences -- the control '
        'arm is not actually fixed, so this test proves nothing')


def test_R_is_restored_after_a_scaled_update():
    """The scale applies to ONE update. If it leaked, a single doubtful frame
    would permanently change how the filter weighs every later one."""
    sm = TrackKalmanSmoother(adaptive_noise=True, measurement_noise=1.0,
                             process_noise=0.01)
    for i in range(10):
        sm.smooth(1, 0.5, 0.5, i * 0.033, False, conf=0.45)
    flt = sm._filters[1]
    sm.smooth(1, 0.5, 0.5, 10 * 0.033, False, conf=0.05)
    assert abs(float(flt._kf.R[0, 0]) - 1.0) < 1e-9, float(flt._kf.R[0, 0])


def test_adaptive_off_is_byte_for_byte_the_old_behaviour():
    """Default-off for every existing caller: a confidence passed to a
    non-adaptive smoother must change nothing."""
    a = TrackKalmanSmoother(adaptive_noise=False, measurement_noise=1.0,
                            process_noise=0.01)
    b = TrackKalmanSmoother(adaptive_noise=False, measurement_noise=1.0,
                            process_noise=0.01)
    for i in range(30):
        p1 = a.smooth(1, 0.5 + i * 0.001, 0.5, i * 0.033, False)
        p2 = b.smooth(1, 0.5 + i * 0.001, 0.5, i * 0.033, False, conf=0.9)
        assert p1 == p2


# --------------------------------------------------------------------------- #
#  The early warning
# --------------------------------------------------------------------------- #
def test_a_sustained_confidence_drop_is_reported():
    """Measured on the `bin` clip: confidence fell 0.891 -> 0.357 BEFORE the
    box began to wander. The loop reads only an absolute floor, so today it
    learns about the degradation from the error signal -- after the fact."""
    t = ConfidenceTrend()
    for _ in range(60):
        t.observe(0.89)
    assert not t.degrading()
    for _ in range(12):
        t.observe(0.36)
    assert t.degrading()


def test_one_bad_frame_is_NOT_a_trend():
    """A single dropout is what the freshness and coast machinery already
    handles. Firing here too would make the warning meaningless."""
    t = ConfidenceTrend()
    for _ in range(60):
        t.observe(0.89)
    t.observe(0.05)
    assert not t.degrading()


def test_the_trend_says_nothing_until_it_has_seen_enough():
    t = ConfidenceTrend()
    for _ in range(5):
        t.observe(0.9)
    r = t.ratio()
    assert r != r          # NaN
    assert not t.degrading()

"""Two independent estimates, compared -- the failure neither can catch alone.

⛔ THE BLIND SPOT. The follower's only quality signals are forward-backward
error and point survival, and both are SELF-REFERENTIAL: they measure whether
LK tracked its own points consistently, not whether those points are on the
target. A follower that has smoothly latched onto a passing fish, or slid onto
the textured background beside a prop, has **perfect fb error and perfect
survival**. The tracking literature names this exactly -- similarity-based
checks "struggle to detect slow, cumulative drift, or cases where the tracker
settles on a visually similar but incorrect background patch".

⭐ The anchor is the independent witness. It is fitted frame-to-REFERENCE
against a stored patch, so its errors are uncorrelated with LK's: it cannot
drift *with* the follower because it never integrates. Until this, `arbitrate`
returned the follower whenever its own confidence cleared the bar and never
looked at the anchor box -- two independent estimates of one target, in one
function, never compared.

⚠ DISAGREEMENT DEMOTES, IT DOES NOT SWAP. When the centres separate we know
one is wrong and NOT which; preferring the anchor would be a guess. The ladder
keeps the follower's box (the fresher of the two) and cuts AUTHORITY, which is
the response that is correct under either explanation.
"""
from __future__ import annotations

import math

import pytest

from mongla_vision.tracking.lock_state import (DISAGREE_PX, Rung,
                                              _AGREE_BOOST, arbitrate)


def _box(cx, cy, w=60.0, h=60.0):
    return (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)


def _call(follow=None, anchor=None, **kw):
    """A mid-gap arbitration: a detection 0.5 s old, so authority is high but
    the ladder is on its carried rungs."""
    args = dict(now=10.0, last_detection_t=9.5, full_s=1.0, zero_s=3.0,
                follow=follow, follow_conf=0.9,
                anchor=anchor, anchor_conf=0.8)
    args.update(kw)
    return arbitrate(**args)


def test_agreeing_rungs_leave_authority_untouched():
    """The common case must cost nothing: two estimates on the same target."""
    s = _call(follow=_box(320, 240), anchor=_box(322, 243))
    assert s.rung is Rung.FOLLOW
    assert s.authority == pytest.approx(1.0)
    assert s.disagreement_px < 5.0


def test_a_drifted_follower_is_demoted_even_with_perfect_self_checks():
    """⭐⭐ THE CASE THE LADDER COULD NOT SEE. `follow_conf` is 0.9 -- the
    follower is certain, because its points round-trip perfectly. They are
    simply on the wrong object. Only the anchor disagrees."""
    s = _call(follow=_box(320, 240), anchor=_box(320 + 4 * DISAGREE_PX, 240))
    assert s.rung is Rung.FOLLOW, 'the box stays; we do not know who is wrong'
    assert s.authority < 0.3, 'a badly disagreeing pair must lose authority'
    assert s.disagreement_px == pytest.approx(4 * DISAGREE_PX)


def test_the_demotion_is_graded_not_a_cliff():
    """A target crossing in front of clutter produces a brief spike, and a
    cliff would make the hull stutter on something that resolves itself."""
    near = _call(follow=_box(320, 240),
                 anchor=_box(320 + 1.5 * DISAGREE_PX, 240)).authority
    far = _call(follow=_box(320, 240),
                anchor=_box(320 + 6.0 * DISAGREE_PX, 240)).authority
    assert 0.0 < far < near < 1.0, 'authority must fall smoothly with distance'


def test_disagreement_just_inside_the_bar_costs_nothing():
    """The bar is generous on purpose: ordinary parallax between a
    frame-to-frame and a frame-to-reference fit must not trip it."""
    s = _call(follow=_box(320, 240),
              anchor=_box(320 + DISAGREE_PX - 2.0, 240))
    assert s.authority == pytest.approx(1.0)


def test_one_rung_alone_reports_NaN_not_zero():
    """⛔ Absent is not agreement. With no anchor there is nothing to compare,
    and 0.0 would read as 'they agree perfectly' to any consumer."""
    s = _call(follow=_box(320, 240), anchor=None)
    assert s.rung is Rung.FOLLOW
    assert math.isnan(s.disagreement_px)
    assert s.authority == pytest.approx(1.0), \
        'a missing witness must not penalise the rung that did answer'


def test_the_anchor_rung_also_reports_the_disagreement():
    """When the follower is below its bar the anchor answers, and the number
    is still worth carrying -- an operator reading a scorecard wants to know
    the two disagreed even when the follower was not used."""
    s = _call(follow=_box(320, 240), follow_conf=0.0, anchor=_box(500, 240))
    assert s.rung is Rung.ANCHOR
    assert s.disagreement_px == pytest.approx(180.0)


def test_a_lost_ladder_is_unaffected():
    """Past zero authority nothing is trusted, and the cross-check must not
    resurrect a lock."""
    s = _call(follow=_box(320, 240), anchor=_box(322, 240),
              now=100.0, last_detection_t=1.0)
    assert s.rung is Rung.LOST


def test_a_detection_outranks_the_cross_check():
    """⚠ A real detection is ground truth for this frame. It must not be
    demoted because one carried rung disagrees with another."""
    s = _call(follow=_box(320, 240), anchor=_box(600, 240),
              detection=_box(318, 241), detection_conf=0.9,
              now=10.0, last_detection_t=10.0)
    assert s.rung is Rung.DETECTION
    assert s.authority == pytest.approx(1.0)


# ── agreement is evidence too (from the operator's own R&D fusion code) ─────


def _decayed(**kw):
    """A detection old enough that authority has decayed below 1.0, so a
    boost has room to show. Full authority for 1 s, zero at 3 s; at 2 s the
    ladder is mid-decay."""
    args = dict(now=10.0, last_detection_t=8.0, full_s=1.0, zero_s=3.0,
                follow_conf=0.9, anchor_conf=0.8)
    args.update(kw)
    return arbitrate(**args)


def test_two_agreeing_rungs_earn_MORE_authority_than_one():
    """⭐ THE IDEA FROM `r_&_d/features_v9.py`. Its fusion boosted confidence
    when the EKF and the visual estimate agreed on direction; the first
    version of this cross-check only punished disagreement and threw that
    half away.

    Two INDEPENDENT estimates -- LK integrating frame to frame, XFeat fitting
    frame to a stored reference -- landing on the same centre is a stronger
    statement than either alone."""
    alone = _decayed(follow=_box(320, 240), anchor=None).authority
    agreed = _decayed(follow=_box(320, 240),
                      anchor=_box(322, 241)).authority
    assert agreed > alone, 'corroboration must be worth something'
    assert agreed == pytest.approx(min(1.0, alone * _AGREE_BOOST))


def test_the_boost_needs_the_witness_to_be_confident_itself():
    """⚠ A low-confidence anchor agreeing is not corroboration -- it is two
    guesses. `anchor_conf` must clear the same bar the anchor rung would."""
    weak = _decayed(follow=_box(320, 240), anchor=_box(322, 241),
                    anchor_conf=0.0).authority
    strong = _decayed(follow=_box(320, 240), anchor=_box(322, 241)).authority
    assert strong > weak


def test_the_boost_never_exceeds_full_authority():
    """⛔ Authority is a fraction of what the control loop may do. Boosting
    past 1.0 would let a CARRIED rung outrank a fresh detection."""
    s = _call(follow=_box(320, 240), anchor=_box(321, 240))
    assert s.authority <= 1.0


def test_merely_not_conflicting_is_not_agreement():
    """The boost needs the two to be CLOSE, not just inside the disagreement
    bar. A pair 60 px apart on a 640-wide frame is not corroboration."""
    close = _decayed(follow=_box(320, 240), anchor=_box(325, 240)).authority
    apart = _decayed(follow=_box(320, 240),
                     anchor=_box(320 + DISAGREE_PX - 4, 240)).authority
    assert close > apart, 'the bonus must fall off well inside the bar'

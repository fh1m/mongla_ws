"""The arbiter: one target position, and never a fabricated one.

Two properties make it safe to wire to thrusters, and both are asserted here
rather than described:

  1. Authority decays from the last real DETECTION, whatever rung is talking.
     A follower and an anchor cannot between them hold authority at 1.0 while
     the vehicle drives on a position nothing has confirmed in ten seconds.
  2. Nothing is fabricated. Every rung may refuse; refusals fall through to
     LOST rather than to a guess.

The ramp constants come from the measured gap distribution -- p90 0.651 s,
p99 2.418 s -- not from taste.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_vision.tracking.lock_state import (       # noqa: E402
    LockState, Rung, arbitrate, authority_for,
    FULL_AUTHORITY_S, ZERO_AUTHORITY_S)

BOX = (10.0, 20.0, 110.0, 120.0)
OTHER = (200.0, 200.0, 260.0, 260.0)


# --------------------------------------------------------------------------- #
#  Priority
# --------------------------------------------------------------------------- #
def test_a_live_detection_wins_and_RESETS_the_clock():
    """A detection is the confirmation the decay counts time since, so it
    restores full authority -- when its CLOCK says it is live.

    ⛔ PREMISE CORRECTED 2026-09-09. This used to read "however old the
    previous lock was" and passed `last_detection_t=1.0` with `now=100.0` --
    a 99 s old clock alongside a present box. Those two inputs are
    inconsistent for the real caller, which sets `_det_box` and `_det_t`
    together, EXCEPT in one case: `lock_node._on_det` clears its cached box
    only when a message arrives carrying no match, so during a TOTAL detector
    outage the callback never runs and the stale box persists with its old
    clock. That is precisely the case the ladder exists for, and the old
    assertion demanded the wrong answer for it -- measured on the Pi as
    `rung=detection, authority=1.0` held across a 16 s detector pause.
    """
    s = arbitrate(now=100.0, last_detection_t=99.96,
                  detection=BOX, detection_conf=0.9,
                  follow=OTHER, follow_conf=0.9)
    assert s.rung is Rung.DETECTION
    assert s.xyxy == BOX
    assert s.authority == 1.0
    # Its TRUE age (issue #33): a live box is 40 ms old, not zero.
    assert s.age_s == pytest.approx(0.04)


def test_a_STALE_cached_detection_does_not_masquerade_as_live():
    """The defect itself. A box whose clock is past `full_s` must fall through
    to the decay path, where a follower that IS current outranks it."""
    s = arbitrate(now=100.0, last_detection_t=1.0,      # 99 s of silence
                  detection=BOX, detection_conf=0.9,
                  follow=OTHER, follow_conf=0.9,
                  full_s=0.7, zero_s=2.5)
    assert s.rung is not Rung.DETECTION, (
        'a detection box 99 s old was reported as a live detection at full '
        'authority -- "the vehicle has a position" became "the vehicle saw '
        'the target"')
    assert s.rung is Rung.LOST, (
        'past zero_s every rung is out of authority, so LOST is the only '
        'honest answer')


def test_a_stale_detection_yields_to_a_follower_INSIDE_the_horizon():
    """Between full_s and zero_s the ladder still has something to say, and
    the current rung must outrank the frozen one."""
    s = arbitrate(now=100.0, last_detection_t=98.8,     # 1.2 s: past full_s
                  detection=BOX, detection_conf=0.9,
                  follow=OTHER, follow_conf=0.9,
                  full_s=0.7, zero_s=2.5)
    assert s.rung is Rung.FOLLOW
    assert s.xyxy == OTHER
    assert 0.0 < s.authority < 1.0


def test_follow_is_preferred_to_anchor():
    """The follower is the fresher estimate -- one gap old at most, against an
    anchor matching a reference from arbitrarily long ago."""
    s = arbitrate(now=10.2, last_detection_t=10.0,
                  follow=BOX, follow_conf=0.8,
                  anchor=OTHER, anchor_conf=0.9)
    assert s.rung is Rung.FOLLOW and s.xyxy == BOX


def test_anchor_carries_it_when_the_follower_has_refused():
    s = arbitrate(now=10.2, last_detection_t=10.0,
                  follow=None, follow_conf=0.0,
                  anchor=BOX, anchor_conf=0.7)
    assert s.rung is Rung.ANCHOR and s.xyxy == BOX


def test_every_rung_refusing_is_LOST_not_a_guess():
    s = arbitrate(now=10.1, last_detection_t=10.0)
    assert s.rung is Rung.LOST
    assert s.xyxy is None and s.have_target is False
    assert s.confidence == 0.0


def test_a_low_confidence_rung_is_not_used():
    """Same idea as `vision.ctrl_conf` on the detector: a rung that barely
    believes itself should not steer."""
    s = arbitrate(now=10.2, last_detection_t=10.0,
                  follow=BOX, follow_conf=0.01)
    assert s.rung is not Rung.FOLLOW


# --------------------------------------------------------------------------- #
#  The decay -- the property that stops a lower rung driving for ever
# --------------------------------------------------------------------------- #
def test_authority_is_full_inside_the_p90_gap_and_zero_past_the_p99():
    """Measured: p90 gap 0.651 s, p99 2.418 s. Giving up before the p90 would
    abandon nine gaps in ten that the lower rungs genuinely cover; holding past
    the p99 is a story about the past."""
    assert authority_for(0.0) == 1.0
    assert authority_for(FULL_AUTHORITY_S) == 1.0
    assert authority_for(ZERO_AUTHORITY_S) == 0.0
    assert authority_for(1000.0) == 0.0
    assert FULL_AUTHORITY_S >= 0.651        # covers the measured p90
    assert ZERO_AUTHORITY_S >= 2.418        # reaches the measured p99


def test_the_ramp_is_LINEAR_not_a_cliff():
    """A cliff makes the hull lurch the moment the lock ages out -- the relay
    behaviour that made `heading_lock` limit-cycle until its floor was
    tapered."""
    mid = (FULL_AUTHORITY_S + ZERO_AUTHORITY_S) * 0.5
    assert authority_for(mid) == pytest.approx(0.5, abs=1e-6)


def test_a_lower_rung_CANNOT_hold_authority_open():
    """THE safety property. A follower and an anchor reporting perfect
    confidence for ever must still decay to zero, because neither of them has
    CONFIRMED anything -- they have only propagated."""
    s = arbitrate(now=10.0 + ZERO_AUTHORITY_S + 0.01, last_detection_t=10.0,
                  follow=BOX, follow_conf=1.0, anchor=BOX, anchor_conf=1.0)
    assert s.rung is Rung.LOST
    assert s.confidence == 0.0


def test_confidence_multiplies_rung_trust_by_time_decay():
    """A confident anchor on an old lock is still an old lock."""
    mid = (FULL_AUTHORITY_S + ZERO_AUTHORITY_S) * 0.5
    s = arbitrate(now=10.0 + mid, last_detection_t=10.0,
                  anchor=BOX, anchor_conf=0.8)
    assert s.rung is Rung.ANCHOR
    assert s.confidence == pytest.approx(0.4, abs=1e-3)


def test_never_having_seen_a_detection_is_LOST():
    s = arbitrate(now=5.0, last_detection_t=0.0, follow=BOX, follow_conf=1.0)
    assert s.rung is Rung.LOST


def test_the_centre_helper_matches_the_box():
    s = LockState(rung=Rung.DETECTION, xyxy=(0.0, 0.0, 100.0, 50.0))
    assert s.centre == (50.0, 25.0)
    assert LockState().centre is None


# --------------------------------------------------------------------------- #
#  issue #33: a detection inside full_s is not automatically LIVE
# --------------------------------------------------------------------------- #
def test_a_follower_on_this_frame_outranks_a_0_6_s_old_detection():
    """The issue's falsifier. 0.6 s without a new detection is a dozen missed
    frames; the follower tracked the target through them, on the current
    frame. It wins, and the age says how long it has been."""
    s = arbitrate(now=10.0, last_detection_t=9.4,
                  detection=BOX, detection_conf=0.9,
                  follow=OTHER, follow_conf=0.8)
    assert s.rung is Rung.FOLLOW
    assert s.age_s == pytest.approx(0.6)


def test_a_detection_from_this_frame_still_wins_with_full_authority():
    s = arbitrate(now=10.0, last_detection_t=10.0,
                  detection=BOX, detection_conf=0.9,
                  follow=OTHER, follow_conf=0.8)
    assert s.rung is Rung.DETECTION
    assert s.authority == 1.0
    assert s.age_s == 0.0


def test_a_past_live_detection_still_beats_the_anchor_and_says_its_age():
    """No follower: the stale-but-usable detection is still the best evidence
    -- the anchor was never the detector's equal -- but never with age 0."""
    s = arbitrate(now=10.0, last_detection_t=9.6,
                  detection=BOX, detection_conf=0.9,
                  anchor=OTHER, anchor_conf=0.9)
    assert s.rung is Rung.DETECTION
    assert s.age_s == pytest.approx(0.4)


def test_the_live_window_follows_the_detector_it_is_given():
    """48 ms of p95 latency plus one detector interval; the design default is
    the 20 Hz rate the ladder was sized at, and it never exceeds full_s."""
    from mongla_vision.tracking.lock_state import (
        DETECTION_LIVE_S, FULL_AUTHORITY_S, live_window)
    assert DETECTION_LIVE_S == pytest.approx(0.048 + 0.05)
    assert live_window(None) == pytest.approx(DETECTION_LIVE_S)
    assert live_window(1 / 15.0) == pytest.approx(0.048 + 1 / 15.0)
    assert live_window(1 / 3.0) == pytest.approx(0.048 + 1 / 3.0)
    assert live_window(5.0) == pytest.approx(FULL_AUTHORITY_S)
    # A 15 fps camera's every detection must be live for its whole interval,
    # or the rung would flap between DETECTION and FOLLOW on a healthy feed.
    s = arbitrate(now=10.0, last_detection_t=10.0 - (0.048 + 1 / 15.0) + 1e-6,
                  detection=BOX, detection_conf=0.9,
                  follow=OTHER, follow_conf=0.8,
                  live_s=live_window(1 / 15.0))
    assert s.rung is Rung.DETECTION

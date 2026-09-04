"""The instrument must be right before any number it prints means anything.

These tests inject sequences whose answer is known by construction. That is
the only way to trust a measurement tool: `check_tracker.py` has shipped for
rounds reporting a "predicted ratio" that cannot distinguish one 400 ms
blackout from a hundred single-frame flickers, and nothing caught it because
nobody fed it a sequence with a known answer.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_vision.continuity import (           # noqa: E402
    Obs, analyse, count_switches, find_gaps, jitter, reacquire_times, LADDER,
)


def _seq(pattern, hz=30.0, cx=0.5, cy=0.5, tid=1):
    """'..X..' -> frames at `hz`; X = seen, . = missing."""
    dt = 1.0 / hz
    return [Obs(t=i * dt, seen=(c == 'X'), score=0.9 if c == 'X' else 0.0,
                cx=cx if c == 'X' else float('nan'),
                cy=cy if c == 'X' else float('nan'),
                track_id=tid if c == 'X' else -1)
            for i, c in enumerate(pattern)]


# --------------------------------------------------------------------------- #
#  Gaps -- the measurement that does not exist anywhere else
# --------------------------------------------------------------------------- #
def test_a_known_gap_is_measured_exactly():
    """THE core assertion, and the definition matters.

    The gap runs from the LAST SIGHTING to the next one, because that is what
    the control loop compares against: `coast_s` is checked as
    `monotonic() - _last_real[id]` and `_last_real` is stamped on the last
    real detection. Three missing frames at 30 Hz is therefore FOUR frame
    periods, not three -- measuring from the first miss would under-report
    every gap by a frame period, in the direction that flatters us."""
    g = find_gaps(_seq('XX...XX', hz=30.0))
    assert len(g) == 1
    assert g[0].frames == 3
    assert g[0].duration == pytest.approx(4 / 30.0, abs=1e-9)


def test_an_unbroken_run_has_no_gaps():
    assert find_gaps(_seq('XXXXXX')) == []


def test_leading_and_trailing_absence_are_NOT_gaps():
    """Before the first sighting the target may simply not be in view, and
    after the last one the clip ended. Counting those would make a video that
    opens on a wall look like a detector failure -- the same error as
    measuring 'recall' with the camera pointed at a room."""
    g = find_gaps(_seq('...XX...'))
    assert g == []


def test_a_target_never_seen_yields_no_gaps_rather_than_one_huge_one():
    assert find_gaps(_seq('......')) == []


def test_many_short_flickers_are_DISTINGUISHED_from_one_long_blackout():
    """The distinction `check_tracker.py`'s scalar throws away, and the one
    that decides whether coast_s=0.8 is the right size. Both sequences lose
    the same NUMBER of frames."""
    flicker = analyse(_seq('X.X.X.X.X.X.X.X.X.X', hz=30.0), 'flicker')
    blackout = analyse(_seq('XXXXX.........XXXXX', hz=30.0), 'blackout')
    assert flicker.seen == blackout.seen == 10
    # Same frames lost, completely different operational meaning:
    assert len(flicker.gaps) == 9, 'nine separate flickers'
    assert len(blackout.gaps) == 1, 'one continuous blackout'
    # And the one that matters to the ladder -- the longest single absence:
    worst_flicker = max(g.duration for g in flicker.gaps)
    worst_blackout = max(g.duration for g in blackout.gaps)
    assert worst_blackout == pytest.approx(10 / 30.0, abs=1e-9)
    assert worst_flicker == pytest.approx(2 / 30.0, abs=1e-9)
    # A scalar 'predicted ratio' reports these two as identical. That is the
    # defect this module exists to remove.
    assert flicker.presence == blackout.presence


def test_the_ladder_answers_which_rung_covers_the_losses():
    """A 1.2 s gap at 10 Hz is survivable by the Kalman predict window but not
    by coast_s or lost_grace_s -- which is the actual operational question."""
    r = analyse(_seq('X' + '.' * 11 + 'X', hz=10.0), 'one long gap')
    assert r.over(0.80) == 1, 'a 1.2 s gap is not covered by coast_s'
    assert r.over(1.00) == 1, '...nor by lost_grace_s'
    assert r.over(1.50) == 0, '...but the Kalman window would cover it'


def test_every_ladder_rung_is_a_real_constant():
    """If a rung drifts from the code the report silently answers about a
    threshold nobody ships."""
    from duburi_control.motion_vision import VISION_FRESH_ZERO_S
    import re
    src = (Path(__file__).resolve().parents[2] / 'duburi_manager'
           / 'duburi_manager' / 'vision_tunables.py').read_text()

    def _deck(name):
        m = re.search(rf"'vision\.{name}':\s*([0-9.]+)", src)
        assert m, f'{name} not found in vision_tunables.py'
        return float(m.group(1))

    values = [v for _n, v in LADDER]
    for expect, what in ((VISION_FRESH_ZERO_S, 'freshness zero'),
                         (_deck('coast_s'), 'coast_s'),
                         (_deck('lost_grace_s'), 'lost_grace_s')):
        assert any(abs(v - expect) < 1e-9 for v in values), (
            f'{what} = {expect} is not a rung in LADDER -- the report would '
            f'answer about a threshold nobody ships')


# --------------------------------------------------------------------------- #
#  Percentiles, not maxima
# --------------------------------------------------------------------------- #
def test_one_outlier_does_not_move_the_p50():
    """A threshold sized to satisfy a maximum is a threshold chosen by noise
    -- measured live last round, where a 154 ms worst pipeline age turned out
    to be one sample in 2231 with a p99 of 29.6 ms."""
    normal = analyse(_seq('X.X' * 30, hz=30.0), 'normal')
    plus_outlier = analyse(_seq('X.X' * 30 + '.' * 60 + 'X', hz=30.0), 'outlier')
    from duburi_vision.continuity import _pct
    d1 = [g.duration for g in normal.gaps]
    d2 = [g.duration for g in plus_outlier.gaps]
    assert _pct(d2, 50) == pytest.approx(_pct(d1, 50), abs=1e-9)
    assert max(d2) > 10 * max(d1), 'the outlier must still be visible in max'


# --------------------------------------------------------------------------- #
#  ID switches
# --------------------------------------------------------------------------- #
def test_a_relabel_in_the_same_place_is_a_switch():
    o = _seq('XXXX', tid=1) + _seq('XXXX', tid=2)
    for i, x in enumerate(o):
        x.t = i / 30.0
    assert count_switches(o) == 1


def test_a_NEW_object_elsewhere_is_not_a_switch():
    """The reason a bare id-change count is not a switch count."""
    a = _seq('XXXX', cx=0.2, tid=1)
    b = _seq('XXXX', cx=0.9, tid=2)
    for i, x in enumerate(a + b):
        x.t = i / 30.0
    assert count_switches(a + b) == 0


# --------------------------------------------------------------------------- #
#  Reacquire and jitter
# --------------------------------------------------------------------------- #
def test_reacquire_requires_STABLE_return_not_one_lucky_frame():
    """A detector that flickers back for a single frame has given the control
    loop nothing to steer on -- `align_stable_frames` exists for this."""
    o = _seq('XX...X.X..XXX', hz=30.0)
    times = reacquire_times(o, stable_frames=3)
    assert len(times) == 1, times
    assert times[0] > 3 / 30.0, 'the single-frame returns must not count'


def test_jitter_does_not_count_the_jump_across_a_gap():
    """Movement either side of a 500 ms absence is real target motion, not
    jitter. Counting it would let a flickering detector look smooth."""
    a = Obs(t=0.0, seen=True, cx=0.1, cy=0.5)
    miss = [Obs(t=0.03 * i, seen=False) for i in range(1, 16)]
    b = Obs(t=0.5, seen=True, cx=0.9, cy=0.5)
    assert jitter([a] + miss + [b]) == []


def test_presence_is_the_headline():
    r = analyse(_seq('XXXXXXXXXX'), 'perfect')
    assert r.presence == 1.0
    r2 = analyse(_seq('X.X.X.X.X.'), 'half')
    assert r2.presence == pytest.approx(0.5)

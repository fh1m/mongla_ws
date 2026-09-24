"""The rung below detection, tested against cases where truth is known.

Not agreement tests. Each case constructs a situation whose right answer is
known independently of the code, including the two that matter most: a dim
target that IS there, and a departed target whose weak boxes chain just as
convincingly. Separating those is the module's entire reason to exist.
"""
from __future__ import annotations

import math

import pytest

from mongla_vision.tracking.presence import (
    GATE_PX, MAX_CONFIDENCE, MIN_RUN, SCORE_FLOOR, Presence,
    PresenceAccumulator, WeakBox)

# A 640x480 frame with plausible intrinsics, matching the measurement.
W, H, FX, FY = 640.0, 480.0, 500.0, 500.0
CENTRE = (320.0, 240.0)


def make(armed_at=CENTRE, cls='person'):
    acc = PresenceAccumulator()
    acc.note_detection(armed_at[0], armed_at[1], cls, W, H, FX, FY)
    return acc


def walk(acc, start, step, n, cls='person', score=0.30, t0=0.0, dt=0.02):
    """Feed a chain of weak boxes drifting by `step` px per frame."""
    out = []
    x, y = start
    for i in range(n):
        out.append(acc.observe([WeakBox(score, x, y, cls)], t0 + i * dt))
        x += step[0]
        y += step[1]
    return out


def test_a_dim_target_in_the_interior_is_declared_after_min_run():
    acc = make()
    res = walk(acc, CENTRE, (5.0, 0.0), MIN_RUN)
    assert [r.ok for r in res[:-1]] == [False] * (MIN_RUN - 1), \
        'presence must not be declared before the chain is long enough'
    assert res[-1].ok
    assert res[-1].run == MIN_RUN
    assert res[-1].reason == 'presence'


def test_a_departed_target_is_refused_however_well_its_boxes_chain():
    """⭐ THE CASE THE MODULE EXISTS FOR. The measurement found the departed
    regime links at 1.000 -- better than the in-view regime's 0.938. A chain
    is therefore NOT evidence on its own. Armed on a detection in the margin
    band, a perfect chain must still be refused."""
    acc = make(armed_at=(6.0, 240.0))           # hard against the left edge
    assert not acc.armed
    res = walk(acc, (6.0, 240.0), (0.0, 0.0), 12)
    assert not any(r.ok for r in res), \
        'a flawless chain on a departing target must never be declared'
    assert 'not armed' in res[-1].reason


def test_the_interlock_bites_when_broken():
    """Injection-verify: a guard that has never failed against a real defect
    is not a guard. Force `_armed` true -- the single line the interlock sets
    -- and the same departed-target chain is now wrongly declared. That is the
    defect the interlock prevents, demonstrated."""
    acc = make(armed_at=(6.0, 240.0))
    assert not any(r.ok for r in walk(acc, (6.0, 240.0), (0.0, 0.0), 12))
    acc._armed = True                            # the injected defect
    acc._chain = None
    assert any(r.ok for r in walk(acc, (6.0, 240.0), (0.0, 0.0), 12)), \
        'if this passes with the interlock disabled the test proves nothing'


def test_a_box_beyond_the_gate_breaks_the_chain():
    acc = make()
    walk(acc, CENTRE, (0.0, 0.0), MIN_RUN)
    jump = (CENTRE[0] + GATE_PX + 5.0, CENTRE[1])
    r = acc.observe([WeakBox(0.40, jump[0], jump[1], 'person')], 1.0)
    assert not r.ok and r.reason == 'no linked box'


def test_the_chain_survives_motion_inside_the_gate():
    """A target moving at just under the gate must stay linked -- the gate is
    a motion allowance, not a stillness requirement."""
    acc = make()
    res = walk(acc, CENTRE, (GATE_PX - 2.0, 0.0), MIN_RUN + 2)
    assert res[-1].ok and res[-1].run == MIN_RUN + 2


def test_another_class_is_not_evidence():
    """A sub-threshold chair must not count toward a person's chain."""
    acc = make(cls='person')
    res = walk(acc, CENTRE, (0.0, 0.0), 8, cls='chair')
    assert not any(r.ok for r in res)


def test_boxes_below_the_noise_floor_are_not_evidence():
    acc = make()
    res = walk(acc, CENTRE, (0.0, 0.0), 8, score=SCORE_FLOOR - 0.01)
    assert not any(r.ok for r in res)


def test_a_stale_chain_is_dropped_rather_than_carried():
    acc = make()
    walk(acc, CENTRE, (0.0, 0.0), MIN_RUN)
    r = acc.observe([WeakBox(0.40, CENTRE[0], CENTRE[1], 'person')], 99.0)
    assert not r.ok and r.reason == 'stale'


def test_a_detection_resets_the_chain():
    """The rung never runs longer than one dropout. A real detection is the
    reset, exactly as it is for the follower."""
    acc = make()
    assert walk(acc, CENTRE, (0.0, 0.0), MIN_RUN)[-1].ok
    acc.note_detection(CENTRE[0], CENTRE[1], 'person', W, H, FX, FY)
    r = acc.observe([WeakBox(0.40, CENTRE[0], CENTRE[1], 'person')], 5.0)
    assert not r.ok and r.run == 1


def test_confidence_is_capped_below_a_detection():
    """Evidence the model refused to call must never outweigh evidence it
    called, however long the chain runs."""
    acc = make()
    res = walk(acc, CENTRE, (0.0, 0.0), 40)
    assert max(r.confidence for r in res) <= MAX_CONFIDENCE
    assert Presence(False).confidence == 0.0


def test_the_strongest_box_is_taken_not_the_nearest():
    """⛔ Picking the nearest box to our own prediction would let the chain
    chase itself. Given a weak box exactly on the prediction and a stronger
    one offset within the gate, the stronger must win."""
    acc = make()
    acc.observe([WeakBox(0.30, CENTRE[0], CENTRE[1], 'person')], 0.0)
    r = acc.observe([WeakBox(0.22, CENTRE[0], CENTRE[1], 'person'),
                     WeakBox(0.44, CENTRE[0] + 20.0, CENTRE[1], 'person')], 0.02)
    assert math.isclose(acc._chain.x, CENTRE[0] + 20.0)
    assert r.run == 2


def test_an_empty_frame_breaks_the_chain_rather_than_coasting():
    acc = make()
    walk(acc, CENTRE, (0.0, 0.0), MIN_RUN)
    r = acc.observe([], 1.0)
    assert not r.ok and r.reason == 'no linked box'


def test_drop_disarms():
    acc = make()
    acc.drop()
    assert not acc.armed
    assert not acc.observe([WeakBox(0.40, CENTRE[0], CENTRE[1], 'person')],
                           0.0).ok


@pytest.mark.parametrize('run,expect_ok', [(1, False), (2, False), (3, True)])
def test_min_run_is_the_declared_constant(run, expect_ok):
    acc = make()
    assert walk(acc, CENTRE, (0.0, 0.0), run)[-1].ok is expect_ok

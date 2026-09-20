"""Freshness thresholds are DERIVED from the sensor, not configured for one.

A competition can put any camera in front of us. The two we have already
differ by 2.4x -- measured, same instrument, each detector alone:

                     forward        downward
    detections       77.1 Hz         30.1 Hz
    sample age med   22.10 ms        46.51 ms
            max      32.88 ms        56.42 ms
    interval med     13.44 ms        32.41 ms

`sample.age_s` is time since CAPTURE, so a threshold has to cover the PIPELINE
(or the loop never reaches full authority) plus enough INTERVALS to ride out a
missed detection (or it decays during normal operation). Neither term is
constant across cameras, so neither can be a constant.

These tests pin the INVARIANTS rather than the numbers, because the numbers
are supposed to move with the hardware. What must hold for ANY sensor:

  * a healthy camera sits at FULL authority -- if it does not, the vehicle is
    permanently throttled for no reason
  * blind driving is BOUNDED, whatever the frame rate, and always ends before
    `lost_grace_s` declares the target lost -- that ladder is what makes a
    dropout a glide rather than a lurch
  * the ramp is never inverted, zero-width, or a step
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_control.motion_vision import (            # noqa: E402
    _fresh_bounds, _freshness,
    VISION_FRESH_FULL_S, VISION_FRESH_ZERO_S,
    VISION_FRESH_FULL_MAX_S, VISION_FRESH_ZERO_MAX_S,
)

# (name, detections Hz, pipeline age s, worst observed age s)
_MEASURED = [
    ('forward  77 Hz', 77.1, 0.0221, 0.0329),
    ('downward 30 Hz', 30.1, 0.0465, 0.0564),
]
# NOT MEASURED -- shapes, to prove nothing is fitted to the two we own. The
# rates are plausible; the pipeline ages are ASSUMED, so these rows may only
# assert INVARIANTS, never numbers. Anything requiring a real number belongs
# in `_MEASURED`.
_HYPOTHETICAL = [
    ('a 200 Hz camera', 200.0, 0.008, 0.012),
    ('a 15 Hz camera',  15.0,  0.090, 0.120),
]
# A pipeline slow enough that full authority is CAPPED rather than derived.
_SLOW = ('a 5 Hz camera', 5.0, 0.250, 0.400)


@pytest.mark.parametrize('name,hz,pipe,worst',
                         _MEASURED + _HYPOTHETICAL + [_SLOW],
                         ids=[c[0] for c in _MEASURED + _HYPOTHETICAL + [_SLOW]])
def test_authority_is_limited_by_the_HARDWARE_not_by_the_rule(name, hz, pipe,
                                                              worst):
    """Two claims, and which one applies is decided by the DATA, not by which
    row this is.

    A camera whose worst normal age fits inside the absolute cap must reach
    FULL authority -- if the rule throttled it, the vehicle would move
    sluggishly for a reason that is ours and not the hardware's, which is the
    failure that matters most here.

    A camera whose worst normal age EXCEEDS the cap must not. That is not the
    rule being conservative; it is the sensor genuinely not knowing where the
    target is right now. Reduced authority is the honest response and
    `_warn_low_fps` tells the operator which case they are in.
    """
    full, _zero = _fresh_bounds(1.0 / hz, pipe)
    authority = _freshness(worst, 1.0 / hz, pipe)
    if worst <= VISION_FRESH_FULL_MAX_S:
        assert full >= worst, (
            f'{name}: threshold {full * 1000:.1f} ms is below the worst age '
            f'this camera normally produces ({worst * 1000:.1f} ms) -- it '
            f'would never reach full authority, and the limit would be OURS')
        assert authority == 1.0
    else:
        assert authority < 1.0, (
            f'{name}: worst age {worst * 1000:.0f} ms is past the '
            f'{VISION_FRESH_FULL_MAX_S * 1000:.0f} ms cap, so full authority '
            f'would be steering on data the sensor cannot supply')


@pytest.mark.parametrize('name,hz,pipe,worst',
                         _MEASURED + _HYPOTHETICAL,
                         ids=[c[0] for c in _MEASURED + _HYPOTHETICAL])
def test_blind_driving_is_bounded_on_every_sensor(name, hz, pipe, worst):
    """The ladder: authority must reach zero BEFORE `lost_grace_s` (1.0 s)
    declares the target lost, or a dropout is a lurch instead of a glide. A
    5 Hz camera would derive its way past that without the ceiling."""
    _full, zero = _fresh_bounds(1.0 / hz, pipe)
    assert zero <= VISION_FRESH_ZERO_MAX_S
    assert zero < 1.0, f'{name}: still driving when LOST is declared'
    assert _freshness(zero + 0.001, 1.0 / hz, pipe) == 0.0


@pytest.mark.parametrize('name,hz,pipe,worst',
                         _MEASURED + _HYPOTHETICAL,
                         ids=[c[0] for c in _MEASURED + _HYPOTHETICAL])
def test_the_ramp_is_real_on_every_sensor(name, hz, pipe, worst):
    """Not inverted, not zero-width, not a step. A step means the command goes
    from full authority to nothing between two ticks."""
    full, zero = _fresh_bounds(1.0 / hz, pipe)
    assert 0.0 < full < zero, (name, full, zero)
    mid = _freshness((full + zero) / 2.0, 1.0 / hz, pipe)
    assert 0.0 < mid < 1.0, f'{name}: no ramp, authority jumps'


def test_nothing_observed_yet_gives_the_floors():
    """The first frame, and every caller that tracks neither quantity. This is
    the behaviour that shipped before scaling existed."""
    assert _fresh_bounds() == (VISION_FRESH_FULL_S, VISION_FRESH_ZERO_S)
    assert _fresh_bounds(0.0, 0.0) == (VISION_FRESH_FULL_S, VISION_FRESH_ZERO_S)


@pytest.mark.parametrize('interval,pipe', [
    (-1.0, -1.0), (0.0, -5.0), (-0.001, 0.02), (1e9, 1e9), (0.0, 1e9),
])
def test_hostile_inputs_still_yield_a_usable_ramp(interval, pipe):
    """A stalled detector, a clock step, a sensor that vanishes. None of these
    may produce an inverted ramp or an unbounded blind-drive window -- the
    control loop divides by (zero - full)."""
    full, zero = _fresh_bounds(interval, pipe)
    assert 0.0 < full < zero <= VISION_FRESH_ZERO_MAX_S
    assert _freshness(0.0, interval, pipe) == 1.0
    assert _freshness(10.0, interval, pipe) == 0.0


def test_a_slower_camera_never_gets_a_TIGHTER_window():
    """Monotonicity. If a slower sensor derived a tighter threshold, adding a
    worse camera would make the vehicle stop sooner -- backwards."""
    prev_full = prev_zero = 0.0
    for hz in (200.0, 77.0, 30.0, 15.0, 5.0):
        full, zero = _fresh_bounds(1.0 / hz, 0.5 / hz)
        assert full >= prev_full - 1e-9, hz
        assert zero >= prev_zero - 1e-9, hz
        prev_full, prev_zero = full, zero


def test_a_slow_pipeline_does_NOT_earn_full_authority():
    """The failure mode derivation creates, and the reason `full` is capped in
    ABSOLUTE time.

    Feed the rule a pipeline that is always 250 ms late and, left to derive,
    it concludes "250 ms is normal here" and grants full authority. That is
    exactly wrong: 250 ms is ~16 cm of travel at this hull's 0.65 m/s cruise
    whatever the reason it is old. A slow sensor must run at REDUCED
    authority -- the safe answer -- with `_warn_low_fps` telling the operator
    why, rather than the vehicle confidently steering on stale pixels.

    Caught by three integration tests that were asserting decay at 300 ms and
    stopped seeing it once the thresholds became derived.
    """
    name, hz, pipe, worst = _SLOW
    full, zero = _fresh_bounds(1.0 / hz, pipe)
    assert full <= VISION_FRESH_FULL_MAX_S
    assert _freshness(worst, 1.0 / hz, pipe) < 1.0, (
        'a 400 ms-old sample must not command full authority')
    assert zero <= VISION_FRESH_ZERO_MAX_S


def test_the_rule_is_never_LOOSER_than_the_fixed_constants_it_replaced():
    """Scaling must not have quietly widened the window anywhere. The old
    behaviour was a flat full=0.10; the ceiling holds that as a hard bound for
    every sensor, real or hypothetical."""
    for _n, hz, pipe, _w in _MEASURED + _HYPOTHETICAL + [_SLOW]:
        full, _zero = _fresh_bounds(1.0 / hz, pipe)
        assert full <= 0.10 + 1e-9, (_n, full)

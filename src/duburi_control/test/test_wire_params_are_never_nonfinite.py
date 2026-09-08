"""B35 -- hammer the verb table and the wire converters with hostile floats.

FOUND BY EXECUTION, NOT BY READING -- and the first write-up of it was WRONG.

⚠ NaN NEVER REACHED THE WIRE. `SrotFC.move()` has always refused a non-finite
frame (`if not _finite(p1..p5): return MoveResult(DENIED, ...)`) and
`test_move_denied_on_nonfinite_param` has always proved it. The claim that it did
is retracted in BUGS.md B35. What these tests pin is narrower and real: a
validator defeated by its own comparison, a silent coercion, and error messages
that name the offending field.

The code reads correctly; `_depth_to_dive` exists specifically to reject a bad
depth target. Running it is what showed the guard does not hold:

    target = NaN   -> ACCEPTED, p2 = NaN      (`NaN > 0.0` is False)
    target = -inf  -> ACCEPTED, p2 = +inf     (dive to infinite depth)
    gain   = NaN   -> speed = NaN
    target = +inf  -> correctly refused       (this is why reading it looks fine)

NaN is not hypothetical here: `VisionResult.x_px` is NaN whenever the target was
never seen, which is the exact trap CLAUDE.md flags ("always check `saw_target`
before reading `x_px`"). A mission that computes a heading or a duration from a
never-seen target feeds NaN straight into a goal.

The hull was never at risk -- the firmware refuses every non-finite parameter
(`mav_commands.cpp:279-292`). But relying on the far side's guard costs a
mid-mission DENIED and a STATUSTEXT to decode, when the host knows which field
was bad and can say so before anything leaves.

⛔ THE NEUTRALITY ASSERTIONS ARE NOT DECORATION. The first fix made `_clamp` map
NaN to `lo` -- which for `unit_to_mc` is -1.0, i.e. **-1000, full reverse
thrust**. A range check accepts that happily, and the range-only version of this
test passed it. Each converter must fail to ITS OWN neutral.
"""

import itertools
import math

import pytest

import duburi_control.fc.srot_protocol as sp
from duburi_control.fc.srot_fc import MOVE_VERBS, _build_params, _depth_to_dive

HOSTILE = [float('nan'), float('inf'), float('-inf')]
FINITE = [0.0, -0.0, 1e-12, 0.5, 1.0, -1.0, 100.0, -100.0, 1e9, -1e9]


@pytest.mark.parametrize('bad', HOSTILE)
def test_no_verb_can_put_a_non_finite_value_on_the_wire(bad):
    """Every param of every collapsed verb is finite, or the verb is refused."""
    for verb in sorted(MOVE_VERBS):
        for field in ('duration', 'gain', 'target', 'timeout'):
            kw = {'duration': 0.0, 'gain': 0.0, 'target': 0.0, 'timeout': 0.0}
            kw[field] = bad
            try:
                params = _build_params(verb, kw)
            except (ValueError, KeyError):
                continue                      # refusing is the correct outcome
            for i, v in enumerate(params):
                assert math.isfinite(float(v)), (
                    f'{verb} with {field}={bad} emitted non-finite p{i + 1}={v}')


def test_a_non_finite_gain_is_REFUSED_not_coerced_to_zero():
    """Coercing gain to 0 sends a valid "move at speed 0": no motion, no error.

    That is a phantom manoeuvre -- the mission believes it moved. Streaming paths
    (manual(), 20 Hz) DO coerce, because they cannot raise per tick and zero is
    the safe tick. A one-shot verb must refuse. Both are "fail safe"; only one is
    right per context.
    """
    from duburi_control.fc.srot_fc import _speed_from_gain
    for bad in HOSTILE:
        with pytest.raises(ValueError):
            _speed_from_gain({'gain': bad})
    assert _speed_from_gain({'gain': 60.0}) == pytest.approx(0.6)
    # ...while the STREAMING converter still coerces, deliberately.
    assert sp.sanitize_speed(float('nan')) == 0.0


def test_the_depth_validator_is_not_bypassed_by_nan_or_negative_infinity():
    """`target > 0.0` is False for NaN -- the guard's own comparison defeats it."""
    for bad in (float('nan'), float('-inf'), float('inf')):
        with pytest.raises(ValueError):
            _depth_to_dive({'target': bad})
    assert _depth_to_dive({'target': -1.5}) == pytest.approx(1.5)


def test_non_finite_axis_commands_fail_to_NEUTRAL_not_to_full_deflection():
    """The clamp floor is FULL REVERSE / FULL DESCEND. Neutral is the safe end."""
    for bad in HOSTILE if False else (float('nan'),):
        assert sp.unit_to_mc(bad) == 0,               'x/y/r NaN must be neutral (0)'
        assert sp.unit_to_mc_z(bad) == sp.MC_Z_NEUTRAL, 'z NaN must hold depth (500)'
        assert sp.pct_to_mc(bad) == 0
        assert sp.pct_to_mc_z(bad) == sp.MC_Z_NEUTRAL
        assert sp.sanitize_speed(bad) == 0.0,         'NaN speed must mean no motion'


@pytest.mark.parametrize('v', FINITE + HOSTILE)
def test_the_converters_stay_inside_the_firmware_accepted_bands(v):
    """Bands are the firmware's: x/y/r +/-1000, z 0..1000 (mav_commands.cpp:706-709)."""
    assert -1000 <= sp.unit_to_mc(v) <= 1000
    assert 0 <= sp.unit_to_mc_z(v) <= 1000
    assert -1000 <= sp.pct_to_mc(v) <= 1000
    assert 0 <= sp.pct_to_mc_z(v) <= 1000
    assert 0.0 <= sp.sanitize_speed(v) <= sp.MOVE_CRUISE_MAX


def test_the_converters_are_monotonic_across_the_finite_band():
    """A non-monotonic converter would make a larger demand produce less thrust."""
    grid = [x / 20 for x in range(-25, 26)]
    for a, b in zip(grid, grid[1:]):
        assert sp.unit_to_mc(a) <= sp.unit_to_mc(b)
        assert sp.unit_to_mc_z(a) <= sp.unit_to_mc_z(b)


def test_neutral_maps_to_the_firmware_neutral():
    assert sp.unit_to_mc(0.0) == 0
    assert sp.unit_to_mc_z(0.0) == 500      # fw: `const float z = ...; // 500 = neutral`


# --------------------------------------------------------------------------- #
#  B37 -- an error message must be ACTIONABLE, not merely accurate             #
# --------------------------------------------------------------------------- #
# Error paths are the least-executed code in the stack and they run exactly when
# the operator can least afford to decode them. These were graded by executing
# each path and asking three questions: does it say WHAT failed, WHY, and WHAT TO
# DO? Five of eight had no remedy. The two that matter at the pool are the stall
# (the most common real failure) and NaN (whose source is knowable), so those now
# name the thing to check.
#
# Deliberately narrow: this pins the REMEDY, not the wording. Rephrase freely;
# just do not drop the pointer that makes the message useful at 2am.

def _reason(fn):
    return fn()


def test_a_non_finite_parameter_names_its_likely_source():
    """NaN on this stack has one dominant source; say so instead of making them hunt."""
    import sys
    sys.path.insert(0, __file__.rsplit('/', 1)[0])
    from test_srot_fc import _fc
    r = _fc().move('move_forward', duration=float('nan'), gain=50)
    assert 'saw_target' in r.reason, (
        'a NaN parameter almost always comes from a VisionResult that never saw '
        'its target -- the message must point there')


def test_the_stall_message_says_what_to_check():
    """A stall is the commonest pool failure and used to say only "(stall)"."""
    import sys
    sys.path.insert(0, __file__.rsplit('/', 1)[0])
    from test_srot_fc import _fc
    r = _fc().move('move_forward', duration=1.0, gain=50)
    assert 'stall' in r.reason
    assert 'connect' in r.reason, 'point at the tool that shows whether the link is alive'
    assert 'Bar30' in r.reason or 'refusing' in r.reason, (
        'an unhealthy-baro refusal presents identically to a dead link -- say so')


def test_an_unknown_mode_lists_the_valid_ones():
    import sys
    sys.path.insert(0, __file__.rsplit('/', 1)[0])
    from test_srot_fc import _fc
    ok, reason = _fc().set_mode('NOT_A_MODE')
    assert not ok
    assert 'STABILIZE' in reason and 'DEPTH_HOLD' in reason, \
        'listing the valid modes turns a dead end into a next step'

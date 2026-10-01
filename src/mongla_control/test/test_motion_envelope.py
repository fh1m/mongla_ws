"""The host-side motion envelope (issue #15).

`_build_params` checked finiteness only: -50 m went to the board as a dive to
50 m, 3600 as ten turns, -30 as a negative duration. Each bound below catches a
UNITS BUG or a TYPO and restricts no real request; the reasons live beside the
constants in srot_fc.py.
"""
import pytest

import mongla_control.fc.srot_fc as mod
from mongla_control.fc.srot_fc import _build_params


@pytest.mark.parametrize('verb, kw, why', [
    ('set_depth', {'target': -50.0}, 'deeper than'),          # cm typed as m
    ('yaw_right', {'target': 3600.0}, 'more than one revolution'),
    ('turn', {'target': 720.0}, 'more than one revolution'),
    ('move_forward', {'duration': -30.0, 'gain': 40}, 'negative duration'),
    ('move_forward', {'duration': 3000.0, 'gain': 40}, 'past the'),
    ('set_depth', {'target': -1.0, 'timeout': -5.0}, 'negative timeout'),
])
def test_a_units_bug_is_refused_with_its_reason(verb, kw, why):
    with pytest.raises(ValueError, match=why):
        _build_params(verb, kw)


@pytest.mark.parametrize('verb, kw', [
    ('set_depth', {'target': -5.0}),           # TRANSDEC
    ('set_depth', {'target': -1.6}),           # SAUVC deep end
    ('yaw_left', {'target': 360.0}),
    ('turn', {'target': -180.0}),
    ('move_forward', {'duration': 120.0, 'gain': 40}),
])
def test_every_real_request_still_passes(verb, kw):
    _build_params(verb, kw)


def test_the_bounds_are_not_tuning_values():
    """If someone tightens these into tuning, a real pool gets refused."""
    assert mod.MAX_DEPTH_M >= 5.0          # TRANSDEC must fit
    assert mod.MAX_TURN_DEG == 360.0
    assert mod.MAX_DURATION_S >= 300.0


def test_an_unlocked_turn_is_WARNED_not_refused():
    """`turn(head() + step)` is right in the board's own frame, locked or not."""
    from test_srot_fc import _fc
    fc = _fc()
    warned = []
    fc._log_warn = warned.append
    fc.check_yaw_reference = lambda: (False, 'yaw reference NOT LOCKED: SAMPLING')
    fc._relay_move_ack = lambda *a, **k: mod.MoveResult(mod.SUCCEEDED, 'ok')
    res = fc.move('turn', target=90.0)
    assert res.code == mod.SUCCEEDED
    assert warned and 'NOT LOCKED' in warned[0] and 'compass heading' in warned[0]

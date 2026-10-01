"""An explicit 0.0 is a zero, not "unset" (issue #14).

A float field defaults to 0.0, so `fields_for` could not tell "the mission asked
for zero" from "the mission said nothing" and swapped in the default:
`move_forward(gain=0)` drove at 80 % thrust toward the prop it had just centred
on, and `turn` with no target turned to NORTH. The client now lists the fields
it set (`Move.Goal.set_fields`); that list decides.
"""
import pytest

from mongla_control.commands import fields_for
from mongla_interfaces.action import Move


def _goal(**kw):
    g = Move.Goal()
    for k, v in kw.items():
        setattr(g, k, v)
    g.set_fields = list(kw)
    return g


def test_an_explicit_zero_gain_is_zero_not_eighty_percent():
    """THE issue's verification: today this was 80.0."""
    assert fields_for('move_forward', _goal(duration=1.0, gain=0.0))['gain'] == 0.0


def test_an_unset_gain_still_takes_its_default():
    assert fields_for('move_forward', _goal(duration=1.0))['gain'] == 80.0


def test_a_turn_with_no_target_is_REFUSED_not_sent_north():
    with pytest.raises(ValueError, match="'target' not set"):
        fields_for('turn', _goal(timeout=10.0))


def test_a_turn_to_north_is_still_possible():
    assert fields_for('turn', _goal(target=0.0))['target'] == 0.0


def test_set_depth_with_no_target_is_refused_not_surfaced():
    with pytest.raises(ValueError):
        fields_for('set_depth', _goal(timeout=10.0))


def test_a_runtime_default_never_overrides_an_explicit_zero():
    """`vision.*` deck params used to replace an explicit 0 ('yaw gain 0')."""
    k = fields_for('vision_align', _goal(gain=0.0),
                   runtime_defaults={'gain': 30.0})
    assert k['gain'] == 0.0


def test_a_legacy_sender_keeps_the_old_rule():
    """No `set_fields` = an old client; behaviour unchanged, by design."""
    g = Move.Goal()
    g.duration, g.gain = 1.0, 0.0
    assert fields_for('move_forward', g)['gain'] == 80.0


def test_the_client_lists_exactly_what_it_was_given():
    import pathlib
    src = (pathlib.Path(__file__).resolve().parents[2] / 'mongla_planner'
           / 'mongla_planner' / 'client.py').read_text(encoding='utf-8')
    assert 'goal.set_fields = given' in src and 'given.append(name)' in src

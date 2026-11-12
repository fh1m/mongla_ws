"""A browser's verb reaches the vehicle only as a goal the verb itself would accept.

The helm is reachable by anything on the team network, so what it builds from
a request's JSON is the whole of the check between a typed field and the
thrusters: the verb must exist, each field must be that verb's own, and each
value must be the type the action declares.
"""
import pytest

from mongla_batighor.helm import build_goal
from mongla_control.commands import COMMANDS


def test_a_verb_with_its_own_fields_becomes_a_goal_that_lists_them():
    goal, why = build_goal('move_forward', '{"duration": 2, "gain": 0}')
    assert why == '' and goal.cmd == 'move_forward'
    assert goal.duration == 2.0 and goal.gain == 0.0
    # The typed 0 is a real zero, not "unset" (Move.action, issue #14).
    assert sorted(goal.set_fields) == ['duration', 'gain']


def test_no_fields_means_none_given():
    goal, why = build_goal('stop', '')
    assert goal is not None and why == '' and list(goal.set_fields) == []


@pytest.mark.parametrize('cmd, fields, says', [
    ('move_fwd', '{}', 'unknown verb'),
    ('move_forward', '{"target": 3}', 'has no field'),            # another verb's field
    ('move_forward', '{"duration": "2"}', 'must be a number'),
    ('move_forward', '{"duration": true}', 'must be a number'),
    ('move_forward', '{"duration": NaN}', 'must be a number'),
    ('move_forward', '[1, 2]', 'JSON object'),
    ('move_forward', '{duration: 2}', 'not JSON'),
])
def test_anything_else_is_refused_with_a_reason(cmd, fields, says):
    goal, why = build_goal(cmd, fields)
    assert goal is None and says in why, why


def test_every_verb_and_field_in_the_registry_can_be_sent():
    """A field the registry lists but the helm cannot convert would make that
    verb undrivable from a browser, silently."""
    from mongla_interfaces.action import Move
    types = Move.Goal.get_fields_and_field_types()
    sample = {'float': 1.0, 'double': 1.0, 'boolean': True, 'string': 'x'}
    import json
    for cmd, row in COMMANDS.items():
        fields = {f: sample[types[f]] for f in row['fields']}
        goal, why = build_goal(cmd, json.dumps(fields))
        assert goal is not None, (cmd, why)

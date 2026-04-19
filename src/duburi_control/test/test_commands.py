"""Static checks on the COMMANDS registry.

If a row references a field that isn't on Move.Goal, the dispatcher
will explode at runtime -- catch it at unit-test time.
"""

from duburi_interfaces.action import Move

from duburi_control.commands import COMMANDS, STRING_FIELDS, fields_for


# All Move.Goal field names the COMMANDS registry is allowed to use.
GOAL_FIELDS = {'duration', 'gain', 'target', 'target_name', 'timeout',
               'settle', 'yaw_rate_pct'}


def test_every_field_is_on_move_goal():
    for cmd, spec in COMMANDS.items():
        for field in spec['fields']:
            assert field in GOAL_FIELDS, (
                f'{cmd!r} declares unknown Move.Goal field {field!r}')


def test_defaults_are_subset_of_fields():
    for cmd, spec in COMMANDS.items():
        for field in spec['defaults']:
            assert field in spec['fields'], (
                f'{cmd!r} has default for non-listed field {field!r}')


def test_fields_for_substitutes_default_when_unset():
    goal = Move.Goal()
    goal.cmd = 'move_forward'
    goal.duration = 5.0
    # gain + settle left as rosidl default 0.0 -> dispatcher should fill defaults
    kwargs = fields_for('move_forward', goal)
    assert kwargs == {'duration': 5.0, 'gain': 80.0, 'settle': 0.0}


def test_arc_command_has_yaw_rate_field():
    """`arc` is the only command that reads yaw_rate_pct."""
    spec = COMMANDS['arc']
    assert 'yaw_rate_pct' in spec['fields']
    assert spec['defaults']['yaw_rate_pct'] == 30.0


def test_lock_unlock_heading_registered():
    """heading-lock pair must round-trip through the registry."""
    assert 'lock_heading'   in COMMANDS
    assert 'unlock_heading' in COMMANDS
    assert COMMANDS['unlock_heading']['fields'] == []
    assert COMMANDS['lock_heading']['defaults']['timeout'] == 300.0


def test_fields_for_keeps_explicit_zero_when_no_default():
    """`set_depth` has no default for `target`; an explicit 0.0 must
    survive (hold at surface is a valid command)."""
    goal = Move.Goal()
    goal.cmd = 'set_depth'
    goal.target = 0.0
    goal.timeout = 0.0   # rosidl-zero -> default 30.0 substituted
    kwargs = fields_for('set_depth', goal)
    assert kwargs == {'target': 0.0, 'timeout': 30.0, 'settle': 0.0}


def test_fields_for_string_field_uses_default_when_empty():
    goal = Move.Goal()
    goal.cmd = 'set_mode'
    goal.target_name = 'ALT_HOLD'
    # timeout 0.0 -> 8.0
    kwargs = fields_for('set_mode', goal)
    assert kwargs == {'target_name': 'ALT_HOLD', 'timeout': 8.0}


def test_string_fields_constant_matches_actual_string_typed_fields():
    """STRING_FIELDS controls how `fields_for` detects "unset". If a
    new string field is added to Move.Goal, it must be added here too,
    or numeric defaulting will misfire on it."""
    assert 'target_name' in STRING_FIELDS

"""Every COMMANDS field must exist on Move.Goal at the wire level.

A field listed in commands.py but missing from Move.action crashes the CLI and
the action client when they build the goal -- the 'steps'/'flips'/'headroom'
class of pool-day crash. Skipped when mongla_interfaces is not built yet.
"""
import importlib.util

import pytest

from mongla_control.commands import COMMANDS

pytestmark = pytest.mark.skipif(
    importlib.util.find_spec('mongla_interfaces') is None,
    reason='mongla_interfaces not installed -- colcon build --packages-select mongla_interfaces')


def test_every_command_field_exists_on_move_goal():
    from mongla_interfaces.action import Move
    goal = Move.Goal()
    missing = [f"'{cmd}': field '{field}'"
               for cmd, spec in COMMANDS.items()
               for field in spec['fields'] if not hasattr(goal, field)]
    assert not missing, ('Fields in COMMANDS missing on Move.Goal:\n  '
                         + '\n  '.join(missing)
                         + '\nAdd them to Move.action and rebuild mongla_interfaces.')


def test_every_goal_field_is_used_by_some_command():
    """The other direction: a Move.Goal field no verb reads is dead wire."""
    from mongla_interfaces.action import Move
    # `cmd` names the verb and `set_fields` says which fields the caller set
    # (so an explicit 0 is not mistaken for "unset", issue #14): metadata about
    # the goal, read by the dispatcher rather than by any verb.
    used = {f for spec in COMMANDS.values() for f in spec['fields']} | {'cmd', 'set_fields'}
    unused = sorted(set(Move.Goal.get_fields_and_field_types()) - used)
    assert not unused, f'Move.Goal fields no command reads: {unused}'

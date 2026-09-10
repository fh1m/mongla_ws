"""`STRING_FIELDS` / `BOOL_FIELDS` mirror `Move.action` and must not drift.

⛔ THE DEFECT THIS EXISTS FOR, found by running the CLI rather than reading it.
`tool` was added to the action (as `string`), to the command spec, to the verb,
to the loop and to the DSL -- and the `--tool` flag still died with

    duburi vision_align: error: argument --tool: invalid _float_or_head value: 'torpedo'

because the CLI infers a flag's type from `STRING_FIELDS`, a HAND-MAINTAINED
copy of a type the action file already declares. Every static "is it wired"
check passed: the field existed in all five places. The one thing that failed
was using it.

The wire contract is the truth. This derives the expectation from it.
"""
import pathlib
import re

import pytest

from duburi_control.commands import BOOL_FIELDS, COMMANDS, STRING_FIELDS

_ACTION = (pathlib.Path(__file__).resolve().parents[2] / 'duburi_interfaces'
           / 'action' / 'Move.action')


def _declared():
    """{field: rosidl type} from the goal section of the action file."""
    out = {}
    for line in _ACTION.read_text().splitlines():
        line = line.split('#', 1)[0].strip()
        if line.startswith('---'):
            break                      # goal section ends at the first divider
        parts = line.split()
        if len(parts) == 2 and re.match(r'^(string|float\d*|u?int\d*|bool)$', parts[0]):
            out[parts[1]] = parts[0]
    return out


def _fields_in_use():
    used = set()
    for spec in COMMANDS.values():
        used |= set(spec.get('fields', []))
    return used


def test_the_action_actually_parses():
    d = _declared()
    assert len(d) > 20, f'only parsed {len(d)} goal fields -- parser broke'
    assert d.get('camera') == 'string' and d.get('gain', '').startswith('float')


@pytest.mark.parametrize('field', sorted(_fields_in_use()))
def test_every_string_field_in_use_is_declared_as_one(field):
    declared = _declared().get(field)
    if declared != 'string':
        return
    assert field in STRING_FIELDS, (
        f'{field!r} is `string` in Move.action but missing from STRING_FIELDS, '
        f'so the CLI types --{field} as a float and the flag is unusable. '
        f'This passes every "is it wired" check and fails on first use.')


@pytest.mark.parametrize('field', sorted(_fields_in_use()))
def test_every_bool_field_in_use_is_declared_as_one(field):
    if _declared().get(field) != 'bool':
        return
    assert field in BOOL_FIELDS, (
        f'{field!r} is `bool` in Move.action but missing from BOOL_FIELDS')


def test_no_string_field_is_secretly_numeric():
    """The other direction: a name in STRING_FIELDS that the action declares as
    a float would make the CLI hand a string to a numeric wire field."""
    d = _declared()
    wrong = [f for f in STRING_FIELDS if f in d and d[f] != 'string']
    assert not wrong, f'{wrong} are in STRING_FIELDS but not `string` in the action'


def test_tool_specifically_is_a_string():
    """Named because it is the one that was broken, and the symptom is a flag
    nobody can use rather than anything that looks like a fault."""
    assert _declared().get('tool') == 'string'
    assert 'tool' in STRING_FIELDS

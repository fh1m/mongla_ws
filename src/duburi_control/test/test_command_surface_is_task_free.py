"""No verb and no field may be named for a competition task.

THE PRINCIPLE, stated so it survives the next person in a hurry: this stack is a
set of AUV CAPABILITIES that missions compose, not a set of task solves. A verb
called `torpedo_align` or a field called `gate_width` is a capability that can
only ever be used once, and it quietly makes the next competition a rewrite
instead of a config change -- which is the whole argument for SAUVC/RoboSub/TAC
portability.

Audited 2026-09-10: **zero** task words across all 30 verbs and every field.
This test is what keeps that true, because the pressure to add one arrives
exactly when someone is busy shipping a task.

`fire` is deliberately NOT on the list. It means "actuate board channel N" --
the number IS the board's `SERVO{n}_ROLE` (1..16, no host-side map) -- and it
drives a dropper, a torpedo launcher or a grabber release identically. It names
an actuator, not a task.
"""
import re

import pytest

from duburi_control.commands import COMMANDS

# Competition props and task names, from the SAUVC and RoboSub rulebooks. If a
# NEW prop appears, add it here -- the list existing is the point.
_TASK_WORDS = re.compile(
    r'torpedo|slalom|flare|drum|octagon|prequal|blood|rescue|repair|helipad|'
    r'\bbin\b|\bgate\b|\bdropper\b|\bpinger\b|\bshark\b|\bfish\b|\btrash\b',
    re.I)


def test_the_audit_actually_has_verbs_to_check():
    """A guard that silently checks nothing is worse than no guard."""
    assert len(COMMANDS) >= 25, f'only {len(COMMANDS)} verbs found -- registry moved?'


@pytest.mark.parametrize('verb', sorted(COMMANDS))
def test_no_verb_is_named_for_a_task(verb):
    m = _TASK_WORDS.search(verb)
    assert not m, (
        f'verb {verb!r} is named for {m.group(0)!r}. Name the CAPABILITY, not '
        f'the prop it was first used on -- a task-named verb can only be used '
        f'once, and the next competition becomes a rewrite instead of a config '
        f'change.')


@pytest.mark.parametrize('verb', sorted(COMMANDS))
def test_no_field_is_named_for_a_task(verb):
    bad = [f for f in COMMANDS[verb].get('fields', []) if _TASK_WORDS.search(f)]
    assert not bad, (
        f'{verb} has task-named field(s) {bad}. Same argument as the verb: the '
        f'field describes a geometry or a tolerance, so name it that way.')


def test_fire_is_documented_as_generic_actuation():
    """`fire` reads as torpedo-flavoured and is not. Keeping it is a decision,
    so the reasoning has to live where a reader meets the name."""
    import pathlib
    act = (pathlib.Path(__file__).resolve().parents[2] / 'duburi_interfaces'
           / 'action' / 'Move.action').read_text()
    i = act.index('fire_channels')
    why = act[max(0, i - 900):i]
    assert 'NOT a torpedo verb' in why, (
        'the action does not say that `fire` means actuate-a-board-channel, so '
        'the next reader will reasonably assume it is task-specific')

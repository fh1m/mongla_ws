"""Every tunable a vision loop accepts must actually be PASSED to it.

⛔ THE DEFECT THIS EXISTS FOR. `align_loop` and `move_loop` have accepted
`lock_s` since the lock ladder was built, and **no caller ever passed it**. It
kept its `0.0` default, `bbox_error(lock_s=0.0)` never consulted the follower
or the anchor, and the ladder was therefore unreachable from any mission --
while a deck parameter (`vision.lock_s`), a launch argument (`lock`), an
action-doc paragraph and a staging note all described how to switch it on.
Raising the deck default would have changed nothing at all.

That is the widest version of this package's recurring defect: not a value that
is wrong, but a path that does not exist, described everywhere as if it did.

The guard is structural -- compare the loop's keyword-only tunables against the
keywords the verb actually hands it -- so it catches the NEXT one too.
"""
import ast
import pathlib

import pytest

_PKG = pathlib.Path(__file__).resolve().parents[1] / 'duburi_control'
_MV = ast.parse((_PKG / 'motion_vision.py').read_text())
_VV_SRC = (_PKG / 'vision_verbs.py').read_text()
_VV = ast.parse(_VV_SRC)

# WHAT MAKES AN UNPASSED ARGUMENT A DEFECT. Not simply being unpassed -- some
# loop arguments are injection seams with a module-constant default and no
# outside surface at all (`i_lat_max` is one: used once to clamp the integral,
# named in no deck param, no action field and no doc). Those are fine.
#
# The defect is an argument the REST OF THE STACK ADVERTISES AS SETTABLE and
# the verb never forwards, because then a deck parameter, a launch flag or an
# action field describes a path that does not exist. That is what `lock_s` was.
# Deriving the rule from the advertised surface means it keeps working as
# tunables are added, instead of resting on a hand-maintained allowlist.


def _advertised():
    """Names the stack offers as settable: deck params and action fields."""
    import sys
    sys.path.insert(0, str(_PKG.parents[1] / 'duburi_manager'))
    from duburi_manager.vision_tunables import VISION_PARAM_DEFAULTS
    deck = {k.split('.', 1)[1] for k in VISION_PARAM_DEFAULTS if k.startswith('vision.')}
    action = _PKG.parents[1] / 'duburi_interfaces' / 'action' / 'Move.action'
    fields = set()
    if action.exists():
        for line in action.read_text().splitlines():
            parts = line.split('#', 1)[0].split()
            if len(parts) == 2 and parts[0].startswith(('float', 'int', 'bool', 'string')):
                fields.add(parts[1])
    return deck | fields


def _loop_params(name):
    for n in _MV.body:
        if isinstance(n, ast.FunctionDef) and n.name == name:
            a = n.args
            return {p.arg for p in list(a.args) + list(a.kwonlyargs)} - {'self'}
    raise AssertionError(f'{name} not found in motion_vision')


def _passed_keywords(call_name):
    for n in ast.walk(_VV):
        if (isinstance(n, ast.Call)
                and getattr(n.func, 'id', '') == call_name):
            return {k.arg for k in n.keywords if k.arg}
    raise AssertionError(f'no call to {call_name} in vision_verbs')


@pytest.mark.parametrize('loop', ['align_loop', 'move_loop'])
def test_every_tunable_the_loop_accepts_is_actually_passed(loop):
    accepted = _loop_params(loop)
    passed = _passed_keywords(loop)
    advertised = _advertised()
    missing = sorted((accepted & advertised) - passed)
    assert not missing, (
        f'{loop} accepts {missing}, the stack advertises it as settable (deck '
        f'param or action field), and vision_verbs never passes it. The '
        f'argument silently keeps its default, so everything describing how to '
        f'switch it on is describing a path that does not exist.')


def test_lock_s_specifically_reaches_both_loops():
    """Named because it is the one that was dead, and a regression here is
    invisible: the ladder simply never answers."""
    for loop in ('align_loop', 'move_loop'):
        assert 'lock_s' in _passed_keywords(loop), (
            f'{loop} is called without lock_s -- the ladder is unreachable again')
    assert _VV_SRC.count('lock_s=float(lock_s)') == 2

"""A mission may only steer on a class the model IT LOADS actually emits.

⛔ THE DEFECT THIS CLOSES, AND IT SURVIVED AN 85-FILE SWEEP AND A GREEN SUITE.
Retiring `gate_rescue_repair` renamed the model everywhere and renamed the
class list in most places — but `task_gate.py` and `pool_day_practice.py` kept
`set_classes('rescue,repair')` and `vision.align('rescue', ...)` further down.
Both missions RUN, and `task_full_2026` chunks `task_gate`.

⛔⛔ IT FAILS SILENTLY, WHICH IS WHY A TEST IS THE ONLY DEFENCE.
`detection/hailo.py` logs an ERROR and **ignores** an allowlist whose names are
all unknown, so the detector keeps emitting its real classes. `align('rescue')`
then never matches, `fallback=creep_forward` thrusts **open loop for the full
duration**, the verb returns normally, and the mission reports success. That is
the "reports success while the vehicle does nothing" failure CLAUDE.md section
8.6 names as the one that ends runs.

⭐ IT FOLLOWS `set_model`, NOT A GLOBAL UNION, and that is the whole design.
Checking against every class in every sidecar is too weak to have caught this:
`octagon.yaml` legitimately lists `rescue`, so the union says the class exists
and the mission looks fine while loading a graph that never emits it.

⚠ AND THIS MATTERS BECAUSE MODELS CHANGE DAILY during water testing. The
shipped weights are development harnesses; the real ones arrive on the day and
get swapped. So the check keys on the ONE line a mission already writes —
`set_model(...)` — and reads that model's sidecar. **Swapping a model means
editing that line and nothing else.** A test that hard-coded the expected
classes would itself become a file to edit on every swap, which is the defect
it is meant to prevent.
"""
from __future__ import annotations

import ast
import pathlib

import pytest
import yaml

_ROOT = pathlib.Path(__file__).resolve().parents[3]
_MISSIONS = _ROOT / 'src' / 'mongla_planner' / 'mongla_planner' / 'missions'
_MODELS = _ROOT / 'src' / 'mongla_vision' / 'models'

# Verbs whose FIRST positional string argument is a class name.
_CLASS_ARG_VERBS = {'align', 'move', 'detected', 'wait_for', 'anchor_snap',
                    'lock_class'}


def classes_of(stem: str):
    """The classes one model emits, or None when no sidecar ships for it.

    None is not a failure: a mission may name a model whose weights arrive on
    the day. It means "cannot check", and the test says so rather than
    inventing a verdict.
    """
    p = _MODELS / f'{stem}.yaml'
    if not p.is_file():
        return None
    try:
        d = yaml.safe_load(p.read_text()) or {}
    except Exception:                                            # noqa: BLE001
        return None
    return {str(v).strip().lower() for v in (d.get('names') or {}).values()}


def _kw(call, name):
    for k in call.keywords:
        if k.arg == name:
            if isinstance(k.value, ast.Constant):
                return str(k.value.value)
            return getattr(k.value, 'id', '?')      # a module-level _FWD/_DWN
    return None


def _steered_classes_by_model(path):
    """[(model_stem, {classes named under it}, lineno)].

    ⛔ KEYED ON THE NODE, not on program order, and that was a real bug in the
    first version of this test. A dual-camera mission runs TWO detectors:
    `set_model('gate_sharks', node=_FWD)` and
    `set_model('bin_fire_blood', node=_DWN)` are both live at once, so a
    sequential walk credited `align('fire', camera='downward')` to whichever
    model was set last and reported two false positives on real missions.
    A check that cries wolf on correct code gets switched off.

    `camera=` on a verb and `node=` on `set_model` both select the detector;
    they are matched by their tail ('forward' / '_FWD' both end in the same
    word) because the missions use a module constant for one and a literal for
    the other.
    """
    try:
        tree = ast.parse(path.read_text())
    except SyntaxError:
        return []

    def slot(tag):
        t = (tag or '').lower()
        if 'dwn' in t or 'down' in t:
            return 'downward'
        if 'fwd' in t or 'forward' in t:
            return 'forward'
        return 'default'

    models, named = {}, {}
    for n in ast.walk(tree):
        if not isinstance(n, ast.Call):
            continue
        fn = n.func
        name = fn.attr if isinstance(fn, ast.Attribute) else getattr(fn, 'id', '')
        a0 = n.args[0] if n.args else None
        lit = (a0.value.strip() if isinstance(a0, ast.Constant)
               and isinstance(a0.value, str) else None)
        if lit is None:
            continue
        if name == 'set_model':
            models[slot(_kw(n, 'node'))] = (lit, n.lineno)
        elif name == 'set_classes':
            s = slot(_kw(n, 'node'))
            named.setdefault(s, set()).update(
                x.strip().lower() for x in lit.split(',') if x.strip())
        elif name in _CLASS_ARG_VERBS:
            s = slot(_kw(n, 'camera'))
            named.setdefault(s, set()).add(lit.lower())

    out = []
    for s, (model, line) in models.items():
        cls = named.get(s, set())
        # A verb with no camera= lands in 'default'; attribute it to the only
        # model when the mission loads exactly one, and skip it otherwise
        # rather than guess.
        if len(models) == 1:
            cls = cls | named.get('default', set())
        out.append((model, cls, line))
    return out


def _missions():
    return sorted(p for p in _MISSIONS.glob('*.py')
                  if not p.name.startswith('_'))


def test_there_are_sidecars_to_check_against():
    """A sweep that silently finds nothing is worse than no sweep."""
    got = {p.stem for p in _MODELS.glob('*.yaml')}
    assert len(got) >= 3, f'only found {got} -- did the models move?'


def test_at_least_one_mission_is_actually_checked():
    """⛔ The check is worthless if no mission pairs a `set_model` with a
    class. If this fails, the AST walk stopped matching the DSL."""
    checkable = [p.name for p in _missions()
                 if any(cs and classes_of(m) is not None
                        for m, cs, _ in _steered_classes_by_model(p))]
    assert checkable, 'no mission could be checked -- the walk is broken'


@pytest.mark.parametrize('path', _missions(), ids=lambda p: p.name)
def test_every_class_is_emitted_by_the_model_loaded_for_it(path):
    bad = []
    for model, named, line in _steered_classes_by_model(path):
        emits = classes_of(model)
        if emits is None:
            continue                 # weights arrive on the day; cannot check
        for c in sorted(named - emits):
            bad.append(f'{path.name}:{line} loads {model!r} '
                       f'(emits {sorted(emits)}) then steers on {c!r}')
    assert not bad, (
        '\n  '.join([''] + bad) +
        '\n  ⛔ This does NOT fail loudly at runtime: the detector ignores an '
        'all-unknown allowlist and keeps emitting its real classes, so the '
        'verb never matches, the fallback thrusts open loop for its full '
        'duration, and the mission reports success.\n'
        '  Fix by editing the `set_model` line, removing the phase, or '
        'shipping a sidecar whose `names:` include the class.')

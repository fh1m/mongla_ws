"""A subclass __init__ must call super().__init__.

⛔ WHAT THIS CATCHES, and it reached the vehicle. A `set_masks` method was
inserted directly after `self._want_masks = bool(masks)` -- a line of
`HailoSegDetector.__init__`. At method indentation that ENDS the constructor,
so everything after it (including `super().__init__(iou=iou, **kwargs)`) became
the body of `set_masks`. The class still imported, still parsed, and still
looked right in a diff. It failed as
`AttributeError: 'HailoSegDetector' object has no attribute '_ready'` on the
fifteenth inference, then a detector rebuild, then the process exiting 1.

Nothing caught it: the class needs a Hailo device, so no unit test constructs
it. The invariant that WOULD have caught it is the simple one -- a subclass
that defines `__init__` and never chains to its base is broken, whatever the
reason. BumblebeeAS shipped a dead node with exactly this defect.

Hardware-free, whole-package, and the opt-out list is FROZEN: a class that
genuinely must not chain has to say so here, with a reason.
"""
from __future__ import annotations

import ast
import pathlib

import pytest

_SRC = pathlib.Path(__file__).resolve().parents[1] / 'mongla_vision'

# Classes whose __init__ deliberately does not chain. Empty: every subclass of
# a class that HAS a constructor chains today. A new entry needs a reason.
_NO_CHAIN_OK: frozenset[str] = frozenset()


def _classes_with_init() -> set[str]:
    """Every class in the package that defines its own __init__.

    The check only applies when a BASE actually has a constructor to chain to.
    Subclassing an ABC that defines no `__init__` (Camera, Detector, Tracker --
    eight classes here) is ordinary Python, not a dropped chain, and a guard
    that flags it would be turned off within a day.
    """
    out: set[str] = set()
    for path in _modules():
        for cls in [n for n in ast.walk(ast.parse(path.read_text()))
                    if isinstance(n, ast.ClassDef)]:
            if any(isinstance(f, ast.FunctionDef) and f.name == '__init__'
                   for f in cls.body):
                out.add(cls.name)
    return out


def _modules() -> list[pathlib.Path]:
    return sorted(p for p in _SRC.rglob('*.py') if p.name != '__init__.py')


def _chains(fn: ast.FunctionDef) -> bool:
    for node in ast.walk(fn):
        if not isinstance(node, ast.Call):
            continue
        f = node.func
        # super().__init__(...)
        if (isinstance(f, ast.Attribute) and f.attr == '__init__'
                and isinstance(f.value, ast.Call)
                and getattr(f.value.func, 'id', '') == 'super'):
            return True
        # Base.__init__(self, ...) -- the explicit form
        if isinstance(f, ast.Attribute) and f.attr == '__init__':
            return True
    return False


_WITH_INIT = _classes_with_init()


def _offenders(tree: ast.AST) -> list[str]:
    out = []
    for cls in [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]:
        bases = [getattr(b, 'id', '') or getattr(b, 'attr', '')
                 for b in cls.bases]
        if cls.name in _NO_CHAIN_OK:
            continue
        if not any(b in _WITH_INIT for b in bases):
            continue
        for fn in cls.body:
            if isinstance(fn, ast.FunctionDef) and fn.name == '__init__':
                if not _chains(fn):
                    out.append(cls.name)
    return out


@pytest.mark.parametrize('path', _modules(), ids=lambda p: p.name)
def test_every_subclass_init_chains_to_its_base(path):
    found = _offenders(ast.parse(path.read_text()))
    assert not found, (
        f'{path.name}: __init__ never calls super().__init__ in {found}. '
        f'Either the chain was dropped, or an inserted method cut the '
        f'constructor in half and took the chain with it.')

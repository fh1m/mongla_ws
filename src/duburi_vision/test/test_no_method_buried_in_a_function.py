"""No class method may be defined INSIDE another function.

⛔ WHAT THIS CATCHES, and it reached the vehicle. A `set_masks` method was
inserted directly after `self._want_masks = bool(masks)` -- which is a line of
`__init__`, so the rest of the constructor became the new method's body. Every
attribute set after that line stopped existing, and the failure surfaced as
`AttributeError: 'HailoSegDetector' object has no attribute '_ready'` on the
FIFTEENTH inference, then a detector rebuild, then the process exiting.

Nothing caught it earlier: the class needs a Hailo device to construct, so no
unit test instantiates it, and the file imports and parses perfectly.

Structural and hardware-free: walk each class body and assert every `def` that
is meant to be a method is a direct child of the ClassDef. A genuine closure
(a function defined inside a method and used there) is fine -- what is not fine
is a `def` that is indented as if it were a method while sitting inside one,
which is exactly what a bad insertion produces.
"""
from __future__ import annotations

import ast
import pathlib

import pytest

_SRC = pathlib.Path(__file__).resolve().parents[1] / 'duburi_vision'


def _modules() -> list[pathlib.Path]:
    return sorted(p for p in _SRC.rglob('*.py') if p.name != '__init__.py')


def _buried_methods(tree: ast.AST) -> list[str]:
    """Names of defs nested in a method that LOOK like methods (self first)."""
    out = []
    for cls in [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]:
        direct = {id(n) for n in cls.body}
        for method in [n for n in cls.body
                       if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]:
            for inner in ast.walk(method):
                if not isinstance(inner, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    continue
                if id(inner) in direct or inner is method:
                    continue
                args = inner.args.args
                if args and args[0].arg == 'self':
                    out.append(f'{cls.name}.{method.name} -> def {inner.name}(self, ...)')
    return out


@pytest.mark.parametrize('path', _modules(), ids=lambda p: p.name)
def test_no_method_is_buried_inside_another_method(path):
    found = _buried_methods(ast.parse(path.read_text()))
    assert not found, (
        f'{path.name}: a method taking `self` is defined inside another '
        f'method -- the enclosing one was almost certainly cut in half by an '
        f'insertion: {found}')

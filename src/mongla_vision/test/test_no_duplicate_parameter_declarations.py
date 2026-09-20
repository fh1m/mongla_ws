"""A parameter declared twice kills the node at startup, and nothing tests it.

`Node.declare_parameter` raises `ParameterAlreadyDeclaredException` on the
second call for the same name, so a duplicate is not a subtle bug -- the node
dies in its constructor. It is easy to introduce by moving a block of __init__
and re-inserting it, which is exactly how one appeared in `lock_node` on
2026-09-10: an edit script raised BEFORE writing its deletion, then a second
script inserted the block again. The whole vision suite passed with both copies
present, because every test that touches this node reads its SOURCE rather than
constructing it.

Scans the source instead of instantiating, for the same reason: constructing a
node needs rclpy, a graph, and in some cases hardware.
"""
import ast
import pathlib
from collections import Counter

import pytest

_SRC = pathlib.Path(__file__).resolve().parents[2]


def _nodes():
    return sorted(p for p in _SRC.glob('*/*/[a-z]*.py')
                  if 'declare_parameter' in p.read_text())


def _declared_literals(path):
    """Names passed to declare_parameter as a plain string literal."""
    names = []
    for n in ast.walk(ast.parse(path.read_text())):
        if (isinstance(n, ast.Call)
                and getattr(n.func, 'attr', '') == 'declare_parameter'
                and n.args and isinstance(n.args[0], ast.Constant)
                and isinstance(n.args[0].value, str)):
            names.append(n.args[0].value)
    return names


def test_the_scan_actually_finds_nodes():
    """A guard that silently checks nothing is worse than no guard."""
    found = _nodes()
    assert len(found) >= 5, f'only {len(found)} files declare parameters'


@pytest.mark.parametrize('path', _nodes(), ids=lambda p: p.name)
def test_no_parameter_is_declared_twice_in_one_file(path):
    dupes = {n: c for n, c in Counter(_declared_literals(path)).items() if c > 1}
    assert not dupes, (
        f'{path.name} declares {sorted(dupes)} more than once. rclpy raises '
        f'ParameterAlreadyDeclaredException on the second call, so this node '
        f'dies in its constructor.')

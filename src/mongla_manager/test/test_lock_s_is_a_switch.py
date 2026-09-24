"""`lock_s` enables the ladder; its magnitude does nothing.

⚠ The name reads as seconds and an operator will treat it as reach. It is not:
`vision_state.bbox_error` tests `lock_s > 0.0` and never compares the value to
an age. That is the RIGHT behaviour -- the ladder decays its own authority from
the last real detection and stops publishing at zero, so a second numeric
horizon here would be the same quantity in two places and the shorter one would
silently win.

This pins the semantics so a future change that starts honouring the number has
to do it deliberately, and so the misleading name cannot quietly become a
misleading behaviour.
"""
import ast
from pathlib import Path

SRC = (Path(__file__).resolve().parents[2] / 'mongla_manager'
       / 'mongla_manager' / 'vision_state.py')


def test_lock_s_is_only_ever_compared_to_zero():
    tree = ast.parse(SRC.read_text())
    comparisons = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Compare)
                and isinstance(node.left, ast.Name)
                and node.left.id == 'lock_s'):
            for c in node.comparators:
                comparisons.append(ast.dump(c))
    assert comparisons, 'lock_s is never tested at all -- the ladder is unreachable'
    for c in comparisons:
        assert "value=0" in c or "value=0.0" in c, (
            f'lock_s is compared against something other than zero ({c}). If '
            f'that is deliberate, the ladder now has TWO horizons -- its own '
            f'authority decay and this one -- and the shorter silently wins.')


def test_the_ladder_is_still_reachable_from_bbox_error():
    src = SRC.read_text()
    assert 'if lock_s > 0.0:' in src
    assert '_lock_sample(' in src

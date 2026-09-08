"""The recovery sign in the DSL docstring must match the authoritative doc.

⛔ B33. `VisionResult`'s docstring is the first thing a mission author reads --
it is on the class they are holding. Until 2026-09-08 its worked example had the
recovery inverted:

    if res.x_px < -30: duburi.move_right(1)     # target LEFT  -> went right
    elif res.x_px > 30: duburi.move_left(1)     # target RIGHT -> went left

while its own field list says "+x = right" and `vision-results.md` section 3 says
`x_px > 0` -> `move_right`. Copied as written, the recovery drives AWAY from the
target and doubles the error -- and it is the one convention CLAUDE.md singles out
as a trap ("Recovery sign matches `align` itself").

Nothing had copied it yet. This pins the example so nothing can.

The control law is the tie-breaker and it is unambiguous: CLAUDE.md section 6 has
"+ex -> Ch6 > 1500 -> strafe RIGHT (no negation)", i.e. a target seen to the right
makes the vehicle move right. The recovery continues that same direction.
"""

import inspect
import pathlib
import re

import pytest

from duburi_planner.vision_dsl import VisionResult


def _pairs(text):
    """[(comparison, verb)] for every `x_px <cmp> <n>: duburi.move_<dir>` line."""
    return re.findall(r'x_px\s*([<>])\s*-?\d+\s*:\s*duburi\.move_(right|left)', text)


def test_the_dsl_docstring_chases_the_target_rather_than_fleeing_it():
    doc = inspect.getdoc(VisionResult) or ''
    found = _pairs(doc)
    assert found, 'the worked recovery example vanished from the docstring'
    for cmp_op, direction in found:
        expect = 'right' if cmp_op == '>' else 'left'
        assert direction == expect, (
            f'x_px {cmp_op} N -> move_{direction} is inverted: +x means the target '
            f'ended RIGHT of centre, so it must be move_{expect} (B33)')


def test_the_docstring_agrees_with_vision_results_md():
    """Two copies of a sign convention is how they came to disagree."""
    root = pathlib.Path(__file__).resolve().parents[3]
    md = root / '.claude' / 'context' / 'vision-results.md'
    if not md.is_file():
        pytest.skip('vision-results.md not in this checkout')
    text = md.read_text(encoding='utf-8')

    # The authoritative table rows, e.g. "| `x_px > 0` | ... strafe **right** ..."
    assert re.search(r'x_px\s*>\s*0.*?right', text, re.S | re.I), \
        'vision-results.md no longer states x_px > 0 -> right'

    for cmp_op, direction in _pairs(inspect.getdoc(VisionResult) or ''):
        expect = 'right' if cmp_op == '>' else 'left'
        assert direction == expect, \
            f'DSL docstring disagrees with vision-results.md on x_px {cmp_op} 0'


def test_positive_x_means_right_is_still_the_stated_field_semantics():
    """If +x ever stops meaning 'right', every recovery above inverts with it."""
    doc = inspect.getdoc(VisionResult) or ''
    assert re.search(r'\+x\s*=\s*right', doc, re.I), \
        'the "+x = right" field definition is what makes the recovery signs correct'

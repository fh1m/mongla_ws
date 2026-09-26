"""The DShot console decoder, against values read off the real board.

TRUTH test, not an agreement test: every expected number below was produced by
the board on 2026-09-22 with nothing attached to any ESC, and the magnitudes are
known because the two motors were commanded to the SAME demand in opposite
directions.

Why it needs a guard at all: decoded as a signed range around 1048 -- the
obvious reading, and the one taken first -- the mixer comes out asymmetric, the
heave row grows a 3 % imbalance that is not there, and every entry is wrong.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))

from mixer_map import dshot_signed  # noqa: E402


def test_both_halves_have_their_own_zero():
    """48 and 1048 are each the bottom of a direction, so both mean 'no output'.
    A decoder with one midpoint reports 48 as -1000 and stops the vehicle from
    ever reading as idle."""
    assert dshot_signed(1048) == 0
    assert dshot_signed(48) == 0


def test_opposite_directions_at_equal_demand_have_equal_magnitude():
    """Read off the board: a 0.02 fwd demand put M1 at 229 and M2 at 1230. The
    mixer sends those two motors the same magnitude, opposite ways."""
    m1, m2 = dshot_signed(229), dshot_signed(1230)

    assert m1 < 0 < m2
    assert abs(abs(m1) - abs(m2)) <= 1, f'{m1} vs {m2} are not the same size'


def test_full_scale_is_999_each_way():
    assert dshot_signed(2047) == 999
    assert dshot_signed(1047) == -999


def test_the_naive_midpoint_reading_is_what_this_rejects():
    """Guard the guard: 229 decoded as a deviation from 1048 would be -819. If
    this ever passes, the decoder has regressed to the wrong model."""
    assert dshot_signed(229) != 229 - 1048


@pytest.mark.parametrize('value,expected', [
    (1507, 459), (516, -468), (2041, 993), (1041, -993), (1230, 182), (229, -181),
])
def test_values_measured_on_the_board(value, expected):
    assert dshot_signed(value) == expected


def test_the_tool_and_the_module_decode_identically():
    """`tools/mixer_map.py` carries a standalone copy because it is scp'd to the
    vehicle and run from /tmp, where the workspace is not importable. A second
    copy nobody compares is how two truths appear -- so compare them, across the
    whole of both bands and the boundary between them."""
    from mongla_control.actuation_model import dshot_signed as module_impl

    for v in list(range(48, 1048, 7)) + list(range(1048, 2048, 7)) + [
            48, 1047, 1048, 2047]:
        assert int(dshot_signed(v)) == module_impl(v), f'disagree at {v}'


def test_no_other_copy_of_the_band_arithmetic_exists():
    """⛔ THE COPY THAT WENT WRONG WAS THE ONE NOTHING COMPARED. `mixer_map.py`
    has a standalone copy for a stated reason and the test above compares it.
    `tools/control_bench/` had TWO more, uncompared, and they disagreed:
    `flight.py` decoded the reverse band as `-(v - 47)` where every other copy
    uses 48. One count -- but it put a STOPPED thruster at -1/999 of full
    reverse and full reverse at a magnitude of 1.001, outside the range its
    caller is entitled to assume.

    Both now load `actuation_model` through the bench's own `_load_pure`. This
    test fails if a third copy appears: any file that spells a DShot band floor
    as a literal, other than the module that owns the constants and the tool
    that is scp'd to the vehicle without the workspace.
    """
    import re
    from pathlib import Path

    root = Path(__file__).resolve().parents[3]
    owner = {'actuation_model.py',        # owns the constants
             'mixer_map.py',              # runs from /tmp on the vehicle
             'test_dshot_decode.py'}      # this file, which quotes them
    # 1047/1048/1049 and the 48 floor, written as a bare number next to a
    # subtraction -- the shape of a hand-rolled decode.
    pat = re.compile(r'-\s*\(\s*\w+\s*-\s*(4[5-9])\s*\)|'
                     r'\w+\s*-\s*10(4[789])\b')
    offenders = []
    for p in list(root.glob('src/*/*/**/*.py')) + list(root.glob('tools/**/*.py')):
        if p.name in owner or '/install/' in str(p) or '/build/' in str(p):
            continue
        for i, line in enumerate(p.read_text().splitlines(), 1):
            if line.lstrip().startswith('#'):
                continue
            if pat.search(line):
                offenders.append(f'{p.relative_to(root)}:{i}: {line.strip()}')
    assert not offenders, (
        'a hand-rolled DShot band decode -- import `actuation_model.'
        'dshot_signed` instead:\n  ' + '\n  '.join(offenders))

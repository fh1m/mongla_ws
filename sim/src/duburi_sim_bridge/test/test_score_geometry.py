"""The scorer must aim at the props that were actually built.

A torpedo fired dead centre through a real opening was graded PAST_BOARD, and a
marker landing squarely in a crate was graded OUTSIDE_BIN. Neither logged
anything, because nothing was wrong: the scorer was looking exactly where it had
been told to. `board_openings` was a hand-typed TWO-opening default left behind
by the 2026 four-opening rewrite, and the drop target was a single box at the
bins MODEL ORIGIN while four crates hang +-0.52 m off the pipeline.

These tests compare the scorer's geometry against `prop_library` -- the module
that cuts the mesh and paints the texture -- rather than against another typed
constant, because a second copy of a number is how this happened.
"""

import math
import os
import sys

import pytest

yaml = pytest.importorskip('yaml')

ROOT = os.path.join(os.path.dirname(__file__), '..', '..', 'duburi_sim_worlds')
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import prop_library as pl                                         # noqa: E402
from duburi_sim_bridge.scoring import (                           # noqa: E402
    _board_opening_kinds, _board_openings_world, _bin_targets_world)


def _spec():
    with open(os.path.join(ROOT, 'spec', 'robosub.yaml')) as fh:
        return yaml.safe_load(fh)


def _openings(yaw=math.pi):
    spec = _spec()
    board = spec['props']['torpedo_board']
    vals = _board_openings_world(pl, spec, board, spec['pool']['depth'], yaw)
    return [vals[i:i + 3] for i in range(0, len(vals), 3)]


def _grade(y, z, by=0.0):
    """The same test `_grade_torpedo` does, against the derived openings."""
    for idx, (oy, oz, r) in enumerate(_openings()):
        if math.hypot(y - (by + oy), z - oz) <= r:
            return 'through', idx
    return 'past_board', None


# --------------------------------------------------------------- torpedo ---

def test_the_scorer_has_one_opening_per_opening_on_the_board():
    """THE GUARD FOR THIS WHOLE CLASS. Add an opening to the spec and the
    scorer must follow it, or fail loudly -- never quietly grade against the
    board it used to have."""
    assert len(_openings()) == len(pl.torpedo_openings(_spec())) == 4


@pytest.mark.parametrize('idx', range(4))
def test_a_shot_through_the_centre_of_each_opening_scores(idx):
    oy, oz, _r = _openings()[idx]
    outcome, hit = _grade(oy, oz)
    assert outcome == 'through', f'opening {idx} at ({oy:.3f}, {oz:.3f})'
    assert hit == idx


@pytest.mark.parametrize('idx', range(4))
def test_a_shot_one_mm_outside_each_rim_does_not_score(idx):
    oy, oz, r = _openings()[idx]
    assert _grade(oy + r + 0.001, oz)[0] == 'past_board'


def test_the_openings_sit_at_the_plate_not_at_the_pool_floor():
    """Plate-relative z must be lifted by floor + legs + size/2. The stale
    default's -1.10/-1.40 were close enough to the right rows to look
    plausible, which is part of why this survived."""
    spec = _spec()
    board = spec['props']['torpedo_board']
    cz = (-spec['pool']['depth'] + board['leg_height'] + board['size'] / 2.0)
    for _oy, oz, _r in _openings():
        assert abs(oz - cz) == pytest.approx(0.132, abs=1e-6)


def test_the_board_yaw_flips_the_side_an_opening_is_on():
    """A y-sign error is INVISIBLE on a centred shot, so it is checked here
    rather than left to an end-to-end run that cannot see it."""
    at_0 = _openings(yaw=0.0)
    at_pi = _openings(yaw=math.pi)
    assert [round(o[0], 6) for o in at_0] == [round(-o[0], 6) for o in at_pi]
    assert any(abs(o[0]) > 0.05 for o in at_0), 'no off-centre opening to test'


def test_every_opening_is_labelled_from_the_layout_not_the_index():
    """`'large' if idx == 0 else 'small'` cannot describe two of each."""
    kinds = _board_opening_kinds(pl, _spec())
    assert len(kinds) == 4
    assert sorted(kinds) == ['large', 'large', 'small', 'small']


# ------------------------------------------------------------------ bins ---

def _targets(bins_xy=(0.5, 0.0)):
    vals = _bin_targets_world(pl, _spec(), bins_xy)
    return [vals[i:i + 2] for i in range(0, len(vals), 2)]


def _in_bin(x, y):
    spec = _spec()['props']['bin']
    sx, sy = spec['length'], spec['width']
    return any(abs(x - tx) <= sx / 2.0 and abs(y - ty) <= sy / 2.0
               for tx, ty in _targets())


def test_there_is_one_drop_target_per_crate():
    assert len(_targets()) == 4


@pytest.mark.parametrize('idx', range(4))
def test_a_marker_in_each_crate_scores(idx):
    x, y = _targets()[idx]
    assert _in_bin(x, y), f'crate {idx} at ({x:.3f}, {y:.3f})'


def test_a_marker_on_the_pipework_between_the_crates_does_not_score():
    """The bins model ORIGIN. It was the only place that used to score, and it
    is open water on the pipe run between the crates."""
    assert not _in_bin(0.5, 0.0)


def test_the_crates_are_where_the_model_puts_them():
    """Composed with the course's model xy, not left model-relative."""
    assert sorted(round(y, 3) for _x, y in _targets()) == [-0.52, -0.52,
                                                           0.52, 0.52]
    assert sorted(round(x, 3) for x, _y in _targets()) == [0.11, 0.37,
                                                           0.682, 0.942]

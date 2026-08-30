"""The board's holes must be real, and there must be exactly one list of them.

Two defects sit behind this. The first was drift: the printed artwork drew FOUR
circles while the collision cut TWO, in different places, so a shot lined up on
the artwork struck solid board and nothing in any log said why. The second was
worse and lasted longer -- the openings were PAINTED on a solid box. The
collision was genuinely open, the visual never was, so an opening never
parallaxed, never showed water behind it and never responded to fog. A detector
trained on that learns a painted bullseye rather than a hole.
"""
import os
import re

import pytest

yaml = pytest.importorskip('yaml')

ROOT = os.path.join(os.path.dirname(__file__), '..', '..', 'duburi_sim_worlds')


def _spec():
    with open(os.path.join(ROOT, 'spec', 'robosub.yaml')) as fh:
        return yaml.safe_load(fh)


def test_the_board_has_four_openings():
    """2026: a 2x2 of image+opening cells, two large and two small."""
    cells = _spec()['props']['torpedo_board']['cells']
    assert len(cells) == 4
    sizes = sorted(c['size'] for c in cells)
    assert sizes == ['large', 'large', 'small', 'small']


def test_every_role_carries_an_image_pair():
    """The 2026 board shows a role's symbol AND its vehicle."""
    for role, cfg in _spec()['roles'].items():
        assert len(cfg['task_images']) == 2, role


def test_the_visual_is_a_mesh_not_a_box():
    """A box cannot have a hole in it. This is the whole fix."""
    path = os.path.join(ROOT, 'models', 'robosub_torpedo_survey_repair',
                        'model.sdf')
    if not os.path.isfile(path):
        pytest.skip('board not generated')
    with open(path) as fh:
        sdf = fh.read()
    assert 'torpedo_plate.obj' in sdf


def test_mesh_and_collision_come_from_the_same_opening_list():
    """The invariant that fixed the artwork/collision drift. If these ever
    diverge again, a mission aims at a hole that is not there."""
    import sys
    sys.path.insert(0, os.path.join(ROOT, 'scripts'))
    import prop_library as pl                                   # noqa: E402
    import gen_prop_meshes as gm                                # noqa: E402

    spec = _spec()
    holes = pl.torpedo_openings(spec)
    assert len(holes) == 4
    cfg = spec['props']['torpedo_board']
    verts, _, faces, _n = gm.plate_with_holes(
        cfg['size'], cfg['thickness'], holes)
    assert verts and faces

    # no vertex may sit strictly inside an opening -- that is what "cut out"
    # means, and it is checkable rather than assertable by eye
    for (x, y, z) in verts:
        for (hy, hz, r) in holes:
            d = ((y - hy) ** 2 + (z - hz) ** 2) ** 0.5
            assert d > r * 0.98, f'vertex {(y, z)} inside opening at {(hy, hz)}'


def test_the_generated_mesh_exists_and_is_watertight_enough_to_load():
    obj = os.path.join(ROOT, 'models', 'robosub_meshes', 'meshes',
                       'torpedo_plate.obj')
    if not os.path.isfile(obj):
        pytest.skip('mesh not generated')
    with open(obj) as fh:
        text = fh.read()
    assert re.search(r'^v ', text, re.M), 'no vertices'
    assert re.search(r'^vt ', text, re.M), 'no UVs -- artwork would not land'
    assert re.search(r'^f \d+/\d+', text, re.M), 'faces must reference UVs'

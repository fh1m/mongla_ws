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


def _pl():
    import sys
    sys.path.insert(0, os.path.join(ROOT, 'scripts'))
    import prop_library as pl                                   # noqa: E402
    return pl


def test_the_board_has_four_openings():
    """2026: two rows of image+opening slots, two large openings and two small."""
    layout = _pl().torpedo_layout(_spec())
    sizes = sorted(e['kind'] for e in layout if e['radius'] is not None)
    assert sizes == ['large', 'large', 'small', 'small']
    assert sum(1 for e in layout if e['kind'] == 'image') == 4


def test_nothing_on_the_board_overlaps_anything_else():
    """THE CHECK THAT SHOULD HAVE EXISTED IN ROUND 9.

    The board shipped with an image overlapping a ring by 0.031 UV and the
    large ring running 0.006 past the board's own edge, because four column
    centres were typed in by hand 0.235 apart while a large ring's outer radius
    plus an image's half-width needs 0.262. Arithmetic catches that instantly;
    eyes did not, twice.

    This asserts on the SAME radii the mesh cuts and the texture paints, so it
    fails if anyone widens `ring_band` or `image_size_m` past what fits --
    rather than the board silently going wrong again.
    """
    pl, spec = _pl(), _spec()
    cfg = spec['props']['torpedo_board']
    edge = cfg['size'] / 2.0 - cfg['inset']

    for gap in pl.torpedo_gaps(spec):
        assert gap > 0.0, f'row elements do not fit: gap {gap:.4f} m'

    rows = {}
    for e in pl.torpedo_layout(spec):
        assert abs(e['y']) + e['half_width'] <= edge + 1e-9, e
        assert abs(e['z']) + e['half_width'] <= edge + 1e-9, e
        rows.setdefault(round(e['z'], 6), []).append(e)

    for z, elems in rows.items():
        elems.sort(key=lambda e: e['y'])
        for a, b in zip(elems, elems[1:]):
            clear = (b['y'] - b['half_width']) - (a['y'] + a['half_width'])
            assert clear > 0.0, f'row {z}: {a["kind"]}/{b["kind"]} overlap'

    # And the two rows must not run into each other vertically.
    tops = sorted(rows.items())
    lo, hi = tops[0], tops[-1]
    lo_top = lo[0] + max(e['half_width'] for e in lo[1])
    hi_bot = hi[0] - max(e['half_width'] for e in hi[1])
    assert hi_bot > lo_top, 'the two rows overlap'


def test_the_board_stands_on_exactly_two_legs():
    """It had FOUR: two uprights and two raking kickstand braces with foot pads.

    The braces were added so a 0.6 m board on two thin poles would not read as
    unsupported, and the render showed the cost -- a four-legged trestle where
    the task slide and the pool photographs show two legs. Last round fixed the
    braces' rake (`pi - atan2`); this round deletes them, so the test that
    checked the rake is replaced by one that checks they are gone. `brace_rake`
    went with them: a spec key nothing reads is how the last one of these hid.
    """
    xml = _pl().robosub_torpedo_board(_spec(), role='survey_repair')
    assert xml.count('<link name="leg_') == 2
    assert 'brace' not in xml
    assert 'brace_rake' not in _spec()['props']['torpedo_board']


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

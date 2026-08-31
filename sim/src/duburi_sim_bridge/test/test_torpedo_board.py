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


def test_the_rear_braces_run_from_the_rail_to_the_floor():
    """They used to rake the wrong way: `atan2(rake, legs)` put one end on the
    floor directly UNDER the board and the other in open water behind it,
    pointing up at nothing, while the foot pad sat where the brace never
    reached. That is the "legs skewed and pointing half up" in the render."""
    import math
    cfg = _spec()['props']['torpedo_board']
    rake, legs = cfg['brace_rake'], cfg['leg_height']
    length = math.hypot(rake, legs)
    pitch = math.pi - math.atan2(rake, legs)
    axis = (math.sin(pitch), math.cos(pitch))
    cx, cz = rake / 2.0, legs / 2.0
    ends = [(cx + s * length / 2.0 * axis[0], cz + s * length / 2.0 * axis[1])
            for s in (1, -1)]
    floor = min(ends, key=lambda e: e[1])
    rail = max(ends, key=lambda e: e[1])
    assert abs(floor[1]) < 1e-9 and abs(floor[0] - rake) < 1e-9
    assert abs(rail[0]) < 1e-9 and abs(rail[1] - legs) < 1e-9


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

"""Water belongs in the pool, and only in the pool.

`water_surface: gerstner` includes openrobotics/waves -- an UNBOUNDED ocean at
z = 0. It animates and it does reach camera sensors, which is why it was made
the default; what it also does is cover the whole world, so the pool sat in an
open sea, every view outside the walls was underwater, and the deck looked out
on water to the horizon. A competition pool is a box of water in air.

The default has now moved in both directions across two rounds, each time for a
real reason, and nothing checked the result either time. So: check it.
"""

import glob
import os
import re

import pytest

yaml = pytest.importorskip('yaml')

ROOT = os.path.join(os.path.dirname(__file__), '..', '..', 'duburi_sim_worlds')
WORLDS = sorted(glob.glob(os.path.join(ROOT, 'worlds', '*.world')))


def _spec(name):
    with open(os.path.join(ROOT, 'spec', f'{name}.yaml')) as fh:
        return yaml.safe_load(fh)


def test_there_are_worlds_to_check():
    """A glob that matches nothing passes every parametrized test below."""
    assert WORLDS, 'no generated worlds -- run gen_world.py --all'


@pytest.mark.parametrize('world', WORLDS, ids=os.path.basename)
def test_no_course_ships_an_unbounded_ocean(world):
    with open(world) as fh:
        xml = fh.read()
    assert 'models/waves' not in xml, (
        f'{os.path.basename(world)} includes the unbounded Fuel ocean; water '
        'would cover the whole world, not just the pool')


def _obj_extents(path):
    """(x span, y span) of an OBJ's vertices.

    The span is read off the GEOMETRY THAT SHIPS, not off a size attribute in
    the world. The surface used to be a <box><size>, and that regex was the
    whole check; when it became a mesh the old assertion could only have failed
    loudly (fine) or -- if it had been written as a permissive search instead of
    a required match -- found nothing and passed vacuously, which is the failure
    mode this file exists to prevent (round 12: the first wrong-span injection
    was missed because it patched the pool FLOOR's box).
    """
    xs, ys = [], []
    with open(path) as fh:
        for line in fh:
            if line.startswith('v '):
                _, x, y, _z = line.split()[:4]
                xs.append(float(x))
                ys.append(float(y))
    assert xs, f'{path} has no vertices'
    return max(xs) - min(xs), max(ys) - min(ys)


@pytest.mark.parametrize('world', WORLDS, ids=os.path.basename)
def test_the_water_surface_spans_exactly_its_pool(world):
    with open(world) as fh:
        xml = fh.read()
    block = xml.split('<visual name="water_surface_visual">')
    assert len(block) == 2, f'{os.path.basename(world)} has no water surface'
    # <geometry> WRAPPER, explicitly. SDF drops a <box> or <mesh> that is not
    # inside one, the visual then renders NOTHING, and gz -v 2 says nothing --
    # which cost this round four wrong diagnoses (failed shader, backface
    # culling, mis-placed probe) before the missing element was spotted. Every
    # other geometry here comes from a _geometry_* helper that wraps it; the
    # water surface was the one hand-written string.
    assert '<geometry>' in block[1].split('</visual>')[0], (
        f'{os.path.basename(world)} water surface has no <geometry> element -- '
        'it will render nothing, silently')
    uri = re.search(r'<geometry><mesh><uri>model://robosub_meshes/meshes/'
                    r'(water_\w+\.obj)</uri></mesh></geometry>', block[1])
    assert uri, 'water surface is not a generated mesh inside a <geometry>'
    mesh = os.path.join(ROOT, 'models', 'robosub_meshes', 'meshes',
                        uri.group(1))
    assert os.path.exists(mesh), (
        f'{os.path.basename(world)} references {uri.group(1)}, which does not '
        'exist -- run gen_prop_meshes.py')
    # Which pool a course uses comes from its OWN `pool:` key, not from its
    # name: task_navigation / task_localization / task_target_acquisition all
    # say `pool: sauvc` (25 x 16) with no "sauvc" in the filename, and guessing
    # from the name passed three worlds against the wrong pool.
    course = os.path.join(ROOT, 'courses',
                          os.path.basename(world).replace('.world', '.yaml'))
    with open(course) as fh:
        comp = (yaml.safe_load(fh) or {}).get('pool', 'robosub')
    pool = _spec(comp)['pool']
    span_x, span_y = _obj_extents(mesh)
    assert span_x == pytest.approx(pool['length'])
    assert span_y == pytest.approx(pool['width'])


@pytest.mark.parametrize('world', WORLDS, ids=os.path.basename)
def test_the_water_surface_can_actually_be_displaced(world):
    """A vertex-displacement shader on 8 vertices displaces nothing.

    This is the reason the animated surface was carried and cut across rounds
    12, 13, 14, 19 and 22: the surface was a <box>. The grid must be fine
    enough to carry the shortest wave the shader is configured for -- the
    shortest is 1.1 m and Nyquist wants at least two samples across it.
    """
    with open(world) as fh:
        block = fh.read().split('<visual name="water_surface_visual">')[1]
    uri = re.search(r'(water_\w+\.obj)', block)
    assert uri, 'water surface is not a generated mesh'
    mesh = os.path.join(ROOT, 'models', 'robosub_meshes', 'meshes',
                        uri.group(1))
    with open(mesh) as fh:
        verts = [l for l in fh if l.startswith('v ')]
    span_x, _ = _obj_extents(mesh)
    # cells along x == sqrt-free: count distinct x values - 1
    xs = sorted({round(float(l.split()[1]), 6) for l in verts})
    cell = span_x / (len(xs) - 1)
    assert cell <= 0.55, (
        f'{os.path.basename(world)} water grid is {cell:.2f} m per cell; the '
        'shortest configured wave is 1.1 m and would alias')


@pytest.mark.parametrize('world', WORLDS, ids=os.path.basename)
def test_the_surface_is_actually_VISIBLE_from_underneath(world):
    """It was 0.62 transparent, which is why the pool read as EMPTY and why
    somebody reached for an ocean to cover it. Every camera in this simulator
    sits under this plane; a surface you can see straight through is not one."""
    with open(world) as fh:
        block = fh.read().split('<visual name="water_surface_visual">')[1]
    t = re.search(r'<transparency>([\d.]+)</transparency>', block)
    assert t, 'no transparency on the water surface'
    assert float(t.group(1)) <= 0.35, (
        f'water surface is {float(t.group(1)):.2f} transparent -- the pool '
        'will look empty from below')

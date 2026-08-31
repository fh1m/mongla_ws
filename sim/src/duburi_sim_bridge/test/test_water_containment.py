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


@pytest.mark.parametrize('world', WORLDS, ids=os.path.basename)
def test_the_water_surface_spans_exactly_its_pool(world):
    with open(world) as fh:
        xml = fh.read()
    block = xml.split('<visual name="water_surface_visual">')
    assert len(block) == 2, f'{os.path.basename(world)} has no water surface'
    size = re.search(r'<box><size>([\d.]+) ([\d.]+) [\d.]+</size></box>',
                     block[1])
    assert size, 'water surface is not a box'
    # Which pool a course uses comes from its OWN `pool:` key, not from its
    # name: task_navigation / task_localization / task_target_acquisition all
    # say `pool: sauvc` (25 x 16) with no "sauvc" in the filename, and guessing
    # from the name passed three worlds against the wrong pool.
    course = os.path.join(ROOT, 'courses',
                          os.path.basename(world).replace('.world', '.yaml'))
    with open(course) as fh:
        comp = (yaml.safe_load(fh) or {}).get('pool', 'robosub')
    pool = _spec(comp)['pool']
    assert float(size.group(1)) == pytest.approx(pool['length'])
    assert float(size.group(2)) == pytest.approx(pool['width'])


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

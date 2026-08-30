"""Courses must spawn the hull near the surface.

ArduSub captures its depth reference where the vehicle sits at boot. The pool
hull is powered on FLOATING, so its reference is the surface. Every sim course
used to spawn submerged at its own depth, and the readback offset tracked that
spawn depth exactly:

    spawn -0.8 -> -0.344 m     spawn -0.5 -> -0.044 m     spawn -0.3 -> +0.016 m

At -0.344 m two verbs break and interlock: `surface()` never confirms (it
commands 0.0 and the readback plateaus near -0.4) and `mission_reset()`'s baro
re-zero is REFUSED for exceeding the 0.30 m surface bound.

Neither failure names the course, so a new course spawning deep would be
rediscovered as "the depth verbs are broken again". This test is the guard.
"""
from pathlib import Path

import pytest

yaml = pytest.importorskip('yaml')

COURSES = Path(__file__).resolve().parents[2] / 'duburi_sim_worlds' / 'courses'

# -0.4 measured +0.017 m of offset. -0.5 measured -0.044. The bound sits below
# where the error starts to matter and well inside _BARO_SURFACE_BOUND_M (0.30).
MAX_SPAWN_DEPTH_M = 0.55


def _courses():
    return sorted(COURSES.glob('*.yaml'))


def test_there_are_courses_to_check():
    """A silent zero-course pass would make every assertion below vacuous."""
    assert _courses(), f'no course yaml under {COURSES}'


@pytest.mark.parametrize('course', _courses(), ids=lambda p: p.stem)
def test_spawn_is_near_the_surface(course):
    spec = yaml.safe_load(course.read_text()) or {}
    pose = (spec.get('vehicle') or {}).get('pose')
    if pose is None:
        pytest.skip('course uses the generator default spawn')
    depth = abs(float(pose[2]))
    assert depth <= MAX_SPAWN_DEPTH_M, (
        f'{course.stem} spawns at z={pose[2]}, {depth:.2f} m down. ArduSub will '
        f'capture its depth reference there and surface() will never confirm. '
        f'Spawn at -0.4 like the other courses.')

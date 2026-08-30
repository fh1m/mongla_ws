"""Projectile physics and shot scoring.

Where a fired round goes is what the vision and control pipeline has to time a
shot against, so these are the numbers that decide whether a sim-tuned shot
works in the pool. The end-to-end path (fire -> through the opening -> graded)
is measured against a running sim; see PAYLOAD.md.
"""

import math
import re

import pytest

from duburi_sim_bridge.payload_sim import (
    WATER_DENSITY,
    _DROPPER_M,
    _DROPPER_R,
    _TORPEDO_CD_AXIAL,
    _TORPEDO_CD_CROSS,
    _TORPEDO_L,
    _TORPEDO_M,
    _TORPEDO_R,
    _projectile_sdf,
)

# Handbook p. 64: "Each torpedo must fit within a box 2.0" square and 6" long
# (51 x 51 x 152 mm)" and "must weigh no more than 2.0 lbs (0.91 kg) in air".
RULEBOOK_BOX = (0.051, 0.051, 0.152)
RULEBOOK_MAX_MASS = 0.91


def _coeff(sdf, tag):
    return float(re.search(rf'<{tag}>([-\d.e+]+)</{tag}>', sdf).group(1))


def test_torpedo_fits_the_rulebook_box():
    assert 2 * _TORPEDO_R <= RULEBOOK_BOX[0] + 1e-9
    assert _TORPEDO_L <= RULEBOOK_BOX[2] + 1e-9


def test_both_projectiles_are_inside_the_mass_limit():
    """A round over the limit is a 500-point penalty, or disqualification."""
    assert _TORPEDO_M <= RULEBOOK_MAX_MASS
    assert _DROPPER_M <= RULEBOOK_MAX_MASS


def test_torpedo_mass_comes_from_displacement():
    """Derived, never picked.

    An earlier guess of 790 g was 479 g negative on a body that displaces
    311 g, and it plummeted. One line of arithmetic catches that; watching it
    fly does not, because a plummet and a heavy round look alike.
    """
    displaced = math.pi * _TORPEDO_R ** 2 * _TORPEDO_L * WATER_DENSITY
    net = _TORPEDO_M - displaced
    assert 0.0 < net < 0.03, (
        f'torpedo is {net * 1000:+.0f} g off neutral -- it must sink slowly, '
        f'not surface and not plummet')


def test_dropper_sinks_but_is_not_a_rock():
    displaced = (4.0 / 3.0) * math.pi * _DROPPER_R ** 3 * WATER_DENSITY
    assert _DROPPER_M > displaced, 'a floating dropper never lands in the bin'
    assert _DROPPER_M < 4 * displaced


def test_added_mass_is_present_and_anisotropic():
    """THE physics that makes an underwater shot different from a thrown one.

    A body accelerating underwater drags water with it. Broadside, a cylinder's
    added mass is about its own displacement; nose-on it is a tenth of that.
    Without it a round decelerates and turns like an object in air, which is
    exactly what a pipeline would learn to time against and then not find in
    the pool.
    """
    sdf = _projectile_sdf('payload_shot_0', '0 0 -1 0 1.5708 0', 'torpedo')
    displaced = math.pi * _TORPEDO_R ** 2 * _TORPEDO_L * WATER_DENSITY

    flight = abs(_coeff(sdf, 'zDotW'))       # body z is the flight axis
    cross = abs(_coeff(sdf, 'xDotU'))
    assert flight > 0.0 and cross > 0.0, 'added mass is missing'
    assert cross == pytest.approx(displaced, rel=0.05)
    assert flight == pytest.approx(0.1 * displaced, rel=0.05)
    assert cross > 5 * flight, 'added mass must be strongly anisotropic'


def test_drag_rides_the_axes_the_round_actually_flies_along():
    """An SDF cylinder's length is its own z, so the round is pitched 90 deg.

    Body +z is the flight path and body +x points DOWN. Swapped, the shot
    travelled 0.14 m in 2 s where it should cover 2.5 m.
    """
    sdf = _projectile_sdf('payload_shot_0', '0 0 -1 0 1.5708 0', 'torpedo')
    flight, side, down = (_coeff(sdf, 'zWabsW'), _coeff(sdf, 'yVabsV'),
                          _coeff(sdf, 'xUabsU'))
    assert flight < 0 and side < 0 and down < 0, 'drag must oppose motion'
    assert abs(flight) < abs(side), (
        'the flight axis carries more drag than the broadside axes -- the '
        'coefficients are swapped')
    assert side == pytest.approx(down), 'both broadside axes should match'


def test_drag_matches_its_own_geometry():
    """0.5*rho*Cd*A for THIS body, not a number copied from the vehicle."""
    sdf = _projectile_sdf('payload_shot_0', '0 0 -1 0 1.5708 0', 'torpedo')
    area_axial = math.pi * _TORPEDO_R ** 2
    area_cross = 2 * _TORPEDO_R * _TORPEDO_L
    assert abs(_coeff(sdf, 'zWabsW')) == pytest.approx(
        0.5 * WATER_DENSITY * _TORPEDO_CD_AXIAL * area_axial, rel=1e-3)
    assert abs(_coeff(sdf, 'yVabsV')) == pytest.approx(
        0.5 * WATER_DENSITY * _TORPEDO_CD_CROSS * area_cross, rel=1e-3)


def test_the_round_reaches_the_rulebook_standoffs():
    """It must still be moving usefully at the 'far' bars.

    Range is set by quadratic drag: v(x) = v0 * exp(-k x) with
    k = 0.5*rho*Cd*A / (m + added mass). If a round could not cross 1.5 m the
    'farther' band would be unreachable and the task untestable.
    """
    displaced = math.pi * _TORPEDO_R ** 2 * _TORPEDO_L * WATER_DENSITY
    k = (0.5 * WATER_DENSITY * _TORPEDO_CD_AXIAL * math.pi * _TORPEDO_R ** 2
         / (_TORPEDO_M + 0.1 * displaced))
    v0 = 4.5
    for standoff in (1.0, 1.5):
        assert v0 * math.exp(-k * standoff) > 1.0, (
            f'the round is below 1 m/s at the {standoff} m standoff')


def test_a_torpedo_looks_like_a_torpedo():
    """Shape is a diagnostic: a bare cylinder shows no heading in flight."""
    sdf = _projectile_sdf('payload_shot_0', '0 0 -1 0 1.5708 0', 'torpedo')
    assert 'name="nose"' in sdf
    assert sdf.count('name="fin') == 4
    # The collision stays a plain cylinder -- fins are visual only, so they
    # cannot snag on an opening the body would clear.
    assert sdf.count('<collision') == 1
    assert '<cylinder>' in sdf.split('<collision')[1].split('</collision>')[0]


def test_both_projectile_sdfs_are_well_formed():
    import xml.etree.ElementTree as ET

    for kind in ('torpedo', 'dropper'):
        root = ET.fromstring(_projectile_sdf('payload_shot_3', '1 2 -3 0 0 0',
                                             kind))
        model = root.find('model')
        assert model.get('name') == 'payload_shot_3'
        assert model.find('link/collision') is not None

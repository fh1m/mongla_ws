"""Props that are supposed to move, and the ones that are supposed to be solid.

The bug these guard against is silent in every case: a static prop generates
contact but cannot be pushed, an unjointed dynamic prop falls apart, a dynamic
prop missing from the buoyancy whitelist sinks, and a non-colliding link is one
the vehicle drives straight through. None of them log anything.
"""
import os
import sys
import xml.etree.ElementTree as ET

SCRIPTS = os.path.join(os.path.dirname(__file__),
                       '..', '..', 'duburi_sim_worlds', 'scripts')
sys.path.insert(0, SCRIPTS)

import prop_library as pl                                    # noqa: E402

SAUVC = pl.load_spec(competition='sauvc')
ROBOSUB = pl.load_spec(competition='robosub')


def _model(xml):
    return ET.fromstring(xml)


def _bodies(m):
    return m.findall('link')


def test_flare_is_dynamic_and_fully_welded():
    m = _model(pl.bump_flare(SAUVC, 'red'))
    assert m.find('static').text == 'false'
    # every link but the root has to be welded to it
    assert len(m.findall('joint')) == len(_bodies(m)) - 1


def test_flare_sinks_but_stands_up():
    """Net weight down (stays put) with buoyancy above mass (rights itself).

    The tempting alternative -- net buoyant and moored -- does not work: a
    buoyant free body just rises, because buoyancy gives no restoring force in
    translation. These two together are what does.
    """
    import math
    m = _model(pl.bump_flare(SAUVC, 'red'))
    mass = vol = mz = vz = 0.0
    for link in _bodies(m):
        z = float(link.find('pose').text.split()[2])
        kg = float(link.find('inertial/mass').text)
        g = link.find('collision/geometry')
        v = 0.0
        if g is not None:
            cyl, box = g.find('cylinder'), g.find('box')
            if cyl is not None:
                r = float(cyl.find('radius').text)
                v = math.pi * r * r * float(cyl.find('length').text)
            elif box is not None:
                sx, sy, sz = (float(t) for t in box.find('size').text.split())
                v = sx * sy * sz
        mass += kg; mz += kg * z; vol += v; vz += v * z

    assert mass > vol * 1000, 'flare would float away'
    assert vz / vol > mz / mass + 0.1, 'centre of buoyancy is not above the mass'


def test_flare_has_drag_or_it_rings_forever():
    # Measured without it: knocked to 87.6 deg, still swinging 25 deg at t=24s.
    assert 'gz-sim-hydrodynamics-system' in pl.bump_flare(SAUVC, 'red')


def test_slalom_is_dynamic_moored_and_damped():
    """Renamed from ..._welded_...: the weld WAS the bug.

    This asserted one welded tree (`joints == bodies - 1`), which is exactly the
    single rigid body that made a hit on one pipe swing all three. Each pipe is
    now moored to the world and hinged at its own base -- three sub-assemblies,
    six joints for six bodies. Per-pipe independence is asserted in
    test_prop_joints.py.
    """
    m = _model(pl.robosub_slalom(ROBOSUB))
    assert m.find('static').text == 'false'
    assert len(m.findall('joint')) >= len(_bodies(m)) - 1
    assert 'gz-sim-hydrodynamics-system' in pl.robosub_slalom(ROBOSUB)


def test_every_dynamic_prop_is_flagged_in_the_registry():
    """Otherwise it gets NO BUOYANCY and sinks, with nothing logged.

    The generator raises on this too; this catches it a build earlier.
    """
    for name, meta in pl.PROPS.items():
        xml = meta['build'](ROBOSUB if name.startswith('robosub') else SAUVC)
        is_static = '<static>true</static>' in xml
        assert meta['dynamic'] != is_static, (
            f'{name}: registry dynamic={meta["dynamic"]} but '
            f'model is {"static" if is_static else "dynamic"}')


def test_the_hull_cannot_drive_through_a_gate_sign_or_the_bin_pipework():
    gate = _model(pl.robosub_gate(ROBOSUB))
    signs = [l for l in _bodies(gate) if l.get('name').startswith('sign_')]
    assert signs and all(l.find('collision') is not None for l in signs)
    div = [l for l in _bodies(gate) if l.get('name') == 'divider'][0]
    assert div.find('collision') is not None, 'gate divider is drive-through'

    # The bins pipework is emitted by a local pipe() helper but the links are
    # named for what they are -- spine, arms, risers, feet. It was scenery on
    # the argument that nothing in the task pushes the frame; true of the task,
    # false of the vehicle, which descended straight through it.
    bins = _model(pl.robosub_bins(ROBOSUB))
    frame = [l for l in _bodies(bins)
             if l.get('name').startswith(('spine', 'arm_', 'riser_', 'foot'))]
    assert frame, 'no pipework links found -- did they get renamed?'
    assert all(l.find('collision') is not None for l in frame), \
        'the hull can occupy the bin frame'


def test_torpedo_board_draws_one_surface_not_two():
    """The z-fighting that hid the artwork: coplanar plate and face, both drawn."""
    m = _model(pl.robosub_torpedo_board(ROBOSUB))
    strips = [l for l in _bodies(m) if l.get('name').startswith('plate_c')]
    assert strips, 'no collision plate'
    assert all(l.find('visual') is None for l in strips), \
        'collision strips are drawn again over the printed face'
    board = [l for l in _bodies(m) if l.get('name') == 'board'][0]
    assert board.find('visual') is not None and board.find('collision') is None


def test_the_openings_you_see_are_the_openings_you_can_shoot_through():
    """One source of truth. The texture drew four; the collision cut two."""
    plate = pl.torpedo_openings(ROBOSUB)
    uv = pl.torpedo_openings_uv(ROBOSUB)
    # FOUR as of the 2026 layout, not two. What this test is really for is that
    # the two lists stay the SAME list -- they drifted apart once and a shot
    # lined up on the artwork struck solid board.
    assert len(plate) == len(uv) == 4
    size = ROBOSUB['props']['torpedo_board']['size']
    for (y, z, r), (u, v, ru) in zip(plate, uv):
        assert abs(ru - r / size) < 1e-9
        assert 0.0 < u < 1.0 and 0.0 < v < 1.0
    # different sizes, as the handbook says
    assert plate[0][2] != plate[1][2]

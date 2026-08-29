"""The virtual payload board's pure parts.

The end-to-end path (real driver -> PTY -> Gazebo body) is checked by
`duburi_sim_worlds/scripts/payload_check.py` against a running sim. These are
the invariants that do not need one, and each corresponds to a bug that was
live during development.
"""

import math

import pytest

from duburi_sim_bridge.payload_sim import (
    DROPPER_CHANNELS,
    SHOT_SLOTS,
    TORPEDO_CHANNELS,
    _projectile_sdf,
)


def test_shot_slots_match_the_world_generator():
    """The buoyancy whitelist is baked into every world at generation time.

    A projectile spawned under a name outside it gets no buoyancy at all and
    drops to the floor within half a second, silently. So this constant is not
    free to drift from PAYLOAD_SHOT_SLOTS in gen_world.py.
    """
    import os
    import re

    here = os.path.dirname(os.path.abspath(__file__))
    gen = os.path.join(here, '..', '..', 'duburi_sim_worlds', 'scripts',
                       'gen_world.py')
    if not os.path.exists(gen):
        pytest.skip('duburi_sim_worlds sources not present')
    with open(gen) as f:
        m = re.search(r'^PAYLOAD_SHOT_SLOTS\s*=\s*(\d+)', f.read(), re.M)
    assert m, 'PAYLOAD_SHOT_SLOTS not found in gen_world.py'
    assert int(m.group(1)) == SHOT_SLOTS


def test_channel_map_matches_the_real_driver():
    from duburi_control.payload import CHANNEL_NAMES

    for ch in TORPEDO_CHANNELS:
        assert CHANNEL_NAMES[ch].startswith('torpedo')
    for ch in DROPPER_CHANNELS:
        assert CHANNEL_NAMES[ch].startswith('dropper')
    assert set(TORPEDO_CHANNELS) | set(DROPPER_CHANNELS) == set(CHANNEL_NAMES)


def test_torpedo_is_near_neutral_by_displacement():
    """Mass is derived from displacement, not chosen.

    The projectile carries no added-mass model and only modest drag, so a round
    even a few grams heavy sinks visibly over its flight. Measured: 235 g (9 g
    negative) fell 0.81 m in 2 s and never reached the target.
    """
    from duburi_sim_bridge.payload_sim import (
        _TORPEDO_L, _TORPEDO_M, _TORPEDO_R,
    )

    displaced = math.pi * _TORPEDO_R ** 2 * _TORPEDO_L * 1000.0
    net = _TORPEDO_M - displaced
    assert 0.0 < net < 0.004, (
        f'torpedo is {net * 1000:.1f} g off neutral; positive-but-small is '
        f'the requirement (it must settle, not surface or plummet)')


def test_dropper_sinks():
    """A dropper that floats never lands in the bin."""
    from duburi_sim_bridge.payload_sim import _DROPPER_M, _DROPPER_R

    displaced = (4.0 / 3.0) * math.pi * _DROPPER_R ** 3 * 1000.0
    assert _DROPPER_M > displaced * 1.5


def test_drag_is_on_the_axes_the_round_actually_flies_along():
    """An SDF cylinder's length is its own z, so the round is pitched 90 deg.

    Body +z therefore runs along the flight path and body +x points DOWN.
    Writing the coefficients the obvious way round put the streamlined value on
    the sink and the broadside value on the flight: the shot travelled 0.14 m
    in 2 s instead of 2.47 m.
    """
    sdf = _projectile_sdf('payload_shot_0', '0 0 -1 0 1.5708 0', 'torpedo')

    def coeff(tag):
        import re
        return float(re.search(rf'<{tag}>([-\d.e]+)</{tag}>', sdf).group(1))

    flight, depth, side = coeff('zWabsW'), coeff('xUabsU'), coeff('yVabsV')
    assert flight < 0 and depth < 0 and side < 0, 'drag must oppose motion'
    assert abs(flight) < abs(depth), (
        'the flight axis carries more drag than the depth axis -- the '
        'coefficients are swapped')
    assert depth == pytest.approx(side), 'both broadside axes should match'


def test_sdf_names_the_model_and_is_well_formed():
    import xml.etree.ElementTree as ET

    for kind in ('torpedo', 'dropper'):
        sdf = _projectile_sdf('payload_shot_7', '1 2 -3 0 0 0', kind)
        root = ET.fromstring(sdf)
        model = root.find('model')
        assert model.get('name') == 'payload_shot_7'
        assert model.find('pose').text == '1 2 -3 0 0 0'
        assert model.find('link/collision') is not None

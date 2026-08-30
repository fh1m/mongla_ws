"""The virtual payload board's pure parts.

Projectile mass, drag and added mass live in test_projectile.py -- this file
covers the BOARD (slots, channels, wire protocol), that one covers what gets
fired out of it.

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


def test_sdf_names_the_model_and_is_well_formed():
    import xml.etree.ElementTree as ET

    for kind in ('torpedo', 'dropper'):
        sdf = _projectile_sdf('payload_shot_7', '1 2 -3 0 0 0', kind)
        root = ET.fromstring(sdf)
        model = root.find('model')
        assert model.get('name') == 'payload_shot_7'
        assert model.find('pose').text == '1 2 -3 0 0 0'
        assert model.find('link/collision') is not None

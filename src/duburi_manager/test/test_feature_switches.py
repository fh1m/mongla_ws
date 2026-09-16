"""Every feature switch on `bringup.launch.py` must default to what its node ships
AND must reach that node under the node's own parameter name.

These switches exist so each feature can be tested alone and in combination
(`ROADMAP.md` §9). A switch whose default differs from the node changes the
vehicle just by launching it; a switch that is declared but not forwarded is a
knob wired to nothing -- `ros2 launch ... zupt:=false` would run WITH ZUPT and
say nothing. Text-level on purpose, like `test_launch_and_node_defaults_agree`.
"""
import os
import re

import pytest

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_BRINGUP = os.path.join(_SRC, 'duburi_manager', 'launch', 'bringup.launch.py')
_VISION_PI = os.path.join(_SRC, 'duburi_vision', 'launch', 'vision_pi.launch.py')

# launch arg -> (node source, parameter name it must land on, where it is passed)
SWITCHES = {
    'velocity_uplink': ('duburi_manager/duburi_manager/auv_manager_node.py', 'velocity_uplink', _BRINGUP),
    'position_uplink': ('duburi_manager/duburi_manager/auv_manager_node.py', 'position_uplink', _BRINGUP),
    'mixer_aware':     ('duburi_manager/duburi_manager/vision_tunables.py', 'vision.mixer_aware', _BRINGUP),
    'zupt':            ('duburi_localization/duburi_localization/localization_node.py', 'zupt', _BRINGUP),
    'demand_aid':      ('duburi_localization/duburi_localization/localization_node.py', 'demand_aid', _BRINGUP),
    'use_yaw':         ('duburi_localization/duburi_localization/localization_node.py', 'use_yaw', _BRINGUP),
    'caustics':        ('duburi_vision/duburi_vision/flow/flow_node.py', 'caustic_suppression', _VISION_PI),
    'lane_lines':      ('duburi_vision/duburi_vision/flow/flow_node.py', 'lane_lines', _VISION_PI),
    'tile_m':          ('duburi_vision/duburi_vision/flow/flow_node.py', 'tile_m', _VISION_PI),
}


def _launch_default(path, arg):
    m = re.search(rf"DeclareLaunchArgument\(\s*'{arg}'\s*,\s*default_value=\s*'([^']*)'",
                  open(path).read())
    return None if m is None else m.group(1)


def _node_default(rel, param):
    src = open(os.path.join(_SRC, rel)).read()
    m = (re.search(rf"declare_parameter\(\s*'{re.escape(param)}'\s*,\s*([^),\n]+)", src)
         or re.search(rf"'{re.escape(param)}'\s*:\s*([^,\n]+),", src))
    assert m, f'{param} not declared in {rel}'
    return m.group(1).strip().strip("'\"")


def _same(a, b):
    try:
        return float(a) == float(b)
    except ValueError:
        return a.strip().lower() == b.strip().lower()


@pytest.mark.parametrize('arg', sorted(SWITCHES))
def test_the_switch_defaults_to_what_the_node_ships(arg):
    rel, param, _where = SWITCHES[arg]
    assert _launch_default(_BRINGUP, arg) is not None, f'bringup does not declare {arg}'
    assert _same(_launch_default(_BRINGUP, arg), _node_default(rel, param)), arg


@pytest.mark.parametrize('arg', sorted(SWITCHES))
def test_the_switch_reaches_its_node(arg):
    _rel, param, where = SWITCHES[arg]
    src = open(where).read()
    assert re.search(rf"'{re.escape(param)}'\s*:\s*(ParameterValue\(\s*)?"
                     rf"LaunchConfiguration\('{arg}'\)", src), \
        f'{arg} is declared but never passed to {param!r} in {os.path.basename(where)}'
    if where == _VISION_PI:
        # bringup must FORWARD it into the vision include, with the same default there
        assert re.search(rf"'{arg}'\s*:\s*LaunchConfiguration\('{arg}'\)", open(_BRINGUP).read()), arg
        assert _same(_launch_default(_VISION_PI, arg), _launch_default(_BRINGUP, arg)), arg

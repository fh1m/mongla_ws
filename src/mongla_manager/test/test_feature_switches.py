"""Every feature switch on `bringup.launch.py` must default to what its node ships
AND must reach that node under the node's own parameter name.

These switches exist so each feature can be tested alone and in combination
(`ROADMAP.md` §9). A switch whose default differs from the node changes the
vehicle just by launching it; a switch that is declared but not forwarded is a
knob wired to nothing -- `ros2 launch ... zupt:=false` would run WITH ZUPT and
say nothing. Text-level on purpose, like `test_launch_and_node_defaults_agree`.
"""
import os
import sys
import re

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'tools'))
from code_text import source_of  # noqa: E402

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_BRINGUP = os.path.join(_SRC, 'mongla_manager', 'launch', 'bringup.launch.py')
_VISION_PI = os.path.join(_SRC, 'mongla_vision', 'launch', 'vision_pi.launch.py')

# launch arg -> (node source, parameter name it must land on, where it is passed)
SWITCHES = {
    'velocity_uplink': ('mongla_manager/mongla_manager/node_setup.py', 'velocity_uplink', _BRINGUP),
    'position_uplink': ('mongla_manager/mongla_manager/node_setup.py', 'position_uplink', _BRINGUP),
    'mixer_aware':     ('mongla_manager/mongla_manager/vision_tunables.py', 'vision.mixer_aware', _BRINGUP),
    'zupt':            ('mongla_localization/mongla_localization/localization_node.py', 'zupt', _BRINGUP),
    'demand_aid':      ('mongla_localization/mongla_localization/localization_node.py', 'demand_aid', _BRINGUP),
    'use_yaw':         ('mongla_localization/mongla_localization/localization_node.py', 'use_yaw', _BRINGUP),
    'retrodict':       ('mongla_localization/mongla_localization/localization_node.py', 'retrodict', _BRINGUP),
    'caustics':        ('mongla_vision/mongla_vision/flow/flow_node.py', 'caustic_suppression', _VISION_PI),
    'lane_lines':      ('mongla_vision/mongla_vision/flow/flow_node.py', 'lane_lines', _VISION_PI),
    'tile_m':          ('mongla_vision/mongla_vision/flow/flow_node.py', 'tile_m', _VISION_PI),
}


def _launch_default(path, arg):
    m = re.search(rf"DeclareLaunchArgument\(\s*'{arg}'\s*,\s*default_value=\s*'([^']*)'",
                  open(path).read())
    return None if m is None else m.group(1)


def _node_default(rel, param):
    src = source_of(os.path.join(_SRC, rel))   # a node with its mixins
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
    # `flow_` is the prefix the vision process strips before handing a value
    # to the flow node composed inside it (`detector_dual_node.flow_overrides`).
    assert re.search(rf"'(flow_)?{re.escape(param)}'\s*:\s*(ParameterValue\(\s*)?"
                     rf"LaunchConfiguration\('{arg}'\)", src), \
        f'{arg} is declared but never passed to {param!r} in {os.path.basename(where)}'
    if where == _VISION_PI:
        # bringup must FORWARD it into the vision include, with the same default there
        assert re.search(rf"'{arg}'\s*:\s*LaunchConfiguration\('{arg}'\)", open(_BRINGUP).read()), arg
        assert _same(_launch_default(_VISION_PI, arg), _launch_default(_BRINGUP, arg)), arg


# Launch arguments that are not capability switches: what to load, where it
# is, how it looks. Everything else on bringup is a switch and needs a reason.
_NOT_SWITCHES = {
    'mav_device', 'payload_channels', 'vision_profile', 'fwd_model', 'dwn_model',
    'fwd_classes', 'dwn_classes', 'lock_class', 'camera', 'camera_profile',
    'model', 'models', 'active_model', 'classes', 'conf', 'imgsz', 'max_det',
    'batighor_port', 'foxglove_port',
}
_ROADMAP = os.path.join(_SRC, '..', '.claude', 'context', 'ROADMAP.md')


def _switch_rows():
    text = open(_ROADMAP).read()
    sec = text[text.index('### Launch (`ros2 launch mongla_manager bringup'):]
    sec = sec[:sec.index('### Live')]
    rows = {}
    for line in sec.splitlines():
        cells = [c.strip() for c in line.strip().strip('|').split('|')]
        if len(cells) != 5 or not cells[0].startswith('`'):
            continue
        for name in re.findall(r'`([a-z_0-9]+)`', cells[0]):
            rows[name] = (cells[1].strip('`'), cells[4])
    return rows


def test_every_bringup_switch_says_why_it_has_its_default():
    """A mission runs every sensor, so a switch is off only for a written
    reason. This is the check that a default was decided, not inherited:
    flow and the lock ladder sat off for months on reasons that had gone
    stale (2026-11-18)."""
    text = open(_BRINGUP).read()
    declared = dict(re.findall(
        r"DeclareLaunchArgument\(\s*'([a-z_0-9]+)'\s*,\s*default_value=\s*'([^']*)'", text))
    rows = _switch_rows()
    problems = []
    for arg, default in sorted(declared.items()):
        if arg in _NOT_SWITCHES:
            continue
        if arg not in rows:
            problems.append(f'{arg}: no row in ROADMAP.md §9')
            continue
        shown, why = rows[arg]
        if shown != default:
            problems.append(f'{arg}: ROADMAP says {shown!r}, the launch {default!r}')
        if len(why) < 20:
            problems.append(f'{arg}: no reason for its default')
    assert not problems, '\n'.join(problems)

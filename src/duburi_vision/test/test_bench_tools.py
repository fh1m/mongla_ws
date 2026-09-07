"""The bench tools must not lie to the node they measure.

⛔ WHY THIS FILE EXISTS. `flow_launch_check.py` and `flow_derot_ab.py`
publish a synthetic `/duburi/state` so `flow_node` has a height on a bench
with no barometer. One of them also filled in `yaw_deg = 0.0`.

The MANAGER publishes the same topic, carrying the board's real heading --
about 180 deg on this rig. The node keeps whichever message arrived last, and
`DistanceAccumulator` folds `vx*cos(e) - vy*sin(e)` with `e = yaw -
axis_yaw`. A 180 deg alternation flips `cos(e)` between +1 and -1 EVERY
INTERVAL, so successive contributions subtract instead of accumulating.

Cost: 30 cm slides read +0.64, +0.81, -1.73 and -2.99 cm with an unstable
sign, and FOUR causes were investigated and refuted -- gyro noise, axis
mapping, de-rotation gains, projection axis -- before anyone suspected the
instrument. After the fix the same slide read 31.94 cm, 106.5 %.

ZERO IS NOT "UNSET". It is a confident claim of due north. The node skips a
NaN field, so NaN is the only honest thing a fake state message can say
about a quantity it does not measure.

READS THE FILES, NEVER IMPORTS THEM -- these are `tools/` scripts with ROS
imports at module scope, and a test that imports in a worktree resolves
against the main workspace's stale `install/` tree.
"""
import pathlib
import re

import pytest

_TOOLS = pathlib.Path(__file__).resolve().parents[3] / 'tools'


def _state_publishers():
    """Tools that construct a DuburiState to publish."""
    if not _TOOLS.is_dir():
        pytest.skip('tools/ not present')
    out = []
    for py in sorted(_TOOLS.glob('*.py')):
        src = py.read_text()
        if 'DuburiState()' in src and '/duburi/state' in src:
            out.append((py.name, src))
    return out


def test_a_synthetic_state_never_asserts_a_yaw_it_does_not_measure():
    tools = _state_publishers()
    assert tools, ('no tool publishes a synthetic /duburi/state any more -- '
                   'if that is deliberate, delete this test with the reason')
    bad = []
    for name, src in tools:
        for m in re.finditer(r'^\s*\w+\.yaw_deg\s*=\s*(.+?)\s*$',
                             src, re.M):
            val = m.group(1)
            if 'nan' not in val.lower():
                bad.append(f'{name}: yaw_deg = {val}')
    assert not bad, (
        'a bench tool publishes a CONCRETE yaw into /duburi/state:\n  ' +
        '\n  '.join(bad) +
        '\n\nThe manager publishes that topic too. Two publishers '
        'disagreeing by 180 deg\nflip the distance projection sign every '
        'interval and cancel the travel --\nmeasured, and it cost four '
        'wrong diagnoses. Publish NaN.')


def test_depth_is_still_supplied_or_the_node_refuses():
    """The counterpart. Depth SHOULD be concrete: the node refuses without a
    height, and on this bench nothing else provides one -- the manager
    publishes NaN depth because no barometer is fitted. So the honest split
    is: state what you measured (depth, from the operator's tape), NaN what
    you did not (yaw, which comes from the board)."""
    tools = _state_publishers()
    assert tools
    for name, src in tools:
        assert re.search(r'^\s*\w+\.depth_m\s*=\s*[^n]', src, re.M), (
            f'{name} stopped supplying a depth; flow_node will refuse every '
            f'interval with "no depth yet, so no height above the floor"')

"""A calibration must name the camera it describes, and the launch must agree.

⛔ WHY THIS FILE EXISTS. The only calibration we hold was named
`pi_forward_1280x720.json` and wired to the FORWARD camera, while its own
metadata read `pi_test_global_shutter (Microdia USB)` -- USB vendor 0c45, the
SONIX unit, which is the DOWNWARD camera. It was captured 2026-09-03, four
days before the udev rules were found to have the two cameras SWAPPED, so it
was named for the camera the system then believed it was looking at.

Both halves were live and neither logged anything:

  - the FORWARD Fantech published CameraInfo with a 63.82 deg HFOV belonging
    to a different lens, so every pixel->bearing on the srot vision uplink
    used the wrong focal length;
  - the DOWNWARD camera -- the DVL, whose intrinsics round 38 measured a
    3.08 % axis asymmetry to correct -- got NO calibration at all, so
    flow_node fell back to a single focal length and the frame centre. That
    fix was verified in a tool which passed the path by hand, and never
    reached the launch.

This is the FOURTH config in this package measured to reach nothing
(`device_path` into `**_`, the unloaded YAML profile table, `ros2 param set`
on construction-time params), and the round-32 plan predicted a fourth.

READS THE FILES, NEVER IMPORTS THEM: a test that imports in a worktree
resolves against the main workspace's stale `install/` tree, which is how the
CLAHE retraction nearly went the wrong way.
"""
import json
import pathlib
import re

import pytest

_PKG = pathlib.Path(__file__).resolve().parents[1]
_CAL_DIR = _PKG / 'config' / 'calibration'
_LAUNCH = _PKG / 'launch' / 'vision_pi.launch.py'


def _calib_files():
    return sorted(_CAL_DIR.glob('*.json'))


def test_every_calibration_declares_which_camera_it_describes():
    files = _calib_files()
    assert files, 'no calibration files found -- did the path move?'
    for f in files:
        d = json.loads(f.read_text())
        assert d.get('applies_to'), (
            f'{f.name} does not declare `applies_to`. Without it nothing can '
            f'tell which camera it belongs to, which is exactly how it came '
            f'to be wired to the wrong one.')
        assert isinstance(d['applies_to'], list) and d['applies_to']


def test_the_launch_wires_each_calibration_to_a_camera_it_CLAIMS():
    """The guard proper. Parse the launch's own defaults rather than trusting
    the filename -- the filename is the thing that was wrong."""
    src = _LAUNCH.read_text()
    # DeclareLaunchArgument('<side>_calibration', default_value=_calib('X'))
    pat = re.compile(
        r"DeclareLaunchArgument\(\s*\n?\s*'(fwd|dwn)_calibration',\s*\n?\s*"
        r"default_value=(?:_calib\('([^']+)'\)|'')", re.M)
    found = dict((m.group(1), m.group(2)) for m in pat.finditer(src))
    assert set(found) == {'fwd', 'dwn'}, (
        f'could not parse both calibration arguments from '
        f'{_LAUNCH.name}; found {found}')
    side_to_profile = {'fwd': 'pi_forward', 'dwn': 'pi_downward'}
    for side, fname in found.items():
        if not fname:
            continue                      # '' = deliberately uncalibrated
        f = _CAL_DIR / fname
        assert f.exists(), f'{side}_calibration names a missing file {fname}'
        applies = json.loads(f.read_text()).get('applies_to') or []
        want = side_to_profile[side]
        assert want in applies, (
            f'{_LAUNCH.name} wires {fname} to the {side.upper()} camera, but '
            f'that file declares applies_to={applies} and does NOT list '
            f'{want!r}. This is the exact defect the file exists to catch.')


def test_the_downward_camera_IS_calibrated():
    """The DVL's intrinsics are not optional. Round 38 measured a 3.08 % axis
    asymmetry caused by assuming fx==fy, the frame centre as the principal
    point, and no undistortion -- all three come from this file."""
    src = _LAUNCH.read_text()
    m = re.search(r"'dwn_calibration',\s*\n?\s*default_value=_calib\('([^']+)'\)",
                  src)
    assert m, ('the downward camera has no calibration wired. flow_node then '
               'falls back to one focal length and the frame centre, which '
               'is measured to cost 3.08 % of axis asymmetry.')
    assert (_CAL_DIR / m.group(1)).exists()


def test_the_flow_node_is_actually_LAUNCHABLE():
    """flow_node -- the DVL -- existed only as a setup.py entry point and
    appeared in NO launch file, so the bottom-camera velocity sensor had to
    be started by hand. On a pool deck that means it does not get started.

    It must also receive the SAME calibration the downward camera gets;
    passing them from two places is how they came to disagree in the first
    place (see the module docstring)."""
    src = _LAUNCH.read_text()
    assert "executable='flow_node'" in src, (
        'flow_node is in no launch file -- the DVL cannot be brought up with '
        'the rest of the vision stack.')
    assert "'pool_depth_m'" in src, (
        'flow_node is launched without pool_depth_m. It refuses to publish '
        'velocity without it, so the node would come up and stay silent.')
    # The calibration must be the downward one, by reference not by literal.
    flow = src[src.index("executable='flow_node'"):]
    flow = flow[:flow.index('condition=')]
    assert "LaunchConfiguration('dwn_calibration')" in flow, (
        'flow_node must take the SAME dwn_calibration the downward camera '
        'takes, not its own copy of the path.')

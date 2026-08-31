"""No camera offset may put the view above the water surface.

This has now bitten three rounds running, and the failure always looks the same:
the run opens on sky, or `view high` shows the underside of the waves, and it is
found by looking at a screenshot rather than by anything failing.

The arithmetic is simple enough that a test should own it:

  * the water surface is an OPAQUE model (Fuel openrobotics/waves) at z = 0;
  * every course spawns the hull at -0.4 m (Round 7's depth-reference fix);
  * the follow offset is applied in the HULL'S frame, so camera z is
    spawn_z + offset_z.

So any offset z >= 0.4 surfaces the camera at spawn, and the margin has to be
taken at the SPAWN depth rather than at a run depth -- the opening frame is the
one an operator sees first.
"""
import os
import re

import pytest

BRINGUP = os.path.join(os.path.dirname(__file__), '..', '..',
                       'duburi_sim_bringup')

SPAWN_Z = -0.4          # every course, since Round 7
SURFACE_Z = 0.0
MARGIN = 0.1            # do not skim the surface either


def _gui_config():
    path = os.path.join(BRINGUP, 'config', 'gui.config')
    if not os.path.isfile(path):
        pytest.skip('gui.config not found')
    with open(path) as fh:
        return fh.read()


def test_startup_camera_pose_is_underwater():
    """`camera_pose` is a WORLD pose, not an offset -- it must be below z=0 on
    its own. It was +0.9, and that is what made every run open on sky."""
    m = re.search(r'<camera_pose>([-\d.]+) ([-\d.]+) ([-\d.]+)', _gui_config())
    assert m, 'no camera_pose in gui.config'
    z = float(m.group(3))
    assert z < SURFACE_Z - MARGIN, (
        f'camera_pose z={z} is at or above the water surface; the opening '
        f'frame will be sky')


def test_follow_offset_keeps_the_camera_under_at_spawn():
    m = re.search(r'<follow_offset>([-\d.]+) ([-\d.]+) ([-\d.]+)</follow_offset>',
                  _gui_config())
    assert m, 'no follow_offset in gui.config'
    dz = float(m.group(3))
    assert SPAWN_Z + dz < SURFACE_Z - MARGIN, (
        f'follow_offset z={dz} puts the camera at {SPAWN_Z + dz:+.2f} at the '
        f'{SPAWN_Z} m spawn -- above the water')


def test_view_presets_keep_the_camera_under_at_spawn():
    """`view high` and `view far` carry their own offsets and were missed when
    the shared one was fixed."""
    path = os.path.join(BRINGUP, 'scripts', 'duburi_sim')
    if not os.path.isfile(path):
        pytest.skip('duburi_sim not found')
    with open(path) as fh:
        script = fh.read()
    offsets = re.findall(r'_off="x: [-\d.]+, y: [-\d.]+, z: ([-\d.]+)"', script)
    assert offsets, 'no view presets found -- did they move?'
    for raw in offsets:
        dz = float(raw)
        assert SPAWN_Z + dz < SURFACE_Z - MARGIN, (
            f'a view preset offset z={dz} surfaces the camera at the spawn')

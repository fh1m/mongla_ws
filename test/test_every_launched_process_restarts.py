"""Every process bringup can start comes back if it dies mid-run.

The owner's policy (2026-11-10) is written once, in `launch_respawn`. This
guard makes it a rule for bringup and every launch file it includes, so a node added
later as a bare `Node(...)` fails here instead of quietly having no restart.

A process may be left out only with a reason written below.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
from launch_ros.actions import Node

SRC = Path(__file__).resolve().parents[1] / 'src'

#: Bringup, the vision launch files it includes (`vision_stack`), Batighor,
#: and the calibration launch that runs Batighor in its place.
BRINGUP_STACKS = ('mongla_manager/launch/bringup.launch.py',
                  'mongla_vision/launch/vision_pi.launch.py',
                  'mongla_vision/launch/vision.launch.py',
                  'mongla_batighor/launch/batighor.launch.py',
                  'mongla_batighor/launch/calibrate.launch.py')

#: Executables deliberately not restarted, each with its reason.
NOT_RESTARTED = {
    'vision_display': 'an operator window: closing it is a choice, and in '
                      'vision.launch.py closing it ends the group on purpose',
}


def _render(name: str):
    spec = importlib.util.spec_from_file_location(Path(name).stem.replace('.', '_'), SRC / name)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.generate_launch_description().entities


def _nodes(entities):
    return [e for e in entities if isinstance(e, Node)]


@pytest.mark.parametrize('name', BRINGUP_STACKS)
def test_every_process_has_a_restart_or_a_written_reason(name):
    bare = sorted(str(n.node_executable) for n in _nodes(_render(name))
                  if not getattr(n, 'restarts_on_exit', None)
                  and n.node_executable not in NOT_RESTARTED)
    assert not bare, (
        f'{name} starts {bare} with no restart. Wrap it in '
        f'`respawning(...)` (mongla_localization/launch_respawn.py), or add it '
        f'to NOT_RESTARTED with the reason.')


@pytest.mark.parametrize('name', BRINGUP_STACKS)
def test_the_guard_sees_the_processes_it_judges(name):
    # A render that finds no Node would pass the test above vacuously.
    # Two: bringup itself starts the manager and localization; the rest it includes.
    assert len(_nodes(_render(name))) >= 2


def test_every_exemption_is_still_launched():
    seen = {n.node_executable for name in BRINGUP_STACKS for n in _nodes(_render(name))}
    assert set(NOT_RESTARTED) <= seen, set(NOT_RESTARTED) - seen

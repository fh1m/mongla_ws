"""Every launch file an operator can type renders, under every switch.

`test_every_switch_combination_renders.py` walks bringup, singly and in pairs.
The vision launch files are also typed directly -- `ros2 launch mongla_vision
vision_dual.launch.py` on the bench, `video.launch.py` against footage -- and a
switch that raises in one of them shows only when someone types it. So each is
rendered here under its defaults and under every value of every switch it
declares, the way `ros2 launch` would, without starting a process.
"""
from __future__ import annotations

import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools'))
from launch_render import render, source_tree, switches  # noqa: E402

LAUNCH = sorted((ROOT / 'src').glob('*/launch/*.launch.py'))
# What a file needs before it starts anything; it refuses loudly without.
REQUIRED = {'video.launch.py': {'fwd_video': '/footage/gate.mp4'}}
CASES = []
with source_tree():
    for lf in LAUNCH:
        base = REQUIRED.get(lf.name, {})
        CASES.append((lf, dict(base)))
        for name, values in switches(lf).items():
            CASES += [(lf, {**base, name: v}) for v in values]


@pytest.fixture(autouse=True)
def _source_tree():
    with source_tree():
        yield


def test_every_launch_file_is_covered():
    names = {p.name for p in LAUNCH}
    assert {'bringup.launch.py', 'vision.launch.py', 'vision_dual.launch.py',
            'vision_pi.launch.py', 'video.launch.py', 'mission_web.launch.py'} <= names, names


@pytest.mark.parametrize('launch, args', CASES,
                         ids=[f"{lf.stem}:{','.join(f'{k}={v}' for k, v in a.items()) or 'defaults'}"
                              for lf, a in CASES])
def test_it_renders(launch, args):
    nodes = render(args, launch)
    assert nodes, f'{launch.name} {args} starts nothing'
    assert not [n for n in nodes if n.startswith('?')], nodes


def test_video_without_footage_refuses_and_says_how():
    lf = next(p for p in LAUNCH if p.name == 'video.launch.py')
    with pytest.raises(RuntimeError, match='fwd_video:='):
        render({}, lf)

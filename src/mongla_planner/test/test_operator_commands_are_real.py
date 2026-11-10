"""Every `ros2 launch` / `ros2 run` line written anywhere in the repo is real.

`ros2 launch` ignores an undeclared `name:=value` without a word, so a doc
with a stale argument does not fail when you run it. The run just goes ahead
without that setting. This test runs `tools/ci/command_lint.py` over every doc
and source string and fails on any finding: a missing launch file, an
undeclared argument, a value outside its choices, a model with no sidecar, a
verb flag that is not a field, a mission that does not exist, or an
unguarded verb that srot refuses.
"""
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools'))  # shared libraries
sys.path.insert(0, str(ROOT / 'tools' / 'ci'))

import command_lint  # noqa: E402


def test_no_command_in_the_repo_is_wrong():
    found = command_lint.lint()
    assert not found, '\n'.join(
        f'{f.path}:{f.line}: {f.problem}\n    {f.command[:120]}' for f in found)


def test_the_linter_catches_what_it_exists_for():
    """Injection: each class of defect must produce a finding."""
    ck = command_lint.Checker()
    bad = [
        'ros2 launch mongla_manager bringup.launch.py no_such_arg:=1',
        'ros2 launch mongla_manager bringup.launch.py vision_stack:=jetson',
        'ros2 launch mongla_manager bringup.launch.py fwd_model:=no_such_model',
        'ros2 launch mongla_manager no_such.launch.py',
        'ros2 run mongla_manager no_such_exe',
        'ros2 run mongla_planner mission no_such_mission',
        'ros2 run mongla_planner mongla fire --channel 1',
        'ros2 run mongla_planner mongla lock_heading --target 0',
    ]
    for cmd in bad:
        assert ck.check(cmd, 'operator doc', ''), f'not caught: {cmd}'
    assert not ck.check('ros2 launch mongla_manager bringup.launch.py vision:=true',
                        'operator doc', '')

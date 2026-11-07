"""Every `ros2 launch` line written in the docs, the site and the source renders.

`test_operator_commands_are_real` checks each argument NAME on such a line.
That is not the same as the line working: a launch file can refuse a
combination of arguments that each exist (`video.launch.py` with no footage, a
model with no sidecar), and the operator finds out on the day, at the pool,
with the clock running. So each documented line is rendered here exactly as
written -- arguments, conditions, includes, down to the nodes it would start --
and a refusal is a failure, with the file and line it came from.

The settings file is switched off (`MONGLA_BRINGUP=none`) so the result is the
line's own, not whatever ~/.mongla/bringup.yaml says on this machine. A line
with a placeholder (`<pi>`, `$VAR`, `...`) is not a command and is skipped.
"""
import pathlib
import shlex
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools'))
import command_lint  # noqa: E402
from launch_render import render, source_tree  # noqa: E402


def _documented():
    """{(pkg, file, args): first 'path:line' it appears at}."""
    out = {}
    for path in command_lint.sources():
        try:
            text = path.read_text()
        except (OSError, UnicodeDecodeError):
            continue
        for line, cmd in command_lint.commands_in(text):
            if not cmd.startswith('ros2 launch ') or command_lint._PLACEHOLDER.search(cmd):
                continue
            try:
                tok = shlex.split(cmd)
            except ValueError:
                continue
            if len(tok) < 4 or not tok[3].endswith('.launch.py'):
                continue
            args = tuple(t for t in tok[4:] if ':=' in t)
            if len(args) != len(tok[4:]):
                continue                      # a pipe, a flag or prose after it
            key = (tok[2], tok[3], args)
            out.setdefault(key, f'{path.relative_to(ROOT)}:{line}')
    return out


DOCUMENTED = _documented()


def test_the_docs_hold_enough_launch_lines_to_mean_something():
    """If the extraction broke, every test below would pass on nothing."""
    assert len(DOCUMENTED) >= 20, sorted(DOCUMENTED)


@pytest.mark.parametrize('key', sorted(DOCUMENTED), ids=lambda k: ' '.join((k[1],) + k[2]))
def test_a_documented_launch_line_renders(key, monkeypatch):
    pkg, fname, args = key
    launch = ROOT / 'src' / pkg / 'launch' / fname
    monkeypatch.setenv('MONGLA_BRINGUP', 'none')
    monkeypatch.setattr(sys, 'argv', ['ros2', 'launch', pkg, fname, *args])
    values = dict(a.split(':=', 1) for a in args)
    values = {k: v.strip('"\'') for k, v in values.items()}
    with source_tree():
        try:
            started = render(values, launch)
        except Exception as exc:                                  # noqa: BLE001
            pytest.fail(f'{DOCUMENTED[key]}: `ros2 launch {pkg} {fname} '
                        f'{" ".join(args)}` refuses: {type(exc).__name__}: {exc}')
    assert started, f'{DOCUMENTED[key]}: renders but starts no node'

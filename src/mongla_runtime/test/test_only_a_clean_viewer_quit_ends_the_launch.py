"""The viewer's exit ends a launch only on a clean quit.

On a tetherless run a forwarded viewer dies when the tether is pulled; its
exit must not take cameras and detectors with it. Exercised through the real
launch event handler and a real ProcessExited event, not by reading source.
"""
import pytest
from launch import LaunchContext
from launch.actions import EmitEvent, ExecuteProcess
from launch.events.process import ProcessExited

from mongla_runtime.launch_defaults import has_screen, viewer_quit_shuts_launch


def _handled(code):
    viewer = ExecuteProcess(cmd=['true'], name='viewer')
    handler = viewer_quit_shuts_launch(viewer).event_handler
    ev = ProcessExited(returncode=code, action=viewer, name='viewer', cmd=['true'],
                       cwd=None, env=None, pid=1)
    assert handler.matches(ev)
    out = handler.handle(ev, LaunchContext()) or []
    return any(isinstance(a, EmitEvent) for a in out)


@pytest.mark.parametrize('code,shuts', [(0, True), (-6, False), (1, False)])
def test_only_exit_zero_shuts(code, shuts):
    assert _handled(code) is shuts


@pytest.mark.parametrize('env,want', [
    ({}, 'false'),
    ({'DISPLAY': ':0'}, 'true'),
    ({'DISPLAY': 'localhost:10.0'}, 'false'),           # ssh -X: the tether
    ({'WAYLAND_DISPLAY': 'wayland-0'}, 'true'),
])
def test_only_a_local_screen_counts(env, want):
    assert has_screen(env) == want

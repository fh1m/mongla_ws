"""A process that dies mid-run comes back three times, then stays down.

Owner policy (2026-11-10), implemented once in `launch_respawn`: restart after
a delay, at most `MAX_RESTARTS` times; never restart a process that exited 0
or one that is stopping because launch itself is shutting down.

These run real processes under a real `LaunchService`, each appending a line to
a file when it starts, so the count is what launch actually did.
"""
from __future__ import annotations

import threading
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, TimerAction
from launch.actions import Shutdown

from mongla_localization import launch_respawn
from mongla_localization.launch_respawn import respawning


def _starts(tmp_path: Path, exit_code: int, *, sleep_s: float = 0.0,
            max_restarts: int = 3, extra=()) -> int:
    log = tmp_path / 'starts'
    script = f'echo x >> {log}; sleep {sleep_s}; exit {exit_code}'

    def make():
        return ExecuteProcess(cmd=['sh', '-c', script], output='log')

    ls = LaunchService()
    ls.include_launch_description(LaunchDescription(
        [*respawning(make, 'the test process', max_restarts=max_restarts,
                     delay_s=0.05), *extra]))
    # Launch ends on its own once nothing is left running.
    guard = threading.Timer(20.0, ls.shutdown)
    guard.start()
    try:
        ls.run()
    finally:
        guard.cancel()
    return len(log.read_text().splitlines()) if log.exists() else 0


def test_a_crashing_process_runs_once_plus_three_restarts(tmp_path):
    assert launch_respawn.MAX_RESTARTS == 3
    assert _starts(tmp_path, exit_code=3) == 1 + 3


def test_the_limit_is_the_argument_not_a_hidden_constant(tmp_path):
    assert _starts(tmp_path, exit_code=1, max_restarts=1) == 2


def test_a_clean_exit_is_never_restarted(tmp_path):
    assert _starts(tmp_path, exit_code=0) == 1


def test_shutting_launch_down_restarts_nothing(tmp_path):
    # The process would crash, but launch is shut down while it runs: Ctrl-C
    # must stop the stack, not start a second copy of it.
    stop = TimerAction(period=0.5, actions=[Shutdown(reason='operator stop')])
    assert _starts(tmp_path, exit_code=3, sleep_s=5.0, extra=(stop,)) == 1

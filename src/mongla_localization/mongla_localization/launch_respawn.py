"""Restart a vehicle process that dies mid-run, three times, then leave it down.

The owner's policy (2026-11-10): a process that dies during a run is restarted
after 2 s and the mission keeps running. After three restarts it stays down and
says so; nothing ends the mission on this alone. It covers every process
bringup starts, and `test/test_every_launched_process_restarts.py` makes that
a rule. Why each is safe to restart:

- vision: the verbs already treat silent detections as a target lost and
  re-acquire; a detector that stays down reads FAIL on the manager's
  `detector` health row ("publishing nothing").
- the manager: its teardown still tries the disarm, and a new manager that
  hears ARMED latches and refuses every verb but disarm/stop/surface until the
  board reports DISARMED (`node_support.boot_latch`). A restart gives back the
  only host disarm path and moves nothing.
- localization: a restarted filter publishes in frame `odom` until a heading
  anchor arrives, and `dsl_position.pose()` refuses that frame, so a mission
  reads "unknown" rather than a pose at the origin.

Why a helper and not `respawn=True`. Humble's launch has no restart limit
(`respawn_max_retries` arrived in Iron), and the dev box and CI run Humble
while the vehicle runs Jazzy; an unbounded respawn of a detector whose Hailo
is really gone would restart it for the whole run. One implementation for both
distros, written here once.

What it does not restart: a process that exits 0 (it chose to finish). Ctrl-C
needs no check here: launch cancels a pending `TimerAction` on shutdown and
refuses to start a process once shutdown has begun, and the test proves the
stack stops with nothing restarted.
"""
from __future__ import annotations

from typing import Callable

from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit

MAX_RESTARTS = 3
DELAY_S = 2.0


def respawning(make: Callable[[], ExecuteProcess], label: str, *,
               max_restarts: int = MAX_RESTARTS, delay_s: float = DELAY_S) -> list:
    """Launch actions that run `make()` and restart it on a failing exit.

    `make` returns a FRESH launch_ros `Node` each call: a launch action runs
    once, so each restart needs its own. Returns a list to splice into a
    LaunchDescription.
    """
    restarts = [0]

    def one() -> list:
        node = make()
        # The mark the launch-file guard reads
        # (`test/test_every_launched_process_restarts.py`).
        node.restarts_on_exit = label

        def on_exit(event, _context):
            if event.returncode == 0:
                return None
            if restarts[0] >= max_restarts:
                return [LogInfo(msg=(
                    f'ERROR: {label} exited ({event.returncode}) after '
                    f'{max_restarts} restarts; it stays down for this run'))]
            restarts[0] += 1
            return [LogInfo(msg=(
                        f'WARNING: {label} exited ({event.returncode}); restart '
                        f'{restarts[0]}/{max_restarts} in {delay_s:.0f} s')),
                    TimerAction(period=delay_s, actions=one())]

        return [node, RegisterEventHandler(
            OnProcessExit(target_action=node, on_exit=on_exit))]

    return one()

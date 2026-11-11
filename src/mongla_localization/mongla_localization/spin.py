"""One place that decides which rclpy executor a Mongla node spins on.

rclpy's default executor rebuilds a wait set for every callback it delivers.
The events executor (Jazzy's `rclpy.experimental.EventsExecutor`, C++
underneath) is told by the middleware when something is ready. Measured on
Kutum with `tools/bench/executor_cost.py` (measured-bars 73): 0.037 ms of CPU
per callback against 0.251 ms for the default and 0.534 ms for the
multi-threaded one, and a publish-to-callback p99 of 0.07 ms against 0.35 ms.

It is single-threaded, so it fits a node whose callbacks return quickly. The
manager stays on the multi-threaded executor: its action callbacks block for
the length of a move while the telemetry and health timers keep running.

`MONGLA_EXECUTOR` chooses: `events` (the default), or `classic` for the
executor each node used before. Humble has no events executor, so a dev box
gets the classic one and says so once.

It lives in `mongla_localization` for the reason `params_file` does: it is
the one package every node package already depends on.
"""
from __future__ import annotations

import os

ENV = 'MONGLA_EXECUTOR'
CHOICES = ('events', 'classic')


def choice(env=None) -> str:
    """The executor asked for; a value that is not a choice is refused."""
    value = (os.environ if env is None else env).get(ENV, 'events').strip().lower()
    if value not in CHOICES:
        raise ValueError(f'{ENV}={value!r}: expected one of {", ".join(CHOICES)}')
    return value


def make_executor(logger=None, classic=None, env=None):
    """The executor to spin on, and its name.

    `classic` builds the executor the caller used before (default: the
    single-threaded one). It is returned when `MONGLA_EXECUTOR=classic`, and
    when this rclpy has no events executor.
    """
    from rclpy.executors import SingleThreadedExecutor
    classic = classic or SingleThreadedExecutor
    if choice(env) == 'classic':
        return classic(), 'classic'
    try:
        from rclpy.experimental import EventsExecutor
    except ImportError:
        if logger is not None:
            logger.info('[EXEC ] this rclpy has no events executor (Humble); '
                        'spinning on the classic one')
        return classic(), 'classic'
    return EventsExecutor(), 'events'


def spin(*nodes, classic=None) -> None:
    """`rclpy.spin`, on the chosen executor, for one node or several."""
    ex, name = make_executor(nodes[0].get_logger(), classic)
    for n in nodes:
        ex.add_node(n)
    nodes[0].get_logger().info(f'[EXEC ] spinning on the {name} executor')
    try:
        ex.spin()
    finally:
        ex.shutdown()

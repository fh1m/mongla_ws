#!/usr/bin/env python3
"""DuburiClient -- blocking Python API over the /duburi/move ActionServer.

The whole API is one method: `send(cmd, **fields)`. It blocks until the
goal succeeds, fails, or times out. On rejection it raises
`MoveRejected`; on a server-side failure it raises `MoveFailed`. On
success it returns the `Move.Result` so callers can inspect
`final_value` (depth or yaw) and `error_value` (heading error or depth
error).

For ergonomics, attribute access on the client is sugar for `send`:

    client.move_forward(duration=5, gain=60)

is identical to:

    client.send('move_forward', duration=5, gain=60)

The dynamic shortcut is checked against the `COMMANDS` registry, so a
typo (e.g. `client.mvoe_forward(...)`) raises AttributeError instead of
silently sending an unknown goal.
"""

import rclpy
from rclpy.action import ActionClient

from duburi_interfaces.action import Move
from duburi_control           import COMMANDS


class MoveRejected(RuntimeError):
    """Action server returned GoalResponse.REJECT (e.g. another command
    is already active)."""


class MoveFailed(RuntimeError):
    """Action server accepted the goal but Move.Result.success was False
    (timeout, mode rejected, exception, ...)."""


class DuburiClient:
    def __init__(self, node):
        self.node    = node
        self._client = ActionClient(node, Move, '/duburi/move')

    # ------------------------------------------------------------------ #
    #  Connection                                                         #
    # ------------------------------------------------------------------ #

    def wait_for_connection(self, timeout=15.0):
        self.node.get_logger().info('Waiting for /duburi/move action server...')
        if not self._client.wait_for_server(timeout_sec=timeout):
            raise TimeoutError('/duburi/move action server not available')
        self.node.get_logger().info('Action server ready.')

    # ------------------------------------------------------------------ #
    #  Single dispatch point                                              #
    # ------------------------------------------------------------------ #

    def send(self, cmd, **fields):
        """Send a Move goal and block until it completes.

        Returns a `Move.Result`. Raises `MoveRejected` if the server
        refuses the goal, `MoveFailed` if the server accepted but
        `result.success == False`.

        `fields` map directly onto `Move.Goal` field names: any of
        `duration`, `gain`, `target`, `target_name`, `timeout`. Unset
        fields stay at their rosidl defaults (0.0 / '').
        """
        if cmd not in COMMANDS:
            raise ValueError(
                f"Unknown command '{cmd}'. Known: {sorted(COMMANDS)}")

        goal = Move.Goal()
        goal.cmd = cmd
        for name, value in fields.items():
            if value is None:
                continue
            setattr(goal, name, value)

        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self.node, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            raise MoveRejected(f'Goal "{cmd}" was REJECTED by action server')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result

        if not result.success:
            raise MoveFailed(f'Goal "{cmd}" FAILED: {result.message}')
        return result

    # ------------------------------------------------------------------ #
    #  Sugar: client.move_forward(duration=5) -> client.send('move_forward', ...)
    # ------------------------------------------------------------------ #

    def __getattr__(self, name):
        if name in COMMANDS:
            return lambda **fields: self.send(name, **fields)
        raise AttributeError(
            f'{type(self).__name__!r} object has no attribute {name!r}')

    # ------------------------------------------------------------------ #
    #  Feedback                                                           #
    # ------------------------------------------------------------------ #

    def _on_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.get_logger().debug(
            f'[FB] {feedback.phase}  {feedback.status_line}')

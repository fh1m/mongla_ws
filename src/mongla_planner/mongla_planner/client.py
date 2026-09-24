#!/usr/bin/env python3
"""MonglaClient -- blocking Python API over the /mongla/move ActionServer.

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

from mongla_interfaces.action import Move
from mongla_control           import COMMANDS


class MoveRejected(RuntimeError):
    """Action server returned GoalResponse.REJECT (e.g. another command
    is already active)."""


class MoveFailed(RuntimeError):
    """Action server accepted the goal but Move.Result.success was False
    (timeout, mode rejected, exception, ...)."""


class TaskAbandoned(RuntimeError):
    """A `mongla.task(...)` deadline passed: the in-flight goal was cancelled."""


class MissionRefused(RuntimeError):
    """`mongla.require(verb)` found a verb this backend will not run.

    Raised on the deck, before the vehicle is committed -- deliberately the
    opposite of the retired FSM layer, which converted the same refusal into an
    ABORT wired to SURFACE and ended the run looking orderly (J04).
    """


# Verbs a passed task deadline never blocks: stopping, surfacing and releasing
# the vehicle must always be sendable, especially from the fallback that runs
# BECAUSE a task was abandoned.
_DEADLINE_EXEMPT = frozenset({'disarm', 'stop', 'surface', 'unlock_heading',
                              'mission_reset', 'pause', 'set_mode'})


def goal_uuid_hex(goal_handle) -> str:
    """The action goal's UUID as hex, or '' -- joins a scorecard row to manager logs."""
    try:
        return bytes(bytearray(goal_handle.goal_id.uuid)).hex()
    except Exception:                   # noqa: BLE001
        return ''


class MoveTimeout(MoveFailed):
    """The client's OWN deadline elapsed before the server returned a result
    (server crashed/wedged, or a goal overran its limit). The goal is
    cancelled. Subclasses ``MoveFailed`` so every existing
    ``except (MoveFailed, MoveRejected)`` handler treats a stall as a
    (non-fatal, bounded) failure -- the mission never hangs forever."""


# Client-side backstops so a wedged/crashed action server can never hang a
# mission (or its disarm) forever. These sit ON TOP of the server's own
# per-goal time limit -- they only bite when the server stops responding.
_GOAL_ACCEPT_TIMEOUT_S   = 10.0   # server must ACK the goal within this
_RESULT_TIMEOUT_MARGIN_S = 15.0   # backstop = the goal's own limit + this
_RESULT_TIMEOUT_FLOOR_S  = 60.0   # backstop for a goal that carries no time field
_CANCEL_TIMEOUT_S        = 5.0    # bound the cancel handshake too
# Safety / quick verbs get a SHORT independent deadline: they must be fast, and
# the disarm backstop must never wait on a healthy-server assumption.
_QUICK_DEADLINE_S = 12.0
_QUICK_CMDS = frozenset({
    'disarm', 'stop', 'surface', 'mission_reset', 'unlock_heading', 'arm',
    'set_mode',
})

# Detection-interrupt for vision-verb fallbacks (see begin_search_interrupt). When
# armed, send()'s result-wait spins in SLICES so a mission-authored search pattern's
# in-flight control verb is cancelled the instant the target reappears -- and every
# subsequent verb short-circuits -- so the search stops immediately instead of
# carrying the reacquired target back out of frame.
_INTERRUPT_SLICE_S = 0.05   # goal-wait spin granularity while a search interrupt is armed
_CANCEL_RESOLVE_S  = 2.0    # let the cancel + server-side motion-neutral resolve after a trip


class MonglaClient:
    def __init__(self, node):
        self.node    = node
        self._client = ActionClient(node, Move, '/mongla/move')
        self._active_goal_handle = None  # set during send(); cleared after
        # Search-interrupt state (armed only around a vision-verb fallback):
        self._interrupt_check   = None   # callable() -> bool; polled each spin slice
        self._interrupt_tripped = False  # set once the predicate fires -> short-circuit
        # OPT-IN task deadline (monotonic seconds), set by `mongla.task(...)`.
        self._task_deadline = None
        self.last_goal_id = ''

    # ------------------------------------------------------------------ #
    #  Deadline helpers                                                   #
    # ------------------------------------------------------------------ #

    def _result_deadline(self, goal) -> float:
        """Client backstop for a goal's result, in seconds.

        The bound is the goal's OWN server-side time limit
        (``duration``/``timeout``) plus a margin for the server's settle/brake
        cleanup; a goal with no time field falls back to a generous floor.
        Always finite -> a stalled server can never hang the caller forever.

        A safety/quick verb keeps a SHORT fixed floor (the disarm backstop must
        never wait on a healthy-server assumption) -- but the floor is a floor,
        NOT a ceiling: it must still cover the verb's own server budget, or a
        slow-but-legitimate arm/disarm (ACK + is_armed poll) trips MoveTimeout
        before it can finish. That is the "doesn't arm, 12 s crossed" bug: arm's
        real budget is ~18 s (3 s ACK + 15 s poll), disarm's ~26 s, but the old
        fixed 12 s cut them off. `send()` does not apply the COMMANDS defaults,
        so `goal.timeout` is 0.0 on the wire and the effective limit must be read
        from the same registry the server enforces.
        """
        spec_defaults = COMMANDS.get(getattr(goal, 'cmd', ''), {}).get('defaults', {})
        timeout = (float(getattr(goal, 'timeout', 0.0) or 0.0)
                   or float(spec_defaults.get('timeout', 0.0) or 0.0))
        duration = (float(getattr(goal, 'duration', 0.0) or 0.0)
                    or float(spec_defaults.get('duration', 0.0) or 0.0))
        limit = max(timeout, duration)
        backstop = (limit + _RESULT_TIMEOUT_MARGIN_S) if limit > 0.0 else 0.0
        if getattr(goal, 'cmd', '') in _QUICK_CMDS:
            return max(_QUICK_DEADLINE_S, backstop)
        return backstop if backstop > 0.0 else _RESULT_TIMEOUT_FLOOR_S

    def _cancel(self, goal_handle) -> None:
        """Best-effort, bounded cancel of an in-flight goal."""
        try:
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(
                self.node, cancel_future, timeout_sec=_CANCEL_TIMEOUT_S)
        except Exception:   # noqa: BLE001 -- cancel is best-effort; never raise from cleanup
            pass

    def cancel_active(self) -> None:
        """Bounded best-effort cancel of the goal currently in flight, if any.

        Safe to call from a mission's abort handler (Ctrl-C). Usually a no-op:
        ``send()``'s own Ctrl-C path cancels and clears the handle first -- this
        is the belt-and-suspenders backstop for any goal still registered.
        """
        goal_handle = self._active_goal_handle
        if goal_handle is not None:
            self._cancel(goal_handle)

    # ------------------------------------------------------------------ #
    #  Connection                                                         #
    # ------------------------------------------------------------------ #

    def wait_for_connection(self, timeout=15.0):
        self.node.get_logger().info('Waiting for /mongla/move action server...')
        if not self._client.wait_for_server(timeout_sec=timeout):
            raise TimeoutError('/mongla/move action server not available')
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
        `duration`, `gain`, `target`, `target_name`, `timeout`,
        `settle`, `yaw_rate_pct`. Unset fields stay at their rosidl
        defaults (0.0 / '').
        """
        if cmd not in COMMANDS:
            raise ValueError(
                f"Unknown command '{cmd}'. Known: {sorted(COMMANDS)}")

        # Search-interrupt already tripped this fallback -> the target is back;
        # short-circuit every remaining verb (send NOTHING) so the search stops
        # immediately instead of carrying the target out of frame. Bounded by the
        # tripped flag, which end_search_interrupt() clears before the verb re-enters.
        if self._interrupt_tripped:
            return self._search_interrupted_result(cmd)
        import time as _t
        if (self._task_deadline is not None and cmd not in _DEADLINE_EXEMPT
                and _t.monotonic() >= self._task_deadline):
            raise TaskAbandoned(f'task deadline passed before "{cmd}" was sent')

        goal = Move.Goal()
        goal.cmd = cmd
        for name, value in fields.items():
            if value is None:
                continue
            # Accept int anywhere a float is expected so mission code like
            # mongla.yaw_left(target=90) works identically to target=90.0
            if isinstance(value, int) and not isinstance(value, bool):
                value = float(value)
            setattr(goal, name, value)

        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        # Bounded goal-ACCEPT wait: a server that never acknowledges must not
        # hang the caller (wait_for_connection only proves the server existed
        # earlier; it can die between then and now).
        rclpy.spin_until_future_complete(
            self.node, send_future, timeout_sec=_GOAL_ACCEPT_TIMEOUT_S)
        if not send_future.done():
            raise MoveTimeout(
                f'Goal "{cmd}" not accepted within {_GOAL_ACCEPT_TIMEOUT_S:.0f}s '
                f'(action server unresponsive)')
        goal_handle = send_future.result()
        if goal_handle is None:
            raise MoveFailed(f'Goal "{cmd}" send failed (no goal handle returned)')

        if not goal_handle.accepted:
            raise MoveRejected(f'Goal "{cmd}" was REJECTED by action server')

        self._active_goal_handle = goal_handle
        self.last_goal_id = goal_uuid_hex(goal_handle)
        try:
            result_future = goal_handle.get_result_async()
            deadline = self._result_deadline(goal)
            # Fallback-search mode: a detection interrupt is armed, so wait in
            # SLICES and cancel the instant the target reappears. The SAME spin
            # services `_on_detections`, so the interrupt predicate (a pure cache
            # read) sees the fresh sighting without a second thread. On a trip we
            # return a synthesized result DIRECTLY -- bypassing the success-raise
            # below, since a cancelled goal reports success=False.
            if self._interrupt_check is not None or (
                    self._task_deadline is not None and cmd not in _DEADLINE_EXEMPT):
                if self._spin_with_interrupt(result_future, goal_handle, deadline):
                    return self._search_interrupted_result(cmd)
                result = result_future.result().result
                if not result.success:
                    raise MoveFailed(f'Goal "{cmd}" FAILED: {result.message}')
                return result
            try:
                # Bounded RESULT wait: a server that accepts then wedges (or a
                # goal that overruns) must never hang the mission forever. This
                # is the single most important safety backstop -- it also bounds
                # the mission-runner's emergency disarm (which rides this call).
                rclpy.spin_until_future_complete(
                    self.node, result_future, timeout_sec=deadline)
            except KeyboardInterrupt:
                # ⛔ Ctrl-C is an OPERATOR ABORT, not a failed verb. It used to
                # become `MoveFailed` -- an ordinary Exception -- which
                # `retry`/`selector`/`never_fails` and the vision verbs all
                # contain by design, so Ctrl-C during an align skipped ONE step
                # and the mission drove on. Cancel (bounded), then re-raise so
                # the runner's `except KeyboardInterrupt` abort path runs.
                self.node.get_logger().warn(
                    f'Ctrl-C — cancelling goal "{cmd}"...')
                self._cancel(goal_handle)
                rclpy.spin_until_future_complete(
                    self.node, result_future, timeout_sec=15.0)
                raise
            if not result_future.done():
                # Client deadline elapsed with no result -> server stalled/dead
                # or the goal overran. Cancel + fail BOUNDED so the mission (and
                # its disarm backstop) can never hang on a dead server.
                self.node.get_logger().error(
                    f'Goal "{cmd}" exceeded client deadline {deadline:.0f}s — '
                    f'cancelling (server stalled?)')
                self._cancel(goal_handle)
                raise MoveTimeout(
                    f'Goal "{cmd}" did not complete within {deadline:.0f}s '
                    f'(server stalled)')
            result = result_future.result().result
        finally:
            self._active_goal_handle = None

        if not result.success:
            raise MoveFailed(f'Goal "{cmd}" FAILED: {result.message}')
        return result

    # ------------------------------------------------------------------ #
    #  Search-interrupt: cancel a fallback verb the instant the target is back
    # ------------------------------------------------------------------ #

    def begin_search_interrupt(self, predicate) -> None:
        """Arm a detection interrupt for the current vision-verb fallback.

        While armed, ``send()``'s result-wait polls ``predicate()`` (a pure,
        side-effect-free cache read -- the spin itself refreshes the cache) each
        ~50 ms slice. The FIRST True cancels the in-flight goal and trips; every
        subsequent ``send()`` then short-circuits (dispatches no goal) so the rest
        of the search stops immediately rather than driving the reacquired target
        out of frame. Idempotent-safe; pair with ``end_search_interrupt``.
        """
        self._interrupt_check   = predicate
        self._interrupt_tripped = False

    def end_search_interrupt(self) -> bool:
        """Disarm the interrupt; return True iff it tripped (target reacquired)."""
        tripped = self._interrupt_tripped
        self._interrupt_check   = None
        self._interrupt_tripped = False
        return tripped

    def _spin_with_interrupt(self, result_future, goal_handle,
                             deadline: float) -> bool:
        """Sliced goal-wait. Returns True if the interrupt tripped (goal cancelled),
        False if the goal completed on its own. Raises ``MoveTimeout`` on the same
        bounded-stall condition as the normal path so a wedged server can't hang."""
        import time
        end = time.monotonic() + max(deadline, 0.0)
        while not result_future.done():
            try:
                rclpy.spin_until_future_complete(
                    self.node, result_future, timeout_sec=_INTERRUPT_SLICE_S)
            except KeyboardInterrupt:
                # Ctrl-C during a fallback search must cancel the in-flight goal (not
                # leave a thruster override running), then RE-RAISE so the mission's
                # emergency-disarm handler runs. (The normal wait instead swallows
                # Ctrl-C into a bounded MoveFailed; here re-raising is correct because
                # a fallback verb has no result worth returning.)
                self.node.get_logger().warn('Ctrl-C — cancelling fallback goal...')
                self._cancel(goal_handle)
                raise
            if result_future.done():
                return False
            # Predicate reads _det_seen, refreshed by the spin above (single-thread,
            # no nested pump). First True -> cancel + trip.
            if (self._task_deadline is not None
                    and time.monotonic() >= self._task_deadline):
                self._cancel(goal_handle)
                rclpy.spin_until_future_complete(
                    self.node, result_future, timeout_sec=_CANCEL_RESOLVE_S)
                raise TaskAbandoned('task deadline passed mid-goal; goal cancelled')
            if self._interrupt_check is not None and self._interrupt_check():
                self._interrupt_tripped = True
                self._cancel(goal_handle)
                # Let the cancel land + the server-side motion loop neutralise RC.
                rclpy.spin_until_future_complete(
                    self.node, result_future, timeout_sec=_CANCEL_RESOLVE_S)
                return True
            if time.monotonic() >= end:
                self.node.get_logger().error(
                    f'Goal exceeded client deadline {deadline:.0f}s — cancelling '
                    f'(server stalled?)')
                self._cancel(goal_handle)
                raise MoveTimeout(
                    f'Goal did not complete within {deadline:.0f}s (server stalled)')
        return False

    def _search_interrupted_result(self, cmd: str):
        """Synthesized result for a verb cut short by a search interrupt.

        success=True because the interrupt is the DESIRED outcome (the target is
        back); the honest message records why the motion was cut short so the
        scoreboard reads faithfully."""
        result = Move.Result()
        result.success = True
        result.message = f'{cmd}: search interrupted -- target reacquired'
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
            f'[FB   ] {feedback.phase}  {feedback.status_line}')

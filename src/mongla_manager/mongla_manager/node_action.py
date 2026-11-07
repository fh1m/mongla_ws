"""The /mongla/move action server: which goals run, how they stop, what they report.

`ActionMixin` is the part of `AUVManagerNode` that faces the clients. A goal
is accepted or refused (`goal_callback`, via `dispatch_policy`), runs one verb
on the facade (`execute_callback`), can be cancelled (`cancel_callback`), and
streams feedback while it runs (`FeedbackPump`, and the board's own progress
for board verbs). Only one verb holds the gate at a time; `stop`, `disarm`
and `surface` pre-empt it.
"""

import math
import sys
import threading
import time

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)

from mongla_interfaces.action import Move
from mongla_control import COMMANDS, fields_for
from mongla_control.fc.srot_fc import MOVE_VERBS as _BOARD_VERBS_WITH_PROGRESS

from .dispatch_policy import goal_acceptance, _CommandGate
from .vision_tunables import runtime_defaults_for_command, snapshot_from_node


class FeedbackPump:
    """Stream Move.Feedback at ~2.5 Hz while a goal is executing.

    Used as a context manager so the worker thread is guaranteed to be
    joined no matter how the command exits (success, exception, or a
    cancel mid-loop).

    `yaw_provider` is an optional ``fn(attitude) -> (yaw_deg, label)``
    injected by the node so the feedback line reports the same yaw the
    verbs read. When omitted, the attitude's own yaw.

    `vision_provider` is an optional ``fn() -> (x_px, y_px) | None`` -- the live
    signed target-from-centre px the active vision verb reports. When it returns
    a tuple the feedback carries `err_x_px`/`err_y_px` (else NaN) so an operator
    can watch a vision verb converge in real time via `ros2 topic echo`.
    """

    def __init__(self, fc, goal_handle, yaw_provider=None,
                 vision_provider=None):
        self._fc              = fc
        self._goal_handle     = goal_handle
        self._yaw_provider    = yaw_provider
        self._vision_provider = vision_provider
        self._stop            = threading.Event()
        self._thread          = threading.Thread(target=self._run, daemon=True)

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *_):
        self._stop.set()
        self._thread.join(timeout=1.0)

    def _run(self):
        """Stream action feedback while a verb runs.

        GUARDED (B44 class). A bare Thread target: an exception here -- most
        plausibly `publish_feedback()` on a goal that has just been cancelled, or
        a vision provider raising mid-verb -- used to kill the pump outright. The
        verb kept running, but the operator's live `err_x_px` convergence view
        went dead with nothing said. Never fatal, so never allowed to be fatal.
        """
        fails = 0
        while not self._stop.is_set():
          try:
            attitude = self._fc.get_attitude()
            if attitude is not None:
                if self._yaw_provider is not None:
                    yaw_deg, _ = self._yaw_provider(attitude)
                else:
                    yaw_deg = attitude['yaw']
                yaw_str = f'{yaw_deg:.1f}' if yaw_deg is not None else 'N/A'
                vis = (self._vision_provider()
                       if self._vision_provider is not None else None)
                feedback              = Move.Feedback()
                feedback.phase        = 'EXECUTING'
                feedback.current_value = float(attitude['depth'])
                feedback.error_value   = 0.0
                feedback.err_x_px      = float(vis[0]) if vis else math.nan
                feedback.err_y_px      = float(vis[1]) if vis else math.nan
                vis_str = f'  VIS:({vis[0]:+.0f},{vis[1]:+.0f})px' if vis else ''
                feedback.status_line   = (
                    f'YAW:{yaw_str}  DEPTH:{attitude["depth"]:+.2f}m{vis_str}')
                self._goal_handle.publish_feedback(feedback)
            fails = 0
          except Exception:                    # noqa: BLE001 -- see the docstring
            fails += 1
            if fails in (1, 25):               # once, then once more if persistent
                import traceback
                print(f'[FBK  ] feedback pump fault #{fails} (verb continues): '
                      f'{traceback.format_exc(limit=1).strip()}', file=sys.stderr)
          self._stop.wait(timeout=0.4)


class ActionMixin:
    """The action server half of the manager node."""

    def _setup_action_server(self) -> None:
        """The callback groups and the /mongla/move action server."""
        self.action_group = ReentrantCallbackGroup()
        self.timer_group  = MutuallyExclusiveCallbackGroup()

        # The gate is claimed in goal_callback, not in execute_callback.
        #
        # It used to be a plain bool READ in `goal_callback` (the accept/reject
        # decision) and WRITTEN in `execute_callback` (the start of the motion
        # loop). Those are two different callbacks on a ReentrantCallbackGroup
        # under a MultiThreadedExecutor, and nothing held a lock across the gap --
        # so a second goal arriving in that window saw `command_active` still
        # False and was ACCEPTED. Two motion loops then drove the board at once,
        # sharing one abort flag, and whichever finished first cleared the gate
        # while the other was still commanding thrust.
        #
        # REPRODUCED: 40/40 trials at every hand-off delay from 20 ms down to
        # 1 ms (`test_command_gate.py::test_two_goals_cannot_both_claim`).
        # rclpy only sets the WIDTH of the window, not whether it exists.
        #
        # Claiming at the accept decision makes test-and-set one operation. The
        # cost is that an accepted goal which never reaches execute_callback would
        # leak the claim, so the claim carries a monotonic stamp and
        # `_reap_stale_claim()` clears it from the telemetry timer -- loudly,
        # because a leak means an assumption above is wrong.
        self._gate = _CommandGate()
        self.action_server  = ActionServer(
            self, Move, '/mongla/move',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_group,
        )

    # ================================================================== #
    #  MAVLink reader -- only place recv_match() is called                #
    # ================================================================== #

    # ================================================================== #
    #  Action Server callbacks                                            #
    # ================================================================== #

    @property
    def command_active(self) -> bool:
        """Whether a command currently holds the gate.

        Read-only on purpose. Every write used to be a bare assignment from one of
        four places, which is what made the race invisible; the gate object now
        owns the transitions and the only way in is `claim()` / `release()`.
        """
        return self._gate.active

    def goal_callback(self, goal_request):
        # A verb that does not exist is refused here, before it can claim the
        # gate: claimed and then refused in execute_callback, it held the gate
        # for COMMAND_GATE_STALE_S and every real verb in that window bounced.
        if goal_request.cmd not in COMMANDS:
            self.get_logger().warn(f'[ACT  ] Rejected unknown command {goal_request.cmd!r}')
            return GoalResponse.REJECT
        # ONE atomic operation: decide and claim together. `goal_acceptance` stays
        # the pure policy (and stays unit-testable); the gate applies it under its
        # own lock so no second caller can observe the pre-claim state.
        accept, signal_abort = self._gate.claim(goal_request.cmd, goal_acceptance)
        if not accept:
            self.get_logger().warn(
                f'[ACT  ] Rejected {goal_request.cmd} -- command already active')
            return GoalResponse.REJECT
        if signal_abort:
            # Safety command accepted while busy: signal abort so the running
            # loop exits at its next tick, releasing the lock for us.
            self.get_logger().info(
                f'[ACT  ] {goal_request.cmd} (safety) accepted -- signalling abort')
            self.mongla.request_abort()
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('[ACT  ] Cancel requested -- stopping thrusters')
        # Remembered HERE, before the abort below: the running verb can exit
        # and reach `execute_callback`'s result before rclpy has moved the goal
        # to CANCELING, so `is_cancel_requested` alone loses the race.
        self._cancelled_goals = getattr(self, '_cancelled_goals', set())
        self._cancelled_goals.add(bytes(goal_handle.goal_id.uuid))
        # Signal abort FIRST so the running loop exits at its next tick (its
        # own finally neutralises lock-aware), and free the active gate so a
        # queued safety verb (disarm) gets through. Then a backstop neutral.
        self.mongla.request_abort()
        # Releasing here is deliberate (a queued `disarm` must get through) and
        # it is ALSO how a cancel let an ordinary `move_*` in while the cancelled
        # loop was still winding down: the gate is freed for every verb, not just
        # the safety ones. Keep the release -- a stuck gate is worse -- but mark it
        # DRAINING, which `claim()` treats as busy for everything except a safety
        # verb. The cancelled loop clears it from its own `finally`.
        self._gate.begin_drain()
        self.fc.send_neutral()
        return CancelResponse.ACCEPT

    def _was_cancelled(self, goal_handle, wait_s: float = 0.5) -> bool:
        """True once the goal is CANCELING; waits briefly for rclpy to get there.

        `canceled()` is only a legal transition from CANCELING. The cancel
        callback has already fired the abort, so the verb may finish first;
        rclpy's own state change follows on another executor thread within
        milliseconds. Not cancelled at all: False at once, no wait.
        """
        uid = bytes(goal_handle.goal_id.uuid)
        if uid not in getattr(self, '_cancelled_goals', set()):
            return bool(goal_handle.is_cancel_requested)
        self._cancelled_goals.discard(uid)
        deadline = time.monotonic() + wait_s
        while not goal_handle.is_cancel_requested and time.monotonic() < deadline:
            time.sleep(0.005)
        return bool(goal_handle.is_cancel_requested)

    def execute_callback(self, goal_handle):
        request = goal_handle.request
        cmd     = request.cmd

        if cmd not in COMMANDS:
            result = Move.Result()
            result.success = False
            result.message = f'Unknown command: {cmd}'
            goal_handle.abort()
            return result

        # Already claimed in goal_callback. Confirm the hand-off arrived, and take
        # the claim's token: only that token can release the gate, so a loop a
        # safety verb pre-empted cannot free the gate while the safety verb runs.
        token = self._gate.confirm(cmd)
        if token is None:
            # Reaped as a leak, or a later claim took the gate over before this
            # goal started (a second `stop` behind the first). Running now would
            # drive the board with nothing guarding it.
            result = Move.Result()
            result.success = False
            result.message = f'{cmd}: superseded before it started -- not run'
            goal_handle.abort()
            self.get_logger().warn(f'[ACT  ] {cmd} -> NOT RUN (gate held by a later claim)')
            return result
        try:
            gid = bytes(bytearray(goal_handle.goal_id.uuid)).hex()[:8]
        except Exception:                   # noqa: BLE001 -- a log field only
            gid = '?'
        self.get_logger().info(f'[ACT  ] {cmd} -> EXECUTING (goal {gid})')

        try:
            with FeedbackPump(self.fc, goal_handle,
                              yaw_provider=self._effective_yaw_deg,
                              vision_provider=self.mongla.vision_telemetry):
                # Re-snapshot params for every goal so freshly-set
                # `vision.*` values land on the very next command.
                snap = snapshot_from_node(self)
                runtime = runtime_defaults_for_command(cmd, snap)
                self.mongla.vision_mixer_aware = bool(
                    snap.get('vision.mixer_aware', True))
                kwargs = fields_for(cmd, request, runtime_defaults=runtime)
                method = getattr(self.mongla, cmd)
                if cmd in _BOARD_VERBS_WITH_PROGRESS:
                    kwargs['on_progress'] = self._progress_relay(cmd, goal_handle)
                result = method(**kwargs)

            if result.success:
                goal_handle.succeed()
                self.get_logger().info(
                    f'[ACT  ] {cmd} -> DONE ({result.message})')
            elif self._was_cancelled(goal_handle):
                # A cancelled goal ends canceled, not ABORTED (issue #22's
                # executor test found it). The action contract has two terminal
                # failure states for a reason: "you asked me to stop" and "I
                # could not do it" are different facts, and a bag or a tool
                # reading the status could not tell an operator's cancel from a
                # failed verb.
                goal_handle.canceled()
                self.get_logger().warning(
                    f'[ACT  ] {cmd} -> CANCELED ({result.message})')
            else:
                goal_handle.abort()
                self.get_logger().error(
                    f'[ACT  ] {cmd} -> REJECTED ({result.message})')
            return result

        except Exception as exc:
            # Best-effort: surface current depth so the operator sees
            # *something*. Detail (target/current/error) is in
            # result.message.
            attitude = self.fc.get_attitude()
            result = Move.Result()
            result.success     = False
            result.message     = f'{cmd}: exception -- {exc}'
            result.final_value = float(attitude['depth']) if attitude else math.nan
            result.error_value = 0.0
            goal_handle.abort()
            self.get_logger().error(f'[ACT  ] {cmd} FAILED: {exc}')

            # A verb that raised mid-loop skipped its own neutral, so the
            # last MANUAL_CONTROL demand may still be in force. Zero it so
            # the next verb starts from a known state.
            try:
                self.fc.send_neutral()
            except Exception as cleanup_exc:
                self.get_logger().warn(
                    f'[ACT  ] post-failure neutralise raised: {cleanup_exc}')
            return result

        finally:
            self._gate.release(token)

    @staticmethod
    def _progress_relay(cmd, goal_handle):
        """The board's progress fraction, relayed as action feedback."""
        def relay(frac):
            fb = Move.Feedback()
            fb.phase         = cmd
            fb.current_value = float(frac)
            fb.status_line   = f'{cmd} {frac * 100:.0f}%'
            goal_handle.publish_feedback(fb)
        return relay

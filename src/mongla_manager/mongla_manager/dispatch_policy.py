"""Pure goal-acceptance policy for the /mongla/move action server.

Extracted from `auv_manager_node.goal_callback` so the safety-bypass rule is
unit-testable without spinning a live ROS2 node. The rule (CLAUDE.md §13 safety):
the **safety verbs** (`disarm` / `stop` / `surface`) bypass the `command_active`
gate so they always execute even while another command runs — and accepting one
while busy signals a cooperative abort to the running loop so it releases the lock.
"""
from __future__ import annotations

SAFETY_VERBS = frozenset({'disarm', 'stop', 'surface'})


def goal_acceptance(cmd, command_active, safety_verbs=SAFETY_VERBS):
    """Decide whether to accept an incoming goal. Returns ``(accept, signal_abort)``.

    - **idle** (``command_active`` False): accept any verb; no abort.
    - **busy + non-safety verb**: reject (one command at a time).
    - **busy + safety verb**: accept AND signal abort (pre-empt the running loop).
    """
    if not command_active:
        return True, False
    if cmd in safety_verbs:
        return True, True
    return False, False


# ── the gate that enforces the policy above ──────────────────────────────────
#
# ⛔ WHY THIS IS AN OBJECT AND NOT A BOOL. `goal_acceptance` is the right rule and
# it was always correct; the defect was that applying it took TWO steps in two
# different callbacks:
#
#     goal_callback(B)     read  node.command_active        -> False, ACCEPT
#     execute_callback(A)  write node.command_active = True
#
# On a ReentrantCallbackGroup under a MultiThreadedExecutor nothing holds a lock
# across that gap, so both goals were accepted and two motion loops drove the
# board at once. REPRODUCED 40/40 at hand-off delays of 20, 5 and 1 ms.
#
# A test-and-set has to be ONE operation, so the state and the rule live together
# and the rule is applied while the lock is held. There is no public setter.
#
# ⚠ THE LOCK IS HELD ONLY ACROSS THE DECISION. It never wraps a MAVLink write, a
# motion loop, or anything that can block -- a gate that can deadlock the action
# server is worse than the race it replaces.
import threading
import time

# How long an unconfirmed claim may stand before it is treated as leaked. Well
# above any plausible ACCEPT -> EXECUTING hand-off (measured in milliseconds) and
# well below an operator's patience.
COMMAND_GATE_STALE_S = 5.0


class _CommandGate:
    """Atomic `command_active`, with the accept rule applied under the lock.

    States:
      IDLE      nothing holds it; any verb may claim.
      CLAIMED   a verb holds it, confirmed or not; only a safety verb may pre-empt.
      DRAINING  a cancelled loop is still winding down. Treated as busy for
                everything except a safety verb -- which is the point: freeing the
                gate outright on cancel is what let an ordinary `move_*` start
                while the cancelled leg was still commanding thrust.
    """

    def __init__(self, stale_s: float = COMMAND_GATE_STALE_S) -> None:
        self._lock = threading.Lock()
        self._cmd = None          # the verb holding it, or None
        # Which CLAIM holds it. A verb name cannot say: `stop` can pre-empt
        # `stop`, and a pre-empted loop's `finally` must not free the gate its
        # pre-emptor now holds. Kept through a drain, so the cancelled loop is
        # the one that ends it.
        self._token = 0
        self._next_token = 0
        self._claimed_at = 0.0    # monotonic, for the leak reaper
        self._confirmed = False   # execute_callback was reached
        self._draining = False
        self._stale_s = stale_s

    @property
    def active(self) -> bool:
        with self._lock:
            return self._cmd is not None or self._draining

    def claim(self, cmd, policy):
        """Apply `policy(cmd, busy)` and, if it accepts, take the gate.

        Returns the policy's own ``(accept, signal_abort)`` unchanged, so the rule
        stays exactly the one `goal_acceptance` states and this adds only atomicity.
        """
        with self._lock:
            busy = self._cmd is not None or self._draining
            accept, signal_abort = policy(cmd, busy)
            if accept:
                # A safety verb pre-empting a running command takes the gate from
                # it. The running loop's own `finally` then calls release() with
                # ITS token, which no longer matches -- see release().
                self._next_token += 1
                self._token = self._next_token
                self._cmd = cmd
                self._claimed_at = time.monotonic()
                self._confirmed = False
                self._draining = False
            return accept, signal_abort

    def confirm(self, cmd):
        """The claim reached execute_callback: return its token, or None.

        None means the gate does not hold an unconfirmed claim for `cmd` -- it
        was reaped, or a later claim took it over (a second `stop` arriving
        before the first one started). Running anyway would drive the board
        unguarded, so the caller refuses the goal instead.
        """
        with self._lock:
            if self._cmd != cmd or self._confirmed:
                return None
            self._confirmed = True
            return self._token

    def begin_drain(self) -> None:
        """A cancelled loop is winding down: busy for everything but a safety verb.

        Only when something holds the gate. A drain with no owner would never be
        released, because nothing is left to call release(), and the gate would
        refuse every ordinary verb until the node restarts.
        """
        with self._lock:
            if self._cmd is None:
                return
            self._cmd = None
            self._confirmed = False
            self._draining = True

    def release(self, token) -> bool:
        """End the claim `token` names, including its drain. False if not ours.

        ⛔ A PRE-EMPTED LOOP MUST NOT FREE ITS PRE-EMPTOR'S CLAIM. Before the
        token, `release()` cleared whatever held the gate: `disarm` pre-empts a
        `move_forward`, the move's `finally` runs, and the gate is empty while
        `disarm` is still executing -- so an ordinary verb could start beside it.
        """
        with self._lock:
            if token is None or token != self._token:
                return False
            self._cmd = None
            self._confirmed = False
            self._draining = False
            return True

    def reap_stale(self):
        """Release and return a claim that was never confirmed. None if nothing leaked.

        ⚠ Only UNCONFIRMED claims. A long-running command is confirmed and must
        never be timed out here -- a `move_forward 120` is not a leak.
        """
        with self._lock:
            if self._cmd is None or self._confirmed:
                return None
            if time.monotonic() - self._claimed_at < self._stale_s:
                return None
            leaked, self._cmd = self._cmd, None
            self._draining = False
            return leaked

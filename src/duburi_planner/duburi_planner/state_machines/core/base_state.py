"""DuburiState — base class for all Mongla FSM states.

Wraps DuburiMission DSL verbs (never calls Pixhawk/MAVLink directly).
Adds: per-state timeout, exception → ABORT with stop(), blackboard error key.

Subclasses implement _run(bb) → outcome str.
execute() is the YASMIN hook (called by StateMachine); it sets the
start time, wraps _run in try/except, and returns the outcome.
"""
from __future__ import annotations

import time
from typing import TYPE_CHECKING

from yasmin import State, Blackboard

from .outcomes import TIMEOUT, ABORT
from .blackboard import BK

if TYPE_CHECKING:
    from duburi_planner.duburi_dsl import DuburiMission
    from .vehicle_profile import VehicleProfile


class DuburiState(State):
    TIMEOUT_S: float = 60.0

    def __init__(
        self,
        duburi: DuburiMission,
        profile: VehicleProfile,
        outcomes: list[str],
    ) -> None:
        # Merge caller outcomes with the two universal exits every state has.
        all_outcomes = list({*outcomes, TIMEOUT, ABORT})
        super().__init__(outcomes=all_outcomes)
        self.duburi  = duburi
        self.profile = profile
        self._start  = 0.0

    # ------------------------------------------------------------------
    # YASMIN entry point
    # ------------------------------------------------------------------

    def execute(self, blackboard: Blackboard) -> str:
        self._start = time.monotonic()
        try:
            outcome = self._run(blackboard)
        except Exception as exc:
            try:
                self.duburi.stop()
            except Exception:
                pass
            blackboard[BK.LAST_ERROR] = str(exc)
            return ABORT

        # Safety CEILING (this is what makes timed_out() live -- previously nothing
        # ever called it, so every state's TIMEOUT_S was inert). TIMEOUT_S sits with
        # margin above each state's own verb timeout (SetDepthState uses
        # set_depth(timeout=TIMEOUT_S-2); vision states use duration+10), so a normal
        # SUCCEED always completes well under it -- this only fires on a genuine
        # RUNAWAY (a verb that blew past its own bound). We route the overrun through
        # ABORT (stop() + ABORT), NOT a fresh TIMEOUT return: ABORT is wired on EVERY
        # add_state in every plan (-> SURFACE), whereas TIMEOUT transitions are only
        # partially present, so returning TIMEOUT from a state whose plan omitted it
        # would raise in YASMIN. Same safe destination (SURFACE), zero plan churn.
        # (VisionSearchState still returns TIMEOUT explicitly via its own path.)
        if outcome not in (TIMEOUT, ABORT) and self.timed_out():
            try:
                self.duburi.stop()
            except Exception:
                pass
            blackboard[BK.LAST_ERROR] = (
                f'{type(self).__name__} exceeded TIMEOUT_S='
                f'{self.TIMEOUT_S:.0f}s (ran {self.elapsed():.0f}s) -> ABORT')
            return ABORT
        return outcome

    # ------------------------------------------------------------------
    # Subclass API
    # ------------------------------------------------------------------

    def _run(self, bb: Blackboard) -> str:
        raise NotImplementedError

    def timed_out(self) -> bool:
        return time.monotonic() - self._start > self.TIMEOUT_S

    def elapsed(self) -> float:
        return time.monotonic() - self._start

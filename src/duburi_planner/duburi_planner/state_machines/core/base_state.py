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
            return self._run(blackboard)
        except Exception as exc:
            try:
                self.duburi.stop()
            except Exception:
                pass
            blackboard[BK.LAST_ERROR] = str(exc)
            return ABORT

    # ------------------------------------------------------------------
    # Subclass API
    # ------------------------------------------------------------------

    def _run(self, bb: Blackboard) -> str:
        raise NotImplementedError

    def timed_out(self) -> bool:
        return time.monotonic() - self._start > self.TIMEOUT_S

    def elapsed(self) -> float:
        return time.monotonic() - self._start

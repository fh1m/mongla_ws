"""Utility states — countdown, pause, score logging."""
from __future__ import annotations

import time

from yasmin import Blackboard

from ..core.base_state import DuburiState
from ..core.blackboard import BK
from ..core.outcomes import SUCCEED


class CountdownState(DuburiState):
    """Tether-removal countdown window. Sets BK.MISSION_START_T."""
    TIMEOUT_S = 120.0

    def __init__(self, duburi, profile, seconds: int = 10) -> None:
        super().__init__(duburi, profile, [SUCCEED])
        self._seconds = seconds

    def _run(self, bb: Blackboard) -> str:
        self.duburi.countdown(self._seconds)
        bb[BK.MISSION_START_T] = time.monotonic()
        return SUCCEED


class PauseState(DuburiState):
    """Dwell in place for a fixed duration."""
    TIMEOUT_S = 120.0

    def __init__(self, duburi, profile, seconds: float = 3.0) -> None:
        super().__init__(duburi, profile, [SUCCEED])
        self._seconds = seconds

    def _run(self, bb: Blackboard) -> str:
        self.duburi.pause(self._seconds)
        return SUCCEED


class LogScoreState(DuburiState):
    """Export mission scoreboard JSON at end of run."""
    TIMEOUT_S = 10.0

    def __init__(self, duburi, profile) -> None:
        super().__init__(duburi, profile, [SUCCEED])

    def _run(self, bb: Blackboard) -> str:
        self.duburi.log_scoreboard(json_path='auto')
        return SUCCEED

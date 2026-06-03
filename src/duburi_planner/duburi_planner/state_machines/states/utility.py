"""Utility states — countdown, pause, score logging, detector switching."""
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


class SetDetectorState(DuburiState):
    """Switch camera context and/or detector class filter atomically.

    Use between tasks that need different cameras or detection targets:

        sm.add_state('SWITCH_TO_DOWN',
                     SetDetectorState(duburi, profile,
                                      camera='downward', classes='bin_a'),
                     transitions={SUCCEED: 'SCAN_BIN', ABORT: 'SURFACE'})

    Parameters
    ----------
    camera : str or None
        Set as sticky duburi.camera for subsequent vision states.
        None = leave unchanged.
    classes : str or None
        Comma-separated class names forwarded to ros2 param set.
        None = leave unchanged.  '' = all classes.
    model : str or None
        Switch YOLO model weights (duburi.set_model). None = leave unchanged.
    """
    TIMEOUT_S = 10.0

    def __init__(
        self,
        duburi,
        profile,
        camera: str | None = None,
        classes: str | None = None,
        model: str | None = None,
    ) -> None:
        super().__init__(duburi, profile, [SUCCEED])
        self._camera  = camera
        self._classes = classes
        self._model   = model

    def _run(self, bb: Blackboard) -> str:
        if self._model:
            self.duburi.set_model(self._model)
        if self._camera:
            self.duburi.camera = self._camera
        if self._classes is not None:
            self.duburi.set_classes(self._classes)
        return SUCCEED

"""Utility states — countdown, pause, score logging, detector switching."""
from __future__ import annotations

import time

from yasmin import Blackboard

from ..core.base_state import DuburiState
from ..core.blackboard import BK
from ..core.outcomes import SUCCEED, FAILED


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


class FireState(DuburiState):
    """Activate a payload BOARD channel (duburi.fire(channel)).

    channel: the board's own PCA9685 channel, 1..16 -- the same n as SERVO{n}_ROLE.
    There is no host-side mapping; the board decides whether that channel is a
    payload switch or the on-board arm, and an arm channel is refused.
    confirm_pause_s: dwell after firing to confirm actuation.

    Returns FAIL when the shot did not go out, so a plan can branch (retry another
    channel, skip the task, keep its remaining time). This used to return SUCCEED
    unconditionally, which meant a refused shot was indistinguishable from a hit.
    """
    TIMEOUT_S = 8.0

    def __init__(
        self, duburi, profile, channel: int, confirm_pause_s: float = 2.0
    ) -> None:
        super().__init__(duburi, profile, [SUCCEED, FAILED])
        self._channel       = channel
        self._confirm_pause = confirm_pause_s

    def _run(self, bb: Blackboard) -> str:
        res = self.duburi.fire(self._channel)
        self.duburi.pause(self._confirm_pause)
        # `duburi.fire` returns a Move.Result: success is True only when the board
        # accepted the activation. NO_ACK deliberately counts as success-ish upstream
        # -- see FireResult -- but here we only have the boolean, and treating an
        # unacknowledged shot as failure would abandon a task over link jitter.
        return SUCCEED if getattr(res, 'success', bool(res)) else FAILED


class StyleRollState(DuburiState):
    """ACRO roll manoeuvre at end of run (duburi.style_roll()).

    flips    : number of 360° rolls.
    headroom : ascend this many metres before rolling (pool safety).
    gain     : roll speed percent.
    """
    TIMEOUT_S = 60.0

    def __init__(
        self,
        duburi,
        profile,
        flips: int = 1,
        headroom: float = 0.4,
        gain: int = 60,
    ) -> None:
        super().__init__(duburi, profile, [SUCCEED])
        self._flips    = flips
        self._headroom = headroom
        self._gain     = gain

    def _run(self, bb: Blackboard) -> str:
        self.duburi.style_roll(
            flips=self._flips, headroom=self._headroom, gain=self._gain)
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

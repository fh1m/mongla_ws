"""Utility states — countdown, pause, score logging, detector switching."""
from __future__ import annotations

import time

from yasmin import Blackboard

from ..core.base_state import MonglaState
from ..core.blackboard import BK
from mongla_control.fc.base import FIRE_NO_ACK, FIRE_NOT_READY

from ..core.outcomes import SUCCEED, FAILED


class CountdownState(MonglaState):
    """Tether-removal countdown window. Sets BK.MISSION_START_T."""
    TIMEOUT_S = 120.0

    def __init__(self, mongla, profile, seconds: int = 10) -> None:
        super().__init__(mongla, profile, [SUCCEED])
        self._seconds = seconds

    def _run(self, bb: Blackboard) -> str:
        self.mongla.countdown(self._seconds)
        bb[BK.MISSION_START_T] = time.monotonic()
        return SUCCEED


class PauseState(MonglaState):
    """Dwell in place for a fixed duration."""
    TIMEOUT_S = 120.0

    def __init__(self, mongla, profile, seconds: float = 3.0) -> None:
        super().__init__(mongla, profile, [SUCCEED])
        self._seconds = seconds

    def _run(self, bb: Blackboard) -> str:
        self.mongla.pause(self._seconds)
        return SUCCEED


class LogScoreState(MonglaState):
    """Export mission scoreboard JSON at end of run."""
    TIMEOUT_S = 10.0

    def __init__(self, mongla, profile) -> None:
        super().__init__(mongla, profile, [SUCCEED])

    def _run(self, bb: Blackboard) -> str:
        self.mongla.log_scoreboard(json_path='auto')
        return SUCCEED


class FireState(MonglaState):
    """Activate a payload BOARD channel (mongla.fire(channel)).

    channel: the board's own PCA9685 channel, 1..16 -- the same n as SERVO{n}_ROLE.
    There is no host-side mapping; the board decides whether that channel is a
    payload switch or the on-board arm, and an arm channel is refused.
    confirm_pause_s: dwell after firing to confirm actuation.

    Returns FAILED when the shot did not go out, so a plan can branch (retry
    another channel, skip the task, keep its remaining time). This used to return
    SUCCEED unconditionally, which meant a refused shot was indistinguishable from
    a hit.

    The single exception is NO_ACK -- see `_run`.
    """
    TIMEOUT_S = 8.0

    def __init__(
        self, mongla, profile, channel: int, confirm_pause_s: float = 2.0
    ) -> None:
        super().__init__(mongla, profile, [SUCCEED, FAILED])
        self._channel       = channel
        self._confirm_pause = confirm_pause_s

    def _run(self, bb: Blackboard) -> str:
        res = self.mongla.fire(self._channel)
        self.mongla.pause(self._confirm_pause)
        if getattr(res, 'success', bool(res)):
            return SUCCEED
        # NO_ACK is the one failure that is not evidence of a failure: the command
        # very likely went out and only the acknowledgement was lost. Over the
        # BlueOS bridge that is measurably ~8-9% of frames, so failing the state on
        # it would abandon a task over link jitter. Every other code -- the shot was
        # refused, denied, or never attempted -- is a real miss and says so.
        #
        # This branches on `final_value` (the FireResult code) rather than
        # `success`, because `success` is `.ok`, and `.ok` is deliberately False for
        # NO_ACK: "we do not know" must not read as "it fired" to anything that only
        # sees a boolean.
        code = int(getattr(res, 'final_value', FIRE_NOT_READY))
        if code == FIRE_NO_ACK:
            return SUCCEED
        return FAILED


class StyleRollState(MonglaState):
    """ACRO roll manoeuvre at end of run (mongla.style_roll()).

    flips    : number of 360° rolls.
    headroom : ascend this many metres before rolling (pool safety).
    gain     : roll speed percent.
    """
    TIMEOUT_S = 60.0

    def __init__(
        self,
        mongla,
        profile,
        flips: int = 1,
        headroom: float = 0.4,
        gain: int = 60,
    ) -> None:
        super().__init__(mongla, profile, [SUCCEED])
        self._flips    = flips
        self._headroom = headroom
        self._gain     = gain

    def _run(self, bb: Blackboard) -> str:
        self.mongla.style_roll(
            flips=self._flips, headroom=self._headroom, gain=self._gain)
        return SUCCEED


class SetDetectorState(MonglaState):
    """Switch camera context and/or detector class filter atomically.

    Use between tasks that need different cameras or detection targets:

        sm.add_state('SWITCH_TO_DOWN',
                     SetDetectorState(mongla, profile,
                                      camera='downward', classes='bin_a'),
                     transitions={SUCCEED: 'SCAN_BIN', ABORT: 'SURFACE'})

    Parameters
    ----------
    camera : str or None
        Set as sticky mongla.camera for subsequent vision states.
        None = leave unchanged.
    classes : str or None
        Comma-separated class names forwarded to ros2 param set.
        None = leave unchanged.  '' = all classes.
    model : str or None
        Switch YOLO model weights (mongla.set_model). None = leave unchanged.
    """
    TIMEOUT_S = 10.0

    def __init__(
        self,
        mongla,
        profile,
        camera: str | None = None,
        classes: str | None = None,
        model: str | None = None,
    ) -> None:
        super().__init__(mongla, profile, [SUCCEED])
        self._camera  = camera
        self._classes = classes
        self._model   = model

    def _run(self, bb: Blackboard) -> str:
        if self._model:
            self.mongla.set_model(self._model)
        if self._camera:
            self.mongla.camera = self._camera
        if self._classes is not None:
            self.mongla.set_classes(self._classes)
        return SUCCEED

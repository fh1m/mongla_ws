"""Vision states — wrapping DuburiMission DSL vision verbs.

Each state accepts an explicit `camera` parameter (e.g. 'forward', 'downward').
When camera=None the DSL falls back to the sticky `duburi.camera` context.
Passing camera= explicitly makes plans self-documenting and enables camera
switching between tasks without a separate SetDetectorState.
"""
from __future__ import annotations

from yasmin import Blackboard

from ..core.base_state import DuburiState
from ..core.outcomes import SUCCEED, FAILED, TIMEOUT


class VisionFindState(DuburiState):
    """Block until target visible; optionally move while searching.

    Wraps duburi.vision.find(). camera=None → inherits duburi.camera sticky.
    """
    TIMEOUT_S = 60.0

    def __init__(
        self,
        duburi,
        profile,
        target: str,
        camera: str | None = None,
        move: str = 'forward',
        gain: int = 30,
        timeout: float = 45.0,
    ) -> None:
        super().__init__(duburi, profile, [SUCCEED])
        self._target  = target
        self._camera  = camera
        self._move    = move
        self._gain    = gain
        self._timeout = timeout

    def _run(self, bb: Blackboard) -> str:
        kw = dict(target=self._target, move=self._move,
                  gain=self._gain, timeout=self._timeout)
        if self._camera:
            kw['camera'] = self._camera
        result = self.duburi.vision.find(**kw)
        return SUCCEED if result.success else TIMEOUT


class VisionHomeState(DuburiState):
    """Multi-axis vision convergence.

    Wraps duburi.vision.home(). camera=None → inherits duburi.camera sticky.
    All gate_guard / pass_at / dist kwargs forwarded unchanged.
    """
    TIMEOUT_S = 30.0

    def __init__(
        self,
        duburi,
        profile,
        target: str,
        camera: str | None = None,
        yaw: bool = False,
        lat: bool = False,
        depth: bool = False,
        forward: bool = False,
        gate_guard: bool = False,
        pass_at: float = 0.0,
        dist: float = 0.0,
        metric: str = 'area',
        duration: float = 20.0,
        on_lost: str = 'fail',
    ) -> None:
        super().__init__(duburi, profile, [SUCCEED, FAILED])
        self._kwargs = dict(
            target=target,
            yaw=yaw, lat=lat, depth=depth, forward=forward,
            gate_guard=gate_guard,
            pass_at=pass_at or None,
            dist=dist or None,
            metric=metric,
            duration=duration,
            on_lost=on_lost,
        )
        if camera:
            self._kwargs['camera'] = camera
        # Strip None-valued optional keys — DSL inherits param defaults for them
        self._kwargs = {k: v for k, v in self._kwargs.items() if v is not None}
        self.TIMEOUT_S = duration + 5.0

    def _run(self, bb: Blackboard) -> str:
        result = self.duburi.vision.home(**self._kwargs)
        return SUCCEED if result.success else FAILED


class VisionScanState(DuburiState):
    """Incremental yaw orbit until target detected or budget exhausted.

    Wraps duburi.vision.scan(). camera=None → inherits duburi.camera sticky.
    """
    TIMEOUT_S = 100.0

    def __init__(
        self,
        duburi,
        profile,
        target: str,
        camera: str | None = None,
        step: float = 20.0,
        dwell: float = 1.5,
        duration: float = 90.0,
    ) -> None:
        super().__init__(duburi, profile, [SUCCEED])
        self._target   = target
        self._camera   = camera
        self._step     = step
        self._dwell    = dwell
        self._duration = duration
        self.TIMEOUT_S = duration + 5.0

    def _run(self, bb: Blackboard) -> str:
        kw = dict(target=self._target, step=self._step,
                  dwell=self._dwell, duration=self._duration)
        if self._camera:
            kw['camera'] = self._camera
        result = self.duburi.vision.scan(**kw)
        return SUCCEED if result.success else TIMEOUT

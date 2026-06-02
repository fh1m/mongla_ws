"""Vision states — wrapping DuburiMission DSL vision verbs.

Each state delegates entirely to the DSL; no direct RC/MAVLink calls here.
The manager node owns the closed-loop vision control; these states are
thin wrappers that turn DSL outcomes into YASMIN outcome strings.
"""
from __future__ import annotations

from yasmin import Blackboard

from ..core.base_state import DuburiState
from ..core.outcomes import SUCCEED, FAILED, TIMEOUT


class VisionFindState(DuburiState):
    """Block until target class visible; optionally move while searching.

    Wraps duburi.vision.find(). Returns SUCCEED when target first appears;
    TIMEOUT when the find verb's own timeout expires.
    """
    TIMEOUT_S = 60.0

    def __init__(
        self,
        duburi,
        profile,
        target: str,
        move: str = 'forward',
        gain: int = 30,
        timeout: float = 45.0,
    ) -> None:
        super().__init__(duburi, profile, [SUCCEED])
        self._target  = target
        self._move    = move
        self._gain    = gain
        self._timeout = timeout

    def _run(self, bb: Blackboard) -> str:
        result = self.duburi.vision.find(
            target=self._target,
            move=self._move,
            gain=self._gain,
            timeout=self._timeout,
        )
        return SUCCEED if result.success else TIMEOUT


class VisionHomeState(DuburiState):
    """Multi-axis vision convergence.

    Wraps duburi.vision.home(). Passes all gate_guard / pass_at / dist
    kwargs through so plan builders can tune per-task without subclassing.
    """
    TIMEOUT_S = 30.0

    def __init__(
        self,
        duburi,
        profile,
        target: str,
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
            pass_at=pass_at or None,      # 0.0 → None (disabled)
            dist=dist or None,
            metric=metric,
            duration=duration,
            on_lost=on_lost,
        )
        # Strip None-valued optional keys so DSL inherits param defaults
        self._kwargs = {k: v for k, v in self._kwargs.items() if v is not None}
        self.TIMEOUT_S = duration + 5.0

    def _run(self, bb: Blackboard) -> str:
        result = self.duburi.vision.home(**self._kwargs)
        if result.success:
            return SUCCEED
        # DSL returns success=False on on_lost='fail' — signal FAILED
        # so the plan can retry find → home rather than going to SURFACE.
        return FAILED


class VisionScanState(DuburiState):
    """Incremental yaw orbit until target detected or budget exhausted.

    Wraps duburi.vision.scan(). Returns SUCCEED on first detection;
    TIMEOUT if full duration elapses without sighting.
    """
    TIMEOUT_S = 100.0

    def __init__(
        self,
        duburi,
        profile,
        target: str,
        step: float = 20.0,
        dwell: float = 1.5,
        duration: float = 90.0,
    ) -> None:
        super().__init__(duburi, profile, [SUCCEED])
        self._target   = target
        self._step     = step
        self._dwell    = dwell
        self._duration = duration
        self.TIMEOUT_S = duration + 5.0

    def _run(self, bb: Blackboard) -> str:
        result = self.duburi.vision.scan(
            target=self._target,
            step=self._step,
            dwell=self._dwell,
            duration=self._duration,
        )
        return SUCCEED if result.success else TIMEOUT

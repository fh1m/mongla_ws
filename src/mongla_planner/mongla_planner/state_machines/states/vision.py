"""Vision states — wrapping the two MonglaMission vision verbs.

Three states map 1:1 onto the two-verb vision API plus an open-loop
search:

  VisionSearchState -- open-loop creep / yaw-sweep until the target is
                       detected (replaces the old find/scan verbs).
  VisionAlignState  -- mongla.vision.align(): centre on lat/yaw/depth at
                       signed pixel offsets. SUCCEED only when aligned.
  VisionMoveState   -- mongla.vision.move(): drive forward to a bbox fill
                       ratio. SUCCEED only when the fill target is reached.

``camera=None`` falls back to the sticky ``mongla.camera`` context.
Axis flags on VisionAlignState accept ``True`` (centre, offset 0), a
number (signed pixel offset), or ``None``/``False`` (axis off).
"""
from __future__ import annotations

import time

from yasmin import Blackboard

from ..core.base_state import MonglaState
from ..core.outcomes import SUCCEED, FAILED, TIMEOUT


def _axis(value):
    """Normalise an axis flag to align()'s None=off / number=offset form."""
    if value is None or value is False:
        return None
    if value is True:
        return 0.0
    return float(value)


class VisionSearchState(MonglaState):
    """Open-loop search until target detected (replaces find/scan).

    ``pattern='forward'`` creeps ahead in short bursts; ``pattern='yaw'``
    sweeps in yaw steps. Polls ``mongla.detected()`` each step.
    """
    def __init__(
        self,
        mongla,
        profile,
        target: str,
        camera: str | None = None,
        pattern: str = 'forward',
        gain: float = 35.0,
        step_s: float = 0.6,
        yaw_step: float = 20.0,
        timeout: float = 45.0,
    ) -> None:
        super().__init__(mongla, profile, [SUCCEED])
        self._target  = target
        self._camera  = camera
        self._pattern = pattern
        self._gain    = gain
        self._step_s  = step_s
        self._yaw     = yaw_step
        self._timeout = timeout
        self.TIMEOUT_S = timeout + 10.0

    def _run(self, bb: Blackboard) -> str:
        deadline = time.monotonic() + self._timeout
        while time.monotonic() < deadline:
            if self.mongla.detected(self._target, camera=self._camera,
                                    stale_after=1.0):
                return SUCCEED
            if self._pattern == 'yaw':
                self.mongla.yaw_right(self._yaw)
            else:
                self.mongla.move_forward(self._step_s, gain=self._gain)
        return TIMEOUT


class VisionAlignState(MonglaState):
    """mongla.vision.align() — centre on the selected axes at pixel offsets.

    SUCCEED when aligned, FAILED on any miss (LOST / TIMEOUT / NO_CAMERA).
    """
    def __init__(
        self,
        mongla,
        profile,
        target: str,
        camera: str | None = None,
        yaw=None,
        lat=None,
        depth=None,
        fwd=None,
        fwd_mode: str = 'area',
        err: float = 40.0,
        gain: float = 30.0,
        duration: float = 20.0,
        surge_sign: int = 1,
        max_depth_m: float = 0.0,
        fallback=None,
    ) -> None:
        super().__init__(mongla, profile, [SUCCEED, FAILED])
        self._target   = target
        self._camera   = camera
        self._yaw      = _axis(yaw)
        self._lat      = _axis(lat)
        self._depth    = _axis(depth)
        # ``fwd`` on a DOWNWARD camera is the Ch5 fore/aft SURGE pixel offset (the
        # kwargs remap there -- see vision_dsl.align); on a forward camera it is the
        # optional fill%% standoff. ``fwd_mode`` measures the fill. Passed straight
        # through to mongla.vision.align, which owns the downward swap.
        self._fwd      = _axis(fwd)
        self._fwd_mode = fwd_mode
        self._err      = err
        self._gain     = gain
        self._duration = duration
        # Downward-frame knobs (ignored on a forward camera): Ch5 fore/aft surge
        # polarity + deepest allowed setpoint for a fill->depth descent.
        self._surge_sign  = surge_sign
        self._max_depth_m = max_depth_m
        self._fallback = fallback
        self.TIMEOUT_S = duration + 10.0

    def _run(self, bb: Blackboard) -> str:
        result = self.mongla.vision.align(
            self._target, camera=self._camera,
            lat=self._lat, yaw=self._yaw, depth=self._depth,
            fwd=self._fwd, fwd_mode=self._fwd_mode,
            err=self._err, gain=self._gain, duration=self._duration,
            surge_sign=self._surge_sign, max_depth_m=self._max_depth_m,
            fallback=self._fallback)
        return SUCCEED if result.ok else FAILED


class VisionMoveState(MonglaState):
    """mongla.vision.move() — drive forward to a bbox fill ratio.

    SUCCEED when the fill target is reached, FAILED otherwise.
    """
    def __init__(
        self,
        mongla,
        profile,
        target: str,
        camera: str | None = None,
        fwd: float = 95.0,
        mode: str = 'area',
        maintain=None,
        hold=None,
        gain: float = 30.0,
        duration: float = 20.0,
        fallback=None,
    ) -> None:
        super().__init__(mongla, profile, [SUCCEED, FAILED])
        self._target   = target
        self._camera   = camera
        self._fwd      = fwd
        self._mode     = mode
        self._maintain = maintain
        self._hold     = hold
        self._gain     = gain
        self._duration = duration
        self._fallback = fallback
        self.TIMEOUT_S = duration + 10.0

    def _run(self, bb: Blackboard) -> str:
        result = self.mongla.vision.move(
            self._target, camera=self._camera,
            fwd=self._fwd, mode=self._mode,
            maintain=self._maintain, hold=self._hold,
            gain=self._gain, duration=self._duration,
            fallback=self._fallback)
        return SUCCEED if result.ok else FAILED

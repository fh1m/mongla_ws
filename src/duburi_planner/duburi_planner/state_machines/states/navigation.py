"""Navigation states — arm, depth, heading, movement.

MoveForwardState / MoveBackState / MoveLateralState are the plug-and-play
core: pass both distance_m AND duration; each state picks the right verb
based on profile.has_dvl.  Same plan builder → correct FSM for both vehicles.
"""
from __future__ import annotations

from yasmin import Blackboard

from ..core.base_state import DuburiState
from ..core.blackboard import BK
from ..core.outcomes import SUCCEED


# ── ARM ──────────────────────────────────────────────────────────────────────

class ArmState(DuburiState):
    """Arm + optional DVL connect. Sets BK.START_HEADING + BK.DVL_CONNECTED."""
    TIMEOUT_S = 30.0

    def __init__(self, duburi, profile) -> None:
        super().__init__(duburi, profile, [SUCCEED])

    def _run(self, bb: Blackboard) -> str:
        self.duburi.arm()

        if self.profile.has_dvl:
            try:
                self.duburi.dvl_connect()
                bb[BK.DVL_CONNECTED] = True
            except Exception:
                bb[BK.DVL_CONNECTED] = False

        return SUCCEED


# ── DISARM ────────────────────────────────────────────────────────────────────

class DisarmState(DuburiState):
    TIMEOUT_S = 15.0

    def __init__(self, duburi, profile) -> None:
        super().__init__(duburi, profile, [SUCCEED])

    def _run(self, bb: Blackboard) -> str:
        self.duburi.disarm()
        return SUCCEED


# ── SET DEPTH ─────────────────────────────────────────────────────────────────

class SetDepthState(DuburiState):
    """Drive to target depth. TIMEOUT → caller decides whether to retry or abort."""
    TIMEOUT_S = 45.0

    def __init__(self, duburi, profile, depth_m: float, timeout_s: float = 45.0) -> None:
        super().__init__(duburi, profile, [SUCCEED])
        self._depth_m  = depth_m
        self.TIMEOUT_S = timeout_s

    def _run(self, bb: Blackboard) -> str:
        self.duburi.set_depth(self._depth_m, timeout=self.TIMEOUT_S - 2)
        return SUCCEED


# ── LOCK HEADING ─────────────────────────────────────────────────────────────

class LockHeadingState(DuburiState):
    """Engage heading lock. Stores initial heading in BK.START_HEADING."""
    TIMEOUT_S = 30.0

    def __init__(self, duburi, profile, heading: float = 0.0) -> None:
        super().__init__(duburi, profile, [SUCCEED])
        self._heading = heading

    def _run(self, bb: Blackboard) -> str:
        self.duburi.lock_heading(self._heading, timeout=self.TIMEOUT_S - 2)
        bb[BK.START_HEADING] = self._heading
        return SUCCEED


# ── MOVEMENT (plug-and-play DVL / timed) ─────────────────────────────────────

class MoveForwardState(DuburiState):
    """Forward move: DVL distance if profile.has_dvl, else timed.

    Always pass both distance_m and duration — state picks correct verb.
    """
    TIMEOUT_S = 30.0

    def __init__(
        self,
        duburi,
        profile,
        distance_m: float | None = None,
        duration: float | None = None,
        gain: int = 60,
    ) -> None:
        super().__init__(duburi, profile, [SUCCEED])
        self._distance_m = distance_m
        self._duration   = duration
        self._gain       = gain

    def _run(self, bb: Blackboard) -> str:
        if self._distance_m is not None and self.profile.has_dvl:
            self.duburi.move_forward_dist(self._distance_m, gain=self._gain)
        elif self._duration is not None:
            self.duburi.move_forward(self._duration, gain=self._gain)
        return SUCCEED


class MoveBackState(DuburiState):
    TIMEOUT_S = 30.0

    def __init__(
        self,
        duburi,
        profile,
        distance_m: float | None = None,
        duration: float | None = None,
        gain: int = 60,
    ) -> None:
        super().__init__(duburi, profile, [SUCCEED])
        self._distance_m = distance_m
        self._duration   = duration
        self._gain       = gain

    def _run(self, bb: Blackboard) -> str:
        if self._distance_m is not None and self.profile.has_dvl:
            self.duburi.move_back_dist(self._distance_m, gain=self._gain)
        elif self._duration is not None:
            self.duburi.move_back(self._duration, gain=self._gain)
        return SUCCEED


class MoveLateralState(DuburiState):
    TIMEOUT_S = 20.0

    def __init__(
        self,
        duburi,
        profile,
        distance_m: float | None = None,
        duration: float | None = None,
        gain: int = 40,
    ) -> None:
        super().__init__(duburi, profile, [SUCCEED])
        self._distance_m = distance_m
        self._duration   = duration
        self._gain       = gain

    def _run(self, bb: Blackboard) -> str:
        if self._distance_m is not None and self.profile.has_dvl:
            self.duburi.move_lateral_dist(self._distance_m, gain=self._gain)
        elif self._duration is not None:
            # positive distance_m = right; mirror for raw timed move
            self.duburi.move_right(self._duration, gain=self._gain)
        return SUCCEED


# ── TURN (absolute heading snap) ─────────────────────────────────────────────

class TurnState(DuburiState):
    """Snap to absolute compass heading via duburi.turn()."""
    TIMEOUT_S = 30.0

    def __init__(self, duburi, profile, heading_deg: float) -> None:
        super().__init__(duburi, profile, [SUCCEED])
        self._heading_deg = heading_deg

    def _run(self, bb: Blackboard) -> str:
        self.duburi.turn(self._heading_deg)
        return SUCCEED


# ── SURFACE (safe exit) ───────────────────────────────────────────────────────

class SurfaceState(DuburiState):
    """Emergency/planned surface: stop thrusters, ascend to 0m, disarm."""
    TIMEOUT_S = 90.0

    def __init__(self, duburi, profile) -> None:
        super().__init__(duburi, profile, [SUCCEED])

    def _run(self, bb: Blackboard) -> str:
        try:
            self.duburi.release_heading()
        except Exception:
            pass
        self.duburi.stop()
        try:
            self.duburi.set_depth(0.0, timeout=60)
        except Exception:
            pass
        self.duburi.disarm()
        return SUCCEED

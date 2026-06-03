"""Gate pass → bin drop — 2-task FSM demonstrating camera switching.

Task 1 (forward camera, gate+flare model):
  Arm → dive → find gate → home gate → pass gate

Task 2 (downward camera, bin model):
  SetDetector switch → scan for bin → lock above bin → drop → confirm → surface

VehicleProfile controls DVL vs timed for all movement states.
Camera is explicit per-state — no reliance on sticky duburi.camera between tasks.
"""
from __future__ import annotations

from yasmin import StateMachine

from ..core.outcomes import SUCCEED, FAILED, TIMEOUT, ABORT
from ..core.vehicle_profile import VehicleProfile
from ..states.navigation import (
    ArmState, SetDepthState, LockHeadingState,
    MoveForwardState, SurfaceState,
)
from ..states.vision import VisionFindState, VisionHomeState, VisionScanState
from ..states.utility import CountdownState, LogScoreState, SetDetectorState

GATE_THEN_BIN_DEFAULTS: dict = {
    'countdown_s':      10,
    'gate_depth_m':    -0.8,    # task-1 depth
    'bin_depth_m':     -1.5,    # descend for bin scan (bin deeper)
    'gate_heading':     0.0,    # compass heading — set at pool day
    # gate task
    'pass_dist_m':      3.5,
    'pass_duration':    5.0,
    'pass_gain':        80,
    'find_timeout':     45.0,
    'home_duration':    20.0,
    # detector models
    'gate_model':      'gate_flare_medium_100ep',
    'bin_model':       'bin_medium_100ep',
    'gate_classes':    'gate,flare',
    'bin_classes':     'bin_a,bin_b',
    # bin task
    'bin_scan_step':    20.0,
    'bin_scan_dwell':   1.5,
    'bin_scan_duration': 90.0,
    'bin_home_duration': 20.0,
}


def build_gate_then_bin_fsm(
    duburi,
    profile: VehicleProfile,
    params: dict | None = None,
) -> StateMachine:
    """Build gate-pass → bin-drop FSM with explicit camera switching.

    State transitions:
        COUNTDOWN → ARM → DIVE_GATE → LOCK_HEADING
          → FIND_GATE → HOME_GATE → PASS_GATE
          → SWITCH_TO_BIN        ← camera + classes change here
          → DIVE_BIN → SCAN_BIN → LOCK_BIN → DROP_BIN
          → CONFIRM_DROP → LOG_SCORE → SURFACE → succeeded
    """
    p = {**GATE_THEN_BIN_DEFAULTS, **(params or {})}
    sm = StateMachine(outcomes=[SUCCEED, ABORT])

    # ── startup ──────────────────────────────────────────────────────────
    sm.add_state('COUNTDOWN',
                 CountdownState(duburi, profile, seconds=p['countdown_s']),
                 transitions={SUCCEED: 'ARM', ABORT: ABORT})

    sm.add_state('ARM',
                 ArmState(duburi, profile),
                 transitions={SUCCEED: 'DIVE_GATE', ABORT: 'SURFACE'})

    sm.add_state('DIVE_GATE',
                 SetDepthState(duburi, profile, depth_m=p['gate_depth_m']),
                 transitions={SUCCEED: 'LOCK_HEADING', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('LOCK_HEADING',
                 LockHeadingState(duburi, profile, heading=p['gate_heading']),
                 transitions={SUCCEED: 'FIND_GATE', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    # ── task 1: gate (forward camera) ────────────────────────────────────
    sm.add_state('FIND_GATE',
                 VisionFindState(duburi, profile,
                                 target='gate',
                                 camera='forward',       # explicit
                                 move='forward', gain=30,
                                 timeout=p['find_timeout']),
                 transitions={SUCCEED: 'HOME_GATE', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('HOME_GATE',
                 VisionHomeState(duburi, profile,
                                 target='gate',
                                 camera='forward',       # explicit
                                 yaw=True, lat=True,
                                 gate_guard=True, pass_at=0.38,
                                 metric='area',
                                 duration=p['home_duration']),
                 transitions={SUCCEED: 'PASS_GATE',
                               FAILED: 'FIND_GATE',
                               TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('PASS_GATE',
                 MoveForwardState(duburi, profile,
                                  distance_m=p['pass_dist_m'],
                                  duration=p['pass_duration'],
                                  gain=p['pass_gain']),
                 transitions={SUCCEED: 'SWITCH_TO_BIN', ABORT: 'SURFACE'})

    # ── camera + model switch ─────────────────────────────────────────────
    # Atomic: set sticky camera to 'downward', load bin model, filter classes.
    # After this state every subsequent vision state uses 'downward' by default.
    sm.add_state('SWITCH_TO_BIN',
                 SetDetectorState(duburi, profile,
                                  camera='downward',
                                  model=p['bin_model'],
                                  classes=p['bin_classes']),
                 transitions={SUCCEED: 'DIVE_BIN', ABORT: 'SURFACE'})

    # ── task 2: bin (downward camera) ────────────────────────────────────
    sm.add_state('DIVE_BIN',
                 SetDepthState(duburi, profile, depth_m=p['bin_depth_m']),
                 transitions={SUCCEED: 'SCAN_BIN', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('SCAN_BIN',
                 VisionScanState(duburi, profile,
                                 target='bin_a',
                                 camera='downward',      # explicit
                                 step=p['bin_scan_step'],
                                 dwell=p['bin_scan_dwell'],
                                 duration=p['bin_scan_duration']),
                 transitions={SUCCEED: 'LOCK_BIN', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('LOCK_BIN',
                 VisionHomeState(duburi, profile,
                                 target='bin_a',
                                 camera='downward',      # explicit
                                 yaw=True, lat=True,
                                 depth=True,             # nudge depth: enlarge bbox
                                 metric='area',
                                 duration=p['bin_home_duration'],
                                 on_lost='fail'),
                 transitions={SUCCEED: 'DROP_BIN',
                               FAILED: 'SCAN_BIN',
                               TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    # DROP_BIN and CONFIRM_DROP are stubs here — wire to your ESP32 actuator
    # state when the payload serial contract is finalised (see project_payload_actuation).
    # For now they are no-op pauses so the FSM compiles and runs end-to-end.
    sm.add_state('DROP_BIN',
                 _DropStubState(duburi, profile),
                 transitions={SUCCEED: 'CONFIRM_DROP', ABORT: 'SURFACE'})

    sm.add_state('CONFIRM_DROP',
                 _ConfirmDropStubState(duburi, profile, camera='downward'),
                 transitions={SUCCEED: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('LOG_SCORE',
                 LogScoreState(duburi, profile),
                 transitions={SUCCEED: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('SURFACE',
                 SurfaceState(duburi, profile),
                 transitions={SUCCEED: SUCCEED, ABORT: ABORT})

    sm.set_start_state('COUNTDOWN')
    return sm


# ---------------------------------------------------------------------------
# Payload stubs — replace with real ESP32 actuator states when serial
# contract is locked. See memory: project_payload_actuation.
# ---------------------------------------------------------------------------

from ..core.base_state import DuburiState
from ..core.blackboard import BK


class _DropStubState(DuburiState):
    """Stub drop actuator — logs intent, pauses 1s. Replace with ESP32 cmd."""
    TIMEOUT_S = 10.0

    def __init__(self, duburi, profile) -> None:
        super().__init__(duburi, profile, [SUCCEED])

    def _run(self, bb):
        try:
            self.duburi.log.info('[DROP ] stub — actuator not wired yet')
        except Exception:
            pass
        self.duburi.pause(1.0)
        return SUCCEED


class _ConfirmDropStubState(DuburiState):
    """Stub drop confirm — polls for target disappearance from downward cam.
    Falls back to timeout after 3s (assume dropped).
    Replace with real current/bbox confirm when hardware available.
    """
    TIMEOUT_S = 8.0

    def __init__(self, duburi, profile, camera: str = 'downward') -> None:
        super().__init__(duburi, profile, [SUCCEED])
        self._camera = camera

    def _run(self, bb):
        import time
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if not self.duburi.detected('bin_a', camera=self._camera, stale_after=0.5):
                bb[BK.LAST_ERROR] = ''
                return SUCCEED
            self.duburi.pause(0.3)
        return SUCCEED   # timeout → assume dropped; don't abort to surface

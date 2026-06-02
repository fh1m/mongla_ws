"""Gate + Flare FSM — RoboSub 2026 primary task.

build_gate_flare_fsm() produces a StateMachine that works for both bodies:
  - Duburi 4.5  (has_dvl=True)  → passage moves use DVL distance
  - Dubomini 2.0 (has_dvl=False) → passage moves use timed thrust

Override any GATE_FLARE_DEFAULTS key via the params dict argument.
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
from ..states.utility import CountdownState, LogScoreState

GATE_FLARE_DEFAULTS: dict = {
    'countdown_s':    10,
    'depth_m':       -0.8,
    'gate_heading':   0.0,    # compass heading to gate — set at pool day
    'pass_dist_m':    3.5,    # DVL distance through gate
    'pass_duration':  5.0,    # timed fallback (Dubomini)
    'pass_gain':      80,
    'return_dist_m':  1.5,
    'return_duration': 3.0,
    'find_timeout':   45.0,
    'home_duration':  20.0,
    'scan_step':      20.0,
    'scan_dwell':     1.5,
    'scan_duration':  90.0,
}


def build_gate_flare_fsm(
    duburi,
    profile: VehicleProfile,
    params: dict | None = None,
) -> StateMachine:
    """Build and return the gate+flare StateMachine.

    The returned machine is NOT yet executing. Call sm(Blackboard()) to run.
    """
    p = {**GATE_FLARE_DEFAULTS, **(params or {})}
    sm = StateMachine(outcomes=[SUCCEED, ABORT])

    sm.add_state('COUNTDOWN',
                 CountdownState(duburi, profile, seconds=p['countdown_s']),
                 transitions={SUCCEED: 'ARM', ABORT: ABORT})

    sm.add_state('ARM',
                 ArmState(duburi, profile),
                 transitions={SUCCEED: 'DIVE', ABORT: 'SURFACE'})

    sm.add_state('DIVE',
                 SetDepthState(duburi, profile, depth_m=p['depth_m']),
                 transitions={SUCCEED: 'LOCK_HEADING', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('LOCK_HEADING',
                 LockHeadingState(duburi, profile, heading=p['gate_heading']),
                 transitions={SUCCEED: 'FIND_GATE', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('FIND_GATE',
                 VisionFindState(duburi, profile, target='gate',
                                 move='forward', gain=30,
                                 timeout=p['find_timeout']),
                 transitions={SUCCEED: 'HOME_GATE', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('HOME_GATE',
                 VisionHomeState(duburi, profile, target='gate',
                                 yaw=True, lat=True,
                                 gate_guard=True, pass_at=0.38,
                                 metric='area',
                                 duration=p['home_duration']),
                 transitions={SUCCEED: 'PASS_GATE',
                               FAILED: 'FIND_GATE',   # lost target → re-find
                               TIMEOUT: 'SURFACE',
                               ABORT: 'SURFACE'})

    sm.add_state('PASS_GATE',
                 MoveForwardState(duburi, profile,
                                  distance_m=p['pass_dist_m'],
                                  duration=p['pass_duration'],
                                  gain=p['pass_gain']),
                 transitions={SUCCEED: 'FIND_FLARE', ABORT: 'SURFACE'})

    sm.add_state('FIND_FLARE',
                 VisionFindState(duburi, profile, target='flare',
                                 move='forward', gain=30,
                                 timeout=p['find_timeout']),
                 transitions={SUCCEED: 'HOME_FLARE', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('HOME_FLARE',
                 VisionHomeState(duburi, profile, target='flare',
                                 yaw=True, lat=True, depth=True, forward=True,
                                 dist=0.38, metric='height',
                                 duration=p['home_duration']),
                 transitions={SUCCEED: 'SCAN_GATE',
                               FAILED: 'FIND_FLARE',
                               TIMEOUT: 'SURFACE',
                               ABORT: 'SURFACE'})

    sm.add_state('SCAN_GATE',
                 VisionScanState(duburi, profile, target='gate',
                                 step=p['scan_step'],
                                 dwell=p['scan_dwell'],
                                 duration=p['scan_duration']),
                 transitions={SUCCEED: 'RETURN_HOME', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('RETURN_HOME',
                 VisionHomeState(duburi, profile, target='gate',
                                 yaw=True, lat=True,
                                 gate_guard=True, pass_at=0.38,
                                 metric='area',
                                 duration=p['home_duration']),
                 transitions={SUCCEED: 'RETURN_PASS',
                               FAILED: 'SCAN_GATE',
                               TIMEOUT: 'SURFACE',
                               ABORT: 'SURFACE'})

    sm.add_state('RETURN_PASS',
                 MoveForwardState(duburi, profile,
                                  distance_m=p['return_dist_m'],
                                  duration=p['return_duration'],
                                  gain=p['pass_gain']),
                 transitions={SUCCEED: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('LOG_SCORE',
                 LogScoreState(duburi, profile),
                 transitions={SUCCEED: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('SURFACE',
                 SurfaceState(duburi, profile),
                 transitions={SUCCEED: SUCCEED, ABORT: ABORT})

    sm.set_start_state('COUNTDOWN')
    return sm

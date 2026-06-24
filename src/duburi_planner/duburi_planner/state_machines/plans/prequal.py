"""Pre-qualification FSM — gate pass only (RoboSub prequal format).

Simpler than gate_flare: COUNTDOWN → ARM → DIVE → FIND_GATE → HOME_GATE
→ PASS_GATE → LOG_SCORE → SURFACE.

Works on both vehicles via VehicleProfile.
"""
from __future__ import annotations

from yasmin import StateMachine

from ..core.outcomes import SUCCEED, FAILED, TIMEOUT, ABORT
from ..core.vehicle_profile import VehicleProfile
from ..states.navigation import (
    ArmState, SetDepthState, LockHeadingState, MoveForwardState, SurfaceState,
)
from ..states.vision import VisionSearchState, VisionAlignState
from ..states.utility import CountdownState, LogScoreState

PREQUAL_DEFAULTS: dict = {
    'countdown_s':   10,
    'depth_m':      -0.8,
    'gate_heading':  0.0,
    'pass_dist_m':   3.5,
    'pass_duration': 5.0,
    'pass_gain':     80,
    'find_timeout':  45.0,
    'align_duration': 20.0,
    'align_err_px':  40,
    'align_gain':    30,
}


def build_prequal_fsm(
    duburi,
    profile: VehicleProfile,
    params: dict | None = None,
) -> StateMachine:
    p = {**PREQUAL_DEFAULTS, **(params or {})}
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
                 VisionSearchState(duburi, profile, target='gate',
                                   pattern='forward', gain=30,
                                   timeout=p['find_timeout']),
                 transitions={SUCCEED: 'HOME_GATE', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('HOME_GATE',
                 VisionAlignState(duburi, profile, target='gate',
                                  yaw=True, lat=True,
                                  err=p['align_err_px'], gain=p['align_gain'],
                                  duration=p['align_duration']),
                 transitions={SUCCEED: 'PASS_GATE',
                              FAILED: 'FIND_GATE',
                              TIMEOUT: 'SURFACE',
                              ABORT: 'SURFACE'})

    sm.add_state('PASS_GATE',
                 MoveForwardState(duburi, profile,
                                  distance_m=p['pass_dist_m'],
                                  duration=p['pass_duration'],
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

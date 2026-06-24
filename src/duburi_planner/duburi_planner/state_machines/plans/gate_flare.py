"""Gate + Flare FSM — RoboSub 2026 primary task.

build_gate_flare_fsm() produces a StateMachine that works for both bodies:
  - Duburi 4.5  (has_dvl=True)  → passage moves use DVL distance
  - Dubomini 2.0 (has_dvl=False) → passage moves use timed thrust

Vision is the two-verb API: search (open-loop) → align (centre) →
move (drive in). Each align/move SUCCEED only on a real outcome; a miss
routes back to search.

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
from ..states.vision import VisionSearchState, VisionAlignState, VisionMoveState
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
    'align_duration': 20.0,
    'move_duration':  20.0,
    'align_err_px':   40,
    'align_gain':     30,
    'approach_gain':  45,
    'gate_fwd_fill':  42,     # gate area % at standoff
    'flare_fwd_fill': 38,     # flare height % at standoff
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
                 VisionSearchState(duburi, profile, target='gate',
                                   pattern='forward', gain=30,
                                   timeout=p['find_timeout']),
                 transitions={SUCCEED: 'HOME_GATE', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('HOME_GATE',
                 VisionAlignState(duburi, profile, target='gate',
                                  yaw=True, lat=True,
                                  err=p['align_err_px'], gain=p['align_gain'],
                                  duration=p['align_duration']),
                 transitions={SUCCEED: 'MOVE_GATE',
                              FAILED: 'FIND_GATE',
                              TIMEOUT: 'SURFACE',
                              ABORT: 'SURFACE'})

    sm.add_state('MOVE_GATE',
                 VisionMoveState(duburi, profile, target='gate',
                                 fwd=p['gate_fwd_fill'], mode='area',
                                 gain=p['approach_gain'],
                                 duration=p['move_duration']),
                 transitions={SUCCEED: 'PASS_GATE',
                              FAILED: 'PASS_GATE',   # close enough — commit pass
                              TIMEOUT: 'SURFACE',
                              ABORT: 'SURFACE'})

    sm.add_state('PASS_GATE',
                 MoveForwardState(duburi, profile,
                                  distance_m=p['pass_dist_m'],
                                  duration=p['pass_duration'],
                                  gain=p['pass_gain']),
                 transitions={SUCCEED: 'FIND_FLARE', ABORT: 'SURFACE'})

    sm.add_state('FIND_FLARE',
                 VisionSearchState(duburi, profile, target='flare',
                                   pattern='forward', gain=30,
                                   timeout=p['find_timeout']),
                 transitions={SUCCEED: 'HOME_FLARE', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('HOME_FLARE',
                 VisionAlignState(duburi, profile, target='flare',
                                  yaw=True, depth=True,
                                  err=p['align_err_px'], gain=p['align_gain'],
                                  duration=p['align_duration']),
                 transitions={SUCCEED: 'MOVE_FLARE',
                              FAILED: 'FIND_FLARE',
                              TIMEOUT: 'SURFACE',
                              ABORT: 'SURFACE'})

    sm.add_state('MOVE_FLARE',
                 VisionMoveState(duburi, profile, target='flare',
                                 fwd=p['flare_fwd_fill'], mode='height',
                                 gain=p['approach_gain'],
                                 duration=p['move_duration']),
                 transitions={SUCCEED: 'SCAN_GATE',
                              FAILED: 'SCAN_GATE',
                              TIMEOUT: 'SURFACE',
                              ABORT: 'SURFACE'})

    sm.add_state('SCAN_GATE',
                 VisionSearchState(duburi, profile, target='gate',
                                   pattern='yaw', yaw_step=20.0,
                                   timeout=p['find_timeout']),
                 transitions={SUCCEED: 'RETURN_HOME', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('RETURN_HOME',
                 VisionAlignState(duburi, profile, target='gate',
                                  yaw=True, lat=True,
                                  err=p['align_err_px'], gain=p['align_gain'],
                                  duration=p['align_duration']),
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

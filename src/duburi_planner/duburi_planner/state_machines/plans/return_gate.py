"""Return gate task FSM — find gate again, pass back through, style roll.

Standalone:
    ros2 run duburi_planner mission fsm_return

Task flow:
    COUNTDOWN → ARM → SET_DEPTH → LOCK_HEADING → [TURN_RETURN]
    → SWITCH_RETURN → FIND_GATE
    → HOME_GATE → SET_PASS_DEPTH → APPROACH_GATE → PAUSE
    → SURFACE_LEVEL → STYLE_ROLL
    → LOG_SCORE → SURFACE

Pool-day params to fill: return_heading.
"""
from __future__ import annotations

from yasmin import StateMachine

from ..core.outcomes import SUCCEED, FAILED, TIMEOUT, ABORT
from ..core.vehicle_profile import VehicleProfile
from ..states.navigation import (
    ArmState, SetDepthState, LockHeadingState, TurnState, SurfaceState,
)
from ..states.vision import VisionFindState, VisionHomeState, ApproachState
from ..states.utility import (
    CountdownState, PauseState, LogScoreState, SetDetectorState, StyleRollState,
)

RETURN_GATE_DEFAULTS: dict = {
    'countdown_s':       10,
    'search_depth_m':   -0.4,
    'gate_heading':      0.0,
    'return_heading':    None,          # compass bearing back to gate; fill at pool
    'pass_depth_m':     -0.6,
    'pass_bbox_frac':    0.80,
    'find_timeout':      60.0,
    'home_duration':     15.0,
    'approach_duration': 25.0,
    'style_roll_flips':  1,
    'style_roll_headroom': 0.4,
    'style_roll_gain':   60,
    'model':            'gate_rescue_repair',
    'classes':          'gate',
}


def build_return_gate_fsm(
    duburi,
    profile: VehicleProfile,
    params: dict | None = None,
) -> StateMachine:
    p = {**RETURN_GATE_DEFAULTS, **(params or {})}

    sm = StateMachine(outcomes=[SUCCEED, ABORT])

    sm.add_state('COUNTDOWN',
                 CountdownState(duburi, profile, seconds=p['countdown_s']),
                 transitions={SUCCEED: 'ARM', ABORT: ABORT})

    sm.add_state('ARM',
                 ArmState(duburi, profile),
                 transitions={SUCCEED: 'SET_DEPTH', ABORT: 'SURFACE'})

    sm.add_state('SET_DEPTH',
                 SetDepthState(duburi, profile, depth_m=p['search_depth_m']),
                 transitions={SUCCEED: 'LOCK_HEADING', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    _after_lock = 'TURN_RETURN' if p['return_heading'] is not None else 'SWITCH_RETURN'
    sm.add_state('LOCK_HEADING',
                 LockHeadingState(duburi, profile, heading=p['gate_heading']),
                 transitions={SUCCEED: _after_lock, TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    if p['return_heading'] is not None:
        sm.add_state('TURN_RETURN',
                     TurnState(duburi, profile, heading_deg=p['return_heading']),
                     transitions={SUCCEED: 'SWITCH_RETURN', ABORT: 'SURFACE'})

    sm.add_state('SWITCH_RETURN',
                 SetDetectorState(duburi, profile,
                                  camera='forward',
                                  model=p['model'],
                                  classes=p['classes']),
                 transitions={SUCCEED: 'FIND_GATE', ABORT: 'SURFACE'})

    sm.add_state('FIND_GATE',
                 VisionFindState(duburi, profile,
                                 target='gate', camera='forward',
                                 move='forward', gain=30,
                                 timeout=p['find_timeout']),
                 transitions={SUCCEED: 'HOME_GATE', TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('HOME_GATE',
                 VisionHomeState(duburi, profile,
                                 target='gate', camera='forward',
                                 yaw=True, lat=True,
                                 duration=p['home_duration'],
                                 on_lost='hold'),
                 transitions={SUCCEED: 'SET_PASS_DEPTH',
                               FAILED: 'LOG_SCORE',
                               TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('SET_PASS_DEPTH',
                 SetDepthState(duburi, profile, depth_m=p['pass_depth_m']),
                 transitions={SUCCEED: 'APPROACH_GATE', TIMEOUT: 'APPROACH_GATE', ABORT: 'SURFACE'})

    sm.add_state('APPROACH_GATE',
                 ApproachState(duburi, profile,
                               target='gate', camera='forward',
                               dist=p['pass_bbox_frac'], metric='height',
                               duration=p['approach_duration'],
                               lock_mode='pursue', on_lost='hold'),
                 transitions={SUCCEED: 'PAUSE',
                               FAILED: 'LOG_SCORE',
                               TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('PAUSE',
                 PauseState(duburi, profile, seconds=1.0),
                 transitions={SUCCEED: 'SURFACE_LEVEL', ABORT: 'SURFACE'})

    sm.add_state('SURFACE_LEVEL',
                 SetDepthState(duburi, profile, depth_m=p['search_depth_m']),
                 transitions={SUCCEED: 'STYLE_ROLL', TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('STYLE_ROLL',
                 StyleRollState(duburi, profile,
                                flips=p['style_roll_flips'],
                                headroom=p['style_roll_headroom'],
                                gain=p['style_roll_gain']),
                 transitions={SUCCEED: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('LOG_SCORE',
                 LogScoreState(duburi, profile),
                 transitions={SUCCEED: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('SURFACE',
                 SurfaceState(duburi, profile),
                 transitions={SUCCEED: SUCCEED, ABORT: ABORT})

    sm.set_start_state('COUNTDOWN')
    return sm

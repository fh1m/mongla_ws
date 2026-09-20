"""Return gate task FSM — find gate again, pass back through, style roll.

Standalone:
    ros2 run mongla_planner mission fsm_return

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
from ..states.vision import VisionSearchState, VisionAlignState, VisionMoveState
from ..states.utility import (
    CountdownState, PauseState, LogScoreState, SetDetectorState, StyleRollState,
)

RETURN_GATE_DEFAULTS: dict = {
    'countdown_s':       10,
    'search_depth_m':   -0.4,
    'gate_heading':      0.0,
    'return_heading':    None,          # compass bearing back to gate; fill at pool
    'pass_depth_m':     -0.6,
    'pass_fwd_fill':     80,            # % frame the gate fills when passing
    'find_timeout':      60.0,
    'align_duration':    15.0,
    'align_err_px':      40,
    'align_gain':        30,
    'approach_duration': 25.0,
    'approach_gain':     45,
    'style_roll_flips':  1,
    'style_roll_headroom': 0.4,
    'style_roll_gain':   60,
    'model':            'gate_rescue_repair',
    'classes':          'gate',
}


def build_return_gate_fsm(
    mongla,
    profile: VehicleProfile,
    params: dict | None = None,
) -> StateMachine:
    p = {**RETURN_GATE_DEFAULTS, **(params or {})}

    sm = StateMachine(outcomes=[SUCCEED, ABORT])

    sm.add_state('COUNTDOWN',
                 CountdownState(mongla, profile, seconds=p['countdown_s']),
                 transitions={SUCCEED: 'ARM', ABORT: ABORT})

    sm.add_state('ARM',
                 ArmState(mongla, profile),
                 transitions={SUCCEED: 'SET_DEPTH', ABORT: 'SURFACE'})

    sm.add_state('SET_DEPTH',
                 SetDepthState(mongla, profile, depth_m=p['search_depth_m']),
                 transitions={SUCCEED: 'LOCK_HEADING', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    _after_lock = 'TURN_RETURN' if p['return_heading'] is not None else 'SWITCH_RETURN'
    sm.add_state('LOCK_HEADING',
                 LockHeadingState(mongla, profile, heading=p['gate_heading']),
                 transitions={SUCCEED: _after_lock, TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    if p['return_heading'] is not None:
        sm.add_state('TURN_RETURN',
                     TurnState(mongla, profile, heading_deg=p['return_heading']),
                     transitions={SUCCEED: 'SWITCH_RETURN', ABORT: 'SURFACE'})

    sm.add_state('SWITCH_RETURN',
                 SetDetectorState(mongla, profile,
                                  camera='forward',
                                  model=p['model'],
                                  classes=p['classes']),
                 transitions={SUCCEED: 'FIND_GATE', ABORT: 'SURFACE'})

    sm.add_state('FIND_GATE',
                 VisionSearchState(mongla, profile,
                                   target='gate', camera='forward',
                                   pattern='forward', gain=30,
                                   timeout=p['find_timeout']),
                 transitions={SUCCEED: 'HOME_GATE', TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('HOME_GATE',
                 VisionAlignState(mongla, profile,
                                  target='gate', camera='forward',
                                  yaw=0, lat=0,
                                  err=p['align_err_px'], gain=p['align_gain'],
                                  duration=p['align_duration']),
                 transitions={SUCCEED: 'SET_PASS_DEPTH',
                              FAILED: 'LOG_SCORE',
                              TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('SET_PASS_DEPTH',
                 SetDepthState(mongla, profile, depth_m=p['pass_depth_m']),
                 transitions={SUCCEED: 'APPROACH_GATE', TIMEOUT: 'APPROACH_GATE', ABORT: 'SURFACE'})

    sm.add_state('APPROACH_GATE',
                 VisionMoveState(mongla, profile,
                                 target='gate', camera='forward',
                                 fwd=p['pass_fwd_fill'], mode='height',
                                 gain=p['approach_gain'],
                                 duration=p['approach_duration']),
                 transitions={SUCCEED: 'PAUSE',
                              FAILED: 'LOG_SCORE',
                              TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('PAUSE',
                 PauseState(mongla, profile, seconds=1.0),
                 transitions={SUCCEED: 'SURFACE_LEVEL', ABORT: 'SURFACE'})

    sm.add_state('SURFACE_LEVEL',
                 SetDepthState(mongla, profile, depth_m=p['search_depth_m']),
                 transitions={SUCCEED: 'STYLE_ROLL', TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('STYLE_ROLL',
                 StyleRollState(mongla, profile,
                                flips=p['style_roll_flips'],
                                headroom=p['style_roll_headroom'],
                                gain=p['style_roll_gain']),
                 transitions={SUCCEED: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('LOG_SCORE',
                 LogScoreState(mongla, profile),
                 transitions={SUCCEED: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('SURFACE',
                 SurfaceState(mongla, profile),
                 transitions={SUCCEED: SUCCEED, ABORT: ABORT})

    sm.set_start_state('COUNTDOWN')
    return sm

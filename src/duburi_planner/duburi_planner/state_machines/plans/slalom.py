"""Slalom task FSM — weave through red pipes.

Standalone:
    ros2 run duburi_planner mission fsm_slalom

Task flow:
    COUNTDOWN → ARM → SET_DEPTH → LOCK_HEADING → [TURN_SLALOM]
    → SWITCH_SLALOM → FIND_PIPE
    → SLALOM_L → PAUSE_L → SLALOM_R → PAUSE_R
    → LOG_SCORE → SURFACE

Pool-day params to fill: slalom_heading (compass bearing to slalom course).
"""
from __future__ import annotations

from yasmin import StateMachine

from ..core.outcomes import SUCCEED, FAILED, TIMEOUT, ABORT
from ..core.vehicle_profile import VehicleProfile
from ..states.navigation import (
    ArmState, SetDepthState, LockHeadingState, TurnState, SurfaceState,
)
from ..states.vision import VisionFindState, VisionHomeState, VisionScanState
from ..states.utility import CountdownState, PauseState, LogScoreState, SetDetectorState

SLALOM_DEFAULTS: dict = {
    'countdown_s':      10,
    'search_depth_m':  -0.4,
    'gate_heading':     0.0,
    'slalom_heading':   None,       # compass bearing to slalom; fill at pool
    'pipe_offset_px':   80,         # lateral pixel offset; positive = right
    'find_timeout':     60.0,
    'home_duration':    20.0,
    'model':           'slalom_red_pipe',
    'classes':         'red_pipe',
}


def build_slalom_fsm(
    duburi,
    profile: VehicleProfile,
    params: dict | None = None,
) -> StateMachine:
    p = {**SLALOM_DEFAULTS, **(params or {})}

    # Entry point for the task body (after optional heading turn)
    task_entry = 'SWITCH_SLALOM'

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

    # LOCK_HEADING transitions into TURN_SLALOM (if heading set) else task entry
    _after_lock = 'TURN_SLALOM' if p['slalom_heading'] is not None else task_entry
    sm.add_state('LOCK_HEADING',
                 LockHeadingState(duburi, profile, heading=p['gate_heading']),
                 transitions={SUCCEED: _after_lock, TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    if p['slalom_heading'] is not None:
        sm.add_state('TURN_SLALOM',
                     TurnState(duburi, profile, heading_deg=p['slalom_heading']),
                     transitions={SUCCEED: task_entry, ABORT: 'SURFACE'})

    sm.add_state('SWITCH_SLALOM',
                 SetDetectorState(duburi, profile,
                                  camera='forward',
                                  model=p['model'],
                                  classes=p['classes']),
                 transitions={SUCCEED: 'FIND_PIPE', ABORT: 'SURFACE'})

    sm.add_state('FIND_PIPE',
                 VisionFindState(duburi, profile,
                                 target='red_pipe',
                                 camera='forward',
                                 move='forward', gain=30,
                                 timeout=p['find_timeout']),
                 transitions={SUCCEED: 'SLALOM_L', TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('SLALOM_L',
                 VisionHomeState(duburi, profile,
                                 target='red_pipe', camera='forward',
                                 yaw=True, lat=True,
                                 offset_x=+p['pipe_offset_px'],
                                 duration=p['home_duration'],
                                 on_lost='hold'),
                 transitions={SUCCEED: 'PAUSE_L', FAILED: 'LOG_SCORE',
                               TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('PAUSE_L',
                 PauseState(duburi, profile, seconds=0.5),
                 transitions={SUCCEED: 'SLALOM_R', ABORT: 'SURFACE'})

    sm.add_state('SLALOM_R',
                 VisionHomeState(duburi, profile,
                                 target='red_pipe', camera='forward',
                                 yaw=True, lat=True,
                                 offset_x=-p['pipe_offset_px'],
                                 duration=p['home_duration'],
                                 on_lost='hold'),
                 transitions={SUCCEED: 'PAUSE_R', FAILED: 'LOG_SCORE',
                               TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('PAUSE_R',
                 PauseState(duburi, profile, seconds=0.5),
                 transitions={SUCCEED: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('LOG_SCORE',
                 LogScoreState(duburi, profile),
                 transitions={SUCCEED: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('SURFACE',
                 SurfaceState(duburi, profile),
                 transitions={SUCCEED: SUCCEED, ABORT: ABORT})

    sm.set_start_state('COUNTDOWN')
    return sm

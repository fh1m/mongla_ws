"""Torpedo task FSM — align to torpedo board, lock on hole, fire.

Standalone:
    ros2 run duburi_planner mission fsm_torpedo

Task flow:
    COUNTDOWN → ARM → LOCK_HEADING → [TURN_TORPEDO] → SET_DEPTH
    → SWITCH_TORPEDO → FIND_BOARD
    → HOME_BOARD (coarse: full board)
    → SWITCH_FINE → HOME_BLOOD (fine: blood marker → easier to centre)
    → LOCK_HOLE (tight align on hole) → FIRE → PAUSE
    → LOG_SCORE → SURFACE

Pool-day params to fill: torpedo_heading, torpedo_depth_m.
fire_channel=1 → torpedo_1 (channel always explicit; never rely on defaults for fire).
"""
from __future__ import annotations

from yasmin import StateMachine

from ..core.outcomes import SUCCEED, FAILED, TIMEOUT, ABORT
from ..core.vehicle_profile import VehicleProfile
from ..states.navigation import (
    ArmState, SetDepthState, LockHeadingState, TurnState, SurfaceState,
)
from ..states.vision import VisionSearchState, VisionAlignState
from ..states.utility import (
    CountdownState, PauseState, LogScoreState, SetDetectorState, FireState,
)

TORPEDO_FIRE_DEFAULTS: dict = {
    'countdown_s':       10,
    'gate_heading':      0.0,
    'torpedo_heading':   None,          # compass bearing to board; fill at pool
    'torpedo_depth_m':   None,          # depth to align with hole; MUST fill at pool
    'fire_channel':      1,             # torpedo_1 — always explicit
    'find_timeout':      90.0,
    'coarse_err_px':     50,            # full-board alignment is loose
    'coarse_dur':        15.0,
    'coarse_gain':       30,
    'fine_err_px':       14,            # hole lock is tight
    'fine_dur':          12.0,
    'lock_dur':          60.0,
    'fine_gain':         25,
    'model':            'torpedo_blood_hole',
    'board_classes':    'torpedo',
    'fine_classes':     'blood,hole',
}


def build_torpedo_fire_fsm(
    duburi,
    profile: VehicleProfile,
    params: dict | None = None,
) -> StateMachine:
    p = {**TORPEDO_FIRE_DEFAULTS, **(params or {})}

    assert p['torpedo_depth_m'] is not None, (
        'torpedo_depth_m not set — fill in TORPEDO_FIRE_DEFAULTS or params= before pool run')

    sm = StateMachine(outcomes=[SUCCEED, ABORT])

    sm.add_state('COUNTDOWN',
                 CountdownState(duburi, profile, seconds=p['countdown_s']),
                 transitions={SUCCEED: 'ARM', ABORT: ABORT})

    sm.add_state('ARM',
                 ArmState(duburi, profile),
                 transitions={SUCCEED: 'LOCK_HEADING', ABORT: 'SURFACE'})

    _after_lock = 'TURN_TORPEDO' if p['torpedo_heading'] is not None else 'SET_DEPTH'
    sm.add_state('LOCK_HEADING',
                 LockHeadingState(duburi, profile, heading=p['gate_heading']),
                 transitions={SUCCEED: _after_lock, TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    if p['torpedo_heading'] is not None:
        sm.add_state('TURN_TORPEDO',
                     TurnState(duburi, profile, heading_deg=p['torpedo_heading']),
                     transitions={SUCCEED: 'SET_DEPTH', ABORT: 'SURFACE'})

    sm.add_state('SET_DEPTH',
                 SetDepthState(duburi, profile, depth_m=p['torpedo_depth_m']),
                 transitions={SUCCEED: 'SWITCH_TORPEDO', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('SWITCH_TORPEDO',
                 SetDetectorState(duburi, profile,
                                  camera='forward',
                                  model=p['model'],
                                  classes=p['board_classes']),
                 transitions={SUCCEED: 'FIND_BOARD', ABORT: 'SURFACE'})

    sm.add_state('FIND_BOARD',
                 VisionSearchState(duburi, profile,
                                   target='torpedo',
                                   camera='forward',
                                   pattern='forward', gain=30,
                                   timeout=p['find_timeout']),
                 transitions={SUCCEED: 'HOME_BOARD', TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('HOME_BOARD',
                 VisionAlignState(duburi, profile,
                                  target='torpedo', camera='forward',
                                  yaw=0, lat=0, depth=0,
                                  err=p['coarse_err_px'], gain=p['coarse_gain'],
                                  duration=p['coarse_dur']),
                 transitions={SUCCEED: 'SWITCH_FINE',
                              FAILED: 'SWITCH_FINE',
                              TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('SWITCH_FINE',
                 SetDetectorState(duburi, profile, classes=p['fine_classes']),
                 transitions={SUCCEED: 'HOME_BLOOD', ABORT: 'SURFACE'})

    sm.add_state('HOME_BLOOD',
                 VisionAlignState(duburi, profile,
                                  target='blood', camera='forward',
                                  yaw=0, lat=0, depth=0,
                                  err=p['fine_err_px'], gain=p['fine_gain'],
                                  duration=p['fine_dur']),
                 transitions={SUCCEED: 'LOCK_HOLE',
                              FAILED: 'LOCK_HOLE',
                              TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    # Tight lock on the hole itself; only fire if alignment succeeds.
    sm.add_state('LOCK_HOLE',
                 VisionAlignState(duburi, profile,
                                  target='hole', camera='forward',
                                  yaw=0, lat=0, depth=0,
                                  err=p['fine_err_px'], gain=p['fine_gain'],
                                  duration=p['lock_dur']),
                 transitions={SUCCEED: 'FIRE',
                              FAILED: 'LOG_SCORE',
                              TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('FIRE',
                 FireState(duburi, profile,
                           channel=p['fire_channel'], confirm_pause_s=2.0),
                 transitions={SUCCEED: 'PAUSE', FAILED: 'PAUSE',
                              ABORT: 'SURFACE'})

    sm.add_state('PAUSE',
                 PauseState(duburi, profile, seconds=2.0),
                 transitions={SUCCEED: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('LOG_SCORE',
                 LogScoreState(duburi, profile),
                 transitions={SUCCEED: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('SURFACE',
                 SurfaceState(duburi, profile),
                 transitions={SUCCEED: SUCCEED, ABORT: ABORT})

    sm.set_start_state('COUNTDOWN')
    return sm

"""Torpedo task FSM — align to torpedo board, lock on hole, fire.

Standalone:
    ros2 run duburi_planner mission fsm_torpedo

Task flow:
    COUNTDOWN → ARM → LOCK_HEADING → [TURN_TORPEDO] → SET_DEPTH
    → SWITCH_TORPEDO → FIND_BOARD
    → HOME_BOARD (coarse: full board)
    → SWITCH_FINE → HOME_BLOOD (fine: blood marker → easier to centre)
    → LOCK_FIRE_TORPEDO → PAUSE
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
from ..states.vision import (
    VisionFindState, VisionHomeState, VisionLockFireState,
)
from ..states.utility import (
    CountdownState, PauseState, LogScoreState, SetDetectorState,
)

TORPEDO_FIRE_DEFAULTS: dict = {
    'countdown_s':       10,
    'gate_heading':      0.0,
    'torpedo_heading':   None,          # compass bearing to board; fill at pool
    'torpedo_depth_m':   None,          # depth to align with hole; MUST fill at pool
    'fire_channel':      1,             # torpedo_1 — always explicit
    'stable_lock_s':     3.0,
    'max_attempts':      3,
    'find_timeout':      90.0,
    'home_coarse_dur':   15.0,
    'home_fine_dur':     12.0,
    'lock_fire_dur':     60.0,
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
                 VisionFindState(duburi, profile,
                                 target='torpedo',
                                 camera='forward',
                                 move='forward', gain=30,
                                 timeout=p['find_timeout']),
                 transitions={SUCCEED: 'HOME_BOARD', TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('HOME_BOARD',
                 VisionHomeState(duburi, profile,
                                 target='torpedo', camera='forward',
                                 yaw=True, lat=True, depth=True,
                                 duration=p['home_coarse_dur'],
                                 on_lost='hold'),
                 transitions={SUCCEED: 'SWITCH_FINE',
                               FAILED: 'SWITCH_FINE',
                               TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('SWITCH_FINE',
                 SetDetectorState(duburi, profile, classes=p['fine_classes']),
                 transitions={SUCCEED: 'HOME_BLOOD', ABORT: 'SURFACE'})

    sm.add_state('HOME_BLOOD',
                 VisionHomeState(duburi, profile,
                                 target='blood', camera='forward',
                                 yaw=True, lat=True, depth=True,
                                 duration=p['home_fine_dur'],
                                 on_lost='hold'),
                 transitions={SUCCEED: 'LOCK_FIRE',
                               FAILED: 'LOCK_FIRE',
                               TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('LOCK_FIRE',
                 VisionLockFireState(duburi, profile,
                                     target='hole', camera='forward',
                                     fire_channel=p['fire_channel'],
                                     yaw=True, lat=True, depth=True, forward=False,
                                     stable_lock_s=p['stable_lock_s'],
                                     max_attempts=p['max_attempts'],
                                     duration=p['lock_fire_dur'],
                                     kp_yaw=80.0, kp_lat=80.0, kp_depth=0.08,
                                     deadband=0.04),
                 transitions={SUCCEED: 'PAUSE', FAILED: 'LOG_SCORE',
                               TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

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

"""Bin drop task FSM — switch to downward camera, locate bin, drop marker.

Standalone:
    ros2 run duburi_planner mission fsm_bin

Task flow:
    COUNTDOWN → ARM → SET_DEPTH → LOCK_HEADING → [TURN_BIN]
    → DIVE_BIN → SWITCH_DOWNWARD
    → FIND_BIN → HOME_BIN → CONFIRM_PAUSE → FIRE_DROPPER
    → SWITCH_FORWARD → LOG_SCORE → SURFACE

Pool-day params to fill: bin_heading, depth_m.
downward_cam=True on HOME_BIN auto-sets kp_forward=-60 (engine polarity fix).
"""
from __future__ import annotations

from yasmin import StateMachine

from ..core.outcomes import SUCCEED, FAILED, TIMEOUT, ABORT
from ..core.vehicle_profile import VehicleProfile
from ..states.navigation import (
    ArmState, SetDepthState, LockHeadingState, TurnState, SurfaceState,
)
from ..states.vision import VisionFindState, VisionHomeState, VisionScanState
from ..states.utility import (
    CountdownState, PauseState, LogScoreState, SetDetectorState, FireState,
)

BIN_DROP_DEFAULTS: dict = {
    'countdown_s':       10,
    'search_depth_m':   -0.4,
    'gate_heading':      0.0,
    'bin_heading':       None,      # compass bearing to bin; fill at pool
    'bin_depth_m':      -1.0,       # downward camera must see bin clearly
    'fire_channel':      3,         # dropper_1
    'confirm_pause_s':   3.0,       # stability window before drop
    'find_timeout':      90.0,
    'home_duration':     20.0,
    'model':            'bin_fire_blood',
    'classes':          'fire,blood',
    'camera_downward':  'downward',
    'camera_forward':   'forward',
}


def build_bin_drop_fsm(
    duburi,
    profile: VehicleProfile,
    params: dict | None = None,
) -> StateMachine:
    p = {**BIN_DROP_DEFAULTS, **(params or {})}

    task_entry = 'DIVE_BIN'
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

    _after_lock = 'TURN_BIN' if p['bin_heading'] is not None else task_entry
    sm.add_state('LOCK_HEADING',
                 LockHeadingState(duburi, profile, heading=p['gate_heading']),
                 transitions={SUCCEED: _after_lock, TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    if p['bin_heading'] is not None:
        sm.add_state('TURN_BIN',
                     TurnState(duburi, profile, heading_deg=p['bin_heading']),
                     transitions={SUCCEED: task_entry, ABORT: 'SURFACE'})

    sm.add_state('DIVE_BIN',
                 SetDepthState(duburi, profile, depth_m=p['bin_depth_m']),
                 transitions={SUCCEED: 'SWITCH_DOWNWARD', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('SWITCH_DOWNWARD',
                 SetDetectorState(duburi, profile,
                                  camera=p['camera_downward'],
                                  model=p['model'],
                                  classes=p['classes']),
                 transitions={SUCCEED: 'FIND_BIN', ABORT: 'SURFACE'})

    sm.add_state('FIND_BIN',
                 VisionFindState(duburi, profile,
                                 target='fire',
                                 camera=p['camera_downward'],
                                 move='forward', gain=30,
                                 timeout=p['find_timeout']),
                 transitions={SUCCEED: 'HOME_BIN', TIMEOUT: 'SWITCH_FORWARD', ABORT: 'SURFACE'})

    # downward_cam=True: auto-sets kp_forward=-60 (ey polarity inversion)
    sm.add_state('HOME_BIN',
                 VisionHomeState(duburi, profile,
                                 target='fire',
                                 camera=p['camera_downward'],
                                 lat=True, forward=True, yaw=False, depth=False,
                                 duration=p['home_duration'],
                                 on_lost='hold',
                                 kp_forward=-60.0, kp_lat=60.0, deadband=0.06),
                 transitions={SUCCEED: 'CONFIRM_PAUSE',
                               FAILED: 'SWITCH_FORWARD',
                               TIMEOUT: 'SWITCH_FORWARD', ABORT: 'SURFACE'})

    sm.add_state('CONFIRM_PAUSE',
                 PauseState(duburi, profile, seconds=p['confirm_pause_s']),
                 transitions={SUCCEED: 'FIRE_DROPPER', ABORT: 'SURFACE'})

    sm.add_state('FIRE_DROPPER',
                 FireState(duburi, profile,
                           channel=p['fire_channel'], confirm_pause_s=2.0),
                 transitions={SUCCEED: 'SWITCH_FORWARD', ABORT: 'SURFACE'})

    sm.add_state('SWITCH_FORWARD',
                 SetDetectorState(duburi, profile, camera=p['camera_forward']),
                 transitions={SUCCEED: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('LOG_SCORE',
                 LogScoreState(duburi, profile),
                 transitions={SUCCEED: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('SURFACE',
                 SurfaceState(duburi, profile),
                 transitions={SUCCEED: SUCCEED, ABORT: ABORT})

    sm.set_start_state('COUNTDOWN')
    return sm

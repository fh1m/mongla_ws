"""Full RoboSub 2026 competition FSM — Gate → Slalom → Bin → Torpedo → Return+Roll.

Standalone:
    ros2 run duburi_planner mission fsm_full_2026

Flat state machine (all tasks inlined, matching gate_then_bin.py pattern).
Each task section's failures skip to the NEXT task's entry state so the AUV
continues the run even when individual tasks fail or time out.

Pool-day params to fill (all in the FULL_COMP_DEFAULTS dict below):
  gate_heading, slalom_heading, bin_heading, torpedo_heading, return_heading,
  torpedo_depth_m.
"""
from __future__ import annotations

from yasmin import StateMachine

from ..core.outcomes import SUCCEED, FAILED, TIMEOUT, ABORT
from ..core.vehicle_profile import VehicleProfile
from ..states.navigation import (
    ArmState, SetDepthState, LockHeadingState, TurnState, SurfaceState,
)
from ..states.vision import (
    VisionFindState, VisionHomeState, ApproachState, VisionLockFireState,
)
from ..states.utility import (
    CountdownState, PauseState, LogScoreState,
    SetDetectorState, FireState, StyleRollState,
)

FULL_COMP_DEFAULTS: dict = {
    'countdown_s':          10,
    'search_depth_m':      -0.4,
    'gate_heading':         0.0,

    # ── headings (None = don't turn; fill after compass survey) ─────────────
    'slalom_heading':       None,
    'bin_heading':          None,
    'torpedo_heading':      None,
    'return_heading':       None,

    # ── depths ───────────────────────────────────────────────────────────────
    'gate_pass_depth_m':   -0.6,
    'bin_depth_m':         -1.0,
    'torpedo_depth_m':      None,   # MUST fill at pool

    # ── gate task ─────────────────────────────────────────────────────────────
    'gate_pass_bbox':       0.80,
    'gate_find_timeout':    60.0,
    'gate_home_dur':        15.0,
    'gate_approach_dur':    25.0,
    'gate_model':          'gate_rescue_repair',

    # ── slalom task ───────────────────────────────────────────────────────────
    'slalom_pipe_offset':   80,
    'slalom_find_timeout':  60.0,
    'slalom_home_dur':      20.0,
    'slalom_model':        'slalom_red_pipe',

    # ── bin task ──────────────────────────────────────────────────────────────
    'bin_find_timeout':     90.0,
    'bin_home_dur':         20.0,
    'bin_fire_channel':     3,      # dropper_1
    'bin_model':           'bin_fire_blood',

    # ── torpedo task ──────────────────────────────────────────────────────────
    'torpedo_find_timeout': 90.0,
    'torpedo_home_coarse':  15.0,
    'torpedo_home_fine':    12.0,
    'torpedo_lock_dur':     60.0,
    'torpedo_fire_channel': 1,      # torpedo_1 — always explicit
    'torpedo_stable_lock':  3.0,
    'torpedo_max_attempts': 3,
    'torpedo_model':       'torpedo_blood_hole',

    # ── return task ───────────────────────────────────────────────────────────
    'return_find_timeout':  60.0,
    'return_home_dur':      15.0,
    'return_approach_dur':  25.0,
    'style_roll_flips':     1,
    'style_roll_headroom':  0.4,
    'style_roll_gain':      60,
    'return_model':        'gate_rescue_repair',
}


def build_full_competition_fsm(
    duburi,
    profile: VehicleProfile,
    params: dict | None = None,
) -> StateMachine:
    """Build full 5-task competition FSM.

    Each task can fail/timeout; the FSM skips to the next task and continues.
    No torpedo_depth_m assertion here — the torpedo section simply skips if
    the depth is None (transitions to SWITCH_RETURN).
    """
    p = {**FULL_COMP_DEFAULTS, **(params or {})}
    sm = StateMachine(outcomes=[SUCCEED, ABORT])

    # ════════════════════════════════════════════════════════
    # SETUP
    # ════════════════════════════════════════════════════════

    sm.add_state('COUNTDOWN',
                 CountdownState(duburi, profile, seconds=p['countdown_s']),
                 transitions={SUCCEED: 'ARM', ABORT: ABORT})

    sm.add_state('ARM',
                 ArmState(duburi, profile),
                 transitions={SUCCEED: 'DIVE', ABORT: 'SURFACE'})

    sm.add_state('DIVE',
                 SetDepthState(duburi, profile, depth_m=p['search_depth_m']),
                 transitions={SUCCEED: 'LOCK_HDG', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('LOCK_HDG',
                 LockHeadingState(duburi, profile, heading=p['gate_heading']),
                 transitions={SUCCEED: 'SWITCH_GATE', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    # ════════════════════════════════════════════════════════
    # GATE TASK
    # ════════════════════════════════════════════════════════

    sm.add_state('SWITCH_GATE',
                 SetDetectorState(duburi, profile,
                                  camera='forward',
                                  model=p['gate_model'],
                                  classes='gate,rescue,repair'),
                 transitions={SUCCEED: 'FIND_GATE', ABORT: 'SURFACE'})

    sm.add_state('FIND_GATE',
                 VisionFindState(duburi, profile,
                                 target='gate', camera='forward',
                                 move='forward', gain=30,
                                 timeout=p['gate_find_timeout']),
                 transitions={SUCCEED: 'HOME_GATE',
                               TIMEOUT: _slalom_entry(p),
                               ABORT: 'SURFACE'})

    sm.add_state('HOME_GATE',
                 VisionHomeState(duburi, profile,
                                 target='gate', camera='forward',
                                 yaw=True, lat=True,
                                 duration=p['gate_home_dur'], on_lost='hold'),
                 transitions={SUCCEED: 'HOME_MARKER',
                               FAILED: _slalom_entry(p),
                               TIMEOUT: _slalom_entry(p), ABORT: 'SURFACE'})

    sm.add_state('HOME_MARKER',
                 VisionHomeState(duburi, profile,
                                 target='rescue', camera='forward',
                                 yaw=False, lat=True,
                                 duration=10.0, on_lost='hold'),
                 transitions={SUCCEED: 'SET_PASS_DEPTH',
                               FAILED: 'SET_PASS_DEPTH',
                               TIMEOUT: 'SET_PASS_DEPTH', ABORT: 'SURFACE'})

    sm.add_state('SET_PASS_DEPTH',
                 SetDepthState(duburi, profile, depth_m=p['gate_pass_depth_m']),
                 transitions={SUCCEED: 'SWITCH_GATE_CLASSES',
                               TIMEOUT: _slalom_entry(p), ABORT: 'SURFACE'})

    sm.add_state('SWITCH_GATE_CLASSES',
                 SetDetectorState(duburi, profile, classes='gate'),
                 transitions={SUCCEED: 'APPROACH_GATE', ABORT: 'SURFACE'})

    sm.add_state('APPROACH_GATE',
                 ApproachState(duburi, profile,
                               target='gate', camera='forward',
                               dist=p['gate_pass_bbox'], metric='height',
                               duration=p['gate_approach_dur'],
                               lock_mode='pursue', on_lost='hold'),
                 transitions={SUCCEED: 'RESTORE_DEPTH',
                               FAILED: _slalom_entry(p),
                               TIMEOUT: _slalom_entry(p), ABORT: 'SURFACE'})

    sm.add_state('RESTORE_DEPTH',
                 SetDepthState(duburi, profile, depth_m=p['search_depth_m']),
                 transitions={SUCCEED: _slalom_entry(p),
                               TIMEOUT: _slalom_entry(p), ABORT: 'SURFACE'})

    # ════════════════════════════════════════════════════════
    # SLALOM TASK
    # ════════════════════════════════════════════════════════

    if p['slalom_heading'] is not None:
        sm.add_state('TURN_SLALOM',
                     TurnState(duburi, profile, heading_deg=p['slalom_heading']),
                     transitions={SUCCEED: 'SWITCH_SLALOM', ABORT: 'SURFACE'})

    sm.add_state('SWITCH_SLALOM',
                 SetDetectorState(duburi, profile,
                                  camera='forward',
                                  model=p['slalom_model'],
                                  classes='red_pipe'),
                 transitions={SUCCEED: 'FIND_PIPE', ABORT: 'SURFACE'})

    sm.add_state('FIND_PIPE',
                 VisionFindState(duburi, profile,
                                 target='red_pipe', camera='forward',
                                 move='forward', gain=30,
                                 timeout=p['slalom_find_timeout']),
                 transitions={SUCCEED: 'SLALOM_L',
                               TIMEOUT: _bin_entry(p),
                               ABORT: 'SURFACE'})

    sm.add_state('SLALOM_L',
                 VisionHomeState(duburi, profile,
                                 target='red_pipe', camera='forward',
                                 yaw=True, lat=True,
                                 offset_x=+p['slalom_pipe_offset'],
                                 duration=p['slalom_home_dur'], on_lost='hold'),
                 transitions={SUCCEED: 'PAUSE_SL',
                               FAILED: _bin_entry(p),
                               TIMEOUT: _bin_entry(p), ABORT: 'SURFACE'})

    sm.add_state('PAUSE_SL',
                 PauseState(duburi, profile, seconds=0.5),
                 transitions={SUCCEED: 'SLALOM_R', ABORT: 'SURFACE'})

    sm.add_state('SLALOM_R',
                 VisionHomeState(duburi, profile,
                                 target='red_pipe', camera='forward',
                                 yaw=True, lat=True,
                                 offset_x=-p['slalom_pipe_offset'],
                                 duration=p['slalom_home_dur'], on_lost='hold'),
                 transitions={SUCCEED: 'PAUSE_SR',
                               FAILED: _bin_entry(p),
                               TIMEOUT: _bin_entry(p), ABORT: 'SURFACE'})

    sm.add_state('PAUSE_SR',
                 PauseState(duburi, profile, seconds=0.5),
                 transitions={SUCCEED: _bin_entry(p), ABORT: 'SURFACE'})

    # ════════════════════════════════════════════════════════
    # BIN TASK
    # ════════════════════════════════════════════════════════

    if p['bin_heading'] is not None:
        sm.add_state('TURN_BIN',
                     TurnState(duburi, profile, heading_deg=p['bin_heading']),
                     transitions={SUCCEED: 'DIVE_BIN', ABORT: 'SURFACE'})

    sm.add_state('DIVE_BIN',
                 SetDepthState(duburi, profile, depth_m=p['bin_depth_m']),
                 transitions={SUCCEED: 'SWITCH_DOWNWARD',
                               TIMEOUT: _torpedo_entry(p),
                               ABORT: 'SURFACE'})

    sm.add_state('SWITCH_DOWNWARD',
                 SetDetectorState(duburi, profile,
                                  camera='downward',
                                  model=p['bin_model'],
                                  classes='fire,blood'),
                 transitions={SUCCEED: 'FIND_BIN', ABORT: 'SURFACE'})

    sm.add_state('FIND_BIN',
                 VisionFindState(duburi, profile,
                                 target='fire', camera='downward',
                                 move='forward', gain=30,
                                 timeout=p['bin_find_timeout']),
                 transitions={SUCCEED: 'HOME_BIN',
                               TIMEOUT: 'SWITCH_FWD_B',
                               ABORT: 'SURFACE'})

    sm.add_state('HOME_BIN',
                 VisionHomeState(duburi, profile,
                                 target='fire', camera='downward',
                                 lat=True, forward=True, yaw=False, depth=False,
                                 duration=p['bin_home_dur'], on_lost='hold',
                                 kp_forward=-60.0, kp_lat=60.0, deadband=0.06),
                 transitions={SUCCEED: 'CONFIRM_BIN',
                               FAILED: 'SWITCH_FWD_B',
                               TIMEOUT: 'SWITCH_FWD_B', ABORT: 'SURFACE'})

    sm.add_state('CONFIRM_BIN',
                 PauseState(duburi, profile, seconds=3.0),
                 transitions={SUCCEED: 'FIRE_BIN', ABORT: 'SURFACE'})

    sm.add_state('FIRE_BIN',
                 FireState(duburi, profile,
                           channel=p['bin_fire_channel'], confirm_pause_s=2.0),
                 transitions={SUCCEED: 'SWITCH_FWD_B', ABORT: 'SURFACE'})

    sm.add_state('SWITCH_FWD_B',
                 SetDetectorState(duburi, profile, camera='forward'),
                 transitions={SUCCEED: _torpedo_entry(p), ABORT: 'SURFACE'})

    # ════════════════════════════════════════════════════════
    # TORPEDO TASK (skipped if torpedo_depth_m is None)
    # ════════════════════════════════════════════════════════

    if p['torpedo_heading'] is not None:
        sm.add_state('TURN_TORPEDO',
                     TurnState(duburi, profile, heading_deg=p['torpedo_heading']),
                     transitions={SUCCEED: _torpedo_dive_entry(p), ABORT: 'SURFACE'})

    if p['torpedo_depth_m'] is not None:
        sm.add_state('DIVE_TORPEDO',
                     SetDepthState(duburi, profile, depth_m=p['torpedo_depth_m']),
                     transitions={SUCCEED: 'SWITCH_TORPEDO',
                                   TIMEOUT: _return_entry(p), ABORT: 'SURFACE'})

        sm.add_state('SWITCH_TORPEDO',
                     SetDetectorState(duburi, profile,
                                      camera='forward',
                                      model=p['torpedo_model'],
                                      classes='torpedo'),
                     transitions={SUCCEED: 'FIND_BOARD', ABORT: 'SURFACE'})

        sm.add_state('FIND_BOARD',
                     VisionFindState(duburi, profile,
                                     target='torpedo', camera='forward',
                                     move='forward', gain=30,
                                     timeout=p['torpedo_find_timeout']),
                     transitions={SUCCEED: 'HOME_BOARD',
                                   TIMEOUT: _return_entry(p), ABORT: 'SURFACE'})

        sm.add_state('HOME_BOARD',
                     VisionHomeState(duburi, profile,
                                     target='torpedo', camera='forward',
                                     yaw=True, lat=True, depth=True,
                                     duration=p['torpedo_home_coarse'], on_lost='hold'),
                     transitions={SUCCEED: 'SWITCH_FINE_T',
                                   FAILED: 'SWITCH_FINE_T',
                                   TIMEOUT: _return_entry(p), ABORT: 'SURFACE'})

        sm.add_state('SWITCH_FINE_T',
                     SetDetectorState(duburi, profile, classes='blood,hole'),
                     transitions={SUCCEED: 'HOME_BLOOD', ABORT: 'SURFACE'})

        sm.add_state('HOME_BLOOD',
                     VisionHomeState(duburi, profile,
                                     target='blood', camera='forward',
                                     yaw=True, lat=True, depth=True,
                                     duration=p['torpedo_home_fine'], on_lost='hold'),
                     transitions={SUCCEED: 'LOCK_FIRE_T',
                                   FAILED: 'LOCK_FIRE_T',
                                   TIMEOUT: _return_entry(p), ABORT: 'SURFACE'})

        sm.add_state('LOCK_FIRE_T',
                     VisionLockFireState(duburi, profile,
                                         target='hole', camera='forward',
                                         fire_channel=p['torpedo_fire_channel'],
                                         yaw=True, lat=True, depth=True, forward=False,
                                         stable_lock_s=p['torpedo_stable_lock'],
                                         max_attempts=p['torpedo_max_attempts'],
                                         duration=p['torpedo_lock_dur'],
                                         kp_yaw=80.0, kp_lat=80.0, kp_depth=0.08,
                                         deadband=0.04),
                     transitions={SUCCEED: 'PAUSE_T',
                                   FAILED: _return_entry(p),
                                   TIMEOUT: _return_entry(p), ABORT: 'SURFACE'})

        sm.add_state('PAUSE_T',
                     PauseState(duburi, profile, seconds=2.0),
                     transitions={SUCCEED: _return_entry(p), ABORT: 'SURFACE'})

    # ════════════════════════════════════════════════════════
    # RETURN TASK
    # ════════════════════════════════════════════════════════

    if p['return_heading'] is not None:
        sm.add_state('TURN_RETURN',
                     TurnState(duburi, profile, heading_deg=p['return_heading']),
                     transitions={SUCCEED: 'SWITCH_RETURN', ABORT: 'SURFACE'})

    sm.add_state('SWITCH_RETURN',
                 SetDetectorState(duburi, profile,
                                  camera='forward',
                                  model=p['return_model'],
                                  classes='gate'),
                 transitions={SUCCEED: 'FIND_RETURN_GATE', ABORT: 'SURFACE'})

    sm.add_state('FIND_RETURN_GATE',
                 VisionFindState(duburi, profile,
                                 target='gate', camera='forward',
                                 move='forward', gain=30,
                                 timeout=p['return_find_timeout']),
                 transitions={SUCCEED: 'HOME_RETURN_GATE',
                               TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('HOME_RETURN_GATE',
                 VisionHomeState(duburi, profile,
                                 target='gate', camera='forward',
                                 yaw=True, lat=True,
                                 duration=p['return_home_dur'], on_lost='hold'),
                 transitions={SUCCEED: 'SET_PASS_DEPTH_R',
                               FAILED: 'LOG_SCORE',
                               TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('SET_PASS_DEPTH_R',
                 SetDepthState(duburi, profile, depth_m=p['gate_pass_depth_m']),
                 transitions={SUCCEED: 'APPROACH_RETURN', TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('APPROACH_RETURN',
                 ApproachState(duburi, profile,
                               target='gate', camera='forward',
                               dist=p['gate_pass_bbox'], metric='height',
                               duration=p['return_approach_dur'],
                               lock_mode='pursue', on_lost='hold'),
                 transitions={SUCCEED: 'SURFACE_LEVEL',
                               FAILED: 'LOG_SCORE',
                               TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('SURFACE_LEVEL',
                 SetDepthState(duburi, profile, depth_m=p['search_depth_m']),
                 transitions={SUCCEED: 'STYLE_ROLL', TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('STYLE_ROLL',
                 StyleRollState(duburi, profile,
                                flips=p['style_roll_flips'],
                                headroom=p['style_roll_headroom'],
                                gain=p['style_roll_gain']),
                 transitions={SUCCEED: 'LOG_SCORE', ABORT: 'SURFACE'})

    # ════════════════════════════════════════════════════════
    # CLEANUP
    # ════════════════════════════════════════════════════════

    sm.add_state('LOG_SCORE',
                 LogScoreState(duburi, profile),
                 transitions={SUCCEED: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('SURFACE',
                 SurfaceState(duburi, profile),
                 transitions={SUCCEED: SUCCEED, ABORT: ABORT})

    sm.set_start_state('COUNTDOWN')
    return sm


# ── task entry-point helpers ─────────────────────────────────────────────────
# Return the name of the first state for each task, accounting for optional
# heading-turn states. Used for "skip to next task" transitions.

def _slalom_entry(p: dict) -> str:
    return 'TURN_SLALOM' if p['slalom_heading'] is not None else 'SWITCH_SLALOM'

def _bin_entry(p: dict) -> str:
    return 'TURN_BIN' if p['bin_heading'] is not None else 'DIVE_BIN'

def _torpedo_entry(p: dict) -> str:
    if p['torpedo_depth_m'] is None:
        return _return_entry(p)
    return 'TURN_TORPEDO' if p['torpedo_heading'] is not None else 'DIVE_TORPEDO'

def _torpedo_dive_entry(p: dict) -> str:
    return 'DIVE_TORPEDO' if p['torpedo_depth_m'] is not None else _return_entry(p)

def _return_entry(p: dict) -> str:
    return 'TURN_RETURN' if p['return_heading'] is not None else 'SWITCH_RETURN'

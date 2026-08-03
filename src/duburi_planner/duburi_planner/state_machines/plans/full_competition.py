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
    VisionSearchState, VisionAlignState, VisionMoveState,
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

    # ── align/approach tuning (shared) ─────────────────────────────────────────
    'align_err_px':         40,
    'align_gain':           30,
    'approach_gain':        45,

    # ── gate task ─────────────────────────────────────────────────────────────
    'gate_pass_fill':       80,     # % frame the gate fills when passing
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
    'bin_err_px':           30,
    'bin_fire_channel':     3,      # dropper_1
    'bin_model':           'bin_fire_blood',

    # ── torpedo task ──────────────────────────────────────────────────────────
    'torpedo_find_timeout': 90.0,
    'torpedo_coarse_dur':   15.0,
    'torpedo_coarse_err':   50,
    'torpedo_fine_dur':     12.0,
    'torpedo_lock_dur':     60.0,
    'torpedo_fine_err':     14,
    'torpedo_fine_gain':    25,
    'torpedo_fire_channel': 1,      # torpedo_1 — always explicit
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
                 VisionSearchState(duburi, profile,
                                   target='gate', camera='forward',
                                   pattern='forward', gain=30,
                                   timeout=p['gate_find_timeout']),
                 transitions={SUCCEED: 'HOME_GATE',
                              TIMEOUT: _slalom_entry(p),
                              ABORT: 'SURFACE'})

    sm.add_state('HOME_GATE',
                 VisionAlignState(duburi, profile,
                                  target='gate', camera='forward',
                                  yaw=0, lat=0,
                                  err=p['align_err_px'], gain=p['align_gain'],
                                  duration=p['gate_home_dur']),
                 transitions={SUCCEED: 'HOME_MARKER',
                              FAILED: _slalom_entry(p),
                              TIMEOUT: _slalom_entry(p), ABORT: 'SURFACE'})

    sm.add_state('HOME_MARKER',
                 VisionAlignState(duburi, profile,
                                  target='rescue', camera='forward',
                                  lat=0,
                                  err=p['align_err_px'], gain=p['align_gain'],
                                  duration=10.0),
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
                 VisionMoveState(duburi, profile,
                                 target='gate', camera='forward',
                                 fwd=p['gate_pass_fill'], mode='height',
                                 gain=p['approach_gain'],
                                 duration=p['gate_approach_dur']),
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
                 VisionSearchState(duburi, profile,
                                   target='red_pipe', camera='forward',
                                   pattern='forward', gain=30,
                                   timeout=p['slalom_find_timeout']),
                 transitions={SUCCEED: 'SLALOM_L',
                              TIMEOUT: _bin_entry(p),
                              ABORT: 'SURFACE'})

    sm.add_state('SLALOM_L',
                 VisionAlignState(duburi, profile,
                                  target='red_pipe', camera='forward',
                                  yaw=0, lat=+p['slalom_pipe_offset'],
                                  err=p['align_err_px'], gain=p['align_gain'],
                                  duration=p['slalom_home_dur']),
                 transitions={SUCCEED: 'PAUSE_SL',
                              FAILED: _bin_entry(p),
                              TIMEOUT: _bin_entry(p), ABORT: 'SURFACE'})

    sm.add_state('PAUSE_SL',
                 PauseState(duburi, profile, seconds=0.5),
                 transitions={SUCCEED: 'SLALOM_R', ABORT: 'SURFACE'})

    sm.add_state('SLALOM_R',
                 VisionAlignState(duburi, profile,
                                  target='red_pipe', camera='forward',
                                  yaw=0, lat=-p['slalom_pipe_offset'],
                                  err=p['align_err_px'], gain=p['align_gain'],
                                  duration=p['slalom_home_dur']),
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
                 VisionSearchState(duburi, profile,
                                   target='fire', camera='downward',
                                   pattern='forward', gain=30,
                                   timeout=p['bin_find_timeout']),
                 transitions={SUCCEED: 'HOME_BIN',
                              TIMEOUT: 'SWITCH_FWD_B',
                              ABORT: 'SURFACE'})

    sm.add_state('HOME_BIN',
                 VisionAlignState(duburi, profile,
                                  target='fire', camera='downward',
                                  lat=0, depth=0,
                                  err=p['bin_err_px'], gain=p['align_gain'],
                                  duration=p['bin_home_dur']),
                 transitions={SUCCEED: 'CONFIRM_BIN',
                              FAILED: 'SWITCH_FWD_B',
                              TIMEOUT: 'SWITCH_FWD_B', ABORT: 'SURFACE'})

    sm.add_state('CONFIRM_BIN',
                 PauseState(duburi, profile, seconds=3.0),
                 transitions={SUCCEED: 'FIRE_BIN', ABORT: 'SURFACE'})

    sm.add_state('FIRE_BIN',
                 FireState(duburi, profile,
                           channel=p['bin_fire_channel'], confirm_pause_s=2.0),
                 transitions={SUCCEED: 'SWITCH_FWD_B', FAILED: 'SWITCH_FWD_B',
                              ABORT: 'SURFACE'})

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
                     VisionSearchState(duburi, profile,
                                       target='torpedo', camera='forward',
                                       pattern='forward', gain=30,
                                       timeout=p['torpedo_find_timeout']),
                     transitions={SUCCEED: 'HOME_BOARD',
                                  TIMEOUT: _return_entry(p), ABORT: 'SURFACE'})

        sm.add_state('HOME_BOARD',
                     VisionAlignState(duburi, profile,
                                      target='torpedo', camera='forward',
                                      yaw=0, lat=0, depth=0,
                                      err=p['torpedo_coarse_err'], gain=p['align_gain'],
                                      duration=p['torpedo_coarse_dur']),
                     transitions={SUCCEED: 'SWITCH_FINE_T',
                                  FAILED: 'SWITCH_FINE_T',
                                  TIMEOUT: _return_entry(p), ABORT: 'SURFACE'})

        sm.add_state('SWITCH_FINE_T',
                     SetDetectorState(duburi, profile, classes='blood,hole'),
                     transitions={SUCCEED: 'HOME_BLOOD', ABORT: 'SURFACE'})

        sm.add_state('HOME_BLOOD',
                     VisionAlignState(duburi, profile,
                                      target='blood', camera='forward',
                                      yaw=0, lat=0, depth=0,
                                      err=p['torpedo_fine_err'], gain=p['torpedo_fine_gain'],
                                      duration=p['torpedo_fine_dur']),
                     transitions={SUCCEED: 'LOCK_HOLE_T',
                                  FAILED: 'LOCK_HOLE_T',
                                  TIMEOUT: _return_entry(p), ABORT: 'SURFACE'})

        sm.add_state('LOCK_HOLE_T',
                     VisionAlignState(duburi, profile,
                                      target='hole', camera='forward',
                                      yaw=0, lat=0, depth=0,
                                      err=p['torpedo_fine_err'], gain=p['torpedo_fine_gain'],
                                      duration=p['torpedo_lock_dur']),
                     transitions={SUCCEED: 'FIRE_T',
                                  FAILED: _return_entry(p),
                                  TIMEOUT: _return_entry(p), ABORT: 'SURFACE'})

        sm.add_state('FIRE_T',
                     FireState(duburi, profile,
                               channel=p['torpedo_fire_channel'], confirm_pause_s=2.0),
                     transitions={SUCCEED: 'PAUSE_T', FAILED: 'PAUSE_T',
                                  ABORT: 'SURFACE'})

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
                 VisionSearchState(duburi, profile,
                                   target='gate', camera='forward',
                                   pattern='forward', gain=30,
                                   timeout=p['return_find_timeout']),
                 transitions={SUCCEED: 'HOME_RETURN_GATE',
                              TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('HOME_RETURN_GATE',
                 VisionAlignState(duburi, profile,
                                  target='gate', camera='forward',
                                  yaw=0, lat=0,
                                  err=p['align_err_px'], gain=p['align_gain'],
                                  duration=p['return_home_dur']),
                 transitions={SUCCEED: 'SET_PASS_DEPTH_R',
                              FAILED: 'LOG_SCORE',
                              TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('SET_PASS_DEPTH_R',
                 SetDepthState(duburi, profile, depth_m=p['gate_pass_depth_m']),
                 transitions={SUCCEED: 'APPROACH_RETURN', TIMEOUT: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('APPROACH_RETURN',
                 VisionMoveState(duburi, profile,
                                 target='gate', camera='forward',
                                 fwd=p['gate_pass_fill'], mode='height',
                                 gain=p['approach_gain'],
                                 duration=p['return_approach_dur']),
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

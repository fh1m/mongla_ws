"""Full RoboSub 2026 competition run — YASMIN FSM launcher.

    ros2 run duburi_planner mission fsm_full_2026

Chains Gate → Slalom → Bin → Torpedo → Return+Roll in a single flat FSM.
Each task failure skips to the next task — the AUV continues the run.

Pool-day: fill all headings and TORPEDO_DEPTH_M in competition_config.py.
Torpedo task is SKIPPED if TORPEDO_DEPTH_M is None (safe default).
"""
from yasmin import Blackboard
from yasmin_ros import set_ros_loggers

from ..state_machines import build_full_competition_fsm, VehicleProfile
from .competition_config import (
    GATE_SEARCH_DEPTH_M,
    GATE_PASS_DEPTH_M,
    GATE_PASS_BBOX_FRAC,
    BIN_DEPTH_M,
    TORPEDO_DEPTH_M,
    SLALOM_HEADING_DEG,
    BIN_HEADING_DEG,
    TORPEDO_HEADING_DEG,
    RETURN_HEADING_DEG,
    SLALOM_PIPE_OFFSET_PX,
    STYLE_ROLL_HEADROOM_M,
    STYLE_ROLL_GAIN,
)


def run(duburi, log):
    duburi.camera = 'forward'

    profile = VehicleProfile.auto(duburi.client.node)
    log(f'[FSM] full_2026  body={profile.name}  dvl={profile.has_dvl}  '
        f'torpedo_depth={TORPEDO_DEPTH_M}')
    set_ros_loggers()

    sm = build_full_competition_fsm(duburi, profile, params={
        'search_depth_m':       GATE_SEARCH_DEPTH_M,
        'gate_pass_depth_m':    GATE_PASS_DEPTH_M,
        'gate_pass_bbox':       GATE_PASS_BBOX_FRAC,
        'bin_depth_m':          BIN_DEPTH_M,
        'torpedo_depth_m':      TORPEDO_DEPTH_M,   # None = skip torpedo task
        'slalom_heading':       SLALOM_HEADING_DEG,
        'bin_heading':          BIN_HEADING_DEG,
        'torpedo_heading':      TORPEDO_HEADING_DEG,
        'return_heading':       RETURN_HEADING_DEG,
        'slalom_pipe_offset':   SLALOM_PIPE_OFFSET_PX,
        'style_roll_headroom':  STYLE_ROLL_HEADROOM_M,
        'style_roll_gain':      STYLE_ROLL_GAIN,
    })
    outcome = sm(Blackboard())
    log(f'[FSM] full_2026 complete: {outcome}')

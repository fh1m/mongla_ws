"""Return gate task — YASMIN FSM standalone launcher.

    ros2 run duburi_planner mission fsm_return

Pool-day: set RETURN_HEADING_DEG in competition_config.py.
"""
from ..state_machines import run_fsm  # guaranteed disarm-on-exit
from yasmin_ros import set_ros_loggers

from ..state_machines import build_return_gate_fsm, VehicleProfile
from .competition_config import (
    RETURN_HEADING_DEG, GATE_PASS_DEPTH_M, GATE_SEARCH_DEPTH_M,
    GATE_PASS_FWD_FILL, STYLE_ROLL_HEADROOM_M, STYLE_ROLL_GAIN,
)


def run(duburi, log):
    duburi.mission_reset()   # clear heading lock + abort from any previous run
    duburi.camera = 'forward'

    profile = VehicleProfile.auto(duburi.client.node)
    log(f'[FSM] return_gate  body={profile.name}  dvl={profile.has_dvl}')
    set_ros_loggers()

    sm = build_return_gate_fsm(duburi, profile, params={
        'return_heading':       RETURN_HEADING_DEG,
        'search_depth_m':       GATE_SEARCH_DEPTH_M,
        'pass_depth_m':         GATE_PASS_DEPTH_M,
        'pass_fwd_fill':        GATE_PASS_FWD_FILL,
        'style_roll_headroom':  STYLE_ROLL_HEADROOM_M,
        'style_roll_gain':      STYLE_ROLL_GAIN,
    })
    outcome = run_fsm(duburi, sm)  # ALWAYS release-heading + disarm on exit
    log(f'[FSM] return_gate complete: {outcome}')

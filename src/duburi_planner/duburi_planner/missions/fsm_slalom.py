"""Slalom task — YASMIN FSM standalone launcher.

    ros2 run duburi_planner mission fsm_slalom

Pool-day: edit slalom_heading in params= below (or competition_config.py).
"""
from yasmin import Blackboard
from yasmin_ros import set_ros_loggers

from ..state_machines import build_slalom_fsm, VehicleProfile
from .competition_config import SLALOM_HEADING_DEG, SLALOM_PIPE_OFFSET_PX


def run(duburi, log):
    duburi.mission_reset()   # clear heading lock + abort from any previous run
    duburi.camera = 'forward'

    profile = VehicleProfile.auto(duburi.client.node)
    log(f'[FSM] slalom  body={profile.name}  dvl={profile.has_dvl}')
    set_ros_loggers()

    sm = build_slalom_fsm(duburi, profile, params={
        'slalom_heading':  SLALOM_HEADING_DEG,
        'pipe_offset_px':  SLALOM_PIPE_OFFSET_PX,
    })
    outcome = sm(Blackboard())
    log(f'[FSM] slalom complete: {outcome}')

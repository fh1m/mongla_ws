"""Slalom task — YASMIN FSM standalone launcher.

    ros2 run mongla_planner mission fsm_slalom

Pool-day: edit slalom_heading in params= below (or competition_config.py).
"""
from ..state_machines import run_fsm  # guaranteed disarm-on-exit
from yasmin_ros import set_ros_loggers

from ..state_machines import build_slalom_fsm, VehicleProfile
from .competition_config import SLALOM_HEADING_DEG, SLALOM_PIPE_OFFSET_PX


def run(mongla, log):
    mongla.mission_reset()   # clear heading lock + abort from any previous run
    mongla.camera = 'forward'

    profile = VehicleProfile.auto(mongla.client.node)
    log(f'[FSM] slalom  body={profile.name}  dvl={profile.has_dvl}')
    set_ros_loggers()

    sm = build_slalom_fsm(mongla, profile, params={
        'slalom_heading':  SLALOM_HEADING_DEG,
        'pipe_offset_px':  SLALOM_PIPE_OFFSET_PX,
    })
    outcome = run_fsm(mongla, sm)  # ALWAYS release-heading + disarm on exit
    log(f'[FSM] slalom complete: {outcome}')

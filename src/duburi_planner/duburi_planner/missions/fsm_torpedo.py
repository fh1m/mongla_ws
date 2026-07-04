"""Torpedo task — YASMIN FSM standalone launcher.

    ros2 run duburi_planner mission fsm_torpedo

Pool-day: set TORPEDO_HEADING_DEG and TORPEDO_DEPTH_M in competition_config.py.
The FSM will assert torpedo_depth_m is not None before building.
"""
from ..state_machines import run_fsm  # guaranteed disarm-on-exit
from yasmin_ros import set_ros_loggers

from ..state_machines import build_torpedo_fire_fsm, VehicleProfile
from .competition_config import TORPEDO_HEADING_DEG, TORPEDO_DEPTH_M


def run(duburi, log):
    duburi.mission_reset()   # clear heading lock + abort from any previous run
    duburi.camera = 'forward'

    profile = VehicleProfile.auto(duburi.client.node)
    log(f'[FSM] torpedo  body={profile.name}  dvl={profile.has_dvl}')
    set_ros_loggers()

    sm = build_torpedo_fire_fsm(duburi, profile, params={
        'torpedo_heading': TORPEDO_HEADING_DEG,
        'torpedo_depth_m': TORPEDO_DEPTH_M,
    })
    outcome = run_fsm(duburi, sm)  # ALWAYS release-heading + disarm on exit
    log(f'[FSM] torpedo complete: {outcome}')

"""Bin drop task — YASMIN FSM standalone launcher.

    ros2 run mongla_planner mission fsm_bin

Pool-day: edit bin_heading and bin_depth_m in params= below (or competition_config.py).
"""
from ..state_machines import run_fsm  # guaranteed disarm-on-exit
from yasmin_ros import set_ros_loggers

from ..state_machines import build_bin_drop_fsm, VehicleProfile
from .competition_config import BIN_HEADING_DEG, BIN_DEPTH_M


def run(mongla, log):
    mongla.mission_reset()   # clear heading lock + abort from any previous run
    mongla.camera = 'forward'

    profile = VehicleProfile.auto(mongla.client.node)
    log(f'[FSM] bin_drop  body={profile.name}  dvl={profile.has_dvl}')
    set_ros_loggers()

    sm = build_bin_drop_fsm(mongla, profile, params={
        'bin_heading': BIN_HEADING_DEG,
        'bin_depth_m': BIN_DEPTH_M,
    })
    outcome = run_fsm(mongla, sm)  # ALWAYS release-heading + disarm on exit
    log(f'[FSM] bin_drop complete: {outcome}')

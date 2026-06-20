"""Bin drop task — YASMIN FSM standalone launcher.

    ros2 run duburi_planner mission fsm_bin

Pool-day: edit bin_heading and bin_depth_m in params= below (or competition_config.py).
"""
from yasmin import Blackboard
from yasmin_ros import set_ros_loggers

from ..state_machines import build_bin_drop_fsm, VehicleProfile
from .competition_config import BIN_HEADING_DEG, BIN_DEPTH_M


def run(duburi, log):
    duburi.camera = 'forward'

    profile = VehicleProfile.auto(duburi.client.node)
    log(f'[FSM] bin_drop  body={profile.name}  dvl={profile.has_dvl}')
    set_ros_loggers()

    sm = build_bin_drop_fsm(duburi, profile, params={
        'bin_heading': BIN_HEADING_DEG,
        'bin_depth_m': BIN_DEPTH_M,
    })
    outcome = sm(Blackboard())
    log(f'[FSM] bin_drop complete: {outcome}')

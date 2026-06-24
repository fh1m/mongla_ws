"""Pre-qualification — YASMIN FSM mission (gate pass only).

    ros2 run duburi_planner mission prequal_fsm

Works on both Duburi 4.5 (DVL passes) and Dubomini 2.0 (timed passes).
"""
from yasmin import Blackboard
from yasmin_ros import set_ros_loggers

from ..state_machines import build_prequal_fsm, VehicleProfile


def run(duburi, log):
    duburi.mission_reset()
    duburi.models(gate='gate_flare_medium_100ep')
    duburi.camera = 'forward'
    duburi.set_classes('gate')

    profile = VehicleProfile.auto(duburi.client.node)
    log(f'[FSM] body={profile.name}  dvl={profile.has_dvl}')

    set_ros_loggers()
    sm = build_prequal_fsm(duburi, profile)
    outcome = sm(Blackboard())
    log(f'[FSM] mission complete: {outcome}')

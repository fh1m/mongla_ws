"""Pre-qualification — YASMIN FSM mission (gate pass only).

    ros2 run mongla_planner mission prequal_fsm

Works on both Mongla 4.5 (DVL passes) and Mongla_agile 2.0 (timed passes).
"""
from ..state_machines import run_fsm  # guaranteed disarm-on-exit
from yasmin_ros import set_ros_loggers

from ..state_machines import build_prequal_fsm, VehicleProfile


def run(mongla, log):
    mongla.mission_reset()
    mongla.models(gate='gate_flare_medium_100ep')
    mongla.camera = 'forward'
    mongla.set_classes('gate')

    profile = VehicleProfile.auto(mongla.client.node)
    log(f'[FSM] body={profile.name}  dvl={profile.has_dvl}')

    set_ros_loggers()
    sm = build_prequal_fsm(mongla, profile)
    outcome = run_fsm(mongla, sm)  # ALWAYS release-heading + disarm on exit
    log(f'[FSM] mission complete: {outcome}')

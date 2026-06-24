"""Gate + Flare — YASMIN FSM mission.

Launched identically to any detected()-paradigm mission:

    ros2 run duburi_planner mission gate_flare_fsm

VehicleProfile is auto-detected from /duburi_manager ROS params at runtime:
  - Duburi 4.5 (yaw_source=dvl)     → DVL distance passes
  - Dubomini 2.0 (yaw_source=mavlink_ahrs) → timed-thrust passes

Override pool-day params via the params dict (no code edits needed):

    from duburi_planner.state_machines import build_gate_flare_fsm, VehicleProfile
    sm = build_gate_flare_fsm(duburi, profile,
                               params={'gate_heading': 63.0, 'depth_m': -0.9})
"""
from yasmin import Blackboard
from yasmin_ros import set_ros_loggers

from ..state_machines import build_gate_flare_fsm, VehicleProfile


def run(duburi, log):
    duburi.mission_reset()
    # ── detector setup ────────────────────────────────────────────────
    duburi.models(gate='gate_flare_medium_100ep')
    duburi.camera = 'forward'
    duburi.set_classes('gate,flare')   # prime dual-class filter before any vision state

    # ── vehicle capability detection ──────────────────────────────────
    profile = VehicleProfile.auto(duburi.client.node)
    log(f'[FSM] body={profile.name}  dvl={profile.has_dvl}  '
        f'manip={profile.has_manipulators}  depth={profile.mission_depth_m}m')

    # ── build + run FSM ───────────────────────────────────────────────
    # Route YASMIN's internal transition logs to the ROS logger.
    # YASMIN 5.x logs transitions via [INFO] state_machine.cpp automatically.
    set_ros_loggers()

    sm = build_gate_flare_fsm(duburi, profile)
    outcome = sm(Blackboard())
    log(f'[FSM] mission complete: {outcome}')

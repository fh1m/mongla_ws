"""Gate + Flare — YASMIN FSM mission.

Launched identically to any detected()-paradigm mission:

    ros2 run mongla_planner mission gate_flare_fsm

VehicleProfile is auto-detected from /mongla_manager ROS params at runtime:
  - Mongla 4.5 (yaw_source=dvl)     → DVL distance passes
  - Mongla_agile 2.0 (yaw_source=mavlink_ahrs) → timed-thrust passes

Override pool-day params via the params dict (no code edits needed):

    from mongla_planner.state_machines import build_gate_flare_fsm, VehicleProfile
    sm = build_gate_flare_fsm(mongla, profile,
                               params={'gate_heading': 63.0, 'depth_m': -0.9})
"""
from ..state_machines import run_fsm  # guaranteed disarm-on-exit
from yasmin_ros import set_ros_loggers

from ..state_machines import build_gate_flare_fsm, VehicleProfile


def run(mongla, log):
    mongla.mission_reset()
    # ── detector setup ────────────────────────────────────────────────
    mongla.models(gate='gate_flare_medium_100ep')
    mongla.camera = 'forward'
    mongla.set_classes('gate,flare')   # prime dual-class filter before any vision state

    # ── vehicle capability detection ──────────────────────────────────
    profile = VehicleProfile.auto(mongla.client.node)
    log(f'[FSM] body={profile.name}  dvl={profile.has_dvl}  '
        f'manip={profile.has_manipulators}  depth={profile.mission_depth_m}m')

    # ── build + run FSM ───────────────────────────────────────────────
    # Route YASMIN's internal transition logs to the ROS logger.
    # YASMIN 5.x logs transitions via [INFO] state_machine.cpp automatically.
    set_ros_loggers()

    sm = build_gate_flare_fsm(mongla, profile)
    outcome = run_fsm(mongla, sm)  # ALWAYS release-heading + disarm on exit
    log(f'[FSM] mission complete: {outcome}')

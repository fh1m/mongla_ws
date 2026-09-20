"""Gate pass → bin drop — 2-task FSM with camera switching.

Task 1: forward camera, gate+flare model
  arm → dive → find gate → home gate (align yaw+lat) → pass gate

Camera switch: SetDetectorState flips to downward cam + bin model atomically.

Task 2: downward camera, bin model
  dive → scan for bin_a → lock above bin → drop → confirm → surface

    ros2 run mongla_planner mission gate_then_bin_fsm

Works on Mongla 4.5 (DVL passes) and Mongla_agile 2.0 (timed passes).
Swap pool-day params via the params dict.
"""
from ..state_machines import run_fsm  # guaranteed disarm-on-exit
from yasmin_ros import set_ros_loggers

from ..state_machines import build_gate_then_bin_fsm, VehicleProfile, GATE_THEN_BIN_DEFAULTS


def run(mongla, log):
    mongla.mission_reset()
    # ── detector setup (task 1 — gate) ──────────────────────────────
    # SetDetectorState will switch to bin model mid-mission automatically.
    mongla.models(gate='gate_flare_medium_100ep')
    mongla.camera  = 'forward'
    mongla.set_classes('gate,flare')

    # ── vehicle detection ────────────────────────────────────────────
    profile = VehicleProfile.auto(mongla.client.node)
    log(f'[FSM] body={profile.name}  dvl={profile.has_dvl}')

    # ── pool-day params (edit here, not in plan builder) ─────────────
    params = {
        'gate_heading':  0.0,    # compass heading to gate — set on pool day
        'gate_depth_m': -0.8,
        'bin_depth_m':  -1.5,
        'gate_model':   'gate_flare_medium_100ep',
        'bin_model':    'bin_medium_100ep',    # swap for your actual bin model name
        'gate_classes': 'gate,flare',
        'bin_classes':  'bin_a',
    }

    set_ros_loggers()
    sm = build_gate_then_bin_fsm(mongla, profile, params=params)
    outcome = run_fsm(mongla, sm)  # ALWAYS release-heading + disarm on exit
    log(f'[FSM] result: {outcome}')

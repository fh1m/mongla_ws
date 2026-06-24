"""Gate pass → bin drop — 2-task FSM with camera switching.

Task 1: forward camera, gate+flare model
  arm → dive → find gate → home gate (yaw+lat, gate_guard) → pass gate

Camera switch: SetDetectorState flips to downward cam + bin model atomically.

Task 2: downward camera, bin model
  dive → scan for bin_a → lock above bin → drop → confirm → surface

    ros2 run duburi_planner mission gate_then_bin_fsm

Works on Duburi 4.5 (DVL passes) and Dubomini 2.0 (timed passes).
Swap pool-day params via the params dict.
"""
from yasmin import Blackboard
from yasmin_ros import set_ros_loggers

from ..state_machines import build_gate_then_bin_fsm, VehicleProfile, GATE_THEN_BIN_DEFAULTS


def run(duburi, log):
    duburi.mission_reset()
    # ── detector setup (task 1 — gate) ──────────────────────────────
    # SetDetectorState will switch to bin model mid-mission automatically.
    duburi.models(gate='gate_flare_medium_100ep')
    duburi.camera  = 'forward'
    duburi.set_classes('gate,flare')

    # ── vehicle detection ────────────────────────────────────────────
    profile = VehicleProfile.auto(duburi.client.node)
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
    sm = build_gate_then_bin_fsm(duburi, profile, params=params)
    outcome = sm(Blackboard())
    log(f'[FSM] result: {outcome}')

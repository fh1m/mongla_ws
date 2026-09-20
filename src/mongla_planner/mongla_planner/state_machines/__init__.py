"""YASMIN FSM planning layer for Mongla (mongla_ws).

Public API for mission authors:

    from mongla_planner.state_machines import (
        VehicleProfile,
        build_gate_flare_fsm,
        build_prequal_fsm,
    )
    from yasmin import Blackboard

    profile = VehicleProfile.auto(node)     # probe /mongla_manager
    # or:  VehicleProfile.mongla_heavy()
    # or:  VehicleProfile.mongla_agile()

    sm = build_gate_flare_fsm(mongla, profile)
    outcome = sm(Blackboard())

State library lives in states/; plan builders in plans/; core abstractions
(MonglaState, VehicleProfile, BK, outcomes) in core/.
"""
from .core.vehicle_profile import VehicleProfile
from .core.base_state import MonglaState
from .core.outcomes import SUCCEED, FAILED, TIMEOUT, ABORT
from .core.blackboard import BK
from .runner import run_fsm
from .states.utility import SetDetectorState
from .plans.gate_flare import build_gate_flare_fsm, GATE_FLARE_DEFAULTS
from .plans.prequal import build_prequal_fsm, PREQUAL_DEFAULTS
from .plans.gate_then_bin import build_gate_then_bin_fsm, GATE_THEN_BIN_DEFAULTS
from .plans.slalom import build_slalom_fsm, SLALOM_DEFAULTS
from .plans.bin_drop import build_bin_drop_fsm, BIN_DROP_DEFAULTS
from .plans.torpedo_fire import build_torpedo_fire_fsm, TORPEDO_FIRE_DEFAULTS
from .plans.return_gate import build_return_gate_fsm, RETURN_GATE_DEFAULTS
from .plans.full_competition import build_full_competition_fsm, FULL_COMP_DEFAULTS

__all__ = [
    'VehicleProfile', 'MonglaState', 'BK', 'SetDetectorState', 'run_fsm',
    'SUCCEED', 'FAILED', 'TIMEOUT', 'ABORT',
    'build_gate_flare_fsm', 'GATE_FLARE_DEFAULTS',
    'build_prequal_fsm', 'PREQUAL_DEFAULTS',
    'build_gate_then_bin_fsm', 'GATE_THEN_BIN_DEFAULTS',
    'build_slalom_fsm', 'SLALOM_DEFAULTS',
    'build_bin_drop_fsm', 'BIN_DROP_DEFAULTS',
    'build_torpedo_fire_fsm', 'TORPEDO_FIRE_DEFAULTS',
    'build_return_gate_fsm', 'RETURN_GATE_DEFAULTS',
    'build_full_competition_fsm', 'FULL_COMP_DEFAULTS',
]

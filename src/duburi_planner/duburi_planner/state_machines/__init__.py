"""YASMIN FSM planning layer for Mongla (duburi_ws).

Public API for mission authors:

    from duburi_planner.state_machines import (
        VehicleProfile,
        build_gate_flare_fsm,
        build_prequal_fsm,
    )
    from yasmin import Blackboard

    profile = VehicleProfile.auto(node)     # probe /duburi_manager
    # or:  VehicleProfile.duburi45()
    # or:  VehicleProfile.dubomini()

    sm = build_gate_flare_fsm(duburi, profile)
    outcome = sm(Blackboard())

State library lives in states/; plan builders in plans/; core abstractions
(DuburiState, VehicleProfile, BK, outcomes) in core/.
"""
from .core.vehicle_profile import VehicleProfile
from .core.base_state import DuburiState
from .core.outcomes import SUCCEED, FAILED, TIMEOUT, ABORT
from .core.blackboard import BK
from .plans.gate_flare import build_gate_flare_fsm, GATE_FLARE_DEFAULTS
from .plans.prequal import build_prequal_fsm, PREQUAL_DEFAULTS

__all__ = [
    'VehicleProfile', 'DuburiState', 'BK',
    'SUCCEED', 'FAILED', 'TIMEOUT', 'ABORT',
    'build_gate_flare_fsm', 'GATE_FLARE_DEFAULTS',
    'build_prequal_fsm', 'PREQUAL_DEFAULTS',
]

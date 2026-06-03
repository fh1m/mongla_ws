"""Unit tests for Mongla YASMIN FSM state layer.

Tests verify outcomes, DVL/timed branching, and exception → ABORT behaviour
without hardware. All DSL verbs are mocked so tests run offline.

Run:
    PYTHONPATH=src/duburi_planner pytest -q -p no:anyio src/duburi_planner/test/test_fsm_states.py
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
from unittest.mock import MagicMock, patch, call
from yasmin import Blackboard

# ── imports under test ────────────────────────────────────────────────────────
from duburi_planner.state_machines.core.outcomes import SUCCEED, FAILED, TIMEOUT, ABORT
from duburi_planner.state_machines.core.blackboard import BK
from duburi_planner.state_machines.core.vehicle_profile import VehicleProfile
from duburi_planner.state_machines.states.navigation import (
    ArmState, DisarmState, SetDepthState, LockHeadingState,
    MoveForwardState, MoveBackState, MoveLateralState, SurfaceState,
)
from duburi_planner.state_machines.states.vision import (
    VisionFindState, VisionHomeState, VisionScanState,
)
from duburi_planner.state_machines.states.utility import (
    CountdownState, PauseState, LogScoreState,
)


# ── fixtures ──────────────────────────────────────────────────────────────────

def _mock_result(success: bool = True) -> MagicMock:
    r = MagicMock()
    r.success = success
    return r


def _duburi() -> MagicMock:
    """Minimal DuburiMission mock with all DSL verbs stubbed."""
    d = MagicMock()
    d.vision = MagicMock()
    d.vision.find.return_value  = _mock_result(True)
    d.vision.home.return_value  = _mock_result(True)
    d.vision.scan.return_value  = _mock_result(True)
    return d


# ── VehicleProfile ────────────────────────────────────────────────────────────

class TestVehicleProfile:
    def test_duburi45_has_dvl(self):
        p = VehicleProfile.duburi45()
        assert p.has_dvl is True
        assert p.has_manipulators is True
        assert p.name == 'duburi45'

    def test_dubomini_no_dvl(self):
        p = VehicleProfile.dubomini()
        assert p.has_dvl is False
        assert p.has_manipulators is False
        assert p.name == 'dubomini'

    def test_profiles_are_frozen(self):
        p = VehicleProfile.duburi45()
        with pytest.raises(Exception):
            p.has_dvl = False  # type: ignore[misc]

    def test_auto_falls_back_on_no_service(self):
        node = MagicMock()
        node.create_client.return_value.wait_for_service.return_value = False
        profile = VehicleProfile.auto(node)
        assert profile.name == 'dubomini'
        assert profile.has_dvl is False

    def test_auto_falls_back_on_exception(self):
        node = MagicMock()
        node.create_client.side_effect = RuntimeError('no rclpy')
        profile = VehicleProfile.auto(node)
        assert profile.has_dvl is False


# ── ArmState ──────────────────────────────────────────────────────────────────

class TestArmState:
    def test_arms_and_returns_succeed(self):
        d = _duburi()
        state = ArmState(d, VehicleProfile.dubomini())
        assert state.execute(Blackboard()) == SUCCEED
        d.arm.assert_called_once()

    def test_connects_dvl_when_profile_has_dvl(self):
        d = _duburi()
        state = ArmState(d, VehicleProfile.duburi45())
        state.execute(Blackboard())
        d.dvl_connect.assert_called_once()

    def test_does_not_connect_dvl_when_no_dvl(self):
        d = _duburi()
        state = ArmState(d, VehicleProfile.dubomini())
        state.execute(Blackboard())
        d.dvl_connect.assert_not_called()

    def test_dvl_connect_failure_still_returns_succeed(self):
        d = _duburi()
        d.dvl_connect.side_effect = RuntimeError('dvl offline')
        state = ArmState(d, VehicleProfile.duburi45())
        bb = Blackboard()
        assert state.execute(bb) == SUCCEED
        assert bb[BK.DVL_CONNECTED] is False

    def test_arm_exception_returns_abort(self):
        d = _duburi()
        d.arm.side_effect = RuntimeError('arming failed')
        state = ArmState(d, VehicleProfile.dubomini())
        assert state.execute(Blackboard()) == ABORT
        d.stop.assert_called_once()


# ── MoveForwardState ──────────────────────────────────────────────────────────

class TestMoveForwardState:
    def test_dvl_path_when_has_dvl(self):
        d = _duburi()
        state = MoveForwardState(d, VehicleProfile.duburi45(),
                                  distance_m=2.0, duration=4.0, gain=60)
        assert state.execute(Blackboard()) == SUCCEED
        d.move_forward_dist.assert_called_once_with(2.0, gain=60)
        d.move_forward.assert_not_called()

    def test_timed_path_when_no_dvl(self):
        d = _duburi()
        state = MoveForwardState(d, VehicleProfile.dubomini(),
                                  distance_m=2.0, duration=4.0, gain=60)
        state.execute(Blackboard())
        d.move_forward.assert_called_once_with(4.0, gain=60)
        d.move_forward_dist.assert_not_called()

    def test_timed_only_when_no_distance_m(self):
        d = _duburi()
        state = MoveForwardState(d, VehicleProfile.duburi45(),
                                  duration=3.0, gain=50)
        state.execute(Blackboard())
        d.move_forward.assert_called_once_with(3.0, gain=50)
        d.move_forward_dist.assert_not_called()

    def test_exception_returns_abort_and_stops(self):
        d = _duburi()
        d.move_forward.side_effect = RuntimeError('thruster fault')
        state = MoveForwardState(d, VehicleProfile.dubomini(), duration=3.0, gain=60)
        assert state.execute(Blackboard()) == ABORT
        d.stop.assert_called_once()


# ── MoveBackState ─────────────────────────────────────────────────────────────

class TestMoveBackState:
    def test_dvl_path(self):
        d = _duburi()
        state = MoveBackState(d, VehicleProfile.duburi45(), distance_m=1.0, duration=2.0)
        state.execute(Blackboard())
        d.move_back_dist.assert_called_once_with(1.0, gain=60)

    def test_timed_path(self):
        d = _duburi()
        state = MoveBackState(d, VehicleProfile.dubomini(), duration=2.0)
        state.execute(Blackboard())
        d.move_back.assert_called_once_with(2.0, gain=60)


# ── SetDepthState ─────────────────────────────────────────────────────────────

class TestSetDepthState:
    def test_calls_set_depth_and_returns_succeed(self):
        d = _duburi()
        state = SetDepthState(d, VehicleProfile.dubomini(), depth_m=-0.8)
        assert state.execute(Blackboard()) == SUCCEED
        d.set_depth.assert_called_once()
        args, kwargs = d.set_depth.call_args
        assert args[0] == -0.8


# ── VisionFindState ───────────────────────────────────────────────────────────

class TestVisionFindState:
    def test_returns_succeed_on_detection(self):
        d = _duburi()
        d.vision.find.return_value = _mock_result(True)
        state = VisionFindState(d, VehicleProfile.dubomini(), target='gate')
        assert state.execute(Blackboard()) == SUCCEED

    def test_returns_timeout_when_not_found(self):
        d = _duburi()
        d.vision.find.return_value = _mock_result(False)
        state = VisionFindState(d, VehicleProfile.dubomini(), target='gate')
        assert state.execute(Blackboard()) == TIMEOUT

    def test_passes_target_and_move_to_dsl(self):
        d = _duburi()
        state = VisionFindState(d, VehicleProfile.dubomini(),
                                 target='flare', move='yaw_right', gain=40, timeout=30.0)
        state.execute(Blackboard())
        d.vision.find.assert_called_once_with(
            target='flare', move='yaw_right', gain=40, timeout=30.0)


# ── VisionHomeState ───────────────────────────────────────────────────────────

class TestVisionHomeState:
    def test_returns_succeed_on_converge(self):
        d = _duburi()
        d.vision.home.return_value = _mock_result(True)
        state = VisionHomeState(d, VehicleProfile.dubomini(), target='gate',
                                 yaw=True, lat=True, duration=10.0)
        assert state.execute(Blackboard()) == SUCCEED

    def test_returns_failed_on_lost(self):
        d = _duburi()
        d.vision.home.return_value = _mock_result(False)
        state = VisionHomeState(d, VehicleProfile.dubomini(), target='gate',
                                 yaw=True, lat=True, duration=10.0)
        assert state.execute(Blackboard()) == FAILED

    def test_gate_guard_forwarded_to_dsl(self):
        d = _duburi()
        state = VisionHomeState(d, VehicleProfile.dubomini(), target='gate',
                                 yaw=True, lat=True, gate_guard=True,
                                 pass_at=0.38, duration=15.0)
        state.execute(Blackboard())
        _, kwargs = d.vision.home.call_args
        assert kwargs.get('gate_guard') is True
        assert kwargs.get('pass_at') == pytest.approx(0.38)


# ── VisionScanState ───────────────────────────────────────────────────────────

class TestVisionScanState:
    def test_returns_succeed_on_detection(self):
        d = _duburi()
        d.vision.scan.return_value = _mock_result(True)
        state = VisionScanState(d, VehicleProfile.dubomini(), target='gate')
        assert state.execute(Blackboard()) == SUCCEED

    def test_returns_timeout_when_no_detection(self):
        d = _duburi()
        d.vision.scan.return_value = _mock_result(False)
        state = VisionScanState(d, VehicleProfile.dubomini(), target='gate')
        assert state.execute(Blackboard()) == TIMEOUT


# ── SurfaceState ──────────────────────────────────────────────────────────────

class TestSurfaceState:
    def test_stops_ascends_disarms(self):
        d = _duburi()
        state = SurfaceState(d, VehicleProfile.dubomini())
        assert state.execute(Blackboard()) == SUCCEED
        d.stop.assert_called_once()
        d.disarm.assert_called_once()

    def test_release_heading_failure_does_not_abort(self):
        d = _duburi()
        d.release_heading.side_effect = RuntimeError('lock not active')
        state = SurfaceState(d, VehicleProfile.dubomini())
        assert state.execute(Blackboard()) == SUCCEED


# ── CountdownState ────────────────────────────────────────────────────────────

class TestCountdownState:
    def test_calls_countdown_sets_bb(self):
        d = _duburi()
        state = CountdownState(d, VehicleProfile.dubomini(), seconds=3)
        bb = Blackboard()
        assert state.execute(bb) == SUCCEED
        d.countdown.assert_called_once_with(3)
        assert bb[BK.MISSION_START_T] > 0


# ── SetDetectorState ─────────────────────────────────────────────────────────

class TestSetDetectorState:
    def test_sets_camera_on_duburi(self):
        from duburi_planner.state_machines.states.utility import SetDetectorState
        d = _duburi()
        state = SetDetectorState(d, VehicleProfile.dubomini(), camera='downward')
        state.execute(Blackboard())
        assert d.camera == 'downward'

    def test_calls_set_classes(self):
        from duburi_planner.state_machines.states.utility import SetDetectorState
        d = _duburi()
        state = SetDetectorState(d, VehicleProfile.dubomini(), classes='bin_a,bin_b')
        state.execute(Blackboard())
        d.set_classes.assert_called_once_with('bin_a,bin_b')

    def test_calls_set_model(self):
        from duburi_planner.state_machines.states.utility import SetDetectorState
        d = _duburi()
        state = SetDetectorState(d, VehicleProfile.dubomini(), model='bin_medium_100ep')
        state.execute(Blackboard())
        d.set_model.assert_called_once_with('bin_medium_100ep')

    def test_all_three_together(self):
        from duburi_planner.state_machines.states.utility import SetDetectorState
        d = _duburi()
        state = SetDetectorState(d, VehicleProfile.dubomini(),
                                  camera='downward', classes='bin_a', model='bin_v1')
        assert state.execute(Blackboard()) == SUCCEED
        assert d.camera == 'downward'
        d.set_classes.assert_called_once_with('bin_a')
        d.set_model.assert_called_once_with('bin_v1')

    def test_noop_when_all_none(self):
        from duburi_planner.state_machines.states.utility import SetDetectorState
        d = _duburi()
        d.camera = 'forward'
        state = SetDetectorState(d, VehicleProfile.dubomini())
        state.execute(Blackboard())
        assert d.camera == 'forward'   # unchanged
        d.set_classes.assert_not_called()
        d.set_model.assert_not_called()


# ── camera passthrough in vision states ──────────────────────────────────────

class TestVisionStateCameraPassthrough:
    def test_find_forwards_camera_kwarg(self):
        d = _duburi()
        state = VisionFindState(d, VehicleProfile.dubomini(),
                                 target='bin_a', camera='downward',
                                 move='still', timeout=10.0)
        state.execute(Blackboard())
        _, kw = d.vision.find.call_args
        assert kw.get('camera') == 'downward'

    def test_find_no_camera_no_kwarg(self):
        d = _duburi()
        state = VisionFindState(d, VehicleProfile.dubomini(), target='gate')
        state.execute(Blackboard())
        _, kw = d.vision.find.call_args
        assert 'camera' not in kw   # DSL uses sticky duburi.camera

    def test_home_forwards_camera_kwarg(self):
        d = _duburi()
        state = VisionHomeState(d, VehicleProfile.dubomini(),
                                 target='bin_a', camera='downward',
                                 yaw=True, lat=True)
        state.execute(Blackboard())
        _, kw = d.vision.home.call_args
        assert kw.get('camera') == 'downward'

    def test_home_no_camera_no_kwarg(self):
        d = _duburi()
        state = VisionHomeState(d, VehicleProfile.dubomini(), target='gate', yaw=True)
        state.execute(Blackboard())
        _, kw = d.vision.home.call_args
        assert 'camera' not in kw

    def test_scan_forwards_camera_kwarg(self):
        d = _duburi()
        state = VisionScanState(d, VehicleProfile.dubomini(),
                                 target='bin_a', camera='downward')
        state.execute(Blackboard())
        _, kw = d.vision.scan.call_args
        assert kw.get('camera') == 'downward'

    def test_scan_no_camera_no_kwarg(self):
        d = _duburi()
        state = VisionScanState(d, VehicleProfile.dubomini(), target='gate')
        state.execute(Blackboard())
        _, kw = d.vision.scan.call_args
        assert 'camera' not in kw


# ── plan builder smoke test (no hardware) ─────────────────────────────────────

class TestPlanBuilders:
    def test_build_gate_flare_fsm_returns_state_machine(self):
        from yasmin import StateMachine
        from duburi_planner.state_machines import build_gate_flare_fsm
        d = _duburi()
        sm = build_gate_flare_fsm(d, VehicleProfile.dubomini())
        assert isinstance(sm, StateMachine)

    def test_build_prequal_fsm_returns_state_machine(self):
        from yasmin import StateMachine
        from duburi_planner.state_machines import build_prequal_fsm
        d = _duburi()
        sm = build_prequal_fsm(d, VehicleProfile.dubomini())
        assert isinstance(sm, StateMachine)

    def test_gate_flare_params_override_defaults(self):
        from duburi_planner.state_machines import build_gate_flare_fsm, GATE_FLARE_DEFAULTS
        d = _duburi()
        sm = build_gate_flare_fsm(d, VehicleProfile.duburi45(),
                                   params={'gate_heading': 90.0})
        from yasmin import StateMachine
        assert isinstance(sm, StateMachine)

    def test_build_gate_then_bin_fsm_returns_state_machine(self):
        from yasmin import StateMachine
        from duburi_planner.state_machines import build_gate_then_bin_fsm
        d = _duburi()
        sm = build_gate_then_bin_fsm(d, VehicleProfile.dubomini())
        assert isinstance(sm, StateMachine)

    def test_gate_then_bin_has_camera_switch_state(self):
        from duburi_planner.state_machines import build_gate_then_bin_fsm
        d = _duburi()
        sm = build_gate_then_bin_fsm(d, VehicleProfile.duburi45())
        states = sm.get_states()
        assert 'SWITCH_TO_BIN' in states
        assert 'SCAN_BIN' in states
        assert 'LOCK_BIN' in states
        assert 'DROP_BIN' in states

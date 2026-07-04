"""Unit tests for Mongla YASMIN FSM state layer.

Tests verify outcomes, DVL/timed branching, and exception → ABORT behaviour
without hardware. All DSL verbs are mocked so tests run offline.

Run:
    PYTHONPATH=src/duburi_planner pytest -q -p no:anyio src/duburi_planner/test/test_fsm_states.py
"""
import sys
import os
import time
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
    VisionSearchState, VisionAlignState, VisionMoveState,
)
from duburi_planner.state_machines.states.utility import (
    CountdownState, PauseState, LogScoreState,
)


# ── fixtures ──────────────────────────────────────────────────────────────────

def _mock_result(success: bool = True) -> MagicMock:
    r = MagicMock()
    r.success = success
    return r


def _vis_result(ok: bool = True) -> MagicMock:
    """Stand-in for vision_dsl.VisionResult (truthy on .ok)."""
    r = MagicMock()
    r.ok = ok
    return r


def _duburi() -> MagicMock:
    """Minimal DuburiMission mock with all DSL verbs stubbed."""
    d = MagicMock()
    d.vision = MagicMock()
    d.vision.align.return_value = _vis_result(True)
    d.vision.move.return_value  = _vis_result(True)
    d.detected.return_value = True
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


# ── VisionSearchState ─────────────────────────────────────────────────────────

class TestVisionSearchState:
    def test_returns_succeed_on_detection(self):
        d = _duburi()
        d.detected.return_value = True
        state = VisionSearchState(d, VehicleProfile.dubomini(), target='gate',
                                  timeout=5.0)
        assert state.execute(Blackboard()) == SUCCEED

    def test_returns_timeout_when_not_found(self):
        d = _duburi()
        d.detected.return_value = False
        state = VisionSearchState(d, VehicleProfile.dubomini(), target='gate',
                                  timeout=0.2)
        assert state.execute(Blackboard()) == TIMEOUT

    def test_forward_pattern_creeps(self):
        d = _duburi()
        d.detected.return_value = False
        state = VisionSearchState(d, VehicleProfile.dubomini(), target='gate',
                                  pattern='forward', timeout=0.2, gain=35)
        state.execute(Blackboard())
        assert d.move_forward.called

    def test_yaw_pattern_sweeps(self):
        d = _duburi()
        d.detected.return_value = False
        state = VisionSearchState(d, VehicleProfile.dubomini(), target='gate',
                                  pattern='yaw', timeout=0.2, yaw_step=20.0)
        state.execute(Blackboard())
        assert d.yaw_right.called


# ── VisionAlignState ──────────────────────────────────────────────────────────

class TestVisionAlignState:
    def test_returns_succeed_on_align(self):
        d = _duburi()
        d.vision.align.return_value = _vis_result(True)
        state = VisionAlignState(d, VehicleProfile.dubomini(), target='gate',
                                 yaw=0, lat=0, duration=10.0)
        assert state.execute(Blackboard()) == SUCCEED

    def test_returns_failed_on_miss(self):
        d = _duburi()
        d.vision.align.return_value = _vis_result(False)
        state = VisionAlignState(d, VehicleProfile.dubomini(), target='gate',
                                 yaw=0, lat=0, duration=10.0)
        assert state.execute(Blackboard()) == FAILED

    def test_axis_flags_passed_to_dsl(self):
        d = _duburi()
        # True -> centre (0.0); number -> signed offset; None/False -> off.
        state = VisionAlignState(d, VehicleProfile.dubomini(), target='red_pipe',
                                 yaw=True, lat=80, depth=None,
                                 err=30, gain=25, duration=15.0)
        state.execute(Blackboard())
        _, kwargs = d.vision.align.call_args
        assert kwargs.get('yaw') == pytest.approx(0.0)
        assert kwargs.get('lat') == pytest.approx(80.0)
        assert kwargs.get('depth') is None
        assert kwargs.get('err') == 30
        assert kwargs.get('gain') == 25


# ── VisionMoveState ───────────────────────────────────────────────────────────

class TestVisionMoveState:
    def test_returns_succeed_on_reach(self):
        d = _duburi()
        d.vision.move.return_value = _vis_result(True)
        state = VisionMoveState(d, VehicleProfile.dubomini(), target='gate',
                                fwd=80, mode='area')
        assert state.execute(Blackboard()) == SUCCEED

    def test_returns_failed_on_miss(self):
        d = _duburi()
        d.vision.move.return_value = _vis_result(False)
        state = VisionMoveState(d, VehicleProfile.dubomini(), target='gate')
        assert state.execute(Blackboard()) == FAILED

    def test_move_args_passed_to_dsl(self):
        d = _duburi()
        state = VisionMoveState(d, VehicleProfile.dubomini(), target='red_pipe',
                                fwd=60, mode='height', gain=40, duration=18.0)
        state.execute(Blackboard())
        _, kwargs = d.vision.move.call_args
        assert kwargs.get('fwd') == 60
        assert kwargs.get('mode') == 'height'
        assert kwargs.get('gain') == 40


# ── LockHeadingState / DisarmState ─────────────────────────────────────────────

class TestLockHeadingState:
    def test_lock_timeout_is_hold_duration_not_state_timeout(self):
        d = _duburi()
        state = LockHeadingState(d, VehicleProfile.dubomini(), heading=90.0)
        assert state.execute(Blackboard()) == SUCCEED
        # The lock's auto-release must be the long hold duration, NOT the
        # state's execution TIMEOUT_S (which would kill the lock mid-task).
        _, kwargs = d.lock_heading.call_args
        assert kwargs['timeout'] > state.TIMEOUT_S


class TestDisarmState:
    def test_releases_heading_before_disarm(self):
        d = _duburi()
        state = DisarmState(d, VehicleProfile.dubomini())
        assert state.execute(Blackboard()) == SUCCEED
        d.release_heading.assert_called_once()
        d.disarm.assert_called_once()

    def test_release_heading_failure_does_not_block_disarm(self):
        d = _duburi()
        d.release_heading.side_effect = RuntimeError('lock not active')
        state = DisarmState(d, VehicleProfile.dubomini())
        assert state.execute(Blackboard()) == SUCCEED
        d.disarm.assert_called_once()


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
    def test_align_forwards_camera_kwarg(self):
        d = _duburi()
        state = VisionAlignState(d, VehicleProfile.dubomini(),
                                 target='bin_a', camera='downward',
                                 lat=0, depth=0)
        state.execute(Blackboard())
        _, kw = d.vision.align.call_args
        assert kw.get('camera') == 'downward'

    def test_align_default_camera_is_none(self):
        d = _duburi()
        state = VisionAlignState(d, VehicleProfile.dubomini(), target='gate', yaw=0)
        state.execute(Blackboard())
        _, kw = d.vision.align.call_args
        assert kw.get('camera') is None   # DSL falls back to sticky duburi.camera

    def test_move_forwards_camera_kwarg(self):
        d = _duburi()
        state = VisionMoveState(d, VehicleProfile.dubomini(),
                                target='bin_a', camera='downward')
        state.execute(Blackboard())
        _, kw = d.vision.move.call_args
        assert kw.get('camera') == 'downward'

    def test_search_forwards_camera_kwarg(self):
        d = _duburi()
        d.detected.return_value = True
        state = VisionSearchState(d, VehicleProfile.dubomini(),
                                  target='bin_a', camera='downward', timeout=5.0)
        state.execute(Blackboard())
        _, kw = d.detected.call_args
        assert kw.get('camera') == 'downward'


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


# --------------------------------------------------------------------------- #
#  run_fsm: an FSM launcher must ALWAYS release-heading + disarm on exit --    #
#  even when a state returns ABORT normally or the machine raises (an FSM      #
#  could otherwise end ARMED; mission.py's except-only backstop won't fire).   #
# --------------------------------------------------------------------------- #
from unittest.mock import MagicMock
from duburi_planner.state_machines import run_fsm


def test_run_fsm_disarms_on_normal_abort():
    duburi = MagicMock()
    sm = MagicMock(return_value='ABORT')          # terminates ABORT *normally*
    out = run_fsm(duburi, sm)
    assert out == 'ABORT'
    duburi.release_heading.assert_called_once()   # safe exit still runs
    duburi.disarm.assert_called_once()


def test_run_fsm_disarms_on_success():
    duburi = MagicMock()
    sm = MagicMock(return_value='SUCCEED')
    run_fsm(duburi, sm)
    duburi.disarm.assert_called_once()


def test_run_fsm_disarms_even_when_fsm_raises():
    duburi = MagicMock()
    sm = MagicMock(side_effect=RuntimeError('state blew up'))
    with pytest.raises(RuntimeError):
        run_fsm(duburi, sm)
    duburi.disarm.assert_called_once()            # finally still disarmed


def test_run_fsm_safe_exit_never_raises_from_cleanup():
    duburi = MagicMock()
    duburi.disarm.side_effect = RuntimeError('disarm server error')
    duburi.release_heading.side_effect = RuntimeError('lock error')
    sm = MagicMock(return_value='SUCCEED')
    # a failing disarm/release in the finally must not mask the outcome
    assert run_fsm(duburi, sm) == 'SUCCEED'


# ── DuburiState safety CEILING (DSL-H3: timed_out() wired into execute) ────────
class TestStateTimeoutCeiling:
    """A state that overruns its TIMEOUT_S must stop() + route to ABORT (-> SURFACE
    in every plan). Previously timed_out() was never called (dead code)."""

    def _slow_state(self, d, run_s, ceiling_s, outcome=SUCCEED):
        from duburi_planner.state_machines.core.base_state import DuburiState

        class _Slow(DuburiState):
            TIMEOUT_S = ceiling_s

            def _run(self, bb):
                time.sleep(run_s)
                return outcome

        return _Slow(d, VehicleProfile.dubomini(), [SUCCEED, FAILED])

    def test_overrun_returns_abort_and_stops(self):
        d = _duburi()
        state = self._slow_state(d, run_s=0.05, ceiling_s=0.01)   # runs past ceiling
        assert state.execute(Blackboard()) == ABORT
        d.stop.assert_called_once()

    def test_within_ceiling_keeps_natural_outcome(self):
        d = _duburi()
        state = self._slow_state(d, run_s=0.0, ceiling_s=10.0, outcome=SUCCEED)
        assert state.execute(Blackboard()) == SUCCEED
        d.stop.assert_not_called()   # no ceiling trip -> no safety stop

    def test_overrun_records_error(self):
        d = _duburi()
        bb = Blackboard()
        state = self._slow_state(d, run_s=0.05, ceiling_s=0.01)
        state.execute(bb)
        assert 'TIMEOUT_S' in str(bb[BK.LAST_ERROR])

    def test_exception_still_aborts_even_if_also_overran(self):
        # an exception path takes precedence and still returns ABORT (not double-handled)
        from duburi_planner.state_machines.core.base_state import DuburiState

        class _Boom(DuburiState):
            TIMEOUT_S = 0.01

            def _run(self, bb):
                time.sleep(0.03)
                raise RuntimeError('boom')

        d = _duburi()
        state = _Boom(d, VehicleProfile.dubomini(), [SUCCEED])
        assert state.execute(Blackboard()) == ABORT
        d.stop.assert_called_once()

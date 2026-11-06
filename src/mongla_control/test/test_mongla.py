"""The Mongla facade over a recording board.

No board here -- `FakeBoard` has the shape of `SrotFC` (one `move()` per board
verb, `manual()` for the streamed axes) and records what it was asked. The
lock, the arm gate, the result type and the side effects are the real
`Mongla` class.
"""

import logging
import math
import threading
import time

import pytest

from mongla_interfaces.action import Move

from mongla_control.errors import NotArmedError
from mongla_control.fc.base import DENIED, SUCCEEDED, MoveResult
from mongla_control.mongla import Mongla


class FakeBoard:
    """Records calls; reports a fixed attitude. Narrow on purpose, so a facade
    that reaches for a method SrotFC lacks fails here first."""

    def __init__(self, *, depth=-0.5, yaw=10.0, armed=True,
                 cal_ack=(True, 'ACCEPTED'), cal_result_depth=0.0,
                 move_code=SUCCEEDED):
        self._attitude = {'yaw': yaw, 'pitch': 0.0, 'roll': 0.0, 'depth': depth}
        self._armed = armed
        self._mode = 'STABILIZE'
        # Baro calibration: the ack the board returns, and the depth it reads
        # AFTER the re-zero (None = depth unchanged, a cal that ACKed but did
        # not actually zero).
        self._cal_ack = cal_ack
        self._cal_result_depth = cal_result_depth
        self._move_code = move_code
        self.calls = []

    def get_attitude(self):
        return dict(self._attitude)

    def get_attitude_age(self):
        return 0.0

    def calibrate_barometer(self, timeout=6.0):
        self.calls.append(('calibrate_barometer',))
        if self._cal_result_depth is not None:
            self._attitude['depth'] = self._cal_result_depth
        return self._cal_ack

    def get_mode(self):
        return self._mode

    def is_armed(self):
        return self._armed

    def move(self, verb, *, on_progress=None, abort_fn=None, **kw):
        self.calls.append(('move', verb, kw))
        if on_progress is not None:
            on_progress(1.0)
        return MoveResult(self._move_code, f'{verb}: done')

    def manual(self, fwd=0.0, lat=0.0, up=0.0, yaw=0.0):
        self.calls.append(('manual', fwd, lat, up, yaw))

    def send_neutral(self):
        self.calls.append(('send_neutral',))

    def stop_motion(self):
        self.calls.append(('stop_motion',))

    def arm(self, timeout=15.0, abort=None):
        # The real contract: abort is a CALLABLE. Calling it guards the facade
        # wiring -- passing `self._abort_fn()` (a bool) would raise here.
        if abort is not None:
            abort()
        self._armed = True
        return True, 'ACCEPTED'

    def disarm(self, timeout=20.0):
        self._armed = False
        return True, 'ACCEPTED'

    def set_mode(self, name, timeout=8.0):
        self._mode = name
        return True, 'ACCEPTED'


def _mongla(**kw):
    return Mongla(FakeBoard(**kw), logging.getLogger('test.mongla'))


@pytest.fixture
def mongla():
    return _mongla()


# ── results ──────────────────────────────────────────────────────────────────
def test_arm_returns_move_result(mongla):
    result = mongla.arm()
    assert isinstance(result, Move.Result)
    assert result.success is True
    assert result.message.startswith('arm:')


def test_set_mode_returns_move_result(mongla):
    result = mongla.set_mode('MANUAL')
    assert isinstance(result, Move.Result)
    assert result.success is True
    assert mongla.fc.get_mode() == 'MANUAL'


# ── board verbs: one move() each, the board's outcome is the result ──────────
def test_a_board_verb_is_one_move_with_its_arguments(mongla):
    result = mongla.move_forward(duration=3.0, gain=50.0)
    assert mongla.fc.calls == [('move', 'move_forward',
                                {'duration': 3.0, 'gain': 50.0, 'settle': 0.0})]
    assert result.success is True
    assert math.isclose(result.final_value, -0.5, abs_tol=0.01)   # depth now


def test_a_denied_move_is_a_failed_result():
    """The board's DENIED must reach the caller as a failure, never a success
    that moved nothing."""
    m = _mongla(move_code=DENIED)
    assert m.move_forward(duration=1.0).success is False


def test_a_heading_verb_reports_the_heading_it_ended_on(mongla):
    result = mongla.turn(target=90.0)
    assert math.isclose(result.final_value, 10.0)                # the fake's yaw


def test_progress_is_relayed(mongla):
    seen = []
    mongla.set_depth(target=-1.0, on_progress=seen.append)
    assert seen == [1.0]


def test_stop_and_pause_are_board_verbs(mongla):
    mongla.stop()
    mongla.pause(duration=0.5)
    assert [c[1] for c in mongla.fc.calls] == ['stop', 'pause']


# ── the arm gate ─────────────────────────────────────────────────────────────
def test_motion_verb_raises_not_armed_when_disarmed():
    """A board that is disarmed moves nothing, so a silent success would be a
    lie. The action server turns the error into success=False."""
    m = _mongla(armed=False)
    with pytest.raises(NotArmedError):
        m.move_forward(duration=0.05, gain=50.0)
    assert not m.fc.calls, 'nothing may reach the board before the gate'


def test_unarm_safe_verbs_succeed_when_disarmed():
    """stop and pause must work in any arm state -- they are how a mission
    gets the vehicle back to neutral after an emergency."""
    m = _mongla(armed=False)
    assert m.stop().success is True
    assert m.pause(duration=0.0).success is True


def test_motion_succeeds_after_arm():
    m = _mongla(armed=False)
    m.arm()
    assert m.move_forward(duration=0.05, gain=50.0).success is True


def test_verbs_are_serialised_one_at_a_time(mongla):
    """Two threads, one board: the second verb waits for the first."""
    gate = threading.Event()
    inside = []
    real_move = mongla.fc.move

    def slow_move(verb, **kw):
        inside.append(verb)
        gate.wait(1.0)
        return real_move(verb, **kw)

    mongla.fc.move = slow_move
    t = threading.Thread(target=lambda: mongla.move_forward(duration=1.0), daemon=True)
    t.start()
    time.sleep(0.05)
    t2 = threading.Thread(target=lambda: mongla.move_left(duration=1.0), daemon=True)
    t2.start()
    time.sleep(0.05)
    assert inside == ['move_forward'], 'a second verb entered while one was running'
    gate.set()
    t.join(1.0)
    t2.join(1.0)
    assert inside == ['move_forward', 'move_left']


# ── vision telemetry slot + rich _make_result ────────────────────────────────
def test_report_vision_then_telemetry_fresh(mongla):
    mongla.report_vision(-42.0, 8.0)
    assert mongla.vision_telemetry() == (-42.0, 8.0)


def test_vision_telemetry_stale_returns_none(mongla):
    mongla.report_vision(10.0, 20.0)
    time.sleep(0.05)
    assert mongla.vision_telemetry(fresh_s=0.001) is None


def test_vision_telemetry_none_before_any_report(mongla):
    assert mongla.vision_telemetry() is None


def test_make_result_sets_vision_fields(mongla):
    r = mongla._make_result(True, 'x', final_value=0.0, error_value=12.0,
                            end_x_px=-42.0, end_y_px=8.0,
                            fill_frac=0.6, elapsed_s=3.1)
    assert r.end_x_px == pytest.approx(-42.0)
    assert r.end_y_px == pytest.approx(8.0)
    assert r.fill_frac == pytest.approx(0.6)
    assert r.elapsed_s == pytest.approx(3.1)


def test_make_result_defaults_nan_for_non_vision(mongla):
    r = mongla._make_result(True, 'arm', final_value=-0.5)
    assert math.isnan(r.end_x_px) and math.isnan(r.end_y_px)
    assert r.fill_frac == 0.0 and r.elapsed_s == 0.0


# ── depth calibration (barometer re-zero before a dive) ──────────────────────
def test_calibrate_depth_success_when_surfaced_and_disarmed():
    m = _mongla(armed=False, depth=-0.07, cal_result_depth=0.0)
    r = m.calibrate_depth(settle_s=0.0)
    assert r.success is True
    assert ('calibrate_barometer',) in m.fc.calls
    assert abs(r.final_value) < 0.05          # verified post-cal depth ~0


def test_calibrate_depth_skips_when_armed():
    # NEVER calibrate a diving hull -- arm state is the surface proxy.
    m = _mongla(armed=True, depth=-1.5)
    r = m.calibrate_depth(settle_s=0.0)
    assert r.success is False
    assert ('calibrate_barometer',) not in m.fc.calls


def test_calibrate_depth_refuses_when_not_at_surface():
    # disarmed but 0.5 m deep (> 0.30 bound): not surfaced or a baro fault.
    m = _mongla(armed=False, depth=-0.5)
    r = m.calibrate_depth(settle_s=0.0)
    assert r.success is False
    assert ('calibrate_barometer',) not in m.fc.calls


def test_calibrate_depth_fails_when_post_not_zero():
    # the cal ACKs but the depth stays off zero: the verify must FAIL.
    m = _mongla(armed=False, depth=-0.08, cal_result_depth=-0.20)
    r = m.calibrate_depth(settle_s=0.0)
    assert r.success is False
    assert ('calibrate_barometer',) in m.fc.calls


def test_mission_reset_calibrates_when_disarmed(monkeypatch):
    monkeypatch.setattr(time, 'sleep', lambda *_: None)
    m = _mongla(armed=False, depth=-0.06, cal_result_depth=0.0)
    r = m.mission_reset()
    assert r.success is True
    assert ('calibrate_barometer',) in m.fc.calls
    assert ('send_neutral',) in m.fc.calls     # any streamed demand zeroed


def test_mission_reset_skips_calibration_when_armed():
    m = _mongla(armed=True, depth=-1.0)
    r = m.mission_reset()
    assert r.success is True
    assert ('calibrate_barometer',) not in m.fc.calls


def test_mission_reset_survives_calibration_error():
    # A raising calibration must NEVER break mission_reset's safety duties.
    m = _mongla(armed=False, depth=-0.05)

    def _boom(*a, **k):
        raise RuntimeError('cal exploded')
    m.fc.calibrate_barometer = _boom
    assert m.mission_reset().success is True


# ── calc_distance (downward optical-flow odometry bracket) ───────────────────
class _FakeDistanceState:
    def __init__(self, dist=1.23):
        self.dist = dist
        self.started_lateral = None
        self.stopped = False

    def start(self, *, lateral):
        self.started_lateral = lateral
        return True, 'ACTIVE'

    def stop(self):
        self.stopped = True
        return True, 'STOPPED', self.dist


def _mongla_with_distance(ds):
    return Mongla(FakeBoard(), logging.getLogger('test.dist'),
                  distance_provider=lambda: ds)


def test_calc_distance_start_latches_axial_by_default():
    ds = _FakeDistanceState()
    r = _mongla_with_distance(ds).calc_distance('start')
    assert r.success is True
    assert ds.started_lateral is False


def test_calc_distance_start_uses_lateral_after_move_left():
    ds = _FakeDistanceState()
    d = _mongla_with_distance(ds)
    d.move_left(duration=0.02, gain=50.0)     # sets _distance_axis='lateral'
    d.calc_distance('start')
    assert ds.started_lateral is True


def test_calc_distance_stop_returns_metres_in_final_value():
    ds = _FakeDistanceState(dist=2.5)
    r = _mongla_with_distance(ds).calc_distance('stop')
    assert r.success is True
    assert r.final_value == 2.5
    assert ds.stopped is True


def test_calc_distance_no_provider_is_graceful(mongla):
    assert mongla.calc_distance('start').success is False   # reports absence


# ── disarm signals abort: host loops and a queued shot must not outlive it ───
class _AbortWatchingBoard(FakeBoard):
    """Records whether the abort was already signalled when disarm reached the
    wire, and -- like the real `SrotFC.arm` poll -- refuses an arm whose abort
    reads True, so a stale abort left by disarm would show up as a failed arm."""

    def __init__(self, abort_event_getter, **kw):
        super().__init__(**kw)
        self._get_ev = abort_event_getter
        self.abort_set_at_disarm = None

    def disarm(self, timeout=20.0):
        self.abort_set_at_disarm = self._get_ev().is_set()
        return super().disarm(timeout)

    def arm(self, timeout=15.0, abort=None):
        if abort is not None and abort():
            return False, 'aborted'
        return super().arm(timeout, abort)


def _abort_watching_mongla():
    box = {}
    board = _AbortWatchingBoard(lambda: box['m']._abort_event)
    box['m'] = Mongla(board, logging.getLogger('test.mongla'))
    return box['m'], board


def test_disarm_stops_a_running_host_loop():
    """A vision loop (or the `_fire_async` shot thread) exits only on the abort
    flag. Disarm used to signal nothing, so it kept streaming -- and a delayed
    torpedo still left -- after the operator disarmed."""
    m, board = _abort_watching_mongla()
    exited = threading.Event()

    def _host_loop():
        t_end = time.monotonic() + 2.0
        while time.monotonic() < t_end:
            if m._abort_fn():
                exited.set()
                return
            time.sleep(0.02)

    th = threading.Thread(target=_host_loop, daemon=True)
    th.start()
    time.sleep(0.05)
    m.disarm()
    assert exited.wait(1.0), 'the host loop outlived disarm'
    assert board.abort_set_at_disarm is True, 'abort must be signalled BEFORE the disarm'


def test_disarm_does_not_poison_the_next_arm():
    """The abort disarm leaves set is cleared by arm at its own entry -- the
    same clean-slate rule every other verb follows."""
    m, _ = _abort_watching_mongla()
    m.disarm()
    assert m._abort_event.is_set()
    assert m.arm().success is True
    assert not m._abort_event.is_set()

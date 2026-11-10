"""Who keeps the vehicle honest when a process dies (issue #13).

Two decisions, made by the operator on 2026-10-01:
  * a MISSION that dies mid-run (crash, kill, link drop) -> the manager aborts,
    brakes and SURFACEs the armed vehicle;
  * a MANAGER whose executor is stuck -> the heartbeat is WITHHELD, so the
    board's own 5 s GCS failsafe surfaces it. Before this, B-72's heartbeat
    thread kept a hung manager's board fed for ever.
"""
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

import mongla_manager.auv_manager_node as M
import mongla_manager.node_support as node_support


# ── the decision, pure ───────────────────────────────────────────────────────
@pytest.mark.parametrize('attached, silent, armed, want', [
    (False, 99.0, True, 'none'),            # never attached: a CLI verb, ignore
    (True, 1.0, True, 'none'),              # beating
    (True, 3.5, True, 'surface'),           # gone, armed
    (True, 3.5, False, 'detach'),           # gone, disarmed: nothing to surface
])
def test_the_watchdog_rule(attached, silent, armed, want):
    now = 100.0
    assert node_support.mission_watchdog(attached, now - silent, now, armed) == want


# ── the heartbeat gate ───────────────────────────────────────────────────────
def _beats(alive_fn, seconds=0.35):
    sent = []
    hb = node_support._HeartbeatThread(lambda: sent.append(time.monotonic()), period_s=0.05,
                            log=MagicMock(), alive_fn=alive_fn).start()
    time.sleep(seconds)
    hb.stop()
    return len(sent), hb


def test_a_stuck_executor_WITHHOLDS_the_heartbeat():
    """Falsified by any beat: a hung manager must let the board's failsafe fire."""
    n, hb = _beats(lambda: False)
    assert n == 0
    hb._log.error.assert_called_once()        # said once, not every period


def test_a_live_executor_keeps_beating():
    n, _ = _beats(lambda: True)
    assert n >= 4


def test_a_failing_liveness_probe_keeps_beating():
    """Unknown is not dead: a probe bug must not surface a healthy vehicle."""
    def boom():
        raise RuntimeError('probe bug')
    n, _ = _beats(boom)
    assert n >= 4


def test_bring_up_before_the_executor_spins_still_beats():
    node = SimpleNamespace()
    assert M.AUVManagerNode._executor_alive(node) is True     # never stamped
    node._executor_alive_t = time.monotonic() - 10.0
    assert M.AUVManagerNode._executor_alive(node) is False    # stuck
    node._executor_alive_t = time.monotonic()
    assert M.AUVManagerNode._executor_alive(node) is True


# ── the watchdog, acting ─────────────────────────────────────────────────────
def _node(armed, surface_heard=None):
    """`armed` is what a fresh heartbeat says (True / False / None = unknown).
    `surface_heard` is the (armed, mode) a heartbeat heard after the first
    SURFACE request shows; None scripts a board that never answers."""
    n = SimpleNamespace()
    n._mission = {'attached': True, 'name': 'pool_day_practice',
                  'seen': time.monotonic() - 10.0}
    n.kill = None
    n.heard = surface_heard
    n.fc = SimpleNamespace(
        armed_state=lambda: n.armed, is_armed=lambda: bool(n.armed),
        telemetry=lambda: SimpleNamespace(kill_switch=n.kill),
        armed_heard_since=lambda t: n.heard[0] if n.heard else None,
        mode_heard_since=lambda t: n.heard[1] if n.heard else None)
    n.armed = armed
    n.mongla = MagicMock()
    n.mongla.surface.return_value = SimpleNamespace(message='SURFACE engaged')
    n.mongla.disarm.return_value = SimpleNamespace(success=False,
                                                   message='UNCONFIRMED')
    n.get_logger = lambda: MagicMock()
    # A manager that came up on a DISARMED board; the boot tests set 'pending'.
    n._boot_latch = None
    for name in ('_kill_watch_tick', '_recovery_tick', '_request_surface',
                 '_heard_since', '_armed_now', '_executor_alive', '_boot_tick',
                 '_heartbeat_allowed', 'safety_latch_reason'):
        setattr(n, name, getattr(M.AUVManagerNode, name).__get__(n))
    return n


def _tick(n, at):
    """One watchdog tick at monotonic time `at` (the rules take `now`)."""
    real = time.monotonic
    try:
        time.monotonic = lambda: at
        M.AUVManagerNode._mission_watchdog_tick(n)
    finally:
        time.monotonic = real


def test_a_dead_mission_on_an_armed_vehicle_is_aborted_and_SURFACED():
    n = _node(armed=True)
    M.AUVManagerNode._mission_watchdog_tick(n)
    n.mongla.request_abort.assert_called_once()
    # wait=False: the watchdog runs on a timer and must not block it
    n.mongla.surface.assert_called_once_with(wait=False)
    assert n._mission['attached'] is False
    assert n._recovery is not None, 'the recovery is latched, not forgotten'


def test_a_dead_mission_on_a_disarmed_vehicle_is_only_detached():
    n = _node(armed=False)
    M.AUVManagerNode._mission_watchdog_tick(n)
    n.mongla.surface.assert_not_called()
    assert n._mission['attached'] is False


def test_unknown_armed_state_is_treated_as_armed():
    n = _node(armed=None)
    M.AUVManagerNode._mission_watchdog_tick(n)
    n.mongla.surface.assert_called_once_with(wait=False)


def test_an_unanswered_SURFACE_is_retried_then_the_heartbeat_is_withheld():
    """Issue #72: one SURFACE, then nothing, while the heartbeat kept the
    board's own failsafe from firing."""
    n = _node(armed=True)
    t0 = time.monotonic()
    _tick(n, t0)
    assert n.mongla.surface.call_count == 1 and n._heartbeat_allowed()
    _tick(n, t0 + 1.0)                       # inside the retry gap
    assert n.mongla.surface.call_count == 1
    _tick(n, t0 + 2.1)
    _tick(n, t0 + 4.2)
    assert n.mongla.surface.call_count == 3, 'bounded retries'
    assert n._heartbeat_allowed()
    _tick(n, t0 + 6.3)
    assert n.mongla.surface.call_count == 3
    assert not n._heartbeat_allowed(), "then silence, so the board's failsafe fires"
    assert n.safety_latch_reason()


def test_a_confirmed_SURFACE_resolves_the_recovery_and_resumes_the_heartbeat():
    n = _node(armed=True)
    t0 = time.monotonic()
    _tick(n, t0)
    for k in (1, 2, 3):
        _tick(n, t0 + 2.1 * k)
    assert not n._heartbeat_allowed()
    n.heard = (True, 'SURFACE')
    _tick(n, t0 + 9.0)
    assert n._recovery is None and n._heartbeat_allowed()
    assert n.safety_latch_reason() == ''


def test_a_SURFACE_request_that_raises_is_retried():
    n = _node(armed=True)
    n.mongla.surface.side_effect = RuntimeError('USB gone')
    t0 = time.monotonic()
    _tick(n, t0)
    _tick(n, t0 + 2.1)
    assert n.mongla.surface.call_count == 2


def test_an_empty_beat_is_a_clean_detach():
    n = _node(armed=True)
    M.AUVManagerNode._on_mission_alive(n, SimpleNamespace(data=''))
    assert n._mission['attached'] is False
    M.AUVManagerNode._mission_watchdog_tick(n)
    n.mongla.surface.assert_not_called()


# ── the runner's side ────────────────────────────────────────────────────────
def test_the_runner_beats_its_name_and_detaches_on_stop():
    from mongla_planner.mission import MissionKeepAlive
    pub = MagicMock()
    node = MagicMock()
    node.create_publisher.return_value = pub
    k = MissionKeepAlive(node, 'demo').start()
    time.sleep(1.2)
    k.stop()
    datas = [c.args[0].data for c in pub.publish.call_args_list]
    assert datas.count('demo') >= 2
    assert datas[-1] == '' and 'demo' not in datas[-3:], 'detach must come last'


# ── the kill switch: disarm, and hold it until the board confirms ───────────
@pytest.mark.parametrize('killed, armed, want', [
    (True, True, 'disarm'),        # turned while armed: act
    (True, None, 'disarm'),        # armed state unknown: act anyway
    (True, False, 'none'),         # disarmed already: nothing to do
    (None, True, 'none'),          # link lost: says nothing about the switch
    (False, True, 'none'),         # power on: nothing to do
])
def test_the_kill_rule_from_clear(killed, armed, want):
    latch, action = node_support.kill_latch(None, killed, armed, now=10.0)
    assert action == want and (latch is not None) == (want == 'disarm')


def test_the_kill_latch_holds_until_the_board_says_disarmed():
    latch, _ = node_support.kill_latch(None, True, True, now=0.0)
    # still armed, inside the retry gap: wait
    assert node_support.kill_latch(latch, True, True, now=1.0)[1] == 'none'
    # the knob turned back ON, board still armed: keep disarming
    latch, action = node_support.kill_latch(latch, False, True, now=3.1)
    assert action == 'disarm' and latch['tries'] == 2
    # link quiet: unknown is not disarmed
    assert node_support.kill_latch(latch, None, None, now=4.0)[0] is not None
    # a fresh heartbeat says DISARMED: resolved
    assert node_support.kill_latch(latch, True, False, now=5.0) == (None, 'resolved')


def test_kill_retries_never_stop_and_the_last_bounded_one_withholds():
    """Issue #98: after KILL_MAX_TRIES the old rule went quiet for good while
    the heartbeat stayed healthy. Now that try withholds the heartbeat, once,
    and the disarm keeps being re-sent."""
    latch, action = node_support.kill_latch(None, True, True, now=0.0)
    actions = [action]
    t = 0.0
    for _ in range(node_support.KILL_MAX_TRIES + 3):
        t += node_support.KILL_RETRY_S
        latch, action = node_support.kill_latch(latch, True, True, now=t)
        actions.append(action)
    n = node_support.KILL_MAX_TRIES
    assert actions[:n - 1] == ['disarm'] * (n - 1)
    assert actions[n - 1] == 'withhold'
    assert actions[n:] == ['disarm'] * (len(actions) - n), 'retries never stop'
    assert latch['withholding'] is True


def test_a_failed_kill_disarm_is_retried_on_a_later_tick():
    """Issue #71: clear -> engaged -> failed disarm -> still engaged ->
    restored. The first version disarmed exactly once and forgot."""
    n = _node(armed=True)
    n._mission['attached'] = False
    t0 = time.monotonic()
    n.kill = False
    _tick(n, t0)
    n.mongla.disarm.assert_not_called()
    n.kill = True
    _tick(n, t0 + 0.5)
    _tick(n, t0 + 1.0)
    assert n.mongla.disarm.call_count == 1
    _tick(n, t0 + 3.6)                       # still armed: try again
    assert n.mongla.disarm.call_count == 2
    n.kill = False                           # power restored, board still armed
    _tick(n, t0 + 6.7)
    assert n.mongla.disarm.call_count == 3
    assert n.safety_latch_reason(), 'ordinary verbs stay refused'
    n.armed = False                          # the board confirms
    _tick(n, t0 + 7.0)
    assert n._kill_latch is None and n.safety_latch_reason() == ''


def test_no_second_board_never_disarms():
    n = _node(armed=True)
    n._mission['attached'] = False
    for _ in range(3):
        M.AUVManagerNode._mission_watchdog_tick(n)
    n.mongla.disarm.assert_not_called()


def test_an_unconfirmed_kill_disarm_ends_in_a_silent_heartbeat():
    """Issue #98, at the node: kill while armed, every DISARM refused. Within
    the bounded tries the heartbeat gate closes, so the board's link-loss
    failsafe ends the move; it reopens only on a confirmed DISARMED."""
    n = _node(armed=True)
    n._mission['attached'] = False
    n.kill = True
    t0 = time.monotonic()
    _tick(n, t0)
    assert n._heartbeat_allowed()
    for k in range(1, node_support.KILL_MAX_TRIES):
        _tick(n, t0 + k * (node_support.KILL_RETRY_S + 0.1))
    assert not n._heartbeat_allowed(), 'the heartbeat keeps the failsafe silent'
    calls = n.mongla.disarm.call_count
    _tick(n, t0 + 60.0)
    assert n.mongla.disarm.call_count == calls + 1, 'DISARM is still re-sent'
    n.armed = False
    _tick(n, t0 + 61.0)
    assert n._heartbeat_allowed() and n.safety_latch_reason() == ''


@pytest.mark.parametrize('heard, want_latch, want', [
    (None, 'pending', 'none'),      # nothing heard yet: no verdict
    (False, None, 'none'),          # disarmed at start-up: clear
    (True, 'armed', 'latched'),     # armed at start-up: latch
])
def test_the_boot_rule(heard, want_latch, want):
    assert node_support.boot_latch('pending', heard) == (want_latch, want)


def test_a_restart_on_an_armed_board_refuses_ordinary_verbs_until_disarmed():
    """Issue #98: the latches lived in memory, so restarting the manager
    cleared them while the board could still be armed with a move running.
    A board heard ARMED at start-up now latches the new manager."""
    n = _node(armed=True)
    n._mission['attached'] = False
    n._boot_latch = 'pending'
    _tick(n, time.monotonic())
    assert 'ARMED when this manager started' in n.safety_latch_reason()
    _tick(n, time.monotonic() + 30.0)
    assert n.safety_latch_reason(), 'time alone never clears it'
    n.armed = False
    _tick(n, time.monotonic() + 31.0)
    assert n.safety_latch_reason() == '' and n._boot_latch is None
    n.armed = True                    # a deliberate arm after that is not a restart
    _tick(n, time.monotonic() + 32.0)
    assert n.safety_latch_reason() == ''


def test_our_own_arm_before_the_first_tick_is_not_read_as_a_restart():
    """The end-to-end run caught this: an arm sent before the first watchdog
    tick made the next tick see ARMED and latch the manager against its own
    arm. The goal path decides the boot latch from the board first."""
    n = _node(armed=False)
    n._mission['attached'] = False
    n._boot_latch = 'pending'
    assert n.safety_latch_reason() == '' and n._boot_latch is None
    n.armed = True                    # the arm we just allowed
    _tick(n, time.monotonic())
    assert n.safety_latch_reason() == ''


def test_no_heartbeat_yet_refuses_rather_than_guesses():
    n = _node(armed=None)
    n._boot_latch = 'pending'
    assert 'no heartbeat' in n.safety_latch_reason()

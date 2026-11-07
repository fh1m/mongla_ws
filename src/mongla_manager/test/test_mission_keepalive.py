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
def _node(armed):
    n = SimpleNamespace()
    n._mission = {'attached': True, 'name': 'pool_day_practice',
                  'seen': time.monotonic() - 10.0}
    n.kill = None
    n.fc = SimpleNamespace(is_armed=lambda: armed,
                           telemetry=lambda: SimpleNamespace(kill_switch=n.kill))
    n.mongla = MagicMock()
    n.mongla.surface.return_value = SimpleNamespace(message='SURFACE engaged')
    n.get_logger = lambda: MagicMock()
    n._kill_watch_tick = lambda armed: M.AUVManagerNode._kill_watch_tick(n, armed)
    return n


def test_a_dead_mission_on_an_armed_vehicle_is_aborted_and_SURFACED():
    n = _node(armed=True)
    M.AUVManagerNode._mission_watchdog_tick(n)
    n.mongla.request_abort.assert_called_once()
    # wait=False: the watchdog runs on a timer and must not block it
    n.mongla.surface.assert_called_once_with(wait=False)
    assert n._mission['attached'] is False, 'it must act once, not every tick'


def test_a_dead_mission_on_a_disarmed_vehicle_is_only_detached():
    n = _node(armed=False)
    M.AUVManagerNode._mission_watchdog_tick(n)
    n.mongla.surface.assert_not_called()
    assert n._mission['attached'] is False


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


# ── the kill switch: disarm on the edge to ENGAGED ───────────────────────────
@pytest.mark.parametrize('was, now, armed, want', [
    (False, True, True, True),     # turned while armed: act
    (None, True, True, True),      # first reading is engaged: act
    (True, True, True, False),     # still engaged: acted already
    (False, True, False, False),   # disarmed: nothing to do
    (False, None, True, False),    # link lost: says nothing about the switch
    (True, False, True, False),    # power back: never re-arm or resume
])
def test_the_kill_rule(was, now, armed, want):
    assert node_support.kill_watch(was, now, armed) is want


def test_engaging_the_kill_switch_disarms_once():
    n = _node(armed=True)
    n._mission['attached'] = False
    n.kill = False
    M.AUVManagerNode._mission_watchdog_tick(n)
    n.mongla.disarm.assert_not_called()
    n.kill = True
    M.AUVManagerNode._mission_watchdog_tick(n)
    M.AUVManagerNode._mission_watchdog_tick(n)
    n.mongla.disarm.assert_called_once_with(timeout=2.0)


def test_no_second_board_never_disarms():
    n = _node(armed=True)
    n._mission['attached'] = False
    for _ in range(3):
        M.AUVManagerNode._mission_watchdog_tick(n)
    n.mongla.disarm.assert_not_called()

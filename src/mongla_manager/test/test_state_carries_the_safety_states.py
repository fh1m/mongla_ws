"""Leak, kill and the barometer on /mongla/state, decoded once, tri-state.

A lamp that read the board's raw channels said "baro ok" over a code of 3
(not initialised) and "kill clear" over a lost second-board link, where the
firmware sends 0 on purpose (2026-11-24)."""
import math
from types import SimpleNamespace

from mongla_interfaces.msg import MonglaState
from mongla_manager import node_telemetry

U, C, X = MonglaState.SAFETY_UNKNOWN, MonglaState.SAFETY_CLEAR, MonglaState.SAFETY_TRIPPED


def _safety(**tel):
    return node_telemetry.safety_of(SimpleNamespace(telemetry=lambda: SimpleNamespace(**tel)))


def test_a_default_message_reads_unknown_never_clear():
    m = MonglaState()
    assert (m.leak, m.kill_switch, m.baro) == (U, U, U)


def test_a_lost_second_board_link_is_kill_unknown():
    assert _safety(leak=False, kill_switch=None, baro_health=0.0)[:3] == (C, U, C)


def test_baro_code_three_is_a_fault_not_a_score():
    leak, kill, baro, text = _safety(leak=True, kill_switch=True, baro_health=3.0)
    assert (leak, kill, baro) == (X, X, X) and 'not initialised' in text


def test_no_baro_report_is_unknown():
    assert _safety(leak=None, kill_switch=None, baro_health=math.nan) == (U, U, U, '')

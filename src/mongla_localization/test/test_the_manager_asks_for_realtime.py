"""The process that streams commands asks for a realtime priority, and says what it got.

measured-bars 76: beside a full-rate MAVLink reader with every core busy, a
50 Hz loop wakes at worst 5.17 ms late by default and 0.56 ms late with
SCHED_FIFO and a 0.5 ms switch interval. Neither setting is worth anything if
the manager stops asking, or asks after its threads exist.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

from mongla_localization import realtime

MANAGER = (Path(__file__).resolve().parents[2]
           / 'mongla_manager/mongla_manager/auv_manager_node.py')
KUTUM = Path(__file__).resolve().parents[3] / 'kutum'


@pytest.fixture(autouse=True)
def _restore_switch_interval():
    old = sys.getswitchinterval()
    yield
    sys.setswitchinterval(old)


class _Log:
    def __init__(self):
        self.warned = []

    def warning(self, msg):
        self.warned.append(msg)


def test_it_sets_the_priority_and_the_switch_interval():
    calls = []
    took = realtime.apply(_Log(), env={}, setscheduler=lambda *a: calls.append(a))
    assert calls and calls[0][0] == 0 and calls[0][1] == os.SCHED_FIFO
    assert calls[0][2].sched_priority == realtime.PRIORITY
    assert sys.getswitchinterval() == pytest.approx(realtime.SWITCH_INTERVAL_S)
    assert 'SCHED_FIFO' in took


def test_a_machine_that_refuses_keeps_the_short_interval_and_says_so():
    def refuse(*_a):
        raise PermissionError(1, 'Operation not permitted')
    log = _Log()
    took = realtime.apply(log, env={}, setscheduler=refuse)
    assert sys.getswitchinterval() == pytest.approx(realtime.SWITCH_INTERVAL_S)
    assert 'not permitted' in took
    assert log.warned and '99-mongla-realtime.conf' in log.warned[0]


def test_off_changes_nothing():
    before = sys.getswitchinterval()
    calls = []
    took = realtime.apply(_Log(), env={realtime.ENV: 'off'},
                          setscheduler=lambda *a: calls.append(a))
    assert not calls and sys.getswitchinterval() == before and took.startswith('off')


def test_a_misspelt_choice_is_refused():
    with pytest.raises(ValueError):
        realtime.apply(_Log(), env={realtime.ENV: 'yes'})


def test_the_priority_stays_under_the_kernels_interrupt_threads():
    assert 1 <= realtime.PRIORITY < 50


def test_the_manager_asks_before_it_builds_the_node():
    text = MANAGER.read_text()
    main = text[text.index('def main('):]
    assert 'realtime.apply(' in main, 'the manager no longer asks for realtime'
    assert main.index('realtime.apply(') < main.index('AUVManagerNode()'), (
        'threads inherit the scheduling class of their creator: ask before '
        'the node starts its reader, heartbeat and executor threads')


def test_kutum_permits_what_the_manager_asks_for():
    conf = (KUTUM / 'system/limits/99-mongla-realtime.conf').read_text()
    row = [ln.split() for ln in conf.splitlines()
           if ln.strip() and not ln.startswith('#')]
    assert row and row[0][2] == 'rtprio' and int(row[0][3]) >= realtime.PRIORITY
    assert '99-mongla-realtime.conf' in (KUTUM / 'setup.sh').read_text()
    assert 'ulimit -r' in (KUTUM / 'doctor.sh').read_text()

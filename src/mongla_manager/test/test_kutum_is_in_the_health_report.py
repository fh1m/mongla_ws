"""Kutum, the Pi and its Hailo-8, is in the live health report as one group of rows.

Before this, the computer running every mission was graded once, at the
preflight, and never again while the hull was wet. The rows are report-only;
what this checks is that each one reads its number honestly, agrees with the
preflight about the same reading, and can never block the executor.
"""
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_manager import kutum_health as hh  # noqa: E402
from mongla_manager.bringup_grade import FAIL, PASS, WARN  # noqa: E402
from mongla_manager.bringup_host import _disk_verdict, _throttled_verdict  # noqa: E402
from mongla_manager.health import HealthBoard, State  # noqa: E402

GRADE = {PASS: State.OK, WARN: State.DEGRADED, FAIL: State.FAILED}
# What Kutum's chip reported on 2026-11-10, idle: green, no throttling level,
# orange at 104 C and red at 120 C, NN clock requested at 400 MHz.
GREEN = hh.HailoReading(39.2, zone=0, level=-1, orange_c=104.0, red_c=120.0,
                        clock_hz=400_000_000, overcurrent=False)


@pytest.mark.parametrize('c,state', [(45.0, State.OK), (79.9, State.OK),
                                     (80.0, State.DEGRADED), (84.9, State.DEGRADED),
                                     (85.0, State.FAILED), (91.0, State.FAILED)])
def test_temperature_is_graded_at_the_firmware_throttle_points(c, state):
    assert hh.pi_temperature(c).state is state


@pytest.mark.parametrize('raw', [0x0, 0x50000, 0x80000, 0x1, 0x4, 0xE0000])
def test_throttling_reads_exactly_as_the_preflight_does(raw):
    assert hh.pi_power(float(raw)).state is GRADE[_throttled_verdict(raw)[0]]


@pytest.mark.parametrize('gb', [0.2, 0.99, 1.0, 2.9, 3.0, 40.0])
def test_disk_reads_exactly_as_the_preflight_does(gb):
    assert hh.disk(gb).state is GRADE[_disk_verdict(gb)[0]]


@pytest.mark.parametrize('row', [hh.pi_temperature, hh.pi_power, hh.disk, hh.hailo])
def test_no_reading_is_unknown_never_ok(row):
    assert row(None, 'FileNotFoundError: vcgencmd').state is State.UNKNOWN
    assert 'vcgencmd' in row(None, 'FileNotFoundError: vcgencmd').evidence
    assert row(None, '', float('inf')).state is State.UNKNOWN
    # A good number from a sampler that has stopped is not good news.
    stale = row(GREEN if row is hh.hailo else 10.0, '', hh.STALE_S + 1)
    assert stale.state is State.UNKNOWN and 'stopped' in stale.evidence


def test_a_probe_that_raises_is_cached_as_missing_with_its_reason():
    def broken():
        raise OSError('no such zone')
    s = hh.HostSampler({'kutum.pi_temperature': broken, 'kutum.disk': lambda: 12.0})
    s.sample_once()
    value, why, age = s.get('kutum.pi_temperature')
    assert value is None and 'no such zone' in why and age < 1.0
    assert s.get('kutum.disk')[0] == 12.0


def test_a_slow_probe_never_blocks_the_reporter(monkeypatch):
    """The reporter runs in the health tick, on the executor. A vcgencmd that
    takes its whole timeout must cost the tick nothing. Goes through the same
    `register` the manager calls, so wiring a probe into the row fails here."""
    release = threading.Event()

    def slow():
        release.wait(5.0)
        return 40.0
    monkeypatch.setattr(hh, 'read_disk_free_gb', slow)
    board = HealthBoard()
    s = hh.register(board, is_pi=False, has_hailo=False)
    try:
        t0 = time.monotonic()
        rows = board.poll()
        assert time.monotonic() - t0 < 0.1
        assert rows['kutum.disk'].state is State.UNKNOWN
    finally:
        release.set()
        s.stop()


def test_each_row_appears_only_where_its_hardware_is():
    """On a development box a Pi's thermal grade means nothing, and a row that
    is UNKNOWN forever would hide the rows that do."""
    def rows(**kw):
        board = HealthBoard()
        s = hh.register(board, **kw)
        s.stop()
        return set(board.poll())
    assert rows(is_pi=False, has_hailo=False) == {'kutum.disk'}
    assert rows(is_pi=True, has_hailo=False) == {'kutum.disk', 'kutum.pi_temperature',
                                                 'kutum.pi_power'}
    assert rows(is_pi=True, has_hailo=True) == {'kutum.disk', 'kutum.pi_temperature',
                                                'kutum.pi_power', 'kutum.hailo'}


def test_the_manager_registers_the_rows():
    from mongla_manager import node_health
    src = Path(node_health.__file__).read_text(encoding='utf-8')
    assert '_kutum.register(self._health' in src


def test_the_hailo_is_graded_by_its_own_firmware():
    from dataclasses import replace
    assert hh.hailo(GREEN).state is State.OK
    assert '104' in hh.hailo(GREEN).evidence
    # The chip says orange, or has started throttling its clock.
    assert hh.hailo(replace(GREEN, zone=1)).state is State.DEGRADED
    slowed = hh.hailo(replace(GREEN, level=0, clock_hz=350_000_000))
    assert slowed.state is State.DEGRADED and '350 MHz' in slowed.evidence
    assert hh.hailo(replace(GREEN, zone=2)).state is State.FAILED
    assert hh.hailo(replace(GREEN, overcurrent=True)).state is State.FAILED


def test_a_hailo_temperature_past_its_own_limit_is_believed_over_the_zone():
    from dataclasses import replace
    assert hh.hailo(replace(GREEN, temp_c=105.0)).state is State.DEGRADED
    assert hh.hailo(replace(GREEN, temp_c=121.0)).state is State.FAILED


def test_a_hailo_with_no_zones_is_never_ok():
    """If a later HailoRT drops the private health call, the temperature is
    still reported, but nothing graded it, so it cannot read as fine."""
    h = hh.hailo(hh.HailoReading(60.0))
    assert h.state is State.DEGRADED and '60 C' in h.evidence

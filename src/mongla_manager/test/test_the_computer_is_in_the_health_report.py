"""The Pi's temperature, throttling and disk are rows in the live health report.

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

from mongla_manager import host_health as hh  # noqa: E402
from mongla_manager.bringup_grade import FAIL, PASS, WARN  # noqa: E402
from mongla_manager.bringup_host import _disk_verdict, _throttled_verdict  # noqa: E402
from mongla_manager.health import HealthBoard, State  # noqa: E402

GRADE = {PASS: State.OK, WARN: State.DEGRADED, FAIL: State.FAILED}


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
    assert hh.host_disk(gb).state is GRADE[_disk_verdict(gb)[0]]


@pytest.mark.parametrize('row', [hh.pi_temperature, hh.pi_power, hh.host_disk])
def test_no_reading_is_unknown_never_ok(row):
    assert row(None, 'FileNotFoundError: vcgencmd').state is State.UNKNOWN
    assert 'vcgencmd' in row(None, 'FileNotFoundError: vcgencmd').evidence
    assert row(None, '', float('inf')).state is State.UNKNOWN
    # A good number from a sampler that has stopped is not good news.
    stale = row(10.0, '', hh.STALE_S + 1)
    assert stale.state is State.UNKNOWN and 'stopped' in stale.evidence


def test_a_probe_that_raises_is_cached_as_missing_with_its_reason():
    def broken():
        raise OSError('no such zone')
    s = hh.HostSampler({'pi_temperature': broken, 'host_disk': lambda: 12.0})
    s.sample_once()
    value, why, age = s.get('pi_temperature')
    assert value is None and 'no such zone' in why and age < 1.0
    assert s.get('host_disk')[0] == 12.0


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
    s = hh.register(board, is_pi=False)
    try:
        t0 = time.monotonic()
        rows = board.poll()
        assert time.monotonic() - t0 < 0.1
        assert rows['host_disk'].state is State.UNKNOWN
    finally:
        release.set()
        s.stop()


def test_pi_rows_appear_only_on_a_pi():
    """On a development box a Pi's thermal grade means nothing, and a row that
    is UNKNOWN forever would hide the rows that do."""
    off, on = HealthBoard(), HealthBoard()
    a = hh.register(off, is_pi=False)
    b = hh.register(on, is_pi=True)
    try:
        assert set(off.poll()) == {'host_disk'}
        assert set(on.poll()) == {'host_disk', 'pi_temperature', 'pi_power'}
    finally:
        a.stop()
        b.stop()


def test_the_manager_registers_the_rows():
    from mongla_manager import node_health
    src = Path(node_health.__file__).read_text(encoding='utf-8')
    assert '_host.register(self._health' in src

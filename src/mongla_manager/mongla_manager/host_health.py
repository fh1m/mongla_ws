"""The computer's own health in the live report: temperature, throttling and free disk.

The health board watched the board, the link and vision, and nothing watched
the Pi running them. A Pi throttling at 85 C halves the detector rate, and a
full disk stops the bag mid-run; before this, the only place either was read
was the preflight (`bringup_check`), once, before the hull was wet.

Report-only, by the owner's decision: these rows tell the operator, and
nothing acts on them. A computer that is hot is still the only computer.

Sampled on its own thread, never on the executor. `vcgencmd` may take its whole
3 s timeout, and 3 s is `EXECUTOR_STALE_S`: read in the health tick, one slow
reading would withhold the heartbeat and let the board's link-loss failsafe
surface the hull over a temperature. The reporters read the cache and say
UNKNOWN when the sample is old, so a sampler that dies cannot leave good news.

The grades are `bringup_host`'s, imported, so the preflight and the live report
cannot disagree about the same disk or the same bitmask.
"""
from __future__ import annotations

import threading
import time
from typing import Callable, Dict, Optional, Tuple

from .bringup_grade import FAIL, PASS
from .bringup_host import _disk_verdict, _throttled_verdict, read_disk_free_gb, read_throttled
from .health import Health, degraded, failed, ok, unknown

SAMPLE_S = 10.0
# Three missed samples, so one slow vcgencmd is not a fault.
STALE_S = 3 * SAMPLE_S
THERMAL_ZONE = '/sys/class/thermal/thermal_zone0/temp'
# Raspberry Pi firmware begins throttling the cores at 80 C and throttles
# further at its 85 C maximum (raspberrypi.com, "Heating and cooling
# Raspberry Pi 5"). These are the vendor's numbers, not ours to tune.
PI_SOFT_THROTTLE_C = 80.0
PI_MAX_C = 85.0


def read_temp_c(zone: str = THERMAL_ZONE) -> float:
    with open(zone, encoding='ascii') as f:
        return int(f.read().strip()) / 1000.0


class HostSampler:
    """Reads each probe every `period_s` on a daemon thread and caches it.

    A probe that raises caches None with the reason, so a broken probe is a
    fresh UNKNOWN, not the last good reading standing.
    """

    def __init__(self, probes: Dict[str, Callable[[], float]],
                 period_s: float = SAMPLE_S):
        self._probes = dict(probes)
        self._period_s = float(period_s)
        self._lock = threading.Lock()
        self._last: Dict[str, Tuple[Optional[float], str, float]] = {}
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def sample_once(self) -> None:
        for name, probe in self._probes.items():
            try:
                value, why = float(probe()), ''
            except Exception as exc:                    # noqa: BLE001
                value, why = None, f'{type(exc).__name__}: {exc}'
            with self._lock:
                self._last[name] = (value, why, time.monotonic())

    def start(self) -> 'HostSampler':
        def loop():
            while not self._stop.is_set():
                self.sample_once()
                self._stop.wait(self._period_s)
        self._thread = threading.Thread(target=loop, name='host_health', daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()

    def get(self, name: str) -> Tuple[Optional[float], str, float]:
        """(value, why it is missing, age in s); age is inf before the first sample."""
        with self._lock:
            value, why, at = self._last.get(name, (None, 'not sampled yet', 0.0))
        return value, why, (float('inf') if at <= 0.0 else time.monotonic() - at)


def _missing(name: str, why: str, age_s: float) -> Health:
    if age_s == float('inf'):
        return unknown(name, 'not sampled yet')
    if age_s > STALE_S:
        return unknown(name, f'last sample {age_s:.0f}s old -- the sampler stopped')
    return unknown(name, why or 'no reading')


def _from_grade(name: str, grade: str, evidence: str) -> Health:
    return (ok if grade == PASS else failed if grade == FAIL else degraded)(name, evidence)


def pi_temperature(value: Optional[float], why: str = '', age_s: float = 0.0) -> Health:
    if value is None or age_s > STALE_S:
        return _missing('pi_temperature', why, age_s)
    if value >= PI_MAX_C:
        return failed('pi_temperature', f'{value:.0f} C -- at the firmware maximum; '
                      f'cores and GPU throttled, detector rate will not hold')
    if value >= PI_SOFT_THROTTLE_C:
        return degraded('pi_temperature', f'{value:.0f} C -- firmware is throttling the cores')
    return ok('pi_temperature', f'{value:.0f} C')


def pi_power(value: Optional[float], why: str = '', age_s: float = 0.0) -> Health:
    if value is None or age_s > STALE_S:
        return _missing('pi_power', why, age_s)
    return _from_grade('pi_power', *_throttled_verdict(int(value)))


def host_disk(value: Optional[float], why: str = '', age_s: float = 0.0) -> Health:
    if value is None or age_s > STALE_S:
        return _missing('host_disk', why, age_s)
    return _from_grade('host_disk', *_disk_verdict(value))


def register(board, is_pi: bool) -> HostSampler:
    """Start the sampler and put its rows on `board`.

    Temperature and throttling are graded against a Pi's firmware, so they are
    registered only on a Pi; on a development box they would be a permanent
    UNKNOWN that says nothing about the vehicle.
    """
    probes: Dict[str, Callable[[], float]] = {'host_disk': read_disk_free_gb}
    rows = {'host_disk': host_disk}
    if is_pi:
        probes.update(pi_temperature=read_temp_c, pi_power=read_throttled)
        rows.update(pi_temperature=pi_temperature, pi_power=pi_power)
    sampler = HostSampler(probes).start()
    for name, row in rows.items():
        board.register(name, lambda row=row, name=name: row(*sampler.get(name)))
    return sampler

"""Kutum in the live health report: the Pi and the Hailo-8 on it, as one machine.

The health board watched the board, the link and vision, and nothing watched
the computer running them. A Pi throttling at 85 C halves the detector rate;
a Hailo-8 in its orange zone slows its own clock from 400 to 350 MHz and the
detector rate drops with it; a full disk stops the bag mid-run. Before this,
the only place any of them was read was the preflight, once, before the hull
was wet. Kutum is the Pi and the chip together, so the rows sit together:

    kutum.pi_temperature   the SoC, graded at the Pi firmware's 80 and 85 C
    kutum.pi_power         `vcgencmd get_throttled`, graded as the preflight does
    kutum.hailo            the chip's two die sensors, graded by the zones and
                           throttling levels the chip's own firmware reports
    kutum.disk             free space, graded as the preflight does

Report-only, by the owner's decision: these rows tell the operator, and
nothing acts on them. A computer that is hot is still the only computer.

Sampled on its own thread, never on the executor. `vcgencmd` may take its whole
3 s timeout, and 3 s is `EXECUTOR_STALE_S`: read in the health tick, one slow
reading would withhold the heartbeat and let the board's link-loss failsafe
surface the hull over a temperature. The reporters read the cache and say
UNKNOWN when the sample is old, so a sampler that dies cannot leave good news.

The Hailo is read through a control `Device` opened for each sample and
released at once; the control channel works while the detector holds the chip
(measured-bars 70).
"""
from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional, Tuple

from .bringup_grade import FAIL, PASS
from .bringup_host import _disk_verdict, _throttled_verdict, read_disk_free_gb, read_throttled
from .health import Health, degraded, failed, ok, unknown

SAMPLE_S = 10.0
# Three missed samples, so one slow vcgencmd is not a fault.
STALE_S = 3 * SAMPLE_S
THERMAL_ZONE = '/sys/class/thermal/thermal_zone0/temp'
HAILO_DEV = '/dev/hailo0'
# Raspberry Pi firmware begins throttling the cores at 80 C and throttles
# further at its 85 C maximum (raspberrypi.com, "Heating and cooling
# Raspberry Pi 5"). These are the vendor's numbers, not ours to tune.
PI_SOFT_THROTTLE_C = 80.0
PI_MAX_C = 85.0


def read_temp_c(zone: str = THERMAL_ZONE) -> float:
    with open(zone, encoding='ascii') as f:
        return int(f.read().strip()) / 1000.0


@dataclass(frozen=True)
class HailoReading:
    """What the chip says about itself. Thresholds come from the chip, not here."""
    temp_c: float                       # the hotter of the two die sensors
    zone: Optional[int] = None          # 0 green, 1 orange, 2 red
    level: Optional[int] = None         # throttling level; -1 is none
    orange_c: Optional[float] = None
    red_c: Optional[float] = None
    clock_hz: Optional[int] = None      # the NN clock the firmware is asking for
    overcurrent: Optional[bool] = None


def read_hailo() -> HailoReading:
    import hailo_platform as hp  # type: ignore  # only on Kutum; the thread pays the import
    dev = hp.Device()
    try:
        t = dev.control.get_chip_temperature()
        temp = max(float(t.ts0_temperature), float(t.ts1_temperature))
        try:
            # Private in HailoRT 4.24 (the pinned version). If a later version
            # drops it, the row still reports the temperature, against no zone.
            h = dev.control._get_health_information()
        except Exception:                                   # noqa: BLE001
            return HailoReading(temp)
        return HailoReading(temp, int(h.current_temperature_zone),
                            int(h.current_temperature_throttling_level),
                            float(h.orange_temperature_threshold),
                            float(h.red_temperature_threshold),
                            int(h.requested_temperature_clock_freq),
                            bool(h.overcurrent_protection_active))
    finally:
        dev.release()


class HostSampler:
    """Reads each probe every `period_s` on a daemon thread and caches it.

    A probe that raises caches None with the reason, so a broken probe is a
    fresh UNKNOWN, not the last good reading standing.
    """

    def __init__(self, probes: Dict[str, Callable[[], Any]], period_s: float = SAMPLE_S):
        self._probes = dict(probes)
        self._period_s = float(period_s)
        self._lock = threading.Lock()
        self._last: Dict[str, Tuple[Any, str, float]] = {}
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def sample_once(self) -> None:
        for name, probe in self._probes.items():
            try:
                value, why = probe(), ''
            except Exception as exc:                    # noqa: BLE001
                value, why = None, f'{type(exc).__name__}: {exc}'
            with self._lock:
                self._last[name] = (value, why, time.monotonic())

    def start(self) -> 'HostSampler':
        def loop():
            while not self._stop.is_set():
                self.sample_once()
                self._stop.wait(self._period_s)
        self._thread = threading.Thread(target=loop, name='kutum_health', daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()

    def get(self, name: str) -> Tuple[Any, str, float]:
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
    name = 'kutum.pi_temperature'
    if value is None or age_s > STALE_S:
        return _missing(name, why, age_s)
    if value >= PI_MAX_C:
        return failed(name, f'{value:.0f} C -- at the firmware maximum; '
                      f'cores and GPU throttled, detector rate will not hold')
    if value >= PI_SOFT_THROTTLE_C:
        return degraded(name, f'{value:.0f} C -- firmware is throttling the cores')
    return ok(name, f'{value:.0f} C')


def pi_power(value: Optional[int], why: str = '', age_s: float = 0.0) -> Health:
    name = 'kutum.pi_power'
    if value is None or age_s > STALE_S:
        return _missing(name, why, age_s)
    return _from_grade(name, *_throttled_verdict(int(value)))


def hailo(value: Optional[HailoReading], why: str = '', age_s: float = 0.0) -> Health:
    """Graded by the chip's own firmware: its zone, its throttling level and
    its thresholds. The temperature is checked against those thresholds too,
    so a zone field that reads 0 at 121 C is not believed."""
    name = 'kutum.hailo'
    if value is None or age_s > STALE_S:
        return _missing(name, why, age_s)
    r, t = value, f'{value.temp_c:.0f} C'
    if r.zone is None:
        return degraded(name, f'{t}; zones unreadable on this HailoRT, so not graded')
    clock = f', NN clock {r.clock_hz / 1e6:.0f} MHz' if r.clock_hz else ''
    if r.overcurrent:
        return failed(name, f'{t}; OVERCURRENT protection active{clock}')
    if r.zone >= 2 or (r.red_c is not None and r.temp_c >= r.red_c):
        return failed(name, f'{t}; RED zone (chip limit {r.red_c:.0f} C){clock}')
    if r.zone == 1 or (r.level is not None and r.level >= 0) or (
            r.orange_c is not None and r.temp_c >= r.orange_c):
        return degraded(name, f'{t}; orange zone, throttling level {r.level}{clock} '
                        f'-- the detector rate drops with the clock')
    return ok(name, f'{t}; green, throttles at {r.orange_c:.0f} C{clock}')


def disk(value: Optional[float], why: str = '', age_s: float = 0.0) -> Health:
    name = 'kutum.disk'
    if value is None or age_s > STALE_S:
        return _missing(name, why, age_s)
    return _from_grade(name, *_disk_verdict(value))


def register(board, is_pi: bool, has_hailo: Optional[bool] = None) -> HostSampler:
    """Start the sampler and put Kutum's rows on `board`.

    The Pi rows are graded against a Pi's firmware and the Hailo row against
    the chip, so each is registered only where that hardware is; on a
    development box they would be a permanent UNKNOWN that says nothing about
    the vehicle.
    """
    if has_hailo is None:
        has_hailo = os.path.exists(HAILO_DEV)
    probes: Dict[str, Callable[[], Any]] = {'kutum.disk': read_disk_free_gb}
    rows: Dict[str, Callable[..., Health]] = {'kutum.disk': disk}
    if is_pi:
        probes.update({'kutum.pi_temperature': read_temp_c, 'kutum.pi_power': read_throttled})
        rows.update({'kutum.pi_temperature': pi_temperature, 'kutum.pi_power': pi_power})
    if has_hailo:
        probes['kutum.hailo'] = read_hailo
        rows['kutum.hailo'] = hailo
    sampler = HostSampler(probes).start()
    for name, row in rows.items():
        board.register(name, lambda row=row, name=name: row(*sampler.get(name)))
    return sampler

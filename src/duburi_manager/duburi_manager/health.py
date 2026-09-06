"""One health surface. Four states, and UNKNOWN is not OK.

WHAT THIS FIXES. The vehicle already knows a great deal about itself and says
it in a dozen private dialects: `link_alive()` returns a bool, `esc_status_rpm()`
returns a list that is empty both when the thrusters are fine and when nothing
is listening, `BARO_HEALTH` is an enum where 0 is good and 3 means "not
initialised", `YAW_REF` is an enum where only 2 means the heading is absolute,
`check_for_reboot()` is an edge, and the lock ladder has an authority ramp.
Each is correct. Together they are unreadable, and no two of them degrade the
vehicle the same way.

⛔ THE RULE THAT EARNED THIS FILE: **absence is not zero.** It has cost this
project real time, repeatedly and always in the same direction -- a subsystem
that says nothing gets read as a subsystem that is fine:

  * `BARO_HEALTH = 3` was read as a health SCORE and taken for "healthy-ish".
    It is a fault code meaning the barometer never initialised.
  * `esc_msgs == 0` -- no telemetry at all -- would report every thruster
    turning perfectly if a bare list were trusted.
  * `VFR_HUD.throttle` is a hardcoded 0 on this firmware: a field that decodes
    cleanly, means nothing, and looks like an idle vehicle.
  * A pose that never arrives must NOT read as "square".

So there is no boolean here. A subsystem reports `Health(state, evidence, age)`
and the only way to be OK is to say so with fresh evidence.

WHAT IT DELIBERATELY DOES NOT DO. It does not stop the vehicle, refuse a
command, or surface the hull. It is a REPORT. Every existing guard keeps its
own decision -- the arming checks, the fire gate, the DVL refusal -- because a
central authority that can veto anything becomes the thing everybody works
around. This gives them one vocabulary to read, not a new veto.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Dict, List, Optional


class State(Enum):
    """Ordered worst-last so `max()` is the vehicle's overall state."""
    OK = 0          # reporting, and within its bar
    DEGRADED = 1    # reporting, outside its bar, still usable
    UNKNOWN = 2     # NOT reporting. Worse than degraded: nothing is watching.
    FAILED = 3      # reporting, and unusable

    def __lt__(self, other):
        return self.value < other.value


# How long a subsystem may say nothing before it stops being OK. Deliberately
# per-subsystem: the srot link is 10 Hz and the anchor is 3, so one number
# would either nag about the anchor or ignore a dead link for a third of a
# second. Callers pass their own; this is only the fallback.
DEFAULT_STALE_S = 2.0


@dataclass
class Health:
    """One subsystem's answer. `evidence` is why -- never a bare state."""
    name: str
    state: State = State.UNKNOWN
    evidence: str = 'never reported'
    at: float = 0.0                      # monotonic, when this was measured

    @property
    def age_s(self) -> float:
        return float('inf') if self.at <= 0.0 else time.monotonic() - self.at

    @property
    def ok(self) -> bool:
        return self.state is State.OK

    def stale(self, stale_s: float = DEFAULT_STALE_S) -> bool:
        return self.age_s > stale_s

    def __str__(self) -> str:
        age = '--' if self.at <= 0.0 else f'{self.age_s:.1f}s'
        return f'{self.name}: {self.state.name} ({self.evidence}, {age} ago)'


def ok(name: str, evidence: str) -> Health:
    return Health(name, State.OK, evidence, time.monotonic())


def degraded(name: str, evidence: str) -> Health:
    return Health(name, State.DEGRADED, evidence, time.monotonic())


def failed(name: str, evidence: str) -> Health:
    return Health(name, State.FAILED, evidence, time.monotonic())


def unknown(name: str, evidence: str = 'no data') -> Health:
    """Not reporting. Note this carries a TIMESTAMP like the others: "we asked
    just now and got nothing" is a different fact from "nobody has asked"."""
    return Health(name, State.UNKNOWN, evidence, time.monotonic())


class HealthBoard:
    """Every subsystem's state in one place, aged.

    Holds no ROS and no hardware: it takes reporter callables and calls them,
    so the same board runs in the node, in a replay and in a test.
    """

    def __init__(self, stale_s: float = DEFAULT_STALE_S):
        self._stale_s = float(stale_s)
        self._reporters: Dict[str, Callable[[], Health]] = {}
        self._last: Dict[str, Health] = {}

    def register(self, name: str, fn: Callable[[], Health]) -> None:
        self._reporters[name] = fn
        self._last.setdefault(name, Health(name))

    def poll(self) -> Dict[str, Health]:
        """Ask every reporter. A reporter that RAISES is UNKNOWN, not skipped.

        Skipping would leave the previous -- probably OK -- answer standing,
        so a subsystem whose health check is itself broken would keep reporting
        the last good news. That is the failure this whole file exists to stop.
        """
        for name, fn in self._reporters.items():
            try:
                h = fn()
                self._last[name] = h if isinstance(h, Health) else unknown(
                    name, f'reporter returned {type(h).__name__}')
            except Exception as exc:            # noqa: BLE001
                self._last[name] = unknown(name, f'{type(exc).__name__}: {exc}')
        return dict(self._last)

    def get(self, name: str) -> Health:
        return self._last.get(name, Health(name))

    def worst(self) -> State:
        """The vehicle's state: the worst of its parts, with STALE demoted.

        A subsystem that was OK ten seconds ago and has said nothing since is
        not OK; it is UNKNOWN. Ageing here rather than in each reporter means a
        reporter that stops being called cannot leave good news behind.
        """
        if not self._last:
            return State.UNKNOWN
        return max(self._aged(h).state for h in self._last.values())

    def _aged(self, h: Health) -> Health:
        if h.state is not State.UNKNOWN and h.stale(self._stale_s):
            return Health(h.name, State.UNKNOWN,
                          f'{h.evidence} (stale {h.age_s:.1f}s)', h.at)
        return h

    def report(self) -> List[str]:
        """Worst first: an operator reads the top line, not the whole list."""
        rows = sorted((self._aged(h) for h in self._last.values()),
                      key=lambda h: (-h.state.value, h.name))
        return [str(h) for h in rows]

    def not_ok(self) -> List[Health]:
        return [h for h in (self._aged(x) for x in self._last.values())
                if h.state is not State.OK]

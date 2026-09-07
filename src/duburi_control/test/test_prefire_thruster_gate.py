"""The pre-fire thruster gate, and above all its POLARITY.

A torpedo run with a dead thruster should refuse, not discover. But the
symmetric version of that rule -- "refuse unless healthy" -- is a worse bug
than the one it fixes, and these tests exist to pin the asymmetry:

    ok is False  -> REFUSE   (a thruster reported and then stopped)
    ok is None   -> ALLOW    (nothing has been announced; that is most sessions)
    ok is True   -> ALLOW

MEASURED, and the reason None must never refuse: the board announces thruster
presence only at the FIRST ARM and only as English, and ESC telemetry exists
only on Bluejay-flashed ESCs. On this vehicle today `thruster_health()` returns
None permanently. A gate that refused on None would make `fire()` a silent
no-op at competition -- the same shape as the SURFACE-mode bug, pointed at the
payload.
"""
import logging

from duburi_control.duburi import Duburi
from duburi_control.fc.base import FIRE_THRUSTER_FAULT, FIRE_NOT_READY


class _FC:
    def __init__(self, verdict):
        self._verdict = verdict

    def thruster_health(self):
        if isinstance(self._verdict, Exception):
            raise self._verdict
        return self._verdict


class _Payload:
    is_ready = True

    def __init__(self):
        self.fired = []

    def fire(self, ch):
        self.fired.append(ch)
        return True


def _duburi(verdict, payload=None):
    d = Duburi.__new__(Duburi)                 # no MAVLink, no threads
    d.pixhawk = _FC(verdict)
    d.log = logging.getLogger('test_prefire')
    d._payload = payload
    return d


def test_a_KNOWN_BAD_thruster_REFUSES_the_shot():
    """The gate biting. Injected as a real (False, reason), not as absence."""
    pl = _Payload()
    res = _duburi((False, 'thruster 3 LOST telemetry mid-session'), pl)._fire_payload(1)
    assert res.code == FIRE_THRUSTER_FAULT
    assert not res.ok
    assert 'thruster 3' in res.reason, 'the refusal must name WHICH thruster'
    assert pl.fired == [], 'a refused shot must not reach the payload driver'


def test_UNKNOWN_thruster_health_still_FIRES():
    """⛔ The regression that would silently cost every payload point.

    This is the vehicle's real state today, so if this test ever inverts, the
    torpedo stops leaving the tube at competition and nothing logs an error.
    """
    pl = _Payload()
    res = _duburi((None, 'presence announced at first arm only'), pl)._fire_payload(2)
    assert res.ok, 'UNKNOWN must never refuse -- it is most sessions'
    assert pl.fired == [2]


def test_a_HEALTHY_hull_FIRES():
    pl = _Payload()
    assert _duburi((True, 'all 8 thrusters reporting'), pl)._fire_payload(3).ok
    assert pl.fired == [3]


def test_a_backend_with_NO_thruster_health_still_FIRES():
    """The pixhawk backend has no such method. Absence of a gate is not a fault."""
    pl = _Payload()
    d = _duburi(None, pl)
    d.pixhawk = object()                       # genuinely has no such method
    assert d._fire_payload(4).ok
    assert pl.fired == [4]


def test_a_RAISING_probe_is_not_a_hull_fault():
    """A broken probe must not ground the payload. Fail open, loudly."""
    pl = _Payload()
    assert _duburi(RuntimeError('link down'), pl)._fire_payload(1).ok


def test_the_gate_runs_BEFORE_the_payload_readiness_check():
    """Ordering matters for the operator: a dead thruster and a dead payload
    are different problems, and the more consequential one should be named."""
    d = _duburi((False, 'thruster 7 reports no telemetry'), payload=None)
    res = d._fire_payload(1)
    assert res.code == FIRE_THRUSTER_FAULT, 'thruster fault outranks NOT_READY'
    assert res.code != FIRE_NOT_READY

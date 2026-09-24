"""`surface` on the SROT backend: the safety verb that once did nothing.

`test_dispatch_policy.py` proves `surface` is ACCEPTED while another command
runs -- it bypasses the busy gate for exactly that reason. It says nothing
about what the handler then does, and on this backend the answer was: raised
`ModeChangeError` and returned. The facade's `surface` is `set_depth(0)`, which
routes through `_ensure_alt_hold` -- an ArduSub mode the board does not have.

So the one verb whose whole purpose is to always run was the one that could
not. Reaching the handler is not the same as the handler working, which is the
gap these tests close.
"""
import threading
import time

import pytest

from mongla_manager import auv_manager_node as amn


class _Log:
    def __init__(self):
        self.errors = []

    def error(self, m):
        self.errors.append(m)

    def info(self, m):
        pass

    def warning(self, m):
        pass


class _FC:
    """Records the ORDER of the calls, which is the invariant under test."""

    def __init__(self, mode_ok=True, depth=-1.4):
        self.calls = []
        self.mode_ok = mode_ok
        self.depth = depth

    def stop_motion(self):
        self.calls.append('stop_motion')

    def set_mode(self, mode):
        self.calls.append(f'set_mode:{mode}')
        return (True, 'ok') if self.mode_ok else (False, 'NO_ACK')

    def get_attitude(self):
        return {'depth': self.depth, 'yaw': 0.0}


class _Node:
    """The handler is a plain method over `self.fc` / `self.mongla`, so it runs
    unbound against a stub -- no ROS graph, no board."""

    def __init__(self, fc):
        self.fc = fc
        # A REAL Event, set -- exactly what `goal_callback` leaves behind when
        # surface arrives while another verb is running.
        ev = threading.Event()
        ev.set()
        self.mongla = type('D', (), {'_abort_event': ev})()
        self._log = _Log()

    def get_logger(self):
        return self._log


def _run(fc, **kw):
    node = _Node(fc)
    return node, amn.AUVManagerNode._run_srot_surface(node, kw)


def test_it_brakes_before_it_changes_mode():
    """SURFACE does not abort a running primitive -- the board keeps executing
    the active move until something displaces it. So a surface that only
    changed mode would leave the vehicle still driving its last leg while
    ascending, which is worse than either alone.

    Order, not presence: both calls happening in the wrong order looks
    identical in a coverage report.
    """
    fc = _FC()
    _run(fc)
    assert fc.calls == ['stop_motion', 'set_mode:SURFACE']


def test_the_abort_that_brought_surface_here_stays_set():
    """SURFACE IS THE ABORT. `goal_callback` sets the flag when surface arrives
    mid-verb; the handler used to clear it on its next line, so a vision loop
    streaming MANUAL_CONTROL at 20-50 Hz (and a queued fire) never saw it and
    kept going after the operator said surface. The flag must survive the
    handler -- the next verb clears it at its own entry."""
    node, _ = _run(_FC())
    assert node.mongla._abort_event.is_set(), (
        'surface erased the abort it was sent with -- host loops keep running')


def test_a_running_host_loop_actually_stops():
    """The behaviour the flag exists for, end to end: a host loop polling the
    abort once per tick (as align_loop/move_loop do) must exit after surface."""
    ev = threading.Event()

    def _host_loop():                          # a vision loop at 50 Hz
        t_end = time.monotonic() + 2.0
        while time.monotonic() < t_end:
            if ev.is_set():
                return
            time.sleep(0.02)

    th = threading.Thread(target=_host_loop, daemon=True)
    th.start()
    time.sleep(0.05)
    node = _Node(_FC())
    node.mongla._abort_event = ev
    ev.set()                                   # goal_callback's request_abort()
    amn.AUVManagerNode._run_srot_surface(node, {})
    th.join(timeout=1.0)
    assert not th.is_alive(), 'the host loop never saw the abort surface carried'


def test_a_prior_abort_does_not_stop_the_surface_itself():
    """The other half of the old test's intent: with the flag left set, the
    surface must still brake, engage SURFACE and report success. Nothing in
    it may exit on the abort -- it is the thing the abort asked for."""
    fc = _FC()
    node, out = _run(fc)
    assert node.mongla._abort_event.is_set()
    assert fc.calls == ['stop_motion', 'set_mode:SURFACE']
    assert out.success is True


def test_it_reports_the_depth_it_is_ascending_from():
    _, out = _run(_FC(depth=-1.4))
    assert out.final_value == pytest.approx(-1.4)


def test_a_refused_mode_change_is_reported_as_a_failure():
    """The defect this verb already had, in its general form: a safety verb
    that returns success without acting is worse than one that raises, because
    the operator stops looking. If SURFACE will not engage, say so -- and log
    it, since nobody is reading a Result field during an emergency."""
    node, out = _run(_FC(mode_ok=False))
    assert out.success is False
    assert 'NO_ACK' in out.message
    assert node._log.errors, 'a failed surface must reach the log, not only the Result'


def test_it_succeeds_when_the_mode_takes():
    _, out = _run(_FC())
    assert out.success is True
    assert 'SURFACE' in out.message


def test_it_still_brakes_when_the_mode_change_fails():
    """The brake is not conditional on the ascent working. A board that refuses
    SURFACE is precisely when a still-running move matters most."""
    fc = _FC(mode_ok=False)
    _run(fc)
    assert fc.calls[0] == 'stop_motion'

"""`surface`: the safety verb that once did nothing.

`test_dispatch_policy.py` proves `surface` is ACCEPTED while another command
runs -- it bypasses the busy gate for exactly that reason. It says nothing
about what the verb then does, and the answer once was: raised
`ModeChangeError` and returned, because it routed through a depth-hold mode
the board does not have.

So the one verb whose whole purpose is to always run was the one that could
not. Reaching the verb is not the same as the verb working, which is the gap
these tests close. They run the real `Mongla.surface` over a recording board.
"""
import math
import threading
import time

import pytest

from mongla_control.mongla import Mongla


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
        return None if self.depth is None else {'depth': self.depth, 'yaw': 0.0}

    def get_attitude_age(self):
        return 0.0


def _run(fc, **kw):
    m = Mongla(fc, _Log())
    # A set abort -- exactly what the action server leaves behind when surface
    # arrives while another verb is running.
    m._abort_event.set()
    kw.setdefault('wait', False)
    return m, m.surface(**kw)


def test_it_brakes_before_it_changes_mode():
    """SURFACE does not abort a running primitive -- the board keeps executing
    the active move until something displaces it. Order, not presence: both
    calls in the wrong order look identical in a coverage report."""
    fc = _FC()
    _run(fc)
    assert fc.calls == ['stop_motion', 'set_mode:SURFACE']


def test_the_abort_that_brought_surface_here_stays_set():
    """SURFACE IS THE ABORT. Clearing the flag would let a vision loop
    streaming MANUAL_CONTROL (and a queued fire) keep going after the operator
    said surface. The next verb clears it at its own entry."""
    m, _ = _run(_FC())
    assert m._abort_event.is_set(), (
        'surface erased the abort it was sent with -- host loops keep running')


def test_a_running_host_loop_actually_stops():
    """End to end: a host loop polling the abort once per tick (as
    align_loop/move_loop do) must exit after surface."""
    m = Mongla(_FC(), _Log())
    ev = m._abort_event

    def _host_loop():                          # a vision loop at 50 Hz
        t_end = time.monotonic() + 2.0
        while time.monotonic() < t_end:
            if ev.is_set():
                return
            time.sleep(0.02)

    th = threading.Thread(target=_host_loop, daemon=True)
    th.start()
    time.sleep(0.05)
    m.request_abort()                          # the action server's abort
    m.surface(wait=False)
    th.join(timeout=1.0)
    assert not th.is_alive(), 'the host loop never saw the abort surface carried'


def test_a_prior_abort_does_not_stop_the_surface_itself():
    """With the flag left set, the surface must still brake, engage SURFACE
    and report success. It is the thing the abort asked for."""
    fc = _FC()
    m, out = _run(fc)
    assert m._abort_event.is_set()
    assert fc.calls == ['stop_motion', 'set_mode:SURFACE']
    assert out.success is True


def test_a_refused_mode_change_is_reported_as_a_failure():
    """A safety verb that returns success without acting is worse than one
    that raises, because the operator stops looking. And it is logged: nobody
    reads a Result field during an emergency."""
    m, out = _run(_FC(mode_ok=False))
    assert out.success is False
    assert 'NO_ACK' in out.message
    assert m.log.errors, 'a failed surface must reach the log, not only the Result'


def test_it_still_brakes_when_the_mode_change_fails():
    """A board that refuses SURFACE is precisely when a still-running move
    matters most."""
    fc = _FC(mode_ok=False)
    _run(fc)
    assert fc.calls[0] == 'stop_motion'


def test_it_waits_until_the_depth_reads_surfaced():
    """The caller's next step is usually disarm; returning the instant the mode
    took left the hull at depth."""
    fc = _FC(depth=-0.05)
    _, out = _run(fc, wait=True, timeout=1.0)
    assert out.success is True
    assert out.final_value == pytest.approx(-0.05)


def test_a_surface_never_seen_says_so_and_reports_the_depth():
    fc = _FC(depth=-1.4)
    _, out = _run(fc, wait=True, timeout=0.3)
    assert out.success is True          # SURFACE is engaged; the board keeps ascending
    assert 'still at -1.40' in out.message
    assert out.final_value == pytest.approx(-1.4)


def test_no_depth_is_nan_never_zero():
    """0.0 is the surface -- a real value -- so a result carrying it says the
    hull surfaced when nothing was measured at all (#21 item 3)."""
    _, out = _run(_FC(depth=None), wait=True, timeout=0.3)
    assert math.isnan(out.final_value)
    assert 'depth unreadable' in out.message

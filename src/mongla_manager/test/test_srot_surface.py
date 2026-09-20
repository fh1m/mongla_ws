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


class _Abort:
    def __init__(self):
        self.cleared = 0
        self._set = True

    def clear(self):
        self.cleared += 1
        self._set = False


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
        self.mongla = type('D', (), {'_abort_event': _Abort()})()
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


def test_a_prior_abort_does_not_block_it():
    """Every motion loop exits on `_abort_event`. If a previous cancel left it
    set, the surface would be aborted by the thing that prompted it."""
    node, _ = _run(_FC())
    assert node.mongla._abort_event.cleared == 1


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

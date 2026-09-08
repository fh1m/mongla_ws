"""B11, B17, B19 -- three cleanup/health bugs that only bite on the failure path."""

import pytest

from duburi_sensors.sources.composite_bno_dvl import CompositeBnoDvlSource


class _Src:
    def __init__(self, healthy=True, raises=False):
        self.closed, self._healthy, self._raises = False, healthy, raises
    def close(self):
        if self._raises:
            raise IOError('serial port already gone')
        self.closed = True
    def is_healthy(self):
        return self._healthy


# --- B11: is_healthy must mean BOTH, as the docstring always claimed ---------
@pytest.mark.parametrize('bno,dvl,expect', [
    (True, True, True), (True, False, False), (False, True, False), (False, False, False)])
def test_is_healthy_requires_both_sources(bno, dvl, expect):
    c = CompositeBnoDvlSource(_Src(bno), _Src(dvl))
    assert c.is_healthy() is expect, (
        'a dead DVL used to leave the composite reporting healthy, so *_dist '
        'verbs ran against an integrator that was not integrating')


def test_dvl_is_healthy_still_exposes_the_half():
    c = CompositeBnoDvlSource(_Src(True), _Src(False))
    assert c.dvl_is_healthy() is False and c.is_healthy() is False


# --- B19: close() must reach the second source even if the first raises ------
def test_close_reaches_the_dvl_when_the_bno_raises():
    dvl = _Src()
    c = CompositeBnoDvlSource(_Src(raises=True), dvl)
    with pytest.raises(IOError):
        c.close()                       # the error must NOT be swallowed
    assert dvl.closed, 'the second source leaked its socket and reader thread'


def test_close_closes_both_on_the_happy_path():
    bno, dvl = _Src(), _Src()
    CompositeBnoDvlSource(bno, dvl).close()
    assert bno.closed and dvl.closed


# --- B17: HeadingLock.stop() on a lock that was never started ----------------
def test_stopping_an_unstarted_heading_lock_does_not_raise():
    """stop() runs on abort/cleanup paths -- exactly where a lock may be
    constructed and never started (a deferred lock whose first armed command
    never arrived). RuntimeError there skipped the Ch4 release below it."""
    from duburi_control.heading_lock import HeadingLock

    released = []

    class _P:
        def send_rc_yaw_only(self, v): released.append(v)
        def get_attitude(self): return {'yaw': 0.0, 'depth': 0.0}

    class _Y:
        def read_yaw(self): return 0.0
        def is_healthy(self): return True

    class _L:
        def info(self, m): pass
        def warn(self, m): pass
        def warning(self, m): pass
        def error(self, m): pass

    lock = HeadingLock(_P(), target_deg=90, yaw_source=_Y(), log=_L())
    lock.stop()                          # must not raise
    assert released, 'the Ch4 release after the join was being skipped by the raise'

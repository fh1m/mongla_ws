"""J01 -- the heading-lock suspension must nest. It was recorded as latent; it was live.

`Heartbeat.pause/resume` is reentrant and counter-based, with an underflow guard
and a banner explaining why. `HeadingLock.suspend/resume` was a boolean
`threading.Event`, so nesting was lossy: an inner resume cancelled an outer
suspend.

The register filed that as "latent, not live", reasoning that `Duburi.lock`
serialises verbs so two scoped blocks cannot nest. That reasoning was sound and
the conclusion was wrong, because the collision is not scoped-vs-scoped:

  1. `lock_heading` while DISARMED latches a suspend that is DELIBERATELY
     unpaired. duburi.py says why: "Suspend BEFORE start so the daemon never
     emits a single Ch4 write before the first armed command -- no thruster kick
     while surface holders steady the hull."
  2. `pause` is in `_LOCK_PASSIVE_VERBS`, so the deferred-activation hook does
     NOT clear that latch -- but `pause`'s BODY uses the scoped
     `_suspend_heading_lock()`.
  3. Its `finally` sees `is_suspended` and calls `resume()`, clearing a latch it
     never owned.

The lock then streams Ch4 at 50 Hz on a DISARMED hull -- the exact kick the latch
exists to prevent -- and nothing logs it. Reproduced by executing the sequence,
which is the only reason it was found: reading the code supports the "latent"
reading.
"""
import contextlib
import logging
import os
import sys
import threading

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.abspath(os.path.join(_HERE, '..'))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from duburi_control.heading_lock import HeadingLock          # noqa: E402


class _Pixhawk:
    def is_armed(self):
        return False


class _Yaw:
    def read_yaw(self):
        return 0.0


def _lock():
    log = logging.getLogger('test_j01')
    log.addHandler(logging.NullHandler())
    # never started -- this exercises the suspension bookkeeping only, so no
    # thread runs and no Ch4 is ever written.
    return HeadingLock(_Pixhawk(), 0.0, _Yaw(), log)


@contextlib.contextmanager
def _scoped(lk):
    """`duburi._suspend_heading_lock`, transcribed."""
    lk.suspend()
    try:
        yield
    finally:
        if lk.is_suspended:
            lk.resume()


def test_a_scoped_block_does_not_clear_the_deferred_latch():
    """THE regression. This is the sequence that reached the hull."""
    lk = _lock()
    lk.suspend()                          # lock_heading while disarmed
    assert lk.is_suspended
    with _scoped(lk):                     # pause's body
        pass
    assert lk.is_suspended, (
        'a scoped suspension cleared the deferred latch -- the lock now streams '
        'Ch4 on a disarmed hull')


def test_the_deferred_activation_still_releases_it():
    """The fix must not strand the lock suspended forever."""
    lk = _lock()
    lk.suspend()
    with _scoped(lk):
        pass
    lk.resume()                           # the deferred-activation hook
    assert not lk.is_suspended


def test_nesting_is_balanced_to_any_depth():
    lk = _lock()
    with _scoped(lk):
        assert lk.is_suspended
        with _scoped(lk):
            assert lk.is_suspended
            with _scoped(lk):
                assert lk.is_suspended
            assert lk.is_suspended, 'innermost exit released the outer blocks'
        assert lk.is_suspended
    assert not lk.is_suspended, 'the outermost exit did not release'


def test_resume_cannot_underflow_into_a_negative_debt():
    """An unmatched resume must not make the NEXT suspend a no-op.

    Without the guard the count goes negative and a later suspend() only brings
    it back to zero -- the lock would keep streaming through a block that
    believes it has authority.
    """
    lk = _lock()
    for _ in range(5):
        lk.resume()
    assert not lk.is_suspended
    lk.suspend()
    assert lk.is_suspended, 'a suspend after stray resumes did not take effect'


def test_it_matches_the_sibling_primitive_it_should_always_have_matched():
    """Heartbeat solved this first; the two must not diverge again."""
    from duburi_control.heartbeat import Heartbeat
    import inspect
    hb = inspect.getsource(Heartbeat.resume)
    hl = inspect.getsource(HeadingLock.resume)
    assert '> 0' in hb and '> 0' in hl, (
        'the two suspension primitives disagree again -- one guards underflow '
        'and the other does not')


def test_the_counter_is_taken_under_a_lock():
    """suspend() is called from verb threads and resume() from the activation
    hook; `count += 1` is not atomic under either interpreter."""
    lk = _lock()
    barrier = threading.Barrier(8)

    def worker():
        barrier.wait()
        for _ in range(200):
            lk.suspend()
            lk.resume()

    ts = [threading.Thread(target=worker) for _ in range(8)]
    for t in ts:
        t.start()
    for t in ts:
        t.join()
    assert not lk.is_suspended, (
        'balanced suspend/resume pairs across threads did not net to zero -- '
        'the counter is racing')

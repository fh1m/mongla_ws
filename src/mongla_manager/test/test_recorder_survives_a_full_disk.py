"""B44 -- a full disk killed the tlog writer thread silently.

`SrotRecorder._drain` is a bare `threading.Thread` target and `_flush()` does real
disk I/O. `OSError: [Errno 28] No space left on device` -- the ordinary end state
of a Pi after a few pool sessions -- propagated straight out of the thread.
Recording stopped, nothing was logged, and the operator found out when they went
looking for the tlog after the run that went wrong: exactly the run they needed it
for.

Third instance of one shape in this audit (B23 v4l2 pump, B40 MAVLink reader):
a bare Thread target whose body does I/O with no handler. That repetition is the
finding, not the individual bug.
"""

import collections
import threading
import time

import pytest

from mongla_manager.srot_recorder import SrotRecorder


class _Log:
    def __init__(self):
        self.errors = []
    def info(self, m): pass
    def warning(self, m): pass
    def error(self, m): self.errors.append(m)


class _FullDisk:
    def write(self, b): raise OSError(28, 'No space left on device')
    def flush(self): pass
    def close(self): pass


class _Working:
    def __init__(self): self.data = b''
    def write(self, b): self.data += b
    def flush(self): pass
    def close(self): pass


def _rec(fh, log):
    r = SrotRecorder.__new__(SrotRecorder)
    r._lock = threading.Lock()
    r._q = collections.deque(maxlen=100)
    r._fh, r._log, r.path = fh, log, '/tmp/test.tlog'
    r.dropped = r.written = r.write_errors = 0
    r.last_write_error = ''
    r._consec_write_fail = 0
    r._stop = threading.Event()
    return r


def _drain_briefly(r, seconds=0.35):
    t = threading.Thread(target=r._drain, daemon=True)
    t.start()
    time.sleep(seconds)
    alive = t.is_alive()
    r._stop.set()
    t.join(timeout=2.0)
    return alive


def test_the_writer_survives_a_full_disk():
    log = _Log()
    r = _rec(_FullDisk(), log)
    for _ in range(5):
        r._q.append(b'x')
    assert _drain_briefly(r), 'ENOSPC must not kill the recorder thread'


def test_the_failure_is_counted_and_logged():
    log = _Log()
    r = _rec(_FullDisk(), log)
    r._q.append(b'x')
    _drain_briefly(r)
    assert r.write_errors >= 1, 'a swallowed write failure is silent again'
    assert log.errors, 'the operator must be told the log is incomplete'
    assert 'space' in log.errors[0].lower() or 'FAILED' in log.errors[0]
    assert 'DEGRADED' in log.errors[0] or 'incomplete' in log.errors[0], \
        'say what was LOST, not merely that a call failed'


def test_a_healthy_write_still_works_and_counts():
    log = _Log()
    fh = _Working()
    r = _rec(fh, log)
    r._q.append(b'hello')
    _drain_briefly(r)
    assert fh.data == b'hello'
    assert r.written == 1 and r.write_errors == 0 and not log.errors


def test_recovery_resets_the_consecutive_counter():
    """A transient failure must not permanently mute the next real one."""
    log = _Log()
    r = _rec(_FullDisk(), log)
    r._q.append(b'x')
    r._safe_flush()
    assert r._consec_write_fail == 1
    r._fh = _Working()
    r._q.append(b'y')
    r._safe_flush()
    assert r._consec_write_fail == 0, 'a good flush must clear the streak'

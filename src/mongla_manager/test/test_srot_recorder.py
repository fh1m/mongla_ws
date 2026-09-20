"""The raw log must be readable by something that is not us.

A log format is only worth writing if an independent decoder agrees with it.
`.tlog` was chosen for exactly that: pymavlink opens one directly, so the
round-trip below is a real check rather than a check of our own writer against
our own reader.

The other half is the decoupling. The reader thread is the only thing draining
the link and the only place the multiplexed NAMED_VALUE_FLOAT burst can be
de-multiplexed; a recorder that makes it wait on a disk has broken the vehicle
in order to record it.
"""
import struct
import sys
import time
from pathlib import Path

import pytest
from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_manager.srot_recorder import SrotRecorder      # noqa: E402


def _msgs(n=5):
    """Real encoded MAVLink messages, not stand-ins -- the point is that a real
    decoder reads them back."""
    mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
    mav.robust_parsing = True
    out = []
    for i in range(n):
        m = mav.attitude_encode(i * 100, 0.1 * i, 0.0, 0.0, 0.0, 0.0, 0.0)
        m.pack(mav)
        out.append(m)
    return out


def _record(tmp_path, msgs, **kw):
    rec = SrotRecorder(tmp_path / 'run.tlog', **kw).start()
    for m in msgs:
        rec.write(m)
    rec.stop()
    return rec


def test_pymavlink_reads_back_what_we_wrote(tmp_path):
    """The whole justification for the format. If this fails the log is a blob."""
    rec = _record(tmp_path, _msgs(5))
    assert rec.written == 5

    conn = mavutil.mavlink_connection(str(rec.path))
    got = []
    while True:
        m = conn.recv_match(blocking=False)
        if m is None:
            break
        if m.get_type() == 'ATTITUDE':
            got.append(m)
    assert len(got) == 5
    assert [int(m.time_boot_ms) for m in got] == [0, 100, 200, 300, 400], (
        'order must survive the round trip -- a log that reorders is worse '
        'than none, because every rate and gap computed from it is wrong')


def test_the_timestamp_is_wall_clock_microseconds(tmp_path):
    """`.tlog` is `[8-byte big-endian us since epoch][frame]`. Getting the
    endianness or the unit wrong still produces a file that OPENS -- pymavlink
    scans forward for a plausible timestamp -- and then reports times that are
    wrong by decades, which reads as a corrupt log rather than a bad writer.

    Measured, and the reason this test is separate from the round trip above:
    with the timestamp written little-endian, the round trip still passes --
    all five messages come back, in order. pymavlink's `scan_timestamp` walks
    forward until it finds a plausible time and recovers, so the only visible
    symptom is that every time in the file is wrong. A complete log with wrong
    times is worse than a broken one: nothing about it looks broken, and every
    rate, gap and latency derived from it is nonsense.
    """
    t0 = time.time()
    rec = _record(tmp_path, _msgs(1))
    raw = rec.path.read_bytes()
    (tusec,) = struct.unpack('>Q', raw[:8])
    assert abs(tusec / 1e6 - t0) < 5.0


def test_the_producer_does_no_io(tmp_path):
    """Whoever consumes must not pace whoever produces. `write()` is called from
    the only thread draining the link, so it queues and returns -- nothing
    reaches the disk until the writer thread runs."""
    rec = SrotRecorder(tmp_path / 'run.tlog')
    rec.path.parent.mkdir(parents=True, exist_ok=True)
    rec._fh = open(rec.path, 'wb')            # opened, writer NOT started
    for m in _msgs(3):
        rec.write(m)
    rec._fh.flush()
    assert rec.path.stat().st_size == 0, 'write() touched the disk'
    rec._fh.close()


def test_a_slow_writer_drops_frames_and_says_so(tmp_path):
    """Bounded, because unbounded is a slower crash. The count is the point: a
    drop that is not reported turns a lossy log into a log that lies, and every
    rate derived from it is quietly low."""
    rec = SrotRecorder(tmp_path / 'run.tlog', queue_max=3)
    for m in _msgs(10):
        rec.write(m)                          # no writer thread draining
    assert rec.dropped == 7


def test_a_dropped_frame_never_truncates_another(tmp_path):
    """Records are queued whole, so back-pressure loses complete frames. A
    partial frame would desynchronise every reader after it -- one drop would
    cost the rest of the file, not one message."""
    rec = SrotRecorder(tmp_path / 'run.tlog', queue_max=2)
    for m in _msgs(6):
        rec.write(m)
    rec.start()
    rec.stop()
    conn = mavutil.mavlink_connection(str(rec.path))
    n = 0
    while True:
        m = conn.recv_match(blocking=False)
        if m is None:
            break
        if m.get_type() == 'ATTITUDE':
            n += 1
    assert n == 2 and rec.dropped == 4


def test_a_message_with_no_buffer_is_skipped_not_logged_empty(tmp_path):
    """A synthesised message, or one whose pack failed, has no wire bytes. An
    empty record would desynchronise the file exactly like a partial one."""
    class _NoBuf:
        def get_msgbuf(self):
            raise AttributeError('never packed')

    rec = SrotRecorder(tmp_path / 'run.tlog')
    rec.write(_NoBuf())                       # must not raise
    assert len(rec._q) == 0


def test_recording_nothing_still_leaves_a_readable_file(tmp_path):
    rec = _record(tmp_path, [])
    assert rec.path.exists() and rec.path.stat().st_size == 0

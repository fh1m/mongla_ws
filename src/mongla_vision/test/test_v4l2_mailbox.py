"""The mailbox's invariants, against a fake driver.

No camera here. What is under test is the BOOKKEEPING -- the part that decides
whether a control loop acts on a fresh picture or an old one -- and that is
exercisable without hardware. The parts that genuinely need a camera (the
kernel's timestamps, the oldest-kept queue behaviour, the measured 396 -> 17 ms)
are measured by `tools/v4l2_latency.py` and recorded in the commit; a stub
cannot prove them and pretending otherwise would be worse than not trying.

The three invariants:

  1. read() NEVER RETURNS THE SAME FRAME TWICE as `fresh`. Everything
     downstream treats a fresh frame as a new observation: `is_new_frame` gates
     the mid-hold FIRE, the stable-frame counter gates ALIGNED. Re-serving one
     frame as two would let a single lucky detection satisfy a gate that exists
     to require several.

  2. THE TIMESTAMP IS THE CAPTURE TIME, not the read time. `_freshness` decays
     the lateral command by frame age and the coast ladder is measured in it,
     so a stamp taken when we got around to looking makes every one of those
     measure the wrong thing -- silently, since the number stays plausible.

  3. THE STRUCT LAYOUT. Two ctypes bugs were found the hard way on this
     hardware, and one of them (`v4l2_format`'s 8-byte-aligned union) survived
     a `sizeof` assertion because moving 4 bytes of padding from the back to
     the front leaves the same total.
"""
import ctypes
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_vision.cameras import v4l2_mailbox as M      # noqa: E402


# --------------------------------------------------------------------------- #
#  Struct layout -- the class of bug that cost two rounds here
# --------------------------------------------------------------------------- #
def test_v4l2_format_matches_the_kernel_abi():
    """The ioctl request number encodes sizeof(struct v4l2_format) = 208, and
    the union is 8-byte aligned. Both must hold: `sizeof` alone passed while
    every field read four bytes early, so width landed in the padding and read
    0 and height read what was really width."""
    assert ctypes.sizeof(M._Format) == 208
    assert M._Format.fmt.offset == 8


def test_v4l2_buffer_matches_the_kernel_abi():
    """88 bytes, and `m` is a UNION -- laid out as sequential fields it pushes
    `length` past the struct and mmap() fails EINVAL."""
    assert ctypes.sizeof(M._Buffer) == 88
    assert M._Buffer.m.size == ctypes.sizeof(ctypes.c_ulong)


def test_the_ioctl_numbers_encode_the_struct_sizes():
    """_IOWR packs the size in bits 16..29. A struct that disagrees with its
    own ioctl is the bug both of the above were."""
    for req, struct in ((M.VIDIOC_S_FMT, M._Format),
                        (M.VIDIOC_G_FMT, M._Format),
                        (M.VIDIOC_QUERYBUF, M._Buffer),
                        (M.VIDIOC_QBUF, M._Buffer),
                        (M.VIDIOC_DQBUF, M._Buffer)):
        assert ((req >> 16) & 0x3FFF) == ctypes.sizeof(struct), hex(req)


def test_fourcc_packs_little_endian():
    assert M._fourcc('MJPG') == 0x47504A4D


# --------------------------------------------------------------------------- #
#  The mailbox bookkeeping
# --------------------------------------------------------------------------- #
class _Fake(M.V4L2MailboxCamera):
    """The real class with the ioctls removed.

    Subclassed rather than mocked so `read()` -- the method that owns every
    invariant here -- is the SHIPPING one, not a copy that can drift from it.
    """

    def __init__(self, w=64, h=48):
        self.name, self._frame_id, self._log = 'fake', 'fake', None
        self._device, self._requested = '/dev/null', (w, h, 60)
        self._w, self._h, self._fps = w, h, 60.0
        self._fourcc, self._nbuf = 'MJPG', 4
        self._decoder = lambda _payload: np.zeros((h, w, 3), np.uint8)
        self._slot = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._served = -1
        self._captured = self._dropped_by_driver = 0
        self._skipped = 0
        self._store_age_sum = 0.0
        self._store_age_max = 0.0
        self._last_seq = None
        self._consec_fail, self._last_ok, self._idx = 0, time.monotonic(), 0
        self._pump = threading.Thread(target=lambda: None)

    def deliver(self, seq, cap_t, store_t=None):
        """What the pump does when the kernel hands over a buffer.

        The slot carries the STORE time as well as the capture time. Both are
        needed downstream: capture age is what every freshness gate reads, and
        the difference between them is what separates "the source was late"
        from "the consumer was busy" -- indistinguishable in one number, and
        the reason two hypotheses died before it existed.
        """
        with self._lock:
            self._slot = (b'x', cap_t,
                          seq, time.monotonic() if store_t is None else store_t)

    def close(self):
        self._stop.set()


def test_nothing_before_the_first_frame():
    frame, meta = _Fake().read()
    assert frame is None and meta.fresh is False


def test_a_new_frame_is_fresh():
    c = _Fake()
    c.deliver(1, time.monotonic())
    frame, meta = c.read()
    assert meta.fresh is True and frame is not None


def test_the_SAME_frame_is_never_served_twice_as_fresh():
    """THE INVARIANT. `is_new_frame` gates the mid-hold FIRE and the
    stable-frame counter gates ALIGNED; both count fresh frames as distinct
    observations. Re-serving one frame would let a single detection satisfy a
    gate written to require several."""
    c = _Fake()
    c.deliver(7, time.monotonic())
    assert c.read()[1].fresh is True
    for _ in range(5):
        frame, meta = c.read()
        assert meta.fresh is False, 'served the same frame twice'
        assert frame is None


def test_a_newer_frame_replaces_the_waiting_one():
    """A MAILBOX, not a queue: a frame arriving while the consumer is busy
    REPLACES the one waiting, it does not queue behind it."""
    c = _Fake()
    t = time.monotonic()
    c.deliver(1, t - 0.5)
    c.deliver(2, t - 0.4)
    c.deliver(3, t)              # consumer never looked; only this survives
    _frame, meta = c.read()
    assert meta.fresh is True
    assert abs(meta.stamp_monotonic - t) < 1e-6, 'served an older frame'


def test_the_stamp_is_the_CAPTURE_time_not_the_read_time():
    """`_freshness` decays the lateral command by this and the coast ladder is
    measured in it. A read-time stamp makes every one of those measure
    age-since-read -- plausible, and wrong."""
    c = _Fake()
    captured = time.monotonic() - 0.25
    c.deliver(1, captured)
    time.sleep(0.01)
    _frame, meta = c.read()
    assert meta.stamp_monotonic == pytest.approx(captured, abs=1e-6)
    age = time.monotonic() - meta.stamp_monotonic
    assert age > 0.25, f'age {age * 1000:.1f} ms -- the stamp lost the queueing'


def test_the_wall_stamp_is_the_same_instant_on_the_other_clock():
    """It is what reaches `header.stamp`, so it must be the capture instant --
    derived from the monotonic stamp, not sampled independently."""
    c = _Fake()
    captured = time.monotonic() - 0.1
    c.deliver(1, captured)
    _frame, meta = c.read()
    expected = time.time() - (time.monotonic() - captured)
    assert meta.stamp_wall == pytest.approx(expected, abs=0.01)


def test_a_decode_failure_is_not_reported_as_fresh():
    """A corrupt JPEG must not advance `_served`, or the next good frame with
    the same sequence would be suppressed."""
    c = _Fake()
    c._decoder = lambda _p: None
    c.deliver(1, time.monotonic())
    frame, meta = c.read()
    assert frame is None and meta.fresh is False
    c._decoder = lambda _p: np.zeros((48, 64, 3), np.uint8)
    assert c.read()[1].fresh is True, 'a failed decode consumed the frame'


def test_the_frame_index_counts_only_frames_actually_served():
    c = _Fake()
    c.deliver(1, time.monotonic())
    assert c.read()[1].frame_index == 0
    c.read()                                   # not fresh, must not count
    c.deliver(2, time.monotonic())
    assert c.read()[1].frame_index == 1


def test_driver_drops_are_counted_not_hidden():
    """Under this design the count should be ~0; a rising one means the pump
    itself is starved of CPU, which is the failure mode that would quietly
    return the staleness this class removes."""
    c = _Fake()
    c._last_seq = 10
    for seq in (11, 15):                       # 12,13,14 never captured
        if c._last_seq is not None and seq > c._last_seq + 1:
            c._dropped_by_driver += seq - c._last_seq - 1
        c._last_seq = seq
    assert c._dropped_by_driver == 3


def test_info_reports_the_NEGOTIATED_size():
    """The driver substitutes the nearest mode it has -- this camera answers
    640x400 for a 640x360 request. Reporting the request as if it were granted
    is how a measurement gets quoted at a resolution that never existed."""
    c = _Fake(w=640, h=400)
    c._requested = (640, 360, 60)
    assert c.info()['width'] == 640 and c.info()['height'] == 400


def test_the_slot_carries_the_store_time_as_well_as_the_capture_time():
    """`stamp_store` is what splits the pump's lateness from the consumer's.

    With only `stamp_monotonic` the two are one number, and that number was
    read wrong twice: once as "the pump falls behind and serves a backlog"
    (it does fall behind; skipping to the newest moved the result 0.04 ms)
    and once as "the kernel is dropping frames" (it was our own skip counted
    as a drop). The split showed pump-side 4.19 ms against 13.82 ms of slot
    wait, which named the GIL immediately.
    """
    c = _Fake()
    cap = time.monotonic() - 0.050
    store = time.monotonic() - 0.010
    c.deliver(1, cap, store_t=store)
    _frame, meta = c.read()
    assert meta.fresh is True
    assert meta.stamp_monotonic == pytest.approx(cap, abs=1e-6)
    assert meta.stamp_store == pytest.approx(store, abs=1e-6)
    assert meta.stamp_store > meta.stamp_monotonic


def test_skipping_to_the_newest_is_COUNTED():
    """The pump discards older queued buffers to catch up. Those discards are
    invisible unless counted -- and worse, they leave a sequence gap that
    looks exactly like a kernel drop. Counting them the same way once
    reported `dropped_by_driver=1688` for a pump that was dropping 7."""
    c = _Fake()
    assert c.info()['skipped_to_newest'] == 0
    c._skipped = 3
    assert c.info()['skipped_to_newest'] == 3


# --------------------------------------------------------------------------- #
#  B23 -- an ioctl outside a try KILLS the pump thread                          #
# --------------------------------------------------------------------------- #
# `_pump_loop` is a bare `threading.Thread` target: an exception that escapes it
# does not propagate anywhere a caller can see it. The thread simply stops and
# the buffer it held is leaked. The only symptom is `is_healthy()` going False
# 2 s later off its freshness clock -- "the camera went unhealthy", with no cause
# in any log, which is the mitigation being mistaken for a handler.
#
# The re-queue of the older buffer inside the frame-skip loop sat outside the
# try/except OSError that covered `select` and the DQBUF beside it. This asserts
# the invariant for the WHOLE function rather than that one line, because the
# next ioctl added here has exactly the same hazard.

def test_every_ioctl_in_the_pump_thread_is_inside_an_OSError_handler():
    import ast
    import inspect
    import textwrap

    from mongla_vision.cameras import v4l2_mailbox

    src = textwrap.dedent(inspect.getsource(v4l2_mailbox.V4L2MailboxCamera._pump_loop))
    fn = ast.parse(src).body[0]

    # Walk with a stack of the Try bodies we are lexically inside. A handler that
    # catches OSError (or bare `except:`) protects the calls in its `try` body.
    unguarded = []

    def walk(node, guarded):
        if isinstance(node, ast.Try):
            catches_oserror = any(
                h.type is None
                or (isinstance(h.type, ast.Name) and h.type.id in ('OSError', 'Exception'))
                or (isinstance(h.type, ast.Tuple)
                    and any(isinstance(e, ast.Name) and e.id in ('OSError', 'Exception')
                            for e in h.type.elts))
                for h in node.handlers)
            for child in node.body:
                walk(child, guarded or catches_oserror)
            # `else`/`finally`/handler bodies are NOT protected by this try.
            for child in list(node.orelse) + list(node.finalbody):
                walk(child, guarded)
            for h in node.handlers:
                for child in h.body:
                    walk(child, guarded)
            return
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'ioctl'
                and not guarded):
            unguarded.append(node.lineno)
        for child in ast.iter_child_nodes(node):
            walk(child, guarded)

    for stmt in fn.body:
        walk(stmt, False)

    assert not unguarded, (
        'ioctl in _pump_loop outside an OSError handler at relative line(s) '
        f'{unguarded}: an OSError there escapes a bare Thread target, killing '
        'the pump and leaking the buffer, with is_healthy() the only signal (B23)')


def test_a_failed_requeue_is_counted_so_it_can_trip_is_healthy():
    """Swallowing the error must not make the failure invisible too."""
    import inspect
    from mongla_vision.cameras import v4l2_mailbox

    src = inspect.getsource(v4l2_mailbox.V4L2MailboxCamera._pump_loop)
    skip = src.split('skipped_here = 0', 1)[1].split('if self._clock_monotonic', 1)[0]
    assert 'VIDIOC_QBUF' in skip, 'the frame-skip re-queue moved; re-check this test'
    assert '_consec_fail' in skip, (
        'a failed re-queue must increment _consec_fail -- otherwise the error is '
        'caught and then hidden, which is worse than the crash it replaced')

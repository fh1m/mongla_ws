"""A mailbox, not a queue: always the newest frame, with the kernel's timestamp.

WHY THIS EXISTS, IN ONE MEASUREMENT
-----------------------------------
Ask the driver for a frame after the consumer has been busy for 400 ms and this
is what four policies return (Sonix 640x360@60, 4 buffers, kernel timestamps):

    A  plain read()                        396.1 ms old   the old WebcamCamera
    B  drain the queue, take the last       348.1 ms old   the standard recipe
    C  drain, then wait for a new frame      12.0 ms old   blocks 28 ms
    D  keep-up thread (this class)           16.9 ms old   blocks 0 ms

**B is the advice in every reference on this problem and it recovers 12 % of
the staleness.** The reason is a fact about V4L2 that the advice does not
account for, and that this code was written after measuring:

    WHEN THE BUFFERS ARE FULL THE DRIVER KEEPS THE OLDEST FRAMES AND DROPS
    EVERY NEW ONE.

Verified off the driver's own `sequence` counter, on two cameras from different
vendors: stall 400 ms at 60 fps and the queue holds sequence +1..+4 -- the four
frames from the START of the stall. Frames +5..+24 were never captured at all,
because the kernel had no free buffer to put them in. So the queue is not a
backlog of recent frames you can skip through; it is a fossil record of the
moment the consumer stalled, and "drop the stale ones" empties a fossil record
and hands you the least-old fossil.

That leaves two ways to be fresh, and only one of them is free:

  * DRAIN AND THEN WAIT (C). Correct, and it blocks a frame period.
  * NEVER LET THE QUEUE FILL (D, this class). A thread that dequeues
    continuously and requeues immediately means the driver ALWAYS has a free
    buffer, so it never drops a frame and the newest one is always about one
    frame old. Nothing blocks.

Measured under a busy consumer (25 ms of work per tick, the shape of an
inference): 22.4 ms median, 31.95 ms p95 -- bounded, no tail -- with the
consumer's own tick at 33.39/33.40 ms median/p95, i.e. no GIL starvation. The
pump held the camera's full 62.5 Hz with 0 sequence gaps.

WHAT THIS IS NOT
----------------
It is not a faster camera. The frame is ~1.0-1.5 frame periods old no matter
what, because that is how long the sensor and the USB isochronous transfer
take. This removes the QUEUEING on top of that, which is the part that grows
without bound when the consumer is slow.

BUFFER COUNT IS NOT A LEVER, which is worth stating because the code this
replaces set `CAP_PROP_BUFFERSIZE = 1` with the comment "avoid stale frames".
Measured under this class at 2/3/4/6/8 buffers: 26.4-26.6 ms median and
35.3-35.5 ms p95, flat within noise, 1 dropped frame in ~400 at every setting.
The pump is what fixes staleness; the buffer count only decides how much slack
there is before the driver starts dropping. Four is plenty.

THE TIMESTAMP IS THE OTHER HALF
-------------------------------
`FrameMeta.stamp_monotonic` is the kernel's capture time, not the time this
code got around to looking. Everything downstream that reasons about age --
`_freshness`, the coast ladder, `is_new_frame`, the fire gate -- is only as
honest as that field, and stamping it at publish time (which is what the ROS
node did) makes every one of those measure age-since-publish and be
structurally blind to exactly the latency this class exists to remove.
"""

from __future__ import annotations

import ctypes
import fcntl
import mmap
import os
import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np

from .camera import Camera, FrameMeta

# ---- the slice of videodev2.h we need -------------------------------------- #
VIDIOC_G_FMT = 0xC0D05604
VIDIOC_S_FMT = 0xC0D05605
VIDIOC_REQBUFS = 0xC0145608
VIDIOC_QUERYBUF = 0xC0585609
VIDIOC_QBUF = 0xC058560F
VIDIOC_DQBUF = 0xC0585611
VIDIOC_STREAMON = 0x40045612
VIDIOC_STREAMOFF = 0x40045613
VIDIOC_S_PARM = 0xC0CC5616

V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_MEMORY_MMAP = 1
V4L2_FIELD_NONE = 1
V4L2_BUF_FLAG_TIMESTAMP_MASK = 0x0000E000
V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC = 0x00002000


def _fourcc(s: str) -> int:
    return (ord(s[0]) | (ord(s[1]) << 8) | (ord(s[2]) << 16) | (ord(s[3]) << 24))


class _Timeval(ctypes.Structure):
    _fields_ = [('tv_sec', ctypes.c_long), ('tv_usec', ctypes.c_long)]


class _Timecode(ctypes.Structure):
    _fields_ = [('type', ctypes.c_uint32), ('flags', ctypes.c_uint32),
                ('frames', ctypes.c_uint8), ('seconds', ctypes.c_uint8),
                ('minutes', ctypes.c_uint8), ('hours', ctypes.c_uint8),
                ('userbits', ctypes.c_uint8 * 4)]


class _Buffer(ctypes.Structure):
    # `m` is a UNION -- offset / userptr / planes / fd overlay each other.
    # Sequential fields push `length` past the struct and mmap() fails EINVAL.
    class _M(ctypes.Union):
        _fields_ = [('offset', ctypes.c_uint32), ('userptr', ctypes.c_ulong),
                    ('planes', ctypes.c_void_p), ('fd', ctypes.c_int32)]

    _fields_ = [('index', ctypes.c_uint32), ('type', ctypes.c_uint32),
                ('bytesused', ctypes.c_uint32), ('flags', ctypes.c_uint32),
                ('field', ctypes.c_uint32), ('timestamp', _Timeval),
                ('timecode', _Timecode), ('sequence', ctypes.c_uint32),
                ('memory', ctypes.c_uint32), ('m', _M),
                ('length', ctypes.c_uint32), ('reserved2', ctypes.c_uint32),
                ('request_fd', ctypes.c_int32)]


class _Pix(ctypes.Structure):
    _fields_ = [('width', ctypes.c_uint32), ('height', ctypes.c_uint32),
                ('pixelformat', ctypes.c_uint32), ('field', ctypes.c_uint32),
                ('bytesperline', ctypes.c_uint32), ('sizeimage', ctypes.c_uint32),
                ('colorspace', ctypes.c_uint32), ('priv', ctypes.c_uint32),
                ('flags', ctypes.c_uint32), ('ycbcr_enc', ctypes.c_uint32),
                ('quantization', ctypes.c_uint32), ('xfer_func', ctypes.c_uint32)]


class _Format(ctypes.Structure):
    # THE UNION IS 8-BYTE ALIGNED, so the kernel pads 4 bytes after `type`.
    # Without the c_uint64 member ctypes packs the union at offset 4 and every
    # field reads four bytes early -- width lands in the padding and reads 0,
    # height reads what was really width. A `sizeof` check does NOT catch it:
    # 4 bytes moved from the back to the front leaves the same 208 total.
    # `tools/v4l2_latency.py --check` compares against v4l2-ctl, which is the
    # only thing that proves a layout.
    class _U(ctypes.Union):
        _fields_ = [('pix', _Pix), ('raw', ctypes.c_uint8 * 200),
                    ('_align', ctypes.c_uint64)]

    _fields_ = [('type', ctypes.c_uint32), ('fmt', _U)]


assert _Format.fmt.offset == 8 and ctypes.sizeof(_Format) == 208, (
    f'struct v4l2_format: fmt at offset {_Format.fmt.offset} (expected 8), '
    f'size {ctypes.sizeof(_Format)} (expected 208)')


class _Requestbuffers(ctypes.Structure):
    _fields_ = [('count', ctypes.c_uint32), ('type', ctypes.c_uint32),
                ('memory', ctypes.c_uint32), ('capabilities', ctypes.c_uint32),
                ('flags', ctypes.c_uint8), ('reserved', ctypes.c_uint8 * 3)]


class _Captureparm(ctypes.Structure):
    _fields_ = [('capability', ctypes.c_uint32), ('capturemode', ctypes.c_uint32),
                ('timeperframe_num', ctypes.c_uint32),
                ('timeperframe_den', ctypes.c_uint32),
                ('extendedmode', ctypes.c_uint32), ('readbuffers', ctypes.c_uint32),
                ('reserved', ctypes.c_uint32 * 4)]


class _Streamparm(ctypes.Structure):
    class _U(ctypes.Union):
        _fields_ = [('capture', _Captureparm), ('raw', ctypes.c_uint8 * 200)]

    _fields_ = [('type', ctypes.c_uint32), ('parm', _U)]


class V4L2MailboxCamera(Camera):
    """USB camera with a keep-up thread and a one-deep mailbox.

    Same `Camera` interface as `WebcamCamera`, so it is a drop-in for the
    profile factory. `read()` never blocks and never returns the same frame
    twice: `FrameMeta.fresh` is False when the consumer is asking faster than
    the camera produces, which is the honest answer and what lets a control
    loop skip a tick instead of re-acting on a frame it already used.
    """

    source_kind = 'v4l2_mailbox'

    # Four buffers. Measured flat from 2 to 8 under this design (26.4-26.6 ms
    # median), so this is slack for a scheduling hiccup, not a tuning knob.
    _NBUF = 4

    def __init__(self, device='/dev/video0', width=640, height=360, fps=60,
                 frame_id='cam', name='cam', logger=None, fourcc='MJPG'):
        self.name = str(name)
        self._frame_id = str(frame_id)
        self._log = logger
        self._device = device
        self._requested = (int(width), int(height), int(fps))

        self._fd = os.open(str(device), os.O_RDWR)
        try:
            self._negotiate(width, height, fps, fourcc)
            self._map_buffers()
            self._streamon()
        except Exception:
            os.close(self._fd)
            raise

        self._decoder = (self._decode_mjpeg if self._fourcc == 'MJPG'
                         else self._decode_yuyv)

        # THE MAILBOX -- LOCK-FREE, by an atomic reference store.
        #
        # `self._slot = (payload, cap_t, seq)` is ONE `STORE_ATTR`, and the
        # tuple it publishes is immutable. So a reader's `s = self._slot` is
        # one `LOAD_ATTR` and either gets the whole previous frame or the whole
        # new one -- there is no state in which it can see half of each. That
        # is the atomic pointer swap; nothing else is needed for correctness.
        #
        # THE BUFFERING IS BY REFCOUNT, WHICH IS BETTER THAN DOUBLE BUFFERING.
        # A consumer that is mid-decode holds a reference to its tuple, so the
        # pump's next store cannot touch that memory -- the old payload stays
        # alive exactly as long as someone is reading it and is freed the
        # instant they are not. A fixed pair of buffers has to reason about
        # whether the reader finished with buffer B before the writer wraps
        # back onto it; refcounting answers that by construction, and the
        # depth is "however many are actually in use" rather than two.
        #
        # THE LOCK IT REPLACES WAS MEASURED, not assumed away: 0.198 us per
        # store, 2.0x the bare store, which is 0.001 % of a 16 ms frame. So
        # this is not a latency change and is not claimed as one. It is a
        # scheduling one: a lock is a place where the pump can be made to WAIT
        # for a consumer, and on a process whose GIL slice was measured
        # starving an I/O thread by 108 ms (see rt.py), removing every such
        # place from the capture path is worth more than the 0.2 us.
        self._slot: Optional[tuple] = None
        self._stop = threading.Event()
        self._served = -1          # sequence of the frame last handed out
        self._captured = 0
        self._dropped_by_driver = 0
        self._last_seq = None
        self._consec_fail = 0
        self._last_ok = time.monotonic()
        self._idx = 0

        self._pump = threading.Thread(target=self._pump_loop, daemon=True,
                                      name=f'v4l2-pump-{self.name}')
        self._pump.start()

        if self._log:
            self._log.info(
                f'[CAM  ] v4l2 mailbox {device!r}: {self._w}x{self._h} '
                f'@ {self._fps:.0f} {self._fourcc}, {self._nbuf} buffers, '
                f'kernel timestamps')
            if (self._w, self._h) != (int(width), int(height)):
                # The driver substitutes the nearest mode it has. Silently
                # reporting the REQUESTED size is how a measurement ends up
                # quoted at a resolution that never existed.
                self._log.warning(
                    f'[CAM  ] {device!r} substituted {self._w}x{self._h} for '
                    f'the requested {width}x{height} -- check '
                    f'`v4l2-ctl -d <dev> --list-formats-ext`')

    # ------------------------------------------------------------------ #
    #  Setup
    # ------------------------------------------------------------------ #
    def _negotiate(self, width, height, fps, fourcc):
        f = _Format()
        f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        f.fmt.pix.width = int(width)
        f.fmt.pix.height = int(height)
        f.fmt.pix.pixelformat = _fourcc(fourcc)
        f.fmt.pix.field = V4L2_FIELD_NONE
        fcntl.ioctl(self._fd, VIDIOC_S_FMT, f)

        # Read it BACK. S_FMT writes into the caller's struct, but reading a
        # second time through G_FMT is what makes "what did we actually get"
        # independent of our own struct being right about the write path.
        g = _Format()
        g.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        fcntl.ioctl(self._fd, VIDIOC_G_FMT, g)
        self._w, self._h = int(g.fmt.pix.width), int(g.fmt.pix.height)
        got = int(g.fmt.pix.pixelformat)
        self._fourcc = ''.join(chr((got >> (8 * i)) & 0xFF) for i in range(4))
        self._sizeimage = int(g.fmt.pix.sizeimage)

        p = _Streamparm()
        p.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        p.parm.capture.timeperframe_num = 1
        p.parm.capture.timeperframe_den = int(fps)
        try:
            fcntl.ioctl(self._fd, VIDIOC_S_PARM, p)
        except OSError:
            pass        # not every driver honours S_PARM; the rate is advisory
        den = p.parm.capture.timeperframe_den or int(fps)
        num = p.parm.capture.timeperframe_num or 1
        self._fps = float(den) / float(num)

    def _map_buffers(self):
        r = _Requestbuffers()
        r.count = self._NBUF
        r.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        r.memory = V4L2_MEMORY_MMAP
        fcntl.ioctl(self._fd, VIDIOC_REQBUFS, r)
        self._nbuf = int(r.count)
        if self._nbuf < 2:
            raise RuntimeError(
                f'v4l2: driver granted only {self._nbuf} buffers; a mailbox '
                f'needs at least 2 (one in flight, one being filled)')

        self._maps = []
        for i in range(self._nbuf):
            b = _Buffer()
            b.index, b.type, b.memory = (i, V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                         V4L2_MEMORY_MMAP)
            fcntl.ioctl(self._fd, VIDIOC_QUERYBUF, b)
            self._maps.append(mmap.mmap(
                self._fd, b.length, mmap.MAP_SHARED,
                mmap.PROT_READ | mmap.PROT_WRITE, offset=b.m.offset))
            fcntl.ioctl(self._fd, VIDIOC_QBUF, b)

    def _streamon(self):
        t = ctypes.c_int(V4L2_BUF_TYPE_VIDEO_CAPTURE)
        fcntl.ioctl(self._fd, VIDIOC_STREAMON, t)
        self._clock_monotonic: Optional[bool] = None

    # ------------------------------------------------------------------ #
    #  The pump
    # ------------------------------------------------------------------ #
    def _pump_loop(self):
        """Dequeue, copy, requeue IMMEDIATELY, forever.

        Requeueing before doing anything else is the whole mechanism: the
        driver always has a free buffer, so it never drops a frame and never
        accumulates a queue of old ones. The copy is ~2 kB-30 kB of MJPEG
        (memcpy, not a decode) and the decode happens in the consumer, where
        the cost is paid only for frames actually used.
        """
        while not self._stop.is_set():
            try:
                b = _Buffer()
                b.type, b.memory = V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP
                fcntl.ioctl(self._fd, VIDIOC_DQBUF, b)
            except OSError:
                if self._stop.is_set():
                    return
                self._consec_fail += 1
                time.sleep(0.01)
                continue

            if self._clock_monotonic is None:
                self._clock_monotonic = (
                    (b.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK)
                    == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)
                if not self._clock_monotonic and self._log:
                    # Comparing a realtime stamp against time.monotonic()
                    # produces a plausible, meaningless age. Say so rather than
                    # let every freshness gate downstream act on it.
                    self._log.warning(
                        f'[CAM  ] {self._device!r} does not stamp buffers on '
                        f'CLOCK_MONOTONIC -- frame ages will be wrong. '
                        f'Freshness gates and the fire gate depend on this.')

            cap_t = b.timestamp.tv_sec + b.timestamp.tv_usec / 1e6
            seq = int(b.sequence)
            payload = bytes(self._maps[b.index][:b.bytesused])
            fcntl.ioctl(self._fd, VIDIOC_QBUF, b)

            if self._last_seq is not None and seq > self._last_seq + 1:
                # The driver produced frames it could not store. Under this
                # design that should be ~0; a rising count means the pump
                # itself is being starved of CPU.
                self._dropped_by_driver += seq - self._last_seq - 1
            self._last_seq = seq
            self._captured += 1
            self._consec_fail = 0
            self._last_ok = time.monotonic()

            # One atomic store. The tuple is built first and published last,
            # so a reader never observes a partially-assembled frame.
            self._slot = (payload, cap_t, seq)

    # ------------------------------------------------------------------ #
    #  Camera interface
    # ------------------------------------------------------------------ #
    def read(self) -> Tuple[Optional[np.ndarray], FrameMeta]:
        slot = self._slot          # one atomic load; see __init__
        meta = FrameMeta(frame_index=self._idx, width=self._w, height=self._h)
        if slot is None:
            meta.fresh = False
            return None, meta

        payload, cap_t, seq = slot
        if seq == self._served:
            # Asked faster than the camera produces. Returning the same frame
            # again would make it look like a new observation to every
            # freshness gate downstream -- `is_new_frame`, the stable-frame
            # counter, the fire gate. `fresh=False` is the honest answer.
            meta.fresh = False
            meta.stamp_monotonic = cap_t
            return None, meta

        frame = self._decoder(payload)
        if frame is None:
            meta.fresh = False
            return None, meta

        self._served = seq
        self._idx += 1
        meta.fresh = True
        # THE KERNEL'S CAPTURE TIME, not now(). Everything downstream that
        # reasons about age is only as honest as this field.
        meta.stamp_monotonic = cap_t
        meta.stamp_wall = time.time() - (time.monotonic() - cap_t)
        return frame, meta

    def _decode_mjpeg(self, payload: bytes):
        return cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)

    def _decode_yuyv(self, payload: bytes):
        arr = np.frombuffer(payload, np.uint8)
        want = self._w * self._h * 2
        if arr.size < want:
            return None
        return cv2.cvtColor(arr[:want].reshape(self._h, self._w, 2),
                            cv2.COLOR_YUV2BGR_YUYV)

    def is_healthy(self) -> bool:
        return (not self._stop.is_set()
                and self._consec_fail < 30
                and (time.monotonic() - self._last_ok) < 2.0)

    def info(self) -> dict:
        return {
            'name': self.name,
            'source_kind': self.source_kind,
            'width': self._w,
            'height': self._h,
            'fps': self._fps,
            'frame_id': self._frame_id,
            'device': self._device,
            'fourcc': self._fourcc,
            'buffers': self._nbuf,
            'captured': self._captured,
            'dropped_by_driver': self._dropped_by_driver,
        }

    def close(self) -> None:
        self._stop.set()
        if self._pump.is_alive():
            self._pump.join(timeout=1.0)
        try:
            t = ctypes.c_int(V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(self._fd, VIDIOC_STREAMOFF, t)
        except OSError:
            pass
        for m in getattr(self, '_maps', []):
            try:
                m.close()
            except Exception:
                pass
        try:
            os.close(self._fd)
        except OSError:
            pass

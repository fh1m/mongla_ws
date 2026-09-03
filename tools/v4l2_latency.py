#!/usr/bin/env python3
"""Measure camera latency against the KERNEL's own capture timestamp.

WHY NOT JUST TIME `cap.read()`
------------------------------
Timing `read()` measures how long the call took, which is the one thing that is
NOT the latency. A frame that was already sitting in the driver's queue returns
in microseconds and is 60 ms old; a frame we waited for returns in 16 ms and is
brand new. The call duration is close to anti-correlated with staleness, so
every "we read at N Hz" figure in this repo says nothing about how old the
picture is when a control loop acts on it.

The kernel knows. `v4l2_buffer.timestamp` is filled in by the UVC driver when
the frame finishes arriving, on CLOCK_MONOTONIC by default (the driver reports
which clock in `v4l2_buffer.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK`, and this
tool refuses to report a number if it is not monotonic rather than quietly
comparing two different clocks). Subtract it from `time.monotonic()` at the
moment we get the buffer and the difference is the real age of the pixels.

OpenCV's V4L2 backend does not expose it -- there is no `CAP_PROP_*` for the
buffer timestamp -- so this talks to the device directly through ioctl. That is
also what makes the DRAIN experiment possible: dequeue-and-requeue moves a
buffer without decoding it, so "throw away everything stale" costs almost
nothing, where OpenCV's `read()` pays a full MJPEG decode per discarded frame.

WHAT IT REPORTS
    age at dequeue      how old a frame is the instant the kernel hands it over
    queue depth         how many frames are waiting behind it
    drain cost          what it costs to discard them
    age after draining  the number that matters -- the freshest frame available
"""
from __future__ import annotations

import argparse
import ctypes
import fcntl
import mmap
import os
import statistics as st
import time

# ---- the slice of videodev2.h we need -------------------------------------- #
VIDIOC_QUERYCAP = 0x80685600
# _IOWR('V', 5, struct v4l2_format). The size field of the request encodes
# sizeof(struct v4l2_format) = 204, so our Format below must be exactly that or
# the ioctl copies the wrong number of bytes.
VIDIOC_S_FMT = 0xC0D05605
VIDIOC_G_FMT = 0xC0D05604
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

# Which clock the driver stamped the buffer with. Comparing a timestamp from
# one clock against `time.monotonic()` produces a plausible number that is
# meaningless, so this is checked rather than assumed.
V4L2_BUF_FLAG_TIMESTAMP_MASK = 0x0000e000
V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC = 0x00002000


def fourcc(a, b, c, d):
    return ord(a) | (ord(b) << 8) | (ord(c) << 16) | (ord(d) << 24)


class Timeval(ctypes.Structure):
    _fields_ = [('tv_sec', ctypes.c_long), ('tv_usec', ctypes.c_long)]


class Timecode(ctypes.Structure):
    _fields_ = [('type', ctypes.c_uint32), ('flags', ctypes.c_uint32),
                ('frames', ctypes.c_uint8), ('seconds', ctypes.c_uint8),
                ('minutes', ctypes.c_uint8), ('hours', ctypes.c_uint8),
                ('userbits', ctypes.c_uint8 * 4)]


class Buffer(ctypes.Structure):
    # `m` is a UNION in videodev2.h -- offset / userptr / planes / fd overlay
    # each other. Laying them out as sequential fields shifts `length` past the
    # end of the real struct, and mmap() then fails with EINVAL on a length
    # read out of padding. It is a silent class of bug: the ioctls all succeed,
    # so the mistake only surfaces at the one call that uses the field.
    class _M(ctypes.Union):
        _fields_ = [('offset', ctypes.c_uint32),
                    ('userptr', ctypes.c_ulong),
                    ('planes', ctypes.c_void_p),
                    ('fd', ctypes.c_int32)]

    _fields_ = [('index', ctypes.c_uint32), ('type', ctypes.c_uint32),
                ('bytesused', ctypes.c_uint32), ('flags', ctypes.c_uint32),
                ('field', ctypes.c_uint32), ('timestamp', Timeval),
                ('timecode', Timecode), ('sequence', ctypes.c_uint32),
                ('memory', ctypes.c_uint32), ('m', _M),
                ('length', ctypes.c_uint32), ('reserved2', ctypes.c_uint32),
                ('request_fd', ctypes.c_int32)]


class Pix(ctypes.Structure):
    _fields_ = [('width', ctypes.c_uint32), ('height', ctypes.c_uint32),
                ('pixelformat', ctypes.c_uint32), ('field', ctypes.c_uint32),
                ('bytesperline', ctypes.c_uint32), ('sizeimage', ctypes.c_uint32),
                ('colorspace', ctypes.c_uint32), ('priv', ctypes.c_uint32),
                ('flags', ctypes.c_uint32), ('ycbcr_enc', ctypes.c_uint32),
                ('quantization', ctypes.c_uint32), ('xfer_func', ctypes.c_uint32)]


class Format(ctypes.Structure):
    # THE UNION IS 8-BYTE ALIGNED. `struct v4l2_format` is
    # `{ __u32 type; union {...} fmt; }` and the union contains 64-bit members,
    # so the compiler puts 4 bytes of padding after `type`. ctypes packs it at
    # offset 4 unless the union itself has 8-byte alignment, and then EVERY
    # field reads four bytes early: width lands in the padding and reads 0,
    # height reads what was actually width.
    #
    # THE SIZE ASSERTION DID NOT CATCH THIS -- 4 bytes of padding at the front
    # and 4 fewer at the back gives the same 208 total. Two struct bugs in this
    # file were found by a number looking wrong; `--check` compares against
    # v4l2-ctl, which is the only thing that actually proves a layout.
    class _U(ctypes.Union):
        _pack_ = 0
        # 204 bytes: the union in videodev2.h is `__u8 raw_data[200]`, and
        # `type` + 4 bytes of alignment padding bring the struct to the 208 the
        # ioctl request number encodes. ctypes reports the union's own size, so
        # the assertion below checks the whole struct.
        _fields_ = [('pix', Pix), ('raw', ctypes.c_uint8 * 200),
                    ('_align', ctypes.c_uint64)]
    _fields_ = [('type', ctypes.c_uint32), ('fmt', _U)]


assert Format.fmt.offset == 8 and ctypes.sizeof(Format) == 208, (
    # The ioctl request number encodes the struct size; a mismatch makes the
    # kernel copy the wrong byte count. This tool already shipped one struct
    # bug (the v4l2_buffer union) whose only symptom was an EINVAL at a single
    # call site -- a size assertion turns that class of error into a message.
    f'struct v4l2_format: fmt at offset {Format.fmt.offset} (expected 8), size {ctypes.sizeof(Format)} (expected 208)')


class Requestbuffers(ctypes.Structure):
    _fields_ = [('count', ctypes.c_uint32), ('type', ctypes.c_uint32),
                ('memory', ctypes.c_uint32), ('capabilities', ctypes.c_uint32),
                ('flags', ctypes.c_uint8), ('reserved', ctypes.c_uint8 * 3)]


class Captureparm(ctypes.Structure):
    _fields_ = [('capability', ctypes.c_uint32), ('capturemode', ctypes.c_uint32),
                ('timeperframe_num', ctypes.c_uint32),
                ('timeperframe_den', ctypes.c_uint32),
                ('extendedmode', ctypes.c_uint32), ('readbuffers', ctypes.c_uint32),
                ('reserved', ctypes.c_uint32 * 4)]


class Streamparm(ctypes.Structure):
    class _U(ctypes.Union):
        _fields_ = [('capture', Captureparm), ('raw', ctypes.c_uint8 * 200)]
    _fields_ = [('type', ctypes.c_uint32), ('parm', _U)]


class V4L2Camera:
    """Minimal MMAP capture. Enough to dequeue buffers and read their stamps."""

    def __init__(self, dev, width=640, height=360, fps=60, nbuf=4, fmt='MJPG'):
        self.fd = os.open(dev, os.O_RDWR)
        f = Format()
        f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        f.fmt.pix.width = width
        f.fmt.pix.height = height
        f.fmt.pix.pixelformat = fourcc(*fmt)
        f.fmt.pix.field = V4L2_FIELD_NONE
        fcntl.ioctl(self.fd, VIDIOC_S_FMT, f)
        # Read it BACK with G_FMT. A driver silently substitutes the nearest
        # mode it supports -- this camera answers 640x400 for a 640x360 request
        # -- and reporting the requested size as if it were granted is how a
        # measurement ends up quoted at a resolution that never existed. (It
        # did: this tool printed 640x640 for three runs.)
        g = Format()
        g.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        fcntl.ioctl(self.fd, VIDIOC_G_FMT, g)
        self.width, self.height = g.fmt.pix.width, g.fmt.pix.height
        self.requested = (width, height)

        p = Streamparm()
        p.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        p.parm.capture.timeperframe_num = 1
        p.parm.capture.timeperframe_den = int(fps)
        try:
            fcntl.ioctl(self.fd, VIDIOC_S_PARM, p)
        except OSError:
            pass
        self.fps = (p.parm.capture.timeperframe_den
                    / max(1, p.parm.capture.timeperframe_num))

        r = Requestbuffers()
        r.count, r.type, r.memory = nbuf, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP
        fcntl.ioctl(self.fd, VIDIOC_REQBUFS, r)
        self.nbuf = r.count

        self.maps = []
        for i in range(self.nbuf):
            b = Buffer()
            b.index, b.type, b.memory = i, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP
            fcntl.ioctl(self.fd, VIDIOC_QUERYBUF, b)
            self.maps.append(mmap.mmap(self.fd, b.length,
                                       mmap.MAP_SHARED,
                                       mmap.PROT_READ | mmap.PROT_WRITE,
                                       offset=b.m.offset))
            fcntl.ioctl(self.fd, VIDIOC_QBUF, b)

        t = ctypes.c_int(V4L2_BUF_TYPE_VIDEO_CAPTURE)
        fcntl.ioctl(self.fd, VIDIOC_STREAMON, t)
        self.clock_ok = None

    def dequeue(self):
        """(index, capture_monotonic, bytesused, sequence). Blocks for a frame."""
        b = Buffer()
        b.type, b.memory = V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP
        fcntl.ioctl(self.fd, VIDIOC_DQBUF, b)
        if self.clock_ok is None:
            self.clock_ok = ((b.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK)
                             == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)
        return (b.index, b.timestamp.tv_sec + b.timestamp.tv_usec / 1e6,
                b.bytesused, b.sequence)

    def requeue(self, index):
        b = Buffer()
        b.index, b.type, b.memory = index, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP
        fcntl.ioctl(self.fd, VIDIOC_QBUF, b)

    def try_dequeue(self):
        """Non-blocking. None when the driver has nothing waiting."""
        try:
            flags = fcntl.fcntl(self.fd, fcntl.F_GETFL)
            fcntl.fcntl(self.fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
            return self.dequeue()
        except OSError:
            return None
        finally:
            fcntl.fcntl(self.fd, fcntl.F_SETFL, flags)

    def close(self):
        try:
            t = ctypes.c_int(V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(self.fd, VIDIOC_STREAMOFF, t)
        except OSError:
            pass
        for m in self.maps:
            m.close()
        os.close(self.fd)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('device')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=360)
    ap.add_argument('--fps', type=int, default=60)
    ap.add_argument('--nbuf', type=int, default=4)
    ap.add_argument('--seconds', type=float, default=6.0)
    ap.add_argument('--stall', type=float, default=0.4,
                    help='pause before the drain test, to let the queue fill')
    ap.add_argument('--check', action='store_true',
                    help='verify the ctypes layout against v4l2-ctl and exit. '
                         'Run this FIRST on any new kernel: two struct bugs in '
                         'this file were found by a printed number looking '
                         'wrong, and a size assertion caught neither.')
    A = ap.parse_args()

    if A.check:
        import subprocess
        fd = os.open(A.device, os.O_RDWR)
        g = Format()
        g.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        fcntl.ioctl(fd, VIDIOC_G_FMT, g)
        os.close(fd)
        mine = (g.fmt.pix.width, g.fmt.pix.height)
        out = subprocess.run(['v4l2-ctl', '-d', A.device, '--get-fmt-video'],
                             capture_output=True, text=True).stdout
        truth = None
        for line in out.splitlines():
            if 'Width/Height' in line:
                w, h = line.split(':')[1].strip().split('/')
                truth = (int(w), int(h))
        print(f'  our G_FMT {mine[0]}x{mine[1]}   v4l2-ctl {truth[0]}x{truth[1]}')
        ok = mine == truth
        print('  -> ' + ('LAYOUT OK' if ok else 'STRUCT IS WRONG'))
        return 0 if ok else 2

    cam = V4L2Camera(A.device, A.width, A.height, A.fps, A.nbuf)
    print(f'\n  {A.device}  {cam.width}x{cam.height} @ {cam.fps:.0f} '
          f'requested, {cam.nbuf} buffers')

    # -- 1. steady-state: dequeue as fast as the camera produces ------------- #
    # Discard the first frames after STREAMON: the driver has just started the
    # sensor and the isochronous USB stream is still ramping, so frame 1 is
    # tens of milliseconds old through no fault of the pipeline. Leaving it in
    # put an 88 ms max against a 16 ms p95 in three earlier runs and read as a
    # scheduling tail worth chasing.
    for _ in range(15):
        i, _t, _b, _s = cam.dequeue()
        cam.requeue(i)

    ages, periods = [], []
    prev = None
    t_end = time.monotonic() + A.seconds
    while time.monotonic() < t_end:
        idx, cap_t, _n, _seq = cam.dequeue()
        now = time.monotonic()
        ages.append((now - cap_t) * 1000.0)
        if prev is not None:
            periods.append((cap_t - prev) * 1000.0)
        prev = cap_t
        cam.requeue(idx)

    if not cam.clock_ok:
        print('\n  REFUSING TO REPORT AN AGE: the driver did not stamp these '
              'buffers on CLOCK_MONOTONIC, so subtracting time.monotonic() '
              'would compare two different clocks and produce a plausible '
              'wrong number.')
        cam.close()
        return 2

    ages.sort()
    print(f'\n  STEADY STATE ({len(ages)} frames, kernel timestamps)')
    print(f'    rate            {1000.0 / st.median(periods):6.1f} Hz')
    print(f'    age at dequeue  {st.median(ages):6.2f} ms median, '
          f'{ages[int(0.95 * len(ages))]:6.2f} p95, {ages[-1]:6.2f} max')

    # -- 2. what a stall costs, and what draining recovers ------------------ #
    print(f'\n  AFTER A {A.stall * 1000:.0f} ms STALL '
          f'(a slow consumer -- inference, a GC pause, a busy core)')
    first, drained, depths, drain_ms = [], [], [], []
    for _ in range(8):
        for _ in range(4):                       # settle
            i, _t, _n, _s = cam.dequeue()
            cam.requeue(i)
        time.sleep(A.stall)

        i, cap_t, _n, _s = cam.dequeue()
        now = time.monotonic()
        first.append((now - cap_t) * 1000.0)
        cam.requeue(i)

        t0 = time.monotonic()
        depth, last_t = 1, cap_t
        while True:
            got = cam.try_dequeue()
            if got is None:
                break
            i, last_t, _n, _s = got
            cam.requeue(i)
            depth += 1
        drain_ms.append((time.monotonic() - t0) * 1000.0)
        depths.append(depth)
        drained.append((time.monotonic() - last_t) * 1000.0)

    print(f'    frames waiting        {st.median(depths):.1f} median '
          f'(of {cam.nbuf} buffers)')
    print(f'    FIRST frame\'s age     {st.median(first):6.2f} ms   '
          f'<- what a plain read() hands you')
    print(f'    FRESHEST after drain  {st.median(drained):6.2f} ms   '
          f'<- what a mailbox hands you')
    print(f'    cost to drain         {st.median(drain_ms):6.2f} ms '
          f'(dequeue+requeue, no decode)')
    saved = st.median(first) - st.median(drained)
    print(f'\n    -> draining recovers {saved:.1f} ms of staleness for '
          f'{st.median(drain_ms):.2f} ms of work')
    cam.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

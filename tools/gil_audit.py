#!/usr/bin/env python3
"""Every C extension this pipeline calls at rate, checked for GIL behaviour.

THE RULE THIS ENFORCES
----------------------
A C extension that does not release the GIL blocks EVERY other Python thread
in its process for as long as it runs. At 70 Hz that is not a slow function,
it is a periodic freeze of the whole process -- and it presents as a problem
somewhere else entirely.

`pyhailort`'s blocking `InferVStreams.infer()` held it for ~10 ms and the
symptom was a camera whose frames looked 13.8 ms stale. Two hypotheses about
the camera died before anyone looked at the inference call. So: audit the
callers, do not wait for the symptom.

WHAT COUNTS AS A PROBLEM
------------------------
`hold x rate` is the fraction of wall time the process is frozen. A 2 ms call
at 77 Hz is 15 % -- tolerable. A 10 ms call at 77 Hz is 77 %, which is a
process that barely runs anything else. The ranking is by that product, not by
duration, because duration alone says nothing without the rate.

THE CONTROLS ARE NOT OPTIONAL
-----------------------------
A gap number cannot distinguish "this call holds the GIL" from "this machine
schedules badly". The first version of this probe used a SPINNING sampler --
which competes for the very lock it measures -- and reported a median of
0.000 ms for a known GIL holder, i.e. it could not tell its own controls
apart. The sampler sleeps now, and both controls print first. If they do not
separate, the run is void and nothing below it means anything.
"""
import argparse
import os
import statistics as st
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..',
                                'src', 'mongla_vision'))

_TICK_S = 0.001


class _Sampler(threading.Thread):
    daemon = True

    def __init__(self):
        super().__init__()
        self.late = []
        self._run = True

    def run(self):
        while self._run:
            t0 = time.monotonic()
            time.sleep(_TICK_S)
            self.late.append((time.monotonic() - t0 - _TICK_S) * 1000.0)

    def stop(self):
        self._run = False


_ROWS = []


def _measure(label, work, seconds, rate_hz=None, note=''):
    s = _Sampler()
    s.start()
    time.sleep(0.25)
    s.late.clear()
    t_end = time.monotonic() + seconds
    n = 0
    try:
        while time.monotonic() < t_end:
            work()
            n += 1
    except Exception as exc:
        s.stop()
        print(f'  {label:<28} SKIPPED: {exc!r}'[:120])
        return
    s.stop()
    time.sleep(0.05)
    g = sorted(x for x in s.late if x >= 0)
    if not g or not n:
        print(f'  {label:<28} no samples')
        return
    med = st.median(g)
    p99 = g[min(len(g) - 1, int(0.99 * (len(g) - 1)))]
    blocked = 100.0 * sum(1 for x in g if x > 4.0) / len(g)
    call_ms = seconds / n * 1000.0
    # How much of a second the process is frozen, at the rate we really call
    # it. This is the number that decides whether a hold matters.
    duty = (call_ms * rate_hz / 10.0) if rate_hz else float('nan')
    _ROWS.append((label, med, duty, note))
    print(f'  {label:<28} call {call_ms:6.2f} ms  |  other thread late: '
          f'med {med:6.2f}  p99 {p99:6.2f}  >4ms {blocked:5.1f} %'
          + (f'  |  hold x {rate_hz:g}Hz = {duty:5.1f} %' if rate_hz else ''))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=3.0)
    ap.add_argument('--hef', default=os.path.expanduser(
        '~/hailo_models/gate_rescue_repair.hef'))
    ap.add_argument('--serial', default='/dev/ttyUSB0')
    a = ap.parse_args()

    print('\n  CONTROLS -- if these do not separate, the run is VOID\n')
    _measure('control: sleep (releases)', lambda: time.sleep(0.005), a.seconds)

    def _spin():
        t = time.monotonic() + 0.005
        while time.monotonic() < t:
            pass
    _measure('control: python spin (holds)', _spin, a.seconds)

    print('\n  THE PIPELINE\n')

    # ---- camera: MJPEG decode, once per published/inferred frame ----------
    import cv2
    img = (np.random.rand(360, 640, 3) * 255).astype(np.uint8)
    ok, jpg = cv2.imencode('.jpg', img)
    jpg_bytes = jpg.tobytes()
    _measure('cv2.imdecode (MJPEG)',
             lambda: cv2.imdecode(np.frombuffer(jpg_bytes, np.uint8),
                                  cv2.IMREAD_COLOR),
             a.seconds, rate_hz=77, note='camera, per consumed frame')

    # ---- detector: letterbox, once per inference -------------------------
    from mongla_vision.detection.hailo import letterbox
    _measure('letterbox (resize+pad)', lambda: letterbox(img, 640),
             a.seconds, rate_hz=77, note='detector, per inference')

    # ---- ROS: encode + serialise, once per published frame ---------------
    try:
        import rclpy
        from cv_bridge import CvBridge
        br = CvBridge()
        _measure('cv_bridge cv2_to_imgmsg',
                 lambda: br.cv2_to_imgmsg(img, encoding='bgr8'),
                 a.seconds, rate_hz=10, note='camera, viewer feed only')
    except Exception as exc:
        print(f'  cv_bridge                    SKIPPED: {exc!r}'[:110])

    # ---- control: the serial read the srot driver does at 200 Hz ---------
    try:
        import serial
        ser = serial.Serial(a.serial, 115200, timeout=0)
        _measure('pyserial read (srot link)',
                 lambda: (ser.in_waiting, ser.read(4096)),
                 a.seconds, rate_hz=200, note='control, reader thread')
        ser.close()
    except Exception as exc:
        print(f'  pyserial                     SKIPPED: {exc!r}'[:110])

    # ---- the chip: both APIs ---------------------------------------------
    try:
        from hailo_platform import HEF, VDevice, FormatType
        vdev = VDevice()
        hef = HEF(a.hef)
        size = int(hef.get_input_vstream_infos()[0].shape[0])
        model = vdev.create_infer_model(a.hef)
        model.input().set_format_type(FormatType.UINT8)
        model.output().set_format_type(FormatType.FLOAT32)
        cim = model.configure()
        cim.__enter__()
        cim.activate()
        b = cim.create_bindings()
        ib = np.zeros((size, size, 3), np.uint8)
        ob = np.zeros(model.output().shape, np.float32)
        b.input().set_buffer(ib)
        b.output().set_buffer(ob)

        def _async():
            cim.wait_for_async_ready(timeout_ms=1000)
            cim.run_async([b]).wait(1000)
        _async()
        _measure('hailo run_async (SHIPPING)', _async, a.seconds,
                 rate_hz=77, note='detector, per inference')
        cim.deactivate()
        cim.__exit__(None, None, None)
    except Exception as exc:
        print(f'  hailo                        SKIPPED: {exc!r}'[:110])

    # ---- verdict ----------------------------------------------------------
    print('\n  RANKED BY HOLD x RATE -- the fraction of wall time the process'
          '\n  is frozen. Duration alone means nothing without the rate.\n')
    for label, med, duty, note in sorted(
            _ROWS, key=lambda r: (-(r[2] if r[2] == r[2] else -1))):
        verdict = ('HOLDS' if med > 1.0 else 'frees')
        d = f'{duty:5.1f} %' if duty == duty else '    -- '
        print(f'    {d}  {verdict:5}  {label:<28} {note}')
    print()


if __name__ == '__main__':
    main()

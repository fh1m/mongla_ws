#!/usr/bin/env python3
"""Does the Hailo inference call hold the GIL for its whole duration?

WHY IT MATTERS
--------------
The composed vision process measured a frame WAITING 13.82 ms in the camera's
slot with only 1.93 ms of that being our decode. The loop period is 12.16 ms
and inference is 10.22 ms. If `infer()` holds the GIL, no other Python thread
in the process runs during it -- so the capture pump cannot publish a newer
frame, and the consumer necessarily picks up one from before the inference
started. That would make the ~12 ms an ARCHITECTURAL floor for anything
sharing the process, not a tuning problem.

If instead it releases the GIL, the 13.82 ms has another cause and this
hypothesis joins the two already discarded on this pipeline.

METHOD
------
A background thread samples `time.monotonic()` in a tight loop and records the
GAPS. A gap is how long that thread was not scheduled. Run it against:

    idle          -- the floor: OS scheduling and the 5 ms switch interval
    infer         -- the call under test
    time.sleep    -- a KNOWN GIL-releasing call, as a positive control
    a pure-Python spin -- a KNOWN GIL-holding call, as a negative control

The two controls are the point. A gap measurement without them cannot
distinguish "the call holds the GIL" from "this machine schedules badly",
which is exactly the mistake made once already on this hardware: a 76 ms
worst case was reported as a real jitter finding and turned out to be an
artifact of running control loops as threads in one process.
"""
import argparse
import os
import statistics as st
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..',
                                'src', 'duburi_vision'))


_TICK_S = 0.001          # what the sampler ASKS for


class _Sampler(threading.Thread):
    """Sleeps for a fixed tick and records how LATE it woke.

    Deliberately not a spin loop. A spinning sampler is itself a GIL hog: it
    competes with the thread under test, both threads ping-pong on the 5 ms
    switch interval, and the resulting distribution says more about the
    sampler than the subject. The first version of this file did exactly that
    and produced a median gap of 0.000 ms for a KNOWN GIL holder -- i.e. it
    could not tell its own controls apart.

    `time.sleep` releases the GIL, so this thread is descheduled cheaply and
    the only thing that can delay its WAKEUP is not getting the GIL back.
    Lateness is therefore GIL-wait, plus a small OS scheduling floor that the
    controls measure.
    """
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


def _measure(label, work, seconds):
    s = _Sampler()
    s.start()
    time.sleep(0.3)                      # let it settle
    s.late.clear()
    t_end = time.monotonic() + seconds
    n = 0
    while time.monotonic() < t_end:
        work()
        n += 1
    s.stop()
    time.sleep(0.05)
    g = sorted(x for x in s.late if x >= 0)
    if not g:
        print(f'  {label:<24} no samples')
        return
    p99 = g[min(len(g) - 1, int(0.99 * (len(g) - 1)))]
    # The headline number: how often another thread waited longer than a
    # frame period. A median hides this completely -- most ticks are fine and
    # the ones that are not are the whole problem.
    blocked = 100.0 * sum(1 for x in g if x > 4.0) / len(g)
    print(f'  {label:<24} calls={n:5d}  woke late: med {st.median(g):6.2f}  '
          f'p99 {p99:7.2f}  max {max(g):7.2f} ms   >4ms: {blocked:5.1f} %')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='gate_rescue_repair')
    ap.add_argument('--seconds', type=float, default=4.0)
    a = ap.parse_args()

    from duburi_vision.detection.factory import make_detector
    det = make_detector(model_path=a.model, conf=0.15, max_det=100,
                        class_allowlist=None, device='cpu', half=False)
    frame = np.zeros((360, 640, 3), np.uint8)
    det.infer(frame)                      # warm up

    print(f'\n  "woke late" = how long ANOTHER Python thread waited to be '
          f'scheduled.\n  Read the two controls first: if they do not '
          f'separate, the run is void.\n')
    _measure('CONTROL sleep (frees)', lambda: time.sleep(0.010), a.seconds)

    def _spin():
        # ~10 ms of pure Python: the KNOWN GIL holder.
        t = time.monotonic() + 0.010
        x = 0
        while time.monotonic() < t:
            x += 1

    _measure('CONTROL spin (HOLDS)', _spin, a.seconds)
    _measure('hailo infer', lambda: det.infer(frame), a.seconds)
    print()
    print('  If `hailo infer` looks like CONTROL spin, the chip call holds the')
    print('  GIL and no Python thread in this process runs during inference.')
    print('  If it looks like CONTROL sleep, it releases and the 13.8 ms of')
    print('  slot wait has another cause.')
    print()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""How late a periodic Python loop wakes on this kernel, idle and loaded.

Our host loops are Python and the fastest is 50 Hz (a 20 ms period): the vision
verbs' MANUAL_CONTROL stream. This runs a loop on absolute deadlines and
records how late each wake-up was, under the default scheduler and under
SCHED_FIFO, with and without every core busy.

The bar, written before the first run (2026-11-11):

    A realtime (PREEMPT_RT) kernel is worth changing the vehicle's kernel for
    only if, with every core busy, a SCHED_FIFO loop on the stock kernel is
    later than 1 ms at p99.9 or 2 ms at worst (10 % of the 20 ms period).
    Inside that, the fix is a realtime priority on the thread that needs it,
    which costs one syscall.

What would falsify the bench: under load the default scheduler must be clearly
later than SCHED_FIFO. If the two match, the load is not competing with the
loop and the run says nothing.

    python3 tools/bench/wake_latency.py                 # default scheduler, idle
    python3 tools/bench/wake_latency.py --load 4        # every core busy
    sudo python3 tools/bench/wake_latency.py --fifo 80 --load 4
    sudo python3 tools/bench/wake_latency.py --fifo 80 --threads 2 --switch-ms 1
    python3 tools/bench/wake_latency.py --self-test
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
import sys
import threading
import time

# MAVLink 2 is chosen before pymavlink is first imported, at module level, as
# every importer does (test_every_pymavlink_importer_selects_mavlink2.py).
os.environ['MAVLINK20'] = '1'

BAR_P999_MS = 1.0
BAR_MAX_MS = 2.0


def verdict(row: dict) -> tuple[bool, str]:
    """True when the loop is inside the bar (no realtime kernel needed)."""
    ok = row['p999_ms'] <= BAR_P999_MS and row['max_ms'] <= BAR_MAX_MS
    return ok, (f"p99.9 {row['p999_ms']:.3f} ms (bar {BAR_P999_MS}), "
                f"max {row['max_ms']:.3f} ms (bar {BAR_MAX_MS})")


def summarise(late_ns: list[int]) -> dict:
    xs = sorted(late_ns)
    q = lambda f: xs[min(len(xs) - 1, int(f * len(xs)))] / 1e6      # noqa: E731
    return {'n': len(xs), 'p50_ms': q(0.50), 'p99_ms': q(0.99),
            'p999_ms': q(0.999), 'max_ms': xs[-1] / 1e6}


def _burn(stop) -> None:
    x = 0
    while not stop.is_set():
        for _ in range(100000):
            x += 1


def _reader(stop) -> None:
    """The manager's reader thread: parse a full 115200-baud MAVLink stream.

    A serial port at 115200 delivers at most 11 520 bytes a second, so every
    10 ms this parses the 115 bytes that could have arrived. It is the real
    competitor for the interpreter lock in the process that streams
    MANUAL_CONTROL.
    """
    from pymavlink.dialects.v20 import ardupilotmega as mav
    enc = mav.MAVLink(None, srcSystem=1, srcComponent=1)
    blob = b''.join(m.pack(enc) for m in (
        mav.MAVLink_attitude_message(1, .1, .2, .3, .01, .02, .03),
        mav.MAVLink_scaled_imu2_message(1, 1, 2, 3, 4, 5, 6, 7, 8, 9),
        mav.MAVLink_vfr_hud_message(0., 0., 90, 0, -1.2, 0.),
        mav.MAVLink_named_value_float_message(1, b'DEPTH_OUT', 0.5))) * 40
    dec = mav.MAVLink(None)
    dec.robust_parsing = True
    at, nxt = 0, time.monotonic()
    while not stop.is_set():
        dec.parse_buffer(blob[at:at + 115])
        at = (at + 115) % (len(blob) - 115)
        nxt += 0.010
        time.sleep(max(0.0, nxt - time.monotonic()))


def run(hz: float, seconds: float, fifo: int, load: int, threads: int = 0,
        switch_ms: float = 0.0, reader: bool = False) -> dict:
    stop = mp.Event()
    procs = [mp.Process(target=_burn, args=(stop,), daemon=True) for _ in range(load)]
    for p in procs:
        p.start()
    # Busy threads in THIS process hold the interpreter lock in turn, which a
    # realtime priority cannot preempt: the manager's own shape.
    if switch_ms:
        sys.setswitchinterval(switch_ms / 1000.0)
    for _ in range(threads):
        threading.Thread(target=_burn, args=(stop,), daemon=True).start()
    if reader:
        threading.Thread(target=_reader, args=(stop,), daemon=True).start()
    if fifo:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(fifo))
    period = int(1e9 / hz)
    late = []
    nxt = time.monotonic_ns() + period
    end = nxt + int(seconds * 1e9)
    while nxt < end:
        dt = (nxt - time.monotonic_ns()) / 1e9
        if dt > 0:
            time.sleep(dt)
        late.append(time.monotonic_ns() - nxt)
        nxt += period
    stop.set()
    row = summarise(late)
    # When the worst wake-up happened, and how many were over the bar: one
    # late wake-up at start-up is a different fact from one mid-run.
    row['max_at_s'] = late.index(max(late)) / hz
    row['over_bar'] = sum(1 for x in late if x > BAR_MAX_MS * 1e6)
    row.update(hz=hz, fifo=fifo, load=load, threads=threads, reader=reader,
               switch_ms=switch_ms or sys.getswitchinterval() * 1000.0,
               kernel=os.uname().release)
    return row


def _self_test() -> int:
    s = summarise([1000] * 998 + [500_000, 3_000_000])
    bad = []
    if not (s['p50_ms'] == 0.001 and s['max_ms'] == 3.0):
        bad.append(s)
    if verdict({'p999_ms': 0.5, 'max_ms': 1.9})[0] is not True:
        bad.append('inside the bar read as outside')
    if verdict({'p999_ms': 0.5, 'max_ms': 2.1})[0] is not False:
        bad.append('a late worst case read as inside')
    if verdict({'p999_ms': 1.1, 'max_ms': 1.2})[0] is not False:
        bad.append('a late p99.9 read as inside')
    print('self-test', 'FAIL' if bad else 'ok', bad or '')
    return 1 if bad else 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--hz', type=float, default=50.0)
    ap.add_argument('--seconds', type=float, default=30.0)
    ap.add_argument('--fifo', type=int, default=0, help='SCHED_FIFO priority; 0 = default scheduler')
    ap.add_argument('--load', type=int, default=0, help='busy processes beside the loop')
    ap.add_argument('--threads', type=int, default=0,
                    help='busy threads inside this process (they share the interpreter lock)')
    ap.add_argument('--reader', action='store_true',
                    help='a thread parsing a full-rate MAVLink stream, as the manager has')
    ap.add_argument('--switch-ms', type=float, default=0.0,
                    help='sys.setswitchinterval, ms; 0 = leave the default 5 ms')
    ap.add_argument('--self-test', action='store_true')
    a = ap.parse_args()
    if a.self_test:
        return _self_test()
    row = run(a.hz, a.seconds, a.fifo, a.load, a.threads, a.switch_ms, a.reader)
    ok, why = verdict(row)
    print(json.dumps(row))
    print(('inside the bar: ' if ok else 'OUTSIDE the bar: ') + why)
    return 0


if __name__ == '__main__':
    sys.exit(main())

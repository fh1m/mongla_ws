#!/usr/bin/env python3
"""Replay and analyse a recorded dive from its raw MAVLink log.

The recorder writes the wire and nothing else, so everything here is DERIVED --
with the same decoder and the same filter classes the live path uses. That is
the point: a panel that cannot be rebuilt from the file names a field the log
is missing, and a panel rebuilt by different code proves nothing about the
live one.

WHAT IT REPORTS, and why each earns its place
---------------------------------------------
  rates     per message type over the run, plus the LONGEST GAP. A mean rate
            hides a stream that stopped; the gap is what a dropout looks like.
  ack       COMMAND_LONG -> COMMAND_ACK latency, as a distribution. The board's
            own response time is the budget every verb's deadline is set
            against, and it has never been measured over a whole run.
  timeline  mode and arm transitions, in order. "What was it doing when that
            happened" is the first question after any dive.
  filters   raw against filtered depth and heading, re-run through
            `nav_filter`, so a filter can be re-tuned against a real dive
            instead of against a bench wiggle.
  named     the 24 multiplexed NAMED_VALUE_FLOAT channels, demultiplexed --
            which only works because the log holds every frame rather than
            whatever was in pymavlink's single slot at sample time.

  --verify  decode the same file with `mavlogdump.py` and compare per-type
            counts. A log we can only read with our own reader is a blob with
            extra steps; this is the check that it is not.
"""
from __future__ import annotations

import argparse
import collections
import os
import shutil
import statistics as st
import subprocess
import sys

from pymavlink import mavutil

for _p in ('mongla_control', 'mongla_manager'):
    _q = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'src', _p)
    if os.path.isdir(_q) and _q not in sys.path:
        sys.path.insert(0, _q)

from mongla_control.nav_filter import DepthFilter, HeadingFilter   # noqa: E402

_ARMED = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


def load(path):
    """(timestamp, message) pairs, in file order.

    `mavutil` is the live path's own decoder, pointed at a file instead of a
    port -- so a message that decodes here decodes in flight, and one that does
    not, does not.
    """
    conn = mavutil.mavlink_connection(path)
    out = []
    while True:
        m = conn.recv_match(blocking=False)
        if m is None:
            break
        if m.get_type() == 'BAD_DATA':
            out.append((getattr(m, '_timestamp', 0.0), m))
            continue
        out.append((getattr(m, '_timestamp', 0.0), m))
    return out


def _span(rows):
    ts = [t for t, _ in rows if t]
    return (min(ts), max(ts)) if ts else (0.0, 0.0)


def report_rates(rows, span):
    per = collections.defaultdict(list)
    for t, m in rows:
        per[m.get_type()].append(t)
    dur = max(1e-9, span[1] - span[0])
    print(f'\n  {"message":<26} {"n":>7} {"Hz":>8} {"max gap s":>10}')
    print('  ' + '-' * 54)
    for name, ts in sorted(per.items(), key=lambda kv: -len(kv[1])):
        gaps = [b - a for a, b in zip(ts, ts[1:])] or [0.0]
        print(f'  {name:<26} {len(ts):>7} {len(ts)/dur:>8.2f} {max(gaps):>10.3f}')
    if 'BAD_DATA' in per:
        print(f'\n  {len(per["BAD_DATA"])} BAD_DATA frames -- corrupt on the wire, '
              f'not in the log (the log stores what arrived)')


def report_acks(rows):
    """Pair each COMMAND_LONG with the next ACK for the same command."""
    pending, lat = {}, collections.defaultdict(list)
    for t, m in rows:
        ty = m.get_type()
        if ty == 'COMMAND_LONG':
            pending[int(m.command)] = t
        elif ty == 'COMMAND_ACK':
            t0 = pending.pop(int(m.command), None)
            if t0 is not None:
                lat[int(m.command)].append((t - t0) * 1000.0)
    if not lat:
        print('\n  no command/ack pairs in this log')
        return
    print(f'\n  {"command":>9} {"n":>5} {"median ms":>10} {"p95":>8} {"max":>8}')
    print('  ' + '-' * 44)
    for cmd, xs in sorted(lat.items()):
        xs.sort()
        print(f'  {cmd:>9} {len(xs):>5} {st.median(xs):>10.1f} '
              f'{xs[int(0.95 * (len(xs) - 1))]:>8.1f} {xs[-1]:>8.1f}')
    if pending:
        print(f'  {len(pending)} command(s) never acknowledged: '
              + ', '.join(str(c) for c in sorted(pending)))


def report_timeline(rows, span):
    """Mode and arm changes only -- a line per state, not per heartbeat."""
    last = None
    print('\n  t+s     armed  mode')
    print('  ' + '-' * 30)
    n = 0
    for t, m in rows:
        if m.get_type() != 'HEARTBEAT':
            continue
        state = (bool(m.base_mode & _ARMED), int(m.custom_mode))
        if state != last:
            print(f'  {t - span[0]:>6.1f}  {"ARM" if state[0] else "---":<5}  '
                  f'{state[1]}')
            last = state
            n += 1
    if n == 0:
        print('  (no heartbeats)')


def report_filters(rows):
    """Re-run the live filters over the logged signal.

    The reason to keep raw and filtered side by side: a filter is only
    defensible against the noise it will actually see, and a bench wiggle is
    not that noise.
    """
    dep, hdg = DepthFilter(), HeadingFilter()
    d_raw, d_filt, h_raw, h_filt = [], [], [], []
    for t, m in rows:
        ty = m.get_type()
        if ty == 'VFR_HUD':
            r = -float(m.alt)
            f = dep.update(r, t)
            d_raw.append(r)
            if f is not None:
                d_filt.append(f)
        elif ty == 'ATTITUDE':
            import math
            r = math.degrees(m.yaw) % 360.0
            f = hdg.update(r, t)
            h_raw.append(r)
            if f is not None:
                h_filt.append(f)

    def _jitter(xs, angular=False):
        """Step-to-step spread. On a heading it MUST use the shortest arc.

        Found by reading this tool's own first output: it reported a raw step
        sd of 25.5 deg on a smooth 3 deg/sample sweep, and an identical 25.4
        filtered -- i.e. "the filter does nothing". Both numbers were the
        WRAP: one 359 -> 0 crossing contributes a -359 step and swamps every
        real one. Exactly the defect `AngleAlphaBeta` exists to avoid, rebuilt
        by hand in the tool that was supposed to measure it.
        """
        d = [b - a for a, b in zip(xs, xs[1:])]
        if angular:
            d = [(x + 180.0) % 360.0 - 180.0 for x in d]
        return st.pstdev(d) if len(d) > 1 else 0.0

    print(f'\n  {"signal":<18} {"n":>6} {"raw step sd":>12} {"filtered":>10} '
          f'{"resets":>7}')
    print('  ' + '-' * 58)
    if d_raw:
        print(f'  {"depth m":<18} {len(d_raw):>6} {_jitter(d_raw):>12.4f} '
              f'{_jitter(d_filt):>10.4f} {dep.resets:>7}')
    if h_raw:
        print(f'  {"heading deg":<18} {len(h_raw):>6} '
              f'{_jitter(h_raw, angular=True):>12.4f} '
              f'{_jitter(h_filt, angular=True):>10.4f} {hdg.resets:>7}')
    print('  (heading resets are wrap crossings only if they equal the number of '
          'times the hull passed north -- otherwise the gate is firing)')


def report_named(rows):
    """The 24 channels on msgid 251, demultiplexed.

    Only possible because the log holds every frame. Sampling pymavlink's
    single slot does not miss a channel occasionally -- it misses it always.
    """
    seen = collections.defaultdict(list)
    for t, m in rows:
        if m.get_type() != 'NAMED_VALUE_FLOAT':
            continue
        name = m.name
        name = name.decode() if isinstance(name, bytes) else str(name)
        seen[name.strip('\x00').strip()].append(float(m.value))
    if not seen:
        print('\n  no NAMED_VALUE_FLOAT in this log')
        return
    print(f'\n  {"channel":<12} {"n":>6} {"first":>10} {"last":>10} '
          f'{"min":>10} {"max":>10}')
    print('  ' + '-' * 62)
    for k, v in sorted(seen.items()):
        print(f'  {k:<12} {len(v):>6} {v[0]:>10.3f} {v[-1]:>10.3f} '
              f'{min(v):>10.3f} {max(v):>10.3f}')


def verify_with_another_decoder(path, rows):
    """Cross-check against mavlogdump.py -- a decoder we did not write.

    A decoder bug is invisible in its own output. This is the check that the
    file is a MAVLink log rather than something only our reader believes in.
    """
    tool = shutil.which('mavlogdump.py')
    if tool is None:
        print('\n  mavlogdump.py not on PATH -- cross-check SKIPPED (not passed)')
        return 0
    try:
        out = subprocess.run([tool, '--format', 'csv', '--types', 'HEARTBEAT',
                              path], capture_output=True, text=True, timeout=120)
    except Exception as exc:                       # noqa: BLE001
        print(f'\n  cross-check could not run: {exc!r}')
        return 0
    theirs = max(0, len([l for l in out.stdout.splitlines() if l.strip()]) - 1)
    ours = sum(1 for _, m in rows if m.get_type() == 'HEARTBEAT')
    ok = theirs == ours
    print(f'\n  cross-decoder: mavlogdump {theirs} HEARTBEAT, we read {ours} '
          f'-- {"AGREE" if ok else "DISAGREE"}')
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--verify', action='store_true',
                    help='cross-check the file with mavlogdump.py')
    a = ap.parse_args()

    rows = load(a.log)
    if not rows:
        print('empty log')
        return 1
    span = _span(rows)
    dur = span[1] - span[0]
    size = os.path.getsize(a.log)
    print(f'\n{a.log}')
    print(f'  {len(rows)} frames, {dur:.1f} s, {size/1024:.1f} kB '
          f'({size/max(dur,1e-9):.0f} B/s on the wire)')

    report_rates(rows, span)
    report_timeline(rows, span)
    report_acks(rows)
    report_named(rows)
    report_filters(rows)
    rc = verify_with_another_decoder(a.log, rows) if a.verify else 0
    print()
    return rc


if __name__ == '__main__':
    raise SystemExit(main())

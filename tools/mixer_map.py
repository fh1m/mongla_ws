#!/usr/bin/env python3
"""Measure what the mixer ACTUALLY does, using the Pico's console as readback.

⛔ WHY THIS EXISTS. `srot_fc.py` states plainly that this backend has no output
readback -- "SROT has no RC-channel readback (MANUAL_CONTROL only)" -- and the
SOTA ledger rates the allocation gap BLOCKED on that: the achieved wrench is
computed on the board every tick and, as far as the MAVLink wire is concerned,
transmitted nowhere. Every published anti-windup design in this family needs it.

It is not transmitted nowhere. It is on the OTHER USB device. The board presents
two:

    /dev/ttyUSB0   CH340 (1a86:7523)    the ESP32. MAVLink.
    /dev/ttyACM0   Pico 2 (2e8a:000f)   the RP2350's ESC console, which prints
                                        every motor's commanded and output value
                                        at 2 Hz:

    link=1 armed=0 bidir=1 rpm_mode=0 loop=0 | M1[cmd=0 out=0 rpm=0 pres=0 e=0
    d=0 c=0 n=998] M2[...] ... M8[...]

So the readback exists, on a console nobody connected, in text, at 2 Hz. That is
enough to measure the mixer on a bench with NO THRUSTERS ATTACHED, because a
mixer is a linear map from six axis demands to eight motor commands and none of
that involves force.

WHAT THIS MEASURES (no thrusters, no water, no load cell):
    * the mixer matrix B, column by column: drive one axis, read eight outputs
    * the effective sign of every motor -- the product of CAL_MDIRn,
      MOT_n_DIRECTION and FRAME_REVERSE, which no display anywhere shows
    * the COMMAND deadband: the demand below which `out` stays 0

WHAT IT CANNOT MEASURE. Force. Thrust per RPM, the physical stiction floor, and
the current signature of a dead thruster all need a motor on the end of the
wire. Do not read a column of this matrix as newtons.

    python3 tools/mixer_map.py --dry          # disarmed: does anything move?
    python3 tools/mixer_map.py --arm          # the real sweep
"""
from __future__ import annotations

import argparse
import json
import re
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

# ⚠ BY-ID, NOT THE NUMBERED NODE. Measured on 2026-09-22: the CH340 re-enumerates
# spontaneously -- it vanished mid-sweep, came back as /dev/ttyUSB1, then as
# /dev/ttyUSB0 again. A tool holding the number loses the board, or later opens
# whatever takes the freed number. These names are stable across re-enumeration,
# which is the same reason `connection_config.py` matches on them.
MAV_GLOB = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'   # ESP32, MAVLink
PICO_GLOB = '/dev/serial/by-id/usb-Raspberry_Pi_Pico_2_*-if00'  # RP2350 console
BAUD = 115200


def _resolve(pattern: str, what: str) -> str:
    from glob import glob
    hits = sorted(glob(pattern))
    if not hits:
        raise SystemExit(f'no {what} at {pattern} -- is the board plugged in?')
    if len(hits) > 1:
        raise SystemExit(f'{len(hits)} candidates for {what}: {hits}. Refusing '
                         f'to pick one.')
    return hits[0]

AXES = ('fwd', 'lat', 'up', 'yaw')
HOLD_S = 4.0                   # the Pico prints at 2 Hz, so hold for >= 4 lines
SETTLE_S = 1.5                 # let the previous demand wash out before reading
MC_AXIS_MAX, MC_Z_NEUTRAL, MC_Z_MAX = 1000, 500, 1000

_MOTOR = re.compile(
    r'M(\d+)\[cmd=(-?\d+) out=(-?\d+) rpm=(-?\d+) pres=(-?\d+) '
    r'e=(\d+) d=(\d+) c=(\d+) n=(\d+)\]')
_HEAD = re.compile(r'link=(\d+) armed=(\d+) bidir=(\d+) rpm_mode=(\d+) loop=(\d+)')


# ⚠ BIDIRECTIONAL DSHOT IS TWO HALVES, NOT A SIGNED RANGE AROUND A MIDPOINT.
#
#     48 .. 1047    direction A, magnitude = v - 48
#   1048 .. 2047    direction B, magnitude = v - 1048
#
# Both halves count UPWARD from their own floor, so the two zeros are 48 and
# 1048 -- there is no single neutral. Reading the console as "deviation from
# 1048" makes every direction-A motor look like a large negative number: at a
# 0.02 demand M1 reads 229 and M2 reads 1230, which are magnitudes 181 and 182
# in opposite directions, NOT -819 and +182. Decoded the wrong way the mixer
# comes out asymmetric and the heave row appears to have a 3 % imbalance that
# does not exist. Measured 2026-09-22.
DSHOT_A_MIN, DSHOT_B_MIN, DSHOT_SPAN = 48, 1048, 999

# ⚠ THIS IS A SECOND COPY, DELIBERATELY. The decoder also lives in
# `mongla_control.actuation_model`, which is where it belongs -- but this tool is
# scp'd to the vehicle and run from /tmp, where the workspace is not importable,
# and it needs the vectorised (numpy) form that the scalar module does not
# provide. Importing it conditionally and then not using it would be worse than
# this: it would LOOK like one truth while still being two.
#
# So the copy is admitted, and `test_dshot_decode.py` compares the two across the
# whole of both bands. The test is the thing that keeps them honest, not the
# import. Injection-verified: shifting a band floor here by one fails it.
def dshot_signed(v):
    """Console value -> signed magnitude in [-999, +999]. 48 and 1048 are both 0."""
    import numpy as np
    v = np.asarray(v, dtype=float)
    return np.where(v >= DSHOT_B_MIN, v - DSHOT_B_MIN, -(v - DSHOT_A_MIN))


# ─────────────────────────────────────────────────────── the Pico console ──

class PicoConsole:
    """Reads the RP2350's telemetry lines in a thread. Read-only: this never
    writes to the Pico. It is a debug console on a microcontroller that drives
    DShot, and we have no contract for what it accepts."""

    def __init__(self, port: str | None = None):
        import serial
        self.port = port or _resolve(PICO_GLOB, 'Pico console')
        self.ser = serial.Serial(self.port, BAUD, timeout=1.0)
        self.lines: list[tuple[float, str]] = []
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        buf = b''
        while not self._stop.is_set():
            buf += self.ser.read(256)
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                text = raw.decode('utf8', 'replace').strip()
                if text:
                    self.lines.append((time.time(), text))

    def __enter__(self):
        self._t.start()
        return self

    def __exit__(self, *_):
        self._stop.set()
        self._t.join(timeout=2.0)
        self.ser.close()

    def since(self, t0: float) -> list[dict]:
        """Every complete motor report stamped after t0."""
        out = []
        for t, text in list(self.lines):
            if t < t0:
                continue
            motors = _MOTOR.findall(text)
            if len(motors) != 8:
                continue          # a torn line: do not half-read it
            head = _HEAD.search(text)
            out.append({
                't': t,
                'armed': int(head.group(2)) if head else None,
                'cmd': [int(m[1]) for m in motors],
                'out': [int(m[2]) for m in motors],
                'rpm': [int(m[3]) for m in motors],
                'n': [int(m[8]) for m in motors],
                'e': [int(m[5]) for m in motors],
            })
        return out


# ──────────────────────────────────────────────────────────── the board ──

def _unit_to_mc(u: float) -> int:
    return int(round(max(-1.0, min(1.0, u)) * MC_AXIS_MAX))


def _unit_to_mc_z(u: float) -> int:
    return int(round(MC_Z_NEUTRAL + max(-1.0, min(1.0, u)) * MC_Z_NEUTRAL))


class Board:
    def __init__(self, port: str | None = None):
        from pymavlink import mavutil
        port = port or _resolve(MAV_GLOB, 'SROT MAVLink port')
        # The enums live under mavutil, not as a top-level `pymavlink.mavlink`
        # module -- importing that name fails at runtime, and it failed inside
        # the heartbeat thread and the disarm path, which is the worst place.
        self.enums = mavutil.mavlink
        self.mav = mavutil.mavlink_connection(port, baud=BAUD)
        self.mav.wait_heartbeat(timeout=10)
        self._stop = threading.Event()
        self._hb = threading.Thread(target=self._beat, daemon=True)
        self._hb.start()

    def _beat(self) -> None:
        """⛔ The board surfaces after 5 s of silence. Nothing in this tool may
        block long enough to break this."""
        while not self._stop.is_set():
            try:
                self.mav.mav.heartbeat_send(
                    self.enums.MAV_TYPE_GCS, self.enums.MAV_AUTOPILOT_INVALID,
                    0, 0, 0)
            except Exception:
                pass
            time.sleep(1.0)

    def param(self, name: str, timeout: float = 2.0):
        self.mav.mav.param_request_read_send(1, 1, name.encode(), -1)
        t0 = time.time()
        while time.time() - t0 < timeout:
            m = self.mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
            if m and m.param_id.strip('\x00') == name:
                return m.param_value
        return None

    def manual(self, fwd=0.0, lat=0.0, up=0.0, yaw=0.0) -> None:
        self.mav.mav.manual_control_send(
            1, _unit_to_mc(fwd), _unit_to_mc(lat), _unit_to_mc_z(up),
            _unit_to_mc(yaw), 0)

    def stream(self, seconds: float, **axes) -> None:
        """Hold one demand at 20 Hz, the rate the servo path uses."""
        t_end = time.time() + seconds
        while time.time() < t_end:
            self.manual(**axes)
            time.sleep(0.05)

    def set_mode(self, mode_int: int) -> None:
        self.mav.mav.set_mode_send(
            1, self.enums.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_int)

    def arm(self, on: bool) -> None:
        self.mav.mav.command_long_send(
            1, 1, self.enums.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1 if on else 0, 0, 0, 0, 0, 0, 0)

    def close(self) -> None:
        self._stop.set()
        self._hb.join(timeout=2.0)


# ───────────────────────────────────────────────────────── the experiment ──

def read_config(b: Board) -> dict:
    """The three places a motor's sign lives, read live rather than assumed."""
    cfg = {'CAL_MDIR': [], 'MOT_DIRECTION': []}
    for i in range(1, 9):
        cfg['CAL_MDIR'].append(b.param(f'CAL_MDIR{i}'))
        cfg['MOT_DIRECTION'].append(b.param(f'MOT_{i}_DIRECTION'))
    for name in ('FRAME_REVERSE', 'MOT_SPIN_MIN', 'PILOT_EXPO'):
        cfg[name] = b.param(name)
    return cfg


def sweep(b: Board, pico: PicoConsole, *, armed: bool,
          levels=(0.3, -0.3)) -> list[dict]:
    """Drive one axis at a time and record what the eight motors were told."""
    rows = []
    for axis in AXES:
        for level in levels:
            b.stream(SETTLE_S, **{axis: 0.0})
            t0 = time.time()
            b.stream(HOLD_S, **{axis: level})
            reports = pico.since(t0 + 0.5)
            rows.append({'axis': axis, 'level': level, 'armed_requested': armed,
                         'reports': reports})
            print(f'  {axis:>4} {level:+.2f}: {len(reports)} reports'
                  + (f'  cmd={reports[-1]["cmd"]}' if reports else '  (none)'),
                  flush=True)
    b.stream(SETTLE_S)
    return rows


def ladder(b: Board, pico: PicoConsole, axis: str = 'fwd') -> list[dict]:
    """Demand ladder on one axis: what does the board DO with the number we send?

    Two rows close on this (both were assumptions):

    G-04  `PILOT_EXPO` is 0.30 and the vision loop divides nothing out, so the
          gain it thinks it has is not the gain it gets. If the echoed curve is
          straight, the parameter does not do what the doc says.
    B-2   the COMMAND deadband -- the demand below which the motor is told
          nothing at all. NOT the physical stiction floor, which needs a motor.
          `VISION_YAW_MIN_PCT` is 5.0 and `MOT_SPIN_MIN` is 0.15; whether a 5 %
          demand survives to the wire is measurable here and was never measured.
    """
    rows = []
    levels = (0.0, 0.02, 0.05, 0.08, 0.10, 0.15, 0.20, 0.30,
              0.45, 0.60, 0.80, 1.00)
    print(f'\nDEMAND LADDER on {axis}:')
    print('        ' + ' '.join(f'M{i + 1}'.rjust(5) for i in range(8)))
    for lv in levels:
        b.stream(SETTLE_S, **{axis: 0.0})
        t0 = time.time()
        b.stream(HOLD_S, **{axis: lv})
        reps = pico.since(t0 + 0.5)
        cmd = [r['cmd'] for r in reps]
        out = [r['out'] for r in reps]
        med = ([sorted(c[i] for c in cmd)[len(cmd) // 2] for i in range(8)]
               if cmd else None)
        medo = ([sorted(c[i] for c in out)[len(out) // 2] for i in range(8)]
                if out else None)
        rows.append({'axis': axis, 'level': lv, 'cmd_median': med,
                     'out_median': medo, 'n_reports': len(reps)})
        # ALL EIGHT. `up` drives M5..M8 and leaves M1..M4 at neutral, so a
        # printout of the first four shows a column of 1048 and hides the
        # entire result -- which is exactly what it did on the first run.
        print(f'  {lv:4.2f}: cmd '
              + (' '.join(f'{v:5d}' for v in med) if med else '(none)'),
              flush=True)
    b.stream(SETTLE_S)
    return rows


def summarise(rows: list[dict]) -> None:
    print('\nmeasured mixer -- per-motor `cmd` at each axis demand')
    print(f'{"axis":>5} {"level":>6}  ' + ' '.join(f'M{i}'.rjust(6) for i in range(1, 9)))
    moved = False
    for r in rows:
        if not r['reports']:
            print(f'{r["axis"]:>5} {r["level"]:+6.2f}  (no console reports)')
            continue
        last = r['reports'][-1]['cmd']
        moved = moved or any(v != 0 for v in last)
        print(f'{r["axis"]:>5} {r["level"]:+6.2f}  '
              + ' '.join(str(v).rjust(6) for v in last))
    if not moved:
        print('\n⛔ EVERY motor command stayed 0 for every axis. Either the mode '
              'discards MANUAL_CONTROL (AUTO and SURFACE both do, silently), or '
              'the mixer does not run while disarmed. This is a refusal, not a '
              'measurement: nothing here says what the mixer would do.')


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--arm', action='store_true',
                    help='arm before sweeping (PROPELLERS MUST BE CLEAR)')
    ap.add_argument('--dry', action='store_true',
                    help='disarmed only -- does the mixer preview at all?')
    ap.add_argument('--ladder', metavar='AXIS',
                    help='demand ladder on one axis (implies --arm)')
    ap.add_argument('--out', default='mixer_map.json')
    a = ap.parse_args()
    if a.ladder:
        a.arm = True
    if not (a.arm or a.dry):
        ap.error('pass --dry, --arm or --ladder AXIS')

    pico = PicoConsole()
    b = Board()
    result = {'taken': datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')}
    try:
        with pico:
            time.sleep(2.0)
            if not pico.since(0):
                print(f'⛔ no motor reports on {pico.port} in 2 s -- is this the '
                      f'Pico console? Refusing to guess.')
                return 2
            result['config'] = read_config(b)
            print('live direction configuration:')
            for k, v in result['config'].items():
                print(f'  {k:<14} {v}')

            b.set_mode(0)                     # STABILIZE: the only mode that
            time.sleep(1.0)                   # honours MANUAL_CONTROL fully
            if not a.ladder:
                print('\nDISARMED sweep:')
                result['disarmed'] = sweep(b, pico, armed=False)
                summarise(result['disarmed'])

            if a.arm:
                print('\nARMING -- propellers must be clear.')
                b.arm(True)
                time.sleep(2.0)
                armed_now = pico.since(time.time() - 1.5)
                state = armed_now[-1]['armed'] if armed_now else None
                print(f'  Pico reports armed={state}')
                if state != 1:
                    print('⛔ the board did not arm. Not sweeping: a sweep that '
                          'reads zeros because arming failed looks exactly like '
                          'a mixer that does nothing.')
                elif a.ladder:
                    result['ladder'] = ladder(b, pico, a.ladder)
                else:
                    print('\nARMED sweep:')
                    result['armed'] = sweep(b, pico, armed=True)
                    summarise(result['armed'])
    finally:
        # ⛔ SAFETY RULE 1: always have a disarm path. This runs on every exit,
        # including an exception and a Ctrl-C.
        try:
            b.stream(0.5)
            b.arm(False)
            time.sleep(0.5)
            print('\ndisarmed')
        except Exception as exc:
            print(f'⚠ DISARM FAILED: {exc} -- kill the board power')
        b.close()

    Path(a.out).write_text(json.dumps(result, indent=2))
    print(f'wrote {a.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

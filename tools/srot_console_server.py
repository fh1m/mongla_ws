#!/usr/bin/env python3
"""Data plane for the Mongla console: everything the stack knows, once per tick.

Split from the page deliberately. The page is a view; this is the only thing
that touches hardware, and it is the only thing that must never be paced by the
view. The 31.9-vs-50 Hz lesson from the bench applies to every panel added
here: rendering and encoding belong off the control path.

WHAT IS COLLECTED, and where each number comes from
---------------------------------------------------
  vision   camera grab / Hailo infer / decode timing, detection rate, the
           target, raw and filtered bearing
  board    ATTITUDE (roll/pitch/yaw + rates), SCALED_IMU2 (accel/gyro/mag),
           VFR_HUD (depth, heading), SCALED_PRESSURE2, SYS_STATUS,
           BATTERY_STATUS, POWER_STATUS, ESC_TELEMETRY x8, and all 24
           NAMED_VALUE_FLOAT channels
  link     measured bytes/s against the 11520 B/s the 115200 8N1 line carries,
           per-message rates, and the board's own drop/error counters
  host     Pi CPU, temperature, ARM clock, throttle word, memory
  filters  bearing / depth / heading, raw and filtered side by side

Anything not present is reported ABSENT rather than zero. On this board that
distinction is load-bearing: DEPTH_ERR and DEPTH_OUT are SUPPRESSED while the
depth controller is not running, so a zero would read as "settled" for a loop
that is not running at all.
"""
from __future__ import annotations

import collections
import math
import os
import re
import subprocess
import sys
import threading
import time

WS = os.path.expanduser('~/duburi_ws/src')
for _pkg in ('duburi_control', 'duburi_vision', 'duburi_manager'):
    _p = os.path.join(WS, _pkg)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from pymavlink import mavutil                                   # noqa: E402
from duburi_control.fc import srot_protocol as sp               # noqa: E402
from duburi_control.fc.srot_fc import SrotFC                    # noqa: E402
from duburi_control.nav_filter import DepthFilter, HeadingFilter  # noqa: E402

SERIAL_CAPACITY_BPS = 115200 / 10.0     # 8N1 -> 10 bits per byte


class Ring:
    """Fixed-length history. One per traced signal."""

    __slots__ = ('n', 'buf')

    def __init__(self, n=240):
        self.n = n
        self.buf = collections.deque(maxlen=n)

    def push(self, v):
        self.buf.append(v)

    def list(self):
        return list(self.buf)


class RateMeter:
    """Messages per second, per type, over a sliding window.

    A counter divided by uptime answers "what was the average since boot",
    which hides exactly the thing worth seeing -- a stream that stopped. This
    answers "what is happening now".
    """

    def __init__(self, window=4.0):
        self.window = window
        self._t = collections.defaultdict(collections.deque)

    def hit(self, key, now):
        q = self._t[key]
        q.append(now)
        cut = now - self.window
        while q and q[0] < cut:
            q.popleft()

    def rates(self, now):
        cut = now - self.window
        out = {}
        for k, q in self._t.items():
            while q and q[0] < cut:
                q.popleft()
            if q:
                out[k] = len(q) / self.window
        return out


class HostStats:
    """Pi vitals. Sampled slowly -- these move on a human timescale and reading
    them is a syscall storm if you do it per frame."""

    def __init__(self):
        self.data = {}
        self._t = 0.0
        self._prev = None

    def poll(self, now):
        if now - self._t < 1.0:
            return self.data
        self._t = now
        d = {}
        try:
            with open('/proc/stat') as fh:
                parts = [float(x) for x in fh.readline().split()[1:]]
            idle, total = parts[3] + parts[4], sum(parts)
            if self._prev:
                di, dt = idle - self._prev[0], total - self._prev[1]
                if dt > 0:
                    d['cpu'] = max(0.0, min(100.0, 100.0 * (1 - di / dt)))
            self._prev = (idle, total)
        except Exception:
            pass
        try:
            with open('/sys/class/thermal/thermal_zone0/temp') as fh:
                d['temp'] = int(fh.read().strip()) / 1000.0
        except Exception:
            pass
        try:
            with open('/proc/meminfo') as fh:
                mi = {k.strip(): v for k, v in
                      (l.split(':', 1) for l in fh.readlines()[:5])}
            tot = int(mi['MemTotal'].split()[0]) / 1024
            avail = int(mi['MemAvailable'].split()[0]) / 1024
            d['mem_used'], d['mem_total'] = tot - avail, tot
        except Exception:
            pass
        for key, cmd in (('clock', ['vcgencmd', 'measure_clock', 'arm']),
                         ('throttled', ['vcgencmd', 'get_throttled'])):
            try:
                out = subprocess.run(cmd, capture_output=True, text=True,
                                     timeout=0.5).stdout.strip()
                if key == 'clock':
                    d['clock_ghz'] = int(out.split('=')[1]) / 1e9
                else:
                    d['throttled'] = out.split('=')[1]
            except Exception:
                pass
        self.data = d
        return d


class BoardTelemetry:
    """Everything the srot board says, de-multiplexed and rate-metered.

    Runs the same 200 Hz drain `auv_manager_node` does. That is not optional:
    pymavlink keeps ONE message per msgid and all 24 NAMED_VALUE_FLOAT channels
    share msgid 251, so sampling the slot does not miss a channel occasionally
    -- it misses it always.
    """

    def __init__(self, fc, conn):
        self.fc = fc
        self.conn = conn
        self.rates = RateMeter()
        self.att = {}
        self.imu = {}
        self.hud = {}
        self.press = {}
        self.sys = {}
        self.power = {}
        self.batt = {}
        self.esc = {'rpm': [0] * 8, 'temp': [0] * 8,
                    'volt': [0.0] * 8, 'curr': [0.0] * 8}
        self.esc_msgs = 0
        self.rx_bytes = 0
        self.boot_ms = 0
        self.reboots = 0
        self._peak_boot = None
        self.depth_f = DepthFilter()
        self.hdg_f = HeadingFilter()
        self.depth_raw = None
        self.depth_filt = None
        self.hdg_raw = None
        self.hdg_filt = None
        self.trace = {k: Ring() for k in
                      ('depth', 'depth_f', 'hdg', 'hdg_f', 'roll', 'pitch',
                       'gz', 'volt', 'press')}
        self._stop = threading.Event()
        threading.Thread(target=self._reader, daemon=True).start()

    def stop(self):
        self._stop.set()

    def _reader(self):
        while not self._stop.is_set():
            got = False
            while True:
                msg = self.conn.recv_match(blocking=False)
                if msg is None:
                    break
                got = True
                self._ingest(msg)
            if not got:
                time.sleep(0.004)

    def _ingest(self, msg):
        t = msg.get_type()
        now = time.time()
        if t == 'BAD_DATA':
            self.rates.hit('BAD_DATA', now)
            return
        self.rates.hit(t, now)
        try:
            self.rx_bytes += len(msg.get_msgbuf())
        except Exception:
            pass

        if t == 'NAMED_VALUE_FLOAT':
            self.fc.note_named_value(msg)
        elif t == 'BATTERY_STATUS':
            self.fc.note_battery(msg)
            v = [x for x in msg.voltages[:1] if 0 < x < 65535]
            if v:
                self.batt[int(msg.id)] = v[0] / 1000.0
        elif t == 'ATTITUDE':
            self.att = {'roll': math.degrees(msg.roll),
                        'pitch': math.degrees(msg.pitch),
                        'yaw': math.degrees(msg.yaw) % 360.0,
                        'rollspeed': math.degrees(msg.rollspeed),
                        'pitchspeed': math.degrees(msg.pitchspeed),
                        'yawspeed': math.degrees(msg.yawspeed)}
            self.boot_ms = int(msg.time_boot_ms)
            # time_boot_ms going sharply backwards is an unplanned FC restart.
            # Reachable, not theoretical: any second process opening the serial
            # port reboots this board.
            if self._peak_boot is not None and self.boot_ms < self._peak_boot - 3000:
                self.reboots += 1
            self._peak_boot = max(self._peak_boot or 0, self.boot_ms)
            self.hdg_raw = self.att['yaw']
            self.hdg_filt = self.hdg_f.update(self.hdg_raw, now)
            self.trace['hdg'].push(round(self.hdg_raw, 2))
            self.trace['hdg_f'].push(round(self.hdg_filt or 0.0, 2))
            self.trace['roll'].push(round(self.att['roll'], 2))
            self.trace['pitch'].push(round(self.att['pitch'], 2))
            self.trace['gz'].push(round(self.att['yawspeed'], 2))
        elif t == 'SCALED_IMU2':
            self.imu = {'ax': msg.xacc, 'ay': msg.yacc, 'az': msg.zacc,
                        'gx': msg.xgyro, 'gy': msg.ygyro, 'gz': msg.zgyro,
                        'mx': msg.xmag, 'my': msg.ymag, 'mz': msg.zmag,
                        'mag': math.sqrt(msg.xmag ** 2 + msg.ymag ** 2
                                         + msg.zmag ** 2) / 10.0}   # mgauss->uT
        elif t == 'VFR_HUD':
            self.hud = {'heading': msg.heading, 'throttle': msg.throttle,
                        'alt': msg.alt, 'climb': msg.climb}
            self.depth_raw = -float(msg.alt)          # alt = -depth
            self.depth_filt = self.depth_f.update(self.depth_raw, now)
            self.trace['depth'].push(round(self.depth_raw, 3))
            self.trace['depth_f'].push(round(self.depth_filt or 0.0, 3))
        elif t == 'SCALED_PRESSURE2':
            self.press = {'abs': msg.press_abs, 'temp': msg.temperature / 100.0}
            self.trace['press'].push(round(msg.press_abs, 2))
        elif t == 'SYS_STATUS':
            self.sys = {'load': msg.load / 10.0,
                        'drop': msg.drop_rate_comm / 100.0,
                        'errors': msg.errors_comm}
            if 0 < msg.voltage_battery < 65535:
                self.trace['volt'].push(round(msg.voltage_battery / 1000.0, 2))
        elif t == 'POWER_STATUS':
            self.power = {'vcc': msg.Vcc / 1000.0, 'vservo': msg.Vservo / 1000.0}
        elif t in ('ESC_TELEMETRY_1_TO_4', 'ESC_TELEMETRY_5_TO_8'):
            self.esc_msgs += 1
            base = 0 if t.endswith('1_TO_4') else 4
            for i in range(4):
                try:
                    self.esc['rpm'][base + i] = int(msg.rpm[i])
                    self.esc['temp'][base + i] = int(msg.temperature[i])
                    self.esc['volt'][base + i] = msg.voltage[i] / 100.0
                    self.esc['curr'][base + i] = msg.current[i] / 100.0
                except Exception:
                    pass

    # -- the named channels, with absence preserved ------------------------ #
    NAMED = ('LEAK', 'WTEMP', 'BARO_HEALT', 'BARO_P2P', 'YAW_REF', 'MAGACC',
             'GAIN', 'KILL', 'COMP_SEEN', 'CURR', 'DEPTH_CMD', 'DEPTH_ERR',
             'DEPTH_OUT', 'MIX_VERT', 'MIX_VSGN', 'ATUNE', 'STUNT_PRG',
             'HEAP', 'STK_MAV', 'STK_SEN', 'STK_CTL', 'STK_UI', 'STK_LORA',
             'STK_DSH')

    def named(self):
        out = {}
        for k in self.NAMED:
            try:
                v = self.fc._named_value(k, max_age_s=4.0)
            except Exception:
                v = None
            out[k] = None if v is None else round(float(v), 4)
        return out

    def snapshot(self, now):
        rates = self.rates.rates(now)
        return {
            'att': self.att, 'imu': self.imu, 'hud': self.hud,
            'press': self.press, 'sys': self.sys, 'power': self.power,
            'batt': self.batt, 'esc': self.esc, 'esc_msgs': self.esc_msgs,
            'named': self.named(), 'rates': {k: round(v, 2) for k, v in rates.items()},
            'boot_ms': self.boot_ms, 'reboots': self.reboots,
            'depth_raw': self.depth_raw, 'depth_filt': self.depth_filt,
            'hdg_raw': self.hdg_raw, 'hdg_filt': self.hdg_filt,
            'trace': {k: r.list() for k, r in self.trace.items()},
        }

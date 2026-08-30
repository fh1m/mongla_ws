#!/usr/bin/env python3

"""Gamepad teleop — a joystick on the machine running the lab.

WHY THIS AND NOT A NEW MAVLINK NODE. `TeleopStreamer` already owns the hard
parts: the RC channel map, the PWM conversion, the 20 Hz stream, the watchdog,
and the rule that it goes quiet when the sticks are centred so the manager's
heartbeat keeps the pilot link warm. It also connects on **tcp:5763**, which
exists precisely so teleop never fights the manager's `udpin:14550`. A second
RC writer would have to duplicate all of that and then fight the first one, so
this is a device reader that moves the same axes the lab D-pad moves.

TWO PLACES A GAMEPAD CAN BE, and both are supported:

* **On the machine running the browser** — the lab UI reads it with the
  Gamepad API and POSTs to `/api/vehicle/teleop`. This is the QGC shape: the
  ground station reads the stick and sends commands over the link, so it works
  with the lab on another machine. That path needs nothing from this file.
* **On the machine running the lab** — this file, reading the kernel's
  joystick device directly. Lower latency and no browser at all, which is what
  you want for a long dataset-collection run.

THE DEVICE PROTOCOL IS STDLIB. Linux `/dev/input/js*` delivers a fixed 8-byte
record (`struct js_event`: u32 time, s16 value, u8 type, u8 number), so this
needs no `pygame`, no `evdev`, no ROS `joy` package -- just `struct`. That also
makes it testable: the reader takes a path, so a test can feed it the same
bytes the kernel would, and this one is additionally verified against a real
virtual gamepad created through `/dev/uinput`.
"""

from __future__ import annotations

import glob
import os
import struct
import threading
import time
from typing import Optional

# struct js_event { __u32 time; __s16 value; __u8 type; __u8 number; }
_JS_EVENT = struct.Struct('IhBB')
_JS_EVENT_SIZE = _JS_EVENT.size          # 8
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80                     # OR'd on the synthetic startup burst

AXIS_MAX = 32767.0

# Default axis map, in the layout every XInput-style pad reports. Left stick
# drives translation and right stick yaw + vertical, which is the arrangement
# ArduSub's own joystick page uses for a BlueROV: the left hand flies the boat
# over the ground, the right hand points and trims depth.
DEFAULT_AXIS_MAP = {
    'lat': 0,     # left stick X  -> strafe
    'fwd': 1,     # left stick Y  -> forward (inverted below: up is -1)
    'yaw': 3,     # right stick X -> yaw rate
    'up': 4,      # right stick Y -> vertical (inverted below)
}
INVERTED_AXES = ('fwd', 'up')            # kernel reports up as negative

# Buttons, following the ArduSub/QGC joystick-setup convention of putting the
# irreversible action behind a distinct button rather than a stick gesture.
DEFAULT_BUTTON_MAP = {
    0: 'arm',            # A
    1: 'disarm',         # B
    2: 'fire',           # X -- torpedo
    3: 'drop',           # Y -- marker
    4: 'gain_down',      # LB
    5: 'gain_up',        # RB
}

# Payload channels, in the order a press consumes them. The rulebook allows
# "up to two markers" and "up to two torpedoes" (p. 64), so a pad press walks
# 1 -> 2 and then reports the tube empty rather than firing forever. Practising
# against an unlimited magazine teaches a timing that does not exist.
FIRE_CHANNELS = (1, 2)          # torpedo
DROP_CHANNELS = (3, 4)          # marker / dropper

# QGC exposes gain as a first-class control because a single fixed stick scale
# is either too coarse for alignment or too slow for transit. Same idea here.
GAIN_MIN, GAIN_MAX, GAIN_STEP = 0.10, 1.00, 0.10

# Sticks do not return exactly to zero; without a deadzone the vehicle creeps.
DEFAULT_DEADZONE = 0.08
# Expo keeps small stick movements gentle while leaving full deflection at
# full authority -- the difference between "stable, accurate" and "twitchy".
DEFAULT_EXPO = 0.35


def find_device(explicit: str | None = None) -> Optional[str]:
    if explicit:
        return explicit if os.path.exists(explicit) else None
    found = sorted(glob.glob('/dev/input/js*'))
    return found[0] if found else None


def shape(value: float, deadzone: float, expo: float) -> float:
    """Deadzone, then expo, then rescale so full deflection is still 1.0."""
    v = max(-1.0, min(1.0, value))
    if abs(v) <= deadzone:
        return 0.0
    # Rescale past the deadzone so the axis starts from 0, not from a step.
    sign = 1.0 if v > 0 else -1.0
    v = (abs(v) - deadzone) / (1.0 - deadzone)
    return sign * ((1.0 - expo) * v + expo * v ** 3)


class JoystickReader:
    """Reads a Linux joystick device and drives a TeleopStreamer."""

    def __init__(self, teleop, device: str | None = None,
                 axis_map: dict | None = None,
                 button_map: dict | None = None,
                 deadzone: float = DEFAULT_DEADZONE,
                 expo: float = DEFAULT_EXPO,
                 gain: float = 0.55,
                 on_button=None,
                 logger=None) -> None:
        self._teleop = teleop
        self._device = device
        self._axis_map = dict(axis_map or DEFAULT_AXIS_MAP)
        self._button_map = dict(button_map or DEFAULT_BUTTON_MAP)
        self._deadzone = deadzone
        self._expo = expo
        self._gain = gain
        self._on_button = on_button
        self._log = logger

        # Remaining rounds, consumed in order. Reset by `reload`.
        self._magazine = {'fire': list(FIRE_CHANNELS),
                          'drop': list(DROP_CHANNELS)}
        self._axes = {k: 0.0 for k in self._axis_map}
        self._raw = {}
        self._buttons = {}
        self._connected = False
        self._path = ''
        self._name = ''
        self._last_event = 0.0
        self._events = 0
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # -- lifecycle ---------------------------------------------------------

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name='joystick',
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self._connected = False

    def snapshot(self) -> dict:
        return {
            'connected': self._connected,
            'device': self._path,
            'name': self._name,
            'axes': dict(self._axes),
            'buttons': {str(k): v for k, v in self._buttons.items() if v},
            'gain': round(self._gain, 2),
            'rounds': {k: list(v) for k, v in self._magazine.items()},
            'deadzone': self._deadzone,
            'expo': self._expo,
            'events': self._events,
            'age_s': (round(time.monotonic() - self._last_event, 2)
                      if self._last_event else None),
        }

    # -- reader ------------------------------------------------------------

    def _device_name(self, path: str) -> str:
        """Best-effort model name, for the operator to confirm the right pad."""
        node = os.path.basename(path)
        for candidate in glob.glob(f'/sys/class/input/{node}/device/name'):
            try:
                with open(candidate) as f:
                    return f.read().strip()
            except OSError:
                pass
        return node

    def _run(self) -> None:
        while not self._stop.is_set():
            path = find_device(self._device)
            if not path:
                self._connected = False
                self._stop.wait(1.0)
                continue
            try:
                self._read_device(path)
            except OSError as exc:
                if self._log:
                    self._log(f'[JOY  ] {path}: {exc}')
                self._connected = False
                # Centre on disconnect. A pad unplugged mid-run must not leave
                # the last stick value latched into the RC stream.
                self._teleop.set_axes(0, 0, 0, 0)
                self._stop.wait(1.0)

    def _read_device(self, path: str) -> None:
        with open(path, 'rb') as dev:
            os.set_blocking(dev.fileno(), False)
            self._path = path
            self._name = self._device_name(path)
            self._connected = True
            if self._log:
                self._log(f'[JOY  ] {self._name} on {path}')
            while not self._stop.is_set():
                while True:
                    chunk = dev.read(_JS_EVENT_SIZE)
                    if not chunk or len(chunk) < _JS_EVENT_SIZE:
                        break
                    self._handle(*_JS_EVENT.unpack(chunk))
                # PUSH EVERY TICK, NOT ONLY ON AN EVENT.
                #
                # A HELD STICK PRODUCES NO EVENTS. The kernel reports changes,
                # so holding full forward emits one event and then silence --
                # and TeleopStreamer has a 0.35 s watchdog that centres the
                # vehicle when set_axes goes quiet, which is exactly right for
                # a dropped UI but wrong for a stick that is simply steady.
                #
                # Measured before this fix: full forward held for 6 s moved the
                # hull 0.188 m, because it got roughly one watchdog window of
                # thrust per stick movement and coasted the rest. That is what
                # "twitchy teleop" actually is.
                self._push()
                # 50 Hz: comfortably above the 20 Hz RC stream and the 0.35 s
                # watchdog, so neither ever starves.
                self._stop.wait(0.02)

    def _handle(self, _time, value, ev_type, number) -> None:
        self._events += 1
        self._last_event = time.monotonic()
        # The kernel replays current state at open with JS_EVENT_INIT set.
        # Those are real values and must be applied, but a button "press" in
        # that burst is a state report, not an operator action -- firing arm()
        # from it would arm the vehicle the moment the pad is plugged in.
        initial = bool(ev_type & JS_EVENT_INIT)
        kind = ev_type & ~JS_EVENT_INIT

        if kind == JS_EVENT_AXIS:
            for name, index in self._axis_map.items():
                if index == number:
                    v = value / AXIS_MAX
                    if name in INVERTED_AXES:
                        v = -v
                    self._raw[name] = v
                    self._axes[name] = round(
                        shape(v, self._deadzone, self._expo), 3)
        elif kind == JS_EVENT_BUTTON:
            self._buttons[number] = bool(value)
            if value and not initial:
                self._press(number)

    def reload(self) -> None:
        """Refill both tubes -- a new run, not a new vehicle."""
        self._magazine = {'fire': list(FIRE_CHANNELS),
                          'drop': list(DROP_CHANNELS)}

    def _press(self, number: int) -> None:
        action = self._button_map.get(number)
        if action in ('fire', 'drop'):
            remaining = self._magazine[action]
            if not remaining:
                if self._log:
                    self._log(f'[JOY  ] {action}: no rounds left '
                              f'(the rulebook allows two)')
                return
            # PEEK, then consume only if the shot actually went. A fire that
            # the vehicle refuses -- the disarmed interlock is the common one
            # -- must not cost a round: measured, pressing X before arming
            # burned torpedo 1 and the next press fired torpedo 2, so the
            # operator silently had one shot instead of two.
            channel = remaining[0]
            ok = True
            if self._on_button:
                ok = self._on_button(f'{action}:{channel}') is not False
            if ok:
                remaining.pop(0)
            elif self._log:
                self._log(f'[JOY  ] {action} ch={channel} refused -- '
                          f'round not consumed')
            return
        if action == 'gain_up':
            self._gain = min(GAIN_MAX, round(self._gain + GAIN_STEP, 2))
        elif action == 'gain_down':
            self._gain = max(GAIN_MIN, round(self._gain - GAIN_STEP, 2))
        elif action and self._on_button:
            self._on_button(action)

    def _push(self) -> None:
        a = self._axes
        self._teleop.set_axes(a.get('fwd', 0.0), a.get('lat', 0.0),
                              a.get('up', 0.0), a.get('yaw', 0.0), self._gain)

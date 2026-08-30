#!/usr/bin/env python3

"""Continuous RC teleop streamer for the operator lab D-pad.

Connects to ArduSub SITL over TCP (SERIAL2) so we do not fight the manager's
udpin:14550 bind. Streams RC_CHANNELS_OVERRIDE at ~20 Hz while axes are
non-zero; when idle, stops writing so the manager Heartbeat can keep the
pilot link warm with neutrals.
"""

from __future__ import annotations

import os
import threading
import time
from typing import Optional

# ArduSub BlueROV-style channel indices (0-based in the 18-slot override).
CH_PITCH = 0
CH_ROLL = 1
CH_THROTTLE = 2
CH_YAW = 3
CH_FORWARD = 4
CH_LATERAL = 5
NO_OVERRIDE = 65535

DEFAULT_ENDPOINT = os.environ.get('DUBURI_TELEOP_ENDPOINT', 'tcp:127.0.0.1:5763')
# One floor, shared by the UI slider, the gamepad step and this clamp. They
# were 0.15 / 0.10 / 0.05, so the same "minimum" meant three different things.
# Below about 0.10 the command is inside ArduSub's own 30 us RC deadzone and
# the vehicle does not move at all, so a lower floor only buys dead travel.
GAIN_MIN = 0.10
STREAM_HZ = 20.0
WATCHDOG_S = 0.35
# Fraction of full stick. 1.0 = PWM 1900 = full authority, which is also
# MOT_PWM_MAX and the model's <servo_max>, so nothing downstream clips it.
#
# This was 0.55, and the T200 deadband makes that much slower than it reads:
# forward thrust only starts at PWM 1528, so gain 0.55 -> PWM 1720 -> 20.9 N of
# a possible 53.2 N. 55% of stick is 39% of thrust. Full authority is the right
# default for dataset collection and feel-testing; turn it down to go gentle,
# rather than starting throttled and wondering why the hull feels dead.
GAIN_DEFAULT = 1.0


def _pct_to_pwm(pct: float) -> int:
    """-100..100 percent → 1100..1900 µs (0 → 1500)."""
    return max(1100, min(1900, int(1500 + (pct / 100.0) * 400)))


def _axis_to_pwm(axis: float, gain: float) -> int:
    axis = max(-1.0, min(1.0, float(axis)))
    if abs(axis) < 1e-3:
        return 1500
    return _pct_to_pwm(axis * gain * 100.0)


class TeleopStreamer:
    """Thread-safe axis holder + RC stream daemon."""

    def __init__(self, endpoint: str = DEFAULT_ENDPOINT) -> None:
        self._endpoint = endpoint
        self._lock = threading.Lock()
        self._fwd = 0.0
        self._lat = 0.0
        self._up = 0.0
        self._yaw = 0.0
        self._gain = GAIN_DEFAULT
        self._last_update = 0.0
        self._connected = False
        self._error: Optional[str] = None
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._master = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name='lab_teleop', daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._close()

    def set_axes(
        self,
        fwd: float = 0.0,
        lat: float = 0.0,
        up: float = 0.0,
        yaw: float = 0.0,
        gain: Optional[float] = None,
    ) -> dict:
        with self._lock:
            self._fwd = float(fwd)
            self._lat = float(lat)
            self._up = float(up)
            self._yaw = float(yaw)
            if gain is not None:
                self._gain = max(GAIN_MIN, min(1.0, float(gain)))
            self._last_update = time.monotonic()
            return self._snapshot_unlocked()

    def snapshot(self) -> dict:
        with self._lock:
            return self._snapshot_unlocked()

    def _snapshot_unlocked(self) -> dict:
        return {
            'fwd': self._fwd,
            'lat': self._lat,
            'up': self._up,
            'yaw': self._yaw,
            'gain': self._gain,
            'connected': self._connected,
            'error': self._error,
            'endpoint': self._endpoint,
            'active': any(abs(v) > 1e-3 for v in (self._fwd, self._lat, self._up, self._yaw)),
        }

    def _close(self) -> None:
        if self._master is not None:
            try:
                self._master.close()
            except Exception:
                pass
            self._master = None
        self._connected = False

    def _ensure_link(self) -> bool:
        if self._master is not None:
            return True
        try:
            from pymavlink import mavutil

            self._master = mavutil.mavlink_connection(self._endpoint, autoreconnect=True)
            hb = self._master.wait_heartbeat(timeout=2.0)
            if hb is None:
                self._error = 'no heartbeat on teleop link'
                self._close()
                return False
            self._connected = True
            self._error = None
            return True
        except Exception as exc:
            self._error = str(exc)
            self._close()
            return False

    def _send_rc(self, fwd: float, lat: float, up: float, yaw: float, gain: float) -> None:
        if self._master is None:
            return
        values = [NO_OVERRIDE] * 18
        values[CH_PITCH] = 1500
        values[CH_ROLL] = 1500
        values[CH_THROTTLE] = _axis_to_pwm(up, gain)  # up>0 ascend
        values[CH_YAW] = _axis_to_pwm(yaw, gain)
        values[CH_FORWARD] = _axis_to_pwm(fwd, gain)
        values[CH_LATERAL] = _axis_to_pwm(lat, gain)
        self._master.mav.rc_channels_override_send(
            self._master.target_system,
            self._master.target_component,
            *values,
        )

    def _run(self) -> None:
        period = 1.0 / STREAM_HZ
        while not self._stop.is_set():
            t0 = time.monotonic()
            with self._lock:
                fwd, lat, up, yaw = self._fwd, self._lat, self._up, self._yaw
                gain = self._gain
                last = self._last_update
            # Watchdog: stale axes → idle.
            if last > 0 and (time.monotonic() - last) > WATCHDOG_S:
                with self._lock:
                    self._fwd = self._lat = self._up = self._yaw = 0.0
                fwd = lat = up = yaw = 0.0

            active = any(abs(v) > 1e-3 for v in (fwd, lat, up, yaw))
            if active:
                if self._ensure_link():
                    try:
                        self._send_rc(fwd, lat, up, yaw, gain)
                    except Exception as exc:
                        self._error = str(exc)
                        self._close()
            # When idle, do not write — manager Heartbeat owns neutrals.

            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, period - elapsed))

"""BNO085Source — external yaw via ESP32-C3 over USB CDC.

Why this design (no magnetometer, ever)
---------------------------------------
The BNO085 ships with the chip in `SH2_GAME_ROTATION_VECTOR` mode:
gyro + accelerometer fused, magnetometer DISABLED. That gives us a
smooth heading immune to magnetic interference (8 thrusters + aluminum
(Marine 5083) hull + battery currents) but with no absolute Earth reference — the
chip's "yaw 0" is whatever direction it was facing at boot.

To get an Earth-referenced heading without ever using the BNO's mag,
we read the Pixhawk's mag-fused yaw ONCE at startup (sub at the surface,
clean magnetic environment) and lock the offset:

    offset = pixhawk_yaw - bno_raw_yaw          # captured at __init__
    earth_yaw(t) = (bno_raw_yaw(t) + offset) mod 360

After calibration the Pixhawk magnetometer is never read again — gyro
drift only (~0.5 deg/min typical), no in-hull magnetic interference.

Wire contract (firmware side)
-----------------------------
The MCU ships ONE JSON object per line, newline-terminated:

    {"yaw": 123.45, "ts": 12345}\n

  yaw   float   degrees in [0, 360), sensor frame, +CW (right-hand rule
                around gravity). NOT magnetic north — the Jetson side
                applies the calibration offset.
  ts    int     ms since MCU boot. Optional. Diagnostic only.

Stream rate: 50 Hz target. Anything 20-100 Hz works; control loops
poll at 10 Hz so we just need fresher-than-stale samples. Avoid
exactly 200 Hz (documented BNO firmware bug — Adafruit forum thread
"BNO085 Behavior After Stillness").
Baud: 115200.

Design rules (from user spec, see .claude/context/sensors-pipeline.md)
---------------------------------------------------------------------
  * Single source per launch, no mid-run switching.
  * No fallback to MAVLink AHRS — if BNO085 goes silent, read_yaw()
    returns None and the control loop holds its last known value.
  * No fusion. The BNO085 quaternion is already 6DoF-fused on-chip.
  * No filtering. Add later only if pool data demands it.
  * Calibration is one-shot at __init__; restart the node to re-zero.
"""

import json
import threading
import time

import serial          # pyserial


_STALE_S = 0.25        # 12 frames @ 50 Hz; matches our 10 Hz control loop


class BNO085Source:
    """Background reader for the JSON-line protocol above.

    Constructor opens the port and starts the reader thread; it raises
    serial.SerialException if the port can't be opened (operator chose
    this source, operator gets told — no silent fallback).

    If `reference_yaw_provider` is supplied, __init__ also performs a
    bounded calibration: it waits for a fresh BNO sample AND a fresh
    reference (typically Pixhawk AHRS yaw), computes the offset, and
    locks it. Subsequent read_yaw() calls return Earth-referenced yaw.

    If `reference_yaw_provider` is None, __init__ returns immediately
    after starting the reader thread; read_yaw() then returns the raw
    sensor-frame yaw. This raw mode is intended for desk smoke-tests
    via sensors_node, NOT for missions.
    """

    name = 'BNO085'

    def __init__(self, *, port: str, baud: int = 115200, logger=None,
                 reference_yaw_provider=None,
                 calibration_timeout_s: float = 5.0):
        self._port_name = port
        self._baud      = baud
        self._log       = logger

        self._latest_yaw: float | None = None
        self._latest_ts:  float        = 0.0
        self._frames_rx                = 0
        self._parse_errors             = 0

        self._offset_deg: float | None = None     # set on successful calibration

        self._stop = threading.Event()
        self._serial = serial.Serial(port=port, baudrate=baud, timeout=0.1)

        self._thread = threading.Thread(
            target=self._reader_loop,
            name=f'bno085-reader[{port}]',
            daemon=True)
        self._thread.start()

        if self._log:
            self._log.info(f'[SENSOR] BNO085 reader started on {port} @ {baud}')

        if reference_yaw_provider is not None:
            try:
                self._calibrate(reference_yaw_provider, calibration_timeout_s)
            except Exception:
                # Calibration failed — release the serial port before
                # the exception propagates, otherwise the device stays
                # held until the GC runs the destructor.
                self.close()
                raise

    # ------------------------------------------------------------------ #
    #  Calibration — one-shot Earth-reference offset capture              #
    # ------------------------------------------------------------------ #
    def _calibrate(self, reference_provider, timeout_s: float) -> None:
        """Block until both BNO and reference are fresh, then lock offset."""
        deadline = time.monotonic() + timeout_s
        bno_raw = None
        ref     = None

        while time.monotonic() < deadline:
            bno_raw = self._fresh_raw_yaw()
            ref     = reference_provider()
            if bno_raw is not None and ref is not None:
                self._offset_deg = (ref - bno_raw) % 360.0
                if self._log:
                    self._log.info(
                        f'[SENSOR] BNO085 calibrated  '
                        f'pixhawk={ref:.2f}°  bno_raw={bno_raw:.2f}°  '
                        f'offset={self._offset_deg:+.2f}°')
                return
            time.sleep(0.1)

        raise RuntimeError(
            f'BNO085 calibration timed out after {timeout_s:.1f}s — '
            f'bno_fresh={bno_raw is not None}, '
            f'pixhawk_fresh={ref is not None}')

    @property
    def offset_deg(self) -> float | None:
        """Locked Earth-reference offset in degrees, or None if uncalibrated."""
        return self._offset_deg

    # ------------------------------------------------------------------ #
    #  YawSource contract                                                 #
    # ------------------------------------------------------------------ #
    def _fresh_raw_yaw(self) -> float | None:
        """Latest sensor-frame yaw if fresh, else None. No offset applied."""
        if self._latest_yaw is None:
            return None
        if (time.monotonic() - self._latest_ts) > _STALE_S:
            return None
        return self._latest_yaw

    def read_yaw(self) -> float | None:
        raw = self._fresh_raw_yaw()
        if raw is None:
            return None
        if self._offset_deg is None:
            return raw                                  # raw mode (diag only)
        return (raw + self._offset_deg) % 360.0

    def is_healthy(self) -> bool:
        return self.read_yaw() is not None

    def close(self) -> None:
        self._stop.set()
        try:
            self._thread.join(timeout=1.0)
        except Exception as exc:
            if self._log:
                self._log.debug(f'[SENSOR] BNO085 thread join ignored: {exc!r}')
        try:
            self._serial.close()
        except Exception as exc:
            if self._log:
                self._log.debug(f'[SENSOR] BNO085 serial close ignored: {exc!r}')
        if self._log:
            self._log.info(
                f'[SENSOR] BNO085 stopped — frames:{self._frames_rx} '
                f'errors:{self._parse_errors}')

    def __repr__(self) -> str:
        fresh = (time.monotonic() - self._latest_ts) if self._latest_yaw is not None else None
        age   = f'{fresh*1e3:.0f}ms' if fresh is not None else 'NEVER'
        cal   = f'offset={self._offset_deg:+.2f}°' if self._offset_deg is not None else 'RAW'
        return (f'<BNO085Source port={self._port_name} baud={self._baud} '
                f'frames={self._frames_rx} age={age} {cal}>')

    # ------------------------------------------------------------------ #
    #  Reader thread                                                      #
    # ------------------------------------------------------------------ #
    def _reader_loop(self) -> None:
        ser = self._serial
        while not self._stop.is_set():
            try:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='ignore').strip()
                if not line or line[0] != '{':
                    continue

                msg = json.loads(line)
                yaw = float(msg['yaw']) % 360.0

                self._latest_yaw = yaw
                self._latest_ts  = time.monotonic()
                self._frames_rx += 1

            except (ValueError, KeyError, json.JSONDecodeError):
                self._parse_errors += 1
            except serial.SerialException as exc:
                if self._log:
                    self._log.error(f'[SENSOR] BNO085 serial error: {exc}')
                self._stop.set()
                break
            except Exception as exc:
                self._parse_errors += 1
                if self._log and self._parse_errors % 50 == 1:
                    self._log.warn(f'[SENSOR] BNO085 reader: {exc}')

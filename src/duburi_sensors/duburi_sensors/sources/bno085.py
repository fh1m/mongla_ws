"""BNO085Source — external yaw via ESP32-C3 over USB CDC.

Plug-and-play port discovery
----------------------------
Pass ``port='auto'`` (the manager-node default) and the source probes
the host for a recognisable USB CDC device, opens the first one that
delivers a valid JSON line within the probe window, and uses that. The
chosen path is logged loudly so operators see which device was picked.
Pin a specific device with ``port='/dev/ttyACM0'`` to skip discovery.

Why this design (no magnetometer, ever)
---------------------------------------
The reference firmware runs the BNO085 in ``SH2_GYRO_INTEGRATED_RV``
(``GAME_ROTATION_VECTOR``-equivalent for our purposes) — gyro +
accelerometer fused, magnetometer DISABLED. That gives us a smooth
heading immune to magnetic interference (8 thrusters + aluminum (Marine
5083) hull + battery currents) but with no absolute Earth reference —
the chip's heading zero is whatever direction it was facing at boot.

To get an Earth-referenced heading without ever using the BNO's mag,
we read the Pixhawk's mag-fused yaw ONCE at startup (sub at the surface,
clean magnetic environment) and lock the offset:

    offset = pixhawk_yaw - bno_yaw              # captured at __init__
    earth_yaw(t) = (bno_yaw(t) + offset) mod 360

After calibration the Pixhawk magnetometer is never read again — gyro
drift only (~0.5 deg/min typical), no in-hull magnetic interference.

Wire contract (firmware side) and convention conversion
------------------------------------------------------
The MCU ships ONE JSON object per line, newline-terminated:

    {"yaw": 123.45, "pitch": -5.2, "roll": 3.1, "ts": 12345}\n

  yaw   float   degrees in [0, 360) as emitted by the firmware's
                ``atan2(2*(qi*qj + qk*qr), (sqi - sqj - sqk + sqr))``.
                That is the ENU / right-handed convention: rotating the
                board CCW (LEFT as viewed from above, looking down +Z)
                makes the reported yaw INCREASE.
  pitch float   degrees, body-frame pitch (nose up = positive).
                Passed through as-is; no convention flip needed.
  roll  float   degrees, body-frame roll (right side down = positive).
                Passed through as-is; no convention flip needed.
  ts    int     ms since MCU boot. Optional. Diagnostic only.

  pitch and roll are present in firmware ≥ 2026-06; earlier firmware
  omits them. read_pitch() / read_roll() return 0.0 when not yet
  received or stale.

ArduSub / Pixhawk AHRS publishes in NED / compass convention, where
rotating the vehicle CW (RIGHT) is what INCREASES yaw (N=0 E=90 S=180
W=270). ``_reader_loop`` below negates the raw value once at ingestion
so ``read_yaw()`` is compass-convention for the rest of the stack.
Do not apply the negation again anywhere downstream.

Stream rate: 50 Hz target (firmware ships ~50 Hz from a 500 Hz internal
loop). Anything 20-100 Hz works; control loops poll at 10 Hz so we just
need fresher-than-stale samples.
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

from ._discovery import (   # noqa: F401 -- re-exported for external callers
    auto_detect_port,
    _enumerate_candidate_ports,
    _probe_port,
    _BNO_VID_PID,
    _AUTO_PROBE_TIMEOUT_S,
)

_STALE_S = 0.08        # 4 frames @ 50 Hz; tighter freshness to match 50 Hz heading lock


class BNO085Source:
    """Background reader for the JSON-line protocol above.

    Constructor opens the port (or auto-detects one when ``port='auto'``)
    and starts the reader thread; it raises ``serial.SerialException`` if
    the port can't be opened, or ``RuntimeError`` if auto-detect fails.
    Operator chose this source, operator gets told — no silent fallback.

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
        self._baud = baud
        self._log  = logger

        # ``port='auto'`` (or '' / None) -> probe known USB CDC devices.
        if not port or str(port).strip().lower() == 'auto':
            port = auto_detect_port(baud=baud, logger=logger)
        self._port_name = port

        self._latest_yaw: float | None = None
        self._latest_ts:  float        = 0.0
        self._latest_pitch: float      = 0.0
        self._latest_pitch_ts: float   = 0.0
        self._latest_roll: float       = 0.0
        self._latest_roll_ts: float    = 0.0
        self._frames_rx                = 0
        self._parse_errors             = 0

        self._offset_deg: float | None = None     # set on successful calibration

        self._stop = threading.Event()
        self._serial_write_lock = threading.Lock()
        # Lines that had already arrived when we got to the previous one, i.e.
        # how much history this reader has skipped. Non-zero means the thread
        # is not being scheduled promptly and every value it stamps is older
        # than it claims -- see `_reader_loop`.
        self._stale_lines = 0
        # Open with dtr=False first to avoid triggering the ESP32-C3 auto-reset
        # circuit (dev boards wire DTR→EN via RC, causing a reset on port open).
        # After settling, assert dtr=True so the HWCDC starts streaming.
        _ser = serial.Serial()
        _ser.port     = port
        _ser.baudrate = baud
        _ser.timeout  = 0.1
        _ser.dtr      = False   # don't assert DTR on open (would reset ESP32-C3)
        _ser.rts      = False   # don't assert RTS — RTS+DTR combo = esptool reset sequence
        _ser.open()
        time.sleep(0.1)         # let USB CDC ACM settle (kernel cdc_acm sends control msgs)
        _ser.reset_input_buffer()
        _ser.dtr = True         # arm HWCDC device→host stream
        self._serial = _ser

        self._thread = threading.Thread(
            target=self._reader_loop,
            name=f'bno085-reader[{port}]',
            daemon=True)
        self._thread.start()

        if self._log:
            self._log.info(f'[SENS ] BNO085 reader started on {port} @ {baud}')

        if reference_yaw_provider is not None:
            try:
                self._calibrate(reference_yaw_provider, calibration_timeout_s)
            except RuntimeError as exc:
                # Calibration timed out (Pixhawk AHRS slow to warm, or BNO
                # not yet streaming).  Continue in raw (boot-relative) mode
                # rather than killing the sensors node — offset_deg stays None
                # so callers can detect the uncalibrated state.
                if self._log:
                    self._log.warn(
                        f'[SENS ] BNO085 calibration failed: {exc} '
                        f'— running in raw (boot-relative) mode')
            except Exception:
                # Unexpected error (serial fault, etc.) — release port and
                # propagate so the node can restart cleanly.
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
                        f'[SENS ] BNO085 calibrated  '
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

    def read_pitch(self) -> float | None:
        """Current BNO085 pitch in degrees, or None if stale / not yet received.

        Returns None (not 0.0) so callers can distinguish "no data" from
        "actually pitched at 0°". Style-maneuver loops skip stale ticks.
        """
        if (time.monotonic() - self._latest_pitch_ts) > _STALE_S:
            return None
        return self._latest_pitch

    def read_roll(self) -> float | None:
        """Current BNO085 roll in degrees, or None if stale / not yet received.

        Returns None (not 0.0) so callers can distinguish "no data" from
        "actually at 0° roll". Style-maneuver loops skip stale ticks via
        `if cur is None: continue` to avoid corrupting the accumulator.
        """
        if (time.monotonic() - self._latest_roll_ts) > _STALE_S:
            return None
        return self._latest_roll

    def send_command(self, cmd: str) -> None:
        """Write a command string to the BNO over serial (fire-and-forget).

        Thread-safe: guarded by _serial_write_lock so concurrent callers
        and the reader thread (readline) do not interleave writes.
        Non-critical: exceptions swallowed so OLED logging never breaks the
        mission path.
        """
        try:
            with self._serial_write_lock:
                if self._serial and self._serial.is_open:
                    self._serial.write(cmd.encode('utf-8'))
        except Exception:
            pass

    def is_healthy(self) -> bool:
        return self.read_yaw() is not None

    def close(self) -> None:
        self._stop.set()
        try:
            self._thread.join(timeout=1.0)
        except Exception as exc:
            if self._log:
                self._log.debug(f'[SENS ] BNO085 thread join ignored: {exc!r}')
        try:
            self._serial.close()
        except Exception as exc:
            if self._log:
                self._log.debug(f'[SENS ] BNO085 serial close ignored: {exc!r}')
        if self._log:
            self._log.info(
                f'[SENS ] BNO085 stopped — frames:{self._frames_rx} '
                f'errors:{self._parse_errors} skipped:{self._stale_lines}')

    def __repr__(self) -> str:
        fresh = (time.monotonic() - self._latest_ts) if self._latest_yaw is not None else None
        age   = f'{fresh*1e3:.0f}ms' if fresh is not None else 'NEVER'
        cal   = f'offset={self._offset_deg:+.2f}°' if self._offset_deg is not None else 'RAW'
        return (f'<BNO085Source port={self._port_name} baud={self._baud} '
                f'frames={self._frames_rx} skipped={self._stale_lines} '
                f'age={age} {cal}>')

    # ------------------------------------------------------------------ #
    #  Reader thread                                                      #
    # ------------------------------------------------------------------ #
    def _reader_loop(self) -> None:
        """Drain to the NEWEST line, the way the V4L2 pump drains to the newest
        buffer -- and count how far behind we were, because the lag is
        otherwise invisible.

        A tty is FIFO. `readline()` alone returns the OLDEST buffered line, and
        `now = time.monotonic()` below stamps it as if it had just arrived, so
        a reader that falls behind reports history as fresh. That is the same
        defect `camera_node` had (`header.stamp = now()` at publish), on the
        sensor the heading loop closes on.

        THIS IS DEFENCE IN DEPTH, NOT A FIX FOR A MEASURED BUG, and the
        difference is worth stating because the search for one went wrong in an
        instructive way.

        A slow CONSUMER does not starve this reader: measured on a PTY at
        100 Hz, it stays at 0.3 ms of lag, because it runs freely while the
        consumer sleeps. GIL contention does, and dramatically -- with
        CPU-bound PYTHON threads beside it the same reader went to 108 ms of
        lag at 50 Hz with one competitor and 2429 ms at 1.4 Hz with three.

        SO THE OBVIOUS CONCLUSION WAS DRAWN AND THEN MEASURED AGAINST REALITY,
        WHERE IT DID NOT HOLD. Our hot threads are not CPU-bound Python: the
        V4L2 pump blocks in `ioctl`, the decode is inside OpenCV, and this
        reader blocks in `serial` -- all of which RELEASE the GIL. Against the
        real srot board with 225 Hz of real JPEG decoding beside it, the
        MAVLink drain's inter-arrival p95 was 108.70 ms versus 109.75 ms with
        the load removed: no starvation at all. And shortening the GIL slice to
        recover the synthetic figure made the tail WORSE (p95 130.61 ms), since
        more switches on threads that already yield is pure overhead. That
        change was reverted.

        What survives is this drain and its counter. If the reader ever does
        fall behind -- a GC pause, a page fault, another process on the core --
        it skips history in one bulk read instead of parsing through it, and
        `_stale_lines` says so. That observability is the point: through the
        whole synthetic table above the tty backlog stayed at 132 bytes, so
        watching `in_waiting` would NOT have caught it, and a value stamped at
        parse time reports history as fresh with nothing in any log.
        """
        ser = self._serial
        while not self._stop.is_set():
            try:
                raw = ser.readline()
                if not raw:
                    continue
                # Skip to the newest COMPLETE line. `in_waiting` then one bulk
                # read costs a single syscall for the whole backlog; the last
                # fragment is a partial line and is deliberately dropped -- the
                # next `readline()` completes it.
                pending = ser.in_waiting
                if pending:
                    chunks = ser.read(pending).split(b'\n')
                    whole = [c for c in chunks[:-1] if c.strip()]
                    if whole:
                        self._stale_lines += len(whole)
                        raw = whole[-1]
                line = raw.decode('utf-8', errors='ignore').strip()
                if not line or line[0] != '{':
                    continue

                msg = json.loads(line)
                # Firmware ships ENU-native yaw (+CCW: rotating the board
                # LEFT increases yaw). We invert to compass / NED (+CW,
                # N=0 E=90 S=180 W=270) here so every downstream consumer
                # -- heading_error, motion_yaw, HeadingLock, auv_manager
                # telemetry -- sees the same convention as Pixhawk AHRS.
                yaw = (-float(msg['yaw'])) % 360.0
                now = time.monotonic()

                self._latest_yaw = yaw
                self._latest_ts  = now
                if 'pitch' in msg:
                    self._latest_pitch    = float(msg['pitch'])
                    self._latest_pitch_ts = now
                if 'roll' in msg:
                    self._latest_roll     = float(msg['roll'])
                    self._latest_roll_ts  = now
                self._frames_rx += 1

            except (ValueError, KeyError, json.JSONDecodeError):
                self._parse_errors += 1
            except serial.SerialException as exc:
                if self._log:
                    self._log.error(f'[SENS ] BNO085 serial error: {exc}')
                self._stop.set()
                break
            except Exception as exc:
                self._parse_errors += 1
                if self._log and self._parse_errors % 50 == 1:
                    self._log.warn(f'[SENS ] BNO085 reader: {exc}')

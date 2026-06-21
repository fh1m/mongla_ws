"""USB CDC port discovery for the BNO085 auto-detect pipeline.

Extracted from bno085.py so port-scanning logic can be tested and
iterated independently of the BNO085Source sensor class.

Public API
----------
auto_detect_port(*, baud, logger) -> str
    Probe known USB CDC devices; return the first that streams BNO085 JSON.
    Raises RuntimeError with a friendly checklist if nothing answers.
"""

import json
import os
import time

import serial  # pyserial
from serial.tools import list_ports

# BNO085 runs on ESP32-C3 HWCDC — USB VID/PID is stable regardless of /dev node.
# This replaces glob patterns so discovery works even if the port number reassigns.
_BNO_VID_PID: tuple[int, int] = (0x303a, 0x1001)  # Espressif USB JTAG/serial (HWCDC)

# per-candidate probe window; covers ESP32-C3 cold-boot + BNO init + first JSON frame.
# When device is already powered, first frame arrives in <100ms; 3s covers cold boot.
_AUTO_PROBE_TIMEOUT_S = 3.0


def _enumerate_candidate_ports() -> list[str]:
    """Return de-duplicated ports matching the ESP32-C3 HWCDC VID/PID.

    Uses serial.tools.list_ports so discovery works regardless of which
    ttyACM*/ttyUSB* number the OS assigns to the device.
    """
    seen: set[str] = set()
    candidates: list[str] = []
    for info in list_ports.comports():
        if info.vid == _BNO_VID_PID[0] and info.pid == _BNO_VID_PID[1]:
            try:
                real = os.path.realpath(info.device)
            except OSError:
                real = info.device
            if real not in seen:
                seen.add(real)
                candidates.append(info.device)
    return candidates


def _probe_port(path: str, baud: int, logger=None) -> bool:
    """Open `path`, read for up to `_AUTO_PROBE_TIMEOUT_S`, return True
    if at least one parseable `{"yaw":...}` JSON line arrives.

    Retries once if the device disconnects mid-probe (ESP32-C3 USB CDC
    briefly re-enumerates right after a host opens the port -- select()
    reports readable but read() returns EOF, raising SerialException).

    DTR sequence: open with dtr=False (avoids triggering the ESP32-C3
    auto-reset circuit on dev boards), then assert dtr=True so the HWCDC
    starts streaming.  Without dtr=True the device→host path is silently
    gated off; without the deferred assert, the DTR-on-open pulse resets
    the board before it can stream, causing the re-enum race that makes
    detection flaky.
    """
    for attempt in range(2):
        try:
            s = serial.Serial()
            s.port     = path
            s.baudrate = baud
            s.timeout  = 0.2
            s.dtr      = False   # don't assert DTR on open (would reset ESP32-C3)
            s.rts      = False   # don't assert RTS — RTS+DTR combo = esptool reset sequence
            s.open()
            time.sleep(0.1)      # let USB CDC ACM settle (kernel cdc_acm sends control msgs)
            s.reset_input_buffer()
            s.dtr = True         # arm HWCDC device→host stream
        except (serial.SerialException, OSError) as exc:
            if logger:
                logger.debug(f'[SENS ] BNO085 probe skip {path}: {exc}')
            return False   # can't open at all (busy / permission denied)

        disconnected = False
        try:
            deadline = time.monotonic() + _AUTO_PROBE_TIMEOUT_S
            while time.monotonic() < deadline:
                try:
                    raw = s.readline()
                except serial.SerialException as exc:
                    # Device briefly dropped (ESP32 USB re-enum on port open).
                    if logger:
                        logger.debug(
                            f'[SENS ] BNO085 probe {path} disconnect '
                            f'(attempt {attempt + 1}/2): {exc}')
                    disconnected = True
                    break
                if not raw:
                    continue
                try:
                    line = raw.decode('utf-8', errors='ignore').strip()
                except Exception:
                    continue
                if not line or line[0] != '{':
                    continue
                try:
                    msg = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if 'yaw' in msg:
                    return True
        finally:
            try:
                s.close()
            except Exception:
                pass

        if not disconnected or attempt > 0:
            break   # clean timeout or second attempt exhausted
        time.sleep(3.0)   # wait for ESP32 to re-enumerate before retry (3 s covers slow USB hosts)

    return False


def auto_detect_port(*, baud: int = 115200, logger=None) -> str:
    """Probe known USB CDC devices, return the first that streams BNO085 JSON.

    Raises RuntimeError with a friendly checklist if nothing answers.
    """
    candidates = _enumerate_candidate_ports()
    if not candidates:
        raise RuntimeError(
            f'BNO085 auto-detect: no device found with VID=0x{_BNO_VID_PID[0]:04x} '
            f'PID=0x{_BNO_VID_PID[1]:04x} (ESP32-C3 HWCDC). '
            'Plug the ESP32-C3 in, then retry.')

    if logger:
        logger.info(
            f'[SENS ] BNO085 auto-detect: probing {len(candidates)} candidate(s)...')

    for path in candidates:
        if logger:
            logger.info(f'[SENS ]   - probe {path}')
        if _probe_port(path, baud, logger):
            if logger:
                logger.info(f'[SENS ] BNO085 auto-detect: picked {path}')
            return path

    raise RuntimeError(
        f'BNO085 auto-detect: probed {len(candidates)} device(s) '
        f'(VID=0x{_BNO_VID_PID[0]:04x} PID=0x{_BNO_VID_PID[1]:04x}), '
        f'none streamed a parseable {{"yaw":...}} JSON line at {baud} baud '
        f'within {_AUTO_PROBE_TIMEOUT_S:.1f}s each. '
        'Check: (1) MCU is powered + firmware flashed, '
        '(2) user has dialout/uucp group access to /dev/tty*, '
        '(3) no other process (Arduino IDE, screen) holds the port open, '
        '(4) pass -p bno085_port:=<path> to skip auto-detect entirely.')

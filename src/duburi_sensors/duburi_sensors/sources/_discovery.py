"""USB CDC port discovery for the BNO085 auto-detect pipeline.

Extracted from bno085.py so port-scanning logic can be tested and
iterated independently of the BNO085Source sensor class.

Public API
----------
auto_detect_port(*, baud, logger) -> str
    Probe known USB CDC devices; return the first that streams BNO085 JSON.
    Raises RuntimeError with a friendly checklist if nothing answers.
"""

import glob
import json
import os
import time

import serial  # pyserial

# Probe order for `port='auto'`. by-id paths come first because they're
# stable across reboots (the Espressif USB serial number stays put even
# if the kernel renumbers ttyACM*).
_AUTO_PROBE_GLOBS = (
    '/dev/serial/by-id/usb-Espressif*',
    '/dev/serial/by-id/usb-Adafruit*',
    '/dev/serial/by-id/usb-Seeed*',
    '/dev/serial/by-id/usb-1a86*',          # CH340/CH9102 USB-serial
    '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyACM3',
    '/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB3',
)

# per-candidate probe window; covers ESP32-C3 ~2s boot + BNO init + first frame
_AUTO_PROBE_TIMEOUT_S = 7.0


def _enumerate_candidate_ports():
    """Return a de-duplicated, real-path list of ports worth probing.

    Globs expand to whatever is actually plugged in; literal paths only
    survive if the device node exists. The result preserves the
    declaration order in `_AUTO_PROBE_GLOBS`.
    """
    seen = set()
    candidates = []
    for pattern in _AUTO_PROBE_GLOBS:
        if any(ch in pattern for ch in '*?['):
            matches = sorted(glob.glob(pattern))
        else:
            matches = [pattern] if os.path.exists(pattern) else []
        for path in matches:
            try:
                real = os.path.realpath(path)
            except OSError:
                real = path
            if real in seen:
                continue
            seen.add(real)
            candidates.append(path)        # keep the human-friendly name
    return candidates


def _probe_port(path: str, baud: int, logger=None) -> bool:
    """Open `path`, read for up to `_AUTO_PROBE_TIMEOUT_S`, return True
    if at least one parseable `{"yaw":...}` JSON line arrives.

    Retries once if the device disconnects mid-probe (ESP32-C3 USB CDC
    briefly re-enumerates right after a host opens the port -- select()
    reports readable but read() returns EOF, raising SerialException).
    """
    for attempt in range(2):
        try:
            # dtr=True (default) required for ESP32-C3 built-in USB CDC (HWCDC
            # checks DTR before sending data; dtr=False silently drops all output).
            s = serial.Serial()
            s.port     = path
            s.baudrate = baud
            s.timeout  = 0.2
            s.open()
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
        time.sleep(1.5)   # wait for ESP32 to re-enumerate before retry

    return False


def auto_detect_port(*, baud: int = 115200, logger=None) -> str:
    """Probe known USB CDC devices, return the first that streams BNO085 JSON.

    Raises RuntimeError with a friendly checklist if nothing answers.
    """
    candidates = _enumerate_candidate_ports()
    if not candidates:
        raise RuntimeError(
            'BNO085 auto-detect: no candidate serial devices present. '
            'Plug the ESP32-C3 in (USB CDC), then retry. Looked for: '
            f'{list(_AUTO_PROBE_GLOBS)}')

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
        f'BNO085 auto-detect: probed {len(candidates)} device(s), none streamed '
        f'a parseable {{"yaw":...}} JSON line at {baud} baud within '
        f'{_AUTO_PROBE_TIMEOUT_S:.1f}s each. '
        'Check: (1) the MCU is powered + the firmware is flashed, '
        '(2) the host user has dialout/uucp group access to /dev/tty*, '
        '(3) no other process (Arduino IDE Serial Monitor, screen, ...) '
        'is holding the port open, '
        '(4) pass -p bno085_port:=/dev/ttyACM0 (or whichever tty the ESP32 '
        'enumerates as) to skip auto-detect entirely.')

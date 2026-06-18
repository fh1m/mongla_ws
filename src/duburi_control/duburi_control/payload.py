"""ESP32-C3 payload board driver.

Fires torpedos (channels 1, 2) and droppers (channels 3, 4) by sending
a single ASCII digit over USB serial.  Protocol is write-only -- no
reader thread required.  Auto-detect logic mirrors bno085.py: scans
``/dev/serial/by-id/`` globs (Espressif + CH340) and falls back to
``/dev/ttyACM*`` / ``/dev/ttyUSB*``, excluding any ports already claimed
by other drivers (e.g. BNO085).
"""

from __future__ import annotations

import glob
import logging
import time
from typing import Optional

try:
    import serial as _serial_mod
    _SERIAL_OK = True
except ImportError:
    _serial_mod = None  # type: ignore[assignment]
    _SERIAL_OK = False

_LOG = logging.getLogger(__name__)

CHANNEL_NAMES: dict[int, str] = {
    1: 'torpedo_1',
    2: 'torpedo_2',
    3: 'dropper_1',
    4: 'dropper_2',
}

_PORT_GLOBS: list[str] = [
    '/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit*',
    '/dev/serial/by-id/usb-Espressif_*',
    '/dev/serial/by-id/usb-1a86_USB_Single_Serial*',  # CH340
    '/dev/serial/by-id/usb-1a86_*',
    '/dev/ttyACM[0-3]',
    '/dev/ttyUSB[0-3]',
]


class PayloadDriver:
    """Serial interface to the ESP32-C3 payload board.

    Usage::

        driver = PayloadDriver()
        if driver.connect():
            driver.fire(1)   # fire torpedo 1
            driver.fire(3)   # drop dropper 1
        driver.disconnect()
    """

    def __init__(self) -> None:
        self._port: Optional[object] = None  # serial.Serial when open
        self._port_path: str = ''

    # ------------------------------------------------------------------ #
    #  Public API                                                          #
    # ------------------------------------------------------------------ #

    @staticmethod
    def auto_detect_port(exclude: set[str] | None = None) -> str | None:
        """Scan known globs and return the first viable port path.

        Parameters
        ----------
        exclude:
            Set of port paths to skip (e.g. the BNO085 port).
        """
        seen: set[str] = set()
        exclude = exclude or set()
        for pattern in _PORT_GLOBS:
            for path in sorted(glob.glob(pattern)):
                try:
                    import os
                    real = os.path.realpath(path)
                except Exception:
                    real = path
                if real in seen or real in exclude or path in exclude:
                    continue
                seen.add(real)
                return real
        return None

    def connect(self, port: str | None = None,
                exclude: set[str] | None = None,
                baud: int = 115200,
                timeout: float = 1.0) -> bool:
        """Open the serial port.

        Parameters
        ----------
        port:
            Explicit path (e.g. ``'/dev/ttyACM1'``).  If ``None``, calls
            :meth:`auto_detect_port`.
        exclude:
            Ports to skip during auto-detect (forwarded verbatim).
        baud:
            Baud rate (default 115200 matches ESP32 firmware).
        timeout:
            Write timeout in seconds.

        Returns ``True`` on success.
        """
        if not _SERIAL_OK:
            _LOG.error('[PAYLOAD] pyserial not installed — cannot open port')
            return False

        resolved = port or self.auto_detect_port(exclude)
        if not resolved:
            _LOG.warning('[PAYLOAD] no port found during auto-detect')
            return False

        try:
            self._port = _serial_mod.Serial(  # type: ignore[union-attr]
                resolved,
                baudrate=baud,
                timeout=timeout,
                write_timeout=timeout,
                dsrdtr=False,
                rtscts=False,
            )
            # Brief settle — DTR=False prevents ESP32 reset on open
            time.sleep(0.1)
            self._port_path = resolved
            _LOG.info(f'[PAYLOAD] connected on {resolved} @ {baud} baud')
            return True
        except Exception as exc:
            _LOG.warning(f'[PAYLOAD] open failed {resolved}: {exc}')
            self._port = None
            return False

    def fire(self, channel: int) -> bool:
        """Fire payload channel.

        Parameters
        ----------
        channel:
            1 = torpedo_1, 2 = torpedo_2, 3 = dropper_1, 4 = dropper_2.

        Sends a single ASCII digit (b'1' .. b'4') over serial.
        Returns ``True`` if the byte was written without error.
        """
        if channel not in CHANNEL_NAMES:
            _LOG.error(f'[PAYLOAD] invalid channel {channel} (must be 1-4)')
            return False
        if not self.is_ready:
            _LOG.warning(f'[PAYLOAD] fire ch={channel} ({CHANNEL_NAMES[channel]}) — port not open')
            return False
        try:
            self._port.write(bytes([0x30 + channel]))  # type: ignore[union-attr]
            self._port.flush()                          # type: ignore[union-attr]
            _LOG.info(f'[PAYLOAD] fired ch={channel} ({CHANNEL_NAMES[channel]})')
            return True
        except Exception as exc:
            _LOG.error(f'[PAYLOAD] write error ch={channel}: {exc}')
            return False

    def disconnect(self) -> None:
        """Close the serial port."""
        if self._port is not None:
            try:
                self._port.close()  # type: ignore[union-attr]
            except Exception:
                pass
            self._port = None
            _LOG.info(f'[PAYLOAD] disconnected {self._port_path}')
            self._port_path = ''

    @property
    def is_ready(self) -> bool:
        """True when the serial port is open."""
        return self._port is not None and getattr(self._port, 'is_open', False)

    @property
    def port_path(self) -> str:
        """The path of the currently open port, or ``''`` if not connected."""
        return self._port_path

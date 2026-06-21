"""Payload board driver (ESP32 / CH340 DevKit V1 or similar).

Fires torpedos (channels 1, 2) and droppers (channels 3, 4) by sending
a single ASCII digit over USB serial.  Protocol is write-only.

Distinguishing BNO085 vs payload board
---------------------------------------
BNO085 streams JSON (``{"yaw":...}``) continuously on an ESP32-C3 JTAG
device (303a:1001).  The payload board (now a CH340-based DevKit V1,
1a86:7523) is silent unless commanded.  The manager node passes the actual
port path held by the BNO source as ``exclude`` so auto-detect always picks
the silent (payload) device.
"""

from __future__ import annotations

import glob
import logging
import os
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
    '/dev/serial/by-id/usb-1a86_USB_Serial*',         # classic CH340 (1a86:7523)
    '/dev/serial/by-id/usb-1a86_USB_Single_Serial*',  # CH9102 variant
    '/dev/serial/by-id/usb-1a86_*',                   # any other 1a86
    '/dev/ttyUSB[0-3]',                               # CH340 fallback (no by-id)
]

_CHANNEL_MAP_STR = '  ch1=torpedo_1  ch2=torpedo_2  ch3=dropper_1  ch4=dropper_2'


class PayloadDriver:
    """Serial interface to the ESP32-C3 payload board."""

    def __init__(self) -> None:
        self._port: Optional[object] = None  # serial.Serial when open
        self._port_path: str = ''

    @staticmethod
    def auto_detect_port(exclude: set[str] | None = None) -> str | None:
        """Return first viable port path not in ``exclude``."""
        seen: set[str] = set()
        exclude = exclude or set()
        for pattern in _PORT_GLOBS:
            for path in sorted(glob.glob(pattern)):
                try:
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
        """Open the payload serial port.

        ``port=None`` runs auto-detect; ``exclude`` is the set of ports
        already claimed by other drivers (pass the BNO085 port path).
        Returns ``True`` on success.
        """
        if not _SERIAL_OK:
            _LOG.error('[PAYLOAD] pyserial not installed — cannot open port')
            return False

        resolved = port or self.auto_detect_port(exclude)
        if not resolved:
            _LOG.warning('[PAYLOAD] no port found (auto-detect excluded: %s)', exclude)
            return False

        try:
            # Open with dtr=False to avoid triggering the ESP32-C3 auto-reset
            # circuit on dev boards (DTR-on-open pulses EN via the RC cap →
            # board resets → GPIO glitch → relay/solenoid fires before firmware
            # initialises the pin HIGH).
            # DTR is NOT asserted at all: HWCDC's DTR gate only applies to the
            # device→host direction.  The host→device path (our fire byte) works
            # regardless of DTR state, so we never need dtr=True here.
            _p = _serial_mod.Serial()  # type: ignore[union-attr]
            _p.port         = resolved
            _p.baudrate     = baud
            _p.timeout      = timeout
            _p.write_timeout = timeout
            _p.rtscts       = False
            _p.dtr          = False    # prevent DTR→EN reset pulse (CH340 DevKit V1)
            _p.rts          = False    # prevent RTS toggle (esptool reset sequence)
            _p.open()
            time.sleep(0.5)            # USB CDC settle (no reset to wait for)
            _p.reset_input_buffer()    # discard any spurious boot noise
            self._port = _p
            self._port_path = resolved
            _LOG.info('[PAYLOAD] torpedo+dropper board connected on %s @ %d baud%s',
                      resolved, baud, _CHANNEL_MAP_STR)
            return True
        except Exception as exc:
            _LOG.warning('[PAYLOAD] open failed %s: %s', resolved, exc)
            self._port = None
            return False

    def fire(self, channel: int) -> bool:
        """Fire payload channel (1/2 = torpedo, 3/4 = dropper).

        Sends a single ASCII digit over serial; ESP32 firmware pulls the
        corresponding GPIO LOW for 500 ms to actuate the relay/solenoid.
        Returns ``True`` if the byte was written without error.
        """
        if channel not in CHANNEL_NAMES:
            _LOG.error('[PAYLOAD] invalid channel %d (must be 1-4)', channel)
            return False
        if not self.is_ready:
            _LOG.warning('[PAYLOAD] fire ch=%d — port not open (payload not connected)',
                         channel)
            return False
        name = CHANNEL_NAMES[channel]
        try:
            self._port.write(bytes([0x30 + channel]))  # type: ignore[union-attr]
            self._port.flush()                          # type: ignore[union-attr]
            _LOG.info("[PAYLOAD] serial '%d' sent → %s LAUNCHED", channel, name)
            return True
        except Exception as exc:
            _LOG.error('[PAYLOAD] write error ch=%d (%s): %s', channel, name, exc)
            return False

    def disconnect(self) -> None:
        """Close the serial port."""
        if self._port is not None:
            try:
                self._port.close()  # type: ignore[union-attr]
            except Exception:
                pass
            self._port = None
            _LOG.info('[PAYLOAD] disconnected %s', self._port_path)
            self._port_path = ''

    @property
    def is_ready(self) -> bool:
        """True when the serial port is open."""
        return self._port is not None and getattr(self._port, 'is_open', False)

    @property
    def port_path(self) -> str:
        """The path of the currently open port, or ``''`` if not connected."""
        return self._port_path

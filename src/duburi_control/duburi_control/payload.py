"""Payload board driver (ESP32 / CH340 DevKit V1 or similar).

Fires torpedos (channels 1, 2) and droppers (channels 3, 4) by sending
a single ASCII digit over USB serial.  Protocol is write-only.

Port discovery uses USB VID/PID via serial.tools.list_ports so the correct
board is always found regardless of which ttyACM*/ttyUSB* node the OS assigns.

  BNO085 (ESP32-C3 HWCDC): VID=0x303a  PID=0x1001  → handled by duburi_sensors
  Payload (CH340 DevKit V1): VID=0x1a86 PID=0x7523  → handled here
"""

from __future__ import annotations

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

# USB VID/PID for supported payload boards (checked in priority order)
_PAYLOAD_VID_PID: list[tuple[int, int]] = [
    (0x1a86, 0x7523),  # QinHeng CH340 DevKit V1 (primary)
    (0x1a86, 0x7522),  # CH340K variant
    (0x1a86, 0x55d4),  # CH9102 variant
]

# Safe probe byte — firmware only acts on ASCII '1'–'4'; '0' is ignored
VERIFY_BYTE = b'0'

_CHANNEL_MAP_STR = '  ch1=torpedo_1  ch2=torpedo_2  ch3=dropper_1  ch4=dropper_2'


class PayloadDriver:
    """Serial interface to the ESP32-C3 payload board."""

    def __init__(self) -> None:
        self._port: Optional[object] = None  # serial.Serial when open
        self._port_path: str = ''

    @staticmethod
    def auto_detect_port(exclude: set[str] | None = None) -> str | None:
        """Return first payload board port identified by USB VID/PID.

        Uses ``serial.tools.list_ports`` so the correct device is found
        regardless of which ttyACM*/ttyUSB* number the OS assigns.
        """
        try:
            from serial.tools import list_ports
        except ImportError:
            _LOG.error('[PAYLOAD] pyserial not installed — cannot auto-detect port')
            return None
        exclude_real: set[str] = set()
        for e in (exclude or set()):
            try:
                exclude_real.add(os.path.realpath(e))
            except Exception:
                exclude_real.add(e)
        for vid, pid in _PAYLOAD_VID_PID:
            for info in list_ports.comports():
                if info.vid == vid and info.pid == pid:
                    try:
                        real = os.path.realpath(info.device)
                    except Exception:
                        real = info.device
                    if real not in exclude_real:
                        return info.device
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
            time.sleep(0.2)            # CH340 settle (no reset circuit; 0.5s was conservative)
            _p.reset_input_buffer()    # discard any spurious boot noise
            # Verify the serial link is alive before declaring connected.
            # VERIFY_BYTE ('0') is outside the fire command set ('1'–'4') so no relay fires.
            _p.write(VERIFY_BYTE)
            _p.flush()
            self._port = _p
            self._port_path = resolved
            _LOG.info('[PAYLOAD] verified + connected on %s @ %d baud%s',
                      resolved, baud, _CHANNEL_MAP_STR)
            return True
        except Exception as exc:
            _LOG.warning('[PAYLOAD] open/verify failed %s: %s', resolved, exc)
            self._port = None
            return False

    def _close_dead(self) -> None:
        """Close a stale port and clear state so the next fire() triggers VID/PID re-scan."""
        dead_path = self._port_path
        try:
            self._port.close()  # type: ignore[union-attr]
        except Exception:
            pass
        self._port = None
        self._port_path = ''
        _LOG.warning('[PAYLOAD] port %s closed after I/O error — will auto-reconnect on next fire',
                     dead_path)

    def fire(self, channel: int) -> bool:
        """Fire payload channel (1/2 = torpedo, 3/4 = dropper).

        Sends a single ASCII digit over serial; ESP32 firmware pulls the
        corresponding GPIO LOW for 500 ms to actuate the relay/solenoid.
        Returns ``True`` if the byte was written without error.

        If the port went dead (USB re-enumeration / brownout), auto-reconnects
        via VID/PID re-scan before writing.  No same-call retry on write
        failure — the caller decides whether to re-issue (to avoid double-fire
        if the byte reached the relay before the USB drop).
        """
        if channel not in CHANNEL_NAMES:
            _LOG.error('[PAYLOAD] invalid channel %d (must be 1-4)', channel)
            return False
        if not self.is_ready:
            # Port never connected or died — attempt transparent recovery.
            _LOG.info('[PAYLOAD] fire ch=%d — port not ready, attempting auto-reconnect...', channel)
            self.connect()
        if not self.is_ready:
            _LOG.warning('[PAYLOAD] fire ch=%d — not connected (board absent or still re-enumerating)',
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
            # ponytail: no same-call retry — double-fire risk if byte reached relay before USB drop
            self._close_dead()
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

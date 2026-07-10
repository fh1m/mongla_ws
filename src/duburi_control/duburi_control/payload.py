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
import threading
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

# WCH out-of-tree ch341 V1.9 driver creates ttyCH341USB* instead of ttyUSB*.
# These nodes bypass the standard USB-serial sysfs path so list_ports never
# enumerates them — fall back to a glob scan after VID/PID fails.
_PAYLOAD_GLOB_FALLBACKS: list[str] = ['/dev/ttyCH341USB*']

# Safe probe byte — firmware only acts on ASCII '1'–'4'; '0' is ignored
VERIFY_BYTE = b'0'

_CHANNEL_MAP_STR = '  ch1=torpedo_1  ch2=torpedo_2  ch3=dropper_1  ch4=dropper_2'


class PayloadDriver:
    """Serial interface to the ESP32-C3 payload board."""

    def __init__(
        self,
        wired_channels: set[int] | None = None,
        reconnect_settle_s: float = 2.0,
    ) -> None:
        self._port: Optional[object] = None  # serial.Serial when open
        self._port_path: str = ''
        self._wired = wired_channels          # None = accept all 4
        self._reconnect_settle_s = reconnect_settle_s  # ESP32 boot wait after re-enum
        # Serialise fire() writes: a background mid-hold fire thread (which may be
        # mid-reconnect for ~reconnect_settle_s) must not interleave on the serial
        # port with a later standalone fire() goal. Guards fire-vs-fire ONLY --
        # the payload port is separate from the Pixhawk RC port.
        self._fire_lock = threading.Lock()

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

        # Fallback: WCH custom driver — list_ports doesn't enumerate these
        import glob
        for pattern in _PAYLOAD_GLOB_FALLBACKS:
            for dev in sorted(glob.glob(pattern)):
                try:
                    real = os.path.realpath(dev)
                except Exception:
                    real = dev
                if real not in exclude_real:
                    _LOG.debug('[PAYLOAD] VID/PID scan missed %s — found via glob fallback', dev)
                    return dev
        return None

    @staticmethod
    def _diagnose_missing_port() -> str | None:
        """Explain a payload board that is plugged in but has no serial node.

        Root cause seen on the 2026-07 carrier-board/SSD swap: the new Tegra
        kernel shipped with ``CONFIG_USB_SERIAL_CH341`` unset (no ``ch341.ko``),
        so the CH340 enumerates on the USB bus (``lsusb`` shows 1a86:7523) but
        the kernel never creates ``/dev/ttyUSB*``.  A second, classic Ubuntu
        trap is ``brltty`` grabbing the same 1a86:7523 as a braille display via
        ``usbfs``.  Both make ``list_ports`` return nothing with no clue why.

        Scans sysfs (dependency-free) for a known payload VID/PID that is
        present on the bus, and returns an actionable one-line hint, or None
        when no payload board is physically attached.
        """
        try:
            return PayloadDriver._diagnose_missing_port_unsafe()
        except Exception:  # diagnostic runs on the failure path — must never raise
            return None

    @staticmethod
    def _diagnose_missing_port_unsafe() -> str | None:
        import glob as _glob
        known = {(f'{v:04x}', f'{p:04x}') for v, p in _PAYLOAD_VID_PID}
        for vfile in _glob.glob('/sys/bus/usb/devices/*/idVendor'):
            dev_dir = os.path.dirname(vfile)
            try:
                with open(vfile) as f:
                    vid = f.read().strip().lower()
                with open(os.path.join(dev_dir, 'idProduct')) as f:
                    pid = f.read().strip().lower()
            except Exception:
                continue
            if (vid, pid) not in known:
                continue
            # Payload board IS on the bus — why is there no tty node?
            base = os.path.basename(dev_dir)
            drivers = set()
            for intf in _glob.glob(os.path.join(dev_dir, f'{base}:*')):
                link = os.path.join(intf, 'driver')
                if os.path.islink(link):
                    drivers.add(os.path.basename(os.path.realpath(link)))
            if 'usbfs' in drivers:
                return (f'payload board {vid}:{pid} is on the USB bus but held by '
                        "brltty (usbfs) — run tools/install_ch341_driver.sh, or "
                        "'sudo apt-get purge -y brltty'")
            if not any(d.startswith('ch34') for d in drivers):
                return (f'payload board {vid}:{pid} is on the USB bus but no '
                        'ch341 driver bound (kernel missing CONFIG_USB_SERIAL_CH341) '
                        '— run tools/install_ch341_driver.sh')
            return (f'payload board {vid}:{pid} bound to ch341 but no tty node yet '
                    '— replug or re-run start')
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
            hint = self._diagnose_missing_port()
            if hint:
                _LOG.warning('[PAYLOAD] no serial node — %s', hint)
            else:
                _LOG.warning('[PAYLOAD] no port found (no payload board on USB bus; '
                             'excluded: %s)', exclude)
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
        """Close stale port; caller drives reconnect."""
        dead_path = self._port_path
        try:
            self._port.close()  # type: ignore[union-attr]
        except Exception:
            pass
        self._port = None
        self._port_path = ''
        _LOG.warning('[PAYLOAD] port %s closed after I/O error', dead_path)

    def _wait_and_reconnect(self, timeout: float = 3.0) -> bool:
        """Poll VID/PID until CH340 re-enumerates then connect. Returns True on success."""
        _LOG.info('[PAYLOAD] waiting for CH340 re-enumeration (timeout=%.1fs)...', timeout)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            port = self.auto_detect_port()
            if port:
                return self.connect(port=port)
            time.sleep(0.1)
        _LOG.error('[PAYLOAD] re-enumeration timed out after %.1fs', timeout)
        return False

    def fire(self, channel: int) -> bool:
        """Fire payload channel (1/2 = torpedo, 3/4 = dropper).

        Sends a single ASCII digit over serial; ESP32 firmware pulls the
        corresponding GPIO LOW for 500 ms to actuate the relay/solenoid.
        Returns ``True`` if the byte was written without error.

        On write failure (EIO from CH340 re-enumeration after ESP32 crash):
        closes the dead port, polls VID/PID until CH340 re-appears (≤3s),
        waits ``reconnect_settle_s`` for ESP32 to finish booting, then retries
        the write once.  Accept the double-fire risk on wired channels — a missed
        fire is worse than a duplicate in competition.
        """
        if channel not in CHANNEL_NAMES:
            _LOG.error('[PAYLOAD] invalid channel %d (must be 1-4)', channel)
            return False
        if self._wired is not None and channel not in self._wired:
            _LOG.error('[PAYLOAD] ch=%d not in wired set %s — blocked', channel, self._wired)
            return False
        # One writer at a time: a mid-hold fire thread (possibly sleeping in the
        # reconnect retry) must not interleave on the port with another fire.
        with self._fire_lock:
            if not self.is_ready:
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
                _LOG.error('[PAYLOAD] write error ch=%d (%s): %s — reconnecting + retry', channel, name, exc)
                self._close_dead()
                if self._wait_and_reconnect():
                    # ponytail: settle for ESP32 boot before retry; CH340 enumerates ~1s before ESP32 ready
                    time.sleep(self._reconnect_settle_s)
                    try:
                        self._port.write(bytes([0x30 + channel]))  # type: ignore[union-attr]
                        self._port.flush()                          # type: ignore[union-attr]
                        _LOG.warning('[PAYLOAD] ch=%d retry fired after reconnect (%.1fs settle)',
                                     channel, self._reconnect_settle_s)
                        return True
                    except Exception as exc2:
                        _LOG.error('[PAYLOAD] ch=%d retry failed: %s', channel, exc2)
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

#!/usr/bin/env python3
"""
Connection + network profiles for the duburi_manager node.

AUV Ethernet topology (test platform: Duburi 4.2)
-------------------------------------------------

    Pixhawk 2.4.8 -- USB --> Raspberry Pi (BlueOS)   192.168.2.1  / GW 192.168.2.2
                                    |
                                    |  switch
                                    v
                              Jetson Orin Nano       192.168.2.69  (static)
                                    ^
                                    |  UDP 14550   (BlueOS 'inspector' endpoint,
                              ros2 stack                UDP Client -> Jetson:14550)

ROS2 side always listens on ``udpin:0.0.0.0:14550`` -- the same line works
in sim, desk-over-ethernet, and pool modes because BlueOS pushes MAVLink
at us; we never dial out. For desk mode (Pixhawk plugged directly via USB)
we auto-detect the serial device.

BlueOS endpoint config (web UI -> Vehicle -> Pixhawk -> Endpoints)::

    Name:   inspector
    Type:   UDP Client
    IP:     192.168.2.69         (Jetson static IP)
    Port:   14550

BNO085 + Pixhawk-over-BlueOS on the same Jetson
------------------------------------------------
The BNO085 (ESP32-C3 USB CDC) and the Pixhawk (via BlueOS UDP) both
connect to the Jetson simultaneously and there is NO conflict:

  * BNO085  -> USB CDC  -> Jetson serial port  (pyserial read only)
  * Pixhawk -> BlueOS   -> UDP 14550           (pymavlink UDP socket)

These are entirely separate OS-level file descriptors. Pool mode is the
natural setup when Pixhawk is wired through BlueOS on the Raspberry Pi.
Use ``mode:=desk`` (or ``mode:=auto``) when Pixhawk is also connected
directly to the Jetson via a USB cable.

Plug-and-play (``mode=auto`` or ``DEFAULT_MODE``)
--------------------------------------------------
Auto mode picks a profile by probing what is actually present:

  1. UDP 14550 already in use (BlueOS / SITL pushing MAVLink)  -> ``pool``
  2. Pixhawk USB CDC device present                            -> ``desk``
  3. Otherwise                                                 -> ``sim``

All four modes can also be forced explicitly via ``-p mode:=<name>``.

Connection override
-------------------
``-p mav_device:=/dev/ttyACM0``   -> override the connection string directly.
``-p mav_device:=udpin:0.0.0.0:14560`` -> override to a different UDP port.
When ``mav_device`` is set, the profile's default conn string is ignored.
"""

import os
import socket
from glob import glob


NETWORK = {
    'jetson_ip': '192.168.2.69',    # static IP on switch
    'blueos_ip': '192.168.2.1',     # Raspberry Pi hosting BlueOS
    'blueos_gw': '192.168.2.2',     # BlueOS gateway
    'mav_port':  14550,             # MAVLink inspector endpoint port
    'endpoint':  'inspector',       # BlueOS endpoint name (UDP Client)
}

# Sentinel value for desk profile -- replaced with an actual serial path at
# startup by resolve_profile(). Kept separate from None so a caller can
# distinguish "no explicit device" from "auto-detect serial".
_SERIAL_AUTO = 'auto:serial'

PROFILES = {
    'sim':    {'conn': 'udpin:0.0.0.0:14550', 'baud': None},   # Docker + ArduSub SITL
    'pool':   {'conn': 'udpin:0.0.0.0:14550', 'baud': None},   # Jetson on pool deck
    'laptop': {'conn': 'udpin:0.0.0.0:14550', 'baud': None},   # Laptop on same switch
    'desk':   {'conn': _SERIAL_AUTO,           'baud': 115200}, # Pixhawk over USB (auto-detect)
}

# Default: auto-detect so operators never need to pass -p mode:= at all.
DEFAULT_MODE = 'auto'

# ---------------------------------------------------------------------- #
#  Serial auto-detection                                                  #
# ---------------------------------------------------------------------- #

# by-id names come first -- they are stable across reboots and kernel
# renumbering, unlike /dev/ttyACM*. The ESP32-C3 BNO085 board is also
# Espressif and shows up in the Espressif glob; we deliberately use the
# Pixhawk/PX4/ArduPilot/CubePilot product strings here so the probe never
# false-positives on the BNO ESP32.
_PIXHAWK_BY_ID_GLOBS = (
    '/dev/serial/by-id/*Pixhawk*',
    '/dev/serial/by-id/*PX4*',
    '/dev/serial/by-id/*ArduPilot*',
    '/dev/serial/by-id/*CubePilot*',
    '/dev/serial/by-id/*Cube_Pilot*',
    '/dev/serial/by-id/*ardupilot*',
)

# Raw ACM/USB fallback -- used ONLY when no by-id match exists. If a
# BNO ESP32 is also present, both may appear here and we might land on
# the wrong one, but that case should be caught by the by-id list above.
_PIXHAWK_RAW_FALLBACK = (
    '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyACM3',
    '/dev/ttyUSB0', '/dev/ttyUSB1',
)


def _find_pixhawk_serial() -> str | None:
    """Return the first Pixhawk-like serial device path, preferring stable
    by-id names. Returns None if nothing recognisable is found.
    """
    for pattern in _PIXHAWK_BY_ID_GLOBS:
        hits = sorted(glob(pattern))
        if hits:
            return hits[0]
    for path in _PIXHAWK_RAW_FALLBACK:
        if os.path.exists(path):
            return path
    return None


def _pixhawk_serial_present() -> bool:
    """True iff a Pixhawk / PX4 / ArduPilot / CubePilot is on USB CDC.

    We deliberately do NOT rely solely on /dev/ttyACM* node presence --
    ESP32-C3 boards (e.g. BNO085 yaw source firmware) enumerate as ACM
    too and would false-positive this probe. The by-id name is
    authoritative because the USB serial number string is stable.
    """
    return _find_pixhawk_serial() is not None


# ---------------------------------------------------------------------- #
#  UDP probe                                                              #
# ---------------------------------------------------------------------- #

def _udp_port_in_use(port: int) -> bool:
    """True iff something on this host is already bound to UDP `port`.

    BlueOS / SITL bind 14550 to push MAVLink at us; if 14550 is already
    listening, MAVLink is incoming so 'pool' is the right profile.
    """
    sample = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sample.bind(('0.0.0.0', port))
    except OSError:
        return True
    finally:
        sample.close()
    return False


# ---------------------------------------------------------------------- #
#  Mode + profile resolution                                              #
# ---------------------------------------------------------------------- #

def resolve_mode(requested: str, *, logger=None) -> str:
    """Return a concrete profile name -- never 'auto'.

    `requested` may be 'auto' or any explicit profile name. Unknown
    names fall back to auto-detect (then log a warning).
    """
    name = (requested or '').strip().lower() or DEFAULT_MODE

    if name in PROFILES:
        return name

    if name != 'auto':
        if logger:
            logger.warning(
                f"[NET  ] unknown mode {requested!r}; falling back to auto-detect")
        # fall through to auto

    # ---- Plug-and-play: probe what's present ------------------------ #
    if _udp_port_in_use(NETWORK['mav_port']):
        choice, why = 'pool', f"UDP {NETWORK['mav_port']} already listening (BlueOS / SITL)"
    elif _pixhawk_serial_present():
        choice, why = 'desk', 'Pixhawk USB CDC node present'
    else:
        choice, why = 'sim',  'no UDP / serial evidence; assuming local SITL'

    if logger:
        logger.info(f"[NET  ] auto-detect picked mode={choice!r} ({why})")
    return choice


def resolve_profile(mode: str, *, mav_device: str = '', logger=None) -> dict:
    """Return a ready-to-use ``{'conn': ..., 'baud': ...}`` dict.

    Parameters
    ----------
    mode
        A concrete profile name (one of PROFILES keys). Use resolve_mode()
        first if you have a raw 'auto' request from the operator.
    mav_device
        When non-empty, overrides the profile's connection string. Accepts
        either a device path (``/dev/ttyACM0``) or a pymavlink connection
        string (``udpin:0.0.0.0:14560``). Serial paths get baud=115200
        unless the profile already supplies one.
    """
    profile = dict(PROFILES.get(mode, PROFILES['sim']))   # shallow copy

    if mav_device:
        profile['conn'] = mav_device
        if mav_device.startswith('/dev/') and profile['baud'] is None:
            profile['baud'] = 115200
        if logger:
            logger.info(f'[NET  ] mav_device override -> {mav_device}')

    elif profile['conn'] == _SERIAL_AUTO:
        path = _find_pixhawk_serial()
        if path is None:
            if logger:
                logger.warning(
                    '[NET  ] desk mode: no Pixhawk serial device found; '
                    'falling back to UDP 14550 (treat as pool)')
            profile = dict(PROFILES['pool'])
        else:
            if logger:
                logger.info(f'[NET  ] desk mode: auto-picked serial {path}')
            profile['conn'] = path

    return profile


def describe_endpoint(mode: str, mav_device: str = '') -> str:
    """Human-readable connection string for the manager startup banner."""
    if mav_device:
        return mav_device
    profile = PROFILES.get(mode, PROFILES['sim'])
    conn = profile['conn']
    if conn == _SERIAL_AUTO:
        path = _find_pixhawk_serial()
        return path if path else '(no Pixhawk serial found)'
    return conn

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
at us; we never dial out. For desk mode (Pixhawk plugged directly via
USB) we fall back to the serial device.

BlueOS endpoint config (web UI -> Vehicle -> Pixhawk -> Endpoints)::

    Name:   inspector
    Type:   UDP Client
    IP:     192.168.2.69         (Jetson static IP)
    Port:   14550

Plug-and-play (`mode=auto`)
---------------------------
Auto mode picks a profile by checking what's actually present on the
host:

  1. UDP socket already serving 14550 from BlueOS  -> ``pool``
     (any non-loopback bind on 14550 means MAVLink is pushed at us).
  2. Pixhawk over USB CDC (``/dev/ttyACM*``)        -> ``desk``
  3. Otherwise                                       -> ``sim``
     (assumes ArduSub SITL is running and pushing UDP to 14550).

Pool / laptop / sim share the same ``udpin:0.0.0.0:14550`` connection
string, so picking ``pool`` vs ``sim`` is purely cosmetic for the
banner -- both work without code change.
"""

import socket
from glob import glob


NETWORK = {
    'jetson_ip': '192.168.2.69',    # static IP on switch
    'blueos_ip': '192.168.2.1',     # Raspberry Pi hosting BlueOS
    'blueos_gw': '192.168.2.2',     # BlueOS gateway
    'mav_port':  14550,             # MAVLink inspector endpoint port
    'endpoint':  'inspector',       # BlueOS endpoint name (UDP Client)
}

PROFILES = {
    'sim':    {'conn': 'udpin:0.0.0.0:14550', 'baud': None},   # Docker + ArduSub SITL
    'pool':   {'conn': 'udpin:0.0.0.0:14550', 'baud': None},   # Jetson on pool deck
    'laptop': {'conn': 'udpin:0.0.0.0:14550', 'baud': None},   # Laptop on same switch
    'desk':   {'conn': '/dev/ttyACM0',        'baud': 115200}, # Pixhawk over USB
}

# Mode used when the operator passes nothing. We default to 'sim' because
# every real mode (pool / laptop / sim) shares the same UDP connection
# string, so 'sim' works out-of-box on the Jetson (BlueOS pushes to 14550),
# inside the docker (SITL pushes to 14550), AND on a laptop on the same
# switch -- zero config needed. Operators can still opt into 'auto',
# 'pool', 'laptop', or 'desk' explicitly via -p mode:=...
DEFAULT_MODE = 'sim'


# ---------------------------------------------------------------------- #
#  Auto-mode resolver -- picks a profile based on what's plugged in      #
# ---------------------------------------------------------------------- #

def _udp_port_in_use(port: int) -> bool:
    """True iff something on this host is already bound to UDP `port`.

    BlueOS / SITL bind 14550 to push MAVLink at us; if 14550 is
    listening, MAVLink is incoming, so 'pool' is the right profile.
    Best-effort: try to bind ourselves; if EADDRINUSE comes back the
    port is taken.
    """
    sample = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sample.bind(('0.0.0.0', port))
    except OSError:
        return True
    finally:
        sample.close()
    return False


def _pixhawk_serial_present() -> bool:
    """True iff a Pixhawk / PX4 / ArduPilot / CubePilot is on USB CDC.

    We deliberately do NOT fall through to any /dev/ttyACM* node --
    ESP32-C3 boards (e.g. BNO085 yaw source firmware) enumerate as ACM
    too and would false-positive this probe, flipping auto-mode from
    'sim' to 'desk' the moment a BNO is plugged in. The by-id name is
    authoritative because the USB serial number string is stable across
    reboots and port renumbering.
    """
    hits = (glob('/dev/serial/by-id/*Pixhawk*')
            + glob('/dev/serial/by-id/*PX4*')
            + glob('/dev/serial/by-id/*ArduPilot*')
            + glob('/dev/serial/by-id/*CubePilot*'))
    return bool(hits)


def resolve_mode(requested: str, *, logger=None) -> str:
    """Return a concrete profile name -- never 'auto'.

    `requested` may be 'auto' or any explicit profile name. Unknown
    names fall back to DEFAULT_MODE (then re-resolve if that's 'auto').
    """
    name = (requested or '').strip().lower() or DEFAULT_MODE

    if name in PROFILES:
        return name

    if name != 'auto':
        if logger:
            logger.warning(
                f"unknown mode {requested!r}; falling back to auto-detect")
        # fall through to auto

    # ---- Plug-and-play: probe what's present ------------------------ #
    if _udp_port_in_use(NETWORK['mav_port']):
        choice, why = 'pool', f"UDP {NETWORK['mav_port']} already listening (BlueOS / SITL)"
    elif _pixhawk_serial_present():
        choice, why = 'desk', 'Pixhawk USB CDC node present'
    else:
        choice, why = 'sim',  'no UDP / serial evidence; assuming local SITL'

    if logger:
        logger.info(f"[NET ] auto-detect picked mode={choice!r} ({why})")
    return choice


def describe_endpoint(mode: str) -> str:
    """Human-readable line for the manager startup banner."""
    profile = PROFILES.get(mode, PROFILES[DEFAULT_MODE if DEFAULT_MODE in PROFILES else 'sim'])
    return profile['conn']

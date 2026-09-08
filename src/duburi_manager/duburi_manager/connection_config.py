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
import time
from glob import glob


# MEASURED on the vehicle 2026-08-03, not copied from a doc: the Pi answers on
# .2 (MAC e4:5f:01:*, Raspberry Pi OUI) serving the full BlueOS 1.4.2 service set,
# and .1 is the TOPSIDE box. These two were swapped here, which is the Blue
# Robotics convention inverted -- topside .1 / BlueOS .2 is their standard, and it
# is what the hull is actually wired to. Only bringup_check's non-srot network
# section reads these, so the old values cost a false "BlueOS unreachable" rather
# than a broken link -- but that is precisely the line an operator trusts.
# ('blueos_gw' is unused anywhere in the tree; left in place, corrected.)
NETWORK = {
    'jetson_ip': '192.168.2.69',    # static IP on switch
    'topside_ip': '192.168.2.1',    # ground-station / dev box on the internal switch
    'blueos_ip': '192.168.2.2',     # Raspberry Pi hosting BlueOS (verified 2026-08-03)
    'blueos_gw': '192.168.2.2',     # the Pi is also the gateway
    'mav_port':  14550,             # MAVLink inspector endpoint port
    'endpoint':  'inspector',       # BlueOS endpoint name (UDP Client)
    'dvl_ip':    '192.168.2.201',   # Nortek Nucleus 1000 DVL
    'dvl_port':  9000,              # DVL TCP control port
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

# The default flight-controller backend, in ONE place (B47).
#
# It used to be hand-typed in each tool, and `bringup_check` still said
# "pixhawk" long after the vehicle became the SROT board. That is not a
# cosmetic disagreement: on the pixhawk branch the preflight runs its CH340
# payload probe, the SROT board enumerates as 1a86:7523 -- the SAME VID/PID as
# the payload DevKit -- and the check OPENS it. Opening that port reboots the
# flight controller (fc/port_guard.py has the measurements). A preflight tool
# whose job is to make the vehicle safe was rebooting the autopilot and
# reporting PASS.
#
# Anything that needs to know the backend without a live node reads this.
DEFAULT_FLIGHT_CONTROLLER = 'srot'

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
#  SROT board serial (direct USB Type-C -- no BlueOS)                     #
# ---------------------------------------------------------------------- #
# The SROT board (ESP32 DevKit V1) plugs straight into the dev-box / Jetson
# over USB Type-C at 115200 -- there is no BlueOS/UDP router in the loop.
# ESP32 DevKit boards enumerate via a CP210x (Silicon Labs), CH340/CH9102
# (WCH), or a native ESP32 USB-CDC bridge, so we glob all three by stable
# by-id name first, then fall back to raw ttyUSB/ttyACM nodes.
_SROT_BY_ID_GLOBS = (
    # Most specific first -- `_first_existing` takes the first match, so anything
    # that can only be the SROT board must be probed before a generic pattern.
    '/dev/serial/by-id/*SROT*',
    '/dev/serial/by-id/*CP2102*',
    '/dev/serial/by-id/*CP210*',
    '/dev/serial/by-id/*Silicon_Labs*',
    '/dev/serial/by-id/*CH340*',
    '/dev/serial/by-id/*CH910*',
    '/dev/serial/by-id/*USB_Single_Serial*',
    # The board's ESP32 DevKit uses a plain CH340 whose by-id name is the generic
    # `usb-1a86_USB_Serial-if00-port0` (no "CH340" literal). Match it by the WCH VID
    # (1a86) and the generic "USB_Serial" string -- these are STABLE across re-enum,
    # unlike the raw ttyUSB<n> node.
    '/dev/serial/by-id/*1a86*',
    '/dev/serial/by-id/*USB_Serial*',
    # `*Espressif*` and `*ESP32*` are DELIBERATELY ABSENT.
    #
    # The Pixhawk globs above go out of their way to avoid matching an Espressif
    # board ("we deliberately use the Pixhawk/PX4 product strings so the probe
    # never false-positives on the BNO ESP32"), and this list reintroduced exactly
    # that hazard by globbing the vendor name. The SROT board is a CH340, so those
    # two patterns never matched it in the first place -- all they could ever do is
    # grab a DIFFERENT Espressif device and then block at wait_heartbeat on it.
    #
    # The right long-term fix is a `SROT`-branded USB product descriptor in the
    # firmware, which is why `*SROT*` is probed first and for free.
)
_SROT_RAW_FALLBACK = (
    '/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1',
)
SROT_BAUD = 115200


def find_srot_serial() -> str | None:
    """First SROT-like USB-serial device path (stable by-id preferred), or None."""
    for pattern in _SROT_BY_ID_GLOBS:
        hits = sorted(glob(pattern))
        if hits:
            return hits[0]
    for path in _SROT_RAW_FALLBACK:
        if os.path.exists(path):
            return path
    return None


SROT_UDP_CONN = f'udpin:0.0.0.0:{NETWORK["mav_port"]}'


def probe_udp_mavlink(port: int, timeout: float = 2.0) -> str:
    """Tri-state: ``'mavlink'`` | ``'busy'`` | ``'silent'``.

    Deliberately stronger than "is the port in use". A bound port only says some
    process on THIS host holds the socket; it says nothing about a board on the
    other end. So bind, wait for a real datagram, and check the MAVLink magic
    (0xFD v2 / 0xFE v1) before claiming the board is there.

    ``'busy'`` is a THIRD answer and not a failure, which matters: MEASURED here
    2026-08-03, a leftover `duburi_manager start` from a previous run still held
    14550 (a `timeout` had killed the `ros2 run` wrapper but not its child). The
    probe could not bind, reported "no board", and then pymavlink bound the very
    same port a second later and streamed fine -- a flatly self-contradicting
    startup. Collapsing "I could not look" into "nothing is there" is how a
    diagnostic ends up lying, so it gets its own value.
    """
    sample = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sample.bind(('0.0.0.0', int(port)))
    except OSError:
        sample.close()
        return 'busy'
    deadline = time.monotonic() + timeout
    try:
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return 'silent'
            sample.settimeout(remaining)
            try:
                data, _addr = sample.recvfrom(4096)
            except socket.timeout:
                return 'silent'
            except ConnectionRefusedError:
                # ECONNREFUSED on a *receiving* UDP socket, which reads as nonsense
                # until you know the mechanism: when the previous probe socket closed,
                # the sender (Bridget) kept transmitting, the kernel answered ICMP
                # port-unreachable, and that queued error is delivered to the NEXT
                # socket bound to the same port -- on its first recv, immediately.
                #
                # MEASURED: swallowing it as "silent" made this probe flaky in a way
                # that mattered -- 3 of 6 back-to-back attempts returned 'silent' in
                # 0.02 s while the board was streaming ~90 datagrams/s, so
                # bringup_check printed "no MAVLink on UDP" three lines above a
                # healthy heartbeat read over that very link. A gate that
                # contradicts itself is a gate people stop believing.
                #
                # It is a stale notification about a socket that no longer exists,
                # so the only correct response is to keep waiting.
                continue
            except OSError:
                return 'silent'
            # Look for the MAVLink magic ANYWHERE in the datagram, and keep waiting
            # if this one has none -- do not judge on the first byte of the first
            # datagram.
            #
            # MEASURED, and it is the same fact that bit the BAD_DATA analysis: the
            # BlueOS bridge chunks the serial stream at arbitrary offsets, so only
            # ~77% of datagrams begin on a message boundary. Testing `data[0]` alone
            # therefore returned 'silent' on roughly a quarter of attempts -- in
            # 0.02 s, against a board streaming ~90 datagrams/s. Back-to-back runs
            # gave mavlink/silent/mavlink/silent at random, which is exactly the kind
            # of intermittent that gets diagnosed as a hardware fault.
            if data and (0xFD in data or 0xFE in data):
                return 'mavlink'
    finally:
        try:
            sample.close()
        except OSError:
            pass


def resolve_srot_profile(mav_device: str = '', *, logger=None,
                         udp_probe_s: float = 2.0) -> dict:
    """{'conn','baud'} for the SROT board -- auto-detects serial OR UDP.

    ``mav_device`` overrides everything and takes EITHER form:
      * a device path      -- ``/dev/ttyUSB0``, ``/dev/serial/by-id/...`` (baud added)
      * any pymavlink conn -- ``udpin:0.0.0.0:14550``, ``udpout:192.168.2.2:14550``,
                              ``tcp:...`` (no baud)

    Auto order, and the order is the point:

      1. **local USB serial** -- the designed transport (`auv-architecture-2026.md`):
         one Type-C cable, no Pi, and MEASURED at zero BAD_DATA against ~8-9% over
         the bridge. If the cable is there, it always wins.
      2. **UDP 14550** -- the transitional rig, board on the Pi behind a BlueOS
         Bridget serial->UDP bridge. Probed by waiting for an actual MAVLink
         datagram, not by checking whether the port is bound.
      3. neither -- return the UDP listener and say so LOUDLY. A board plugged in a
         moment later over USB will not be picked up by this call, so the message
         has to name the override rather than imply patience will fix it.
    """
    if mav_device:
        baud = SROT_BAUD if mav_device.startswith('/dev/') else None
        if logger:
            how = f'serial @ {SROT_BAUD}' if baud else 'network endpoint'
            logger.info(f'[NET  ] SROT mav_device override -> {mav_device} ({how})')
        return {'conn': mav_device, 'baud': baud}

    path = find_srot_serial()
    if path is not None:
        if logger:
            logger.info(f'[NET  ] SROT: auto-detected USB serial {path} @ {SROT_BAUD}')
        return {'conn': path, 'baud': SROT_BAUD}

    state = probe_udp_mavlink(NETWORK['mav_port'], udp_probe_s) if udp_probe_s > 0 \
        else 'silent'

    if state == 'mavlink' and logger:
        logger.info(
            f'[NET  ] SROT: no local USB serial, but MAVLink IS arriving on UDP '
            f'{NETWORK["mav_port"]} -- using {SROT_UDP_CONN} (board on the Pi behind '
            f'a BlueOS bridge). Link quality is measurably worse than direct USB; '
            f'prefer the Type-C cable for a water run.')
    elif state == 'busy' and logger:
        # Almost always a leftover manager from a previous run. Naming that first
        # is the difference between a 10-second fix and a hardware hunt.
        logger.warn(
            f'[NET  ] SROT: UDP {NETWORK["mav_port"]} is already held by another '
            f'process, so the board could not be probed. Using {SROT_UDP_CONN} '
            f'anyway -- if a previous `duburi_manager start` is still running, this '
            f'node will get NOTHING and block at wait_heartbeat. Check with: '
            f'ss -lunp | grep {NETWORK["mav_port"]}')
    elif logger:
        logger.error(
            f'[NET  ] SROT: NO BOARD FOUND -- no USB-serial device, and no MAVLink on '
            f'UDP {NETWORK["mav_port"]} in {udp_probe_s:.0f}s. Falling back to '
            f'{SROT_UDP_CONN}; the node will BLOCK at wait_heartbeat until something '
            f'arrives. Plug in the SROT Type-C cable, or name the link explicitly with '
            f'-p mav_device:=/dev/serial/by-id/<yours>  |  '
            f'-p mav_device:={SROT_UDP_CONN}')
        for line in diagnose_bridge():
            logger.error(f'[NET  ] {line}')
    return {'conn': SROT_UDP_CONN, 'baud': None}


def diagnose_bridge(timeout: float = 3.0) -> list:
    """Ask BlueOS why nothing is arriving, and return actionable lines.

    WHY THIS EXISTS, and it will happen again on every single reflash: a Bridget
    bridge holds an OPEN FILE DESCRIPTOR on /dev/ttyUSB0. Flashing the board makes
    the USB device re-enumerate, so that descriptor goes dead -- but the bridge is
    still listed, BlueOS still reports the serial port present, and the API answers
    exactly as it does when everything is fine. Nothing is wrong ANYWHERE that a
    human can see, and no data flows.

    OBSERVED 2026-08-06: after the rev-5 flash, neither duburi_ws nor Bondor could
    connect. Bridge listed, /dev/ttyUSB0 listed, Pi pingable, zero datagrams. Delete
    + re-POST the identical bridge and it came straight back. That is a five-second
    fix that reads like a dead board, which is why the recovery command belongs in
    the error text rather than in someone's memory.

    Best-effort and short-timeout: this runs on an error path, so it must never be
    the reason a diagnostic hangs.
    """
    import json
    import urllib.request

    base = f'http://{NETWORK["blueos_ip"]}:27353/v1.0'
    body = ('{"serial_path":"/dev/ttyUSB0","baud":115200,'
            f'"ip":"{NETWORK["topside_ip"]}",'
            '"udp_target_port":14550,"udp_listen_port":14551}')
    # Two entries, not one embedded newline: every line the caller prints gets its own
    # log prefix, and a half-prefixed command is exactly the sort of thing that gets
    # copied wrong at 2am on a pool deck.
    recreate = [
        f"  curl -s -X DELETE {base}/bridges -H 'Content-Type: application/json' -d '{body}'",
        f"  curl -s -X POST   {base}/bridges -H 'Content-Type: application/json' -d '{body}'",
    ]
    try:
        with urllib.request.urlopen(f'{base}/bridges', timeout=timeout) as resp:
            bridges = json.loads(resp.read().decode())
    except Exception:                                        # noqa: BLE001
        return [f'BlueOS Bridget is not answering on {base} -- is the Pi up, and is '
                f'this host on the AUV switch? (ping {NETWORK["blueos_ip"]})',
                'If the board is on THIS host instead, plug in the Type-C cable.']
    if not bridges:
        return ['BlueOS has NO bridge configured -- that is why nothing is arriving. '
                'Create it:'] + recreate
    return [
        f'BlueOS DOES have a bridge configured ({bridges}) and it is still silent.',
        'That is the reflash signature: a bridge holds an open fd on /dev/ttyUSB0, and '
        'flashing the board re-enumerates the device, so the fd is dead while the '
        'bridge still LISTS as healthy. Delete and re-create it -- same body, and it '
        'comes straight back:',
    ] + recreate


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

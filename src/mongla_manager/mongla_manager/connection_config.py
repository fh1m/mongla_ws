#!/usr/bin/env python3
"""How the manager finds the SROT board.

The board plugs into the Pi over ONE USB Type-C cable (a CH340 at 115200).
`resolve_srot_profile()` turns that into a pymavlink ``{'conn', 'baud'}``:

  1. ``mav_device`` set           -> use it (a device path or any pymavlink conn)
  2. exactly one SROT-like serial -> use it (``find_srot_serial``, refuses to guess)
  3. MAVLink arriving on UDP 14550 -> a serial->UDP bridge on another host
  4. nothing                       -> the UDP listener, and say so LOUDLY

Override: ``-p mav_device:=/dev/srot`` or ``-p mav_device:=udpin:0.0.0.0:14550``.
"""

import os
import socket
import time
from glob import glob


MAV_UDP_PORT = 14550    # where a bridged board's MAVLink arrives


# ---------------------------------------------------------------------- #
#  SROT board serial (direct USB Type-C)                                  #
# ---------------------------------------------------------------------- #
# The SROT board (ESP32 DevKit V1) plugs straight into the Pi over USB Type-C
# at 115200. ESP32 DevKit boards enumerate via a CP210x (Silicon Labs),
# CH340/CH9102 (WCH), or a native ESP32 USB-CDC bridge, so we glob all three by
# stable by-id name first, then fall back to raw ttyUSB/ttyACM nodes.
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
    # `*Espressif*` and `*ESP32*` are DELIBERATELY ABSENT. The SROT board is a
    # CH340, so those two patterns never match it -- all they could ever do is
    # grab a DIFFERENT Espressif device and then block at wait_heartbeat on it.
    #
    # The right long-term fix is a `SROT`-branded USB product descriptor in the
    # firmware, which is why `*SROT*` is probed first and for free.
)
_SROT_RAW_FALLBACK = (
    '/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1',
)
SROT_BAUD = 115200


# A udev symlink naming THIS board by its physical USB port -- the only stable
# identity a serial-less CH340 has. Written by `kutum/system/udev/make_srot_rule.sh`.
SROT_UDEV_LINK = '/dev/srot'


class AmbiguousSrotSerial(RuntimeError):
    """More than one device could be the SROT board -- refuse to guess (#20)."""


def _distinct(paths) -> list:
    """Paths that are different DEVICES (a by-id link and its ttyUSB are one)."""
    seen, out = set(), []
    for p in paths:
        real = os.path.realpath(p)
        if real not in seen:
            seen.add(real)
            out.append(p)
    return out


def find_srot_serial() -> str | None:
    """The SROT board's serial path, None if absent -- and REFUSES to guess.

    ISSUE #20. The board's bridge is a plain CH340 with NO serial number, so
    its by-id name is shared by every CH340 on the machine -- an ESC flasher, a
    LoRa dongle, a spare ESP32 DevKit. This used to return the FIRST match, and
    opening the wrong one resets whatever it is (DTR) and then hung in
    wait_heartbeat. Order now:

      1. `/dev/srot` -- the udev link for this board's physical port, if made
      2. a SROT-branded by-id name (the firmware ask) -- unique by construction
      3. the generic patterns TOGETHER, de-duplicated by device: exactly one ->
         use it; more than one -> AmbiguousSrotSerial, listing them
      4. raw ttyUSB/ACM nodes, under the same one-or-refuse rule
    """
    if os.path.exists(SROT_UDEV_LINK):
        return SROT_UDEV_LINK
    branded = _distinct(sorted(glob('/dev/serial/by-id/*SROT*')))
    if len(branded) == 1:
        return branded[0]
    generic = []
    for pattern in _SROT_BY_ID_GLOBS:
        generic += sorted(glob(pattern))
    tiers = (_distinct(branded + generic),
             _distinct([p for p in _SROT_RAW_FALLBACK if os.path.exists(p)]))
    for found in tiers:
        if len(found) == 1:
            return found[0]
        if len(found) > 1:
            raise AmbiguousSrotSerial(
                f'{len(found)} devices could be the SROT board: {found}. Opening '
                f'the wrong one RESETS it. Unplug the others, pass '
                f'mav_device:=<path>, or pin this board once with '
                f'kutum/system/udev/make_srot_rule.sh (-> {SROT_UDEV_LINK}).')
    return None


SROT_UDP_CONN = f'udpin:0.0.0.0:{MAV_UDP_PORT}'


def probe_udp_mavlink(port: int, timeout: float = 2.0) -> str:
    """Tri-state: ``'mavlink'`` | ``'busy'`` | ``'silent'``.

    Deliberately stronger than "is the port in use". A bound port only says some
    process on THIS host holds the socket; it says nothing about a board on the
    other end. So bind, wait for a real datagram, and check the MAVLink magic
    (0xFD v2 / 0xFE v1) before claiming the board is there.

    ``'busy'`` is a THIRD answer and not a failure, which matters: MEASURED here
    2026-08-03, a leftover `mongla_manager start` from a previous run still held
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
                # the sender kept transmitting, the kernel answered ICMP
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
            # serial->UDP bridge chunks the serial stream at arbitrary offsets, so only
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

      1. **local USB serial** -- the designed transport (`srot-architecture.md`):
         one Type-C cable, no Pi, and MEASURED at zero BAD_DATA against ~8-9% over
         the bridge. If the cable is there, it always wins.
      2. **UDP 14550** -- the board on another host behind a serial->UDP
         bridge. Probed by waiting for an actual MAVLink
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

    state = probe_udp_mavlink(MAV_UDP_PORT, udp_probe_s) if udp_probe_s > 0 \
        else 'silent'

    if state == 'mavlink' and logger:
        logger.info(
            f'[NET  ] SROT: no local USB serial, but MAVLink IS arriving on UDP '
            f'{MAV_UDP_PORT} -- using {SROT_UDP_CONN} (board behind '
            f'a serial->UDP bridge). Link quality is measurably worse than direct USB; '
            f'prefer the Type-C cable for a water run.')
    elif state == 'busy' and logger:
        # Almost always a leftover manager from a previous run. Naming that first
        # is the difference between a 10-second fix and a hardware hunt.
        logger.warn(
            f'[NET  ] SROT: UDP {MAV_UDP_PORT} is already held by another '
            f'process, so the board could not be probed. Using {SROT_UDP_CONN} '
            f'anyway -- if a previous `mongla_manager start` is still running, this '
            f'node will get NOTHING and block at wait_heartbeat. Check with: '
            f'ss -lunp | grep {MAV_UDP_PORT}')
    elif logger:
        logger.error(
            f'[NET  ] SROT: NO BOARD FOUND -- no USB-serial device, and no MAVLink on '
            f'UDP {MAV_UDP_PORT} in {udp_probe_s:.0f}s. Falling back to '
            f'{SROT_UDP_CONN}; the node will BLOCK at wait_heartbeat until something '
            f'arrives. Plug in the SROT Type-C cable, or name the link explicitly with '
            f'-p mav_device:=/dev/serial/by-id/<yours>  |  '
            f'-p mav_device:={SROT_UDP_CONN}')
    return {'conn': SROT_UDP_CONN, 'baud': None}

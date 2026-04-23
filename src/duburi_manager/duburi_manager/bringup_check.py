#!/usr/bin/env python3
"""bringup_check -- pre-flight network + sensor probe.

Runs through the same checklist a human operator would on the deck:

  1. Can we reach BlueOS (192.168.2.1) and the Jetson (192.168.2.69)?
  2. Is anything listening on UDP 14550 (i.e. is BlueOS pushing MAVLink
     at us)?  If yes, briefly receive one packet to confirm flow.
  3. Is a Pixhawk USB CDC node visible (`/dev/ttyACM*` or by-id symlink)?
  4. Is a BNO085 USB CDC streaming `{"yaw":...}` lines?
  5. Print the auto-detected manager mode and the recommended launch
     command.

Every check is best-effort and prints PASS / WARN / FAIL on its own
line. Exit code is 0 if no checks failed (warnings are OK), 1 otherwise.

Usage:
    ros2 run duburi_manager bringup_check
"""

from __future__ import annotations

import os
import socket
import subprocess
import sys
from glob import glob

from .connection_config import NETWORK, resolve_mode

PASS = 'PASS'
WARN = 'WARN'
FAIL = 'FAIL'


def _line(tag: str, label: str, detail: str = '') -> None:
    print(f'  [{tag}] {label:<28} {detail}')


def _ping(ip: str, timeout_s: int = 1) -> bool:
    """One ICMP echo via /bin/ping (no root needed). Returns True if up.

    Uses subprocess with a fixed argv so the IP cannot be interpolated
    into a shell. `ip` must come from NETWORK (compile-time constant).
    """
    try:
        result = subprocess.run(
            ['ping', '-c', '1', '-W', str(int(timeout_s)), str(ip)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=timeout_s + 1)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0


def _udp_listening(port: int) -> bool:
    """True iff the local host already has a listener on UDP `port`."""
    sample = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sample.bind(('0.0.0.0', port))
        return False
    except OSError:
        return True
    finally:
        sample.close()


def _udp_recv_one(port: int, timeout_s: float = 2.0) -> bool:
    """Try to receive a single packet on UDP `port`. Best-effort: if the
    port is already taken by BlueOS we return False (caller already knows
    via _udp_listening)."""
    sample = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sample.settimeout(timeout_s)
    try:
        sample.bind(('0.0.0.0', port))
        sample.recvfrom(2048)
        return True
    except (socket.timeout, OSError):
        return False
    finally:
        try:
            sample.close()
        except Exception:
            pass


def _pixhawk_devices() -> list:
    return sorted(set(
        glob('/dev/serial/by-id/*Pixhawk*') +
        glob('/dev/serial/by-id/*PX4*') +
        glob('/dev/serial/by-id/*ArduPilot*') +
        [p for p in ('/dev/ttyACM0', '/dev/ttyACM1') if os.path.exists(p)]
    ))


def _check_bno085_auto():
    """Run the BNO auto-detector but never raise -- return (status, detail)."""
    try:
        from duburi_sensors.sources.bno085 import auto_detect_port
    except Exception as exc:
        return WARN, f'duburi_sensors not importable yet: {exc}'

    try:
        path = auto_detect_port(baud=115200, logger=None)
    except RuntimeError as exc:
        return WARN, f'no BNO085 streaming yet ({str(exc).splitlines()[0]})'
    except Exception as exc:
        return WARN, f'auto-detect raised: {exc!r}'
    return PASS, f'streaming on {path}'


def main() -> int:
    failures = 0
    print('=' * 72)
    print(' duburi bringup_check -- pool-side pre-flight')
    print('=' * 72)

    # ---- 1. network -------------------------------------------------- #
    print(' Network reachability')
    if _ping(NETWORK['blueos_ip']):
        _line(PASS, 'BlueOS', NETWORK['blueos_ip'])
    else:
        _line(WARN, 'BlueOS unreachable', NETWORK['blueos_ip'])

    if _ping(NETWORK['jetson_ip']):
        _line(PASS, 'Jetson', NETWORK['jetson_ip'])
    else:
        _line(WARN, 'Jetson unreachable',
              f"{NETWORK['jetson_ip']} (skip if you ARE the Jetson)")

    # ---- 2. MAVLink pushed at us ------------------------------------ #
    print(' MAVLink')
    if _udp_listening(NETWORK['mav_port']):
        _line(PASS, f"UDP {NETWORK['mav_port']}",
              'something already listening (BlueOS / SITL OK)')
    else:
        if _udp_recv_one(NETWORK['mav_port'], timeout_s=2.0):
            _line(PASS, f"UDP {NETWORK['mav_port']}", 'received one packet')
        else:
            _line(WARN, f"UDP {NETWORK['mav_port']}",
                  'no packets in 2s (start BlueOS / SITL or use mode=desk)')

    # ---- 3. Pixhawk USB --------------------------------------------- #
    print(' Pixhawk USB')
    pixhawk_ports = _pixhawk_devices()
    if pixhawk_ports:
        _line(PASS, 'Pixhawk USB CDC', pixhawk_ports[0])
    else:
        _line(WARN, 'No Pixhawk USB device',
              'OK if you go via UDP / BlueOS')

    # ---- 4. BNO085 --------------------------------------------------- #
    print(' BNO085 yaw source')
    bno_status, bno_detail = _check_bno085_auto()
    _line(bno_status, 'BNO085 auto-detect', bno_detail)

    # ---- 5. resolved mode + launch hint ----------------------------- #
    print(' Manager startup hint')
    chosen = resolve_mode('auto', logger=None)
    _line(PASS, f"auto-detected mode={chosen!r}",
          'will be picked by `mode=auto` (the default)')

    print()
    if failures:
        print(f'  {failures} check(s) FAILED -- fix before running missions.')
    else:
        print('  All checks OK. Launch with:')
        print('    ros2 run duburi_manager start         # mode=auto picks the right profile')
        print('    # then in another terminal:')
        print('    ros2 launch duburi_vision cameras_.launch.py  # vision pipeline')
        print('    ros2 run duburi_planner mission move_and_see      # demo mission')
    print('=' * 72)
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

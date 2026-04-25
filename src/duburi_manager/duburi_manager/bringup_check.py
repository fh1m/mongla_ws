#!/usr/bin/env python3
"""bringup_check -- pre-flight network + sensor probe.

Runs through the same checklist a human operator would on the deck:

  1. Network: can we reach BlueOS (192.168.2.1) and the Jetson (192.168.2.69)?
  2. MAVLink: is anything listening on UDP 14550 / do we receive a packet?
  3. Pixhawk USB CDC: any device visible?
  4. DVL: is the Nucleus 1000 (192.168.2.201) reachable and TCP port 9000 open?
  5. BNO085: is the USB CDC streaming {"yaw":...} JSON?
  6. Manager startup hint: auto-detected mode + recommended launch command.

Every check prints PASS / WARN / FAIL on its own line.
Exit code is 0 if no FAIL (WARNs are OK), 1 otherwise.

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
    print(f'  [{tag}] {label:<32} {detail}')


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
    """Try to receive a single packet on UDP `port`."""
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


def _check_dvl(host: str, port: int) -> tuple[str, str]:
    """Ping DVL IP then attempt TCP connection to the control port."""
    if not _ping(host, timeout_s=1):
        return WARN, f'{host} unreachable (DVL off, not on switch, or sim mode)'
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(2.0)
    try:
        s.connect((host, port))
        s.close()
        return PASS, f'reachable at {host}:{port}'
    except (socket.timeout, ConnectionRefusedError, OSError) as exc:
        return WARN, f'ping OK but TCP {port} not accepting ({exc})'


def _check_bno085_auto() -> tuple[str, str]:
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
    warnings = 0

    def ok(label: str, detail: str = '') -> None:
        _line(PASS, label, detail)

    def warn(label: str, detail: str = '') -> None:
        nonlocal warnings
        warnings += 1
        _line(WARN, label, detail)

    def fail(label: str, detail: str = '') -> None:
        nonlocal failures
        failures += 1
        _line(FAIL, label, detail)

    print('=' * 72)
    print(' duburi bringup_check -- pool-side pre-flight')
    print('=' * 72)

    # ---- 1. network -------------------------------------------------- #
    print(' Network reachability')
    if _ping(NETWORK['blueos_ip']):
        ok('BlueOS', NETWORK['blueos_ip'])
    else:
        warn('BlueOS unreachable',
             f"{NETWORK['blueos_ip']}  (expected in pool/desk mode)")

    if _ping(NETWORK['jetson_ip']):
        ok('Jetson', NETWORK['jetson_ip'])
    else:
        warn('Jetson unreachable',
             f"{NETWORK['jetson_ip']}  (skip if you ARE the Jetson)")

    # ---- 2. MAVLink -------------------------------------------------- #
    print(' MAVLink')
    if _udp_listening(NETWORK['mav_port']):
        ok(f"UDP {NETWORK['mav_port']}",
           'listener already bound (BlueOS / SITL OK)')
    elif _udp_recv_one(NETWORK['mav_port'], timeout_s=2.0):
        ok(f"UDP {NETWORK['mav_port']}", 'received one packet')
    else:
        warn(f"UDP {NETWORK['mav_port']}",
             'no packets in 2 s — start BlueOS / SITL or use mode=desk')

    # ---- 3. Pixhawk USB --------------------------------------------- #
    print(' Pixhawk USB')
    pixhawk_ports = _pixhawk_devices()
    if pixhawk_ports:
        ok('Pixhawk USB CDC', pixhawk_ports[0])
    else:
        warn('No Pixhawk USB device', 'OK for UDP/BlueOS pool mode')

    # ---- 4. DVL (Nortek Nucleus 1000) -------------------------------- #
    print(' DVL (Nortek Nucleus 1000)')
    dvl_status, dvl_detail = _check_dvl(NETWORK['dvl_ip'], NETWORK['dvl_port'])
    if dvl_status == PASS:
        ok('Nucleus 1000', dvl_detail)
    else:
        warn('Nucleus 1000', dvl_detail)

    # ---- 5. BNO085 --------------------------------------------------- #
    print(' BNO085 yaw source')
    bno_status, bno_detail = _check_bno085_auto()
    if bno_status == PASS:
        ok('BNO085 auto-detect', bno_detail)
    else:
        warn('BNO085 auto-detect', bno_detail)

    # ---- 6. resolved mode + launch hint ----------------------------- #
    print(' Manager startup hint')
    chosen = resolve_mode('auto', logger=None)
    ok(f'auto-detected mode={chosen!r}',
       'will be picked by `mode=auto` (the default)')

    print()
    if failures:
        print(f'  {failures} check(s) FAILED -- fix before running missions.')
    elif warnings:
        print(f'  {warnings} warning(s) (see above) -- review before pool day.')
        print('  Launch with:')
        _print_launch_hint()
    else:
        print('  All checks OK. Launch with:')
        _print_launch_hint()
    print('=' * 72)
    return 1 if failures else 0


def _print_launch_hint() -> None:
    print('    ros2 run duburi_manager start              '
          '# mode=auto picks the right profile')
    print('    ros2 launch duburi_vision cameras_.launch.py  '
          '# vision pipeline (optional)')
    print('    ros2 run duburi_planner mission move_and_see   '
          '# demo mission')
    print()
    print('  DVL distance commands (pool, yaw_source=dvl or bno085_dvl):')
    print('    ros2 run duburi_planner duburi move_forward_dist --distance_m 2.0')


if __name__ == '__main__':
    sys.exit(main())

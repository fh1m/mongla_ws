#!/usr/bin/env python3
"""bringup_check -- RoboSub-proof pre-flight probe.

Walks every subsystem the AUV needs and reports whether each is not just
*present* but *connected and usable*, so problems are caught on the deck
instead of mid-mission.  Sections:

  A. Compute / environment   ROS sourced, duburi pkgs import, disk free
  B. Vision dependencies     numpy<2, system cv2 (not pip-shadowed), trackers
  C. Serial drivers          ch341 (payload) present, brltty absent
  D. Network                 BlueOS + Jetson reachability
  E. MAVLink / autopilot     heartbeat, ARMED state, flight mode, battery V
  F. Pixhawk USB             direct CDC device (desk mode)
  G. Yaw source (BNO085)     USB CDC streaming {"yaw":...}
  H. DVL (Nortek Nucleus)    IP reachable + control port open
  I. Payload board (CH340)   port found + serial link verified (safe '0' byte)
  J. Cameras                 forward + downward USB cameras enumerated
  K. Vision models           TensorRT .engine built (else slow .pt fallback)
  L. Jetson power mode       MAXN for full inference FPS

Each check prints PASS / WARN / FAIL.  Exit code is 0 when no FAIL (WARNs
are advisory).  Pass ``--strict`` to make any WARN also exit non-zero (a
hard pre-mission gate).  ``--skip-mavlink`` skips the UDP 14550 probe.

Usage:
    ros2 run duburi_manager bringup_check
    ros2 run duburi_manager bringup_check --strict
"""

from __future__ import annotations

import os
import shutil
import socket
import subprocess
import sys
from glob import glob

from .connection_config import NETWORK, resolve_mode

PASS = 'PASS'
WARN = 'WARN'
FAIL = 'FAIL'

# Battery thresholds (4S LiPo nominal ~14.8 V, 3S ~11.1 V). We can't know the
# pack chemistry here, so only flag an obviously-flat pack; the operator owns
# the real go/no-go on voltage.
_BATT_LOW_V = 13.5      # below this on a 4S -> WARN (recharge/swap)
_BATT_FLAT_V = 12.0     # below this -> almost certainly a dying/low pack


def _line(tag: str, label: str, detail: str = '') -> None:
    print(f'  [{tag}] {label:<34} {detail}')


# --------------------------------------------------------------------------- #
# low-level probes
# --------------------------------------------------------------------------- #
def _ping(ip: str, timeout_s: int = 1) -> bool:
    """One ICMP echo via /bin/ping (no root needed). Returns True if up.

    Fixed argv so the IP cannot be interpolated into a shell; `ip` comes from
    NETWORK (compile-time constant).
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


# VID/PIDs that are NOT the Pixhawk even though they enumerate as ttyACM/USB:
# BNO085 (ESP32-C3 native CDC) and the CH340 payload board.
_NON_PIXHAWK_VIDPID = {(0x303a, 0x1001), (0x1a86, 0x7523),
                       (0x1a86, 0x7522), (0x1a86, 0x55d4)}


def _pixhawk_devices() -> list:
    """Serial devices that are plausibly the Pixhawk (desk/USB mode).

    by-id name matches are trusted outright. The bare ttyACM* fallback is
    filtered so the BNO085 (ttyACM0 on this hull) and the CH340 payload are
    never mis-reported as the autopilot.
    """
    named = (glob('/dev/serial/by-id/*Pixhawk*') +
             glob('/dev/serial/by-id/*PX4*') +
             glob('/dev/serial/by-id/*ArduPilot*'))
    found = set(named)

    try:
        from serial.tools import list_ports
        for info in list_ports.comports():
            dev = info.device
            if not (dev.startswith('/dev/ttyACM') or dev.startswith('/dev/ttyUSB')):
                continue
            if (info.vid, info.pid) in _NON_PIXHAWK_VIDPID:
                continue
            # An unknown ACM/USB device that isn't the BNO/payload MIGHT be the
            # Pixhawk on direct USB. Only add ttyACM* (ArduPilot enumerates ACM).
            if dev.startswith('/dev/ttyACM'):
                found.add(dev)
    except Exception:
        pass
    return sorted(found)


# --------------------------------------------------------------------------- #
# A. compute / environment
# --------------------------------------------------------------------------- #
def _check_ros_env() -> list[tuple[str, str, str]]:
    out: list[tuple[str, str, str]] = []
    distro = os.environ.get('ROS_DISTRO', '')
    if distro:
        out.append((PASS, 'ROS distro', distro))
    else:
        out.append((FAIL, 'ROS_DISTRO unset',
                    'source /opt/ros/humble/setup.bash + install/setup.bash'))
    dom = os.environ.get('ROS_DOMAIN_ID', '(unset -> 0)')
    out.append((PASS, 'ROS_DOMAIN_ID', str(dom)))

    # The three packages the mission path imports. A broken build shows here
    # BEFORE `start` fails with a confusing traceback.
    for pkg in ('duburi_control', 'duburi_interfaces', 'duburi_planner'):
        try:
            __import__(pkg)
            out.append((PASS, f'import {pkg}', 'ok'))
        except Exception as exc:  # noqa: BLE001 - report, never raise
            out.append((FAIL, f'import {pkg}',
                        f'{type(exc).__name__}: {str(exc).splitlines()[0]} '
                        '-- rebuild: ./build_duburi.sh'))
    return out


def _check_disk() -> tuple[str, str]:
    try:
        free_gb = shutil.disk_usage('/').free / 1e9
    except OSError as exc:
        return WARN, f'could not stat /: {exc}'
    if free_gb < 1.0:
        return FAIL, f'{free_gb:.1f} GB free -- logging/models will fail'
    if free_gb < 3.0:
        return WARN, f'{free_gb:.1f} GB free -- low for bag/log capture'
    return PASS, f'{free_gb:.1f} GB free'


# --------------------------------------------------------------------------- #
# B. vision dependencies (the JetPack pitfalls, known-issues E1-E3)
# --------------------------------------------------------------------------- #
def _check_vision_deps() -> list[tuple[str, str, str]]:
    out: list[tuple[str, str, str]] = []

    # numpy<2 -- numpy 2 breaks the cv_bridge / cv2 ABI => every vision node
    # dies with `_ARRAY_API not found` (known-issues E1).
    try:
        import numpy
        ver = numpy.__version__
        major = int(ver.split('.')[0])
        if major >= 2:
            out.append((FAIL, 'numpy version', f'{ver} -- MUST be <2 '
                        '(numpy 2 kills every vision node: _ARRAY_API). '
                        'pip install "numpy<2" (1.26.4)'))
        else:
            out.append((PASS, 'numpy version', f'{ver} (<2, ABI ok)'))
    except Exception as exc:  # noqa: BLE001
        out.append((WARN, 'numpy import', f'{exc}'))

    # cv2 must be the SYSTEM build (GUI-capable); a pip opencv-python shadows
    # it headless and kills vision_display (known-issues E2).
    try:
        import cv2
        path = getattr(cv2, '__file__', '')
        if 'dist-packages' in path or '/usr/lib' in path:
            out.append((PASS, 'cv2 (system)', f'{cv2.__version__}'))
        else:
            out.append((WARN, 'cv2 not system build',
                        f'{cv2.__version__} @ {path} -- pip opencv may shadow '
                        'the GUI build; uninstall pip opencv-python*'))
    except Exception as exc:  # noqa: BLE001
        out.append((FAIL, 'cv2 import', f'{exc} -- vision cannot run'))

    # pip opencv-python shadow detector (independent of the import path check).
    try:
        import importlib.metadata as _md
        shadows = [d for d in ('opencv-python', 'opencv-contrib-python',
                               'opencv-python-headless')
                   if _pkg_present(_md, d)]
        if shadows:
            out.append((WARN, 'pip opencv installed',
                        f'{", ".join(shadows)} -- shadows system cv2; '
                        'pip uninstall them (known-issues E2)'))
    except Exception:
        pass

    # trackers 2.5.0 PyPI wheel is a broken dud; 2.4.0 is the good one (E3).
    try:
        import importlib.metadata as _md
        tver = _md.version('trackers')
        if tver.startswith('2.5'):
            out.append((WARN, 'trackers version',
                        f'{tver} -- the 2.5.0 wheel is a broken 9.7 kB dud; '
                        'pip install "trackers==2.4.0" --no-deps (E3)'))
        else:
            out.append((PASS, 'trackers version', tver))
    except Exception:
        out.append((WARN, 'trackers not installed',
                    'OC-SORT/ByteTrack HUD unavailable; '
                    'pip install "trackers==2.4.0" --no-deps'))
    return out


def _pkg_present(md, name: str) -> bool:
    try:
        md.version(name)
        return True
    except Exception:
        return False


# --------------------------------------------------------------------------- #
# C. serial drivers (payload CH340 + brltty trap, known-issues E5)
# --------------------------------------------------------------------------- #
def _check_serial_drivers() -> list[tuple[str, str, str]]:
    out: list[tuple[str, str, str]] = []

    # ch341 must exist for the CH340 payload board to get a /dev/ttyUSB* node.
    # A fresh Tegra kernel (carrier-board/SSD swap) can ship without it.
    have_ch341 = subprocess.run(['modinfo', 'ch341'],
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL).returncode == 0 \
        if shutil.which('modinfo') else None
    if have_ch341 is None:
        out.append((PASS, 'ch341 driver', 'modinfo absent -- skipped'))
    elif have_ch341:
        out.append((PASS, 'ch341 driver', 'available (payload can enumerate)'))
    else:
        out.append((WARN, 'ch341 driver MISSING',
                    'payload CH340 will have no /dev/ttyUSB* -- run '
                    'tools/install_ch341_driver.sh (known-issues E5)'))

    # brltty greedily grabs the CH340 (1a86:7523) as a braille display.
    try:
        installed = subprocess.run(['dpkg', '-l', 'brltty'],
                                   capture_output=True, text=True).stdout
        if '\nii  brltty' in installed or installed.startswith('ii  brltty'):
            out.append((WARN, 'brltty installed',
                        'steals the payload CH340 via usbfs -- '
                        'sudo apt-get purge -y brltty (known-issues E5)'))
        else:
            out.append((PASS, 'brltty absent', 'payload CH340 not hijacked'))
    except (FileNotFoundError, OSError):
        pass  # not a dpkg system
    return out


# --------------------------------------------------------------------------- #
# E. MAVLink / autopilot (heartbeat + armed + mode + battery)
# --------------------------------------------------------------------------- #
def _check_mavlink() -> list[tuple[str, str, str]]:
    port = NETWORK['mav_port']

    # If a listener is already bound, the manager/SITL is running -- don't
    # steal the port; just confirm it's there.
    if _udp_listening(port):
        return [(PASS, f'UDP {port} listener bound',
                 'manager / SITL already running -- deep probe skipped')]

    # Otherwise deep-probe with pymavlink: bind, wait for a heartbeat, and
    # read ARMED state + flight mode + battery. Releases the port on exit.
    try:
        from pymavlink import mavutil
    except Exception as exc:  # noqa: BLE001
        if _udp_recv_one(port, timeout_s=2.0):
            return [(PASS, f'UDP {port}', 'received one packet (pymavlink absent)')]
        return [(WARN, f'UDP {port}', f'no packets; pymavlink absent ({exc})')]

    import time
    out: list[tuple[str, str, str]] = []
    conn = None
    try:
        conn = mavutil.mavlink_connection(f'udpin:0.0.0.0:{port}')
        # Wait for a heartbeat from a REAL autopilot -- BlueOS's mavlink-router
        # and any GCS also emit HEARTBEATs (autopilot=INVALID), and matching
        # those gives a bogus sys=0 / armed / mode reading.
        hb = None
        deadline = time.time() + 5.0
        while time.time() < deadline:
            msg = conn.recv_match(type='HEARTBEAT', timeout=1.0, blocking=True)
            if msg is None:
                continue
            if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                continue  # GCS / onboard computer / router heartbeat
            hb = msg
            conn.target_system = msg.get_srcSystem()
            conn.target_component = msg.get_srcComponent()
            break
        if hb is None:
            out.append((WARN, f'UDP {port} no autopilot heartbeat',
                        'BlueOS not pushing / SITL down / use mode=desk'))
            return out
        out.append((PASS, 'MAVLink heartbeat',
                    f'sys={conn.target_system} comp={conn.target_component}'))

        # Pre-start, ArduSub streams only what's been requested (the manager
        # sets rates at startup). Ask for the extended-status group so the
        # battery check below actually gets a SYS_STATUS.
        try:
            conn.mav.request_data_stream_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1)
        except Exception:
            pass

        # Armed state -- a deck check MUST see disarmed.
        armed = bool(hb.base_mode &
                     mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if armed:
            out.append((WARN, 'vehicle ARMED',
                        'expected DISARMED on the deck -- verify before water'))
        else:
            out.append((PASS, 'vehicle disarmed', 'safe deck state'))

        try:
            mode = conn.flightmode
            out.append((PASS, 'flight mode', str(mode)))
        except Exception:
            pass

        # Battery from SYS_STATUS (voltage_battery in mV; 0/65535 = unknown).
        volts = None
        sysst = conn.recv_match(type='SYS_STATUS', timeout=3, blocking=True)
        if sysst is not None and getattr(sysst, 'voltage_battery', 0) not in (0, 65535):
            volts = sysst.voltage_battery / 1000.0
        else:
            bat = conn.recv_match(type='BATTERY_STATUS', timeout=2, blocking=True)
            if bat is not None:
                cells = [v for v in getattr(bat, 'voltages', []) if 0 < v < 65535]
                if cells:
                    volts = sum(cells) / 1000.0
        if volts is not None:
            if volts < _BATT_FLAT_V:
                out.append((WARN, 'battery voltage',
                            f'{volts:.2f} V -- FLAT, swap/charge before mission'))
            elif volts < _BATT_LOW_V:
                out.append((WARN, 'battery voltage',
                            f'{volts:.2f} V -- low for a 4S pack'))
            else:
                out.append((PASS, 'battery voltage', f'{volts:.2f} V'))
        else:
            out.append((WARN, 'battery voltage',
                        'no SYS_STATUS/BATTERY_STATUS -- telemetry not flowing yet'))
        return out
    except Exception as exc:  # noqa: BLE001
        out.append((WARN, f'UDP {port} probe error', f'{exc}'))
        return out
    finally:
        try:
            if conn is not None:
                conn.close()
        except Exception:
            pass


# --------------------------------------------------------------------------- #
# H. DVL
# --------------------------------------------------------------------------- #
def _check_dvl(host: str, port: int) -> tuple[str, str]:
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


# --------------------------------------------------------------------------- #
# G. BNO085 yaw source
# --------------------------------------------------------------------------- #
def _check_bno085_auto() -> tuple[str, str]:
    try:
        from duburi_sensors.sources.bno085 import auto_detect_port
    except Exception as exc:  # noqa: BLE001
        return WARN, f'duburi_sensors not importable yet: {exc}'
    try:
        path = auto_detect_port(baud=115200, logger=None)
    except RuntimeError as exc:
        return WARN, f'no BNO085 streaming yet ({str(exc).splitlines()[0]})'
    except Exception as exc:  # noqa: BLE001
        return WARN, f'auto-detect raised: {exc!r}'
    return PASS, f'streaming on {path}'


# --------------------------------------------------------------------------- #
# I. payload board (CH340) -- present AND link-verified
# --------------------------------------------------------------------------- #
def _check_payload() -> tuple[str, str]:
    """Auto-detect the payload board and verify the serial link is usable.

    A successful connect writes only VERIFY_BYTE ('0'), which is outside the
    fire command set ('1'-'4'), with dtr/rts=False -- no channel can fire.
    """
    try:
        from duburi_control.payload import PayloadDriver
    except Exception as exc:  # noqa: BLE001
        return WARN, f'duburi_control.payload not importable: {exc}'

    try:
        port = PayloadDriver.auto_detect_port(exclude=set())
    except Exception as exc:  # noqa: BLE001
        return WARN, f'auto-detect raised: {exc!r}'

    if not port:
        # Explain WHY -- board on the bus but no node (ch341/brltty), or absent.
        try:
            hint = PayloadDriver._diagnose_missing_port()
        except Exception:
            hint = None
        if hint:
            return WARN, hint
        return WARN, ('no payload board on USB bus '
                      '(fire() will log-stub -- ok for control-only tests)')

    # Verify the link opens and accepts a byte.
    drv = PayloadDriver()
    try:
        ok = drv.connect(port=port)
    except Exception as exc:  # noqa: BLE001
        return WARN, f'found {port} but open/verify failed: {exc!r}'
    finally:
        try:
            drv.disconnect()
        except Exception:
            pass
    if ok:
        return PASS, f'verified serial link on {port}'
    return WARN, f'found {port} but verify failed (board not responding)'


# --------------------------------------------------------------------------- #
# J. cameras
# --------------------------------------------------------------------------- #
def _physical_cameras() -> list[str]:
    """Distinct USB video capture devices, keyed by stable by-path port.

    A single UVC camera exposes several /dev/videoN nodes (capture + metadata);
    counting by-path *ports* (strip the interface + -video-indexN suffix)
    collapses those to one entry per physical camera and is reboot-stable.
    """
    ports: dict[str, str] = {}
    for link in sorted(glob('/dev/v4l/by-path/*-video-index0')):
        base = os.path.basename(link)
        key = base.split('-video-index')[0]      # includes :1.x interface
        port = key.rsplit(':', 1)[0]              # drop interface -> USB port
        ports.setdefault(port, base)
    return list(ports.values())


def _check_cameras() -> tuple[str, str]:
    cams = _physical_cameras()
    n = len(cams)
    if n == 0:
        raw = sorted(glob('/dev/video*'))
        if raw:
            return WARN, (f'{len(raw)} /dev/video* node(s) but no by-path link '
                          '-- non-UVC or udev issue; run v4l2-ctl --list-devices')
        return WARN, 'no /dev/video* devices -- cameras unplugged?'
    detail = f'{n} USB camera(s): ' + ', '.join(
        p.split('.usb-')[-1] if '.usb-' in p else p for p in cams)
    if n >= 2:
        return PASS, detail
    return WARN, detail + '  (2 expected: forward + downward)'


# --------------------------------------------------------------------------- #
# K. vision models (TensorRT engine vs slow .pt fallback)
# --------------------------------------------------------------------------- #
def _models_dirs() -> list[str]:
    """All plausible model dirs (src dev tree + installed share), existing only.

    The .pt weights are gitignored and usually live in the src tree; the
    installed share/ copy is often empty. Scan both and merge.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    cands: list[str] = []
    # Walk up from this file (installed under .../duburi_ws/install/...) and
    # from the cwd looking for a workspace root that has src/duburi_vision/models.
    for start in (here, os.getcwd()):
        d = start
        for _ in range(8):
            probe = os.path.join(d, 'src', 'duburi_vision', 'models')
            if os.path.isdir(probe):
                cands.append(probe)
            parent = os.path.dirname(d)
            if parent == d:
                break
            d = parent
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(os.path.join(
            get_package_share_directory('duburi_vision'), 'models'))
    except Exception:
        pass
    seen: list[str] = []
    for c in cands:
        rp = os.path.realpath(c)
        if os.path.isdir(rp) and rp not in seen:
            seen.append(rp)
    return seen


def _check_models() -> tuple[str, str]:
    dirs = _models_dirs()
    if not dirs:
        return WARN, 'models dir not found -- vision detection disabled'
    # Merge by stem across dirs; a stem is engine-accelerated if a .engine
    # exists for it in ANY model dir.
    stems: dict[str, bool] = {}
    for d in dirs:
        for p in glob(os.path.join(d, '*.pt')):
            stem = os.path.splitext(os.path.basename(p))[0]
            has_engine = os.path.exists(os.path.join(d, stem + '.engine'))
            stems[stem] = stems.get(stem, False) or has_engine
    if not stems:
        return WARN, (f'no .pt weights in {", ".join(dirs)} '
                      '(gitignored -- copy them onto the Jetson)')
    with_engine = [s for s, e in stems.items() if e]
    if len(with_engine) == len(stems):
        return PASS, (f'{len(stems)} model(s), all TensorRT-accelerated '
                      '(.engine present)')
    missing = [s for s, e in stems.items() if not e]
    return WARN, (f'{len(with_engine)}/{len(stems)} models have a .engine; '
                  f'{", ".join(missing)} will run slow .pt (~3-4 Hz) -- '
                  'ros2 run duburi_vision export_engine --all (ON THE JETSON)')


# --------------------------------------------------------------------------- #
# L. Jetson power
# --------------------------------------------------------------------------- #
def _check_jetson_power() -> tuple[str, str]:
    try:
        out = subprocess.run(['nvpmodel', '-q'], capture_output=True, text=True,
                             timeout=3).stdout
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        return PASS, 'not a Jetson (nvpmodel absent) -- skipped'
    text = out.replace('\n', ' ').strip()
    low = text.lower()
    if 'maxn' in low or 'nv power mode: 0' in low or ': 0' in low:
        return PASS, f'MAXN ({text[:48]})'
    return WARN, (f'NOT MaxN ({text[:48]}) -- run: '
                  'sudo nvpmodel -m 0 && sudo jetson_clocks')


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    strict = '--strict' in argv
    skip_mav = '--skip-mavlink' in argv

    failures = 0
    warnings = 0

    def emit(status: str, label: str, detail: str = '') -> None:
        nonlocal failures, warnings
        if status == FAIL:
            failures += 1
        elif status == WARN:
            warnings += 1
        _line(status, label, detail)

    def section(title: str) -> None:
        print(f' {title}')

    print('=' * 72)
    print(' duburi bringup_check -- RoboSub pre-flight'
          + ('  [STRICT]' if strict else ''))
    print('=' * 72)

    # ---- A. compute / environment ---------------------------------- #
    section('A. Compute / environment')
    for st, lbl, det in _check_ros_env():
        emit(st, lbl, det)
    st, det = _check_disk()
    emit(st, 'disk free (/)', det)

    # ---- B. vision dependencies ------------------------------------ #
    section('B. Vision dependencies (JetPack pitfalls)')
    for st, lbl, det in _check_vision_deps():
        emit(st, lbl, det)

    # ---- C. serial drivers ----------------------------------------- #
    section('C. Serial drivers (payload CH340)')
    for st, lbl, det in _check_serial_drivers():
        emit(st, lbl, det)

    # ---- D. network ------------------------------------------------- #
    section('D. Network reachability')
    if _ping(NETWORK['blueos_ip']):
        emit(PASS, 'BlueOS', NETWORK['blueos_ip'])
    else:
        emit(WARN, 'BlueOS unreachable',
             f"{NETWORK['blueos_ip']}  (expected in pool/desk mode)")
    if _ping(NETWORK['jetson_ip']):
        emit(PASS, 'Jetson', NETWORK['jetson_ip'])
    else:
        emit(WARN, 'Jetson unreachable',
             f"{NETWORK['jetson_ip']}  (skip if you ARE the Jetson)")

    # ---- E. MAVLink / autopilot ------------------------------------ #
    section('E. MAVLink / autopilot')
    if skip_mav:
        emit(WARN, 'MAVLink probe skipped', '--skip-mavlink')
    else:
        for st, lbl, det in _check_mavlink():
            emit(st, lbl, det)

    # ---- F. Pixhawk USB -------------------------------------------- #
    section('F. Pixhawk USB (desk mode)')
    pix = _pixhawk_devices()
    if pix:
        emit(PASS, 'Pixhawk USB CDC', pix[0])
    else:
        emit(PASS, 'no Pixhawk USB device', 'ok for UDP/BlueOS pool mode')

    # ---- G. BNO085 -------------------------------------------------- #
    section('G. Yaw source (BNO085)')
    st, det = _check_bno085_auto()
    emit(st, 'BNO085 auto-detect', det)
    if st == PASS:
        _line('NOTE', 'EKF ext-nav yaw needs', 'VISO_TYPE=1, EK3_SRC1_YAW=6 '
              '(2 MB fmuv3); manager re-checks at startup')

    # ---- H. DVL ----------------------------------------------------- #
    section('H. DVL (Nortek Nucleus 1000)')
    st, det = _check_dvl(NETWORK['dvl_ip'], NETWORK['dvl_port'])
    emit(st, 'Nucleus 1000', det)

    # ---- I. payload ------------------------------------------------- #
    section('I. Payload board (CH340)')
    st, det = _check_payload()
    emit(st, 'payload serial link', det)

    # ---- J. cameras ------------------------------------------------- #
    section('J. Cameras (forward + downward)')
    st, det = _check_cameras()
    emit(st, 'USB cameras', det)

    # ---- K. vision models ------------------------------------------ #
    section('K. Vision models (TensorRT)')
    st, det = _check_models()
    emit(st, 'model engines', det)

    # ---- L. Jetson power ------------------------------------------- #
    section('L. Jetson power mode (vision FPS)')
    st, det = _check_jetson_power()
    emit(st, 'GPU power mode', det)

    # ---- resolved mode + launch hint ------------------------------- #
    section('Manager startup hint')
    chosen = resolve_mode('auto', logger=None)
    emit(PASS, f'auto-detected mode={chosen!r}',
         'picked by mode=auto (the default)')

    # ---- summary --------------------------------------------------- #
    print()
    print('-' * 72)
    if failures:
        print(f'  {failures} FAIL, {warnings} WARN '
              '-- fix FAILs before running anything.')
        rc = 1
    elif warnings:
        print(f'  0 FAIL, {warnings} WARN -- review above before pool day.')
        rc = 1 if strict else 0
        print('  Launch with:')
        _print_launch_hint()
    else:
        print('  All checks PASS. Launch with:')
        _print_launch_hint()
        rc = 0
    print('=' * 72)
    return rc


def _print_launch_hint() -> None:
    print('    ros2 run duburi_manager start              '
          '# mode=auto picks the right profile')
    print('    ros2 launch duburi_vision vision.launch.py camera:=forward  '
          '# vision pipeline (optional)')
    print('    ros2 run duburi_planner mission move_and_see   '
          '# demo mission')
    print()
    print('  DVL distance commands (pool, yaw_source=dvl or bno085_dvl):')
    print('    ros2 run duburi_planner duburi move_forward_dist --distance_m 2.0')


if __name__ == '__main__':
    sys.exit(main())

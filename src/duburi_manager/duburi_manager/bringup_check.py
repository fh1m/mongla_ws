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

``--srot`` switches sections D/E/F/I to the **SROT control board** (direct USB
serial, firmware Hengla): it probes the serial port, the vehicle HEARTBEAT,
armed state, depth telemetry and GAIN, and skips BlueOS / UDP 14550 / Pixhawk
USB / the payload CH340 entirely -- on a SROT vehicle none of those exist, so
they would pass or fail for the wrong reasons, and the payload scan would grab
the board's own port (same CH340 VID/PID).

Usage:
    ros2 run duburi_manager bringup_check
    ros2 run duburi_manager bringup_check --strict
    ros2 run duburi_manager bringup_check --srot        # SROT-based vehicle
"""

from __future__ import annotations

import os
import shutil
import socket
import subprocess
import sys
import time
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
                        '-- rebuild: ./build_dubomini.sh'))
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
# B. vision dependencies (the JetPack pitfalls, jetson-and-env-traps.md E1-E3)
# --------------------------------------------------------------------------- #
def _check_vision_deps() -> list[tuple[str, str, str]]:
    out: list[tuple[str, str, str]] = []

    # numpy<2 -- numpy 2 breaks the cv_bridge / cv2 ABI => every vision node
    # dies with `_ARRAY_API not found` (jetson-and-env-traps.md E1).
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
    # it headless and kills vision_display (jetson-and-env-traps.md E2).
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
                        'pip uninstall them (jetson-and-env-traps.md E2)'))
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
# C. serial drivers (payload CH340 + brltty trap, jetson-and-env-traps.md E5)
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
                    'tools/install_ch341_driver.sh (jetson-and-env-traps.md E5)'))

    # brltty greedily grabs the CH340 (1a86:7523) as a braille display.
    try:
        installed = subprocess.run(['dpkg', '-l', 'brltty'],
                                   capture_output=True, text=True).stdout
        if '\nii  brltty' in installed or installed.startswith('ii  brltty'):
            out.append((WARN, 'brltty installed',
                        'steals the payload CH340 via usbfs -- '
                        'sudo apt-get purge -y brltty (jetson-and-env-traps.md E5)'))
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
        except Exception as exc:                    # noqa: BLE001
            # B21: this used to `pass`, appending NO ROW AT ALL. In a checklist a
            # check that can vanish is worse than one that fails -- a FAIL is
            # read, an absent line is not, and the exit code was unchanged either
            # way, so the gate passed with a check silently missing. The two
            # neighbours here already emit on both branches (the armed check
            # PASS/WARNs; the battery read treats 0/65535 as unknown).
            out.append((WARN, 'flight mode',
                        f'unreadable: {type(exc).__name__}'))

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
    # Mode 0 is the HIGHEST-performance nvpmodel mode on Orin. On a "Super" board
    # that is MAXN (~25W); on an older/backup board it's the top 15W mode -- both
    # are the correct max for THAT board, so mode 0 = PASS either way (do NOT nag
    # for "MAXN" by name -- a non-Super board has no MAXN and its 15W max is fine).
    # jetson_clocks pins clocks to that mode's ceiling and does NOT survive a
    # reboot, so PASS still reminds to (re-)run it -- an un-pinned board silently
    # runs the detector at half FPS.
    if 'nv power mode: 0' in low or ': 0' in low or 'maxn' in low:
        return PASS, (f'max-perf mode ({text[:48]}) -- also run '
                      '`sudo jetson_clocks` (does NOT survive reboot)')
    return WARN, (f'NOT max-perf mode ({text[:48]}) -- run: sudo nvpmodel -m 0 '
                  '&& sudo jetson_clocks  (mode 0 = this board\'s max; on a '
                  'non-Super board that is its top 15W mode -- expected, not MAXN)')


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
# A still bench barometer is stable to well under 1 mbar; this is deliberately loose
# so ordinary weather/HVAC drift over a few seconds cannot trip it.
_BARO_JITTER_MBAR = 15.0
_BARO_SANE_LO, _BARO_SANE_HI = 800.0, 1100.0
# |DEPTH_OUT| at/above this while disarmed = the loop is already demanding full heave.
_DEPTH_OUT_LIMIT = 0.90


def _behaviour_rev_verdict(rev: int | None, required: int) -> tuple[str, str, str]:
    """Grade the board's SROT_FW_BEHAVIOUR_REV. Pure, so it is testable without a board.

    Deliberately ASYMMETRIC, and the asymmetry is the design:

      * a board that ANSWERS with a too-old rev is making a definite statement --
        `stop` will coast -- so this is a FAIL and the preflight should stop.
      * a board that answers NOTHING is far more likely a dropped frame or a
        firmware without REQUEST_MESSAGE than a genuine old board, and failing a
        whole preflight on a comms hiccup is its own hazard. WARN loudly instead.

    `0` is not "unknown": it is what firmware older than 2026-08-01 reports,
    because that build never populated the field. It fails closed like any other
    too-old revision.
    """
    if rev is None:
        return (WARN, 'FW behaviour rev unknown',
                f'board did not answer AUTOPILOT_VERSION; host needs >= {required}. '
                f'If this firmware predates 2026-08-01, MOVE_STOP COASTS and there is '
                f'no host brake -- stop/abort will NOT decelerate')
    if rev < required:
        return (FAIL, 'FW behaviour rev too old',
                f'board reports {rev}, host requires >= {required}. MOVE_STOP coasts '
                f'and the host brake was removed -- stop/abort would not decelerate '
                f'the hull. Flash rev >= {required} (erase+upload; export params first)')
    return (PASS, 'FW behaviour rev', f'{rev} (>= {required} required)')


def _read_param(conn, name: str, tries: int = 3, timeout_s: float = 1.2):
    """One PARAM_REQUEST_READ by name, with retries. None if it never answers.

    SEQUENTIAL ONLY -- never pipeline two of these. pymavlink keeps a single slot per
    msgid, so a second request in flight overwrites the first and the first then times
    out looking exactly like a dropped frame. Retries are for the bridge, which loses
    ~8-9% of frames; on direct USB one pass is enough.

    The name match strips trailing NULs: a 16-char param_id arrives unterminated.
    """
    import duburi_control.fc.srot_protocol as sp      # noqa: F811
    for _ in range(tries):
        conn.mav.param_request_read_send(
            sp.VEHICLE_SYSID, sp.VEHICLE_COMPID, name.encode(), -1)
        end = time.time() + timeout_s
        while time.time() < end:
            msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.4)
            if msg is None:
                continue
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode(errors='ignore')
            if pid.strip('\x00').strip() == name:
                return float(msg.param_value)
    return None


def _gcs_failsafe_verdict(enable: float | None,
                          compid: float | None) -> tuple[str, str, str]:
    """Is the GCS-loss failsafe scoped to US, or wildcarded into bench mode?

    MEASURED 2026-08-07: the vehicle read FS_GCS_ENABLE=1, FS_GCS_COMPID=0 and this
    preflight returned 0 FAIL. That is the reason this function exists.

    `0` is the wildcard -- the board then counts ANY heartbeat that is not its own, so
    Bondor on the bench keeps the failsafe satisfied. In the water that means a dead
    Jetson is indistinguishable from a live GCS: the failsafe never fires, and a hull
    with nobody driving it station-keeps instead of surfacing. `191`
    (MAV_COMP_ID_ONBOARD_COMPUTER) is our compid and scopes it to the companion.

    It regresses silently and repeatedly. Bondor's bench mode wildcards it at runtime,
    which is fine -- until someone presses Save, and then a bench escape hatch is in the
    flight vehicle's NVS with nothing in any log. It was corrected to 191 on 2026-08-06
    and read back 0 the next day.

    srot-integration.md has said "check this value before every water session" since
    then. A prose instruction is not a gate; this is. Same class of blind spot as
    FRAME_REVERSE: state that lives in NVS, invisible to the tool meant to catch it.
    """
    import duburi_control.fc.srot_protocol as sp      # noqa: F811
    if enable is None or compid is None:
        return (WARN, 'GCS failsafe scope unknown',
                'could not read FS_GCS_ENABLE/FS_GCS_COMPID -- check bench mode by hand')
    if enable < 0.5:
        return (WARN, 'GCS failsafe DISABLED',
                'FS_GCS_ENABLE=0 -- losing the companion will not surface the vehicle')
    ours = sp.SOURCE_COMPID
    if int(compid) == ours:
        return (PASS, 'GCS failsafe scope', f'FS_GCS_COMPID={ours} (scoped to us)')
    if int(compid) == 0:
        return (FAIL, 'GCS failsafe WILDCARDED (bench mode)',
                f'FS_GCS_COMPID=0 saved to flash -- any station satisfies the failsafe, '
                f'so a dead Jetson will NOT surface the hull. Set it to {ours} and save '
                f'(on fw rev >= 7 wait for the "Params saved to flash" statustext, not '
                f'the ACK -- the NVS write is deferred)')
    return (WARN, 'GCS failsafe scoped elsewhere',
            f'FS_GCS_COMPID={int(compid)}, we are {ours} -- our heartbeat does not feed it')


def _baro_health_verdict(health: int | None, present: int | None) -> tuple[str, str, str]:
    """Grade the Bar30 from SYS_STATUS's health bitfield. Pure, testable without a board.

    WHY THIS IS A PREFLIGHT LINE AND NOT A CURIOSITY. Since fw behaviour rev 3 the board
    validates the Bar30's calibration PROM (CRC-4) and refuses DEPTH_HOLD / AUTO / PATTERN
    when the baro is unhealthy or its sample is stale. `SROT_MOVE` auto-enters AUTO. So an
    unhealthy Bar30 means EVERY move verb is refused -- `move_forward` included -- and on
    the deck that presents as "arm succeeded, the vehicle just will not move", with the
    reason arriving only as a STATUSTEXT nobody was watching.

    Read it here, where the answer is cheap, rather than at the water's edge.

    Unlike depth/WTEMP -- which the board SUPPRESSES when unhealthy, and which ride
    messages pymavlink truncates or multiplexes -- these two bits are in SYS_STATUS's
    BASE fields, so they are the one part of rev 3's health reporting we can actually read
    (contrast srot_protocol.SYS_STATUS_HAS_EXTENDED_HEALTH, which is where LEAK went).
    """
    from pymavlink import mavutil
    bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
    if health is None or present is None:
        return (WARN, 'baro health unknown',
                'no SYS_STATUS; if the Bar30 is unhealthy the board refuses AUTO and '
                'EVERY move verb is denied, move_forward included')
    if not present & bit:
        return (WARN, 'baro not present', 'board reports no absolute-pressure sensor -- '
                                          'DEPTH_HOLD/AUTO refused, so no move verb runs')
    if not health & bit:
        return (FAIL, 'Bar30 unhealthy',
                'PROM CRC failed or the sample is stale. The board refuses DEPTH_HOLD/AUTO/'
                'PATTERN, and SROT_MOVE enters AUTO -- so every move verb is DENIED. '
                'Check the Bar30 wiring/I2C and power-cycle; the PROM is read at boot')
    return (PASS, 'Bar30 health', 'healthy (AUTO/DEPTH_HOLD available)')


def _baro_noise_verdict(press: list[float]) -> tuple[str, str, str]:
    """Grade the barometer on VARIANCE, not just plausibility. Pure and testable.

    The firmware validates every Bar30 sample against a deliberately wide band
    (~[300, 40000] mbar) so a judgement call cannot ground the vehicle by accident.
    That catches a dead or wildly corrupt sensor. It is structurally blind to the
    failure this hull actually has:

        MEASURED 2026-08-02, bench, still: 30 samples spanning 321..740 mbar, with
        water temperature swinging 6..30 C over the same window.

    Every one of those readings is inside the band, so `SCALED_PRESSURE2` keeps
    streaming and `SYS_STATUS` reports the barometer HEALTHY -- while the depth
    derived from it wanders metres and the depth controller saturates against it.
    A per-sample band cannot see that; peak-to-peak over a window can.

    This is why the check lives here and not only in the firmware: it needs several
    samples, and a pre-arm check on the board sees one.
    """
    if not press:
        return (WARN, 'no SCALED_PRESSURE2',
                'suppressed (fw rev 3+ withholds it when the baro is unhealthy/stale) '
                'or the Bar30 is not fitted -- depth is NOT trustworthy either way')
    spread = max(press) - min(press)
    mean = sum(press) / len(press)
    if spread > _BARO_JITTER_MBAR:
        return (FAIL, 'barometer NOISE',
                f'{len(press)} samples span {spread:.1f} mbar ({min(press):.0f}..'
                f'{max(press):.0f}); a still bench baro is stable to <1 mbar. Each '
                f'sample is inside the firmware plausibility band, so the board still '
                f'reports it HEALTHY -- reseat the Bar30 connector / check I2C. Depth '
                f'and the depth loop are fiction until this is fixed')
    if not (_BARO_SANE_LO <= mean <= _BARO_SANE_HI):
        return (FAIL, 'barometer out of range',
                f'{mean:.0f} mbar, expected {_BARO_SANE_LO:.0f}..{_BARO_SANE_HI:.0f} '
                f'(sea level ~1013). Depth derived from this is wrong by metres')
    return (PASS, 'barometer', f'{mean:.1f} mbar, spread {spread:.2f} mbar')


def _depth_loop_verdict(depth_cmd: float | None,
                        depth_p: float | None = None) -> tuple[str, str, str]:
    """Grade the DISARMED depth chain from DEPTH_CMD. Pure and testable.

    ⚠ THIS USED TO READ `DEPTH_OUT` AND THAT STOPPED WORKING AT fw REV 8, silently.
    Rev 8 suppresses DEPTH_OUT while the controller is not running -- the honest-absence
    fix we asked for -- and this probe only ever runs disarmed, when it never runs. So
    the old code hit its `None` -> WARN branch every time on a perfectly healthy board,
    and the one line standing between a phantom barometer and full vertical thrust
    became advisory noise.

    `DEPTH_CMD` is `depth::preview(depth, 0.10)`: computed on demand, so live while
    disarmed, same +/-1.0 clamp, and proportional-only -- it reflects the CURRENT baro
    sample rather than accumulated windup, which is exactly the question here.

    The 2026-08-02 phantom depth (-3..-6.7 m at the surface) pins it at -1.00. A healthy
    surface reading (~0.03 m) gives ~-0.22. The threshold is carried in METRES and
    converted through DEPTH_P, because comparing a clamped output against a fixed number
    silently means a different physical depth once anyone retunes the gain.
    """
    import duburi_control.fc.srot_protocol as sp      # noqa: F811
    if depth_cmd is None:
        return (WARN, 'depth preview absent',
                'no DEPTH_CMD -- the board has declared the barometer unhealthy/stale, '
                'so it will refuse DEPTH_HOLD/AUTO and every move verb with it')
    gain = depth_p or sp.DEPTH_P_DEFAULT
    # Saturation first -- while clamped the true depth is beyond the clamp, so the
    # recovery below would report a benign value for exactly the case this exists to
    # catch (the 2026-08-02 phantom baro pinned DEPTH_CMD at -1.00).
    if abs(depth_cmd) >= sp.DEPTH_CMD_SATURATED:
        return (FAIL, 'barometer IMPLAUSIBLE',
                f'DEPTH_CMD={depth_cmd:+.2f} is SATURATED -- the board is seeing a '
                'depth it cannot express. ' + 'Arming would command FULL vertical thrust (mixer throttle column is -1 on all four verticals) with the horizontals idle. DO NOT ARM. If the barometer VARIANCE line above is PASS this is a large zero offset, not a dead sensor -- run `ros2 run duburi_planner duburi calibrate_depth` and re-check')
    # Unsaturated: recover the board's own depth. The preview's fixed 0.10 m target is
    # baked into DEPTH_CMD, so subtracting it turns "error against a target" into "how
    # far is the barometer from zero" -- which is the question, and which stops a
    # healthy in-air board from spending a third of the budget on a constant.
    depth_m = depth_cmd / gain + sp.DEPTH_PREVIEW_TARGET_M
    if abs(depth_m) >= sp.DEPTH_ERR_ARM_LIMIT_M:
        return (FAIL, 'barometer IMPLAUSIBLE',
                f'board reads {depth_m:+.2f} m of depth at the surface (limit '
                f'{sp.DEPTH_ERR_ARM_LIMIT_M:.2f} m, DEPTH_CMD={depth_cmd:+.2f}, '
                f'DEPTH_P={gain:g}). Arming would command FULL vertical thrust '
                '(mixer throttle column is -1 on all four verticals) with the '
                'horizontals idle. DO NOT ARM')
    if abs(depth_m) > sp.DEPTH_OFFSET_WARN_M:
        return (WARN, 'barometer offset at the surface',
                f'{depth_m:+.2f} m while disarmed in air -- run '
                '`ros2 run duburi_planner duburi calibrate_depth` before diving')
    return (PASS, 'barometer at surface',
            f'{depth_m:+.2f} m (DEPTH_CMD={depth_cmd:+.2f})')


def _yaw_ref_verdict(raw: float | None) -> tuple[str, str, str]:
    """Is ATTITUDE.yaw a magnetic heading, or relative to wherever the BNO booted?

    Only LOCKED means absolute. Anything else and an absolute `turn` aims at a number
    that means nothing -- and does it silently: the move completes normally, on the
    wrong bearing. MAGACC is NOT a proxy (see SrotFC.check_yaw_reference).
    """
    import duburi_control.fc.srot_protocol as sp      # noqa: F811
    if raw is None:
        return (WARN, 'yaw reference unknown',
                'no YAW_REF -- firmware older than rev 9. Prefer relative turns')
    state = int(raw)
    if state == sp.YAW_REF_LOCKED:
        return (PASS, 'yaw reference', 'LOCKED -- heading is absolute, `turn` is safe')
    return (WARN, 'yaw reference NOT locked',
            f'{state} = {sp.YAW_REF_NAMES.get(state, "unrecognised")}. ATTITUDE.yaw is '
            'boot-relative, so an absolute `turn` will aim at a meaningless heading -- '
            'prefer relative turns')


def _resolve_srot_for_check(device: str = '') -> tuple[str, str]:
    """(conn, probe_state) for the SROT board. `probe_state` is '', 'mavlink',
    'busy' or 'silent' -- '' when the operator named the endpoint explicitly.

    Split out so the section header, the checks and the closing startup hint all
    describe the SAME transport from ONE resolution. Calling the resolver in each
    place re-probes UDP (2 s each) and lets them disagree, which is how this tool
    ended up printing "no MAVLink on UDP" three lines above a healthy heartbeat
    read over exactly that link.
    """
    if device:
        return device, ''
    try:
        from .connection_config import (find_srot_serial, probe_udp_mavlink,
                                        SROT_UDP_CONN, NETWORK)
    except Exception:                              # noqa: BLE001
        return device or '/dev/ttyUSB0', ''
    path = find_srot_serial()
    if path is not None:
        return path, ''
    return SROT_UDP_CONN, probe_udp_mavlink(NETWORK['mav_port'], 2.0)


def _check_srot(skip_mav: bool, device: str = '') -> list[tuple[str, str, str]]:
    """SROT board: port/endpoint, vehicle heartbeat, GAIN, depth sign.

    Replaces the BlueOS/UDP-14550/Pixhawk-USB probes, which on a direct-USB SROT
    vehicle pass or fail for entirely the wrong reasons.

    `device` accepts any pymavlink connection string for the transitional setup where
    the board hangs off a Pi and reaches us as UDP through a BlueOS/Bridget bridge.
    Without it this section can only ever FAIL on such a rig -- `find_srot_serial()`
    looks for a local CH340 that is, correctly, plugged into the Pi instead. That is
    the tool that decides whether to go in the water reporting "no board" about a
    board that is streaming fine.
    """
    out: list[tuple[str, str, str]] = []
    try:
        from .connection_config import find_srot_serial, SROT_BAUD, resolve_srot_profile
    except Exception as exc:                       # noqa: BLE001
        return [(FAIL, 'connection_config import', str(exc))]

    # `device` is normally supplied by _resolve_srot_for_check so the whole section
    # agrees on one transport; the fallback keeps this callable on its own.
    port = device or resolve_srot_profile()['conn']
    is_serial = port.startswith('/dev/')
    baud_kw = {'baud': SROT_BAUD} if is_serial else {}
    if is_serial:
        out.append((PASS, 'SROT serial port', f'{port} @ {SROT_BAUD}'))
    else:
        out.append((PASS, 'SROT endpoint (bridged)', port))

    if skip_mav:
        out.append((WARN, 'SROT MAVLink probe skipped', '--skip-mavlink'))
        return out

    try:
        from pymavlink import mavutil
        # NOT `from duburi_control.fc import srot_protocol`: that runs
        # duburi_control/__init__.py, which imports Duburi -> duburi_interfaces, so it
        # needs a fully-built, fully-sourced workspace. This whole tool exists to
        # diagnose a workspace that ISN'T, and section A already reports a missing
        # duburi_interfaces properly -- having the board section die of the same cause
        # would add a second, misleading FAIL and hide the actual board readings.
        from .srot_connect import _load_srot_protocol
        sp = _load_srot_protocol()
        if sp is None:
            out.append((FAIL, 'srot_protocol unavailable',
                        'cannot decode modes/ACKs -- rebuild: ./build_dubomini.sh'))
            return out
    except Exception as exc:                       # noqa: BLE001
        out.append((FAIL, 'pymavlink/srot_protocol import', str(exc)))
        return out

    # A preflight that reboots the flight controller is not a preflight. Opening
    # this port asserts DTR and resets the ESP32 (measured -- fc/port_guard.py),
    # so running bringup_check against a live manager silently disarms the
    # vehicle and wipes its configured stream rates. Report it as a WARN section
    # rather than doing it.
    from duburi_control.fc.port_guard import PortGuard, PortBusy
    _guard = PortGuard(port)
    try:
        _guard.acquire()
    except PortBusy as exc:
        out.append((WARN, 'srot serial port',
                    f'{port} is already in use -- skipped. Opening it would REBOOT '
                    f'the board. {str(exc).splitlines()[0]}'))
        return out

    conn = None
    try:
        conn = mavutil.mavlink_connection(port, **baud_kw)
        deadline = time.time() + 6.0
        hb = None
        while time.time() < deadline:
            msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2.0)
            if msg is None:
                break
            # Ignore our own / any GCS heartbeat -- only the vehicle counts.
            if getattr(msg, 'autopilot', 0) != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                hb = msg
                break
        if hb is None:
            out.append((FAIL, 'no vehicle HEARTBEAT', f'{port}: board powered? correct port?'))
            return out
        mode = sp.mode_name(getattr(hb, 'custom_mode', -1))
        armed = bool(getattr(hb, 'base_mode', 0)
                     & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        out.append((PASS, 'SROT heartbeat', f'mode={mode}'))
        # Pre-mission gate: nothing should be armed before the operator says so.
        out.append((FAIL if armed else PASS, 'armed state',
                    'ARMED -- disarm before bench/pool work' if armed else 'disarmed'))

        # ---- firmware behaviour revision -- the hull-safety gate ------------ #
        # THE most consequential line in this section, and the reason it is here
        # rather than only inside arm(): below rev 2 the board's MOVE_STOP applies
        # zero braking thrust, and this host no longer carries the reverse-leg
        # brake that used to cover it. `stop` and every abort would simply not
        # decelerate the hull, silently. Finding that out on the bench is the
        # whole point -- SrotFC.arm() also refuses, but that is the pool deck.
        conn.mav.command_long_send(
            sp.VEHICLE_SYSID, sp.VEHICLE_COMPID,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
            float(sp.MSG_ID_AUTOPILOT_VERSION), 0, 0, 0, 0, 0, 0)
        av = conn.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=3.0)
        rev = None if av is None else int(getattr(av, 'middleware_sw_version', 0))
        out.append(_behaviour_rev_verdict(rev, sp.FW_BEHAVIOUR_REV_REQUIRED))

        # Collect a couple of seconds of telemetry for the value checks.
        end = time.time() + 2.5
        while time.time() < end:
            conn.recv_match(blocking=True, timeout=0.5)

        vhud = conn.messages.get('VFR_HUD')
        if vhud is None:
            out.append((WARN, 'no VFR_HUD', 'depth unavailable (Bar30 fitted?)'))
        else:
            depth = float(vhud.alt)
            # NEGATIVE below the surface is the stack-wide convention. A positive
            # reading out of water is normal (~0); a positive one submerged means the
            # sign regressed and every depth guard is silently disabled.
            out.append((PASS, 'depth telemetry', f'{depth:+.2f} m (negative = submerged)'))

        # VFR_HUD.alt keeps streaming even when the board has declared the baro dead, so
        # the line above cannot tell you the reading is trustworthy. This one can.
        sysst = conn.messages.get('SYS_STATUS')
        out.append(_baro_health_verdict(
            None if sysst is None else int(getattr(sysst, 'onboard_control_sensors_health', 0)),
            None if sysst is None else int(getattr(sysst, 'onboard_control_sensors_present', 0))))

        # Barometer + depth loop. Both need SEVERAL samples (the failure mode is
        # variance, not a bad single reading), so gather a window before judging.
        press, named = [], {}
        end = time.time() + 6.0
        while time.time() < end:
            msg = conn.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            mt = msg.get_type()
            if mt == 'SCALED_PRESSURE2':
                press.append(float(msg.press_abs))
            elif mt == 'NAMED_VALUE_FLOAT':
                # 20+ scalars share this msgid and burst together, so the cache holds
                # only whichever landed last -- read them as they arrive.
                nm = msg.name
                nm = nm.decode() if isinstance(nm, bytes) else str(nm)
                nm = nm.strip('\x00').strip()
                named[nm] = float(msg.value)
        out.append(_baro_noise_verdict(press))
        out.append(_depth_loop_verdict(named.get('DEPTH_CMD'),
                                       _read_param(conn, 'DEPTH_P')))
        out.append(_yaw_ref_verdict(named.get('YAW_REF')))

        # GAIN halves MANUAL_CONTROL until it is 1.0, and fw R14 means a PARAM_SET may
        # never have persisted on a board flashed before 8cb4203.
        gain = named.get('GAIN')
        if gain is None:
            out.append((WARN, 'GAIN not seen', 'could not confirm MANUAL_CONTROL authority'))
        elif gain < 0.99:
            out.append((WARN, 'GAIN below 1.0',
                        f'{gain:.2f} -- MANUAL_CONTROL is scaled by this; '
                        f'params may not have persisted (fw R14)'))
        else:
            out.append((PASS, 'GAIN', f'{gain:.2f}'))

        # Bench mode, saved to flash. See _gcs_failsafe_verdict for why this is graded
        # rather than left to the "check it before every session" line in the docs.
        out.append(_gcs_failsafe_verdict(_read_param(conn, 'FS_GCS_ENABLE'),
                                         _read_param(conn, 'FS_GCS_COMPID')))
    except Exception as exc:                       # noqa: BLE001
        out.append((FAIL, 'SROT MAVLink probe raised', str(exc)))
    finally:
        if conn is not None:
            try:
                conn.close()
            except Exception:                      # noqa: BLE001
                pass
        # Release AFTER the close, not before: between them the device is still
        # open, and handing the claim over early is exactly the window where a
        # waiting process opens it and reboots the board.
        _guard.release()
    return out


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    # `--help` used to fall through and run the FULL hardware probe -- an operator
    # asking what the flags are instead got a 12-section scan of the vehicle.
    if '-h' in argv or '--help' in argv:
        print('usage: bringup_check [--srot] [--srot-device=<conn>] [--strict]\n'
              '                     [--skip-mavlink]\n'
              '\n'
              '  --srot           SROT control board over direct USB serial (this\n'
              '                   branch\'s default vehicle). Replaces the network /\n'
              '                   UDP-14550 / Pixhawk-USB probes, which on a SROT\n'
              '                   vehicle pass or fail for entirely the wrong reasons.\n'
              '  --srot-device=   board is not on THIS host: any pymavlink conn\n'
              '                   string, e.g. udpin:0.0.0.0:14550 when the SROT is\n'
              '                   on the Pi behind a BlueOS/Bridget serial->UDP\n'
              '                   bridge. Without it this section FAILs "no board"\n'
              '                   on a rig whose board is streaming fine.\n'
              '  --strict         any WARN exits non-zero (hard pre-mission gate)\n'
              '  --skip-mavlink   skip the autopilot probe (no board/link attached)\n'
              '\n'
              'Exit 0 = nothing FAILed. On --srot the line that gates the water is\n'
              '"FW behaviour rev": below 2 the board coasts on stop and arm() refuses.')
        return 0
    strict = '--strict' in argv
    skip_mav = '--skip-mavlink' in argv
    # The SROT vehicle has no Pi, no BlueOS, no UDP and no Pixhawk: sections D/E/F
    # would report on infrastructure that is not supposed to exist.
    srot = '--srot' in argv
    # Transitional rig: board on the Pi, reaching us as UDP via a BlueOS bridge.
    srot_device = next((a.split('=', 1)[1] for a in argv
                        if a.startswith('--srot-device=')), '')

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

    if srot:
        # ---- D-F (SROT): the board, however it is attached ---------------- #
        # Resolve ONCE here so the section header, the checks and the closing
        # startup hint all describe the SAME transport. Resolving in more than one
        # place re-probes UDP (2 s each) and lets them disagree.
        srot_conn, srot_state = _resolve_srot_for_check(srot_device)
        section('D-F. SROT control board'
                + (' (bridged over UDP)' if not srot_conn.startswith('/dev/')
                   else ' (direct USB serial)'))
        if srot_state == 'silent':
            emit(WARN, 'SROT auto-detect',
                 'no USB serial and no MAVLink on UDP -- falling back to '
                 f'{srot_conn}. Plug in the Type-C cable, bring up the BlueOS '
                 'bridge, or pass --srot-device=<conn>')
        elif srot_state == 'busy':
            emit(WARN, 'SROT auto-detect',
                 f'UDP port already held by another process -- could not probe. '
                 f'Using {srot_conn}. A leftover `duburi_manager start` is the '
                 f'usual cause: ss -lunp | grep 14550')
        for st, lbl, det in _check_srot(skip_mav, srot_conn):
            emit(st, lbl, det)
    else:
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

    # ---- G. yaw source ---------------------------------------------- #
    # Both G and H probe hardware that is NOT FITTED on a srot vehicle. Left
    # ungated they emit two WARNs the operator can do nothing about -- and one of
    # them ("Plug the ESP32-C3 in") instructs them to reinstall a board that was
    # deliberately removed. Unactionable WARNs are not free: they train an operator
    # to skim the WARN column, which is where the firmware-rev gate lives.
    if srot:
        section('G. Yaw source (BNO085 on the control board)')
        emit(PASS, 'BNO085 rides the MAVLink link',
             'on the board (I2C0), fused at 500 Hz -> ATTITUDE; '
             'use yaw_source:=mavlink_ahrs')
        _line('NOTE', 'USB ESP32-C3 + BNO085', 'REMOVED from the hull -- '
              'yaw_source:=bno085 would open a device that is not there')
    else:
        section('G. Yaw source (BNO085)')
        st, det = _check_bno085_auto()
        emit(st, 'BNO085 auto-detect', det)
        if st == PASS:
            _line('NOTE', 'EKF ext-nav yaw needs', 'VISO_TYPE=1, EK3_SRC1_YAW=6 '
                  '(2 MB fmuv3); manager re-checks at startup')

    # ---- H. DVL ----------------------------------------------------- #
    if srot:
        section('H. DVL (Nortek Nucleus 1000)')
        _line('NOTE', 'DVL not fitted', 'never validated in water and not on the '
              'SROT wire; move_*_dist stay refused (vehicle-spec.md "DVL status")')
    else:
        section('H. DVL (Nortek Nucleus 1000)')
        st, det = _check_dvl(NETWORK['dvl_ip'], NETWORK['dvl_port'])
        emit(st, 'Nucleus 1000', det)

    # ---- I. payload ------------------------------------------------- #
    if srot:
        # The payload is the board's own PCA9685 over MAVLink -- there is no separate
        # USB board to probe, and probing would be actively harmful: the old payload
        # ESP32 is the SAME CH340 VID/PID as the SROT board, so the scan opens the
        # board's own port and fights the MAVLink link.
        section('I. Payload (SROT PCA9685 over MAVLink)')
        emit(PASS, 'payload transport', 'PCA9685 over MAVLink; no separate USB board')
        # The transport being up is NOT the payload working, and the old PASS on this
        # line is what hid the dead fire() path: link up, line green, nothing able
        # to actuate.
        #
        # NOTE, not WARN, and the distinction is load-bearing: whether a channel is
        # fireable is a board ROLE read over a live MAVLink link by the running node,
        # which a standalone preflight has no session for. An unconditional WARN would
        # make `--strict` -- the hard pre-mission gate -- exit non-zero on EVERY srot
        # run for a condition nobody can clear from here. A gate that always fails is
        # a gate people stop running. Say the true thing and point at the answer.
        _line('NOTE', 'payload channels',
              'fire(N) addresses BOARD channel N directly (no map). Which channels '
              'are fireable is read from the board at bring-up -- confirm the '
              'manager logs "[PAYLOAD] board roles: FIREABLE (switch) [...]" and '
              'that your intended channel is in that list')
    else:
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
    if srot:
        # `mode` selects among the UDP-14550 PROFILES, which resolve_srot_profile()
        # bypasses entirely -- there is no BlueOS to listen to. Printing
        # "auto-detected mode='sim'" on a real SROT vehicle is worse than printing
        # nothing: it looks like the preflight thinks this is a simulation.
        # Still true when bridged: `mode` picks among the UDP PROFILES, and
        # resolve_srot_profile() bypasses those whether the transport is a device
        # or an endpoint. But the operator must pass the SAME endpoint to the
        # manager, so print the command rather than the word "serial".
        if srot_conn.startswith('/dev/'):
            emit(PASS, 'connection',
                 f'direct USB serial ({srot_conn}) -- auto-detected; `mode` does '
                 f'not apply on srot')
        else:
            emit(PASS, 'connection',
                 f'bridged over UDP ({srot_conn}) -- the manager auto-detects the '
                 f'same endpoint; `mode` does not apply on srot')
    else:
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
        _print_launch_hint(srot)
    else:
        print('  All checks PASS. Launch with:')
        _print_launch_hint(srot)
        rc = 0
    print('=' * 72)
    return rc


def _print_launch_hint(srot: bool = False) -> None:
    if srot:
        # Deliberately NOT the pixhawk hint: it recommends move_forward_dist, which is
        # permanently refused on srot (no DVL fitted, never validated), and a bare
        # `start` whose defaults are right but whose profile talk is meaningless here.
        print('    ros2 launch duburi_manager bringup.launch.py   '
              '# srot + mavlink_ahrs are the defaults')
        print('      # fire(N) = BOARD channel N -- see the [PAYLOAD] roles line;'
              ' nothing else needs setting')
        print('    ros2 run duburi_planner duburi arm')
        print('    ros2 run duburi_planner duburi move_forward --duration 5 --gain 40')
        print('    ros2 run duburi_planner duburi stop')
        print()
        print('  ⛔ BEFORE WATER -- the depth loop has never run closed, and it runs')
        print('     under EVERY AUTO move (move_forward included), not just set_depth:')
        print('    ros2 run duburi_planner duburi set_mode --target_name DEPTH_HOLD')
        print('      1. hand-raise/lower the sub -> verticals must push BACK toward depth')
        print('      2. trip the leak input at depth -> the demand must be ASCEND')
        return
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

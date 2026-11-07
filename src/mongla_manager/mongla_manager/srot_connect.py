#!/usr/bin/env python3
"""``ros2 run mongla_manager connect`` -- point at the SROT board, see the vehicle.

The SROT board has one USB cable and no web UI, so this is the surface for
looking at the vehicle: open the serial link, and print everything the board says.

Why this is not `bringup_check`. That is a pass/fail GATE -- it grades, it
exits non-zero, and `--strict` makes any WARN fatal. Those are the right semantics
for "may I dive", and the wrong ones for "show me the vehicle": a monitor that exits
non-zero because something is merely unusual cannot be left running while you watch a
number change. `connect` never grades and always exits 0 (barring a link failure).
They share the wire knowledge below; they do not share a verdict.

READING RULE, and it is the whole reason this file is careful: **absence is data.**
Since fw behaviour rev 3 the board SUPPRESSES `WTEMP` / `SCALED_PRESSURE2` rather
than publishing a value it cannot stand behind, and sends `0` in
`SCALED_IMU2.temperature` as MAVLink's "not provided" sentinel. A consumer that
renders a missing value as `0.0` re-creates exactly the failure the firmware fixed --
a Bar30 read during a PROM reset race once published `-51 C` and `+2.87 m` in air
with nothing marking them wrong. Everything here renders absence as `--`.
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time

# MAVLink 2 must be selected BEFORE pymavlink is imported: the default dialect
# binding is v1.0 ardupilotmega, which has no ESC_TELEMETRY_1_TO_4 (11030) at all.
os.environ.setdefault('MAVLINK20', '1')

from pymavlink import mavutil                                        # noqa: E402

from mongla_control.fc.port_guard import PortGuard, PortBusy

try:
    from .connection_config import SROT_BAUD, resolve_srot_profile
except ImportError:                                                  # direct execution
    from mongla_manager.connection_config import SROT_BAUD, resolve_srot_profile


class _StderrLogger:
    """resolve_srot_profile() logs through a ROS-style logger; this tool has none.

    Everything it says goes to STDERR, never stdout, so `--json` stays a clean
    machine-readable document even when auto-detect is chatty.
    """
    def info(self, msg):  print(msg, file=sys.stderr)
    def warn(self, msg):  print(msg, file=sys.stderr)
    def warning(self, msg): print(msg, file=sys.stderr)
    def error(self, msg): print(msg, file=sys.stderr)

def _load_srot_protocol():
    """Import the wire constants WITHOUT dragging in the ROS package chain.

    `mongla_control/__init__.py` imports `Mongla`, which imports `mongla_interfaces`
    -- a generated ROS message package. So the obvious
    `from mongla_control.fc import srot_protocol` only works on a fully-built, fully
    sourced workspace, and this tool is frequently the FIRST thing anyone runs on a
    fresh box or a half-built tree.

    `srot_protocol` is pure constants with no ROS dependency, so when the package
    import fails we load the file directly. Degrading silently to "mode unknown"
    would be the same sin this whole tool exists to prevent: rendering absent
    knowledge as a plausible-looking value.
    """
    try:
        from mongla_control.fc import srot_protocol as mod
        return mod
    except Exception:                                            # noqa: BLE001
        pass
    import importlib.util
    here = os.path.dirname(os.path.abspath(__file__))
    for rel in ('../../mongla_control/mongla_control/fc/srot_protocol.py',
                '../../../mongla_control/mongla_control/fc/srot_protocol.py'):
        cand = os.path.normpath(os.path.join(here, rel))
        if os.path.exists(cand):
            spec = importlib.util.spec_from_file_location('_srot_protocol', cand)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            return mod
    return None


sp = _load_srot_protocol()

try:
    from . import srot_format as sfmt
    from . import srot_changes
except ImportError:                                                  # direct execution
    from mongla_manager import srot_format as sfmt
    from mongla_manager import srot_changes

BOLD, DIM, RESET = sfmt.BOLD, sfmt.DIM, sfmt.RESET
RED, YEL, GRN, CYA = sfmt.RED, sfmt.YEL, sfmt.GRN, sfmt.CYA

# The board answers MAV_CMD_REQUEST_MESSAGE for this; it carries SROT_FW_BEHAVIOUR_REV
# in middleware_sw_version, which is the one number that decides whether `stop` brakes.
MSG_AUTOPILOT_VERSION = 148

# How many change lines the dashboard keeps on screen. The full history stays in the
# list; this only bounds what a fixed-height panel shows.
_CHANGE_LOG_LINES = 12

# Sea-level pressure is ~1013 mbar. The firmware's own plausibility band is a wide
# [300, 40000] mbar, chosen deliberately loose so it cannot ground the vehicle by
# accident -- but that band is applied PER SAMPLE, so it cannot see a sensor whose
# every reading is individually plausible and collectively noise. This is that check.
BARO_JITTER_MBAR = 15.0     # a still bench baro is stable to well under 1 mbar
BARO_SANE_LO, BARO_SANE_HI = 800.0, 1100.0

# Modes in which the depth controller is not closed, so DEPTH_OUT is idle by
# construction rather than by health (fw task_control_loop: the depth branch runs in
# DEPTH_HOLD / AUTO / PATTERN).
_DEPTH_LOOP_IDLE_MODES = frozenset({'MANUAL', 'STABILIZE', 'ACRO', 'MOTOR_DETECT',
                                    'MOTOR_TUNE', 'AUTOTUNE'})

_NAMED_GROUPS = (
    # 'BARO_HEALT' is the TRUNCATED wire name -- NAMED_VALUE_FLOAT.name is 10 chars
    # and the board does not shorten its own string. See sp.NAME_BARO_HEALTH.
    ('depth loop', ('DEPTH_CMD', 'DEPTH_ERR', 'DEPTH_OUT', 'MIX_VERT', 'MIX_VSGN',
                    'BARO_P2P', 'BARO_HEALT')),
    ('move',       ('MV_STATE', 'MV_TYPE', 'MV_PROG', 'STUNT_PRG')),
    ('vehicle',    ('GAIN', 'KILL', 'LEAK', 'WTEMP', 'CURR', 'MAGACC', 'ATUNE',
                    'COMP_SEEN')),
    ('firmware',   ('HEAP', 'STK_MAV', 'STK_SEN', 'STK_CTL', 'STK_UI',
                    'STK_LORA', 'STK_DSH')),
)


def _f(value, fmt='{:.2f}', suffix=''):
    """Render a numeric, or `--` when absent. Delegates to srot_format so this tool,
    the dashboard, --json and the manager's [SROT ] block cannot drift apart."""
    return sfmt.fmt(value, fmt, suffix)


class Snapshot:
    """Everything the board said during one listening window."""

    def __init__(self):
        self.named: dict[str, float] = {}
        self.named_hz: dict[str, int] = {}
        self.batteries: dict[int, tuple] = {}
        self.msg_counts: dict[str, int] = {}
        self.press_samples: list[float] = []
        self.wtemp_samples: list[float] = []
        self.msgs: dict = {}
        self.statustexts: list[str] = []
        self.roles: dict[int, int] = {}      # PCA channel (1-based) -> SERVOn_ROLE
        self.window_s = 0.0

    def get(self, msgtype):
        return self.msgs.get(msgtype)


def collect(conn, seconds: float) -> Snapshot:
    """Listen (read-only) for `seconds` and fold everything into a Snapshot."""
    snap = Snapshot()
    t0 = time.time()
    while time.time() - t0 < seconds:
        msg = conn.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype == 'BAD_DATA':
            snap.msg_counts['BAD_DATA'] = snap.msg_counts.get('BAD_DATA', 0) + 1
            continue
        snap.msg_counts[mtype] = snap.msg_counts.get(mtype, 0) + 1
        snap.msgs[mtype] = msg

        if mtype == 'NAMED_VALUE_FLOAT':
            # The board rides 20+ scalars on this ONE msgid and bursts them together,
            # while pymavlink keeps a single message per msgid. Reading its cache would
            # return whichever name landed last -- see SrotFC._named_value. Fold every
            # message as it arrives instead; this loop is the only place that sees them.
            name = msg.name
            name = name.decode() if isinstance(name, bytes) else str(name)
            name = name.strip('\x00').strip()
            if name:
                snap.named[name] = float(msg.value)
                snap.named_hz[name] = snap.named_hz.get(name, 0) + 1
                if name == 'WTEMP':
                    snap.wtemp_samples.append(float(msg.value))
        elif mtype == 'BATTERY_STATUS':
            # TWO instances: id 0 = PM1 (electronics), id 1 = PM2 (thruster pack, over
            # ESP-NOW). Same single-slot trap as NAMED_VALUE_FLOAT -- key by id.
            bid = int(getattr(msg, 'id', 0))
            volts = list(getattr(msg, 'voltages', []) or [])
            snap.batteries[bid] = (
                sfmt.mv_to_volts(volts[0] if volts else None),
                sfmt.centiamps_to_amps(getattr(msg, 'current_battery', None)))
        elif mtype == 'SCALED_PRESSURE2':
            snap.press_samples.append(float(msg.press_abs))
        elif mtype == 'STATUSTEXT':
            txt = msg.text
            txt = txt.decode() if isinstance(txt, bytes) else str(txt)
            txt = txt.strip('\x00').strip()
            if txt and txt not in snap.statustexts:
                snap.statustexts.append(txt)

    snap.window_s = time.time() - t0
    return snap


def baro_verdict(press: list[float]) -> tuple[str, str]:
    """(level, message) for the barometer, judged on VARIANCE as well as value.

    The firmware validates each sample against a deliberately wide plausibility band.
    That catches a dead sensor and a wildly corrupt one, but it is blind to the
    failure actually seen on this hardware: a loose Bar30 connector, where every
    individual reading falls inside the band and the SEQUENCE is noise. The board
    then reports the barometer healthy, `SCALED_PRESSURE2` keeps streaming, and the
    depth controller closes on it. Peak-to-peak over a window is what sees that.
    """
    if not press:
        return ('WARN', 'no SCALED_PRESSURE2 -- suppressed (baro unhealthy/stale) or absent')
    spread = max(press) - min(press)
    mean = statistics.fmean(press)
    if spread > BARO_JITTER_MBAR:
        return ('FAIL',
                f'NOISE: {len(press)} samples span {spread:.1f} mbar '
                f'({min(press):.1f}..{max(press):.1f}). A still bench baro is stable to '
                f'<1 mbar. Every sample is inside the firmware plausibility band, so the '
                f'board reports it HEALTHY -- check the Bar30 connector/I2C wiring')
    if not (BARO_SANE_LO <= mean <= BARO_SANE_HI):
        return ('FAIL', f'{mean:.1f} mbar -- outside {BARO_SANE_LO:.0f}..{BARO_SANE_HI:.0f} '
                        f'(sea level ~1013); depth derived from this is fiction')
    return ('OK', f'{mean:.1f} mbar, spread {spread:.2f} mbar over {len(press)} samples')


def depth_loop_verdict(named: dict, mode: str | None = None) -> tuple[str, str]:
    """(level, message) for the depth controller, read DISARMED and read-only.

    `DEPTH_OUT` is the real controller's last output. Saturated while disarmed and
    stationary means the loop is holding a demand it cannot satisfy -- and because the
    mixer's throttle column is -1 for all four verticals and 0 for all four
    horizontals, that demand lands as FULL vertical thrust with the horizontals idle
    the instant you arm. This is the cheapest possible read of that condition.
    """
    out = named.get('DEPTH_OUT')
    err = named.get('DEPTH_ERR')
    if out is None:
        return ('WARN', 'DEPTH_OUT absent -- cannot tell what the loop would do on arm')
    # A settled reading in a mode that does not RUN the loop proves nothing. Observed
    # live: in MANUAL the board reports DEPTH_OUT=+0.00 with a barometer that is
    # visibly noise, and the saturation returns the moment SROT_MOVE enters AUTO.
    # Reporting a bare OK there is false reassurance about the exact failure this
    # check exists to catch.
    if mode in _DEPTH_LOOP_IDLE_MODES and abs(out) < 0.25:
        return ('WARN',
                f'DEPTH_OUT={out:+.2f}, but the board is in {mode} -- the depth loop '
                f'is NOT running, so this is not evidence it is healthy. Re-check in '
                f'DEPTH_HOLD (every SROT_MOVE enters AUTO, which closes the loop)')
    if abs(out) >= 0.99:
        return ('FAIL',
                f'SATURATED: DEPTH_OUT={out:+.2f}, DEPTH_ERR={_f(err, "{:+.2f}", " m")}. '
                f'The mixer throttle column is -1 on all four verticals, so on ARM this '
                f'becomes FULL vertical thrust with the horizontals idle. DO NOT ARM')
    if abs(out) > 0.25:
        return ('WARN', f'DEPTH_OUT={out:+.2f} while disarmed -- a standing demand')
    return ('OK', f'DEPTH_OUT={out:+.2f}, DEPTH_ERR={_f(err, "{:+.2f}", " m")}')


def read_roles(conn, timeout: float = 0.6) -> dict:
    """PCA channel (1-based) -> SERVOn_ROLE, read from the board.

    SCOPE: roles only, deliberately. This is the link/bring-up diagnostic and it runs
    BEFORE any reader thread exists, hence its own recv_match loop rather than
    `SrotFC.get_param`. Payload IDENTITY (`SERVOn_FUNCTION`) is reported by
    `SrotPayload.preflight_roles` on the running node, which already reads it in the
    same traversal. Duplicating that here would make a THIRD place that believes
    something about the payload map -- which is the exact failure the board-side
    FUNCTION param exists to end.

    These are PARAMETERS, not telemetry, so they have to be asked for one at a time.
    Worth the ~16 round-trips: the role decides whether a channel is payload (switch)
    or the on-board manipulator arm (PWM), and driving the arm during a drop is the
    failure this whole read exists to prevent.

    RETRIED, because a single dropped reply is indistinguishable from a channel
    that is not a switch. OBSERVED on the bridged link: channel 9 disappeared from
    BOTH the SWITCH and SERVO lists between two runs a minute apart -- one lost
    PARAM_VALUE out of 16 -- which reads to an operator as "9 is not fireable" when
    it is. On a transport measured at ~8-9% frame loss, one-shot reads over 16
    round-trips will drop one most of the time.
    """
    if sp is None:
        return {}
    roles = {}
    for ch in range(1, sp.PCA9685_NUM_CH + 1):
        name = sp.PCA_ROLE_PARAM_FMT.format(ch)
        for _attempt in range(3):
            conn.mav.param_request_read_send(conn.target_system,
                                             conn.target_component,
                                             name.encode(), -1)
            deadline = time.time() + timeout
            while time.time() < deadline:
                pv = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.3)
                if pv is None:
                    continue
                pid = pv.param_id
                pid = pid.decode() if isinstance(pid, bytes) else str(pid)
                if pid.strip('\x00') == name:
                    roles[ch] = int(pv.param_value)
                    break
            if ch in roles:
                break
    return roles


def read_safety_gates(conn, timeout: float = 0.6) -> dict:
    """{param: float|None} for every gate in `sp.SAFETY_GATES`.

    Same one-at-a-time PARAM_VALUE dance as `read_roles` and for the same
    reason: PARAM_VALUE is a single pymavlink slot, so a burst request would
    leave most of them unread. A parameter that never answers stays None --
    reported as UNREADABLE, never as a pass.
    """
    if sp is None:
        return {}
    out = {}
    for name, _exp, _sev, _gates in sp.SAFETY_GATES:
        out[name] = None
        for _attempt in range(3):
            conn.mav.param_request_read_send(conn.target_system,
                                             conn.target_component,
                                             name.encode(), -1)
            deadline = time.time() + timeout
            while time.time() < deadline:
                pv = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.3)
                if pv is None:
                    continue
                pid = pv.param_id
                pid = pid.decode() if isinstance(pid, bytes) else str(pid)
                if pid.strip('\x00') == name:
                    out[name] = float(pv.param_value)
                    break
            if out[name] is not None:
                break
    return out


def _baro_healthy(sysst):
    """The board's own barometer-health bit; None with no SYS_STATUS.

    Depth is read from VFR_HUD, which is not gated on this bit, so the presence
    of a depth value says nothing about the sensor behind it."""
    if sysst is None:
        return None
    bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
    return bool(sysst.onboard_control_sensors_health & bit)


def watch_fields(snap: Snapshot) -> dict:
    """The subset of a Snapshot the change log watches, in display units.

    Deliberately small and FLAT. The change detector compares scalars; giving it the raw
    Snapshot would mean re-deciding units (radians vs degrees, mV vs V) in two places,
    which is how the yaw bug happened in the first place.
    """
    hb = snap.get('HEARTBEAT')
    att = snap.get('ATTITUDE')
    hud = snap.get('VFR_HUD')
    sysst = snap.get('SYS_STATUS')
    n = snap.named

    baro_ok = _baro_healthy(sysst)

    batt = snap.batteries
    main = batt.get(sp.BATTERY_ID_MAIN if sp else 0)
    thr = batt.get(sp.BATTERY_ID_THRUSTER if sp else 1)

    return {
        'armed': (bool(getattr(hb, 'base_mode', 0)
                       & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else None),
        'mode': (sp.mode_name(getattr(hb, 'custom_mode', -1)) if (hb and sp) else None),
        'heading_deg': sfmt.heading_deg(getattr(att, 'yaw', None)) if att else None,
        'roll_deg': sfmt.signed_deg(getattr(att, 'roll', None)) if att else None,
        'pitch_deg': sfmt.signed_deg(getattr(att, 'pitch', None)) if att else None,
        # Depth is only meaningful when the board vouches for the barometer -- VFR_HUD.alt
        # keeps streaming regardless (mav_stream.cpp:258), so an ungated value would look
        # like a rock-steady 0 m rather than a missing sensor.
        'depth_m': (getattr(hud, 'alt', None) if baro_ok else None),
        'baro_healthy': baro_ok,
        # Snapshot.batteries holds (voltage, current) TUPLES -- unlike SrotFC's
        # per-id dicts. Two shapes for the same fact is exactly the drift this file is
        # trying to end, but changing Snapshot's shape here would touch the render path
        # mid-fix; unpack explicitly and leave one TODO rather than two conventions.
        'battery_v': main[0] if main else None,
        'thruster_v': thr[0] if thr else None,
        'water_temp_c': n.get('WTEMP'),
        'depth_out': n.get('DEPTH_OUT'),
        'depth_err_m': n.get('DEPTH_ERR'),
        'gain': n.get('GAIN'),
        'mag_accuracy': n.get('MAGACC'),
        'leak': None if n.get('LEAK') is None else n['LEAK'] >= 0.5,
        'kill': None if n.get('KILL') is None else n['KILL'] >= 0.5,
    }


def _armed(hb) -> bool:
    """True when the heartbeat says ARMED; False when it does not, or is absent."""
    return bool(getattr(hb, 'base_mode', 0)
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else False


def _mode(hb) -> str:
    """The board's mode name, `--` with no heartbeat, `??` with no protocol."""
    if sp is None:
        return '?? (srot_protocol unavailable -- mode NOT decoded)'
    if hb is None:
        return '--'
    return sp.mode_name(getattr(hb, 'custom_mode', -1))


def as_dict(snap: Snapshot) -> dict:
    hb, av, att, hud = (snap.get(k) for k in ('HEARTBEAT', 'AUTOPILOT_VERSION',
                                              'ATTITUDE', 'VFR_HUD'))
    baro_lvl, baro_msg = baro_verdict(snap.press_samples)
    depth_lvl, depth_msg = depth_loop_verdict(
        snap.named, sp.mode_name(getattr(hb, 'custom_mode', -1)) if (hb and sp) else None)
    return {
        'behaviour_rev': int(getattr(av, 'middleware_sw_version', 0)) if av else None,
        'armed': bool(getattr(hb, 'base_mode', 0)
                      & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else None,
        'mode': sp.mode_name(getattr(hb, 'custom_mode', -1)) if (hb and sp) else None,
        'batteries': {str(k): {'voltage': v[0], 'current': v[1]}
                      for k, v in snap.batteries.items()},
        # Same 0..360 heading convention as the display and /mongla/state. `yaw` is
        # kept as an alias so existing consumers do not break, but both are wrapped.
        'attitude_deg': ({'roll': sfmt.signed_deg(att.roll),
                          'pitch': sfmt.signed_deg(att.pitch),
                          'heading': sfmt.heading_deg(att.yaw),
                          'yaw': sfmt.heading_deg(att.yaw)} if att else None),
        'depth_m': getattr(hud, 'alt', None),
        'named': snap.named,
        'msg_rates_hz': {k: round(v / max(snap.window_s, 0.1), 2)
                         for k, v in snap.msg_counts.items()},
        'baro': {'verdict': baro_lvl, 'detail': baro_msg,
                 'samples': snap.press_samples},
        'depth_loop': {'verdict': depth_lvl, 'detail': depth_msg},
        'pca_roles': snap.roles,
    }


def main(argv=None) -> int:
    # Imported here because the report reads this module. Same two forms as
    # srot_format above: in the package, or run as a file.
    try:
        from .connect_report import render
    except ImportError:
        from mongla_manager.connect_report import render
    ap = argparse.ArgumentParser(
        prog='connect',
        description='Connect to the SROT board and show everything it sends.')
    ap.add_argument('--path', default=None,
                    help='serial device, OR any pymavlink connection string when the '
                         'board is not on this host -- e.g. udpin:0.0.0.0:14550 for a '
                         'SROT reached through a serial->UDP bridge. '
                         'Default: autodetect local USB serial, same as the manager.')
    ap.add_argument('--baud', type=int, default=SROT_BAUD)
    ap.add_argument('--duration', type=float, default=6.0,
                    help='listening window in seconds (default 6)')
    ap.add_argument('--watch', action='store_true',
                    help='refresh continuously until Ctrl-C')
    ap.add_argument('--json', action='store_true', help='machine-readable output')
    ap.add_argument('--no-roles', action='store_true',
                    help='skip the PCA9685 role read (16 param round-trips)')
    args = ap.parse_args(argv)

    # Same resolver the manager uses, so `connect`, `bringup_check --srot` and
    # `start` can never disagree about WHERE the board is -- a tool that looks in a
    # different place than the node it is meant to diagnose is worse than no tool.
    if args.path:
        path = args.path
    else:
        prof = resolve_srot_profile(logger=_StderrLogger() if not args.json else None)
        path = prof['conn']

    # `baud` is meaningful only for a real serial device. Passing it alongside a
    # udpin:/tcp: string is harmless to pymavlink (it ignores it off the serial
    # path) but printing "@ 115200" next to a UDP endpoint states a baud rate for
    # a link that has none -- and this tool exists to stop exactly that kind of
    # confident-but-wrong number. Same /dev/ test resolve_srot_profile() uses.
    is_serial = path.startswith('/dev/')
    baud_kw = {'baud': args.baud} if is_serial else {}
    if not args.json:
        where = f'{path} @ {args.baud}' if is_serial else path
        print(f'{DIM}connecting to {where} ...{RESET}')
    # Refuse rather than reboot. This is a hand-run diagnostic, so the realistic
    # way the flight controller gets restarted mid-mission is an operator running
    # THIS while the manager is up: opening the port asserts DTR and resets the
    # ESP32 (measured -- fc/port_guard.py). A tool that silently disarms the
    # vehicle it is inspecting is worse than one that will not start.
    _guard = PortGuard(path)
    try:
        _guard.acquire()
    except PortBusy as exc:
        print(f'{exc}', file=sys.stderr)
        return 2
    try:
        conn = mavutil.mavlink_connection(
            path, **baud_kw,
            source_system=(sp.SOURCE_SYSID if sp else 255),
            source_component=(sp.SOURCE_COMPID if sp else 191))
    except Exception as exc:                                  # noqa: BLE001
        print(f'could not open {path}: {exc}', file=sys.stderr)
        if is_serial:
            print('  if this is EIO the CH340 is wedged -- unplug and replug the cable.',
                  file=sys.stderr)
        return 2

    if conn.wait_heartbeat(timeout=10) is None:
        print(f'no HEARTBEAT on {path}.', file=sys.stderr)
        if is_serial:
            print('  Board powered? Correct port?', file=sys.stderr)
        else:
            # On a bridged link "silent" has one more failure mode than on serial:
            # the board can be healthy with the bridge simply not running -- or,
            # after a reflash, still running on a dead file descriptor.
            print('  Is the serial->UDP bridge running? After a reflash, restart it.',
                  file=sys.stderr)
        return 2

    # Read-only: AUTOPILOT_VERSION is not streamed, it must be asked for. Nothing
    # else in this tool ever writes to the vehicle.
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
        float(MSG_AUTOPILOT_VERSION), 0, 0, 0, 0, 0, 0)

    prev_fields = None
    change_log: list[str] = []
    try:
        while True:
            snap = collect(conn, args.duration)
            if not args.no_roles:
                snap.roles = read_roles(conn)

            fields = watch_fields(snap)
            for lvl, _field, msg in srot_changes.diff(prev_fields, fields):
                col = {'CRIT': RED, 'WARN': YEL}.get(lvl, DIM)
                change_log.append(f'  {DIM}{time.strftime("%H:%M:%S")}{RESET} '
                                  f'{col}{lvl:<4}{RESET} {msg}')
            prev_fields = fields

            if args.json:
                print(json.dumps(as_dict(snap), indent=2, default=str))
            elif args.watch:
                # Home + erase-to-end-of-screen rather than a full clear: a full \033[2J
                # blanks the terminal every tick, so the panel visibly flickers and any
                # text you scrolled back to read is destroyed.
                lines = render(snap, conn)
                if change_log:
                    lines.append(f'\n{BOLD}== changes =={RESET}   '
                                 f'{DIM}(newest last){RESET}')
                    lines.extend(change_log[-_CHANGE_LOG_LINES:])
                lines.append(f'\n{DIM}Ctrl-C to stop{RESET}')
                out = '\n'.join(ln + '\033[K' for ln in lines)
                print(f'\033[H{out}\033[J', end='', flush=True)
            else:
                print('\n'.join(render(snap, conn)))
            if not args.watch:
                return 0
            # AUTOPILOT_VERSION is one-shot; re-ask so a --watch session keeps showing it.
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                float(MSG_AUTOPILOT_VERSION), 0, 0, 0, 0, 0, 0)
    except KeyboardInterrupt:
        print()
        return 0


if __name__ == '__main__':
    sys.exit(main())

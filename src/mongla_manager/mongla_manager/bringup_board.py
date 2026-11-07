"""Bringup checks of the SROT board (`bringup_check --srot`): firmware, barometer, depth loop, failsafe, memory.

The verdict functions are pure -- they take values already read -- so each is tested
without a board. `_check_srot` is the one that talks to it, read-only.
"""
from __future__ import annotations

import os
os.environ['MAVLINK20'] = '1'   # before any pymavlink import: the board speaks MAVLink 2
import time  # noqa: E402

from .bringup_grade import FAIL, PASS, WARN


# Battery thresholds (4S LiPo nominal ~14.8 V, 3S ~11.1 V). We can't know the
# pack chemistry here, so only flag an obviously-flat pack; the operator owns
# the real go/no-go on voltage.
_BATT_LOW_V = 13.5      # below this on a 4S -> WARN (recharge/swap)


_BATT_FLAT_V = 12.0     # below this -> almost certainly a dying/low pack


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
    import mongla_control.fc.srot_protocol as sp      # noqa: F811
    for _ in range(tries):
        conn.mav.param_request_read_send(
            sp.VEHICLE_SYSID, sp.VEHICLE_COMPID, name.encode(), -1)
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
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
    companion is indistinguishable from a live GCS: the failsafe never fires, and a hull
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
    import mongla_control.fc.srot_protocol as sp      # noqa: F811
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
                f'so a dead companion will NOT surface the hull. Set it to {ours} and save '
                f'(on fw rev >= 7 wait for the "Params saved to flash" statustext, not '
                f'the ACK -- the NVS write is deferred)')
    return (WARN, 'GCS failsafe scoped elsewhere',
            f'FS_GCS_COMPID={int(compid)}, we are {ours} -- our heartbeat does not feed it')


def _baro_health_verdict(health: int | None, present: int | None) -> tuple[str, str, str]:
    """Grade the Bar30 from SYS_STATUS's health bitfield. Pure, testable without a board.

    Why this is a preflight line and not a curiosity. Since fw behaviour rev 3 the board
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

    This used to read `DEPTH_OUT` and that stopped working at fw REV 8, silently.
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
    import mongla_control.fc.srot_protocol as sp      # noqa: F811
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
                'depth it cannot express. ' + 'Arming would command FULL vertical thrust (mixer throttle column is -1 on all four verticals) with the horizontals idle. DO NOT ARM. If the barometer VARIANCE line above is PASS this is a large zero offset, not a dead sensor -- run `ros2 run mongla_planner mongla calibrate_depth` and re-check')
    # Unsaturated: recover the board's own depth. The preview's fixed 0.10 m target is
    # baked into DEPTH_CMD, so subtracting it turns "error against a target" into "how
    # far is the barometer from zero" -- which is the question, and which stops a
    # healthy in-air board from spending a third of the budget on a constant.
    depth_m = depth_cmd / gain + sp.DEPTH_PREVIEW_TARGET_M
    # Report in this stack's sign, not the board's. `depth_m` is the board's
    # internal POSITIVE-DOWN depth; everything an operator reads here -- the
    # `depth telemetry` line printed a few rows above, MonglaState, set_depth -- is
    # NEGATIVE below the surface. Measured on the vehicle 2026-09-14, the same
    # preflight printed "depth telemetry +1.19 m" as PASS and "board reads -1.22 m"
    # as FAIL: one fact, 1.19 vs 1.22 m, with opposite signs in one report. An
    # operator cannot tell from that whether the baro thinks it is above or below
    # zero, which is precisely the question the line exists to answer.
    shown = -depth_m
    if abs(depth_m) >= sp.DEPTH_ERR_ARM_LIMIT_M:
        return (FAIL, 'barometer IMPLAUSIBLE',
                f'board reads {shown:+.2f} m at the surface (negative = submerged, the '
                f'same sign as the depth telemetry line; limit '
                f'{sp.DEPTH_ERR_ARM_LIMIT_M:.2f} m, DEPTH_CMD={depth_cmd:+.2f}, '
                f'DEPTH_P={gain:g}). Arming would command FULL vertical thrust '
                '(mixer throttle column is -1 on all four verticals) with the '
                'horizontals idle. DO NOT ARM')
    if abs(depth_m) > sp.DEPTH_OFFSET_WARN_M:
        return (WARN, 'barometer offset at the surface',
                f'{shown:+.2f} m while disarmed in air (negative = submerged) -- run '
                '`ros2 run mongla_planner mongla calibrate_depth` before diving')
    return (PASS, 'barometer at surface',
            f'{depth_m:+.2f} m (DEPTH_CMD={depth_cmd:+.2f})')


def _yaw_ref_verdict(raw: float | None) -> tuple[str, str, str]:
    """Is ATTITUDE.yaw a magnetic heading, or relative to wherever the BNO booted?

    Only LOCKED means absolute. Anything else and an absolute `turn` aims at a number
    that means nothing -- and does it silently: the move completes normally, on the
    wrong bearing. MAGACC is NOT a proxy (see SrotFC.check_yaw_reference).
    """
    import mongla_control.fc.srot_protocol as sp      # noqa: F811
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


STACK_TASKS = ('STK_MAV', 'STK_SEN', 'STK_CTL', 'STK_UI', 'STK_LORA', 'STK_DSH')


# Bytes of stack a FreeRTOS task has NEVER dipped below (uxTaskGetStackHighWaterMark
# on ESP-IDF reports bytes). Conventional margins, NOT measured failure points: on this
# board the lowest seen was STK_DSH = 2868 bytes on a quiet bench, 2026-09-11.
STACK_FAIL_BYTES = 512


STACK_WARN_BYTES = 1024


def _stack_headroom_verdict(named: dict) -> tuple[str, str, str]:
    """Is any board task close to overflowing its stack?

    The board has always sent this and nothing read it. Six per-task high-water
    marks go out as NAMED_VALUE_FLOAT; a stack overflow on the ESP32 is a hard
    reset of the control board -- thrusters stop mid-run -- and it is the one crash
    that announces itself in advance, in a number we were already receiving.

    It is a HIGH-WATER mark: the minimum ever seen since boot, so it only gets
    worse as more code paths run. A bench value is optimistic; the grade is a
    floor, not a certificate.
    """
    got = {n: named.get(n) for n in STACK_TASKS}
    seen = {n: v for n, v in got.items() if v is not None}
    if not seen:
        return (WARN, 'board stack headroom unknown',
                'no STK_* values -- firmware does not report them, or none arrived')
    name, low = min(seen.items(), key=lambda kv: kv[1])
    detail = ', '.join(f'{n[4:]}={int(v)}' for n, v in sorted(seen.items()))
    if low < STACK_FAIL_BYTES:
        return (FAIL, 'board stack nearly exhausted',
                f'{name} has {int(low)} bytes left ({detail}). A stack overflow '
                'hard-resets the control board mid-run')
    if low < STACK_WARN_BYTES:
        return (WARN, 'board stack low',
                f'{name} has {int(low)} bytes left ({detail})')
    missing = [n for n, v in got.items() if v is None]
    tail = f'; not reported: {", ".join(missing)}' if missing else ''
    return (PASS, 'board stack headroom',
            f'lowest {name}={int(low)} bytes ({detail}){tail}')


# Free heap as the board reports it: ESP.getFreeHeap(), CURRENT free bytes, not a
# low-water mark. MEASURED 156180 B, flat across 48 samples in two recorded bench
# sessions on the Pi (~/mongla_runs/bench_20260902_224032.tlog, txrx_...224254).
# RE-MEASURED 2026-09-15 on the live board (newer firmware: STK_DSH moved 2868 ->
# 2900): 156628 B in three separate `connect --json` reads, +0.3 %. Re-measure
# after any firmware flash that adds a task or a buffer; the stack check beside
# this grades absolute margin and needs no baseline, this one does.
HEAP_BASELINE_BYTES = 156180


# A quarter gone from a value that never moved at rest is a leak or a new
# allocation path; either should be known before a run. Derived from the
# measured baseline, not from a failure point.
HEAP_WARN_FRACTION = 0.75


# Conventional floor, NOT a measured failure point: below this an ESP32 task or
# queue allocation can fail at runtime, which on this board is a reset.
HEAP_FAIL_BYTES = 32768


def _heap_verdict(named: dict) -> tuple[str, str, str]:
    """Is the control board running out of heap?

    Sent since the telemetry existed, read by nothing but a printout. Same
    story as the stack marks beside it. Limit: this is CURRENT free heap, so a
    leak that recovers between reads is invisible -- the right instrument is
    ESP.getMinFreeHeap(), the heap's own high-water mark, which is a one-line
    firmware PR (queued in the research queue).
    """
    heap = named.get('HEAP')
    if heap is None:
        return (WARN, 'board heap unknown',
                'no HEAP value -- firmware does not report it, or none arrived')
    heap = int(heap)
    if heap < HEAP_FAIL_BYTES:
        return (FAIL, 'board heap nearly exhausted',
                f'{heap} B free (< {HEAP_FAIL_BYTES}). An allocation failure on the '
                'ESP32 resets the control board mid-run')
    floor = int(HEAP_BASELINE_BYTES * HEAP_WARN_FRACTION)
    if heap < floor:
        return (WARN, 'board heap below its measured baseline',
                f'{heap} B free vs {HEAP_BASELINE_BYTES} B measured at rest '
                f'(warn under {floor}). A leak or a new allocation path')
    return (PASS, 'board heap', f'{heap} B free (baseline {HEAP_BASELINE_BYTES} B)')


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
                                        SROT_UDP_CONN, MAV_UDP_PORT)
    except Exception:                              # noqa: BLE001
        return device or '/dev/ttyUSB0', ''
    try:
        path = find_srot_serial()
    except Exception as exc:                       # noqa: BLE001 -- AmbiguousSrotSerial
        return '', f'ambiguous: {exc}'
    if path is not None:
        return path, ''
    return SROT_UDP_CONN, probe_udp_mavlink(MAV_UDP_PORT, 2.0)


def _check_srot(skip_mav: bool, device: str = '') -> list[tuple[str, str, str]]:
    """SROT board: port/endpoint, vehicle heartbeat, GAIN, depth sign.

    `device` accepts any pymavlink connection string, for a rig where the board
    hangs off another host and reaches us as UDP through a serial->UDP bridge.
    Without it this section can only ever FAIL on such a rig -- `find_srot_serial()`
    looks for a local CH340 that is, correctly, plugged into the Pi instead. That is
    the tool that decides whether to go in the water reporting "no board" about a
    board that is streaming fine.
    """
    out: list[tuple[str, str, str]] = []
    try:
        from .connection_config import SROT_BAUD, resolve_srot_profile
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
        # NOT `from mongla_control.fc import srot_protocol`: that runs
        # mongla_control/__init__.py, which imports Mongla -> mongla_interfaces, so it
        # needs a fully-built, fully-sourced workspace. This whole tool exists to
        # diagnose a workspace that ISN'T, and section A already reports a missing
        # mongla_interfaces properly -- having the board section die of the same cause
        # would add a second, misleading FAIL and hide the actual board readings.
        from .srot_connect import _load_srot_protocol
        sp = _load_srot_protocol()
        if sp is None:
            out.append((FAIL, 'srot_protocol unavailable',
                        'cannot decode modes/ACKs -- rebuild: ./build_mongla.sh'))
            return out
    except Exception as exc:                       # noqa: BLE001
        out.append((FAIL, 'pymavlink/srot_protocol import', str(exc)))
        return out

    # A preflight that reboots the flight controller is not a preflight. Opening
    # this port asserts DTR and resets the ESP32 (measured -- fc/port_guard.py),
    # so running bringup_check against a live manager silently disarms the
    # vehicle and wipes its configured stream rates. Not doing it is right.
    #
    # But not checked is not passed (2026-10-03, on the vehicle). This was a
    # WARN, so with a manager holding the port the whole board section --
    # barometer, kill, leak, firmware rev -- was skipped and the gate EXITED 0:
    # it passed over a Bar30 it FAILS with the port free. The question this
    # gate answers is "can it arm?", and an unread board does not answer it.
    from mongla_control.fc.port_guard import PortGuard, PortBusy
    _guard = PortGuard(port)
    try:
        _guard.acquire()
    except PortBusy as exc:
        out.append((FAIL, 'srot serial port',
                    f'{port} is held by another process, so the board was NOT '
                    f'checked (opening it would REBOOT the board). Stop the '
                    f'holder and rerun. {str(exc).splitlines()[0]}'))
        return out

    conn = None
    try:
        conn = mavutil.mavlink_connection(port, **baud_kw)
        deadline = time.monotonic() + 6.0
        hb = None
        while time.monotonic() < deadline:
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
        end = time.monotonic() + 2.5
        while time.monotonic() < end:
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
        end = time.monotonic() + 6.0
        while time.monotonic() < end:
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
        out.append(_stack_headroom_verdict(named))
        out.append(_heap_verdict(named))

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

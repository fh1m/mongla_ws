"""Each subsystem's private dialect, translated once.

Every function here is a pure translation of something the vehicle ALREADY
knows into the one vocabulary in `health.py`. The value is not the plumbing --
it is that each translation writes down, in one place, what that subsystem's
numbers actually mean, including the several that read backwards.

The recurring shape, and it is the reason this file exists: **the absence of
bad news is not good news.** Every reporter below distinguishes "reporting, and
fine" from "not reporting", because the vehicle has been bitten by the second
being read as the first at least four times.
"""
from __future__ import annotations

from typing import Optional

from .health import Health, degraded, failed, ok, unknown

# BARO_HEALTH is a FAULT CODE, not a score (bar30.h:33). 0 is the good one, and
# 3 -- the largest -- means the barometer never initialised. Read as a score it
# looks like "mostly healthy", which is how it was read once.
_BARO = {0: (ok, 'healthy'),
         1: (degraded, 'jitter'),
         2: (degraded, 'read failures'),
         3: (failed, 'NOT INITIALISED -- depth hold is unavailable')}

# YAW_REF: ONLY 2 means ATTITUDE.yaw is absolute. Every other value is a
# refusal with a different cause, and on any of them an absolute turn is a
# turn to an unknown heading.
_YAW_REF = {0: (degraded, 'no reference yet'),
            1: (degraded, 'converging'),
            2: (ok, 'LOCKED -- heading is absolute'),
            3: (degraded, 'refused: sensor'),
            4: (degraded, 'refused: field strength'),
            5: (degraded, 'refused: noise')}


def board_link(fc) -> Health:
    """The srot serial link. Arrival time is the RIGHT clock here.

    Measured: the board emits ATTITUDE at exactly 100.000 ms (sd 0.000) and the
    host sees sd 4.741 ms, so all the jitter is ours. For LIVENESS that does
    not matter -- the question is whether anything arrived, not when it was
    measured -- and a board-side stamp would be actively worse, because
    `time_boot_ms` resets on reboot, which is exactly when liveness matters.
    """
    try:
        alive = bool(fc.link_alive())
    except Exception as exc:                     # noqa: BLE001
        return unknown('board_link', f'{type(exc).__name__}: {exc}')
    return ok('board_link', 'heartbeat fresh') if alive else \
        failed('board_link', 'no heartbeat within the stale window')


def mavlink_reader(is_alive, fault_count, last_fault) -> Health:
    """Is the HOST's MAVLink reader thread still running? (B40)

    ⛔ THIS EXISTS TO STOP ONE SPECIFIC MISDIAGNOSIS. `reader_loop` is a bare
    `threading.Thread` target and the only thing draining the link. If it dies,
    `master.messages` stops updating, every reading ages out, and `board_link`
    above reports "no heartbeat within the stale window" -- i.e. a host-side
    software fault is indistinguishable from a dead cable. At the pool, with the
    hull in the water, that sends someone to re-seat a USB-C connector while the
    real cause sits in this process.

    So this is deliberately reported SEPARATELY from `board_link`, and its
    message says which side is at fault. A dead reader is FAILED; faults it
    survived are DEGRADED, because the telemetry they interrupted was real.
    """
    try:
        alive = bool(is_alive())
        faults = int(fault_count())
    except Exception as exc:                     # noqa: BLE001
        return unknown('mavlink_reader', f'{type(exc).__name__}: {exc}')
    if not alive:
        return failed('mavlink_reader',
                      'READER THREAD IS DEAD -- this is a HOST fault, not the '
                      'link. Do not go looking at the cable; restart the manager')
    if faults:
        return degraded('mavlink_reader',
                        f'{faults} fault(s) survived; last: {last_fault()}')
    return ok('mavlink_reader', 'draining the link')


def barometer(named_value) -> Health:
    """`named_value` is a callable name -> float|None (None = not reporting)."""
    v = named_value('BARO_HEALTH')
    if v is None:
        return unknown('barometer', 'BARO_HEALTH absent')
    fn, why = _BARO.get(int(v), (failed, f'unknown code {int(v)}'))
    return fn('barometer', why)


def heading_reference(named_value) -> Health:
    v = named_value('YAW_REF')
    if v is None:
        return unknown('heading_ref', 'YAW_REF absent')
    fn, why = _YAW_REF.get(int(v), (degraded, f'unknown code {int(v)}'))
    return fn('heading_ref', why)


def leak_sensor(leak_enabled: Optional[bool], leaking: Optional[bool]) -> Health:
    """⛔ `LEAK_EN = 0` reads DRY, and that is the trap.

    With the failsafe disabled the sensor still answers "no leak", so the
    vehicle looks safe precisely when nothing is watching. Measured on this
    board: LEAK_EN = 0. Neither the leak failsafe nor the pre-arm refusal is
    armed, so a leak would neither block arming nor surface the hull.
    """
    if leak_enabled is None:
        return unknown('leak', 'LEAK_EN absent')
    if not leak_enabled:
        return failed('leak', 'LEAK_EN=0 -- NOTHING IS WATCHING (reads dry '
                              'either way; not a safe state)')
    if leaking is None:
        return unknown('leak', 'enabled but no reading')
    return failed('leak', 'LEAK DETECTED') if leaking else \
        ok('leak', 'enabled and dry')


def thrusters(health) -> Health:
    """Grade the DRIVER's own presence verdict. Never a message count.

    ⛔ THE OBVIOUS GATE IS A FALSE OK. The board streams `ESC_STATUS` for all
    eight slots whether or not an ESC is attached -- measured, 958 CRC-valid
    frames across two recorded sessions with NOTHING plugged in, every rpm
    exactly 0. So "frames are arriving" and "len(rpm) == 8" are both TRUE on a
    vehicle with no thrusters, and gating on them would report OK to the
    pre-fire gate this reporter exists to feed. A count of messages measures
    the LINK; it says nothing about what is on the end of it.

    `health` is `fc.thruster_health()` -- `(ok, reason)`, built on the board's
    first-arm presence announcement -- or None when the backend has no such
    method. `ok is None` is UNKNOWN and must stay UNKNOWN: on this hull today
    that is the CORRECT answer, not a defect to engineer away.
    """
    if health is None:
        return unknown('thrusters', 'backend reports no thruster health')
    ok_flag, reason = health
    if ok_flag is None:
        return unknown('thrusters', reason)
    if ok_flag is False:
        return failed('thrusters', reason)
    return ok('thrusters', reason)


def thruster_power(kill) -> Health:
    """The 2nd-board rotary kill switch. Three states, because the wire has three.

    ⛔ UNKNOWN IS NOT OK HERE, AND IT IS NOT FAILED EITHER. The switch lives on
    the second board and reaches this one over ESP-NOW; the firmware reports
    kill=false on link loss on purpose (`espnow_link.cpp:49`, justified as
    "display-only" -- but it is not, it goes onto the wire as NAMED_VALUE_FLOAT
    "KILL"). So a missing second board is indistinguishable from a live one
    unless BATTERY_STATUS instance 1 is also present, which is what `srot_fc`
    uses to fold the third state back in.

    Reported UNKNOWN rather than FAILED because a bench vehicle with no second
    board is a legitimate configuration, not a fault -- but it is emphatically
    not OK, because nothing is watching the switch that can stop the hull dead.
    """
    if kill is None:
        return unknown('thruster_power', 'no 2nd-board link -- kill state unseen')
    if kill:
        return failed('thruster_power', 'kill switch ENGAGED -- thruster power CUT')
    return ok('thruster_power', 'thruster power live')


def detector(rate_hz: Optional[float], min_hz: float = 5.0) -> Health:
    if rate_hz is None:
        return unknown('detector', 'no detections topic')
    if rate_hz <= 0.0:
        return failed('detector', 'publishing nothing')
    return ok('detector', f'{rate_hz:.1f} Hz') if rate_hz >= min_hz else \
        degraded('detector', f'{rate_hz:.1f} Hz (below {min_hz:.0f})')


def target_lock(authority: Optional[float], rung: str = '') -> Health:
    """The lock ladder's own decay, restated.

    DEGRADED rather than FAILED while a lower rung still holds: buying time on
    the follower or the anchor is the ladder working, not the ladder failing.
    Authority at zero IS the loss, and it is reported as one.
    """
    if authority is None:
        return unknown('target_lock', 'ladder not running')
    if authority <= 0.0:
        return failed('target_lock', f'authority 0 ({rung or "lost"})')
    if authority >= 1.0:
        return ok('target_lock', f'full authority ({rung or "detection"})')
    return degraded('target_lock',
                    f'authority {authority:.2f} on {rung or "a lower rung"}')


def target_pose(pose_msg, max_tilt_deg: float = 0.0) -> Health:
    """The 6-DoF pose, judged against the aim gate that would use it.

    NO POSE IS NOT SQUARE. And the WORSE flip branch is what counts -- the
    point estimate alone is the lucky branch.
    """
    if pose_msg is None:
        return unknown('target_pose', 'no pose published')
    if not pose_msg.ok:
        return degraded('target_pose', pose_msg.reason or 'refused')
    worst = float(pose_msg.off_axis_deg) + max(float(pose_msg.yaw_spread_deg),
                                               float(pose_msg.pitch_spread_deg))
    if max_tilt_deg > 0.0 and worst > max_tilt_deg:
        return degraded('target_pose',
                        f'{worst:.1f} deg worst-case > {max_tilt_deg:.0f} gate')
    return ok('target_pose', f'{worst:.1f} deg worst-case, '
                             f'range {pose_msg.range_m:.2f} m')

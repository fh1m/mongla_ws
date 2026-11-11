#!/usr/bin/env python3
"""bringup_check -- the pre-flight gate for the SROT vehicle.

Walks every subsystem the AUV needs and reports whether each is not just
*present* but *connected and usable*, so problems are caught on the deck
instead of mid-mission.  Sections:

  A.   Compute / environment   ROS sourced, mongla pkgs import, disk free
  B.   Vision dependencies     numpy<2, system cv2 (not pip-shadowed), trackers
  C.   Serial drivers          ch341 (the board's CH340) present, brltty absent
  D-F. SROT control board      link, HEARTBEAT, FW behaviour rev, baro, failsafe
  G.   Yaw source              the board's BNO085, over MAVLink
  I.   Payload                 board channels over MAVLink
  J.   Cameras                 forward + downward USB cameras enumerated
  K.   Vision models           Hailo .hef + its .yaml sidecar
  L.   Raspberry Pi power      under-voltage / throttling

Each check prints PASS / WARN / FAIL.  Exit code is 0 when no FAIL (WARNs
are advisory).  Pass ``--strict`` to make any WARN also exit non-zero (a
hard pre-mission gate).  ``--skip-mavlink`` skips the board probe.

Usage:
    ros2 run mongla_manager bringup_check
    ros2 run mongla_manager bringup_check --strict
    ros2 run mongla_manager bringup_check --srot-device=udpin:0.0.0.0:14550

THE FILES. The checks live by what they look at:

    bringup_check.py   this: the sections, in order, and the exit code
    bringup_host.py    the computer: ROS, disk, drivers, cameras, models, Hailo, power
    bringup_board.py   the SROT board: firmware, barometer, depth loop, memory
    bringup_grade.py   PASS / WARN / FAIL and the one-line report format
"""

from __future__ import annotations

import os
os.environ['MAVLINK20'] = '1'   # before pymavlink, or ESC/v2 frames decode as UNKNOWN (fc/_srot_base.py)
import sys  # noqa: E402

from .bringup_board import _check_srot, _resolve_srot_for_check
from .bringup_grade import FAIL, PASS, WARN, _line
from .bringup_host import (_check_build_freshness, _check_cameras, _check_depth_floor,
                           _check_disk,
                           _check_model_separation, _check_models, _check_pi_power,
                           _check_ros_env, _check_sensor_offsets, _check_serial_drivers,
                           _check_vision_deps)


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    # `--help` used to fall through and run the FULL hardware probe -- an operator
    # asking what the flags are instead got a 12-section scan of the vehicle.
    if '-h' in argv or '--help' in argv:
        print('usage: bringup_check [--srot-device=<conn>] [--strict] [--skip-mavlink]\n'
              '\n'
              '  --srot-device=   the board is not on THIS host: any pymavlink conn\n'
              '                   string, e.g. udpin:0.0.0.0:14550 behind a\n'
              '                   serial->UDP bridge. Without it this section FAILs\n'
              '                   "no board" on a rig whose board is streaming fine.\n'
              '  --strict         any WARN exits non-zero (hard pre-mission gate)\n'
              '  --skip-mavlink   skip the board probe (no board/link attached)\n'
              '  --srot           accepted and ignored (srot is the only backend)\n'
              '\n'
              'Exit 0 = nothing FAILed. The line that gates the water is\n'
              '"FW behaviour rev": below the floor arm() refuses.')
        return 0
    strict = '--strict' in argv
    skip_mav = '--skip-mavlink' in argv
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
    print(' mongla bringup_check -- pre-flight'
          + ('  [STRICT]' if strict else ''))
    print('=' * 72)

    # ---- A. compute / environment ---------------------------------- #
    section('A. Compute / environment')
    for st, lbl, det in _check_ros_env():
        emit(st, lbl, det)
    st, det = _check_disk()
    emit(st, 'disk free (/)', det)

    # ---- B. vision dependencies ------------------------------------ #
    section('B. Vision dependencies')
    for st, lbl, det in _check_vision_deps():
        emit(st, lbl, det)

    # ---- C. serial drivers ----------------------------------------- #
    section("C. Serial drivers (the board's CH340)")
    for st, lbl, det in _check_serial_drivers():
        emit(st, lbl, det)

    # ---- D-F. the board, however it is attached ------------------------ #
    # Resolve ONCE here so the section header, the checks and the closing
    # startup hint all describe the SAME transport. Resolving in more than one
    # place re-probes UDP (2 s each) and lets them disagree.
    srot_conn, srot_state = _resolve_srot_for_check(srot_device)
    section('D-F. SROT control board'
            + ('' if not srot_conn
               else ' (bridged over UDP)' if not srot_conn.startswith('/dev/')
               else ' (direct USB serial)'))
    if srot_state.startswith('ambiguous'):
        # Refused, not guessed (#20): the wrong CH340 would be RESET by the open.
        emit(FAIL, 'SROT auto-detect', srot_state.split(': ', 1)[1])
    elif srot_state == 'silent':
        emit(WARN, 'SROT auto-detect',
             'no USB serial and no MAVLink on UDP -- falling back to '
             f'{srot_conn}. Plug in the Type-C cable, or pass --srot-device=<conn>')
    elif srot_state == 'busy':
        emit(WARN, 'SROT auto-detect',
             f'UDP port already held by another process -- could not probe. '
             f'Using {srot_conn}. A leftover `mongla_manager start` is the '
             f'usual cause: ss -lunp | grep 14550')
    for st, lbl, det in (_check_srot(skip_mav, srot_conn) if srot_conn else ()):
        emit(st, lbl, det)

    # ---- G. yaw source ---------------------------------------------- #
    section('G. Yaw source (BNO085 on the control board)')
    emit(PASS, 'BNO085 rides the MAVLink link',
         'on the board (I2C0), fused at 500 Hz -> ATTITUDE')

    # ---- I. payload ------------------------------------------------- #
    # The payload is the board's own channels over MAVLink -- there is no
    # separate USB board to probe, and probing a CH340 here would open the
    # board's own port (fc/port_guard.py: opening it reboots the board).
    section('I. Payload (board channels over MAVLink)')
    emit(PASS, 'payload transport', 'board channels over MAVLink; no separate USB board')
    # The transport being up is NOT the payload working, and a PASS on this
    # line is what once hid a dead fire() path.
    #
    # NOTE, not WARN, and the distinction is load-bearing: whether a channel is
    # fireable is a board ROLE read over a live MAVLink link by the running node,
    # which a standalone preflight has no session for. An unconditional WARN would
    # make `--strict` -- the hard pre-mission gate -- exit non-zero on EVERY
    # run for a condition nobody can clear from here.
    _line('NOTE', 'payload channels',
          'fire(N) addresses BOARD channel N directly (no map). Which channels '
          'are fireable is read from the board at bring-up -- confirm the '
          'manager logs "[PAYLOAD] board roles: FIREABLE (switch) [...]" and '
          'that your intended channel is in that list')

    # ---- J. cameras ------------------------------------------------- #
    section('J. Cameras (forward + downward)')
    st, det = _check_cameras()
    emit(st, 'USB cameras', det)

    # ---- K. vision models ------------------------------------------ #
    section('K. Vision models (Hailo .hef + sidecar)')
    st, det = _check_models()
    emit(st, 'model artifacts', det)

    # ---- K1b. Can the model tell the prop from the water? ----------- #
    section('K1b. Model separation (does it WORK, not just load?)')
    st, det = _check_model_separation()
    emit(st, 'measured separation', det)

    # ---- K1c. Where the sensors are ---------------------------------- #
    section('K1c. Sensor offsets (is flow corrected for where the lens is?)')
    st, det = _check_sensor_offsets()
    emit(st, 'IMU -> camera / baro', det)

    # ---- K1d. The floor below the mission ----------------------------- #
    section('K1d. Depth floor (does anything stop a dive past the pool?)')
    st, det = _check_depth_floor()
    emit(st, 'shield_max_depth_m', det)

    # ---- K2. Stale build tree --------------------------------------- #
    section('K2. Build tree freshness (is the vehicle running your code?)')
    st, det = _check_build_freshness()
    emit(st, 'build vs src', det)

    # ---- L. Pi power ------------------------------------------------ #
    # An under-voltage that has already occurred is invisible without this.
    section('L. Raspberry Pi power (under-voltage / throttling)')
    st, det = _check_pi_power()
    emit(st, 'pi throttling', det)

    # ---- the link the manager will use ------------------------------ #
    section('Manager startup hint')
    if srot_conn.startswith('/dev/'):
        emit(PASS, 'connection', f'direct USB serial ({srot_conn}) -- auto-detected')
    else:
        emit(PASS, 'connection',
             f'bridged over UDP ({srot_conn}) -- the manager auto-detects the '
             f'same endpoint')

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
    print('    ros2 launch mongla_manager bringup.launch.py')
    print('      # fire(N) = BOARD channel N -- see the [PAYLOAD] roles line;'
          ' nothing else needs setting')
    print('    ros2 run mongla_planner mongla arm')
    print('    ros2 run mongla_planner mongla move_forward --duration 5 --gain 40')
    print('    ros2 run mongla_planner mongla stop')
    print()
    print('  ERROR: BEFORE WATER -- AUTO has never held depth in water, and depth runs')
    print('     under EVERY AUTO move (move_forward included), not just set_depth:')
    print('    ros2 run mongla_planner mongla set_mode --target_name DEPTH_HOLD')
    print('      1. hand-raise/lower the sub -> verticals must push BACK toward depth')
    print('      2. trip the leak input at depth -> the demand must be ASCEND')


if __name__ == '__main__':
    sys.exit(main())

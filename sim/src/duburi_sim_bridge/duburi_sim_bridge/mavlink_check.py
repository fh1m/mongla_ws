#!/usr/bin/env python3

"""Assert that ArduSub SITL satisfies the MAVLink half of the drop-in contract.

`contract_check` covers the ROS topics. This covers the other half, which is not
ROS at all: duburi_ws speaks to the vehicle with direct pymavlink over
udpin:0.0.0.0:14550 and needs a specific set of messages at specific rates.

It connects the same way duburi_ws does and asks for the same message intervals,
so if this passes there is nothing left for the autonomy stack to trip over.

Usage:
    ros2 run duburi_sim_bridge mavlink_check
    ros2 run duburi_sim_bridge mavlink_check --timeout 90

Exits non-zero if the contract is unmet, so it can gate CI.
"""

import argparse
import sys
import time

from pymavlink import mavutil

# What duburi_manager's connection_config.py uses in sim mode.
DEFAULT_ENDPOINT = 'udpin:0.0.0.0:14550'

# Message id -> (name, rate the autonomy stack asks for, required)
WANTED = {
    30: ('ATTITUDE', 50.0, True),
    178: ('AHRS2', 50.0, True),
    0: ('HEARTBEAT', 1.0, True),
    1: ('SYS_STATUS', 5.0, False),
    147: ('BATTERY_STATUS', 2.0, False),
    33: ('GLOBAL_POSITION_INT', 10.0, False),
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--endpoint', default=DEFAULT_ENDPOINT)
    parser.add_argument('--timeout', type=float, default=60.0,
                        help='Seconds to wait for the first heartbeat.')
    parser.add_argument('--sample', type=float, default=10.0,
                        help='Seconds to measure message rates over.')
    args = parser.parse_args()

    print(f'connecting to {args.endpoint}')
    link = mavutil.mavlink_connection(args.endpoint)

    heartbeat = link.wait_heartbeat(timeout=args.timeout)
    if heartbeat is None:
        print(f'\nFAIL: no HEARTBEAT within {args.timeout:.0f}s.\n'
              '  Is ArduSub running? Check that sim.launch.py got past '
              'wait_for_gazebo, and that nothing else already holds UDP 14550.')
        return 1

    print(f'HEARTBEAT from system {link.target_system} '
          f'component {link.target_component}')

    # Request the same intervals duburi_ws requests at startup.
    for msg_id, (_name, hz, _required) in WANTED.items():
        link.mav.command_long_send(
            link.target_system, link.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0,
        )

    counts = {}
    start = time.time()
    while time.time() - start < args.sample:
        msg = link.recv_match(blocking=True, timeout=1.0)
        if msg is not None:
            counts[msg.get_type()] = counts.get(msg.get_type(), 0) + 1
    elapsed = time.time() - start

    print(f'\nrates measured over {elapsed:.1f}s')
    print('-' * 62)
    ok = True
    for msg_id, (name, wanted_hz, required) in WANTED.items():
        rate = counts.get(name, 0) / elapsed
        if counts.get(name, 0) == 0:
            status = 'MISSING' if required else 'absent'
            if required:
                ok = False
        # Allow a wide band: ArduSub schedules on its own loop and will not hit
        # a requested rate exactly, and being fast is not a failure.
        elif rate < wanted_hz * 0.5:
            status = 'SLOW' if required else 'slow'
            if required:
                ok = False
        else:
            status = 'ok'
        print(f'  {status:<8} {name:<22} {rate:6.1f} Hz  (asked {wanted_hz:.0f})')

    extra = sorted(set(counts) - {n for n, _, _ in WANTED.values()})
    if extra:
        print(f'\nalso received: {", ".join(extra)}')

    ahrs2 = link.messages.get('AHRS2')
    if ahrs2 is not None:
        print(f'\nAHRS2  yaw {ahrs2.yaw:+.3f} rad   '
              f'altitude {ahrs2.altitude:+.3f} m   '
              f'roll {ahrs2.roll:+.3f}  pitch {ahrs2.pitch:+.3f}')

    print('-' * 62)
    print('MAVLink contract satisfied\n' if ok
          else 'MAVLink contract NOT satisfied\n')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())

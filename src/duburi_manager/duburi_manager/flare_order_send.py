"""``ros2 run duburi_manager flare_order R-B-Y --port /dev/ttyACM0`` -- tell the vehicle.

SAUVC Task 4: the judges tell the TEAM the flare order after Navigation, and the vehicle
may not surface. This runs on the topside laptop, plugged into the LoRa ground station's
USB, and sends the order as MAV_CMD_SROT_FLARE_ORDER. The ground station relays it over
LoRa into the board's normal command dispatch, which latches it in RAM and ACKs.

It REPEATS until the board ACKs, because one LoRa uplink slot can be lost and the command
is idempotent under its nonce. The ACK comes back through the ground station's config-plane
relay, so "ACCEPTED" here means the vehicle itself has the order -- not that the radio sent.

It replaces Bondor on the ground-station port for the few seconds it runs: the station only
transmits uplink while something on USB is talking to it, and this tool heartbeats so it is.

The link has to close. SF7 LoRa in pool water is a metres-scale link: hold the vehicle at
the listen station near the starting zone, antenna outside the hull, before sending.

Exit 0 = vehicle ACCEPTED, 1 = no ACK before --timeout, 2 = vehicle DENIED / bad input.
"""
from __future__ import annotations

import argparse
import os
import random
import sys
import time

os.environ.setdefault('MAVLINK20', '1')

from pymavlink import mavutil                                        # noqa: E402

from duburi_control.fc import srot_protocol as sp                    # noqa: E402

# Topside identity, same as the ground station's synthesised GCS (sys 255).
SRC_SYS, SRC_COMP = 255, 190
BOARD_SYS, BOARD_COMP = 1, 1
RESEND_S = 1.0


def send_until_acked(conn, colours, nonce, timeout_s, *, clock=time.monotonic,
                     out=print) -> int:
    """Send the order every RESEND_S until the board ACKs; return the exit code."""
    params = sp.flare_order_params(colours, nonce)
    deadline = clock() + timeout_s
    next_send = 0.0
    sent = 0
    while clock() < deadline:
        now = clock()
        if now >= next_send:
            conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            conn.mav.command_long_send(BOARD_SYS, BOARD_COMP, sp.CMD_SROT_FLARE_ORDER,
                                       0, *params)
            sent += 1
            next_send = now + RESEND_S
        msg = conn.recv_match(type=['COMMAND_ACK', 'STATUSTEXT', 'NAMED_VALUE_FLOAT'],
                              blocking=True, timeout=0.2)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == 'STATUSTEXT' and 'FLARE' in msg.text:
            out(f'[vehicle] {msg.text}')
        elif kind == 'NAMED_VALUE_FLOAT' and msg.name == 'LORA_RSSI':
            out(f'[station] LORA_RSSI {msg.value:.0f} dBm (sent {sent})')
        elif kind == 'COMMAND_ACK' and msg.command == sp.CMD_SROT_FLARE_ORDER:
            name = sp.ACK_NAMES.get(msg.result, str(msg.result))
            if msg.result == sp.ACK_ACCEPTED:
                out(f'ACCEPTED: vehicle has {"-".join(colours)} (nonce {nonce}, '
                    f'{sent} sent)')
                return 0
            out(f'vehicle refused the order: {name}')
            return 2
    out(f'NO ACK after {timeout_s:.0f} s ({sent} sent). Is the vehicle at the listen '
        f'station, and is the ground station USB this port?')
    return 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('order', help="e.g. R-B-Y, RBY or red,blue,yellow")
    ap.add_argument('--port', required=True,
                    help='LoRa ground station USB, e.g. /dev/ttyACM0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--timeout', type=float, default=60.0)
    args = ap.parse_args(argv)
    try:
        colours = sp.parse_flare_order(args.order)
        sp.flare_order_params(colours, 0)
    except ValueError as exc:
        print(f'bad order: {exc}')
        return 2
    nonce = random.randint(1, 65535)
    print(f'sending {"-".join(colours)} nonce {nonce} to {args.port}')
    conn = mavutil.mavlink_connection(args.port, baud=args.baud,
                                      source_system=SRC_SYS, source_component=SRC_COMP)
    try:
        return send_until_acked(conn, colours, nonce, args.timeout)
    finally:
        conn.close()


if __name__ == '__main__':
    sys.exit(main())

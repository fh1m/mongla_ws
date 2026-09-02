#!/usr/bin/env python3
"""Live: can we actually raise ATTITUDE to the firmware's 20 ms floor?

SROT_MESSAGE_RATES has asked for 50 Hz since the srot backend was written, but
the board has only ever been observed at its 10 Hz default -- the request has
never been verified against hardware. Also probes the floor itself: asking for
100 Hz must be CLAMPED (or denied), not silently accepted and ignored.
"""
import sys, time, statistics as st
sys.path.insert(0, '/home/fh1m/duburi_ws/src/duburi_control')
from pymavlink import mavutil
from duburi_control.fc import srot_protocol as sp
from duburi_control.fc.port_guard import PortGuard

DEV='/dev/ttyUSB0'
ATT = mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE
g=PortGuard(DEV); g.acquire()
m = mavutil.mavlink_connection(DEV, baud=115200,
        source_system=sp.SOURCE_SYSID, source_component=sp.SOURCE_COMPID)
m.wait_heartbeat(timeout=10)
print("connected\n")

def measure(dur=6.0):
    ts=[]; t0=time.time()
    while time.time()-t0<dur:
        msg=m.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
        if msg: ts.append(time.time())
    if len(ts)<3: return None,None
    d=[1000*(ts[i]-ts[i-1]) for i in range(1,len(ts))]
    return len(ts)/dur, st.median(d)

def ask(hz):
    us = 0.0 if hz<=0 else 1e6/hz
    m.mav.command_long_send(sp.VEHICLE_SYSID, sp.VEHICLE_COMPID,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, float(ATT), us,0,0,0,0,0)
    t0=time.time()
    while time.time()-t0 < 2.0:
        a=m.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.5)
        if a and a.command==mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
            names={0:'ACCEPTED',1:'TEMP_REJECTED',2:'DENIED',3:'UNSUPPORTED',4:'FAILED',5:'IN_PROGRESS'}
            return names.get(a.result,str(a.result))
    return 'no ACK'

hz,ms = measure(); print(f"baseline           : {hz:.2f} Hz  (period {ms:.1f} ms)")
print(f"ask 50 Hz          : ACK {ask(50)}")
hz,ms = measure(); print(f"  measured         : {hz:.2f} Hz  (period {ms:.1f} ms)")
print(f"ask 100 Hz (10 ms) : ACK {ask(100)}")
hz2,ms2 = measure(); print(f"  measured         : {hz2:.2f} Hz  (period {ms2:.1f} ms)")
print(f"  -> {'CLAMPED at the 20 ms floor' if ms2 > 15 else 'went above 50 Hz!'}")
print(f"restore default    : ACK {ask(0)}")
hz3,ms3 = measure(4.0); print(f"  measured         : {hz3:.2f} Hz  (period {ms3:.1f} ms)")
m.close(); g.release()

# RESULT, fw rev 14, 2026-09-03:
#   baseline  10.17 Hz (100.0 ms) | ask 50 -> 50.17 Hz (20.0 ms), ACK ACCEPTED
#   ask 100   50.00 Hz ( 20.0 ms) -- ACK ACCEPTED, silently CLAMPED at the floor
#   restore   10.25 Hz (100.0 ms)
# The clamp is the finding: the ACK cannot tell you the rate you got. Measure.

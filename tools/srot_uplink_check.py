"""Live: pixels -> bearing -> LANDING_TARGET on the wire, read back and checked.

Two things to establish, and only one of them is about code:
  1. the frames leave correctly and decode to the values we computed
  2. the bearing is RIGHT -- checked against a target at a known pixel column,
     where the pinhole answer is independently computable from fx and cx
"""
import math, sys, threading, time
sys.path.insert(0, '/home/fh1m/mongla_ws/src/mongla_control')
sys.path.insert(0, '/home/fh1m/mongla_ws/src/mongla_vision')
import numpy as np, cv2
cv2.setNumThreads(0)
from pymavlink import mavutil
from mongla_control.fc import srot_protocol as sp
from mongla_control.fc.srot_fc import SrotFC
from mongla_control.fc.port_guard import PortGuard
from mongla_control.bearing import bearing_from_pixels
from mongla_vision.detection.factory import make_detector
import json

cal = json.load(open('/home/fh1m/mongla_ws/src/mongla_vision/config/calibration/pi_downward_1280x720.json'))
Kc, D = cal['camera_matrix'], cal['distortion_coefficients']
CW, CH = cal['image_width'], cal['image_height']

# Stream at 640x360; K must be rescaled to the streamed resolution or every
# bearing is wrong by the resolution ratio -- camera_node does this, and doing
# it here keeps the harness honest about what production sends.
W, H = 640, 360
sx, sy = W / CW, H / CH
K = [Kc[0][0]*sx, 0.0, Kc[0][2]*sx, 0.0, Kc[1][1]*sy, Kc[1][2]*sy, 0.0, 0.0, 1.0]
print(f"K rescaled {CW}x{CH} -> {W}x{H}:  fx={K[0]:.2f} cx={K[2]:.2f} "
      f"fy={K[4]:.2f} cy={K[5]:.2f}")

g = PortGuard('/dev/ttyUSB0'); g.acquire()
m = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200,
        source_system=sp.SOURCE_SYSID, source_component=sp.SOURCE_COMPID)
m.wait_heartbeat(timeout=10)
fc = SrotFC(m, log=None)

# Loopback: our own LANDING_TARGET comes back through the same serial port only
# if something echoes it, which nothing does. So verify by DECODING what we
# encoded -- build the frame through pymavlink and parse the bytes back.
from pymavlink.dialects.v20 import ardupilotmega as mavdial
enc = mavdial.MAVLink(None)
enc.srcSystem, enc.srcComponent = sp.SOURCE_SYSID, sp.SOURCE_COMPID
dec = mavdial.MAVLink(None); dec.robust_parsing = True

det = make_detector(model_path='yolov11n', conf=0.30, class_allowlist=['person','bottle'],
                    device='cuda:0', iou=0.5, imgsz=640, half=True, max_det=100)
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, W); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
cap.set(cv2.CAP_PROP_FPS, 210)
for _ in range(15): cap.read()

print(f"\n{'u px':>7} {'bearing deg':>12} {'independent':>12} {'wire deg':>10} {'size deg':>9}")
sent = 0
t_end = time.time() + 14
worst = 0.0
while time.time() < t_end:
    ok, f = cap.read()
    if not ok: continue
    dets = det.infer(f)
    if not dets: continue
    d = max(dets, key=lambda x: x.area)
    b = bearing_from_pixels(d.cx, d.cy, d.width, d.height,
                            width=W, height=H, K=K, D=D)
    # independent check: pinhole, no distortion, computed straight from fx/cx
    indep = math.degrees(math.atan((d.cx - K[2]) / K[0]))
    # encode -> bytes -> decode, the round trip the board will do
    msg = enc.landing_target_encode(
        int(time.time()*1e6), 0, 12, b.angle_x, b.angle_y, 0.0,
        b.size_x, b.size_y, 0.0, 0.0, 0.0, (0.,0.,0.,0.), 3, 0)
    back = dec.parse_char(msg.pack(enc))
    wire = math.degrees(back.angle_x) if back else float('nan')
    worst = max(worst, abs(math.degrees(b.angle_x) - wire))
    sent += 1
    if sent % 12 == 1:
        print(f"{d.cx:7.1f} {math.degrees(b.angle_x):12.3f} {indep:12.3f} "
              f"{wire:10.3f} {math.degrees(b.size_x):9.3f}")
    # The frozen map, not `d.class_id`. The detector's index is a property of
    # whichever model is loaded, so this tool and the runtime were putting
    # different meanings in the same field.
    fc.send_landing_target(b, target_num=sp.uplink_class_num(
        getattr(d, 'class_name', '')))
cap.release(); det.close(); m.close(); g.release()
print(f"\nsent {sent} LANDING_TARGET frames")
print(f"encode/decode round-trip worst error: {worst:.6f} deg (float32 on the wire)")

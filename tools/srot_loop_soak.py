"""The shipping config, sustained. A sweep finds a ceiling; a soak finds drift.

VISION_LOOP_HZ_SROT is 50, not the 70 the sweep reached, and this checks that
choice holds over minutes rather than seconds. Jitter is reported as a
DISTRIBUTION, because a control loop is hurt by its worst ticks, not its median.
"""
import math, os, statistics as st, sys, time, json
WS=os.path.expanduser('~/duburi_ws/src')
for p in ('duburi_control','duburi_vision'):
    q=os.path.join(WS,p)
    if q not in sys.path: sys.path.insert(0,q)
import cv2; cv2.setNumThreads(0)
from pymavlink import mavutil
from duburi_control.bearing import BearingFilter, bearing_from_pixels
from duburi_control.fc import srot_protocol as sp
from duburi_control.fc.port_guard import PortGuard
from duburi_control.fc.srot_fc import SrotFC
from duburi_control.motion_rates import VISION_LOOP_HZ_SROT
from duburi_vision.detection.factory import make_detector

W,H=640,360
c=json.load(open(os.path.expanduser('~/duburi_ws/src/duburi_vision/config/calibration/pi_downward_1280x720.json')))
Kc,D=c['camera_matrix'],c['distortion_coefficients']
sx,sy=W/c['image_width'],H/c['image_height']
K=[Kc[0][0]*sx,0,Kc[0][2]*sx,0,Kc[1][1]*sy,Kc[1][2]*sy,0,0,1]
HZ=VISION_LOOP_HZ_SROT
SECONDS=float(sys.argv[1]) if len(sys.argv)>1 else 90.0
print(f"soak: VISION_LOOP_HZ_SROT = {HZ} Hz for {SECONDS:.0f} s\n")

g=PortGuard('/dev/ttyUSB0'); g.acquire()
conn=mavutil.mavlink_connection('/dev/ttyUSB0',baud=115200,
  source_system=sp.SOURCE_SYSID,source_component=sp.SOURCE_COMPID)
conn.wait_heartbeat(timeout=10)
fc=SrotFC(conn,log=None)
fc.set_message_rate(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,50)
# B28: REFUSE TO MEASURE INTO A BOARD THAT DISCARDS THE COMMAND.
# `SROT_MOVE` leaves the board latched in AUTO, and in AUTO the firmware
# throws away every axis of MANUAL_CONTROL and reports nothing. A sweep run
# in that state drives zero thrust and still prints a full set of numbers --
# a plausible measurement standing in for an absent one, which is this
# project's signature defect. Reuse the verb path's own verified set-mode
# rather than a second copy of it.
from duburi_control.vision_verbs import _require_srot_vision_mode
_require_srot_vision_mode(fc, None, 'srot_loop_soak')
det=make_detector(model_path='yolov11n',conf=0.35,class_allowlist=['person'],
                  device='cuda:0',iou=0.5,imgsz=640,half=True,max_det=20)
cap=cv2.VideoCapture(0,cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH,W); cap.set(cv2.CAP_PROP_FRAME_HEIGHT,H)
cap.set(cv2.CAP_PROP_FPS,210); cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
for _ in range(20): cap.read()
bf=BearingFilter(); period=1.0/HZ
ticks=[]; lat=[]; quarters=[[],[],[],[]]
t0=time.perf_counter(); tp=t0
while (now:=time.perf_counter())-t0 < SECONDS:
    a=time.perf_counter()
    ok,f=cap.read()
    if not ok: continue
    d=det.infer(f)
    t=max(d,key=lambda x:x.area) if d else None
    br=(bearing_from_pixels(t.cx,t.cy,t.width,t.height,width=W,height=H,K=K,D=D)
        if t is not None else None)
    b=bf.update(br,time.perf_counter())
    cmd=0.0
    if b is not None and abs(b.angle_x)>math.radians(2.5):
        cmd=max(-1.,min(1.,b.angle_x/math.radians(20.)))*0.25
    fc.manual(fwd=0.,lat=cmd,up=0.,yaw=0.)
    z=time.perf_counter()
    lat.append((z-a)*1000); ticks.append((z-tp)*1000); tp=z
    quarters[min(3,int(4*(z-t0)/SECONDS))].append((z-a)*1000)
    time.sleep(max(0.,period-(time.perf_counter()-a)))
cap.release(); det.close(); conn.close(); g.release()

n=len(ticks); s=sorted(ticks); tgt=1000.0/HZ
late=sum(1 for v in ticks if v>tgt*1.25)
print(f"ticks {n} in {SECONDS:.0f} s -> {n/SECONDS:.2f} Hz "
      f"(target {HZ:.0f})")
print(f"\nperiod ms   median {st.median(ticks):6.2f}   target {tgt:.2f}")
print(f"            p95 {s[int(.95*n)]:6.2f}   p99 {s[int(.99*n)]:6.2f}   "
      f"max {max(ticks):6.2f}")
print(f"            sd  {st.pstdev(ticks):6.3f}")
print(f"late ticks (>25 % over)  {late}  = {100*late/n:.2f} %")
print(f"\ne2e latency ms  median {st.median(lat):.2f}  p95 {sorted(lat)[int(.95*len(lat))]:.2f}")
print("\ndrift check -- median e2e per quarter of the run:")
for i,q in enumerate(quarters):
    if q: print(f"  Q{i+1}  {st.median(q):6.2f} ms   ({len(q)} ticks)")
sp_=[st.median(q) for q in quarters if q]
print(f"  spread across quarters: {max(sp_)-min(sp_):.2f} ms "
      f"-> {'STABLE' if max(sp_)-min(sp_) < 1.0 else 'DRIFTING'}")

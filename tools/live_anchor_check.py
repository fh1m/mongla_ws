#!/usr/bin/env python3
"""The anchor holding a target on the LIVE vehicle camera, with no detector.

⭐ This is the rung's whole purpose, exercised in the one condition that makes
it necessary: the detector reports nothing, so a box-based tracker has nothing
to carry forward. The bank does not need one -- it is handed a frame, and it
says where the remembered scene went.

Enrol from the first frames, then locate on everything after, live, on the Pi.
"""
import sys, time, importlib.util
import numpy as np, cv2

SRC = '/home/fh1m/mongla_ws/src/mongla_vision'
sys.path.insert(0, SRC)
spec = importlib.util.spec_from_file_location(
    'xfeat_onnx', f'{SRC}/mongla_vision/anchor/xfeat_onnx.py')
X = importlib.util.module_from_spec(spec); sys.modules['xfeat_onnx'] = X
spec.loader.exec_module(X)
# Load `bank` by PATH, not as a package: `mongla_vision/__init__.py` pulls in
# preflight -> rclpy, and this script deliberately runs outside the ROS graph.
_bs = importlib.util.spec_from_file_location(
    'bank', f'{SRC}/mongla_vision/anchor/bank.py')
_bm = importlib.util.module_from_spec(_bs)
sys.modules['bank'] = _bm
# `bank.py` does `from .anchor import ...`; give it a package to resolve that.
import types
_pkg = types.ModuleType('anchorpkg'); _pkg.__path__ = [f'{SRC}/mongla_vision/anchor']
sys.modules['anchorpkg'] = _pkg
_bm.__package__ = 'anchorpkg'
_as = importlib.util.spec_from_file_location(
    'anchorpkg.anchor', f'{SRC}/mongla_vision/anchor/anchor.py')
_am = importlib.util.module_from_spec(_as); sys.modules['anchorpkg.anchor'] = _am
_as.loader.exec_module(_am)
_bs.loader.exec_module(_bm)
CheckpointBank = _bm.CheckpointBank

be = X.XFeatONNX('/home/fh1m/hailo_models/xfeat_320x240.onnx', top_k=1024, threads=3)
bank = CheckpointBank(be, capacity=64, period_s=0.333)

cap = cv2.VideoCapture('/dev/video0')
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
for _ in range(6): cap.read()

print('enrolling 4 checkpoints over ~2 s ...')
for i in range(4):
    ok, f = cap.read()
    if not ok: continue
    g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
    r = bank.enrol(g, roi=None, det_conf=1.0, label='scene', force=True)
    print(f'  {i}: {r.reason} {r.keypoints} kp')
    time.sleep(0.4)

print(f'bank {bank.size}, shortlist<= {bank.shortlist_k()}')
print(f'{"t":>5} {"inliers":>8} {"ref":>4} {"dx px":>7} {"dy px":>7} {"scale":>6} {"ms":>6}')
t0 = time.perf_counter()
held = 0; n = 0
while time.perf_counter() - t0 < 12:
    ok, f = cap.read()
    if not ok: break
    g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
    t1 = time.perf_counter(); p = bank.locate(g, label='scene')
    ms = (time.perf_counter() - t1) * 1000
    n += 1; held += int(p.ok)
    pose = p.pose
    dx = dy = sc = float('nan')
    if pose is not None and pose.ok:
        dx, dy, sc = pose.tx, pose.ty, pose.scale
    print(f'{time.perf_counter()-t0:>5.1f} {p.inliers:>8} {str(p.index):>4} '
          f'{dx:>7.1f} {dy:>7.1f} {sc:>6.2f} {ms:>6.1f}')
    time.sleep(0.25)
cap.release()
print(f'\nheld the scene on {100*held/max(1,n):.0f}% of {n} frames; '
      f'measured match cost {bank.match_ms:.1f} ms')

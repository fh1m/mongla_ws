"""Hot-path OpenCV cost, per frame, for whichever cv2 this interpreter imports.

Bar, written before the run (2026-11-11): a newer OpenCV is worth leaving the
distro's for if the summed per-frame hot path (decode + letterbox resize +
BGR2RGB + BGR2GRAY + corners + pyramidal LK) drops by at least 25 %, AND
cv_bridge and hailo_platform still import and convert in the same interpreter.
That bar was measured with cv_bridge (measured-bars 77); the stack has since
dropped it for `mongla_vision.image_msg` (measured-bars 78), so the check below
is that our own conversion still round-trips under the cv2 and NumPy on test.
Falsifier for the bench: every op's output is hashed or summarised, so two
builds that disagree on pixels show it.
"""
import json, sys, time
import numpy as np
out = {'python': sys.version.split()[0], 'numpy': np.__version__}
try:
    import cv2
    out['cv2'] = cv2.__version__; out['cv2_file'] = cv2.__file__
    bi = cv2.getBuildInformation()
    out['kleidicv'] = any('KleidiCV' in l for l in bi.splitlines())
except Exception as e:
    out['cv2_error'] = repr(e)[:200]; print(json.dumps(out)); sys.exit(0)
try:
    __import__('hailo_platform'); out['hailo_platform'] = 'imports'
except Exception as e:
    out['hailo_platform'] = 'FAILS: ' + repr(e)[:160]
try:
    from mongla_vision import image_msg
    a = np.zeros((36, 64, 3), np.uint8); a[3, 5] = (1, 2, 3)
    back = image_msg.to_array(image_msg.to_msg(a, 'bgr8'), 'bgr8')
    out['image_msg'] = 'round-trips' if (back == a).all() else 'WRONG PIXELS'
except Exception as e:
    out['image_msg'] = 'FAILS: ' + repr(e)[:160]
rng = np.random.default_rng(0)
base = cv2.GaussianBlur(rng.integers(0, 255, (360, 640, 3), dtype=np.uint8), (0, 0), 3)
base = cv2.add(base, rng.integers(0, 40, base.shape, dtype=np.uint8))
jpg = cv2.imencode('.jpg', base, [cv2.IMWRITE_JPEG_QUALITY, 85])[1]
g0 = cv2.cvtColor(base, cv2.COLOR_BGR2GRAY); g1 = np.roll(g0, 3, axis=1)
p0 = cv2.goodFeaturesToTrack(g0, 200, 0.01, 8)
ops = {
    'decode':   lambda: cv2.imdecode(jpg, cv2.IMREAD_COLOR),
    'resize':   lambda: cv2.resize(base, (640, 640), interpolation=cv2.INTER_LINEAR),
    'bgr2rgb':  lambda: cv2.cvtColor(base, cv2.COLOR_BGR2RGB),
    'bgr2gray': lambda: cv2.cvtColor(base, cv2.COLOR_BGR2GRAY),
    'corners':  lambda: cv2.goodFeaturesToTrack(g0, 200, 0.01, 8),
    'lk':       lambda: cv2.calcOpticalFlowPyrLK(g0, g1, p0, None, winSize=(21, 21), maxLevel=3)[0],
}
tot = 0.0
for name, fn in ops.items():
    for _ in range(20): fn()
    ts = []
    for _ in range(400):
        t = time.perf_counter(); r = fn(); ts.append((time.perf_counter() - t) * 1e3)
    med = float(np.median(ts)); tot += med
    out[name] = round(med, 3)
    out[name + '_sum'] = round(float(np.asarray(r, dtype=np.float64).sum()), 1)
out['total_ms'] = round(tot, 3)
print(json.dumps(out))

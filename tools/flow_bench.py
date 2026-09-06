#!/usr/bin/env python3
"""flow_bench -- measure the bottom camera AS A DVL against a tape measure.

Runs the SHIPPING flow modules (`flow_math`, `flow_velocity`, `NavEstimator`)
on the real bottom camera and the real srot gyro, and reports the distance it
believes it travelled against a distance you measured with a tape.

⛔ IT DRIVES WHAT SHIPS. Not a parallel implementation of it -- round 33 lost a
finding to exactly that, when a test reimplemented the node's loop and passed
against a copy without the bug. If this bench is accurate and the vehicle is
not, the difference is wiring, not maths.

ONE PHASE PER INVOCATION, because an unattended protocol measures an untouched
rig. Each run prompts, counts down, records a window, and stops.

    python3 tools/flow_bench.py check                  # preflight, moves nothing
    python3 tools/flow_bench.py fwd   --truth 0.30
    python3 tools/flow_bench.py back  --truth 0.30
    python3 tools/flow_bench.py lat   --truth 0.30

HEIGHT IS LENS TO FLOOR and there is no default: velocity scales linearly with
it, so a guessed height is a clean multiplier on every number below. Pass
--height.

⛔ THE FRAME-RATE TRAP THIS BENCH EXISTS TO EXPOSE. `MIN_NET_FLOW_PX = 0.5` is
a per-interval PIXEL floor, so the SPEED it refuses depends on the frame rate:

    v_min = min_flow_px * h / (f * dt)

At 210 fps, h=0.72 m, f=514 that is 0.147 m/s -- the floor silently refuses
everything slower than 15 cm/s, which is most of station-keeping. At 30 fps the
same constant refuses 2 cm/s. Same code, same water, a 7x difference in what it
will not measure. `--decimate` sets the flow baseline so this is measurable
rather than theoretical: sweep it and watch the error.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from collections import deque

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)


def _load(name, relpath):
    """Load ONE shipping module by path, without its package __init__.

    `duburi_vision/__init__` reaches rclpy, so a plain import would make this
    bench require a sourced ROS environment to measure optical flow -- which it
    does not need: flow_math, flow_velocity and nav_estimator import numpy and
    nothing else. Loading the files directly keeps the bench runnable on any
    box while still exercising THE CODE THAT SHIPS, which is the property that
    matters. A reimplementation here would be worthless.
    """
    import importlib.util
    path = os.path.join(_ROOT, relpath)
    if not os.path.exists(path):
        raise SystemExit(f'cannot find {path} -- run from the workspace root')
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


_fm = _load('_fb_flow_math',
            'src/duburi_vision/duburi_vision/distance/flow_math.py')
_fv = _load('_fb_flow_velocity',
            'src/duburi_vision/duburi_vision/distance/flow_velocity.py')
_ne = _load('_fb_nav_estimator',
            'src/duburi_manager/duburi_manager/estimator/nav_estimator.py')

flow_dispersion = _fm.flow_dispersion
solve_planar_motion = _fm.solve_planar_motion
forward_backward_error = _fm.forward_backward_error
interp_rate = _fm.interp_rate
robust_flow = _fm.robust_flow
flow_velocity = _fv.flow_velocity
NavEstimator = _ne.NavEstimator

F_AIR_PX = 513.94          # 1027.87 at 1280 wide, /2 for 640. Round 25.
F_WATER_PX = 741.0

_FEATURE_PARAMS = dict(maxCorners=160, qualityLevel=0.01, minDistance=8,
                       blockSize=7)
_LK_PARAMS = dict(winSize=(21, 21), maxLevel=3,
                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                            30, 0.01))
_MIN_TRACKS = 6


class Gyro:
    """srot body rates, in a reader thread.

    ATTITUDE carries rollspeed/pitchspeed/yawspeed in rad/s directly, so it is
    preferred over SCALED_IMU2 (int16 milli-rad/s, and a scaling convention is
    one more thing to get silently wrong). Both are recorded so the bench can
    say whether they agree.
    """

    def __init__(self, port='/dev/ttyUSB0', baud=115200):
        from pymavlink import mavutil
        self._m = mavutil.mavlink_connection(port, baud=baud)
        self.buf = deque(maxlen=4096)       # (t, pitch_rate, roll_rate)
        self.yaw_deg = 0.0
        self.imu2 = deque(maxlen=4096)      # (t, gx, gy, gz) rad/s
        self.n_att = 0
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def _loop(self):
        while not self._stop.is_set():
            try:
                msg = self._m.recv_match(blocking=True, timeout=0.5)
            except Exception:
                continue
            if msg is None:
                continue
            now = time.monotonic()
            k = msg.get_type()
            if k == 'ATTITUDE':
                # flow_math.interp_rate wants (t, pitch_rate, roll_rate).
                self.buf.append((now, float(msg.pitchspeed),
                                 float(msg.rollspeed)))
                self.yaw_deg = math.degrees(float(msg.yaw))
                self.n_att += 1
            elif k == 'SCALED_IMU2':
                self.imu2.append((now, msg.xgyro * 1e-3, msg.ygyro * 1e-3,
                                  msg.zgyro * 1e-3))

    def rms(self, t0, t1):
        s = [(p, r) for (t, p, r) in self.buf if t0 <= t <= t1]
        if not s:
            return 0.0
        return float(np.sqrt(np.mean([p * p + r * r for p, r in s])))

    def close(self):
        self._stop.set()


def open_camera(dev, w, h, fps, fourcc='MJPG'):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f'cannot open {dev}')
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def frame_ok(gray):
    """The covered-lens guard, and it is not optional.

    A covered lens produced '11 px of flow' at 33 % LK survival on this bench
    and nearly became an algorithm fix. Brightness, contrast and corner count
    all have to be real before any number below means anything.
    """
    n = cv2.goodFeaturesToTrack(gray, 200, 0.01, 8)
    n = 0 if n is None else len(n)
    return (gray.mean() >= 15 and gray.std() >= 10 and n >= 30,
            f'mean={gray.mean():.1f} std={gray.std():.1f} corners={n}')


def run(args):
    f_px = F_WATER_PX if args.medium == 'water' else F_AIR_PX
    h = args.height
    print(f'\n=== flow_bench: {args.phase.upper()} ===')
    print(f'    camera {args.device}  {args.width}x{args.height_px} '
          f'MJPG@{args.fps}   decimate={args.decimate}')
    print(f'    medium={args.medium}  f={f_px:.2f}px   LENS-TO-FLOOR '
          f'h={h:.3f}m')
    if args.truth:
        print(f'    truth  = {args.truth * 100:.1f} cm  (tape)')
    v_min = args.min_flow * h / (f_px * (args.decimate / args.fps))
    print(f'    the {args.min_flow}px floor refuses below {v_min:.3f} m/s '
          f'at this baseline')

    gyro = None
    if not args.no_gyro:
        try:
            gyro = Gyro(args.port)
            time.sleep(1.0)
            if gyro.n_att == 0:
                print('!!! no ATTITUDE from the board -- rates unavailable')
        except Exception as exc:
            print(f'!!! gyro unavailable ({exc}); continuing WITHOUT '
                  f'de-rotation -- keep the rig FLAT or the numbers are void')

    cap = open_camera(args.device, args.width, args.height_px, args.fps)
    ok, _ = cap.read()
    ok2, g0 = cap.read()
    if not (ok and ok2):
        raise SystemExit('camera gave no frames')
    gray0 = cv2.cvtColor(g0, cv2.COLOR_BGR2GRAY)
    good, why = frame_ok(gray0)
    print(f'    frame check: {why}')
    if not good:
        raise SystemExit('FRAME CHECK FAILED -- lens covered, or no texture.')

    if args.phase == 'compare':
        compare(cap, args, f_px, h)
        cap.release()
        if gyro:
            gyro.close()
        return

    if args.phase == 'sweep':
        sweep(cap, args, f_px, h)
        cap.release()
        if gyro:
            gyro.close()
        return

    if args.phase == 'synth':
        synth(cap, args, f_px, h)
        cap.release()
        if gyro:
            gyro.close()
        return

    if args.phase == 'check':
        # Noise floor: hold still, and report what the pipeline reports.
        print('\n>>> HOLD THE RIG STILL. Measuring the noise floor for 5 s...')
        res = measure(cap, gyro, args, f_px, h, seconds=5.0)
        print(f'\n    STATIONARY over {res["dur"]:.1f}s:')
        print(f'      reported displacement : {res["dist"] * 100:+.2f} cm  '
              f'(should be ~0)')
        print(f'      path length           : {res["path"] * 100:.2f} cm')
        print(f'      intervals used/refused: {res["used"]}/{res["refused"]}')
        print(f'      median px/interval    : {res["px_med"]:.3f}')
        print(f'      gyro rms              : {res["gyro_rms"]:.4f} rad/s')
        print('\n    Preflight OK. Now run a phase: fwd / back / lat')
        cap.release()
        if gyro:
            gyro.close()
        return

    print(f'\n>>> GET READY. On GO, slide the rig {args.truth * 100:.0f} cm '
          f'{args.phase.upper()} at a steady, unhurried pace,')
    print('>>> then STOP and hold still until it says DONE.')
    for i in (3, 2, 1):
        print(f'    ... {i}')
        time.sleep(1.0)
    print(f'\n>>> GO  ({args.window:.0f} s window)\n')
    res = measure(cap, gyro, args, f_px, h, seconds=args.window, live=True)
    print('\n>>> DONE')

    d = res['dist']
    print(f'\n    measured  : {d * 100:+.2f} cm')
    if args.truth:
        err = abs(d) - args.truth
        print(f'    truth     : {args.truth * 100:.1f} cm')
        print(f'    ERROR     : {err * 100:+.2f} cm  '
              f'({100.0 * abs(d) / args.truth:.1f} % of truth)')
    print(f'    path      : {res["path"] * 100:.2f} cm   '
          f'(vs |displacement| {abs(d) * 100:.2f})')
    print(f'    intervals : {res["used"]} used / {res["refused"]} refused '
          f'({100.0 * res["refused"] / max(1, res["used"] + res["refused"]):.0f} %)')
    print(f'    px/interval median {res["px_med"]:.2f}  '
          f'p90 {res["px_p90"]:.2f}')
    print(f'    filter    : px={res["nav_px"] * 100:+.2f} '
          f'py={res["nav_py"] * 100:+.2f} cm  sigma={res["nav_sigma"] * 100:.2f} cm  '
          f'({res["nav_fixes"]} fixes, {res["nav_rejected"]} gated out)')
    print(f'    gyro rms  : {res["gyro_rms"]:.4f} rad/s')
    print(f'    frames    : {res["frames"]} captured   '
          f'median-fallback {res["fallback"]}')
    print(f'    yaw_img rms: {res["yaw_img_rms"]:.4f} rad/s  '
          f'(gyro {res["gyro_rms"]:.4f})')
    if res['reasons']:
        top = sorted(res['reasons'].items(), key=lambda kv: -kv[1])[:3]
        print('    top refusal reasons:')
        for r, n in top:
            print(f'      {n:5d}  {r}')
    cap.release()
    if gyro:
        gyro.close()





def compare(cap, args, f_px, h):
    """MEDIAN vs PLANAR RIGID FIT, with rotation present. Truth exact.

    The case that separates them. A median models translation ONLY, so every
    other component of the flow field has to leak somewhere -- and where it
    leaks depends on how the feature points happen to be distributed, which is
    a property of the floor, not of the motion. That is the dangerous shape: it
    works on a bench with even texture and biases in a pool with a bright patch
    in one corner.

    Truth here is the transform we applied, so both estimators are scored
    against the same exact number rather than against each other.
    """
    ok, frame = cap.read()
    for _ in range(5):
        ok, frame = cap.read()
    base = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    H, W = base.shape[:2]
    cx, cy = W / 2.0, H / 2.0
    fdt = args.decimate / args.fps

    print(f'\n    MEDIAN vs PLANAR RIGID FIT  ({W}x{H}, dt={fdt * 1000:.1f} ms)')
    print(f'    truth is the applied transform, exact\n')
    hdr = (f'    {"case":<34} {"median dx,dy":>16} {"planar dx,dy":>16} '
           f'{"med err":>9} {"pln err":>9} {"yaw meas":>10} {"yaw true":>9}')
    print(hdr)
    print('    ' + '-' * (len(hdr) - 4))

    cases = [
        ('pure translation 6px', 6.0, 0.0),
        ('translation 6px + rot 0.5deg', 6.0, 0.5),
        ('translation 6px + rot 1.5deg', 6.0, 1.5),
        ('translation 6px + rot 3.0deg', 6.0, 3.0),
        ('pure rotation 1.5deg', 0.0, 1.5),
    ]
    for label, tpx, rot_deg in cases:
        M = cv2.getRotationMatrix2D((cx, cy), rot_deg, 1.0)
        M[1, 2] += tpx                       # translate along image +y
        img = cv2.warpAffine(base, M, (W, H), flags=cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_REFLECT)
        pts = cv2.goodFeaturesToTrack(base, mask=None, **_FEATURE_PARAMS)
        nxt, status, _ = cv2.calcOpticalFlowPyrLK(base, img, pts, None,
                                                  **_LK_PARAMS)
        # Forward-backward rejection before either estimator sees the points.
        if args.fb_px > 0:
            fb = forward_backward_error(base, img, pts, nxt, _LK_PARAMS)
            if fb is not None and status is not None:
                st = np.asarray(status).reshape(-1).astype(bool)
                st &= (fb <= args.fb_px)
                status = st.astype(np.uint8).reshape(-1, 1)

        med = robust_flow(pts, nxt, status, min_tracks=_MIN_TRACKS)
        pm = solve_planar_motion(pts, nxt, status, fdt, cx=cx, cy=cy,
                                 ransac_px=args.ransac_px)
        # TRUTH: where the optical axis actually went under this transform.
        c = np.array([cx, cy, 1.0])
        moved = M @ c
        tdx, tdy = float(moved[0] - cx), float(moved[1] - cy)
        true_yaw = math.radians(rot_deg) / fdt

        m_err = (math.hypot(med[0] - tdx, med[1] - tdy)
                 if med else float('nan'))
        p_err = (math.hypot(pm.dx_px - tdx, pm.dy_px - tdy)
                 if pm.ok else float('nan'))
        ms = f'{med[0]:+6.2f},{med[1]:+6.2f}' if med else '     --'
        ps = f'{pm.dx_px:+6.2f},{pm.dy_px:+6.2f}' if pm.ok else '     --'
        ys = f'{pm.yaw_rate:+8.3f}' if pm.ok else '      --'
        print(f'    {label:<34} {ms:>16} {ps:>16} '
              f'{m_err:>9.3f} {p_err:>9.3f} {ys:>10} {true_yaw:>9.3f}')

    print(f'\n    err = |estimated - true| translation of the optical axis, px.')
    print(f'    yaw in rad/s; the planar fit measures it, the median cannot.')
    print(f'    inliers on the last case: {pm.n_inliers}/{pm.n_points}, '
          f'residual {pm.residual_px:.3f} px')


def sweep(cap, args, f_px, h):
    """Where does it stop measuring? Speed x flow-baseline, exact truth.

    This is the question that decides whether the bottom camera is a DVL or a
    transit-only odometer. `MIN_NET_FLOW_PX` is a per-interval PIXEL floor, so
    the SPEED it refuses scales with the frame rate:

        v_min = min_flow_px * h / (f * dt)

    Station-keeping is the AUV's most common state and its slowest, so a floor
    that silently refuses slow motion refuses exactly the regime the sensor
    exists for -- and it does it by returning NO measurement, which reads as
    "not moving" to anything integrating.
    """
    ok, frame = cap.read()
    for _ in range(5):
        ok, frame = cap.read()
    base = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    truth_m = args.truth

    speeds = [0.02, 0.05, 0.10, 0.20, 0.40, 0.80]
    decs = [int(x) for x in args.sweep_decimate.split(',')]

    print(f'\n    SWEEP -- recovery of a {truth_m * 100:.0f} cm synthetic '
          f'translation, truth exact')
    print(f'    h={h:.3f}m f={f_px:.2f}px   capture {args.fps} fps\n')
    print('      speed |' + ''.join(f'  dec={d:<3d}(={args.fps // d:>3d}Hz)'
                                    for d in decs)
          + f'  || ADAPTIVE({args.target_px:.0f}px @{args.fps}Hz)')
    print('     -------+' + '-' * (18 * len(decs) + 26))
    for v_ms in speeds:
        row = f'    {v_ms:5.2f} |'
        for dec in decs:
            dt = dec / args.fps
            step_px = f_px * v_ms * dt / h
            n = max(2, int(round(truth_m / (v_ms * dt))))
            got, used, ref = _synth_run(base, args, f_px, h, step_px, n, dt)
            pct = 100.0 * got / truth_m
            flag = ' ' if 97.0 <= pct <= 103.0 else '!'
            row += f'  {pct:6.1f}%{flag}{used:>4}/{used + ref:<4}'
        # ADAPTIVE: always at native capture rate, re-anchoring on displacement.
        fdt = 1.0 / args.fps
        spx = f_px * v_ms * fdt / h
        nn = max(2, int(round(truth_m / (v_ms * fdt))))
        g2, u2, r2 = _synth_adaptive(base, args, f_px, h, spx, nn, fdt,
                                     target_px=args.target_px)
        p2 = 100.0 * g2 / truth_m
        f2 = ' ' if 97.0 <= p2 <= 103.0 else '!'
        row += f'  || {p2:6.1f}%{f2}{u2:>4}/{u2 + r2:<4}'
        print(row)
    print(f'\n    "!" = outside +-3 %.  used/total intervals after the gates.')
    print(f'    v_min from the {args.min_flow}px floor:')
    for dec in decs:
        dt = dec / args.fps
        print(f'      dec={dec:<3d} ({args.fps // dec:>3d} Hz): '
              f'refuses below {args.min_flow * h / (f_px * dt):.3f} m/s')



def _synth_adaptive(base, args, f_px, h, step_px, n, frame_dt,
                    target_px=8.0, max_px=25.0):
    """ADAPTIVE KEYFRAME BASELINE -- the answer to the fixed-rate floor.

    ⛔ WHY THE FIXED BASELINE FAILS. `MIN_NET_FLOW_PX` is a per-interval pixel
    floor, so the SPEED it refuses is `min_px * h / (f * dt)` -- it scales with
    the frame rate. Measured on this camera at h=0.72: at 210 Hz it refuses
    everything below 0.147 m/s, and it refuses by returning NO measurement,
    which reads downstream as "not moving". A 30 cm move at 5 cm/s produced
    0 of 1260 usable intervals and reported 0.0 cm.

    THE FIX IS NOT A SMALLER CONSTANT. Lower the floor and noise integrates
    into drift; raise the rate and slow motion vanishes. Both are symptoms of
    measuring over a FIXED time when the thing that matters is DISPLACEMENT.

    So: track against an ANCHOR frame and emit a measurement only when the
    accumulated shift is worth measuring (`target_px`), then re-anchor. Every
    emitted interval then carries the same ~8 px of signal against the same
    ~0.06 px of noise, at ANY speed -- slow motion simply integrates longer
    before it reports. `max_px` re-anchors early so LK never has to match
    across more displacement than its window can follow.

    This is the standard VO keyframe trade and it also removes a second error
    source: within a baseline the flow is measured frame-to-ANCHOR, so it does
    not accumulate the per-frame noise that frame-to-frame chaining does.
    """
    anchor = anchor_pts = None
    anchor_t = 0.0
    dist = 0.0
    used = emitted = 0
    t = 0.0
    for i in range(n + 1):
        M = np.float32([[1, 0, 0], [0, 1, i * step_px]])
        img = cv2.warpAffine(base, M, (base.shape[1], base.shape[0]),
                             flags=cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_REFLECT)
        t = i * frame_dt
        if anchor is None:
            anchor, anchor_t = img, t
            anchor_pts = cv2.goodFeaturesToTrack(img, mask=None,
                                                 **_FEATURE_PARAMS)
            continue
        nxt, status, _ = cv2.calcOpticalFlowPyrLK(anchor, img, anchor_pts,
                                                  None, **_LK_PARAMS)
        flow = robust_flow(anchor_pts, nxt, status, min_tracks=_MIN_TRACKS)
        if flow is None:
            anchor, anchor_t = img, t
            anchor_pts = cv2.goodFeaturesToTrack(img, mask=None,
                                                 **_FEATURE_PARAMS)
            continue
        mag = math.hypot(flow[0], flow[1])
        n_ok = int(np.asarray(status).reshape(-1).astype(bool).sum())
        # Emit when there is enough signal, or before LK loses the anchor.
        if mag < target_px and mag < max_px and n_ok >= _MIN_TRACKS \
                and i < n:
            continue
        dt = t - anchor_t
        disp = flow_dispersion(anchor_pts, nxt, status)
        v = flow_velocity(flow[0], flow[1], dt, f_px=f_px, height_m=h,
                          dispersion_px=disp, min_net_flow_px=args.min_flow,
                          rot_fraction_max=args.rot_max,
                          max_dispersion_ratio=args.max_disp)
        emitted += 1
        if v.ok:
            used += 1
            dist += math.hypot(v.vx, v.vy) * dt
        anchor, anchor_t = img, t
        anchor_pts = cv2.goodFeaturesToTrack(img, mask=None, **_FEATURE_PARAMS)
    return dist, used, emitted - used


def _synth_run(base, args, f_px, h, step_px, n, dt):
    """One synthetic translation. Returns (metres, used, refused)."""
    prev = prev_pts = None
    dist = 0.0
    used = refused = 0
    for i in range(n + 1):
        M = np.float32([[1, 0, 0], [0, 1, i * step_px]])
        img = cv2.warpAffine(base, M, (base.shape[1], base.shape[0]),
                             flags=cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_REFLECT)
        if prev is None:
            prev = img
            prev_pts = cv2.goodFeaturesToTrack(img, mask=None,
                                               **_FEATURE_PARAMS)
            continue
        nxt, status, _ = cv2.calcOpticalFlowPyrLK(prev, img, prev_pts, None,
                                                  **_LK_PARAMS)
        flow = robust_flow(prev_pts, nxt, status, min_tracks=_MIN_TRACKS)
        disp = flow_dispersion(prev_pts, nxt, status)
        if nxt is not None and status is not None:
            st = np.asarray(status).reshape(-1).astype(bool)
            prev_pts = (nxt[st].reshape(-1, 1, 2) if st.sum() >= _MIN_TRACKS
                        else cv2.goodFeaturesToTrack(img, mask=None,
                                                     **_FEATURE_PARAMS))
        prev = img
        if flow is None:
            refused += 1
            continue
        v = flow_velocity(flow[0], flow[1], dt, f_px=f_px, height_m=h,
                          dispersion_px=disp, min_net_flow_px=args.min_flow,
                          rot_fraction_max=args.rot_max,
                          max_dispersion_ratio=args.max_disp)
        if not v.ok:
            refused += 1
            continue
        used += 1
        dist += math.hypot(v.vx, v.vy) * dt
    return dist, used, refused


def synth(cap, args, f_px, h):
    """Recover a KNOWN displacement from real texture. Truth is exact.

    ⛔ VALIDATE THE INSTRUMENT BEFORE IT MEASURES. A real slide couples four
    unknowns at once -- how far the rig actually went, how flat it stayed, what
    the height really is, and whether the maths is right. Digitally translating
    a REAL frame by a KNOWN pixel count removes three of them, so if this comes
    back wrong the scale chain is wrong and no amount of sliding will fix it.

    A real frame, not a synthetic pattern: LK's behaviour depends on the
    texture it is given, and a checkerboard would flatter it.
    """
    ok, frame = cap.read()
    for _ in range(5):
        ok, frame = cap.read()
    if not ok:
        raise SystemExit('no frame to translate')
    base = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    truth_m = args.truth
    total_px = f_px * truth_m / h
    n = args.synth_steps
    step_px = total_px / n
    dt = args.decimate / args.fps

    print(f'\n    SYNTHETIC TRANSLATION -- truth is exact')
    print(f'      {truth_m * 100:.1f} cm at h={h:.3f} f={f_px:.2f} '
          f'= {total_px:.2f} px total')
    print(f'      {n} steps of {step_px:.3f} px, dt={dt * 1000:.2f} ms '
          f'=> {step_px * h / (f_px * dt):.3f} m/s')

    prev_pts = None
    prev = None
    dist_x = dist_y = 0.0
    used = refused = 0
    reasons = {}
    for i in range(n + 1):
        # Shift along image +y (which flow_velocity maps to body x/forward).
        shift = i * step_px
        M = np.float32([[1, 0, 0], [0, 1, shift]])
        img = cv2.warpAffine(base, M, (base.shape[1], base.shape[0]),
                             flags=cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_REFLECT)
        if prev is None:
            prev = img
            prev_pts = cv2.goodFeaturesToTrack(img, mask=None,
                                               **_FEATURE_PARAMS)
            continue
        nxt, status, _ = cv2.calcOpticalFlowPyrLK(prev, img, prev_pts, None,
                                                  **_LK_PARAMS)
        flow = robust_flow(prev_pts, nxt, status, min_tracks=_MIN_TRACKS)
        disp = flow_dispersion(prev_pts, nxt, status)
        if nxt is not None and status is not None:
            st = np.asarray(status).reshape(-1).astype(bool)
            prev_pts = (nxt[st].reshape(-1, 1, 2) if st.sum() >= _MIN_TRACKS
                        else cv2.goodFeaturesToTrack(img, mask=None,
                                                     **_FEATURE_PARAMS))
        prev = img
        if flow is None:
            refused += 1
            continue
        v = flow_velocity(flow[0], flow[1], dt, f_px=f_px, height_m=h,
                          pitch_rate=0.0, roll_rate=0.0, dispersion_px=disp,
                          min_net_flow_px=args.min_flow,
                          rot_fraction_max=args.rot_max,
                          max_dispersion_ratio=args.max_disp)
        if not v.ok:
            refused += 1
            key = v.reason.split('(')[0].strip()
            reasons[key] = reasons.get(key, 0) + 1
            continue
        used += 1
        dist_x += v.vx * dt
        dist_y += v.vy * dt

    got = math.hypot(dist_x, dist_y)
    print(f'\n      recovered : {got * 100:+.3f} cm   '
          f'(x={dist_x * 100:+.3f} y={dist_y * 100:+.3f})')
    print(f'      truth     : {truth_m * 100:.3f} cm')
    print(f'      ERROR     : {(got - truth_m) * 100:+.3f} cm  '
          f'({100.0 * got / truth_m:.2f} % of truth)')
    print(f'      intervals : {used} used / {refused} refused')
    if reasons:
        for r, c in sorted(reasons.items(), key=lambda kv: -kv[1])[:3]:
            print(f'        {c:4d}  {r}')


def measure(cap, gyro, args, f_px, h, seconds, live=False):
    """The LIVE path, running the same algorithm flow_node ships.

    Adaptive keyframe baseline + forward-backward rejection + planar rigid fit
    + de-rotation from the mean gyro rate over the baseline. If this and the
    vehicle disagree, the difference is wiring, not maths.
    """
    nav = NavEstimator()
    anchor = anchor_pts = None
    anchor_t = None
    used = refused = frames = 0
    fallback = 0
    reasons = {}
    px = []
    dist_x = dist_y = 0.0
    path = 0.0
    yaw_img_hist = []
    t_start = time.monotonic()
    last_report = t_start

    while time.monotonic() - t_start < seconds:
        ok, frame = cap.read()
        if not ok:
            continue
        frames += 1
        t = time.monotonic()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        H, W = gray.shape[:2]

        if anchor is None or anchor_pts is None or len(anchor_pts) < _MIN_TRACKS:
            anchor, anchor_t = gray, t
            anchor_pts = cv2.goodFeaturesToTrack(gray, mask=None,
                                                 **_FEATURE_PARAMS)
            continue

        nxt, status, _ = cv2.calcOpticalFlowPyrLK(anchor, gray, anchor_pts,
                                                  None, **_LK_PARAMS)
        if args.fb_px > 0 and nxt is not None and status is not None:
            fb = forward_backward_error(anchor, gray, anchor_pts, nxt,
                                        _LK_PARAMS)
            if fb is not None:
                st = np.asarray(status).reshape(-1).astype(bool)
                st &= (fb <= args.fb_px)
                status = st.astype(np.uint8).reshape(-1, 1)

        flow = robust_flow(anchor_pts, nxt, status, min_tracks=_MIN_TRACKS)
        if flow is None:
            anchor, anchor_t = gray, t
            anchor_pts = cv2.goodFeaturesToTrack(gray, mask=None,
                                                 **_FEATURE_PARAMS)
            refused += 1
            reasons['LK lost the anchor'] = reasons.get('LK lost the anchor', 0) + 1
            continue

        n_ok = int(np.asarray(status).reshape(-1).astype(bool).sum())
        mag = math.hypot(flow[0], flow[1])
        dt = t - anchor_t
        if (mag < args.target_px and n_ok >= _MIN_TRACKS
                and dt < args.max_baseline):
            continue

        disp = flow_dispersion(anchor_pts, nxt, status)
        yaw_img = 0.0
        if not args.no_planar:
            pm = solve_planar_motion(anchor_pts, nxt, status, dt,
                                     cx=W / 2.0, cy=H / 2.0,
                                     ransac_px=args.ransac_px)
            if pm.ok:
                flow = (pm.dx_px, pm.dy_px)
                n_ok = pm.n_inliers
                disp = pm.residual_px
                yaw_img = -pm.yaw_rate
            else:
                fallback += 1
        px.append(math.hypot(flow[0], flow[1]))

        pr = rr = 0.0
        if gyro is not None and gyro.buf:
            r = _fm.integrate_rate(list(gyro.buf), t - dt, t)
            if r:
                pr, rr = r
        pr *= -args.gain_y
        rr *= -args.gain_x

        anchor, anchor_t = gray, t
        anchor_pts = cv2.goodFeaturesToTrack(gray, mask=None, **_FEATURE_PARAMS)

        v = flow_velocity(flow[0], flow[1], dt, f_px=f_px, height_m=h,
                          pitch_rate=pr, roll_rate=rr, dispersion_px=disp,
                          min_net_flow_px=args.min_flow,
                          rot_fraction_max=args.rot_max,
                          max_dispersion_ratio=args.max_disp)
        if not v.ok:
            refused += 1
            key = v.reason.split('(')[0].strip()
            reasons[key] = reasons.get(key, 0) + 1
            continue

        used += 1
        yaw_img_hist.append(yaw_img)
        sigma = max(1e-3, (h / (f_px * dt)) * (disp if disp else 1.0)
                    / math.sqrt(max(1, n_ok)))
        yaw = math.radians(gyro.yaw_deg) if gyro else 0.0
        nav.predict(t, yaw)
        nav.update_velocity(v.vx, v.vy, sigma)
        dist_x += v.vx * dt
        dist_y += v.vy * dt
        path += math.hypot(v.vx, v.vy) * dt

        if live and time.monotonic() - last_report >= 1.0:
            last_report = time.monotonic()
            print(f'    t={time.monotonic() - t_start:4.1f}s  '
                  f'x={dist_x * 100:+7.2f}cm y={dist_y * 100:+7.2f}cm  '
                  f'px={px[-1]:5.2f} pts={n_ok:3d}  used={used}')

    dur = time.monotonic() - t_start
    axis = dist_y if args.phase == 'lat' else dist_x
    st = nav.state()
    return dict(dist=axis, dist_x=dist_x, dist_y=dist_y, path=path,
                nav_px=st.px, nav_py=st.py, nav_sigma=st.pos_sigma,
                nav_fixes=st.n_fixes, nav_rejected=st.n_rejected,
                used=used, refused=refused, reasons=reasons, dur=dur,
                frames=frames, skipped=0, fallback=fallback,
                px_med=float(np.median(px)) if px else 0.0,
                px_p90=float(np.percentile(px, 90)) if px else 0.0,
                yaw_img_rms=float(np.sqrt(np.mean(np.square(yaw_img_hist))))
                if yaw_img_hist else 0.0,
                gyro_rms=gyro.rms(t_start, time.monotonic()) if gyro else 0.0)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('phase', choices=('check', 'synth', 'sweep', 'compare', 'fwd', 'back', 'lat'))
    p.add_argument('--truth', type=float, default=0.30, help='metres (tape)')
    p.add_argument('--height', type=float, required=True,
                   help='LENS to floor, metres. No default: it is a clean '
                        'multiplier on every number.')
    p.add_argument('--device', default='/dev/duburi_cam_downward')
    p.add_argument('--width', type=int, default=640)
    p.add_argument('--height-px', type=int, default=360)
    p.add_argument('--fps', type=int, default=210)
    p.add_argument('--decimate', type=int, default=7,
                   help='process every Nth frame; sets the flow baseline')
    p.add_argument('--window', type=float, default=8.0)
    p.add_argument('--medium', choices=('air', 'water'), default='air')
    p.add_argument('--port', default='/dev/ttyUSB0')
    p.add_argument('--no-gyro', action='store_true')
    p.add_argument('--gain-x', type=float, default=1.0)
    p.add_argument('--gain-y', type=float, default=1.0)
    p.add_argument('--min-flow', type=float, default=0.5)
    p.add_argument('--rot-max', type=float, default=0.80)
    p.add_argument('--max-disp', type=float, default=5.0)
    p.add_argument('--synth-steps', type=int, default=60)
    p.add_argument('--sweep-decimate', default='1,3,7,14')
    p.add_argument('--target-px', type=float, default=8.0)
    p.add_argument('--ransac-px', type=float, default=2.0)
    p.add_argument('--no-planar', action='store_true')
    p.add_argument('--max-baseline', type=float, default=0.75)
    p.add_argument('--fb-px', type=float, default=1.0,
                   help='forward-backward reject threshold px; 0=off')
    run(p.parse_args())


if __name__ == '__main__':
    main()

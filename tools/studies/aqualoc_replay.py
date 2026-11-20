#!/usr/bin/env python3
"""Replay real underwater data through the shipped filter and flow chain.

Why this exists. Every localization number we have comes from the bench, a
tank slide, or synthetic tests; the vehicle has never been in water under this
stack. AQUALOC (Ferrera et al., IJRR 2019) is the closest public match to our
sensor set: a DOWNWARD monocular camera, a 200 Hz MEMS IMU and a pressure
sensor, with Kalibr extrinsics and a COLMAP trajectory rescaled to the depth
readings. Running it through `RIEKF` and the flow chain as shipped
(`detect_corners`, `forward_backward_error`, `solve_planar_motion`,
`flow_dispersion`, `flow_velocity`) tests the chain on water it was not tuned
on. Bead 5ag.8; the bar is written there and in the run folder before a run.

What it cannot test. AQUALOC has no altimeter, and `flow_velocity` refuses
without height. So ONE constant height is fitted on the calibration sequence
(01) only and applied unchanged to the others; a sequence flown at another
altitude reads as a scale error, and that is reported, not hidden. It is a
harbour seabed, not a pool: tiles, lane lines and props are not here.

Frames. AQUALOC's IMU has +y down (it reads -9.6 m/s^2 on y at rest), and the
camera's optical axis lies along it. Body is FRD: x_b = x_imu, y_b = -z_imu,
z_b = y_imu. Image to camera velocity, for a camera at height h over a plane:

    du = -f (v_x / h + w_y) dt        dv = -f (v_y / h - w_x) dt

which is `flow_velocity` with dx = -du, dy = -dv, roll_rate = w_y and
pitch_rate = -w_x, its vy then being camera v_x and its vx camera v_y. The
self-test checks that mapping on a synthetic floor with a known motion.

Metrics need no world alignment: displacement and yaw are expressed in the
FIRST camera frame, for both the filter and COLMAP, so an arbitrary
reconstruction frame cancels.

Result, 2026-11-21 (bar in bead 5ag.8, written before the run). Height fitted on
01: 1.52 m. Final horizontal error over sequences 02-07 (path 11.7-68.9 m):
IMU+depth alone diverges to 6.6-151 km, since depth is refused while velocity is
unobserved (B-56); with the shipped flow chain 5.8-29.5 m, 24-104 % of the path;
with the image-rotation yaw variant 1.4-15.4 m, 5-49 %. Position criterion
passed on 6 of 6; yaw failed on all six (75-177 deg), as predicted: nothing in
the filter observes heading against a gyro drifting about 1.5 deg/s. The variant
helped on 6 of 6 but halved the error on only 3, short of its bar. Self-fitted,
06 reads 0.60 m, against 1.52 m on 01: height does not carry between runs.

**Warning: the position "pass" says almost nothing about accuracy.** Its
baseline drifts kilometres, so nearly any aid clears a 50 % cut. The number
that matters is the absolute error, 24-104 % of the path, against the 2-4 %
the best published DVL-free systems reach (localization research report). The
conditions are harsher than the vehicle's (harbour floor, a fitted rather than
measured height, a poor MEMS gyro, no heading source), so this is not a
prediction of pool accuracy either way. It shows the chain runs on real water
and that heading and height are its limits; the in-water test against a
measured floor target (bead 5ag.1) still decides whether localization works.

    python3 tools/studies/aqualoc_replay.py --self-test
    python3 tools/studies/aqualoc_replay.py --root $MONGLA_DATA/datasets/aqualoc/harbor
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import pathlib
import sys

import numpy as np

_REPO = pathlib.Path(__file__).resolve().parents[2]
for _p in ('src/mongla_localization', 'src/mongla_vision'):
    sys.path.insert(0, str(_REPO / _p))

from mongla_localization.inekf import RIEKF, State  # noqa: E402
from mongla_vision.flow.flow_features import (  # noqa: E402
    detect_corners, forward_backward_error)
from mongla_vision.flow.flow_math import (  # noqa: E402
    flow_dispersion, solve_planar_motion)
from mongla_vision.flow.flow_velocity import flow_velocity  # noqa: E402

# Body FRD from AQUALOC's IMU axes (see the docstring).
R_BI = np.array([[1.0, 0.0, 0.0],
                 [0.0, 0.0, -1.0],
                 [0.0, 1.0, 0.0]])
LK = dict(winSize=(21, 21), maxLevel=3,
          criteria=(3, 30, 0.01))       # cv2.TERM_CRITERIA_EPS | COUNT
FB_MAX_PX = 1.0
FLOW_SIGMA = 0.05                       # localization_node flow_sigma default
DEPTH_SIGMA = 0.02                      # localization_node depth_sigma default


def camera_velocity(du, dv, dt, *, f, fy, h, w_cam, dispersion=None):
    """Camera-frame (v_x, v_y) from an image shift, through `flow_velocity`."""
    fv = flow_velocity(-du, -dv, dt, f_px=f, fy_px=fy, height_m=h,
                       roll_rate=float(w_cam[1]), pitch_rate=-float(w_cam[0]),
                       dispersion_px=dispersion)
    if not fv.ok:
        return None, fv.reason
    return np.array([fv.vy, fv.vx]), ''


# ── data ─────────────────────────────────────────────────────────────────────
def _rows(path):
    with open(path) as fh:
        return [r for r in csv.reader(fh) if r and not r[0].startswith('#')]


def load_sequence(root: pathlib.Path, n: int) -> dict:
    import yaml
    raw = root / f'seq_{n:02d}' / 'raw_data'
    imu = np.array([[float(x) for x in r] for r in _rows(raw / f'harbor_imu_sequence_{n:02d}.csv')])
    depth = np.array([[float(x) for x in r] for r in _rows(raw / f'harbor_depth_sequence_{n:02d}.csv')])
    imgs = [(int(r[0]), r[1].strip()) for r in _rows(raw / f'harbor_img_sequence_{n:02d}.csv')]
    gt = np.loadtxt(root / 'groundtruth' / f'new_harbor_colmap_traj_sequence_{n:02d}.txt')
    cal = yaml.safe_load((root / 'calibration' / 'harbor_imu_camera_calib.yaml').read_text())['cam0']
    cam = yaml.safe_load((root / 'calibration' / 'harbor_camera_calib.yaml').read_text())['cam0']
    shift = float(cal.get('timeshift_cam_imu', 0.0))
    t0 = imu[0, 0]
    # Kalibr: t_imu = t_cam + timeshift_cam_imu. Everything on the IMU clock, seconds.
    return {
        'imu_t': (imu[:, 0] - t0) * 1e-9, 'gyro': imu[:, 1:4], 'accel': imu[:, 4:7],
        'depth_t': (depth[:, 0] - t0) * 1e-9, 'depth': depth[:, 1],
        'img_t': np.array([(t - t0) * 1e-9 + shift for t, _ in imgs]),
        'img_path': [raw / f'harbor_images_sequence_{n:02d}' / p for _, p in imgs],
        'gt': gt,
        'T_imu_cam': np.array(cal['T_imu_cam'], dtype=float),
        'K': cam['intrinsics'], 'dist': np.array(cam['distortion_coeffs'], dtype=float),
    }


def gt_track(seq):
    """COLMAP poses -> (t, p_world, R_world_cam) at the GT frames."""
    gt = seq['gt']
    idx = gt[:, 0].astype(int)
    t = seq['img_t'][idx]
    p = gt[:, 1:4]
    q = gt[:, 4:8]                      # qx qy qz qw
    R = np.array([_quat_to_R(x) for x in q])
    return t, p, R


def _quat_to_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# ── flow on real frames ──────────────────────────────────────────────────────
def frame_flows(seq, i_from, i_to):
    """Per image pair: (t_mid, dt, du, dv, dispersion, image yaw rate) at the principal point,
    undistorted, from the shipped corner/LK/RANSAC chain. None where it refuses."""
    import cv2
    fx, fy, cx, cy = seq['K']
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    D = seq['dist'].reshape(4, 1)
    out = []
    prev = None
    for i in range(i_from, i_to):
        g = cv2.imread(str(seq['img_path'][i]), cv2.IMREAD_GRAYSCALE)
        if g is None:
            prev = None
            continue
        if prev is not None:
            pg, pt = prev
            p0 = detect_corners(pg)
            dt = seq['img_t'][i] - pt
            rec = None
            if p0 is not None and len(p0) >= 8 and dt > 0:
                p1, st, _ = cv2.calcOpticalFlowPyrLK(pg, g, p0, None, **LK)
                fb = forward_backward_error(pg, g, p0, p1, LK)
                ok = st.reshape(-1).astype(bool)
                if fb is not None:
                    ok &= np.asarray(fb).reshape(-1) < FB_MAX_PX
                if ok.sum() >= 8:
                    u0 = cv2.fisheye.undistortPoints(p0[ok].reshape(-1, 1, 2), K, D, P=K)
                    u1 = cv2.fisheye.undistortPoints(p1[ok].reshape(-1, 1, 2), K, D, P=K)
                    m = solve_planar_motion(u0, u1, None, dt, cx=cx, cy=cy)
                    if m.ok:
                        disp = flow_dispersion(u0, u1, np.ones(len(u0)))
                        rec = (pt + dt / 2, dt, m.dx_px, m.dy_px, disp, m.yaw_rate)
            out.append(rec)
        prev = (g, seq['img_t'][i])
    return out


def mean_gyro(seq, t0, t1):
    k = (seq['imu_t'] >= t0) & (seq['imu_t'] <= t1)
    return seq['gyro'][k].mean(axis=0) if k.any() else np.zeros(3)


# ── height, fitted on the calibration sequence only ──────────────────────────
def fit_height(seq, flows) -> float:
    """Median of |v_gt| / |v_flow at h=1 m| over intervals where both exist."""
    t, p, R = gt_track(seq)
    R_ic = seq['T_imu_cam'][:3, :3]
    fx, fy = seq['K'][0], seq['K'][1]
    ratios = []
    for rec in flows:
        if rec is None:
            continue
        tm, dt, du, dv, disp, _ = rec
        j = np.searchsorted(t, tm)
        if j <= 0 or j >= len(t):
            continue
        v_w = (p[j] - p[j - 1]) / (t[j] - t[j - 1])
        v_c = R[j - 1].T @ v_w
        w_c = R_ic.T @ mean_gyro(seq, tm - dt / 2, tm + dt / 2)
        v1, _ = camera_velocity(du, dv, dt, f=fx, fy=fy, h=1.0, w_cam=w_c, dispersion=disp)
        if v1 is None or np.linalg.norm(v1) < 1e-3:
            continue
        ratios.append(np.linalg.norm(v_c[:2]) / np.linalg.norm(v1))
    return float(np.median(ratios)) if ratios else float('nan')


# ── the filter run ───────────────────────────────────────────────────────────
def tilt_from_accel(a_b):
    """FRD attitude, yaw 0, from specific force at rest: R^T (0,0,-g) = a."""
    ax, ay, az = a_b / np.linalg.norm(a_b)
    roll = math.atan2(-ay, -az)
    pitch = math.atan2(ax, math.hypot(ay, az))
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Ry @ Rx


def run_filter(seq, flows, height, use_flow: bool, image_yaw: bool = False) -> dict:
    """`image_yaw`: also feed the heading integrated from the image rotation
    (`solve_planar_motion`'s yaw rate; the scene turns against the camera, so
    body yaw = -image rotation) through `update_yaw`. A study variant, not a
    shipped path: it stands in for the board's AHRS heading, which AQUALOC
    lacks, using a sensor that fails differently from the gyro."""
    t_gt, p_gt, R_gt = gt_track(seq)
    t_start, t_end = t_gt[0], t_gt[-1]
    R_ic = seq['T_imu_cam'][:3, :3]
    r_cam_b = R_BI @ seq['T_imu_cam'][:3, 3]          # camera position in body
    R_bc = R_BI @ R_ic
    k0 = np.searchsorted(seq['imu_t'], t_start)
    a0 = R_BI @ seq['accel'][max(0, k0 - 20):k0 + 20].mean(axis=0)
    f = RIEKF(state=State(R=tilt_from_accel(a0)))
    fx, fy = seq['K'][0], seq['K'][1]
    flow_q = [r for r in flows if r is not None and t_start <= r[0] <= t_end]
    fi, di = 0, int(np.searchsorted(seq['depth_t'], t_start))
    track = []
    n = {'flow': 0, 'flow_refused': 0, 'depth': 0, 'depth_refused': 0,
         'yaw': 0, 'yaw_refused': 0}
    img_yaw = 0.0
    t_prev = seq['imu_t'][k0]
    for k in range(k0 + 1, len(seq['imu_t'])):
        tk = seq['imu_t'][k]
        if tk > t_end:
            break
        f.predict(R_BI @ seq['gyro'][k], R_BI @ seq['accel'][k], tk - t_prev)
        t_prev = tk
        while di < len(seq['depth_t']) and seq['depth_t'][di] <= tk:
            ok = f.update_depth(-seq['depth'][di], sigma=DEPTH_SIGMA)
            n['depth' if ok else 'depth_refused'] += 1
            di += 1
        while use_flow and fi < len(flow_q) and flow_q[fi][0] <= tk:
            tm, dt, du, dv, disp, yaw_rate = flow_q[fi]
            fi += 1
            img_yaw -= yaw_rate * dt
            if image_yaw:
                ok = f.update_yaw(math.degrees(img_yaw), sigma_deg=2.0)
                n['yaw' if ok else 'yaw_refused'] += 1
            w_c = R_ic.T @ mean_gyro(seq, tm - dt / 2, tm + dt / 2)
            v_c, why = camera_velocity(du, dv, dt, f=fx, fy=fy, h=height, w_cam=w_c,
                                       dispersion=disp)
            if v_c is None:
                n['flow_refused'] += 1
                continue
            v_b = R_bc @ np.array([v_c[0], v_c[1], 0.0])
            ok = f.update_body_velocity_xy(float(v_b[0]), float(v_b[1]),
                                           FLOW_SIGMA ** 2, FLOW_SIGMA ** 2,
                                           lever_arm=r_cam_b)
            n['flow' if ok else 'flow_refused'] += 1
        track.append((tk, f.X.p + f.X.R @ r_cam_b, f.X.R @ R_bc))
    return {'track': track, 'counts': n, 'accepted': f.accepted, 'rejected': f.rejected}


def score(seq, run) -> dict:
    """Final horizontal displacement error and yaw error, in the first camera frame."""
    t_gt, p_gt, R_gt = gt_track(seq)
    tr = run['track']
    if not tr:
        return {'ok': False}
    ts = np.array([x[0] for x in tr])
    _, pe0, Re0 = tr[0]
    _, pe1, Re1 = tr[np.searchsorted(ts, t_gt[-1]) - 1]
    d_gt = R_gt[0].T @ (p_gt[-1] - p_gt[0])
    d_est = Re0.T @ (pe1 - pe0)
    path = float(np.sum(np.linalg.norm(np.diff(p_gt, axis=0), axis=1)))

    def yaw(R0, R1):
        Rr = R0.T @ R1
        return math.degrees(math.atan2(Rr[1, 0], Rr[0, 0]))

    yaw_err = (yaw(Re0, Re1) - yaw(R_gt[0], R_gt[-1]) + 180) % 360 - 180
    err = float(np.linalg.norm(d_est[:2] - d_gt[:2]))
    return {'ok': True, 'path_m': path, 'final_err_m': err,
            'drift_pct': 100 * err / path if path > 0 else float('nan'),
            'gt_disp_m': float(np.linalg.norm(d_gt[:2])),
            'yaw_err_deg': yaw_err, **run['counts']}


# ── self-test: the image-to-camera mapping on a synthetic floor ─────────────
def _self_test() -> int:
    rng = np.random.default_rng(0)
    f, cx, cy, h, dt = 400.0, 320.0, 256.0, 2.0, 0.05
    pts = np.c_[rng.uniform(-2, 2, (200, 2)), np.full(200, h)]   # floor, camera frame
    ok = True
    cases = [((0.3, 0.0), (0.0, 0.0, 0.0)), ((0.0, -0.2), (0.0, 0.0, 0.0)),
             ((0.25, 0.1), (0.02, -0.03, 0.0)), ((-0.2, 0.15), (-0.03, 0.02, 0.0)), ((-0.1, 0.2), (0.0, 0.0, 0.3))]
    for (vx, vy), w in cases:
        v = np.array([vx, vy, 0.0])
        w = np.array(w)
        th = np.linalg.norm(w * dt)
        if th > 0:
            k = w * dt / th
            Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
            Rd = np.eye(3) + math.sin(th) * Kx + (1 - math.cos(th)) * Kx @ Kx
        else:
            Rd = np.eye(3)
        p1 = (Rd.T @ (pts - v * dt).T).T                 # camera moved by v dt, rotated by Rd
        u0 = f * pts[:, :2] / pts[:, 2:3] + [cx, cy]
        u1 = f * p1[:, :2] / p1[:, 2:3] + [cx, cy]
        m = solve_planar_motion(u0.astype(np.float32), u1.astype(np.float32), None, dt,
                                cx=cx, cy=cy)
        got, why = camera_velocity(m.dx_px, m.dy_px, dt, f=f, fy=f, h=h, w_cam=w)
        good = got is not None and np.allclose(got, [vx, vy], atol=0.02)
        print(('ok   ' if good else 'FAIL ') + f'v={vx:+.2f},{vy:+.2f} w={tuple(w)} -> '
              + (f'{got[0]:+.3f},{got[1]:+.3f}' if got is not None else why))
        ok &= good
    a = np.array([0.5, -0.3, -9.7])
    R = tilt_from_accel(a)
    good = np.allclose(R.T @ [0, 0, -1], a / np.linalg.norm(a), atol=1e-9)
    print(('ok   ' if good else 'FAIL ') + 'tilt from gravity')
    ok &= good
    print('self-test', 'PASSED' if ok else 'FAILED')
    return 0 if ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--root', help='AQUALOC harbour folder (seq_NN/, calibration/, groundtruth/)')
    ap.add_argument('--calib-seq', type=int, default=1)
    ap.add_argument('--seqs', type=int, nargs='+', default=[2, 3, 4, 5, 6, 7])
    ap.add_argument('--out', help='results JSON')
    ap.add_argument('--self-test', action='store_true')
    a = ap.parse_args(argv)
    if a.self_test:
        return _self_test()
    if not a.root:
        ap.error('--root is required')
    root = pathlib.Path(a.root)
    cal = load_sequence(root, a.calib_seq)
    gi = cal['gt'][:, 0].astype(int)
    h = fit_height(cal, frame_flows(cal, gi[0], gi[-1] + 1))
    print(f'height fitted on sequence {a.calib_seq:02d}: {h:.3f} m', flush=True)
    res = {'height_m': h, 'calib_seq': a.calib_seq, 'seqs': {}}
    for n in a.seqs:
        if not (root / f'seq_{n:02d}').is_dir():
            print(f'seq {n:02d}: missing, skipped')
            continue
        seq = load_sequence(root, n)
        gi = seq['gt'][:, 0].astype(int)
        flows = frame_flows(seq, gi[0], gi[-1] + 1)
        row = {'imu_depth': score(seq, run_filter(seq, flows, h, use_flow=False)),
               'imu_depth_flow': score(seq, run_filter(seq, flows, h, use_flow=True)),
               'variant_image_yaw': score(seq, run_filter(seq, flows, h, use_flow=True,
                                                          image_yaw=True)),
               'flow_pairs': len(flows), 'flow_ok': sum(r is not None for r in flows)}
        res['seqs'][n] = row
        print(f'seq {n:02d}: ' + json.dumps(row), flush=True)
    if a.out:
        pathlib.Path(a.out).write_text(json.dumps(res, indent=1))
    return 0


if __name__ == '__main__':
    os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
    sys.exit(main())

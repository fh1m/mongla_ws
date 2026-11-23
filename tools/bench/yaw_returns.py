#!/usr/bin/env python3
"""Heading error under hand motion, against a physical reference edge.

    tools/bench/yaw_returns.py --self-test
    tools/bench/yaw_returns.py --record 600 --out run.npz    # manager running, vision off
    tools/bench/yaw_returns.py --analyse run.npz

WHY (bead 5ag.9). The AQUALOC replay left heading as the localization error
that remains, and the board's yaw had only been measured at rest. The truth
here is a physical edge: the rig rests against the same table edge again and
again, so its true heading at every rest is the same, and any change in the
board's yaw between rests is error. Between rests the operator yaws the rig
freely by hand.

Rests are found from the board's own gyro, never from the operator's word:
a rest is at least REST_S seconds with every gyro axis under REST_RAD_S.
Every rest is reported, the first is the reference.

The second source. The downward camera's planar-fit image yaw rate,
integrated over the same run, is scored against the same rests. The bead's
bar: image yaw is fused only if it cuts the heading error at the rests by at
least half against the board's yaw, on three runs. The image heading uses the
sign `td_measure` found (image yaw on gyro z, sign -) and the flow node's
starting offset is irrelevant at rest.

Self-test: a synthetic run with a chosen board drift and a chosen image bias
must give back both errors, and a run with no rest must be refused.
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np

REST_S = 4.0
REST_RAD_S = 0.03
IMAGE_SIGN = -1.0          # image yaw rate matches gyro z with sign - (measured-bars 108)


def rests(gyr_t, gyr_xyz):
    """[(t0, t1)] spans where every axis stays under REST_RAD_S for REST_S."""
    quiet = np.all(np.abs(gyr_xyz) < REST_RAD_S, axis=1)
    out, start = [], None
    for t, q in zip(gyr_t, quiet):
        if q and start is None:
            start = t
        elif not q and start is not None:
            if t - start >= REST_S:
                out.append((start, t))
            start = None
    if start is not None and gyr_t[-1] - start >= REST_S:
        out.append((start, gyr_t[-1]))
    return out


def _unwrap_deg(a):
    return np.degrees(np.unwrap(np.radians(a)))


def score(d) -> dict:
    spans = rests(d['gyr_t'], d['gyr_xyz'])
    if len(spans) < 2:
        return {'ok': False, 'why': f'{len(spans)} rest(s) found; need the start and at least one return'}
    yaw_t, yaw = d['yaw_t'], _unwrap_deg(d['yaw_deg'])
    it, iv = d['img_t'], d['img_v']
    img_head = np.degrees(np.concatenate(([0.0], np.cumsum(IMAGE_SIGN * iv[1:] * np.diff(it)))))

    def mean_in(t, v, span):
        m = (t >= span[0] + 0.5) & (t <= span[1] - 0.5)
        return float(np.mean(v[m])) if m.any() else float('nan')

    board = [mean_in(yaw_t, yaw, s) for s in spans]
    image = [mean_in(it, img_head, s) for s in spans] if len(it) else [float('nan')] * len(spans)
    rows = []
    for k, s in enumerate(spans):
        rows.append({'t_s': round(s[0] - spans[0][0], 1),
                     'board_err_deg': board[k] - board[0],
                     'image_err_deg': image[k] - image[0]})
    tm = np.array([r['t_s'] for r in rows]) / 60.0
    be = np.array([r['board_err_deg'] for r in rows])
    slope = float(np.polyfit(tm, be, 1)[0]) if len(rows) >= 3 else float('nan')
    b_rms = float(np.sqrt(np.mean(be[1:] ** 2)))
    ie = np.array([r['image_err_deg'] for r in rows])
    i_rms = float(np.sqrt(np.nanmean(ie[1:] ** 2))) if np.isfinite(ie[1:]).any() else float('nan')
    return {'ok': True, 'rests': rows, 'board_drift_deg_per_min': slope,
            'board_rms_deg': b_rms, 'image_rms_deg': i_rms,
            'image_halves_board_error': bool(i_rms <= 0.5 * b_rms) if np.isfinite(i_rms) else None,
            'minutes': float((spans[-1][1] - spans[0][0]) / 60.0)}


def _synthetic(drift_deg_min, img_bias_deg_s, with_rests=True):
    """Rest 6 s, swing 20 s, repeated; board yaw drifts linearly."""
    t = np.arange(0.0, 180.0, 0.02)
    period = 26.0
    phase = t % period
    moving = (phase > 6.0) if with_rests else np.ones_like(t, bool)
    rate = np.where(moving, 0.8 * np.sin(2 * np.pi * (phase - 6.0) / 20.0), 0.0)
    true = np.degrees(np.cumsum(rate) * 0.02)
    board = true + drift_deg_min * t / 60.0
    gyr = np.zeros((len(t), 3))
    gyr[:, 2] = rate
    img_v = IMAGE_SIGN * (rate + np.radians(img_bias_deg_s))
    return {'gyr_t': t, 'gyr_xyz': gyr, 'yaw_t': t, 'yaw_deg': board % 360.0,
            'img_t': t, 'img_v': img_v}


def _self_test() -> int:
    ok = True

    def check(cond, what):
        nonlocal ok
        print(('ok   ' if cond else 'FAIL ') + what)
        ok &= bool(cond)

    r = score(_synthetic(2.0, 0.0))
    check(r['ok'] and abs(r['board_drift_deg_per_min'] - 2.0) < 0.05,
          f"a 2.0 deg/min board drift comes back ({r.get('board_drift_deg_per_min'):.3f})")
    check(abs(r['image_rms_deg']) < 0.05 and r['image_halves_board_error'] is True,
          'an unbiased image heading is scored near zero and halves the board error')
    r2 = score(_synthetic(0.0, 0.05))
    check(r2['image_rms_deg'] > 1.0 and r2['image_halves_board_error'] is False,
          f"an image bias of 0.05 deg/s shows as error ({r2['image_rms_deg']:.2f} deg rms)")
    r3 = score(_synthetic(1.0, 0.0, with_rests=False))
    check(not r3['ok'], 'a run with no rest is refused, not scored')
    print('self-test', 'PASSED' if ok else 'FAILED')
    return 0 if ok else 1


def record(seconds: float, out: str, device: str) -> int:
    import threading
    import cv2
    import rclpy
    from geometry_msgs.msg import Vector3Stamped
    from mongla_interfaces.msg import MonglaState
    from mongla_vision.cameras.v4l2_mailbox import V4L2MailboxCamera
    from mongla_vision.flow.flow_math import solve_planar_motion

    gyr, yaw = [], []
    rclpy.init()
    node = rclpy.create_node('yaw_returns')

    def on_rates(m):
        gyr.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                    m.vector.x, m.vector.y, m.vector.z))

    def on_state(m):
        if np.isfinite(m.yaw_deg):
            yaw.append((time.time(), float(m.yaw_deg)))

    node.create_subscription(Vector3Stamped, '/mongla/imu_rates', on_rates, 50)
    node.create_subscription(MonglaState, '/mongla/state', on_state, 50)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    cam = V4L2MailboxCamera(device=device, width=640, height=360, fps=210)
    img, prev, prev_t = [], None, None
    t_end = time.monotonic() + seconds
    print(f'[yaw] recording {seconds:.0f} s: rest against the edge, yaw freely, return, repeat')
    while time.monotonic() < t_end:
        frame, meta = cam.read()
        if frame is None or not meta.fresh:
            time.sleep(0.002)
            continue
        g = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (320, 180),
                       interpolation=cv2.INTER_AREA)
        t = float(meta.stamp_wall)
        if prev is not None:
            pts = cv2.goodFeaturesToTrack(prev, 200, 0.01, 6)
            if pts is not None and len(pts) >= 12:
                nxt, st, _ = cv2.calcOpticalFlowPyrLK(prev, g, pts, None)
                pm = solve_planar_motion(pts.reshape(-1, 2), nxt.reshape(-1, 2),
                                         st.reshape(-1), t - prev_t, cx=160.0, cy=90.0)
                img.append((0.5 * (t + prev_t), pm.yaw_rate if pm.ok else 0.0))
        prev, prev_t = g, t
    cam.close()
    node.destroy_node()
    rclpy.shutdown()
    g, y, i = np.array(gyr), np.array(yaw), np.array(img)
    np.savez(out, gyr_t=g[:, 0], gyr_xyz=g[:, 1:4], yaw_t=y[:, 0], yaw_deg=y[:, 1],
             img_t=i[:, 0], img_v=i[:, 1])
    print(f'[yaw] saved {out}: {len(g)} gyro, {len(y)} yaw, {len(i)} image samples')
    return analyse(out)


def analyse(path: str) -> int:
    import json
    r = score(dict(np.load(path)))
    print(json.dumps(r, indent=1))
    return 0 if r['ok'] else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--self-test', action='store_true')
    ap.add_argument('--record', type=float, metavar='SECONDS')
    ap.add_argument('--out', default='yaw_returns.npz')
    ap.add_argument('--device', default='/dev/mongla_cam_downward')
    ap.add_argument('--analyse', metavar='NPZ')
    a = ap.parse_args(argv)
    if a.self_test:
        return _self_test()
    if a.record:
        return record(a.record, a.out, a.device)
    if a.analyse:
        return analyse(a.analyse)
    ap.error('one of --self-test, --record, --analyse')
    return 2


if __name__ == '__main__':
    sys.exit(main())

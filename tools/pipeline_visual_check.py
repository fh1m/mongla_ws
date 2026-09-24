#!/usr/bin/env python3
"""Render what the ladder SEES, frame by frame, so it can be checked by eye.

⛔ WHY THIS EXISTS. Every number in `measured-bars.md` §20-§28 is an inlier
count, and an inlier count says a match is good without saying good AT WHAT.
Two hundred inliers on the prop and two hundred on the pool wall behind it are
the same number and completely different situations -- §22 is exactly that
failure, found only because a negative was finally run.

So this draws the things a number cannot show:
  - the detector's box, with its confidence
  - the bank's matched quad, warped through the homography
  - the surviving correspondences, reference -> live
  - which reference in the bank answered, and its inlier count
  - annotations carried through H (the trainingless-detector property)
  - the identity verdict and WHY, from `recognise()`

A green quad sitting on water while the prop is elsewhere is instantly obvious
here and invisible in a table.
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'src', 'mongla_vision')


def load(model, threads):
    sys.path.insert(0, os.path.abspath(SRC))
    spec = importlib.util.spec_from_file_location(
        'xfeat_onnx', os.path.join(SRC, 'mongla_vision', 'anchor', 'xfeat_onnx.py'))
    X = importlib.util.module_from_spec(spec)
    sys.modules['xfeat_onnx'] = X
    spec.loader.exec_module(X)
    from mongla_vision.anchor.bank import CheckpointBank
    return X, CheckpointBank, X.XFeatONNX(model, top_k=1024, threads=threads)


def letterbox(img, size=640):
    h, w = img.shape[:2]
    r = min(size / h, size / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    out = np.full((size, size, 3), 114, np.uint8)
    out[:nh, :nw] = cv2.resize(img, (nw, nh))
    return out, r


def detect(sess, iname, img, conf_th):
    lb, r = letterbox(img)
    x = lb[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    y = sess.run(None, {iname: x})[0][0]
    conf = y[4:].max(axis=0)
    cls = y[4:].argmax(axis=0)
    i = int(conf.argmax())
    if conf[i] < conf_th:
        return None, 0.0, -1
    cx, cy, bw, bh = y[:4, i] / r
    return ((cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2),
            float(conf[i]), int(cls[i]))


def draw(frame, be, det, conf, pose, names):
    """Everything the ladder believes, on one canvas."""
    h, w = frame.shape[:2]
    sx, sy = w / be.w, h / be.h
    out = frame.copy()

    if det is not None:
        x1, y1, x2, y2 = (int(v) for v in det)
        cv2.rectangle(out, (x1, y1), (x2, y2), (60, 200, 255), 2)
        cv2.putText(out, f'det {conf:.2f}', (x1, max(16, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 200, 255), 1, cv2.LINE_AA)

    ok = pose is not None and pose.ok
    col = (80, 255, 80) if (pose and pose.identity_ok) else (
        (0, 200, 255) if ok else (0, 0, 220))
    if ok and pose.pose is not None and pose.pose.corners is not None:
        q = np.asarray(pose.pose.corners, np.float32).reshape(-1, 2).copy()
        q[:, 0] *= sx; q[:, 1] *= sy
        cv2.polylines(out, [q.astype(np.int32)], True, col, 2, cv2.LINE_AA)
        # Correspondences: what the number is actually made of.
        lp = getattr(pose.pose, 'live_pts', None)
        if lp is not None:
            for px, py in np.asarray(lp, np.float32)[:220]:
                cv2.circle(out, (int(px * sx), int(py * sy)), 1, (80, 255, 80), -1)
    if ok and pose.points:
        for nm, (px, py) in pose.points.items():
            p = (int(px * sx), int(py * sy))
            cv2.drawMarker(out, p, (255, 120, 255), cv2.MARKER_CROSS, 18, 2)
            cv2.putText(out, nm, (p[0] + 8, p[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 120, 255), 1, cv2.LINE_AA)

    bar = [f'bank {pose.searched if pose else 0} searched',
           f'inliers {pose.inliers if pose else 0}',
           f'ref #{pose.index}' if pose and pose.index is not None else 'ref -',
           f'label {pose.label!r}' if pose and pose.label else 'label -']
    cv2.rectangle(out, (0, 0), (w, 46), (24, 24, 24), -1)
    cv2.putText(out, '  |  '.join(bar), (8, 18), cv2.FONT_HERSHEY_SIMPLEX,
                0.48, (235, 235, 235), 1, cv2.LINE_AA)
    why = (pose.identity_why if pose else '') or 'no match'
    verdict = 'IDENTITY OK' if (pose and pose.identity_ok) else 'not asserted'
    cv2.putText(out, f'{verdict}: {why[:110]}', (8, 38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--xfeat', required=True)
    ap.add_argument('--detector', default='')
    ap.add_argument('--video', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--label', default='prop')
    ap.add_argument('--refs', type=int, default=6)
    ap.add_argument('--frames', type=int, default=8)
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--threads', type=int, default=2)
    ap.add_argument('--enrol-all', action='store_true',
                    help='ignore scale coverage; enrol every candidate (the '
                         'OLD behaviour, for an A/B)')
    a = ap.parse_args()

    X, Bank, be = load(a.xfeat, a.threads)
    sess = iname = None
    if a.detector:
        import onnxruntime as ort
        sess = ort.InferenceSession(a.detector, providers=['CPUExecutionProvider'])
        iname = sess.get_inputs()[0].name

    cap = cv2.VideoCapture(a.video)
    if not cap.isOpened():
        raise SystemExit(f'cannot open {a.video}')
    N = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    os.makedirs(a.out, exist_ok=True)

    bank = Bank(be, capacity=16, period_s=0.333)
    # Enrol from the FIRST third, query the LAST -- otherwise the bank is being
    # shown the frames it was built from and every quad is trivially correct.
    #
    # ⭐ Enrolment is gated on SCALE COVERAGE, which is the production rule: a
    # view earns a slot when its apparent size is outside the band the bank
    # already holds, even while the existing references match fine.
    for j in range(a.refs):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(N * (0.10 + 0.18 * j / max(1, a.refs - 1))))
        ok, f = cap.read()
        if not ok:
            continue
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        roi = conf = None
        if sess is not None:
            roi, conf, _ = detect(sess, iname, f, a.conf)
        ann = None
        if roi is not None:
            # Two points ON the prop, in full-frame pixels: the aim point and
            # its upper-left quarter. Stand-ins for hole centres marked once.
            x1, y1, x2, y2 = roi
            ann = {'aim': ((x1 + x2) / 2, (y1 + y2) / 2),
                   'q1': (x1 + (x2 - x1) * 0.3, y1 + (y2 - y1) * 0.3)}
        # ⛔ SCALE COVERAGE IS ADDITIVE IN PRODUCTION, NOT RESTRICTIVE.
        # `lock_node` enrols when `not has_reference or stale or uncovered` --
        # an uncovered scale is one MORE reason to take a checkpoint, never a
        # reason to skip one. An earlier version of this harness inverted that
        # and skipped covered views; it cut the bank from 14 references to 3
        # and identity assertions from 3/8 to 0/8, which is a measurement of
        # the inverted policy and not of the shipped one.
        covered = bank.covers_scale(g, roi) if bank.size else False
        r = bank.enrol(g, roi=roi, det_conf=conf or 1.0, label=a.label,
                       annotations=ann, force=True)
        print(f'  enrol {j}: {r.reason} {r.keypoints} kp'
              + (f' roi={tuple(int(v) for v in roi)}' if roi is not None else ' whole-frame')
              + (f' scale={bank.scales[-1]:.0f}' if bank.size else '')
              + ('' if covered else '  <- NEW SCALE'))

    print(f'  bank: {bank.size} references, shortlist<= {bank.shortlist_k()}')
    written = []
    for j in range(a.frames):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(N * (0.55 + 0.40 * j / max(1, a.frames - 1))))
        ok, f = cap.read()
        if not ok:
            continue
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        det = conf = None
        cls = -1
        if sess is not None:
            det, conf, cls = detect(sess, iname, f, a.conf)
        pose = bank.recognise(g, label=a.label,
                              det_class=a.label if det is not None else None,
                              det_age_s=0.1)
        p = os.path.join(a.out, f'{j:02d}.png')
        cv2.imwrite(p, draw(f, be, det, conf or 0.0, pose, None))
        written.append(p)
        print(f'  frame {j}: inliers {pose.inliers:>4}  ref #{pose.index}  '
              f'identity {"OK " if pose.identity_ok else "no "} {pose.identity_why}')
    cap.release()

    # One contact sheet, because eight windows is not a review.
    imgs = [cv2.imread(p) for p in written]
    if imgs:
        hgt = 260
        rs = [cv2.resize(i, (int(i.shape[1] * hgt / i.shape[0]), hgt)) for i in imgs]
        rows = [np.hstack(rs[i:i + 4]) for i in range(0, len(rs), 4)]
        wmax = max(r.shape[1] for r in rows)
        rows = [np.pad(r, ((0, 0), (0, wmax - r.shape[1]), (0, 0))) for r in rows]
        sheet = os.path.join(a.out, 'contact_sheet.png')
        cv2.imwrite(sheet, np.vstack(rows))
        print(f'\n  contact sheet: {sheet}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

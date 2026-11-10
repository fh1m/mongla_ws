#!/usr/bin/env python3
"""Does the HELD rung put the lock where the target is, and only there?

HELD lets a box under the acting bar keep a lock the bar acquired, when it
lands near where the lock already is (`lock_frames._held_from`). It replaces
the follower's integrated guess with the detector's own look. That is only an
improvement if the held box is closer to the target than what it replaces,
and only safe if it does not hold a lock on water with no target in it.

Two steps, because the detector runs on the vehicle and the scoring does not:

    # on Kutum, through the HEF, every box down to the association floor
    python3 tools/studies/held_rung_score.py collect --model models/gate_sharks.hef \\
        --class gate --clip gate_back.mp4 --out gate_back.jsonl

    # anywhere: replay the real ladder code, HELD off and on
    python3 tools/studies/held_rung_score.py score \\
        --positive gate_back.jsonl gate.jsonl --negative torpedo.jsonl

The replay uses `arbitrate` and `_held_from` themselves, with no follower and
no anchor, so the only rungs are DETECTION, HELD and LOST and any change is
HELD's.

PASS CRITERIA, written 2026-11-10, before the first run:

  1. ACCURACY. Truth is the detector's NEXT box that clears the bar, within
     `LOOKAHEAD_S`: an independent later observation, not the method grading
     itself. For every frame HELD answers, compare the centre error of the
     held box against the error of the last strong box carried forward (what
     a follower that has not moved would say, and the best case for it in a
     static scene). PASS when the median held error is SMALLER, over at least
     `MIN_SCORED` scored frames across the positive clips.
  2. SAFETY. On clips asserted to contain no target, the share of frames with
     a lock must rise by no more than 1.0 point with HELD on. HELD cannot
     acquire, so this can only move if it keeps false locks alive longer.
  3. VALUE. On the positive clips, the share of frames whose lock comes from
     the detector (DETECTION or HELD) must rise. If it does not, HELD has
     nothing to do on this footage, whatever 1 and 2 say.

FALSIFIER: `--shuffle` moves every weak box to a uniformly random place in
the frame before the replay. Criterion 1 must FAIL. If it still passes, the
scorer is rewarding something other than the boxes being where the target is.

Reported, not gated: rung shares per clip, how many weak boxes each clip had,
the held-error and carried-error distributions.
"""
from __future__ import annotations

import argparse
import json
import random
import statistics
import sys
from pathlib import Path
from types import SimpleNamespace

LOOKAHEAD_S = 0.25
MIN_SCORED = 30
SAFETY_POINTS = 1.0


def collect(a) -> int:
    """Run the HEF over a clip; one JSON line per frame, every box of the class."""
    import cv2
    from mongla_vision.detection.hailo import HailoDetector

    det = HailoDetector(model_path=a.model, conf=a.act_conf,
                        class_allowlist=(a.cls,), warmup=True)
    det.update_assoc_conf(a.assoc_conf)
    cap = cv2.VideoCapture(a.clip)
    n = 0
    with open(a.out, 'w', encoding='utf-8') as out:
        while n < a.frames:
            ok, frame = cap.read()
            if not ok:
                break
            # The frame's own timestamp, never n / fps: these recordings are
            # variable-rate and declare 250-1000 fps, so a nominal rate would
            # squeeze 90 s into 2 and the authority decay would never act.
            t = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
            boxes = [[round(d.score, 4)] + [round(v, 1) for v in d.xyxy]
                     for d in det.infer(frame)]
            out.write(json.dumps({'t': round(t, 4), 'w': frame.shape[1],
                                  'h': frame.shape[0], 'boxes': boxes}) + '\n')
            n += 1
    det.close()
    print(f'{a.clip}: {n} frames over {t:.1f} s -> {a.out}')
    return 0


def _bbox(x1, y1, x2, y2):
    """A vision_msgs-shaped box, which is what `_held_from` reads."""
    return SimpleNamespace(center=SimpleNamespace(position=SimpleNamespace(
        x=(x1 + x2) / 2, y=(y1 + y2) / 2)), size_x=x2 - x1, size_y=y2 - y1)


def _centre(b):
    return ((b[0] + b[2]) / 2, (b[1] + b[3]) / 2)


def replay(frames, act_conf, held_on, shuffle_rng=None):
    """Per frame: (rung, box) from the real ladder, detector rungs only."""
    import numpy as np
    from mongla_vision.lock_frames import LockFramesMixin
    from mongla_vision.tracking.lock_state import arbitrate

    st = SimpleNamespace(_act_conf=act_conf, _lock_centre=None, _gray=None)
    last_t, out = 0.0, []
    for f in frames:
        st._gray = np.zeros((1, int(f['w'])), np.uint8)
        strong = [b for b in f['boxes'] if b[0] >= act_conf]
        weak = [b for b in f['boxes'] if b[0] < act_conf]
        if shuffle_rng is not None:
            moved = []
            for s, x1, y1, x2, y2 in weak:
                bw, bh = x2 - x1, y2 - y1
                cx = shuffle_rng.uniform(0, f['w'])
                cy = shuffle_rng.uniform(0, f['h'])
                moved.append([s, cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2])
            weak = moved
        det = max(strong, key=lambda b: b[0]) if strong else None
        held = None
        if det is None and held_on:
            held = LockFramesMixin._held_from(
                st, [(b[0], _bbox(*b[1:])) for b in weak])
        if det is not None:
            last_t = f['t']
        r = arbitrate(now=f['t'], last_detection_t=last_t,
                      detection=None if det is None else tuple(det[1:]),
                      detection_conf=0.0 if det is None else det[0],
                      held=None if held is None else held[0],
                      held_conf=0.0 if held is None else held[1],
                      held_age_s=0.0)
        st._lock_centre = r.centre if r.have_target else None
        out.append((r.rung.value, r.xyxy, det))
    return out


def accuracy(frames, ladder):
    """(held errors, carried errors) against the next strong box."""
    held_err, carried_err = [], []
    last_strong = None
    for i, (f, (rung, box, det)) in enumerate(zip(frames, ladder)):
        if det is not None:
            last_strong = det[1:]
            continue
        if rung != 'held' or last_strong is None:
            continue
        truth = None
        for g, (_, _, d2) in zip(frames[i + 1:], ladder[i + 1:]):
            if g['t'] - f['t'] > LOOKAHEAD_S:
                break
            if d2 is not None:
                truth = _centre(d2[1:])
                break
        if truth is None:
            continue
        hc, cc = _centre(box), _centre(last_strong)
        held_err.append(((hc[0] - truth[0]) ** 2 + (hc[1] - truth[1]) ** 2) ** 0.5)
        carried_err.append(((cc[0] - truth[0]) ** 2 + (cc[1] - truth[1]) ** 2) ** 0.5)
    return held_err, carried_err


def _load(path):
    """Frames in time order. OpenCV reports position 0 for the last frame of
    some files, and a frame whose time does not advance would reset every
    age the ladder measures, so it is dropped rather than believed."""
    out = []
    with open(path, encoding='utf-8') as f:
        for line in f:
            if line.strip():
                fr = json.loads(line)
                if not out or fr['t'] > out[-1]['t']:
                    out.append(fr)
    return out


def _shares(ladder):
    n = max(len(ladder), 1)
    out = {}
    for rung, _, _ in ladder:
        out[rung] = out.get(rung, 0) + 1
    return {k: 100.0 * v / n for k, v in out.items()}


def score(a) -> int:
    rng = random.Random(0) if a.shuffle else None
    held_all, carried_all = [], []
    det_gain = []
    for p in a.positive:
        frames = _load(p)
        off = replay(frames, a.act_conf, False)
        on = replay(frames, a.act_conf, True, rng)
        h, c = accuracy(frames, on)
        held_all += h
        carried_all += c
        s_off, s_on = _shares(off), _shares(on)
        from_det = lambda s: s.get('detection', 0.0) + s.get('held', 0.0)  # noqa: E731
        det_gain.append(from_det(s_on) - from_det(s_off))
        weak = sum(1 for f in frames for b in f['boxes'] if b[0] < a.act_conf)
        print(f'+ {Path(p).name}: {len(frames)} frames, {weak} weak boxes; '
              f'off {_fmt(s_off)} | on {_fmt(s_on)}; scored {len(h)}')
    rise = []
    for p in a.negative:
        frames = _load(p)
        s_off = _shares(replay(frames, a.act_conf, False))
        s_on = _shares(replay(frames, a.act_conf, True, rng))
        lock = lambda s: 100.0 - s.get('lost', 0.0)  # noqa: E731
        rise.append(lock(s_on) - lock(s_off))
        print(f'- {Path(p).name}: lock {lock(s_off):.1f} % -> {lock(s_on):.1f} %')

    med = lambda v: statistics.median(v) if v else float('nan')  # noqa: E731
    print(f'\nheld error    median {med(held_all):.1f} px over {len(held_all)}')
    print(f'carried error median {med(carried_all):.1f} px')
    checks = [
        ('1 accuracy: held closer than carried, enough frames',
         len(held_all) >= MIN_SCORED and med(held_all) < med(carried_all)),
        (f'2 safety: lock on negatives rises <= {SAFETY_POINTS} pt',
         all(r <= SAFETY_POINTS for r in rise)),
        ('3 value: more of the lock comes from the detector',
         bool(det_gain) and all(g > 0.0 for g in det_gain)),
    ]
    for name, ok in checks:
        print(f'  [{"PASS" if ok else "FAIL"}] {name}')
    return 0 if all(ok for _, ok in checks) else 1


def _fmt(s):
    return ' '.join(f'{k}={v:.1f}' for k, v in sorted(s.items()))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('collect')
    c.add_argument('--model', required=True, help='path to the .hef')
    c.add_argument('--class', dest='cls', default='gate')
    c.add_argument('--clip', required=True)
    c.add_argument('--out', required=True)
    c.add_argument('--frames', type=int, default=10 ** 9)
    c.add_argument('--act-conf', type=float, default=0.45)
    c.add_argument('--assoc-conf', type=float, default=0.10)
    s = sub.add_parser('score')
    s.add_argument('--positive', nargs='+', required=True)
    s.add_argument('--negative', nargs='*', default=[])
    s.add_argument('--act-conf', type=float, default=0.45)
    s.add_argument('--shuffle', action='store_true',
                   help='falsifier: scatter weak boxes; criterion 1 must fail')
    a = ap.parse_args()
    return collect(a) if a.cmd == 'collect' else score(a)


if __name__ == '__main__':
    sys.exit(main())

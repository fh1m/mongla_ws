#!/usr/bin/env python3
"""Would STARK-Lightning on the Hailo-8 carry a box through a detection gap
longer than the follower does?

The follower (`tracking/follower.py`) carries the last box on sparse LK and
refuses when its points stop round-tripping. STARK-Lightning
(`stark_lightning_repvgg_a0`, Hailo Model Zoo v2.20, compiled for the Hailo-8)
finds the template's appearance again in a search crop, independently of the
detector and of XFeat geometry, which is what the ladder wants from a rung
(bead po7.16).

Truth is known by construction. Detections come from the model on the clip
(`held_rung_score.py collect` writes them). A window starts on a frame with a
box at or above the acting bar, and both rungs are seeded there with that
box, as the ladder seeds the follower. The detector's later boxes are then
HIDDEN from both and used only to score: each later frame that has a strong
box is a truth frame, and a rung holds while its box keeps IoU >= 0.5 with
it. Hold time is the seconds from the seed to the first truth frame it fails
(a refusal is a failure), capped at the horizon, 2.5 s, which covers the
measured p99 gap (2.418 s). Frames with no strong box are not scored.
Windows start at least 1 s apart, and only where at least 60 % of the frames
in the horizon have truth.

Bar, the bead's, written before the first run: STARK passes when, paired per
window, its hold time is longer than the follower's (the median of the paired
difference above 0, and the lower end of its 95 % bootstrap interval above 0,
over at least 30 windows), and then only if a chip-cost run keeps both
detectors at >= 95 % of their rate with it running. Falsifier: `frozen`, the
seed box carried unchanged, is scored beside them and must hold for less time
than either rung, or the scorer is rewarding something other than following.

The first full run FAILED the falsifier (frozen held the horizon in 96 % of
the gate windows: the target barely moves in frame, so every rung saturates),
so the bar is now scored on MOVING windows only, chosen from the truth alone
and not from either rung: those where the frozen box drops below IoU 0.5 within
the horizon. This rule was added after seeing that failure, and the all-window
numbers are still printed. The bar is unchanged otherwise.

Runs on Kutum (the HEF needs the chip):

    python3 tools/studies/stark_rung.py --hef ~/eval_models/stark_lightning_repvgg_a0.hef \\
        --pair gate.jsonl:~/data/clips/gate.mkv --pair torpedo.jsonl:~/data/clips/torpedo.mkv
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.expanduser('~/mongla_ws/src/mongla_vision'))
sys.path.insert(1, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                                'src', 'mongla_vision'))
from mongla_vision.tracking.follower import Follower  # noqa: E402

ACT_CONF = 0.45
HORIZON_S = 2.5
SPACING_S = 1.0
MIN_TRUTH = 0.6
IOU = 0.5
# Hailo Model Zoo `tracker_infer.py`, which mirrors STARK's own tracker.
TEMPLATE_FACTOR, TEMPLATE_SIZE = 2.0, 128
SEARCH_FACTOR, SEARCH_SIZE = 5.0, 320
FEAT, STRIDE = 20, 16


def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    i = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i
    return i / u if u > 0 else 0.0


def sample_target(rgb, xyxy, factor, size):
    """A square crop of side sqrt(w*h)*factor around the box, zero-padded,
    resized; and size/crop. STARK's `sample_target`, as the Model Zoo ports it."""
    x, y = xyxy[0], xyxy[1]
    w, h = xyxy[2] - xyxy[0], xyxy[3] - xyxy[1]
    crop = max(1, int(np.ceil(np.sqrt(max(w, 1e-3) * max(h, 1e-3)) * factor)))
    x1 = int(round(x + 0.5 * w - crop * 0.5))
    y1 = int(round(y + 0.5 * h - crop * 0.5))
    x2, y2 = x1 + crop, y1 + crop
    px1, py1 = max(0, -x1), max(0, -y1)
    px2, py2 = max(x2 - rgb.shape[1] + 1, 0), max(y2 - rgb.shape[0] + 1, 0)
    patch = rgb[y1 + py1:y2 - py2, x1 + px1:x2 - px2]
    patch = np.pad(patch, ((py1, py2), (px1, px2), (0, 0)))
    return cv2.resize(patch, (size, size), interpolation=cv2.INTER_LINEAR), size / crop


def corners(maps):
    """STARK's `Corner_Predictor_Lite.soft_argmax` on the HEF's two 20x20 maps:
    each corner is the softmax-weighted mean of the cell centres. Normalised
    xyxy in the search crop."""
    m = np.asarray(maps, np.float64).reshape(FEAT, FEAT, 2)
    idx = (np.arange(FEAT) + 0.5) * STRIDE
    out = []
    for c in (0, 1):
        p = np.exp(m[..., c] - m[..., c].max())
        p /= p.sum()
        out += [float((p.sum(0) * idx).sum()), float((p.sum(1) * idx).sum())]
    return np.array(out) / (FEAT * STRIDE)


class Stark:
    def __init__(self, hef):
        from hailo_platform import FormatType, HailoSchedulingAlgorithm, VDevice
        p = VDevice.create_params()
        p.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
        self._vd = VDevice(p)
        self._model = self._vd.create_infer_model(hef)
        names = sorted(self._model.input_names)              # input_layer1, input_layer2
        self._tin, self._sin = names
        for n in names:
            self._model.input(n).set_format_type(FormatType.UINT8)
        self._model.output().set_format_type(FormatType.FLOAT32)
        self._cim = self._model.configure()
        self._cim.__enter__()
        self._b = self._cim.create_bindings()
        self._out = np.empty(self._model.output().shape, np.float32)
        self._b.output().set_buffer(self._out)
        self._tmpl = None
        self.box = None

    def reset(self, rgb, xyxy):
        self._tmpl = np.ascontiguousarray(sample_target(rgb, xyxy, TEMPLATE_FACTOR, TEMPLATE_SIZE)[0])
        self.box = list(map(float, xyxy))

    def step(self, rgb):
        search, rf = sample_target(rgb, self.box, SEARCH_FACTOR, SEARCH_SIZE)
        self._b.input(self._tin).set_buffer(self._tmpl)
        self._b.input(self._sin).set_buffer(np.ascontiguousarray(search))
        self._cim.run([self._b], 1000)
        bn = corners(self._out) * SEARCH_SIZE / rf
        cx = (self.box[0] + self.box[2]) / 2
        cy = (self.box[1] + self.box[3]) / 2
        half = 0.5 * SEARCH_SIZE / rf
        x1, y1, x2, y2 = bn[0] + cx - half, bn[1] + cy - half, bn[2] + cx - half, bn[3] + cy - half
        H, W = rgb.shape[:2]                                 # STARK's clip_box, margin 10
        x1, y1 = min(max(0, x1), W - 10), min(max(0, y1), H - 10)
        x2, y2 = min(max(10, x2), W), min(max(10, y2), H)
        self.box = [x1, y1, max(x1 + 10, x2), max(y1 + 10, y2)]
        return self.box

    def close(self):
        self._cim.__exit__(None, None, None)


def truth_of(rec, prev):
    strong = [b for b in rec['boxes'] if b[0] >= ACT_CONF]
    if not strong:
        return None
    if prev is None:
        return max(strong, key=lambda b: b[0])[1:]
    pc = ((prev[0] + prev[2]) / 2, (prev[1] + prev[3]) / 2)
    return min(strong, key=lambda b: ((b[1] + b[3]) / 2 - pc[0]) ** 2 + ((b[2] + b[4]) / 2 - pc[1]) ** 2)[1:]


def windows(recs):
    """Seed indices: a strong box, then enough truth in the horizon."""
    out, last = [], -1e9
    for i, r in enumerate(recs):
        if r['t'] - last < SPACING_S or truth_of(r, None) is None:
            continue
        js = []
        for j in range(i + 1, len(recs)):
            if recs[j]['t'] < recs[j - 1]['t'] or recs[j]['t'] - r['t'] > HORIZON_S:
                break          # past the horizon, or the clip's end (collect writes t=0 there)
            js.append(j)
        if not js or recs[js[-1]]['t'] - r['t'] < 0.9 * HORIZON_S:
            continue
        if sum(truth_of(recs[j], None) is not None for j in js) < MIN_TRUTH * len(js):
            continue
        out.append(i)
        last = r['t']
    return out


def run_window(recs, i, frames, stark):
    """One seed `i`: hold time of each rung, and STARK's median call time."""
    seed = truth_of(recs[i], None)
    f0 = frames[i]
    fol = Follower()
    fol.reset(cv2.cvtColor(f0, cv2.COLOR_BGR2GRAY), seed)
    stark.reset(cv2.cvtColor(f0, cv2.COLOR_BGR2RGB), seed)
    hold = {'follower': None, 'stark': None, 'frozen': None}
    prev, t0, ms = seed, recs[i]['t'], []
    for j in range(i + 1, len(recs)):
        dt = recs[j]['t'] - t0
        if dt > HORIZON_S or recs[j]['t'] < recs[j - 1]['t'] or j not in frames:
            break
        f = frames[j]
        r = fol.step(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)) if hold['follower'] is None else None
        e0 = cv2.getTickCount()
        sb = stark.step(cv2.cvtColor(f, cv2.COLOR_BGR2RGB)) if hold['stark'] is None else None
        ms.append(1000 * (cv2.getTickCount() - e0) / cv2.getTickFrequency())
        tr = truth_of(recs[j], prev)
        if tr is None:
            continue
        prev = tr
        boxes = {'follower': (r.xyxy if r is not None and r.ok else None), 'stark': sb, 'frozen': seed}
        for k, b in boxes.items():
            if hold[k] is None and (b is None or iou(b, tr) < IOU):
                hold[k] = dt
    return {k: (HORIZON_S if v is None else v) for k, v in hold.items()} | {
        't': t0, 'stark_ms': float(np.median(ms)) if ms else 0.0}


def run_clip(jsonl, clip, stark):
    """Stream the clip once, holding only the frames of the windows still open
    (a whole long clip does not fit in the Pi's memory: the first version was
    killed)."""
    recs = [json.loads(x) for x in open(jsonl)]
    seeds = windows(recs)
    cap = cv2.VideoCapture(clip)
    buf, rows, pending = {}, [], list(seeds)
    for k in range(len(recs)):
        if not pending:
            break
        ok, f = cap.read()
        if not ok:
            break
        if k >= pending[0]:
            buf[k] = f
        # a window closes when its horizon has been read
        while pending and (k + 1 >= len(recs) or recs[k]['t'] - recs[pending[0]]['t'] > HORIZON_S
                           or recs[k]['t'] < recs[k - 1]['t']):
            i = pending.pop(0)
            if i in buf:
                row = run_window(recs, i, buf, stark)
                row['clip'] = os.path.basename(clip)
                rows.append(row)
            for key in [x for x in buf if pending and x < pending[0] or not pending]:
                del buf[key]
    cap.release()
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--hef', required=True)
    ap.add_argument('--pair', action='append', required=True, help='detections.jsonl:clip')
    ap.add_argument('--boot', type=int, default=10000)
    ap.add_argument('--out', default='', help='write every window row as JSON')
    a = ap.parse_args()
    stark = Stark(os.path.expanduser(a.hef))
    rows = []
    try:
        for p in a.pair:
            j, c = p.split(':', 1)
            got = run_clip(os.path.expanduser(j), os.path.expanduser(c), stark)
            rows += got
            if not got:
                print(f'{os.path.basename(c)}: no windows')
            for k in ('follower', 'stark', 'frozen') if got else ():
                h = [r[k] for r in got]
                print(f'{os.path.basename(c):14s} {k:9s} windows {len(h):3d}  hold median {np.median(h):.2f} s'
                      f'  full horizon {np.mean([x >= HORIZON_S for x in h]):.0%}')
    finally:
        stark.close()
    if not rows:
        return 1
    allrows = rows
    rows = [r for r in rows if r['frozen'] < HORIZON_S]
    print(f'moving windows (a frozen box fails inside the horizon): {len(rows)} of {len(allrows)}; '
          f'all-window medians follower {np.median([r["follower"] for r in allrows]):.2f} '
          f'stark {np.median([r["stark"] for r in allrows]):.2f} s')
    if a.out:
        json.dump(allrows, open(a.out, 'w'))
    if len(rows) < 2:
        print('too few moving windows')
        return 1
    d = np.array([r['stark'] - r['follower'] for r in rows])
    rng = np.random.default_rng(0)
    b = np.median(d[rng.integers(0, len(d), (a.boot, len(d)))], 1)
    lo, hi = np.percentile(b, (2.5, 97.5))
    for k in ('follower', 'stark', 'frozen'):
        print(f'ALL {k:9s} hold median {np.median([r[k] for r in rows]):.2f} s  '
              f'mean {np.mean([r[k] for r in rows]):.2f} s')
    for c in sorted({r['clip'] for r in rows}):
        sub = [r for r in rows if r['clip'] == c]
        print(f'  {c:14s} moving windows {len(sub):3d}  follower median {np.median([r["follower"] for r in sub]):.2f} s'
              f'  stark median {np.median([r["stark"] for r in sub]):.2f} s')
    print(f'stark - follower, paired over {len(d)} windows: median {np.median(d):+.2f} s '
          f'(95 % {lo:+.2f} to {hi:+.2f}), mean {d.mean():+.2f} s; STARK call p50 '
          f'{np.median([r["stark_ms"] for r in rows]):.1f} ms')
    best_rung = min(np.median([r['follower'] for r in rows]), np.median([r['stark'] for r in rows]))
    held = np.median([r['frozen'] for r in rows]) <= best_rung
    print('FALSIFIER ' + ('ok: a frozen box holds least' if held else
                          'FAILED: a frozen box holds as long as a rung; the scorer is not measuring following'))
    ok = len(d) >= 30 and np.median(d) > 0 and lo > 0
    print(f'VERDICT accuracy clause: {"PASS, the chip-cost run decides" if ok else "FAIL, the follower stays"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

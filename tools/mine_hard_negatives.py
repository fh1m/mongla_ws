#!/usr/bin/env python3
"""Mine the frames the detector is most confidently WRONG about.

⛔ WHAT THIS IS FOR (B-60). `gate_sharks` fires a `gate` on 28 of 34
visually-clean clips -- 0.81 on bare pool floor, 0.89 on empty water, 0.60 on
pool tiles. No confidence bar separates those, because the score is not wrong:
the learned concept is. The class has latched onto pool structure -- lane
lines, floor seams, the wall/floor horizon -- which is present in every frame
of every clip.

A model learns that when its training set contains only frames that CONTAIN
the prop. It is never shown a pool with no gate in it, so "pool" and "gate"
are never separated. The fix is not tuning. It is showing it the negative.

⭐ HARD negatives, not random ones. A frame the model already ignores teaches
nothing. The valuable frames are the ones where it is confidently wrong, so
this keeps the HIGHEST-scoring detections from clips a human has confirmed do
not contain the class, and writes them with an EMPTY label file -- which is
how YOLO is told "there is nothing of any class here".

⚠ THE CLIP LIST IS AN ASSERTION, NOT A GUESS. `--negative` means somebody
looked at `tools/label_contact_sheet.py`'s sheet and confirmed the prop is
absent. Feeding it a clip that does contain the prop teaches the model to
suppress a true detection, which is a worse defect than the one being fixed.
That is not hypothetical: labelling from filenames already produced one wrong
call this week (a `rescue` "false positive" that was a real panel).

    python3 tools/mine_hard_negatives.py --model <model.pt> \\
        --negative <clip> [<clip> ...] --per-clip 60
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def resolve(spec: str) -> pathlib.Path:
    p = pathlib.Path(spec)
    if p.is_file():
        return p
    from archive_root import archive_root
    q = pathlib.Path(archive_root()) / spec
    if not q.is_file():
        raise SystemExit(f'no clip {spec}')
    return q


def mine(model, clip: pathlib.Path, per_clip: int, stride: int,
         limit: int, floor: float):
    """The frames this clip scores highest on -- the model's worst mistakes."""
    import cv2
    cap = cv2.VideoCapture(str(clip))
    hits = []
    n = 0
    while n < limit:
        ok, frame = cap.read()
        if not ok:
            break
        n += 1
        if n % stride:
            continue
        r = model.predict(frame, conf=floor, imgsz=640, verbose=False)[0]
        best = max((float(b.conf[0]) for b in r.boxes), default=0.0)
        if best > 0.0:
            hits.append((best, n, frame.copy()))
    cap.release()
    hits.sort(key=lambda h: -h[0])
    return hits[:per_clip]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--negative', nargs='+', required=True,
                    help='clips a human CONFIRMED do not contain the prop')
    ap.add_argument('--out', default='')
    ap.add_argument('--per-clip', type=int, default=60)
    ap.add_argument('--stride', type=int, default=7)
    ap.add_argument('--limit', type=int, default=1500)
    ap.add_argument('--floor', type=float, default=0.25,
                    help='ignore mistakes weaker than this; they teach little')
    a = ap.parse_args()

    import cv2
    from data_root import data_root
    from ultralytics import YOLO

    out = a.out or data_root('negatives')
    img_dir = os.path.join(out, 'images')
    lbl_dir = os.path.join(out, 'labels')
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(lbl_dir, exist_ok=True)

    model = YOLO(str(pathlib.Path(a.model)), task='detect')
    total = 0
    manifest = []
    print(f'mining hard negatives from {len(a.negative)} confirmed clips\n')
    for spec in a.negative:
        clip = resolve(spec)
        got = mine(model, clip, a.per_clip, a.stride, a.limit, a.floor)
        stem = clip.stem.replace(' ', '_')[:40]
        for score, frame_no, img in got:
            name = f'neg_{stem}_{frame_no:06d}'
            cv2.imwrite(os.path.join(img_dir, name + '.jpg'), img)
            # ⭐ AN EMPTY LABEL FILE IS THE WHOLE POINT. In YOLO an image with
            # a zero-byte .txt is a declared background: "nothing of any class
            # is here". Omitting the file instead would make the loader skip
            # the image entirely and teach nothing.
            open(os.path.join(lbl_dir, name + '.txt'), 'w').close()
            manifest.append({'clip': str(clip), 'frame': frame_no,
                             'was_scored': round(score, 4)})
            total += 1
        worst = got[0][0] if got else 0.0
        print(f'  {len(got):3d} frames   worst mistake {worst:.2f}   '
              f'{clip.name[:50]}')

    with open(os.path.join(out, 'manifest.json'), 'w') as fh:
        json.dump(manifest, fh, indent=1)
    print(f'\nwrote {total} background frames to {out}')
    print('  images/  the frames the model was most confidently wrong on')
    print('  labels/  one EMPTY .txt each -- YOLO reads that as "background"')
    print('\n⚠ Add these to the TRAINING split only. A background frame in the '
          'validation split inflates precision without proving anything, and '
          'the whole point is that this model already scores perfectly on a '
          'set that holds no background.')
    return 0


if __name__ == '__main__':
    sys.exit(main())

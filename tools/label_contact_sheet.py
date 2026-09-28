#!/usr/bin/env python3
"""Render every clip's strongest detection of one class, so a human can label.

⛔ WHY THIS EXISTS. Every detection bar in `measured-bars.md` section 1 was
derived from labelled held-out PAIRS -- frames that contain the prop. A set
built that way cannot measure a false positive on open water, because it holds
no open water. That is how B-59 hid: `gate_sharks` claims a `gate` on
90.8 % of gate-free frames at the shipped conf 0.15.

`negative_clip_check.py` fixes the measurement but needs NEGATIVE clips, and a
negative is a clip somebody LOOKED at. Labelling from filenames does not work
and I have already proved it: I marked `torpedo_1.mkv` a negative for `rescue`
and the tool reported 12.0 % of frames at conf 0.45 -- rendering the boxes
showed them tight around a real panel with two circular cut-outs. The
detections were right and the label was wrong.

So this produces the evidence a label needs: one tile per clip, carrying that
clip's HIGHEST-scoring detection of the class with the box drawn on it. If the
strongest example in a whole clip is empty water, the clip is a negative and
every weaker box in it is too.

⚠ It labels nothing itself. It renders; a person decides.

    python3 tools/label_contact_sheet.py --model <model.pt> --class gate
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

TILE_W, TILE_H, COLS = 426, 320, 4


def class_index(model_path: pathlib.Path, want: str) -> int:
    import yaml
    side = model_path.with_suffix('.yaml')
    if not side.exists():
        raise SystemExit(f'no sidecar {side.name}')
    names = yaml.safe_load(side.read_text()).get('names', {})
    for k, v in names.items():
        if str(v) == want:
            return int(k)
    raise SystemExit(f'class {want!r} not in {sorted(names.values())}')


def best_tile(model, clip: pathlib.Path, idx: int, label: str,
              stride: int, limit: int):
    """The frame carrying this clip's strongest detection of `idx`.

    ⭐ THE STRONGEST, not a random or a first hit. A label has to survive the
    clip's best case: if even the most confident example is empty water, the
    whole clip is a negative for that class.
    """
    import cv2
    cap = cv2.VideoCapture(str(clip))
    best = None
    n = 0
    while n < limit:
        ok, frame = cap.read()
        if not ok:
            break
        n += 1
        if n % stride:
            continue
        r = model.predict(frame, conf=0.15, imgsz=640, verbose=False)[0]
        for b in r.boxes:
            if int(b.cls[0]) != idx:
                continue
            s = float(b.conf[0])
            if best is None or s > best[0]:
                best = (s, frame.copy(), [float(v) for v in b.xyxy[0]], n)
    cap.release()
    if best is None:
        return None, 0.0
    score, frame, (x1, y1, x2, y2), at = best
    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)),
                  (0, 220, 255), 3)
    cv2.rectangle(frame, (0, 0), (frame.shape[1], 34), (0, 0, 0), -1)
    cv2.putText(frame, f'{label[:34]}  {score:.2f} f{at}', (6, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 2)
    return cv2.resize(frame, (TILE_W, TILE_H)), score


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--class', dest='cls', required=True)
    ap.add_argument('--inventory', default='')
    ap.add_argument('--out', default='')
    ap.add_argument('--stride', type=int, default=25)
    ap.add_argument('--limit', type=int, default=900)
    ap.add_argument('--max-clips', type=int, default=40)
    a = ap.parse_args()

    from data_root import data_root
    import cv2
    import numpy as np
    from ultralytics import YOLO

    inv = a.inventory or os.path.join(data_root('xfeat'),
                                      'footage_inventory.json')
    raw = json.load(open(inv))
    clips = [c for c in (raw if isinstance(raw, list) else raw.get('clips', []))
             if c.get('clean')][:a.max_clips]
    if not clips:
        raise SystemExit(f'no clean clips in {inv}')

    model_path = pathlib.Path(a.model)
    idx = class_index(model_path, a.cls)
    model = YOLO(str(model_path), task='detect')

    out_dir = a.out or data_root('labelling')
    tiles, rows = [], []
    print(f'{model_path.name}  class {a.cls!r} (index {idx})  '
          f'{len(clips)} clean clips\n')
    for c in clips:
        p = pathlib.Path(c['path'])
        name = p.name
        tile, score = best_tile(model, p, idx, name, a.stride, a.limit)
        rows.append((name, c.get('venue', '?'), score))
        print(f'  {score:.2f}  {c.get("venue","?")[:28]:<28} {name[:46]}')
        if tile is not None:
            tiles.append(tile)

    if not tiles:
        print(f'\nNo {a.cls!r} detection anywhere -- every clip is a negative.')
        return 0
    while len(tiles) % COLS:
        tiles.append(np.zeros((TILE_H, TILE_W, 3), np.uint8))
    sheet = np.vstack([np.hstack(tiles[i:i + COLS])
                       for i in range(0, len(tiles), COLS)])
    dst = os.path.join(out_dir, f'sheet_{a.cls}.jpg')
    cv2.imwrite(dst, sheet)
    print(f'\nwrote {dst}  ({len(rows)} clips)')
    print('⚠ LOOK AT IT. A clip whose STRONGEST box is empty water is a '
          'negative for this class; pass those to negative_clip_check.py '
          '--negative. Do not label from filenames.')
    return 0


if __name__ == '__main__':
    sys.exit(main())

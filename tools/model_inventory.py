#!/usr/bin/env python3
"""Every model on this box, what it detects, and whether it has been checked.

⛔ WHY THIS EXISTS. Four models ship in `src/mongla_vision/models/`. There are
**66 `.pt` files on this machine, 32 of them carrying a gate class**, spread
across `Music/detect/`, `Downloads/DUBURI_Models/`, three pendrive backups and
a 2026 season folder. Work proceeded for weeks on the four, while dedicated
single-class gate models sat unevaluated a directory away.

That is the same shape as the footage problem: a `find` from the repo root
does not reach `Work/`, `Music/detect` or `tmp/smol backups/`, so "we don't
have one" kept being concluded from a search that could not have found one.

⭐ IT DEDUPLICATES BY CONTENT. The same weights appear under several names --
`Music/detect/.../best.pt` and
`tmp/smol backups/pendrive_2/MODELS_USA/.../best.pt` are frequently
byte-identical. Ranking both wastes a GPU hour and reports a "tie" that is one
model counted twice.

⚠ `verified_negatives` IS THE COLUMN THAT MATTERS. A model is not usable
because it exists and has classes; it is usable when it has been measured on
footage where the prop is ABSENT (`tools/negative_clip_check.py`). B-60: the
shipped gate model scores **0.92 on a clip with no gate**, higher than the
0.89 it gives the real gate, and every bar in `measured-bars.md` section 1 was
derived without a single negative frame. Until that column is true, a model's
numbers describe only how it behaves when the answer is yes.

    python3 tools/model_inventory.py --prop gate
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
import pathlib
import sys
import warnings

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

# Every place models have actually been found. Adding a root here is cheaper
# than rediscovering the same gap a fourth time.
ROOTS = (
    '/home/fh1m/Envs/dockers/auv-ros2/Ros_workspaces/mongla_ws/'
    'src/mongla_vision/models',
    '/home/fh1m/Music/detect',
    '/home/fh1m/Downloads/DUBURI_Models',
    '/home/fh1m/Downloads',
    '/home/fh1m/tmp/smol backups',
    '/home/fh1m/Work/Projects/Duburi',
)

# Stock checkpoints that are not ours and detect nothing we care about.
_SKIP = ('yolo11n', 'yolov8n', 'yolov8s', 'yolov8m', 'yolo26n', 'last.pt')


def sha1(path: pathlib.Path, cap: int = 4 << 20) -> str:
    """Hash the first few MB. Full hashing 66 files costs minutes and the
    leading megabytes of two different checkpoints never collide in practice."""
    h = hashlib.sha1()
    with open(path, 'rb') as fh:
        h.update(fh.read(cap))
    return h.hexdigest()[:12]


def scan(roots):
    warnings.filterwarnings('ignore')
    from ultralytics import YOLO

    out, by_hash = [], {}
    for r in roots:
        root = pathlib.Path(r)
        if not root.exists():
            continue
        for p in sorted(root.rglob('*.pt')):
            if any(s in p.name for s in _SKIP):
                continue
            try:
                names = YOLO(str(p)).names
            except Exception:                                    # noqa: BLE001
                continue
            digest = sha1(p)
            out.append({
                'path': str(p),
                'classes': [str(v) for _k, v in sorted(names.items())],
                'sha1': digest,
                'size_mb': round(p.stat().st_size / 1e6, 1),
                'mtime': dt.date.fromtimestamp(p.stat().st_mtime).isoformat(),
                # ⛔ Nothing here may set this true. It becomes true only when
                # negative_clip_check.py has been run with visually-confirmed
                # negative clips and the result recorded.
                'verified_negatives': False,
                'duplicate_of': by_hash.get(digest),
            })
            by_hash.setdefault(digest, str(p))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--prop', default='',
                    help='only models with a class containing this substring')
    ap.add_argument('--out', default='')
    ap.add_argument('--roots', nargs='*', default=list(ROOTS))
    a = ap.parse_args()

    from data_root import data_root

    rows = scan(a.roots)
    if a.prop:
        rows = [r for r in rows
                if any(a.prop.lower() in c.lower() for c in r['classes'])]

    uniq = [r for r in rows if not r['duplicate_of']]
    dupes = len(rows) - len(uniq)
    print(f'{len(rows)} models'
          + (f' with a {a.prop!r} class' if a.prop else '')
          + f'   {len(uniq)} unique, {dupes} duplicate copies\n')

    props = {}
    for r in uniq:
        for c in r['classes']:
            props.setdefault(c, []).append(r)

    print(f'{"class":<28}{"models":>7}   newest')
    for c, rs in sorted(props.items(), key=lambda kv: -len(kv[1]))[:25]:
        newest = max(r['mtime'] for r in rs)
        print(f'{c[:27]:<28}{len(rs):>7}   {newest}')

    dst = os.path.join(a.out or data_root('models'), 'model_inventory.json')
    with open(dst, 'w') as fh:
        json.dump(rows, fh, indent=1)
    print(f'\nwrote {dst}')
    unver = sum(1 for r in uniq if not r['verified_negatives'])
    print(f'⚠ {unver}/{len(uniq)} unique models have NEVER been measured '
          f'against footage where the prop is ABSENT. Until that is done a '
          f"model's numbers describe only how it behaves when the answer is "
          f'yes -- which is how B-60 hid. Run tools/negative_clip_check.py.')
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3

"""Turn recorded sim runs into a YOLO dataset ready for `yolo train`.

`record_cameras --frames --labels` already writes frames and ground-truth YOLO
labels per run. What was missing was the last mechanical step -- the train/val
split, the directory layout ultralytics expects, and data.yaml -- so every
attempt to train on sim data started with the same half hour of file shuffling.

    ros2 run duburi_sim_bridge dataset_to_yolo \
        --runs sim_murky_* sim_clear_* --out ~/sim_yolo --camera front

Design notes worth keeping:

* The split is by RUN, not by frame. Consecutive frames from one recording are
  near-duplicates, so a random frame split puts near-copies of the same image in
  both halves and the validation score becomes a memorisation score. With one
  run it falls back to a contiguous tail split, which is still far better than
  random for the same reason.
* Empty label files are KEPT, not dropped. A frame where nothing is visible is a
  true negative, and YOLO needs those or it learns that every image contains a
  prop.
* Class indices come from each run's classes.txt and are verified to agree
  across runs. gt_labels.CLASSES is append-only for exactly this reason, but a
  dataset mixing runs from before and after a class was added would silently
  mislabel, so it is checked rather than assumed.
"""

from __future__ import annotations

import argparse
import glob
import os
import shutil
import sys
from pathlib import Path


def _runs(patterns: list[str], root: Path) -> list[Path]:
    out: list[Path] = []
    for pat in patterns:
        p = Path(pat)
        hits = [p] if p.is_absolute() or p.exists() else [
            Path(h) for h in sorted(glob.glob(str(root / pat)))
        ]
        out.extend(h for h in hits if (h / 'meta.json').is_file())
    seen, uniq = set(), []
    for r in out:
        if r.resolve() not in seen:
            seen.add(r.resolve())
            uniq.append(r)
    return uniq


def _classes(runs: list[Path]) -> list[str]:
    names = None
    for r in runs:
        f = r / 'classes.txt'
        if not f.is_file():
            raise SystemExit(f'{r.name}: no classes.txt -- recorded without --labels?')
        got = [ln.strip() for ln in f.read_text().splitlines() if ln.strip()]
        if names is None:
            names = got
        elif got != names:
            raise SystemExit(
                f'{r.name} has different classes than {runs[0].name}.\n'
                f'  {runs[0].name}: {names}\n  {r.name}: {got}\n'
                'Class indices are positional; mixing these would mislabel.'
            )
    return names or []


def _pairs(run: Path, camera: str) -> list[tuple[Path, Path]]:
    frames = run / 'frames' / camera
    labels = run / 'labels' / camera
    if not frames.is_dir():
        return []
    out = []
    for img in sorted(frames.iterdir()):
        if img.suffix.lower() not in ('.png', '.jpg', '.jpeg'):
            continue
        lab = labels / (img.stem + '.txt')
        # A missing label file is NOT the same as an empty one: empty means
        # "nothing visible here" and is a usable negative, missing means the
        # label pass did not run for this frame and the image is unusable.
        if lab.is_file():
            out.append((img, lab))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--runs', nargs='+', required=True,
                    help='Run directories or globs under --datasets.')
    ap.add_argument('--datasets', default='',
                    help='Where the runs live (default: <sim>/datasets).')
    ap.add_argument('--out', required=True, help='YOLO dataset directory to create.')
    ap.add_argument('--camera', default='front', choices=('front', 'bottom'))
    ap.add_argument('--val-frac', type=float, default=0.2)
    ap.add_argument('--link', action='store_true',
                    help='Hardlink instead of copying (same filesystem only).')
    a = ap.parse_args(argv)

    root = Path(a.datasets) if a.datasets else Path(__file__).resolve().parents[3] / 'datasets'
    runs = _runs(a.runs, root)
    if not runs:
        raise SystemExit(f'no recorded runs matched {a.runs} under {root}')
    names = _classes(runs)

    per_run = {r: _pairs(r, a.camera) for r in runs}
    per_run = {r: v for r, v in per_run.items() if v}
    if not per_run:
        raise SystemExit(f'no {a.camera} frames with labels in {[r.name for r in runs]}')

    if len(per_run) > 1:
        n_val = max(1, round(len(per_run) * a.val_frac))
        val_runs = list(per_run)[-n_val:]
        split = {'val' if r in val_runs else 'train': None for r in per_run}
        buckets = {'train': [], 'val': []}
        for r, pairs in per_run.items():
            buckets['val' if r in val_runs else 'train'].extend(pairs)
        how = f'by run ({len(per_run) - n_val} train / {n_val} val)'
    else:
        run, pairs = next(iter(per_run.items()))
        cut = int(len(pairs) * (1.0 - a.val_frac))
        buckets = {'train': pairs[:cut], 'val': pairs[cut:]}
        how = 'contiguous tail of a single run'

    out = Path(a.out).expanduser()
    for sub in ('images/train', 'images/val', 'labels/train', 'labels/val'):
        (out / sub).mkdir(parents=True, exist_ok=True)

    place = os.link if a.link else shutil.copyfile
    for split_name, pairs in buckets.items():
        for i, (img, lab) in enumerate(pairs):
            stem = f'{img.parent.parent.parent.name}_{i:06d}'
            for src, dst in ((img, out / f'images/{split_name}/{stem}{img.suffix}'),
                             (lab, out / f'labels/{split_name}/{stem}.txt')):
                if dst.exists():
                    dst.unlink()
                place(src, dst)

    (out / 'data.yaml').write_text(
        f'# GENERATED by dataset_to_yolo from {len(per_run)} recorded sim run(s).\n'
        f'path: {out}\ntrain: images/train\nval: images/val\n\n'
        f'nc: {len(names)}\nnames:\n' +
        ''.join(f'  {i}: {n}\n' for i, n in enumerate(names))
    )

    n_tr, n_va = len(buckets['train']), len(buckets['val'])
    empt = sum(1 for p in (out / 'labels/train').iterdir() if p.stat().st_size == 0)
    print(f'runs      : {", ".join(r.name for r in per_run)}')
    print(f'camera    : {a.camera}')
    print(f'split     : {how}')
    print(f'train/val : {n_tr} / {n_va} images   ({empt} train negatives)')
    print(f'classes   : {len(names)} -> {", ".join(names)}')
    print(f'\nwrote {out}/data.yaml\n\n  yolo detect train data={out}/data.yaml '
          f'model=yolo11n.pt epochs=60 imgsz=640')
    return 0


if __name__ == '__main__':
    sys.exit(main())

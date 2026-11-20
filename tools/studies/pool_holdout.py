#!/usr/bin/env python3
"""Hold one pool out: does a detector work in water it has never seen?

Why this exists. Every detector here scores near 0.98-0.995 mAP50 on a random
split, and on a random split the test frames are near-copies of training
frames from the same run. That measures memory. The question a competition
asks is whether the model finds the props in a pool it was not trained in,
and that needs a split by SOURCE: train on every pool but one, test on the
one held out. (Research report "YOLO on Hailo 8 model pipeline", item 1.)

What it builds. From several YOLO datasets (other teams' labelled pools, our
archive), one dataset with:

  * one coarse class set, by keyword, because every team names the props its
    own way ('Red pail', 'red_tub', 'Blue Bucket', 'ember-biru' are one prop).
    Colour stays out of the class: teams disagree on it, and the hue of a box
    is read after detection. Classes nothing maps are dropped, not guessed;
  * train = every image of the other sources (optionally capped per source),
    val = their own validation splits, test = every image of the held-out one;
  * no held-out image in train under another name. Teams fork each other's
    datasets, so the same frame can sit in two sources. Every training image
    whose 64-bit difference hash is within `--hamming` bits of a held-out one
    is dropped and counted. Without this the test leaks and reads high.

Images are symlinked, labels rewritten. The output is a `data.yaml` that
`yolo detect train` and `yolo detect val split=test` read directly.

    python3 tools/studies/pool_holdout.py --self-test
    python3 tools/studies/pool_holdout.py --holdout ntu \\
        --sources $MONGLA_DATA/datasets/universe/* --cap 3000

Falsifier for the dedupe, in the self-test: a planted copy of a held-out
image in a training source must be found and dropped, and a different image
must not be.
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import random
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # tools/

# Coarse class by keyword, first match wins. Order matters: 'gate pole' is a
# pole, not a gate; 'tubs' (a group box around several tubs) is not one drum.
COARSE = (
    ('pole', None),
    ('tubs', None),
    ('gate', 'gate'),
    ('flare', 'flare'),
    ('pail', 'drum'), ('bucket', 'drum'), ('tub', 'drum'), ('drum', 'drum'),
    ('ember', 'drum'),             # Indonesian for bucket
)
CLASSES = ('gate', 'flare', 'drum')
IMG_EXT = ('.jpg', '.jpeg', '.png', '.bmp')


def coarse(name: str):
    n = str(name).strip().lower().replace('-', ' ').replace('_', ' ')
    for key, out in COARSE:
        if key in n.split() or key in n:
            return out
    return None


def dhash(path) -> int | None:
    """64-bit difference hash: 9x8 grey, each pixel brighter than its right
    neighbour. Robust to re-encoding and small resizes, which is what a
    re-uploaded fork does to a frame."""
    import cv2
    im = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if im is None:
        return None
    s = cv2.resize(im, (9, 8), interpolation=cv2.INTER_AREA)
    bits = (s[:, 1:] > s[:, :-1]).flatten()
    return int(np.packbits(bits).view('>u8')[0])


def _popcount(x: np.ndarray) -> np.ndarray:
    x = x.astype(np.uint64)
    c = np.zeros(x.shape, dtype=np.uint64)
    while x.any():
        c += x & np.uint64(1)
        x >>= np.uint64(1)
    return c


def near(h: int, bank: np.ndarray, bits: int) -> bool:
    if bank.size == 0:
        return False
    return bool((_popcount(bank ^ np.uint64(h)) <= bits).any())


def source_items(root: pathlib.Path):
    """(split, image, label path, {source class id: coarse name or None})."""
    import yaml
    y = yaml.safe_load((root / 'data.yaml').read_text())
    names = y['names']
    names = names if isinstance(names, list) else [names[k] for k in sorted(names)]
    cmap = {i: coarse(n) for i, n in enumerate(names)}
    out = []
    for split in ('train', 'valid', 'val', 'test'):
        idir = root / split / 'images'
        if not idir.is_dir():
            continue
        for img in sorted(idir.iterdir()):
            if img.suffix.lower() in IMG_EXT:
                out.append((split, img, root / split / 'labels' / (img.stem + '.txt')))
    return out, cmap


def rewrite(label: pathlib.Path, cmap: dict) -> list[str]:
    lines = []
    if label.exists():
        for line in label.read_text().splitlines():
            p = line.split()
            if len(p) != 5:                  # polygons or junk: not a box
                continue
            c = cmap.get(int(float(p[0])))
            if c is not None:
                lines.append(' '.join([str(CLASSES.index(c))] + p[1:]))
    return lines


def build(sources, holdout: str, out: pathlib.Path, cap: int = 0,
          hamming: int = 4, seed: int = 0) -> dict:
    sources = [pathlib.Path(s) for s in sources]
    held = [s for s in sources if holdout in s.name]
    if len(held) != 1:
        raise SystemExit(f'--holdout {holdout!r} matches {[s.name for s in held]}; need one')
    held = held[0]
    rng = random.Random(seed)
    for sub in ('train', 'val', 'test'):
        (out / sub / 'images').mkdir(parents=True, exist_ok=True)
        (out / sub / 'labels').mkdir(parents=True, exist_ok=True)
    report = {'holdout': held.name, 'classes': list(CLASSES), 'sources': {},
              'dropped_as_holdout_copies': 0}

    def place(sub, src, img, lines, tag):
        name = f'{tag}__{img.name}'
        dst = out / sub / 'images' / name
        if not dst.exists():
            os.symlink(img.resolve(), dst)
        (out / sub / 'labels' / (pathlib.Path(name).stem + '.txt')).write_text(
            '\n'.join(lines) + ('\n' if lines else ''))

    items, cmap = source_items(held)
    bank = []
    for split, img, lab in items:
        place('test', held, img, rewrite(lab, cmap), held.name)
        h = dhash(img)
        if h is not None:
            bank.append(h)
    bank = np.array(bank, dtype=np.uint64)
    report['sources'][held.name] = {'test': len(items),
                                    'map': {str(k): v for k, v in cmap.items()}}

    for src in sources:
        if src == held:
            continue
        items, cmap = source_items(src)
        tr = [i for i in items if i[0] == 'train']
        va = [i for i in items if i[0] in ('valid', 'val')]
        rng.shuffle(tr)
        if cap:
            tr = tr[:cap]
        kept = {'train': 0, 'val': 0, 'dropped': 0}
        for sub, group in (('train', tr), ('val', va)):
            for _, img, lab in group:
                h = dhash(img)
                if h is not None and near(h, bank, hamming):
                    kept['dropped'] += 1
                    continue
                place(sub, src, img, rewrite(lab, cmap), src.name)
                kept[sub] += 1
        report['dropped_as_holdout_copies'] += kept['dropped']
        kept['map'] = {str(k): v for k, v in cmap.items()}
        report['sources'][src.name] = kept

    (out / 'data.yaml').write_text(
        f'path: {out}\ntrain: train/images\nval: val/images\ntest: test/images\n'
        'names:\n' + ''.join(f'  {i}: {c}\n' for i, c in enumerate(CLASSES)))
    (out / 'holdout.json').write_text(json.dumps(report, indent=1))
    return report


def _self_test() -> int:
    import cv2
    rng = np.random.default_rng(0)
    with tempfile.TemporaryDirectory() as td:
        td = pathlib.Path(td)

        def make(name, names, frames):
            for split in ('train', 'valid'):
                (td / name / split / 'images').mkdir(parents=True)
                (td / name / split / 'labels').mkdir(parents=True)
            (td / name / 'data.yaml').write_text(
                'names: [' + ', '.join(repr(n) for n in names) + ']\n')
            for i, (split, im, lab) in enumerate(frames):
                cv2.imwrite(str(td / name / split / 'images' / f'f{i}.jpg'), im)
                (td / name / split / 'labels' / f'f{i}.txt').write_text(lab)

        def img():
            return (rng.random((64, 96, 3)) * 255).astype(np.uint8)

        held_frame = img()
        make('team_a', ['Gate', 'Red pail', 'Cloth'],
             [('train', held_frame, '0 .5 .5 .2 .2\n1 .3 .3 .1 .1\n2 .1 .1 .1 .1\n')])
        copy = cv2.resize(cv2.resize(held_frame, (80, 54)), (96, 64))  # a re-encoded fork
        make('team_b', ['gate pole', 'flare', 'blue_tub', 'tubs'],
             [('train', copy, '1 .5 .5 .1 .1\n'),
              ('train', img(), '0 .5 .5 .1 .1\n1 .2 .2 .1 .1\n2 .7 .7 .1 .1\n3 .5 .5 .9 .9\n'),
              ('valid', img(), '2 .4 .4 .1 .1\n')])
        rep = build([td / 'team_a', td / 'team_b'], 'team_a', td / 'out')
        ok = True

        def check(cond, what):
            nonlocal ok
            print(('ok   ' if cond else 'FAIL ') + what)
            ok &= bool(cond)

        check(rep['dropped_as_holdout_copies'] == 1, 'the re-encoded copy of a held-out frame is dropped')
        check(rep['sources']['team_b']['train'] == 1, 'the other training frame is kept')
        test_lab = (td / 'out/test/labels/team_a__f0.txt').read_text().split('\n')
        check(test_lab[:2] == ['0 .5 .5 .2 .2', '2 .3 .3 .1 .1'] and len([x for x in test_lab if x]) == 2,
              'Gate -> gate, Red pail -> drum, Cloth dropped')
        tr = (td / 'out/train/labels').glob('*.txt')
        lab = next(iter(tr)).read_text().split('\n')
        check([x for x in lab if x] == ['1 .2 .2 .1 .1', '2 .7 .7 .1 .1'],
              "'gate pole' and the 'tubs' group box dropped; flare and blue_tub kept")
        check(coarse('ember-biru') == 'drum' and coarse('Blue Bucket') == 'drum'
              and coarse('gate-loreng') == 'gate' and coarse('golf-merah') is None,
              'other teams\' names map')
        print('self-test', 'PASSED' if ok else 'FAILED')
        return 0 if ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--sources', nargs='+')
    ap.add_argument('--holdout', help='substring naming the one source to hold out')
    ap.add_argument('--out', help='output dir (default: <data root>/holdout/<holdout>)')
    ap.add_argument('--cap', type=int, default=0, help='max training images per source (0 = all)')
    ap.add_argument('--hamming', type=int, default=4)
    ap.add_argument('--self-test', action='store_true')
    a = ap.parse_args(argv)
    if a.self_test:
        return _self_test()
    if not (a.sources and a.holdout):
        ap.error('--sources and --holdout are required')
    if a.out:
        out = pathlib.Path(a.out)
    else:
        from data_root import data_root
        out = pathlib.Path(data_root('holdout', a.holdout))
    rep = build(a.sources, a.holdout, out, cap=a.cap, hamming=a.hamming)
    print(json.dumps({k: (v if k != 'sources' else
                          {s: {x: y for x, y in d.items() if x != 'map'} for s, d in v.items()})
                      for k, v in rep.items()}, indent=1))
    print(f'data: {out / "data.yaml"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

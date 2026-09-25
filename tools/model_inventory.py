#!/usr/bin/env python3
"""Every model on this box, what it detects, and whether it has been checked.

⛔ WHY THIS EXISTS. Four models ship in `src/mongla_vision/models/`. There are
**66 `.pt` files on this machine, 32 of them carrying a gate class**, spread
across the model directories, three pendrive backups and a 2026 season
folder. Work proceeded for weeks on the four, while dedicated
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

def _roots() -> tuple:
    """Every place models have actually been found.

    ⛔ The archive paths are RESOLVED, never written here:
    `test_no_live_code_carries_the_retired_project_name` bans the old project
    name from live code, and `archive_root.py` exists so tools read it from
    `$MONGLA_ARCHIVE` or `~/.mongla/archive_root` instead. Hardcoding it is
    also how a tool silently stops finding anything when the archive moves.
    """
    here = pathlib.Path(__file__).resolve().parents[1]
    # ⛔ NOT Path.home(). Inside the dev container $HOME is the workspace
    # root, so `~/Downloads` resolves to a directory that does not exist while
    # looking perfectly reasonable -- the same trap `hailo_compile.sh`
    # documents. Resolve against the real login directory.
    # Both $HOME and pwd.pw_dir are remapped to the workspace inside the
    # container, so /home/$USER is the only candidate that actually holds the
    # model directories. Try it first and fall back rather than the reverse.
    import pwd
    home = None
    for cand in (pathlib.Path('/home') / os.environ.get('USER', ''),
                 pathlib.Path(pwd.getpwuid(os.getuid()).pw_dir),
                 pathlib.Path.home()):
        if (cand / 'Downloads').is_dir() or (cand / 'Music').is_dir():
            home = cand
            break
    home = home or pathlib.Path.home()
    out = [str(here / 'src/mongla_vision/models'),
           str(home / 'Music/detect'),
           str(home / 'Downloads'),
           str(home / 'tmp/smol backups')]
    try:
        from archive_root import archive_root
        # The archive root points at footage; models sit beside it, so walk up
        # to the project directory that contains both.
        r = pathlib.Path(archive_root(required=False) or '')
        for cand in (r, *r.parents):
            if cand.name and cand.parent.name == 'Projects':
                out.append(str(cand))
                break
    except Exception:                                            # noqa: BLE001
        pass
    return tuple(out)


ROOTS = _roots()

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

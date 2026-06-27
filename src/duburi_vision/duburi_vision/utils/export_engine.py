#!/usr/bin/env python3
"""export_engine -- build TensorRT FP16 engines from the .pt models.

Run this ON THE JETSON (not in docker / not on the dev box). A TensorRT
engine is device- and TRT/JetPack-version-locked, so it MUST be built on the
machine that will run inference, and rebuilt after a JetPack/TensorRT upgrade.

The detector prefers a ``<stem>.engine`` over ``<stem>.pt`` automatically
(see ``yolo._resolve_model_path``), so once the engine exists beside the .pt
the next launch uses the fast path -- confirm with the ``[YOLO ] backend=
TensorRT engine`` log line. On a dev box without an engine the detector falls
back to the .pt transparently.

Usage::

    # one model
    ros2 run duburi_vision export_engine gate_flare_medium_100ep
    # several
    ros2 run duburi_vision export_engine gate_nano_100ep flare_medium_100ep
    # every .pt in src/duburi_vision/models/
    ros2 run duburi_vision export_engine --all
    # custom input size (must match the detector's imgsz)
    ros2 run duburi_vision export_engine --all --imgsz 640

FP16 (half=True), never INT8 -- INT8 needs a calibration set and craters
small-target recall (the torpedo hole). imgsz is baked into the engine at
export and must match the detector's ``imgsz`` param at runtime.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from duburi_vision.detection.yolo import _find_src_models_dir


def _models_dir() -> Path:
    d = _find_src_models_dir()
    if d is None:
        print("[EXPORT] could not locate src/duburi_vision/models/", file=sys.stderr)
        sys.exit(2)
    return d


def _export_one(stem_pt: Path, imgsz: int) -> bool:
    from ultralytics import YOLO
    out = stem_pt.with_suffix('.engine')
    print(f"[EXPORT] {stem_pt.name} -> {out.name}  (imgsz={imgsz}, FP16) ...")
    try:
        YOLO(str(stem_pt)).export(
            format='engine', half=True, imgsz=imgsz, device=0)
    except Exception as exc:   # noqa: BLE001 -- report and continue to next model
        print(f"[EXPORT] FAILED {stem_pt.name}: {exc!r}", file=sys.stderr)
        return False
    # Ultralytics writes <stem>.engine next to the .pt by default.
    if out.exists():
        print(f"[EXPORT] OK  {out}")
        return True
    print(f"[EXPORT] export ran but {out.name} not found", file=sys.stderr)
    return False


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog='export_engine')
    ap.add_argument('stems', nargs='*',
                    help='model stems in models/ (without .pt), e.g. gate_nano_100ep')
    ap.add_argument('--all', action='store_true',
                    help='export every .pt in src/duburi_vision/models/')
    ap.add_argument('--imgsz', type=int, default=640,
                    help='input size baked into the engine (match detector imgsz; default 640)')
    args = ap.parse_args(argv)

    models_dir = _models_dir()
    if args.all:
        targets = sorted(models_dir.glob('*.pt'))
    else:
        if not args.stems:
            ap.error('pass model stems or --all')
        targets = [models_dir / f'{s}.pt' for s in args.stems]

    missing = [t for t in targets if not t.exists()]
    if missing:
        for m in missing:
            print(f"[EXPORT] not found: {m}", file=sys.stderr)
        return 2
    if not targets:
        print(f"[EXPORT] no .pt models in {models_dir}", file=sys.stderr)
        return 2

    ok = sum(_export_one(t, args.imgsz) for t in targets)
    print(f"[EXPORT] done: {ok}/{len(targets)} engine(s) built")
    return 0 if ok == len(targets) else 1


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""Compile an ultralytics checkpoint to a HEF, reproducibly.

⚠ RUN IT THROUGH `tools/hailo_compile.sh`, never directly. That wrapper owns
the venv and clears `PYTHONPATH`, and without it the DFC silently imports the
ROS overlay's onnx and dies claiming `onnx.mapping` is missing.

    ./tools/hailo_compile.sh gate_rescue_repair --nms-score-th 0.05

⭐ WHY THIS TOOL EXISTS. A HEF bakes its NMS score threshold into the on-chip
post-process; no ROS parameter can lower it, because the chip has already
thrown those boxes away. Parsing all four deployed HEFs on 2026-09-24 found
`sauvc_sim` and `bin_fire_blood` at 0.050, stock `yolov11n` at 0.200 -- and
`gate_rescue_repair`, the COMPETITION model, at 0.200. Every host-side bar
below 0.20 was decoration on that model, including the murky profile's 0.10
and a measured underwater score p10 of 0.167 the chip could not emit (B-58).

Nothing caught it because a threshold set too low never errors. It simply
stops changing anything, which is indistinguishable from a bar that was
already permissive enough. So this tool PRINTS the baked threshold it compiled
and tells you to parse the result -- a compile that succeeds is not a result.

THE STEPS, and why each is where it is:

  1. export   .pt -> .onnx    ultralytics, opset 11, nms=False
  2. discover end nodes       the six detect-head convolutions, READ from the
                              graph rather than hardcoded -- they are at
                              `model.22` for YOLOv8 and `model.23` for YOLO11,
                              and a wrong guess yields a HEF that compiles and
                              detects nothing
  3. parse    .onnx -> .har   cut at those nodes; the decode belongs to Hailo
  4. optimise                 INT8, calibrated on REAL frames
  5. compile  .har -> .hef

⛔ THE CALIBRATION SET IS NOT A FORMALITY. INT8 quantisation picks its scales
from whatever it is shown. Calibrating on clean or synthetic imagery and then
flying in murky water is how a model loses its faintest detections -- exactly
the ones the low score threshold exists to keep. Frames come from the real
archive via `archive_root.py`, and this refuses to run without them rather
than quietly substituting random noise.
"""
from __future__ import annotations

import argparse
import os
import pathlib
import random
import re
import subprocess
import sys

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))

# The six detect-head convolutions, in the order Hailo expects: the bbox
# regression and class branch of each scale, interleaved.
_HEAD = re.compile(r'/cv[23]\.(\d+)/cv[23]\.\d+\.2/Conv$')
_VIDEO = ('.mp4', '.mkv', '.mov', '.avi', '.MP4', '.MKV')


def export_onnx(pt: pathlib.Path, imgsz: int) -> pathlib.Path:
    """ultralytics lives in the SYSTEM env, not the DFC venv -- the two have
    incompatible numpy pins -- so the export runs out of process."""
    out = pt.with_suffix('.onnx')
    if out.exists():
        print(f'[hailo] reusing {out.name}')
        return out
    print(f'[hailo] exporting {pt.name} -> onnx')
    subprocess.run(
        ['python3', '-c',
         'import sys;from ultralytics import YOLO;'
         'YOLO(sys.argv[1]).export(format="onnx", imgsz=int(sys.argv[2]),'
         ' opset=11, simplify=True, nms=False, dynamic=False)',
         str(pt), str(imgsz)],
        check=True, env={k: v for k, v in os.environ.items()
                         if k != 'PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'})
    return out


def end_nodes(onnx_path: pathlib.Path) -> list:
    """Read the cut points out of the graph.

    ⛔ Hardcoding these is how you get a HEF that compiles cleanly and detects
    nothing: the head sits at `model.22` on YOLOv8 and `model.23` on YOLO11,
    and nothing downstream complains about cutting in the wrong place.
    """
    import onnx
    g = onnx.load(str(onnx_path)).graph
    found = [n.name for n in g.node
             if n.op_type == 'Conv' and _HEAD.search(n.name)]
    if len(found) != 6:
        raise SystemExit(
            f'[hailo] REFUSING: found {len(found)} detect-head convolutions, '
            f'expected 6. This export does not have the shape this tool knows '
            f'how to cut. Inspect it before compiling:\n  '
            + '\n  '.join(found))
    # Sort by scale, then bbox branch (cv2) before class branch (cv3).
    return sorted(found, key=lambda n: (int(_HEAD.search(n).group(1)),
                                        '/cv3.' in n))


def calib_frames(n: int, imgsz: int):
    """Real frames, sampled across the whole archive rather than one clip.

    Drawing them from one video would calibrate on one venue's water, which is
    the same mistake as calibrating on synthetic imagery wearing a disguise.
    """
    import cv2
    import numpy as np
    from archive_root import archive_root

    root = pathlib.Path(archive_root())
    clips = [p for p in root.rglob('*') if p.suffix in _VIDEO]
    if not clips:
        raise SystemExit(
            f'[hailo] REFUSING: no video under {root}. INT8 scales taken from '
            f'synthetic data lose the faintest detections in real water, so '
            f'this will not substitute noise. Set MONGLA_ARCHIVE.')
    random.Random(0).shuffle(clips)
    out, per = [], max(1, n // min(len(clips), 40) + 1)
    for clip in clips:
        if len(out) >= n:
            break
        cap = cv2.VideoCapture(str(clip))
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        if total > 1:
            for k in range(per):
                cap.set(cv2.CAP_PROP_POS_FRAMES, int(total * (k + 0.5) / per))
                ok, img = cap.read()
                if ok:
                    out.append(cv2.resize(img, (imgsz, imgsz))[:, :, ::-1])
        cap.release()
    if len(out) < n // 2:
        raise SystemExit(f'[hailo] REFUSING: only {len(out)} readable frames '
                         f'of {n} requested across {len(clips)} clips.')
    print(f'[hailo] calibration: {len(out)} frames from {len(clips)} clips '
          f'under {root}')
    # ⛔ uint8, NOT float32. 1024 x 640 x 640 x 3 is 1.26 GB as uint8 and
    # 5.0 GB as float32 -- and it was that 5 GB which pushed me to cut the set
    # to 256 frames, crossing the DFC's 1024 optimisation cliff and producing
    # the corrupted HEF in B-61. The DFC casts internally, so staging narrow
    # costs nothing and removes the reason to shrink the set.
    return np.stack(out[:n]).astype('uint8')


def class_names(pt: pathlib.Path) -> list:
    import yaml
    side = pt.with_suffix('.yaml')
    if not side.exists():
        raise SystemExit(
            f'[hailo] REFUSING: no sidecar {side.name}. A .hef carries no '
            f'names table, so without it the detector allowlist is empty and '
            f'it returns [] every frame while looking healthy.')
    d = yaml.safe_load(side.read_text())
    names = d.get('names', d)
    if isinstance(names, dict):
        return [names[k] for k in sorted(names)]
    return list(names)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('model', help='name under --models, e.g. gate_rescue_repair')
    ap.add_argument('--models', default=str(
        _HERE.parent / 'src/mongla_vision/models'))
    ap.add_argument('--out', default=str(pathlib.Path.home() / 'hailo_work'))
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--nms-score-th', type=float, default=0.05)
    ap.add_argument('--nms-iou-th', type=float, default=0.70)
    # ⛔ 1024 IS A CLIFF, NOT A GUIDELINE. Below it the DFC prints
    #
    #   [warning] Reducing optimization level to 1 (the accuracy won't be
    #   optimized and compression won't be used) because there's less data
    #   than the recommended amount (1024)
    #
    # and silently drops AdaRound and Quantization-Aware Fine-Tuning. The HEF
    # still compiles, still loads, still runs at full speed -- and returns
    # corrupted scores. Measured: a 258-frame build scored `repair` 0.86 on
    # every frame of gate.mkv where the .pt scored `gate` 0.53-0.89 (B-61).
    #
    # ⚠ I set this to 256 myself to avoid an OOM, which is how the cliff got
    # crossed. The memory problem is real -- 1024 x 640 x 640 x 3 float32 is
    # 5.0 GB resident -- and the fix is uint8 staging in `calib_frames`, not a
    # smaller set. `--allow-reduced-optimization` exists for a deliberate
    # throwaway build and says so in the log.
    ap.add_argument('--calib-frames', type=int, default=1024)
    ap.add_argument('--allow-reduced-optimization', action='store_true',
                    help='proceed below 1024 frames; the HEF will have '
                         'corrupted scores and must not be deployed')
    a = ap.parse_args()

    pt = pathlib.Path(a.models) / f'{a.model}.pt'
    if not pt.exists():
        raise SystemExit(f'[hailo] no checkpoint {pt}')
    out = pathlib.Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    names = class_names(pt)
    onnx_path = export_onnx(pt, a.imgsz)
    ends = end_nodes(onnx_path)
    print(f'[hailo] {a.model}: {len(names)} classes {names}')
    print('[hailo] cutting at:\n  ' + '\n  '.join(ends))

    from hailo_sdk_client import ClientRunner

    runner = ClientRunner(hw_arch='hailo8')
    runner.translate_onnx_model(
        str(onnx_path), a.model,
        start_node_names=['images'],
        end_node_names=ends,
        net_input_shapes={'images': [1, 3, a.imgsz, a.imgsz]})

    har = out / f'{a.model}_parsed.har'
    runner.save_har(str(har))
    print(f'[hailo] parsed -> {har.name}')

    # ⭐ THE THRESHOLD THIS WHOLE TOOL IS ABOUT. It is compiled in here and
    # cannot be changed afterwards by any runtime setting.
    script = (
        'normalization1 = normalization([0.0, 0.0, 0.0], '
        '[255.0, 255.0, 255.0])\n'
        f'nms_postprocess(meta_arch=yolov8, engine=cpu, '
        f'classes={len(names)}, '
        f'nms_scores_th={a.nms_score_th}, nms_iou_th={a.nms_iou_th})\n')
    runner.load_model_script(script)
    print(f'[hailo] model script:\n{script}')

    # ⛔ THE CLIFF, REFUSED BEFORE THE HOUR IS SPENT. Issue #47 asked for a
    # build guard on the DFC's "Reducing optimization level" warning; this is
    # it, checked on the input rather than scraped from the log, so it fires
    # before the 40-minute optimise rather than after.
    if a.calib_frames < 1024 and not a.allow_reduced_optimization:
        raise SystemExit(
            f'[hailo] REFUSING: {a.calib_frames} calibration frames is below '
            f'the DFC\'s 1024 threshold. Below it the compiler drops AdaRound '
            f'and Quantization-Aware Fine-Tuning, and the HEF compiles, loads '
            f'and runs at full speed while returning CORRUPTED SCORES -- a '
            f'258-frame build read `repair` 0.86 on every frame where the .pt '
            f'read `gate` 0.53-0.89 (B-61). Raise --calib-frames, or pass '
            f'--allow-reduced-optimization for a throwaway build that must '
            f'not be deployed.')
    if a.calib_frames < 1024:
        print('[hailo] ⚠ REDUCED OPTIMIZATION requested. This HEF will have '
              'corrupted scores. Do not deploy it.')

    runner.optimize(calib_frames(a.calib_frames, a.imgsz))
    opt = out / f'{a.model}_optimized.har'
    runner.save_har(str(opt))
    print(f'[hailo] optimised -> {opt.name}')

    hef = out / f'{a.model}.hef'
    hef.write_bytes(runner.compile())
    print(f'[hailo] wrote {hef} ({hef.stat().st_size / 1e6:.1f} MB)')

    # ⚠ A HEF THAT COMPILES IS NOT A RESULT. The threshold is the reason this
    # was run, so verify it off the artifact rather than trusting the request.
    print(f'\n[hailo] VERIFY before deploying -- a compile that succeeded '
          f'proves nothing about the number you asked for:\n'
          f'  hailortcli parse-hef {hef}\n'
          f'  expect: Score threshold: {a.nms_score_th:.3f}   '
          f'Classes: {len(names)}\n'
          f'  then re-run the conf sweep ON THE HEF, since every bar in '
          f'measured-bars.md section 1 was taken on the ONNX path.')
    return 0


if __name__ == '__main__':
    sys.exit(main())

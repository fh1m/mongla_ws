#!/usr/bin/env python3
"""Compile an ultralytics checkpoint to a HEF, reproducibly.

Run it through `tools/bench/hailo_compile.sh`, never directly. That wrapper owns
the venv and clears `PYTHONPATH`, and without it the DFC silently imports the
ROS overlay's onnx and dies claiming `onnx.mapping` is missing.

    ./tools/bench/hailo_compile.sh gate_sharks --nms-score-th 0.05

Why this tool exists. A HEF bakes its NMS score threshold into the on-chip
post-process; no ROS parameter can lower it, because the chip has already
thrown those boxes away. Parsing all four deployed HEFs on 2026-09-24 found
`sauvc_sim` and `bin_fire_blood` at 0.050, stock `yolov11n` at 0.200 -- and
`gate_sharks`, the COMPETITION model, at 0.200. Every host-side bar
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

The calibration set is not a formality. INT8 quantisation picks its scales
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
sys.path.insert(1, str(_HERE.parent))  # tools/: shared libraries

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

    Hardcoding these is how you get a HEF that compiles cleanly and detects
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


def calib_frames(n: int, imgsz: int, seed: int = 0):
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
    random.Random(seed).shuffle(clips)
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
    # uint8, NOT float32. 1024 x 640 x 640 x 3 is 1.26 GB as uint8 and
    # 5.0 GB as float32 -- and it was that 5 GB which pushed me to cut the set
    # to 256 frames, crossing the DFC's 1024 optimisation cliff and producing
    # the corrupted HEF in B-61. The DFC casts internally, so staging narrow
    # costs nothing and removes the reason to shrink the set.
    return np.stack(out[:n]).astype('uint8')


DFC_MIN_CALIB = 1024
_DOWNGRADE = re.compile(r'Reducing optimization level to (\d+)')


def downgrade_reasons(*, frames: int, gpu, log_lines=()) -> list:
    """Every reason this build would silently lose optimisation (issue #47).

    The DFC drops AdaRound and fine-tuning in TWO cases, warns, and carries on:
    fewer than 1024 calibration images, or no GPU. Either way the HEF compiles,
    loads, runs at full speed, and returns degraded scores. Checked three ways,
    because each alone has a hole:
      - the frames ACTUALLY DELIVERED, not requested (`calib_frames` accepts
        as few as half of what was asked);
      - the GPU, BEFORE the hour-long optimise (`gpu` None = could not tell);
      - the DFC's own warning, scraped from its log AFTER, which catches any
        reason this list does not know about.
    """
    why = []
    if frames < DFC_MIN_CALIB:
        why.append(f'{frames} calibration frames delivered, below the DFC\'s '
                   f'{DFC_MIN_CALIB}')
    if gpu is False:
        why.append('no GPU visible: the DFC drops to optimisation level 0')
    for line in log_lines:
        m = _DOWNGRADE.search(str(line))
        if m:
            why.append(f'the DFC reported: {m.group(0)}')
    return why


# The other way a full build goes wrong: The optimiser itself (2026-11-06).
# The shipped gate_sharks.hef passed every downgrade check -- 1038 frames, a
# GPU, level 2 -- and its Quantization-Aware Fine-Tuning DIVERGED: distill
# losses rose 0.13 -> 0.26 across epochs and conv54 sat at 1.000. On the DFC
# emulator that build loses a median 0.339 of confidence on gate footage
# where a level-0 build of the same model loses -0.010
# (tools/bench/hailo_int8_shift.py, measured-bars). Nothing warned. So the optimised
# model is checked against its own float self on frames it was NOT
# calibrated on, and a large shift refuses the compile.
#
# 0.10: the healthy builds measured so far sit at -0.01 (emulator, level 0)
# and 0.04-0.06 (chip vs .pt, gate_rescue_repair); the diverged one at 0.34.
MAX_INT8_SHIFT = 0.10
MIN_VERIFY_FRAMES = 16


def int8_shift_reasons(stats: dict, max_shift: float = MAX_INT8_SHIFT) -> list:
    """Why an optimised model's INT8 scores cannot be trusted, from the
    `hailo_int8_shift.shift_stats` of fp vs quantised on held-out frames."""
    if stats.get('n', 0) < MIN_VERIFY_FRAMES:
        return [f'only {stats.get("n", 0)} held-out frames scored >= 0.25 on '
                f'the float path (need {MIN_VERIFY_FRAMES}): INT8 accuracy is '
                f'UNVERIFIED -- pass --verify-clip footage containing a target']
    if stats['median'] > max_shift:
        return [f'INT8 loses a median {stats["median"]:.3f} of confidence '
                f'against the float model (bar {max_shift:.2f}, p90 '
                f'{stats["p90"]:.3f}): the optimiser degraded this build']
    return []


def _gpu_visible():
    """True / False if TensorFlow (which the DFC runs on) can say; else None."""
    try:
        import tensorflow as tf
        return bool(tf.config.list_physical_devices('GPU'))
    except Exception:                       # noqa: BLE001 -- unknown, not absent
        return None


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
    ap.add_argument('model', help='name under --models, e.g. gate_sharks')
    ap.add_argument('--models', default=str(
        _HERE.parents[1] / 'src/mongla_vision/models'))
    ap.add_argument('--out', default=str(pathlib.Path.home() / 'hailo_work'))
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--nms-score-th', type=float, default=0.05)
    ap.add_argument('--nms-iou-th', type=float, default=0.70)
    # 1024 is a cliff, not a guideline. Below it the DFC prints
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
    # I set this to 256 myself to avoid an OOM, which is how the cliff got
    # crossed. The memory problem is real -- 1024 x 640 x 640 x 3 float32 is
    # 5.0 GB resident -- and the fix is uint8 staging in `calib_frames`, not a
    # smaller set. `--allow-reduced-optimization` exists for a deliberate
    # throwaway build and says so in the log.
    ap.add_argument('--calib-frames', type=int, default=1024)
    ap.add_argument('--allow-reduced-optimization', action='store_true',
                    help='proceed below 1024 frames; the HEF will have '
                         'corrupted scores and must not be deployed')
    # The escape hatch for a layer that will not quantise. Some models fail
    # with, for example:
    #
    #   NegativeSlopeExponentNonFixable: Quantization failed in layer
    #   <model>/conv54 due to unsupported required slope. Desired shift is
    #   8.0, but op has only 8 data bits.
    #
    # That means the layer's weight/activation range is too wide for INT8 --
    # a property of how the model was TRAINED (unfolded batch-norm, or an
    # unusually wide activation), not of the calibration set. It is NOT fixed
    # by more frames: `gate_candidate` failed on the SAME 1024-frame set that
    # compiled `gate_sharks` cleanly, which is what rules the
    # calibration set out as the cause.
    #
    # Raising just that layer to 16-bit costs a little throughput on one layer
    # and leaves the rest INT8. Name the layer the error names; blanket
    # 16-bit would quietly discard the accelerator's main advantage.
    ap.add_argument('--verify-clip', action='append', default=[],
                    help='footage containing a target, for the INT8 check '
                         'after the optimise (default: a fresh archive sample)')
    ap.add_argument('--allow-int8-shift', action='store_true',
                    help='compile even if INT8 loses more than '
                         f'{MAX_INT8_SHIFT} of confidence; must not be deployed')
    ap.add_argument('--finetune', choices=('on', 'off'), default='on',
                    help='off disables Quantization-Aware Fine-Tuning, for a '
                         'model whose fine-tune diverges (see MAX_INT8_SHIFT)')
    ap.add_argument('--a16-layers', default='',
                    help='comma-separated layers to keep at 16-bit, for ones '
                         'that fail INT8 quantisation (see the error text)')
    a = ap.parse_args()

    pt = (pathlib.Path(a.models) / f'{a.model}.pt').resolve()   # before the chdir below
    if not pt.exists():
        raise SystemExit(f'[hailo] no checkpoint {pt}')
    out = pathlib.Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    # The DFC writes its logs (acceleras.log, allocator.log, hailo_sdk.*.log)
    # into the WORKING directory. Run from the output directory so they land
    # beside the artifact they describe -- they were committed to the repo by
    # accident once -- and so the downgrade scan below reads THIS run's log.
    os.chdir(out)

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

    # The threshold this whole tool is about. It is compiled in here and
    # cannot be changed afterwards by any runtime setting.
    script = (
        'normalization1 = normalization([0.0, 0.0, 0.0], '
        '[255.0, 255.0, 255.0])\n'
        f'nms_postprocess(meta_arch=yolov8, engine=cpu, '
        f'classes={len(names)}, '
        f'nms_scores_th={a.nms_score_th}, nms_iou_th={a.nms_iou_th})\n')
    for layer in [s.strip() for s in a.a16_layers.split(',') if s.strip()]:
        # The layer name the DFC reports is `<model>/<layer>`; the script
        # wants it bare, so accept either spelling rather than making the
        # caller reformat an error message.
        bare = layer.split('/')[-1]
        script += (f'quantization_param([{bare}], '
                   f'precision_mode=a16_w16)\n')
        print(f'[hailo] WARNING: {bare} kept at 16-bit: it would not quantise to '
              f'INT8. The rest of the network is unaffected.')
    if a.finetune == 'off':
        script += 'post_quantization_optimization(finetune, policy=disabled)\n'
    runner.load_model_script(script)
    print(f'[hailo] model script:\n{script}')

    # The cliff, refused before the hour is spent. Issue #47 asked for a
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
        print('[hailo] WARNING: REDUCED OPTIMIZATION requested. This HEF will have '
              'corrupted scores. Do not deploy it.')

    frames = calib_frames(a.calib_frames, a.imgsz)
    gpu = _gpu_visible()
    before = downgrade_reasons(frames=len(frames), gpu=gpu)
    if before and not a.allow_reduced_optimization:
        raise SystemExit('[hailo] REFUSING before the optimise: '
                         + '; '.join(before) + '. --allow-reduced-optimization '
                         'for a throwaway build that must not be deployed.')

    import logging

    class _Keep(logging.Handler):
        lines: list = []

        def emit(self, record):
            _Keep.lines.append(record.getMessage())

    keep = _Keep(level=logging.WARNING)
    logging.getLogger().addHandler(keep)
    # The DFC also writes its own log file in the working directory, through
    # a logger that may not propagate; read what this optimise appended.
    dfc_log = pathlib.Path.cwd() / 'hailo_sdk.client.log'
    start = dfc_log.stat().st_size if dfc_log.exists() else 0
    try:
        runner.optimize(frames)
    finally:
        logging.getLogger().removeHandler(keep)
    if dfc_log.exists():
        with dfc_log.open('r', errors='replace') as fh:
            fh.seek(start)
            _Keep.lines.extend(fh.read().splitlines())
    after = downgrade_reasons(frames=len(frames), gpu=gpu,
                              log_lines=_Keep.lines)
    if after and not a.allow_reduced_optimization:
        raise SystemExit('[hailo] REFUSING after the optimise -- nothing was '
                         'compiled: ' + '; '.join(after))
    opt = out / f'{a.model}_optimized.har'
    runner.save_har(str(opt))
    print(f'[hailo] optimised -> {opt.name}')

    # The optimised model against its own float self, on frames it was not
    # calibrated on -- the second, independent observation.
    import numpy as np
    from hailo_int8_shift import frames as clip_frames, int8_vs_fp
    held = (np.concatenate([clip_frames(c, 100, a.imgsz) for c in a.verify_clip])
            if a.verify_clip else
            calib_frames(256, a.imgsz, seed=1).astype('float32'))
    shift = int8_vs_fp(runner, held)
    print(f'[hailo] INT8 vs float on {len(held)} held-out frames: n={shift["n"]} '
          f'median {shift["median"]:+.3f} p90 {shift["p90"]:+.3f}')
    bad = int8_shift_reasons(shift)
    if bad and not a.allow_int8_shift:
        raise SystemExit('[hailo] REFUSING after the optimise -- nothing was '
                         'compiled: ' + '; '.join(bad) + '. Try --finetune off, '
                         'or --allow-int8-shift for a build that must not be '
                         'deployed.')
    # Provenance beside the artifact, so a runtime or a reviewer can see what
    # this HEF was built with rather than infer it.
    (out / f'{a.model}.hef.provenance.txt').write_text(
        f'calibration_frames: {len(frames)}\n'
        f'gpu_visible: {gpu}\n'
        f'reduced_optimization: {bool(after)}\n'
        f'reasons: {after}\n'
        f'finetune: {a.finetune}\n'
        f'int8_shift_median: {shift["median"]:.4f} (n={shift["n"]}, '
        f'p90 {shift["p90"]:.4f})\n'
        f'int8_shift_reasons: {bad}\n'
        f'nms_score_th: {a.nms_score_th}\n')

    hef = out / f'{a.model}.hef'
    hef.write_bytes(runner.compile())
    print(f'[hailo] wrote {hef} ({hef.stat().st_size / 1e6:.1f} MB)')
    # The checksum ties this record to ONE artifact: the vehicle's copy is
    # checked against it (`md5sum ~/hailo_models/<model>.hef`), because a HEF
    # rebuilt with the same name is otherwise indistinguishable from the old.
    import hashlib
    with (out / f'{a.model}.hef.provenance.txt').open('a') as fh:
        fh.write(f'hef_md5: {hashlib.md5(hef.read_bytes()).hexdigest()}\n')

    # A HEF that compiles is not a result. The threshold is the reason this
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

---
name: train-model
description: Fine-tune a YOLO detection model for a competition target and put it on the vehicle (train -> sidecar -> Hailo .hef -> launch/mission). Use when building or swapping a detection model.
---

# Train a model and put it on the vehicle

The vehicle runs a compiled Hailo `.hef`, not the `.pt`. A model is three
files with the same **stem**, side by side in `src/mongla_vision/models/`
(and `~/models` on the vehicle, which `./build_mongla.sh` mirrors in):

| file | what it is | without it |
|---|---|---|
| `<stem>.pt` | the trained weights (dev box, CUDA) | nothing to compile |
| `<stem>.hef` | the compiled model the Hailo-8 runs | the Pi falls back to CPU, ~1 Hz |
| `<stem>.yaml` | **the sidecar**: class names, `recommended_conf` | ⛔ empty allowlist, a silent `[]` every frame |

The models in the repo are **development harnesses**. Real ones are trained
and swapped on water days, so never hard-code a class list in more than one
place (CLAUDE.md §9).

Check the current Ultralytics CLI against docs.ultralytics.com before running;
its argument surface drifts between releases.

## 1. Dataset

```
datasets/<task>/
├── images/{train,val}/*.jpg
├── labels/{train,val}/*.txt      # YOLO format: cls cx cy w h, normalised
└── <task>.yaml                   # paths + names
```

Split by **session, not by frame** — neighbouring frames of one clip in both
train and val make validation recall meaningless. Keep datasets out of
`/tmp` (a RAM disk here; training is OOM-killed). ⛔ Do not add image
"enhancement" (CLAHE, dehaze) to the pipeline: measured in 17 configurations,
never positive.

## 2. Train

```bash
yolo train data=datasets/<task>/<task>.yaml model=yolo11n.pt epochs=100 imgsz=640
cp runs/detect/train/weights/best.pt src/mongla_vision/models/<stem>.pt
```

## 3. Write the sidecar

`src/mongla_vision/models/<stem>.yaml` — copy `gate_sharks.yaml` and edit:
`names` (index -> class, exactly as trained) and `recommended_conf` (the bar
you **measured** on held-out footage, with the false-positive rate beside
it). Record the measurement in `.claude/context/measured-bars.md`.

## 4. Compile for the Hailo-8 (dev box)

```bash
./tools/hailo_compile.sh --check              # toolchain only
./tools/hailo_compile.sh <stem> --nms-score-th 0.05
```

The compile now measures the INT8 model against its own float on held-out
frames and **refuses** a build that loses more than 0.10 confidence
(`MAX_INT8_SHIFT`). If it refuses, try `--finetune off` before anything else
(B-82: a diverged fine-tune cost `gate_sharks` 0.34). The NMS score threshold
is baked into the `.hef`: runtime `conf` can only tighten it.

## 5. Use it

Nothing to edit in a config file — the model is chosen by **stem**:

```bash
./build_mongla.sh
ros2 launch mongla_manager bringup.launch.py vision:=true fwd_model:=<stem>
```

or put `fwd_model: <stem>` in `~/.mongla/bringup.yaml`. A mission picks it
with `mongla.set_model('<stem>')` and `mongla.set_classes('<class>')`, and
`test_missions_only_name_classes_a_model_emits.py` fails if a mission names
a class the model does not emit.

## 6. Verify before water

```bash
ros2 run mongla_vision vision_check --camera forward --require-class <class>
python3 tools/composed_lock_harness.py --help      # detection rung on archive footage
```

Never claim it works without seeing detections on real footage at the
`recommended_conf` you wrote down.

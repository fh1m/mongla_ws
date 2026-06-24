# duburi_vision / models

Drop YOLO26 weight files here. **No YAML file is required.** The detector
reads class names directly from the model's embedded names table (trained
with Ultralytics, so `model.names` is always populated). Class names log
at startup regardless.

Optionally place a `<stem>.yaml` sidecar to override the embedded names.
This is only needed if the embedded table uses integer IDs you want to
remap to human names (e.g., a COCO model retrained with different labels).

## Naming convention

Use descriptive stems. YAML sidecar is optional (only needed to override embedded names):

| Weight file                      | Notes                                      |
| -------------------------------- | ------------------------------------------ |
| `gate_nano_100ep.pt`             | Gate-only, nano, 100 epochs                |
| `gate_medium_100ep.pt`           | Gate-only, medium, 100 epochs              |
| `gate_medium_200ep.pt`           | Gate-only, medium, 200 epochs              |
| `flare_medium_100ep.pt`          | Flare-only, medium, 100 epochs             |
| `gate_flare_medium_100ep.pt`     | Gate + flare combined (prequal default)    |
| `yolo26_nano_pretrained.pt`      | COCO 80-class pretrained (auto-download)   |

## Class index YAML format

Mirrors Ultralytics `data.yaml`:

```yaml
names:
  0: gate
  1: flare
  2: buoy
```

The detector resolves names from the YAML first, then falls back to the
model's embedded names table (if any). This lets custom models override
their internal class list without retraining.

## Selecting a model and class filter

Pass just the **stem name** (no path, no `.pt`) at launch:

```bash
# Gate-only model
ros2 launch duburi_vision vision.launch.py camera:=forward model:=gate_medium_100ep classes:=gate

# Flare-only model
ros2 launch duburi_vision vision.launch.py camera:=forward model:=flare_medium_100ep classes:=flare

# Combined model — start with gate class, switch to flare during mission
ros2 launch duburi_vision vision.launch.py camera:=forward model:=gate_flare_medium_100ep classes:=gate
```

Switch the class filter **live** without restarting the detector (node =
`/duburi_detector_<camera>`):

```bash
ros2 param set /duburi_detector_forward classes gate
ros2 param set /duburi_detector_forward classes "gate,flare"
```

Or from within a mission DSL script:

```python
duburi.set_classes('gate')       # gate approach phase
duburi.set_classes('flare')      # flare search phase
duburi.set_classes('')           # all classes (debug)
```

The resolver looks up `models/<stem>.pt` in the package share directory
(installed by `setup.py`). If not found locally it falls back to
`<stem>.pt` and Ultralytics will attempt an auto-download.

## Offline testing with a video file

Run the pipeline on a `.mp4` / `.avi` before pool day:

```bash
ros2 launch duburi_vision vision.launch.py camera:=forward \
    video_file:=/tmp/pool_run.mp4 model:=gate_nano_100ep classes:=gate
```

All downstream nodes (`detector_node`, `tracker_node`, vision verbs) are
identical — `video_file` is just another camera source.

## Pretrained YOLO11 — ROBOSUB tested ★ recommended

COCO 80-class pretrained. Best for sim, webcam, and bench testing.
Use `person` class. Auto-downloads on first use.

| Alias       | Ultralytics stem | Size    | Best for                        |
| ----------- | ---------------- | ------- | ------------------------------- |
| `yolov11n`  | `yolo11n`        | ~5 MB   | ★ Default. CPU + Jetson Orin.  |
| `yolov11s`  | `yolo11s`        | ~19 MB  | A notch more accurate.          |
| `yolov11m`  | `yolo11m`        | ~42 MB  | Decent on Jetson Orin Nano.     |
| `yolov11l`  | `yolo11l`        | ~51 MB  | Desktop / training rig.         |
| `yolov11x`  | `yolo11x`        | ~113 MB | Desktop only.                   |

Quick start:

```bash
ros2 run duburi_vision vision_display --ros-args \
    -p launch_pipeline:=true -p model:=yolov11n -p classes:=person
ros2 run duburi_planner mission move_and_see
```

## Pretrained YOLO26 — previous family

Kept for backwards compatibility.

| Stem                         | Size     | Best for                             |
| ---------------------------- | -------- | ------------------------------------ |
| `yolo26_nano_pretrained`     | ~5 MB    | Jetson Orin + RTX 2060.              |
| `yolo26_small_pretrained`    | ~19 MB   | A notch more accurate.               |
| `yolo26_medium_pretrained`   | ~42 MB   | Decent on Jetson Orin Nano.          |
| `yolo26_large_pretrained`    | ~51 MB   | Desktop / training rig.              |
| `yolo26_xlarge_pretrained`   | ~113 MB  | Desktop only.                        |

## Custom-trained weights

Once we have RoboSub-class data (gate, buoys, dropper marker, torpedo
target, etc.), train with the YOLO26 recipe and drop the resulting
`best.pt` here renamed to a descriptive stem. Add a matching YAML for
the class index.

## Competition models (RoboSub 2026)

| Weight file                      | Camera   | Classes                           | Status    | Chunk                  |
| -------------------------------- | -------- | --------------------------------- | --------- | ---------------------- |
| `gate_rescue_repair.pt`          | forward  | gate(0), rescue(1), repair(2)     | ✅ ready   | task_gate, task_return |
| `slalom_red_pipe.pt`             | forward  | red_pipe(0)                       | ✅ ready   | task_slalom            |
| `bin_fire_blood.pt`              | downward | blood(0), fire(1)                 | ✅ ready   | task_bin               |
| `torpedo_blood_hole.pt`          | forward  | torpedo(0), blood(1), hole(2)     | ✅ ready   | task_torpedo           |

Place `.pt` files in this directory on the Jetson. YAML sidecars are already committed.
Class index order matters — pass exactly the class name string the yaml defines.

### Dual-camera usage (competition launch)

The competition launch (`vision_dual.launch.py`) runs two detectors, named
`/duburi_detector_<camera>`:

```bash
# forward detector — node: /duburi_detector_forward
ros2 param set /duburi_detector_forward active_model gate_rescue_repair
ros2 param set /duburi_detector_forward classes "gate,rescue,repair"

# downward detector — node: /duburi_detector_downward
ros2 param set /duburi_detector_downward active_model bin_fire_blood
ros2 param set /duburi_detector_downward classes "fire,blood"
```

**Pass `camera=`** when calling `set_model()` / `set_classes()` / `use()` from a
dual-cam mission — the helper derives the node as `/duburi_detector_<camera>`:

```python
duburi.set_model('gate_rescue_repair', camera='forward')   # → /duburi_detector_forward
duburi.set_classes('gate,rescue,repair', camera='forward')
duburi.set_model('bin_fire_blood', camera='downward')      # → /duburi_detector_downward
```

### Lazy detection (save GPU)

Both detectors start `paused=True` in the competition launch. Missions
activate inference only for the task that needs it:

```python
duburi.resume_detector('forward')   # → ros2 param set /duburi_detector_forward paused false
# ... task code ...
duburi.pause_detector('forward')    # → ros2 param set /duburi_detector_forward paused true
```

Pausing a detector drops its `_infer_loop` CPU/GPU load to ~0 while keeping
the camera streaming and the tracker running.

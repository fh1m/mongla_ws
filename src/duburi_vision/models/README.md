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
ros2 launch duburi_vision cameras_.launch.py model:=gate_medium_100ep classes:=gate

# Flare-only model
ros2 launch duburi_vision cameras_.launch.py model:=flare_medium_100ep classes:=flare

# Combined model — start with gate class, switch to flare during mission
ros2 launch duburi_vision cameras_.launch.py model:=gate_flare_medium_100ep classes:=gate
```

Switch the class filter **live** without restarting the detector:

```bash
ros2 param set /duburi_detector classes gate
ros2 param set /duburi_detector classes "gate,flare"
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
ros2 launch duburi_vision cameras_.launch.py \
    video_file:=/tmp/pool_run.mp4 model:=gate_nano_100ep classes:=gate
```

All downstream nodes (`detector_node`, `tracker_node`, vision verbs) are
identical — `video_file` is just another camera source.

## Pretrained YOLO26 sizes

| Stem                         | Size     | Best for                             |
| ---------------------------- | -------- | ------------------------------------ |
| `yolo26_nano_pretrained`     | ~5 MB    | Default. Jetson Orin + RTX 2060.     |
| `yolo26_small_pretrained`    | ~19 MB   | A notch more accurate.               |
| `yolo26_medium_pretrained`   | ~42 MB   | Decent on Jetson Orin Nano.          |
| `yolo26_large_pretrained`    | ~51 MB   | Desktop / training rig.              |
| `yolo26_xlarge_pretrained`   | ~113 MB  | Desktop only.                        |

References:
- https://docs.ultralytics.com/models/yolo26
- https://docs.ultralytics.com/guides/yolo26-training-recipe/

## Custom-trained weights

Once we have RoboSub-class data (gate, buoys, dropper marker, torpedo
target, etc.), train with the YOLO26 recipe and drop the resulting
`best.pt` here renamed to a descriptive stem. Add a matching YAML for
the class index.

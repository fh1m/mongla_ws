# duburi_vision / models

Drop YOLO26 weight files here. Alongside each weight file, place a YAML
with the same stem (`<model_name>.yaml`) to supply the class index. The
detector logs all available classes at startup when the YAML is present.

## Naming convention

Use descriptive stems, not ultralytics shorthand:

| Weight file                    | Index YAML                       | Notes                        |
| ------------------------------ | -------------------------------- | ---------------------------- |
| `yolo26_nano_pretrained.pt`    | `yolo26_nano_pretrained.yaml`    | COCO 80-class (default)      |
| `yolo26_small_pretrained.pt`   | `yolo26_small_pretrained.yaml`   | COCO, higher accuracy        |
| `gate_nano_100ep.pt`           | `gate_nano_100ep.yaml`           | Gate-only (single class)     |
| `gate_medium_100ep.pt`         | `gate_medium_100ep.yaml`         | Gate-only, medium accuracy   |
| `gate_flare_v1.pt`             | `gate_flare_v1.yaml`             | Gate + flare (two classes)   |

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

Pass just the **stem name** (no path, no `.pt`) and a `classes` CSV:

```bash
# Gate-only model, gate class
ros2 launch duburi_vision cameras_.launch.py model:=gate_nano_100ep classes:=gate

# Flare model (once trained)
ros2 launch duburi_vision cameras_.launch.py model:=flare_v1 classes:=flare

# Combined model — gate task
ros2 launch duburi_vision cameras_.launch.py model:=gate_flare_v1 classes:=gate

# Combined model — both classes
ros2 launch duburi_vision cameras_.launch.py model:=gate_flare_v1 classes:=gate,flare
```

Switch the class filter live without restarting the detector:

```bash
ros2 param set /duburi_detector classes gate
ros2 param set /duburi_detector classes "gate,flare"
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

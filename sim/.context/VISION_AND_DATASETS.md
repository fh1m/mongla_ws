# Vision, datasets and missions in the sim — every combination

One page for the whole loop: bring the sim up, run the vision pipeline in any
camera/model combination, collect a dataset (Gazebo-labelled or hand-labelled),
train, and run a mission against the result.

Companion pages: [SIM_VISION_TRAINING.md](SIM_VISION_TRAINING.md) is the
narrative walkthrough with the training step; this page is the reference matrix.
[ROBOSUB_AND_ACOUSTICS.md](ROBOSUB_AND_ACOUSTICS.md) covers the props and pools.

---

## 0. Sourcing, every terminal

```bash
source /opt/ros/humble/setup.bash
source ~/Ros_workspaces/duburi_ws/install/setup.bash        # autonomy FIRST
source ~/Ros_workspaces/duburi_ws/sim/install/setup.bash
export GZ_IP=127.0.0.1
export DUBURI_WS=~/Ros_workspaces/duburi_ws
```

Build in the same order, and only after a code change:

```bash
cd ~/Ros_workspaces/duburi_ws && ./build_dubomini.sh
cd sim && ./build_sim.sh
```

---

## 1. Courses and props

13 courses across two competitions. `duburi_sim sim course:=<name>`:

| Course | Competition | What is in it |
|---|---|---|
| `pool_empty` | SAUVC | bare pool — hydrodynamic tuning, step response |
| `sauvc26_qualification` | SAUVC | start zone + qualification gate at ~10 m |
| `sauvc26_final` | SAUVC | **full run** — gate, orange flare, 4 drums + mat, 3 bump flares |
| `task_navigation` | SAUVC | gate drill at 7 m (short on purpose) |
| `task_target_acquisition` | SAUVC | drums + mat only |
| `task_localization` | SAUVC | the three bump flares |
| `robosub26_full` | RoboSub | **full run** — all six tasks, both role variants |
| `robosub26_smoke` | RoboSub | bare 20 × 12 × 2.1 m pool |
| `rs_task_gate` | RoboSub | the pass-through gate + a path marker |
| `rs_task_slalom` | RoboSub | three WHITE/RED/WHITE pipe sets |
| `rs_task_bins` | RoboSub | the PVC pipeline with four crates |
| `rs_task_torpedo` | RoboSub | printed board + pinger |
| `rs_task_octagon` | RoboSub | floating octagon, resupply table, collectibles |

26 spawnable props. List them, and put any one anywhere at runtime:

```bash
ros2 run duburi_sim_scenarios props list
ros2 run duburi_sim_scenarios props add robosub_bins bins_2 4.0 1.5
ros2 run duburi_sim_scenarios props move bins_2 5.0 -1.0
ros2 run duburi_sim_scenarios props remove bins_2
```

### Making a new course

A course is one YAML in `duburi_sim_worlds/courses/`. Nothing else — the
template, physics, lighting, buoyancy whitelist and pool shell are shared.

```yaml
name: my_drill
description: >
  What this drills, and WHY the distances are what they are. Say which numbers
  are rulebook and which are yours -- the next person cannot tell otherwise.

competition: robosub          # or omit for sauvc
scene:
  lighting: competition       # clear | competition | murky
  water_surface: gerstner     # gerstner (default) | plane | none

vehicle:
  model: duburi_heavy
  name: duburi
  pose: [-6.0, 0.0, -0.6]
  yaw: 0.0

props:
  - model: robosub_gate       # any name from `props list`
    name: gate
    xy: [-1.0, 0.0]
    yaw: 0.0                  # optional
    z_offset: 0.0             # optional nudge off the anchor
```

```bash
cd sim/src/duburi_sim_worlds && python3 scripts/gen_world.py courses/my_drill.yaml
cd ../.. && ./build_sim.sh
```

`--all` regenerates textures, prop models and every world together.

> **Never hand-edit `models/*/model.sdf` or `worlds/*.world`.** Both are
> generated. Edit `spec/<competition>.yaml` (dimensions),
> `scripts/prop_library.py` (geometry) or the course YAML, then regenerate.

### Making a new prop

1. Dimensions into `spec/<competition>.yaml`, quoting the rulebook page.
2. A builder in `scripts/prop_library.py` returning `model(name, body)`.
3. One `PROPS` entry: `build`, `anchor` (`ANCHOR_FLOOR`/`ANCHOR_SURFACE`),
   `dynamic`.
4. **Append its name to `DETECTION_CLASSES`** — never insert, never reorder.
   The index is both the YOLO class and the Gazebo semantic label, so moving an
   entry silently relabels every dataset ever recorded. A prop missing from
   that list is invisible to the bounding-box camera, with no warning.
5. `python3 scripts/gen_world.py --all`, rebuild, and **render it** — two
   geometry bugs got through review and were caught only by looking.

---

## 2. Running the vision pipeline — every combination

Two launch files. `vision.launch.py` drives ONE camera, `vision_dual.launch.py`
drives both. The dual file prefixes every per-camera argument `fwd_`/`dwn_`.

> **`ros2 launch` silently ignores an unknown `key:=value`.** `model:=x` on the
> dual launch is accepted, does nothing, and you get plausible detections from
> the DEFAULT weights. Verify with
> `ros2 param get /duburi_detector_forward active_model`.

### Single camera, single model

```bash
# live USB webcam
ros2 launch duburi_vision vision.launch.py \
    camera:=forward device:=4 model:=gate_rescue_repair classes:=gate,rescue \
    paused:=false viewer:=true

# the sim's front camera
ros2 launch duburi_vision vision.launch.py \
    camera:=forward topic:=/duburi/sim/front_camera/image_fx \
    model:=sauvc_sim classes:=final_gate,orange_flare paused:=false

# a recorded video file
ros2 launch duburi_vision vision.launch.py \
    camera:=forward video_file:=~/clips/gate.mp4 loop:=true \
    model:=gate_rescue_repair paused:=false
```

### Single camera, multi-model (switch mid-mission)

```bash
ros2 launch duburi_vision vision.launch.py \
    camera:=forward topic:=/duburi/sim/front_camera/image_fx \
    models:=gate_rescue_repair,bin_fire_blood active_model:=gate_rescue_repair \
    paused:=false
```

Switch from a mission with `duburi.set_model('bin_fire_blood')`, or from a
`ClassRef` (`duburi.models.bin.fire`), which switches model *and* class.

### Dual camera, single model each

```bash
ros2 launch duburi_vision vision_dual.launch.py \
    fwd_topic:=/duburi/sim/front_camera/image_fx \
    dwn_topic:=/duburi/sim/bottom_camera/image_fx \
    fwd_model:=sauvc_sim fwd_classes:=final_gate,orange_flare,starting_zone \
    dwn_model:=sauvc_sim dwn_classes:=drum_red,drum_blue \
    device_cls:=cpu paused:=false viewer:=true
```

### Dual camera, multi-model each

```bash
ros2 launch duburi_vision vision_dual.launch.py \
    fwd_topic:=/duburi/sim/front_camera/image_fx \
    dwn_topic:=/duburi/sim/bottom_camera/image_fx \
    fwd_models:=gate_rescue_repair,torpedo_blood_hole \
    dwn_models:=bin_fire_blood \
    device_cls:=cpu paused:=false
```

### Dual camera on recorded video

```bash
ros2 launch duburi_vision vision_dual.launch.py \
    fwd_video:=~/clips/front.mp4 dwn_video:=~/clips/bottom.mp4 \
    fwd_loop:=true dwn_loop:=true paused:=false
```

### The arguments that fail silently if you skip them

| Argument | Why |
|---|---|
| `paused:=false` | Both launches default to **paused** (missions resume the detector they need). Without it the HUD shows `det=ERR dets=0` and looks broken. Live fix: `ros2 param set /duburi_detector_forward paused false` |
| `fwd_topic:` / `topic:` | No topic source means the node opens a **webcam**, not the sim |
| `fwd_classes:` | The allowlist does **not** follow the model. New weights without it → every detection filtered → a silent `[]` forever |
| `device_cls:=cpu` | On a box with no CUDA the detector node **dies** at the `cuda:0` default |
| `image_fx` not `image_raw` | `image_fx` is the water. `image_raw` is a clean render no pool has |
| `dwn_topic:` on the dual launch | Without it the downward node tries webcam index 4, fails, and dies |

### Live tuning, no restart

```bash
ros2 param set /duburi_detector_forward conf 0.35
ros2 param set /duburi_detector_forward classes "gate,flare"
ros2 param set /duburi_detector_forward max_det 10
ros2 param set /duburi_detector_forward paused true
```

### Watching it

```bash
ros2 run rqt_image_view rqt_image_view /duburi/vision/forward/image_debug
ros2 topic echo /duburi/vision/forward/detections
ros2 run duburi_vision vision_check --camera forward --require-class gate
ros2 launch duburi_vision mission_web.launch.py      # browser console, :8090
```

---

## 3. Dataset collection

### A. Gazebo bounding-box labels (recommended, and free)

Ground truth straight from the renderer: occlusion- and truncation-correct, no
hand-labelling, and **runtime-spawned props are labelled too**.

```bash
ros2 run duburi_sim_bridge record_cameras \
    --duration 60 --frames --labels \
    --lighting murky --course robosub26_full --label transit_murky
```

| Flag | Effect |
|---|---|
| `--frames` | dump PNGs as well as the MP4 |
| `--labels` | YOLO labels from the bounding-box cameras + `classes.txt` |
| `--lighting clear\|competition\|murky` | override the course's water; implies `--fx` |
| `--fx` | apply the course's own turbidity |
| `--cameras front,bottom` | which cameras (default both) |
| `--duration 0` | run until Ctrl-C |

**Fly the vehicle while it records.** A stationary capture is N copies of one
image; a model trained on it learns one viewpoint. From another terminal:

```bash
ros2 run duburi_planner duburi arm
ros2 run duburi_planner duburi set_depth --target -1.0
ros2 run duburi_planner duburi move_forward --duration 16 --gain 45
ros2 run duburi_planner duburi yaw_right   --target 25
ros2 run duburi_planner duburi move_forward --duration 12 --gain 40
ros2 run duburi_planner duburi disarm
```

Verify on the only criterion that matters — **frames == labels == meta.counts**:

```bash
D=$(ls -td sim/datasets/transit_murky_* | head -1)
ls $D/frames/front | wc -l ; ls $D/labels/front | wc -l
find $D/labels/front -size +0 | wc -l          # non-empty
python3 -c "import json;print(json.load(open('$D/meta.json'))['counts'])"
```

### B. Hand labelling

Record frames without labels, then annotate:

```bash
ros2 run duburi_sim_bridge record_cameras --duration 60 --frames \
    --lighting competition --course sauvc26_final --label handlabel
```

Point any annotator at `sim/datasets/<run>/frames/front/`. Write YOLO `.txt`
files into `labels/front/` with matching stems, and a `classes.txt` listing
class names in index order. `dataset_to_yolo` then treats the run identically
to a Gazebo-labelled one.

> Keep an **empty** `.txt` for a frame with nothing in it. Empty means "no
> objects here" and is a usable negative; *missing* means the frame was never
> labelled and is unusable. `dataset_to_yolo` drops the second and keeps the
> first.

### C. Build the YOLO dataset

```bash
ros2 run duburi_sim_bridge dataset_to_yolo \
    --runs 'transit_*' --out ~/sim_yolo --camera front --link
```

It splits **by run** (consecutive frames are near-duplicates; a random split
makes the val score a memorisation score), keeps empty labels as negatives, and
warns about the things that make a good mAP meaningless — zero-instance
classes, and a val run with no vehicle motion.

### D. Train, install, run back

```bash
yolo detect train data=~/sim_yolo/data.yaml model=yolo11n.pt epochs=60 imgsz=640
cp runs/detect/train/weights/best.pt ~/models/my_model.pt
# a .yaml sidecar of class names MUST sit beside it, or the allowlist is empty
# and the detector emits [] every frame
cd ~/Ros_workspaces/duburi_ws && ./build_dubomini.sh
```

Then §2 with `model:=my_model`.

### E. OpenCV, without ROS

Every dataset is plain PNG + txt, so nothing stops you working directly:

```python
import cv2, glob
D = 'sim/datasets/transit_murky_20260829_010203'
names = open(f'{D}/classes.txt').read().split()
for f in sorted(glob.glob(f'{D}/frames/front/*.png'))[:20]:
    img = cv2.imread(f); h, w = img.shape[:2]
    for line in open(f.replace('/frames/', '/labels/').replace('.png', '.txt')):
        if not line.strip():
            continue
        c, xc, yc, bw, bh = line.split()
        xc, yc, bw, bh = (float(v) for v in (xc, yc, bw, bh))
        p0 = (int((xc - bw / 2) * w), int((yc - bh / 2) * h))
        p1 = (int((xc + bw / 2) * w), int((yc + bh / 2) * h))
        cv2.rectangle(img, p0, p1, (0, 255, 0), 2)
        cv2.putText(img, names[int(c)], (p0[0], p0[1] - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.imshow('gt', img); cv2.waitKey(300)
```

A live OpenCV view of any camera or debug topic:

```bash
ros2 run duburi_vision vision_display          # the mission HUD
ros2 run rqt_image_view rqt_image_view /duburi/vision/forward/image_debug
```

### F. From the browser

`duburi_sim lab` → **operate** → the `record` panel: name, per-camera
checkboxes, `fx`, `frames`, `labels`, then `● record` / `■ stop + download`,
which zips the run. The **datasets** page lists every run with its integrity
badge (`frames == labels == meta.counts`) and a zip link.

> The browser path does not yet expose `--lighting` or a non-zero `--duration`;
> those are CLI-only. Noted in the plan, not yet built.

---

## 4. Running a mission

```bash
# T1 world     T2 stack (BEFORE vision -- `stack` kills a running vision launch)
ros2 run duburi_sim_bringup duburi_sim stop
ros2 run duburi_sim_bringup duburi_sim sim course:=sauvc26_final
ros2 run duburi_sim_bringup duburi_sim stack --no-vision

# T3 vision (see §2)      T4 prove the loop, then fly
ros2 run duburi_sim_bridge contract_check
ros2 run duburi_sim_bringup duburi_sim smoke
ros2 run duburi_planner mission --list
ros2 run duburi_planner mission task_gate
```

Single verbs, for tuning:

```bash
ros2 run duburi_planner duburi vision_align --camera forward \
    --target_class final_gate --axes yaw,lat --err_px 40 --gain 30 --duration 25
ros2 run duburi_planner duburi vision_move --camera forward \
    --target_class final_gate --fwd_fill 70 --mode area --gain 35 --duration 30
```

**Order matters: stack BEFORE vision.** `duburi_sim stack` runs a cleanup pass
matching `ros2 launch duburi_vision`, so starting the stack second kills the
vision pipeline with nothing logged, and the verbs then fail `NO_CAMERA`.

---

## 5. What does not transfer to the pool

Detection thresholds and vision gains. Sim imagery is cleaner than pool water
even at `murky`. **Control behaviour and every `/duburi/move` verb do transfer.**

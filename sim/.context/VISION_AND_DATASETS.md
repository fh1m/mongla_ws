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

## The torpedo board's openings are REAL now — and why that mattered most

The collision plate was always genuinely open, tiled into strips around each
hole. The **visual never was**: one solid box with dark disks painted on an RGB
texture with no alpha. So an opening never parallaxed, never showed water or a
prop behind it, and never responded to light or fog. **A detector trained on
that learns a painted bullseye, not a hole** — which is the transfer failure
this simulator exists to catch, and the reason the board read as clip art beside
a photograph.

Resolution was never the bottleneck. The panel was already 512 px across a
0.6 m board — about 853 px/m, thirteen times the pool floor. What it lacked was
a hole, anti-aliasing (every circle was a boolean mask), and any surface
treatment at all: it was the one prop in the library that ignored the `rng` it
accepted while every other prop had weave, ribbing, scuffs or noise.

**A box cannot have a hole in it**, so the visual is now a generated mesh —
`scripts/gen_prop_meshes.py`, cut from `prop_library.torpedo_openings()`, the
same list the collision strips are tiled from. That invariant is load-bearing:
the artwork and the collision drifted apart once before, and a shot lined up on
the printed circle struck solid board with nothing in any log to say why.

Three mesh traps, each of which produced a *silent* wrong render:

- **`<mesh>` must be wrapped in `<geometry>`.** Without it SDF logs "XML
  Element[mesh] … not defined in SDF", copies it through as an unknown child,
  and the renderer fails the visual — the board simply is not drawn, its legs
  floating on their own.
- **The mesh needs vertex normals.** Without `vn` the material never samples its
  albedo map and the board renders as a flat white plate: geometry perfect,
  artwork gone, mesh loading without complaint.
- **Do not put geometry behind an opening.** A thin rim cylinder added for a
  labelling experiment rendered as a solid disc that plugged all four holes —
  four painted dots again, by a different route.

2026 layout, from the TeamTime "Task 4 — Deploy (Torpedoes)" slide: **four
openings** (two large, two small) in a 2×2, with all four emergency images on
both boards and only the image/size pairing distinguishing the two versions.
The cells live in `spec/robosub.yaml`, so a rules update is one edit.

## Per-visual labels do NOT give per-visual boxes — measured, negative

The pool detectors are trained on **sub-features**: `gate_rescue_repair.pt`
classifies `gate`/`rescue`/`repair`, `bin_fire_blood.pt` classifies
`blood`/`fire`. The sim labels **whole models** (`robosub_gate`, `robosub_bins`),
so **a dataset captured in simulation cannot train any model a mission actually
runs.** That is worth stating plainly, because it is the largest remaining
sim-to-real gap in vision and it is not obvious from any log.

`gz-sim-label-system` accepts a `<visual>`-scope label, and it does change which
class a model is annotated as — with the gate's model label suppressed and only
the boards labelled, `repair` appeared in **268 of 268** bounding-box frames.
But it does **not** produce a box per visual. With the frame visuals labelled
`robosub_gate` and the two boards labelled `repair`/`rescue`, **267 of 267**
frames carried only `robosub_gate`. The sensor emits one box per model; a visual
label merely competes to name it.

So this path is closed. Getting sub-feature classes needs each sub-feature to be
its **own model**, which costs the joints that make the boards swing — a real
trade, not a small edit. The `label=` argument on `visual()` is kept, unused,
with this finding in its docstring, because the next person to try will reach
for exactly that argument.

> Unexplained and pre-existing: every bounding-box frame also carries a
> **class 242**, which is outside `DETECTION_CLASSES`. It predates this work and
> appears on every course. Worth chasing before trusting a raw box count.

## The task images are vendored artwork now, not a font (2026-08-31)

RoboNation prints **Microsoft Fluent 3D** emoji — the gradient flame and the
smooth magenta teardrop on the Task 3 and Task 4 slides are that set. They are
MIT licensed, 256×256 RGBA with real shading, and `scripts/fetch_emoji.py`
vendors the eight we need into `models/robosub_textures/emoji/` so a checkout
builds the same props with no network.

The font path is a **fallback**, and it is a poor one for two measured reasons.
`NotoColorEmoji` is a CBDT bitmap face with a **single 109 px strike**, so every
glyph was drawn at 109 px and resampled up to fill a 256 px placard or a 1024 px
board — soft by construction, and most of why the props read as cartoonish next
to a photograph. And Noto's droplet is **blue** where RoboNation's is magenta,
so the old code collapsed the glyph to luminance and multiplied by a tint,
throwing away the shading that makes it legible and leaving a flat pink blob.

> **`droplet` is the DROP OF BLOOD (🩸), not the water droplet (💧).** Fluent has
> both, and the first fetch took the wrong one — obvious the moment it rendered.
> It matters beyond looks: a *blue* drop would make the sim task **easier** than
> the pool task, because blue-vs-orange separates far more cleanly than
> magenta-vs-orange. A test checks the artwork's own colour, not its filename.

Role placards went 256 → 512 px, since the source is now a 256 px render rather
than a 109 px strike and there is real detail to keep.

## Normal maps — the map nothing here had

Until now **no material in this tree used a normal map**, and SDF 1.9's
`<pbr><metal>` has accepted one all along. A perfectly smooth surface with its
detail painted into the albedo is the clearest single tell of a CG render, and
the worst case for feature matching: descriptors key on local gradients, and a
painted gradient does not move when the light does.

`make_normal_map()` derives a tangent-space map from a height field (PVC
extrusion seams, moulded-plastic pebbling, flare weave). `textured_material()`
gained `normal_map=` and `double_sided=`.

**Measured, by stripping every `<normal_map>` from the generated models and
re-rendering the same frame** — only the map changes:

| height fields | pixels differing >2 levels | mean abs diff |
|---|---|---|
| first attempt | 0.39 % | 0.098 |
| **after raising amplitude and encode strength** | **4.85 %** | **0.473** |

The first pass was applied but nearly a no-op — a feature that costs a texture
fetch and earns nothing. Worth stating, because "it's in the SDF" is not
evidence that a map is doing anything.

## `box_type` was invalid, so no dataset had occlusion correctness

The bounding-box sensors were declared `<box_type>2d</box_type>`. **`2d` is not a
valid token** — the set is `full_2d`, `full_box_2d`, `visible_2d`,
`visible_box_2d`, `3d` — so gz fell back to its default `BBT_FULLBOX2D`,
documented as *"the full box of occluded objects"*.

Every dataset captured here before 2026-08-31 therefore has **no occlusion
correctness**: a crate hidden behind another crate got a full box, which is the
exact defect the geometric AABB projector was replaced to fix. Two comments in
the tree claimed the opposite, and both are corrected.

An invalid token costs nothing visible. It does **not** print "Unknown bounding
box type" (checked before and after, both zero), so nothing in a log ever said
so. Now `visible_2d`.

## One box per TOP-LEVEL MODEL — nested models do not help either

The pool detectors are trained on **sub-features**: `gate_rescue_repair.pt`
classifies `gate`/`rescue`/`repair`, `bin_fire_blood.pt` classifies
`blood`/`fire`. The sim can only annotate **whole props**, so a dataset captured
here still cannot train a model any mission runs. That is the largest remaining
sim-to-real gap in vision, and it is now measured rather than assumed.

Round 8 established that a `<visual>`-scope label does not create its own box.
Round 9 tested the fallback the plan named — **nested `<model>`s**, since
`Ogre2BoundingBoxCamera::MergeMultiLinksModels2D()` merges per-*link* boxes up
to the model and a nested model looked like it should be its own merge group.

Four runs on `rs_task_gate`, ~270 bbox frames each:

| gate model label | signs | frame | labels emitted |
|---|---|---|---|
| `robosub_gate` | nested, 27/28 | plain links | **12 only** |
| suppressed | nested, 27/28 | plain links | **27 only**, own box 195×36 |
| suppressed | nested, 27/28 | nested, 12 | **12 only** |

The middle row looks like success and is not. `repair` appeared only because
nothing else under the gate carried a label — with the frame nested and
labelled, 12 wins again. **gz collapses everything beneath a top-level model
instance into one box and picks one label for it.** Nesting changes nothing.

So the experiment was reverted whole: the gate is a flat model with a
model-scope label again, and `repair`/`rescue` are **not** in
`DETECTION_CLASSES`, because nothing emits them. The only path left is making
each sub-feature its own **top-level model placed by the course**, which costs
the joints that make the boards swing — a real trade, not a small edit.

> **Class 242 identified.** Its box is 638×476 on a 640×480 image — the whole
> frame. It is the pool shell or the Fuel `waves` surface, not a prop.
> `box_labels.py:77` discards it (out of range), so no dataset is affected, but
> any raw box count is off by one per frame.

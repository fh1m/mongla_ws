# Sim → vision → dataset → model → mission, terminal by terminal

Everything here was executed on 2026-08-28 and the numbers are measured, not
estimated. It takes you from a cold shell to a **sim-native YOLO model** driving
`vision_align` / `vision_move` inside Gazebo.

> **Why a sim-native model at all.** The competition weights
> (`gate_rescue_repair`, `bin_fire_blood`) are trained on *real* RoboSub props.
> Pointed at the SAUVC sim they mis-fire — a pool floor edge scored `gate 46 %`
> in testing. That is not a pipeline fault, it is a domain gap, and the fix is a
> model trained on sim imagery. Before this document there was no such model and
> no path to one; the missing link was `dataset_to_yolo`.

---

## 0. Sourcing — once per terminal

Autonomy first, then sim. `stack.launch.py` includes autonomy launch files by
share directory, so the order matters.

```bash
source /opt/ros/humble/setup.bash
source ~/Ros_workspaces/duburi_ws/install/setup.bash
source ~/Ros_workspaces/duburi_ws/sim/install/setup.bash
export GZ_IP=127.0.0.1
export DUBURI_WS=~/Ros_workspaces/duburi_ws
```

Build order is the same, and only needed after a code change:

```bash
cd ~/Ros_workspaces/duburi_ws && ./build_dubomini.sh
cd sim && ./build_sim.sh
```

---

## 1. T1 — the world

`stop` FIRST, always. Two simulators on one machine fight over UDP 14550 and
gz-transport, and the failure looks like flaky physics rather than a duplicate.

```bash
ros2 run duburi_sim_bringup duburi_sim stop
ros2 run duburi_sim_bringup duburi_sim sim course:=sauvc26_final
#   --headless                for dataset capture / CI (no GUI)
#   course:=task_navigation   7 m gate drill
#   course:=task_target_acquisition   drums + mat only
```

Confirm the cameras are actually running before anything else. **`ros2 topic hz`
is not a good instrument here** — it deserialises every ~1 MB image in Python
and its own cost shows up as the publisher being slow. Count arrivals instead:

```bash
ros2 topic hz /duburi/sim/front_camera/image_raw   # expect ~12 Hz, jitter ~10 ms
```

Healthy is **~12 Hz with single-digit-millisecond jitter**. If you see ~3 Hz with
several hundred ms of jitter, something re-enabled the DVL beam visuals — see
[TROUBLESHOOTING.md](TROUBLESHOOTING.md).

---

## 2. T2 — the control stack

```bash
ros2 run duburi_sim_bringup duburi_sim stack --no-vision
```

`--no-vision` because §3 launches vision separately, which is what you want
while iterating on models: restart the detector without restarting the vehicle.

Prove the loop before trusting a mission:

```bash
ros2 run duburi_sim_bridge contract_check
ros2 run duburi_sim_bringup duburi_sim smoke
```

---

## 3. T3 — the vision pipeline on Gazebo's cameras

Both sim cameras, both detectors, boxes on `image_debug`:

```bash
ros2 launch duburi_vision vision_dual.launch.py \
    fwd_topic:=/duburi/sim/front_camera/image_fx \
    dwn_topic:=/duburi/sim/bottom_camera/image_fx \
    model:=gate_rescue_repair dwn_model:=bin_fire_blood \
    device_cls:=cpu paused:=false viewer:=true
```

Four things here are not optional and each one fails silently if you skip it:

| Argument | Why |
|---|---|
| `fwd_topic:` / `dwn_topic:` | Without a topic source the camera node opens a **webcam**, not the sim. |
| `paused:=false` | The launch defaults to **paused** — missions resume the detector they need. Without it the HUD reads `det=ERR dets=0` and looks broken. Resume live with `ros2 param set /duburi_detector_forward paused false`. |
| `device_cls:=cpu` | On a box with no CUDA the detector node **dies** at the `cuda:0` default. |
| `image_fx` not `image_raw` | `image_fx` is the water. `image_raw` is a clean render no pool has. |

Healthy looks like this on the `duburi_display` line:

```
[VIS] fps=2.3 dets=1 | gate(50%) ex=-0.00 ey=+0.72 cam=OK det=OK trk=OK
```

`cam=OK det=OK trk=OK` is the thing to read. Watch the boxes with
`ros2 run rqt_image_view rqt_image_view /duburi/vision/forward/image_debug`.

---

## 4. T4 — capture a dataset (auto-labelled)

The recorder writes frames, an MP4, and **YOLO labels projected from Gazebo
ground truth** — so there is no hand-labelling step at all.

```bash
ros2 run duburi_sim_bridge record_cameras \
    --duration 60 --frames --labels \
    --lighting murky --course sauvc26_final --label transit_murky
```

**Drive the vehicle while it records.** A recording of a stationary vehicle is
600 copies of one image; a model trained on it learns one viewpoint. From a
fifth terminal, during the capture:

```bash
ros2 run duburi_planner duburi arm
ros2 run duburi_planner duburi set_depth --target -0.6
ros2 run duburi_planner duburi move_forward --duration 14 --gain 45
ros2 run duburi_planner duburi yaw_right   --target 25
ros2 run duburi_planner duburi move_forward --duration 10 --gain 40
ros2 run duburi_planner duburi disarm
```

`--lighting` overrides the course's water, so a **murky/clear pair of the same
scene** comes out of one sim launch — two courses would also differ in prop
placement, which is not the variable you want to isolate:

```bash
ros2 run duburi_sim_bridge record_cameras --duration 60 --frames --labels \
    --lighting clear --course sauvc26_final --label transit_clear
```

`--lighting` implies `--fx`. Both apply the filter **in-process** from
`image_raw`; they do not subscribe to `image_fx`, which is why capture runs at
~10 fps instead of the 2.4 fps it used to.

Runs land in `sim/datasets/<label>_<UTC stamp>/`. Verify on the only criterion
that matters — **frames == labels == meta.counts**:

```bash
D=$(ls -td sim/datasets/transit_murky_* | head -1)
ls $D/frames/front | wc -l ; ls $D/labels/front | wc -l
python3 -c "import json;print(json.load(open('$D/meta.json'))['counts'])"
```

---

## 5. T5 — build the YOLO dataset

```bash
ros2 run duburi_sim_bridge dataset_to_yolo \
    --runs 'transit_*' --out ~/sim_yolo_gate --camera front --link
```

```
split     : by run (3 train / 1 val)
train/val : 1315 / 154 images   (216 train negatives)
classes   : 11 -> qual_gate, final_gate, orange_flare, ... target_mat
```

Two choices in there worth knowing, because they change what your val number
means:

* **The split is by RUN, not by frame.** Consecutive frames of a recording are
  near-duplicates. A random frame split puts near-copies in both halves and the
  validation score becomes a memorisation score. With a single run it falls back
  to a contiguous tail.
* **Empty label files are kept.** A frame with nothing visible is a true
  negative, and without them the model learns that every image contains a prop.

---

## 6. T6 — train

```bash
yolo detect train data=~/sim_yolo_gate/data.yaml model=yolo11n.pt \
     epochs=40 imgsz=640 batch=16 device=0
```

Weights land in `runs/detect/train/weights/best.pt`. Install them where the
detector looks, **with a class sidecar** — a missing `.yaml` means an empty
allowlist and a silent `[]` every frame:

```bash
cp runs/detect/train/weights/best.pt ~/models/sauvc_sim.pt
python3 - <<'PY'
import yaml, pathlib
names = (pathlib.Path.home()/'sim_yolo_gate'/'data.yaml').read_text()
names = yaml.safe_load(names)['names']
(pathlib.Path.home()/'models'/'sauvc_sim.yaml').write_text(
    yaml.safe_dump({'names': names}, sort_keys=False))
PY
cd ~/Ros_workspaces/duburi_ws && ./build_dubomini.sh   # mirrors ~/models into the tree
```

> Weights are gitignored by extension and live in `~/models`;
> `build_dubomini.sh` mirrors them into `src/duburi_vision/models/`. Model
> identity is the **stem**, so `sauvc_sim.pt` + `sauvc_sim.yaml` is referred to
> everywhere as `sauvc_sim`.

---

## 7. T7 — run the model back in the sim

```bash
ros2 launch duburi_vision vision_dual.launch.py \
    fwd_topic:=/duburi/sim/front_camera/image_fx \
    model:=sauvc_sim device_cls:=cpu paused:=false viewer:=true
```

Then drive it with the two vision verbs:

```bash
ros2 run duburi_planner duburi vision_align --camera forward \
    --target_class final_gate --axes yaw,lat --err_px 40 --gain 30 --duration 25
ros2 run duburi_planner duburi vision_move  --camera forward \
    --target_class final_gate --fwd_fill 70 --mode area --gain 35 --duration 30
```

The detector's always-on line tells you the target is live before you commit a
verb to it:

```
[ align lat=-12 depth=+40px ] (308,280) align ['final_gate'] center -> (0,0)
```

---

## The loop, once you have all this

Capture → convert → train → run back in sim → **find where it fails** → capture
that case → retrain. The whole point of the sim is that the failing case is
reproducible and the labels are free. Vary `--lighting`, vary the course, and
vary the trajectory; do not vary all three in one run or you cannot tell which
one moved the number.

**What does NOT transfer to the pool:** detection thresholds and vision gains.
Sim imagery is cleaner than pool water even at `murky`. Control behaviour and
every `/duburi/move` verb do transfer.

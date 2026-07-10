# Operator tooling — Foxglove, rosbag record/replay, scorecards

> **What this is:** pool-day debugging + practice quality-of-life. **Everything here is
> off the live mission path** — no node in `auv_manager_node` / `motion_vision.py` /
> detector / camera / tracker / missions is touched. Foxglove and rosbag are opt-in;
> the scorecard already ran (this just gives it a home). Nothing here can slow or
> perturb a mission unless you explicitly turn it on.

One idea unifies all three: **one folder per pool session holds everything for a run.**
`DUBURI_RUN_DIR` (default `~/duburi_runs`) is where scorecards land and where
`pool_record.sh` writes bags — so after a session you grab one directory and have the
telemetry, the bag, and the per-verb scorecard together.

---

## 1. Foxglove — live telemetry (`foxglove:=true`)

BumblebeeAS's `controlkitv3` is Foxglove-based; the borrowable idea is **Foxglove
itself**, not their code. `foxglove_bridge` is a stock ROS 2 Humble package — a C++
WebSocket server that auto-exposes every topic. Our vision topics are standard
`vision_msgs/Detection2DArray` + `sensor_msgs/Image`, which Foxglove renders natively
(2D boxes over the image); `/duburi/state` (custom `DuburiState`) shows in the
Raw-Messages panel; `/duburi/move` action feedback (`err_x_px`/`err_y_px`) plots live.

**Install (once, on the Jetson):**
```bash
sudo apt install ros-humble-foxglove-bridge
```

**Run (opt-in launch arg, off by default):**
```bash
ros2 launch duburi_manager bringup.launch.py vision:=true foxglove:=true
# custom port: foxglove:=true foxglove_port:=8766
```
Then in the **Foxglove desktop app**: *Open connection → Foxglove WebSocket →*
`ws://192.168.2.69:8765` (the Jetson IP). Load the shared layout at
`src/duburi_vision/foxglove/duburi_layout.json` so the whole team sees the same view.
The bridge is launched with `include_hidden:=true` so the `/duburi/move/_action/feedback`
plot (err_x_px/err_y_px) isn't silently empty. **The layout is a starting point** — its
panel-config schema is Foxglove-version-sensitive; if a panel loads blank, add it manually
(Image → `/duburi/vision/forward/image_debug`, Plot → the `_action/feedback` err fields)
and re-save the JSON.

### ⚠ Gate A — prove FPS is unperturbed (mandatory before trusting it in-water)

Mongla is **FPS-coupled** (the vision loop freshness-decays the translational command by
frame age). `foxglove_bridge` viewing raw images from two cameras over FathomX is real
tether bandwidth + serialization on the Orin Nano — the classic "new bottleneck." So:

```bash
# baseline (bridge OFF):
ros2 topic hz /duburi/vision/forward/detections
# repeat with foxglove:=true AND an operator viewing images in Foxglove.
# Detection Hz must be essentially unchanged.
```
Mitigations, in order: the bridge already sets `use_compression:=true`; **view one
camera at a time**; prefer the `image_debug` overlay over `image_raw`; if Hz still drops,
un-check the image panels and keep only `detections` / `state` / `move` feedback — still
very useful, near-zero cost.

### ⚠ Gate B — prove the OFFLINE path this week, not on run day

A team abroad on venue Wi-Fi cannot depend on Foxglove cloud login. **Before run day**,
on the actual competition laptop: download the Foxglove **desktop** app, then connect to
`ws://<jetson-ip>:8765` with **internet OFF**. Confirm it connects and the layout loads
with no account. If it demands a login, you found out in the hotel, not at the pool.

---

## 2. rosbag record / replay — `scripts/pool_record.sh`

`ros2 bag record` is built into Humble (zero code). The helper wraps it with a **topic
allowlist** (keeps bags small — off the raw-image firehose) and the shared run tree.
**Payoff for scarce pool time:** record every run, then replay it **offline, dry** to
tune detection conf / gains / mission timings without being in the water. Foxglove plays
bags natively, so a recorded run reviews exactly like a live one.

```bash
scripts/pool_record.sh record gate_run        # debug allowlist → ~/duburi_runs/bag_gate_run_<ts>
scripts/pool_record.sh record --full gate_run  # + raw image_raw/camera_info (BIG; only if needed)
scripts/pool_record.sh replay ~/duburi_runs/bag_gate_run_20260710_141530
scripts/pool_record.sh list                    # runs + sizes + recent scorecards
```
Default allowlist: `state`, `imu_rates`, `move` action feedback/status, and per-camera
`detections` / `tracks` / `image_debug` / `vis_range` / `distance`. Ctrl-C stops and
finalizes the bag. Override the parent dir with `DUBURI_RUN_DIR`.

**Replay-to-tune loop:** `replay` a bag → run `vision_display` or Foxglove against it →
`ros2 param set /duburi_detector_forward conf 0.45` and watch the effect — all on the
bench, no pool. (The control loop needs the manager+MAVLink; **detection/vision tuning**
is what replays cleanly.)

---

## 3. Mission scorecards — dedicated folder + traceability

Already emitted by `mission.py` on every exit (success, exception, or Ctrl-C) via
`DuburiMission.log_scoreboard`. What changed: it now writes into **`DUBURI_RUN_DIR`**
(default `~/duburi_runs`) as `<mission>_<ts>.json` instead of littering the CWD, and the
JSON carries `mission`, ISO `timestamp`, and best-effort `git_sha` (which code ran this
run) alongside `total_s` / `success_count` / `fail_count` / per-verb `phases`. No mission
code changes — the runner passes the mission name it already knows.

```bash
ros2 run duburi_planner mission fsm_full_2026   # → ~/duburi_runs/fsm_full_2026_<ts>.json
# put a whole session in one place:
DUBURI_RUN_DIR=~/duburi_runs/2026-champs ros2 run duburi_planner mission fsm_full_2026
```
`git_sha` is best-effort (2 s timeout, returns `''` if git/the repo is unavailable) — it
never raises on pool day.

### Unified per-run logs (both terminals, zero code)

rclpy/rcl already writes a per-process log tree; point it at the run folder so the
**manager** terminal (`[STATE]`/`[ARDUB]`/`[RC ]`/`[ACT]` telemetry) **and** the mission
terminal both land in one place — more complete than an in-process tee (which would only
see the mission terminal). Export the same `ROS_LOG_DIR` in each terminal of a session:

```bash
export DUBURI_RUN_DIR=~/duburi_runs/2026-champs
export ROS_LOG_DIR=$DUBURI_RUN_DIR/logs        # rcl logs for every node started here
# terminal 1:
ros2 launch duburi_manager bringup.launch.py vision:=true foxglove:=true 2>&1 | tee $DUBURI_RUN_DIR/manager_console.log
# terminal 2 (same two exports):
ros2 run duburi_planner mission fsm_full_2026  2>&1 | tee $DUBURI_RUN_DIR/mission_console.log
```
Now `~/duburi_runs/2026-champs/` holds the bag, the scorecard, the rcl logs, and both
console tees for that session. The `tee` is optional (rcl logs already persist); it just
gives you the exact colored terminal output too.

---

## What we deliberately did NOT add

The rest of BumblebeeAS's robustness is a **metric-world paradigm** (`frames`
compile-time TF types, PnP `pose_estimator`, pose-clustering `filters`, UKF fusing
DVL+IMU+FOG). Mongla is **pixel-native** by design and achieves target stability by other
means we already ship (OC-SORT + Kalman coast, `detected()` hysteresis,
`align_stable_frames`, freshness-decay, the mid-hold station-keep). Porting the metric
stack days before competition is a rewrite, not an add — recorded as a post-competition
R&D note. An inference-time image-enhancement node was also rejected: our YOLO11 models
were trained on raw frames, so a filter creates train/serve skew and *degrades* detection
— the right lever for murky water is conf tuning / a pool-frame fine-tune, not a filter.
Scouting detail lives in the plan; do not revisit these mid-competition.

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

## 0. The per-run debug workflow (start here) — tested on-device 2026-07-10

Every run becomes fully reviewable: **live** in Foxglove while it happens, and **offline**
from one folder afterwards (MCAP bag + scorecard + node logs). One helper —
`scripts/pool_session.sh` — pins that folder so you don't hand-juggle env vars.

**Pin the run folder in EVERY terminal** (same label = same folder):
```bash
cd ~/Ros_workspaces/duburi_ws && source install/setup.bash
source scripts/pool_session.sh gate_am     # exports DUBURI_RUN_DIR + ROS_LOG_DIR
```

**Terminal 1 — preflight + vehicle + telemetry + live viz** (the launch file is the
`ros2 run duburi_manager start` equivalent that *also* wires vision + Foxglove; the same
`-p` params become `arg:=` — `bno085_port`/`payload_port` default to `auto` already):
```bash
ros2 run duburi_manager bringup_check         # 12-section preflight; fix any FAIL first
ros2 launch duburi_manager bringup.launch.py \
     mode:=pool yaw_source:=bno085 vision:=true foxglove:=true
```
> Prefer your existing `ros2 run duburi_manager start …`? It has no Foxglove arg — start
> the bridge yourself in a spare terminal:
> `ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 -p use_compression:=true -p include_hidden:=true`

**Terminal 2 — record the run** (MCAP, into the pinned folder):
```bash
scripts/pool_record.sh record gate_am         # Ctrl-C to stop + finalize the bag
```

**Terminal 3 — drive it** (mission OR CLI — scorecard auto-writes into the folder):
```bash
ros2 run duburi_planner mission fsm_full_2026
#   or hand-fly:  ros2 run duburi_planner duburi arm ; ... ; duburi disarm
```

**Topside laptop — watch live:** Foxglove desktop → *Open connection → Foxglove WebSocket*
→ `ws://192.168.2.69:8765`, load `src/duburi_vision/foxglove/duburi_layout.json`.

**After the run — one folder has everything:**
```bash
scripts/pool_record.sh list                   # bags + recent scorecards
ls ~/duburi_runs/gate_am/                      # bag_gate_am_<ts>/  <mission>_<ts>.json  logs/
```

**Offline replay-to-tune (bench, no pool time):**
```bash
source scripts/pool_session.sh gate_am
scripts/pool_record.sh replay ~/duburi_runs/gate_am/bag_gate_am_<ts>
# other terminal: ros2 run duburi_vision vision_display   (or drag the .mcap into Foxglove)
# then: ros2 param set /duburi_detector_forward conf 0.45  and watch the effect
```

Sections 1–3 below detail each piece.

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
sudo apt install ros-humble-foxglove-bridge ros-humble-rosbag2-storage-mcap
```
Both are declared as `exec_depend` in `duburi_manager/package.xml`, so on a fresh
image `rosdep install --from-paths src` restores them — you only run the apt line
by hand if rosdep isn't set up. (`foxglove_bridge` = live telemetry;
`rosbag2_storage_mcap` = the MCAP bag format `pool_record.sh` writes, §2.)

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

A team abroad on venue Wi-Fi cannot depend on a cloud login. We resolved this by
standardizing on **Lichtblick** (the offline-safe fork — see §4), which has no account
wall at all. **Still prove it before run day**, on the actual competition laptop: connect
to `ws://<jetson-ip>:8765` with **internet OFF** and confirm it connects and the layout
loads. (If you instead use the official Foxglove desktop app, this gate is where its
login prompt would bite you — hence the Lichtblick choice.)

---

## 2. rosbag record / replay — `scripts/pool_record.sh`

`ros2 bag record` is built into Humble (zero code). The helper wraps it with a **topic
allowlist** (keeps bags small — off the raw-image firehose) and the shared run tree.
**Payoff for scarce pool time:** record every run, then replay it **offline, dry** to
tune detection conf / gains / mission timings without being in the water. The helper
records **MCAP** (`-s mcap`), Foxglove's native format — so you can either drag the
`.mcap` straight into the Foxglove desktop app (schemas travel with the bag, so the
custom `DuburiState` renders), or `replay` it into a live `foxglove_bridge`; a recorded
run reviews exactly like a live one.

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
see the mission terminal). `scripts/pool_session.sh` sets both `DUBURI_RUN_DIR` and
`ROS_LOG_DIR` for you; **source it (same label) in every terminal**:

```bash
source scripts/pool_session.sh 2026-champs     # → DUBURI_RUN_DIR=~/duburi_runs/2026-champs, ROS_LOG_DIR=.../logs
# terminal 1:
ros2 launch duburi_manager bringup.launch.py vision:=true foxglove:=true 2>&1 | tee $DUBURI_RUN_DIR/manager_console.log
# terminal 2 (same `source pool_session.sh 2026-champs`):
ros2 run duburi_planner mission fsm_full_2026  2>&1 | tee $DUBURI_RUN_DIR/mission_console.log
```
Now `~/duburi_runs/2026-champs/` holds the bag, the scorecard, the rcl logs, and both
console tees for that session. The `tee` is optional (rcl logs already persist); it just
gives you the exact colored terminal output too. (Manual equivalent, no helper:
`export DUBURI_RUN_DIR=~/duburi_runs/2026-champs; export ROS_LOG_DIR=$DUBURI_RUN_DIR/logs`.)

---

## 4. Ground station (dev box) — the viewer + offline replay

> **How you reach the Jetson at all** (mosh+tmux terminal, NoMachine desktop, killing the
> polkit password popups, drop-proof recovery) is its own guide:
> [`remote-access.md`](remote-access.md). This section is just the **viewer**.

The Jetson runs the **bridge** (server); the dev box / operator laptop runs the **viewer**
(client) and **replays bags**. The viewer we standardized on is **Lichtblick** — Bosch's
MIT open-source fork of Foxglove Studio — chosen over the official Foxglove desktop app
precisely because it has **no account / no login wall** and is guaranteed to work fully
offline at the venue (Gate B). It loads the same layout and speaks the same
Foxglove-WebSocket protocol as the bridge.

**Dev-box install (once)** — all three, verified on the `auv-ros2` distrobox 2026-07-10:
```bash
# viewer (offline-safe): latest .deb from github.com/lichtblick-suite/lichtblick/releases
sudo apt install -y /path/to/lichtblick-<ver>-linux-amd64.deb   # provides `lichtblick`
# bridge (attach the viewer to LOCAL sim for dry practice):
sudo apt install -y ros-humble-foxglove-bridge
# MCAP storage plugin -- REQUIRED for `pool_record.sh replay` / `ros2 bag play` here.
# Stock Humble desktop only has sqlite3, so without this an MCAP bag errors
# "invalid choice: 'mcap'". (The Jetson gets it via package.xml; the dev box needs it too.)
sudo apt install -y ros-humble-rosbag2-storage-mcap
```

**Connect to the live AUV:** launch `lichtblick` → *Open connection → Foxglove WebSocket*
→ `ws://192.168.2.69:8765` (the Jetson). Then *Layouts → Import* and pick
`src/duburi_vision/foxglove/duburi_layout.json`. This is pure WebSocket over the
tether/switch — it needs **no** matching `ROS_DOMAIN_ID` and **no** DDS discovery on the
dev box (that's why it's robust across the network).

> **⚠ Viewer-vs-bridge protocol version (real gotcha).** `foxglove_bridge` 3.4.x is the
> new Foxglove-SDK server and requires the WebSocket subprotocol **`foxglove.sdk.v1`**; the
> *classic* `foxglove.websocket.v1` gets a silent **HTTP 400 "handshake failed"**.
> **Lichtblick 1.27.0 speaks `foxglove.sdk.v1`** (verified — it negotiates and connects), so
> the standardized pairing is fine. But an **old** Foxglove Studio that only knows the
> classic token will fail to connect to this bridge — if a teammate's viewer won't connect,
> update it (or pin the bridge to a 0.7.x/`foxglove.websocket.v1` build). Diagnose from the
> bridge terminal: a good client logs a channel subscription; a version-mismatched one logs
> `Dropping client …: handshake failed`.

**Dry practice with NO AUV (local sim on the dev box):** the bridge runs here too, so you
can rehearse the whole Foxglove workflow against Gazebo SITL before pool day:
```bash
ros2 launch duburi_manager bringup.launch.py mode:=sim yaw_source:=mavlink_ahrs \
    vision:=true foxglove:=true viewer:=false
# then Lichtblick → ws://localhost:8765
```

**Pull a run off the Jetson and replay it offline** (the highest-ROI loop — tune detection
without the pool). Bags live in `~/duburi_runs` on the Jetson; copy the whole run folder
so the bag, scorecard, and logs come together:
```bash
rsync -av jetson@192.168.2.69:~/duburi_runs/  ~/duburi_runs/     # or scp -r
# replay locally (needs the MCAP plugin above + duburi_interfaces built here):
scripts/pool_record.sh replay ~/duburi_runs/bag_gate_run_<ts>
# then Lichtblick → ws://localhost:8765 sees the replayed topics.
```
**Two distinct replay paths — don't conflate them:**
- **`pool_record.sh replay` / `ros2 bag play`** re-publishes the bag onto live ROS topics →
  needs `ros-humble-rosbag2-storage-mcap` **and** `duburi_interfaces` built. Use it when you
  also want `vision_display` or live `ros2 param set` against the replayed stream.
- **Open the `.mcap` file directly in Lichtblick** (*Open local file*) → needs **no ROS, no
  plugin, no `duburi_interfaces`** (MCAP carries the schemas, so `DuburiState` still renders).
  Best on a bare laptop.

> **Note — the dev box is a distrobox** (`auv-ros2`, Ubuntu 22.04, ROS Humble). Lichtblick
> is a GUI app; it launches fine with the container's `DISPLAY`/Wayland passthrough. If it
> won't open a window on the host, run it from the host instead — the connection is just a
> WebSocket, so where the viewer runs doesn't matter as long as it can reach the Jetson IP.

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

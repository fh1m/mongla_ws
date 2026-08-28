# Mongla — AUV ROS2 Control Stack

> **Project**: Mongla — a ROS2 / ArduSub control stack for `vectored_6dof` AUVs.
> **Test platform**: Duburi 4.2 (`vectored_6dof`, 8x T200, Pixhawk 2.4.8 + ArduSub 4.x).
> **This codebase**: `~/Ros_workspaces/duburi_ws` (ROS2 Humble, Ubuntu 22.04 in distrobox).
> Workspace folder name and the `/duburi/*` action namespace are kept for backwards compatibility with the test vehicle's tooling.

> **Precedence note for agents:** if anything below contradicts the
> actual package layout in `src/`, the package layout wins. The
> canonical hardware reference is
> [`.claude/context/vehicle-spec.md`](.claude/context/vehicle-spec.md);
> tracked code bugs are in
> [`.claude/context/known-issues.md`](.claude/context/known-issues.md).
> Some legacy `.claude/context/*.md` files (notably `proven-patterns.md`)
> describe historical 2023/2025 codebases, not this workspace.

> **Mongla = the soul; Duburi 4.5 + Dubomini 2.0 = the bodies.** Mongla is the
> codebase (`duburi_ws`) — one ROS 2 Humble control/mission/vision/sim brain that
> runs on two competition bodies: **Duburi 4.5** (primary; sensors + manipulators)
> and **Dubomini 2.0** (agile, manipulator-free). The Duburi 4.5 public spec itself
> names the software "Mongla (duburi_ws)". `duburi_ws`, `/duburi/*`, and "4.2" are
> kept for back-compat — **do not bulk-rename them.**
>
> **RoboSub 2026 framing (committed scope vs what's built — read before trusting either).**
> Per the tech-lead reconciliation decision (2026-05-31, P0.1), the **committed
> 2026 target** is the full TDR (`TDR26_BRACU_Duburi.pdf`): a dual-vehicle run —
> **Duburi 4.5** (primary; sensors + grabber/dropper/torpedo) and **Dubomini 2.0**
> (agile, 8-thruster, no DVL/manipulators) — coordinated over **IVC**, sequenced by
> a **YASMIN FSM**, with the full 7-task set. The detector is **YOLO11** (this
> *corrects* the TDR's YOLO26 line — YOLO11 is the committed family; YOLO26 is
> legacy/backwards-compat only).
>
> Read every doc through **three states, never blurred:**
> - **BUILT & TESTED (phase 1, today):** the proven 4.2-derived **single-vehicle**
>   Duburi stack — imperative `detected()` missions, YOLO11, Gate / Return /
>   search-align (≈800 pt), the control / MAVLink / vision core. This is what runs.
> - **COMMITTED, NOT YET IMPLEMENTED (phase 2):** Dubomini control path, **IVC**,
>   the **YASMIN FSM**, the remaining tasks (Slalom / Bins / Torpedo / Octagon /
>   path-markers), underwater preprocessing, and ESP32-serial payload actuation.
>   These are committed build tickets with **zero or partial code today** — never
>   describe them as running.
> - **`detected()` is NOT "instead of an FSM":** it is the prototyping +
>   per-subsystem unit-test + FSM-fallback layer. The committed YASMIN FSM (phase 2)
>   wraps these same DSL verbs as states.
>
> The workspace name, the `/duburi/*` namespace, and "4.2" in hardware tables are
> deliberately kept for back-compat — **do not bulk-rename them.**
>
> **Central development board (start here for status/bugs/fixes/tickets):**
> [`.claude/context/development-board.md`](.claude/context/development-board.md).
> Supporting detail: full audit + gap matrix
> [`.claude/context/robosub-2026-audit.md`](.claude/context/robosub-2026-audit.md);
> phase schedule [`.claude/context/robosub-2026-roadmap.md`](.claude/context/robosub-2026-roadmap.md).
> When docs and the TDR disagree, **code is ground truth** — fix the gap or mark the
> claim as committed-phase-2; never edit docs to assert a capability the code lacks.

---

## 1. Hardware Overview (test platform: Duburi 4.2 hull → 4.5 build)

> Full spec lives in [`.claude/context/vehicle-spec.md`](.claude/context/vehicle-spec.md). Short table here.

| Component             | Spec                                                         |
|-----------------------|--------------------------------------------------------------|
| Hull                  | Octagonal, **Marine 5083 aluminum**, in-house                |
| Frame type (ArduSub)  | `vectored_6dof` (8× T200) — same as BlueROV2 Heavy           |
| Flight controller     | Pixhawk 2.4.8 running ArduSub 4.x                            |
| Companion             | Raspberry Pi running BlueOS                                  |
| Main SBC              | Nvidia Jetson Orin Nano (all ROS2 nodes live here)           |
| Depth sensor          | Bar30 (read via ArduSub `AHRS2.altitude`)                    |
| External IMU          | **ESP32-C3 + BNO085** over USB CDC, opt-in via `yaw_source`  |
| DVL                   | Nortek Nucleus1000 @ `192.168.2.201` — **stub only**         |
| Cameras               | Blue Robotics Low-Light HD USB (forward + downward)          |
| Tether                | FathomX power-over-Ethernet                                  |
| Power                 | Dual LiPo (propulsion + compute on isolated rails)           |
| Payload               | Slingshot torpedo, aluminum grabber (current-sensed), solenoid dropper |

> **Sim proxy:** Gazebo runs a BlueROV2 Heavy-derived model because it shares the `vectored_6dof` frame. Hull shape and exact mass differ; control behavior matches. The simulator is **in this repo** at [`sim/`](sim/) — see §3a.

> **Why no VectorNav**: TDR Appendix A lists VN200; we use BNO085 instead — see `vehicle-spec.md` §"Why BNO085 instead of the TDR's VectorNav VN200".

---

## 2. Network Topology (AUV Internal Ethernet)

```
[Onboard Ethernet Switch]
       ├── Jetson Orin Nano  → 192.168.2.69   (static, ROS2 host, UDP 14550 listener)
       ├── Raspberry Pi 4B   → 192.168.2.1    (BlueOS — MAVLink router, web UI)
       │      Gateway         → 192.168.2.2
       ├── DVL Nucleus1000   → 192.168.2.201  (driver TODO)
       └── Pixhawk 2.4.8     → via BlueOS over USB

MAVLink endpoint (configured in BlueOS web UI):
  Name: "inspector"  |  Type: UDP Client
  IP: 192.168.2.69 (Jetson)  |  Port: 14550

Ground Station → Remote Desktop / SSH to Jetson (192.168.2.69)
              → BlueOS UI via http://192.168.2.1
```

Connection strings live in `src/duburi_manager/duburi_manager/connection_config.py` under `PROFILES`. Default for every profile is `udpin:0.0.0.0:14550` (Jetson is the listener; BlueOS pushes to it).

---

## 3. Operating Modes

`auv_manager_node` ships with `mode:=auto` as the default. The
`resolve_mode` helper in
`src/duburi_manager/duburi_manager/connection_config.py` probes the
runtime environment and picks one of the four legacy profiles below
without operator intervention:

| Probe                                                  | Picked profile |
|--------------------------------------------------------|----------------|
| UDP `14550` already in use (BlueOS pushing MAVLink)    | `pool`         |
| Pixhawk USB CDC present (`/dev/serial/by-id/*ardupilot*` or `/dev/ttyACM0`) | `desk` |
| Neither                                                | `sim`          |

| `mode:=`   | Connection string         | Use case                                         |
|------------|---------------------------|--------------------------------------------------|
| `auto`     | (resolved at startup)     | Plug-and-play default. Banner prints what was picked. |
| `sim`      | `udpin:0.0.0.0:14550`     | Docker dev + Gazebo SITL (ArduSub `--out` to us) |
| `pool`     | `udpin:0.0.0.0:14550`     | Pool testing — Jetson on AUV, BlueOS pushes      |
| `laptop`   | `udpin:0.0.0.0:14550`     | Tether laptop on the switch instead of Jetson    |
| `desk`     | `udpin:0.0.0.0:14550`     | Pixhawk plugged directly via USB through BlueOS  |

> All four explicit profiles use the same listener line. The difference is
> documentation + the printed startup banner / sanity hints. There is
> **no** `HARDWARE` mode.

Run `ros2 run duburi_manager bringup_check` at the start of every
session — a RoboSub-proof per-subsystem preflight (12 sections, A–L) that
verifies each subsystem is not just present but **usable**: compute/env
(ROS sourced, duburi pkgs import, disk), vision deps (numpy<2, system cv2,
trackers — the JetPack E1–E3 traps), serial drivers (**ch341 present +
brltty absent** — the payload CH340 trap, E5), network, **MAVLink** (real
autopilot heartbeat, ARMED state, flight mode, battery V — filters out
BlueOS/GCS heartbeats), Pixhawk USB (BNO/payload VID excluded), BNO085,
DVL, **payload** (auto-detect + safe serial-link verify with the `'0'`
byte), cameras (forward+downward by-path count), vision models (TensorRT
`.engine` vs slow `.pt`), and Jetson MAXN power. Each line is PASS/WARN/FAIL;
exit `0` unless a FAIL. `--strict` makes any WARN exit non-zero (hard
pre-mission gate); `--skip-mavlink` skips the UDP 14550 probe.

### 3a. Simulator (`sim/`) — in-repo Gazebo + ArduSub SITL

The simulator lives at [`sim/`](sim/): a **second colcon workspace inside this
repo** — Gazebo Harmonic, ArduSub SITL, the SAUVC pool with courses and props,
front + bottom cameras, ground truth, and a FastAPI/React operator lab.

```text
duburi_ws/
  src/   autonomy  — six packages; UNCHANGED, same test set
  sim/   simulator — six duburi_sim_* packages
    COLCON_IGNORE  — keeps the root colcon build at exactly six packages
    build_sim.sh   — colcon build --base-paths src (the marker self-ignores otherwise)
```

Build autonomy first, then the sim; source in the same order (the sim's
`stack.launch.py` includes autonomy launch files by share directory):

```bash
./build_dubomini.sh
cd sim && ./build_sim.sh

source /opt/ros/humble/setup.bash
source ~/Ros_workspaces/duburi_ws/install/setup.bash
source ~/Ros_workspaces/duburi_ws/sim/install/setup.bash
export GZ_IP=127.0.0.1
```

Terminal by terminal (full walkthrough: [README](README.md#-simulator--gazebo--ardusub-sitl),
operator detail: [`sim/README.md`](sim/README.md)):

```bash
# T1 — world + SITL. `stop` FIRST, always: one sim only.
ros2 run duburi_sim_bringup duburi_sim stop
ros2 run duburi_sim_bringup duburi_sim sim          # --headless / course:=sauvc26_final

# T2 — this codebase against the sim (MAVLink UDP 14550)
export DUBURI_WS=~/Ros_workspaces/duburi_ws
ros2 run duburi_sim_bringup duburi_sim stack --no-vision

# T3 — prove the loop before trusting a mission
ros2 run duburi_sim_bridge contract_check
ros2 run duburi_sim_bringup duburi_sim smoke

# T4 — operator lab (binds 127.0.0.1; port in /tmp/duburi-$USER/lab_port.txt)
ros2 run duburi_sim_bringup duburi_sim lab
```

> **`mavlink_check` runs with the stack DOWN** — it binds 14550 too, so against a
> live manager it silently steals the autonomy link and looks like a sim fault.
>
> **`flight_controller:=pixhawk`** is required on `srot` and **inert on `main`**:
> `IncludeLaunchDescription.execute()` raises only for *missing required* args, so
> extra keys are dropped with no log line. [`test_sim_contract_drift.py`](src/duburi_manager/test/test_sim_contract_drift.py)
> asserts on it for exactly that reason — the launch itself will never complain.
>
> **The sim has a DVL** (Gazebo ships one natively). `duburi_sim stack` defaults
> to `yaw_source=sim_dvl` — MAVLink AHRS heading + DVL position — so the `*_dist`
> verbs close a real loop in sim. Without a DVL they now **RAISE** rather than
> falling back to a timed guess: that fallback drove 2.361 m for a 1.0 m command
> and reported success. See [`sim/.context/DVL_AND_SONAR.md`](sim/.context/DVL_AND_SONAR.md)
> — especially the axis mapping and the sim-time-vs-wall-clock trap, both of
> which produce a sensor that looks fine and reads wrong. **Sonar does not exist
> in Gazebo Harmonic**; use `gpu_lidar` or the DVL's altitude.
>
> **RViz** (`duburi_sim rviz`) draws the robot model, TF, DVL beams and — the
> point — **ground truth and the stack's believed pose at once**, so the AHRS2
> offset is visible rather than tabulated. URDF is GENERATED from the same
> `configs.yaml` as the SDF; there is no hand-written robot description to drift.
> Nothing published TF before this, which is why RViz was previously unusable.
>
> **End-to-end check:** `ros2 run duburi_planner mission sim_shakedown` — arm,
> hold depth, out, back, surface, disarm. Symmetric legs *are* the return-to-origin
> mechanism; measure the residual against `/duburi/sim/ground_truth`. Measured
> 0.266 m and 0.200 m horizontal, along-track only 0.037/0.011 m — cross-track
> heading drift dominates.
>
> **Sim depth readback is offset; the hull is not.** Depth comes from
> `AHRS2.altitude` (secondary DCM) while ArduSub controls on EKF3. Measured error
> **0.33 m at the surface, ~0.16 m at depth** — not constant. Consequences:
> `surface()` never confirms (targets 0.00, AHRS2 plateaus ~−0.4) and
> `mission_reset`'s baro re-zero is REFUSED (`|−0.36| > 0.30` surface bound), the
> two interlocking. `set_depth` is fine in substance — true depth measured within
> 2.5 cm of command. Pool floor is **1.6 m**; deeper targets bottom out. Do NOT
> "fix" by changing the depth source: AHRS2 is the pool-verified hardware path.
> Full table: [`sim/.context/TROUBLESHOOTING.md`](sim/.context/TROUBLESHOOTING.md).
>
> **Water turbidity comes from `underwater_fx`, NOT the world's `<fog>`.** Measured
> 2026-08-28: gz-sim 8 ignores `<scene><fog>` on camera renders entirely — 18 m →
> 3 m left the far wall crisp — so the `lighting:` presets were decorative and every
> dataset had identical clarity whatever preset its course named. Each preset now
> also carries an `fx` block, written beside the world as `<course>.fx.yaml` and
> loaded by `bridge.launch.py`. Check it took with
> `ros2 param get /underwater_fx turbidity` (murky→0.8, competition→0.45). Never
> tune turbidity in a `.world`; nothing changes. Attenuation is still uniform, not
> per-pixel — see [`sim/.context/TROUBLESHOOTING.md`](sim/.context/TROUBLESHOOTING.md).
>
> **Pointing vision at a sim camera topic used to receive NOTHING.** `RosTopicCamera`
> subscribed RELIABLE; every camera publisher (`ros_gz` image_bridge, `underwater_fx`,
> BlueOS) is BEST_EFFORT, and rclpy answers a QoS mismatch with one WARN and then
> silence — clean launch, healthy nodes, zero frames. Now `qos_profile_sensor_data`.
> Run the full pipeline on Gazebo cameras with
> `vision_dual.launch.py fwd_topic:=…/front_camera/image_fx dwn_topic:=…/bottom_camera/image_fx paused:=false`
> — **`paused:=false` matters**: the launch defaults to paused (missions resume the
> detector they need), so without it the HUD reads `det=ERR dets=0` and looks broken.
>
> **Vision in sim runs BOTH cameras** — `duburi_sim stack` gives
> `/duburi_detector_forward` (sim front cam) and `/duburi_detector_downward`
> (bottom cam). Until 2026-08-28 `vision:=true` started **nothing**:
> `IncludeLaunchDescription` does not scope `launch_arguments`, so the manager
> include's `vision: 'false'` leaked out and disabled the vision include via its
> own `IfCondition`, silently. A `GroupAction(scoped=True)` fixes it and a drift
> test asserts the scope survives. Any sim note saying "run `--no-vision`"
> predates this.
>
> **`bin_fire_blood.pt` has NO `bin` class** — embedded names are
> `{0: blood, 1: fire}`. Asking for `classes:=bin` detects nothing, silently.
>
> **TWO COMPETITIONS now.** `spec/arena.yaml` split into `spec/sauvc.yaml` +
> `spec/robosub.yaml`; a course picks one with `competition: robosub`. RoboSub's
> pool is **2.1 m** deep vs SAUVC's 1.6 m, which is why this needed a refactor and
> not a config line: depth-spanning props bake pool depth in at generation time,
> and textures are sized from the pool (one shared set stretched over a different
> pool is wrong with **no error**). 14 RoboSub props + 6 courses
> (`robosub26_full`, `rs_task_*`). The gate's red/black **asymmetry** and the
> torpedo board's two **openings** are real geometry, because scoring depends on
> them. Acoustics: `ros2 run duburi_sim_bridge hydrophone` reports bearing /
> elevation / SNR per ping with dropouts, range-growing noise and **multipath
> ghosts** (a confident wrong bearing, not noise — it is what breaks homing that
> averages). Verified against ground truth: median 5.4° error, 7 % ghosts. Full
> detail: [`sim/.context/ROBOSUB_AND_ACOUSTICS.md`](sim/.context/ROBOSUB_AND_ACOUSTICS.md).
>
> **Props follow the SAUVC rulebook** and every prop can be spawned anywhere at
> runtime: `ros2 run duburi_sim_scenarios props add <model> <name> <x> <y>`.
> Six courses, three of them single-task. Textures are generated
> (`gen_world.py --all`), never hand-committed.
>
> **Vision in sim has two silent failure modes.** `duburi_sim stack` defaults to
> `model:=gate_rescue_repair`; the `.pt` is gitignored (mirrored from `~/models`)
> and a missing `.yaml` sidecar yields an empty allowlist and a silent `[]` every
> frame. Require the model stem, a non-empty allowlist, AND the `[ align … ]` line.

**Bare SITL without Gazebo** still works for arming/depth/motion-verb checks:

```bash
sim_vehicle.py -L RATBeach -v ArduSub -f vectored_6dof --model=JSON \
  --out=udp:0.0.0.0:14550 --out=udp:127.0.0.1:14551 --console
```

Sim-tuned **detection thresholds and vision gains do not transfer** — sim imagery
is too clean. Control behaviour and every `/duburi/move` verb do.

---

## 4. Software Architecture

### 4.1 Package Map (real, today)

```
duburi_ws/src/
├── duburi_interfaces/    # ROS2 message + action defs
│   ├── action/Move.action        # single dispatcher action
│   └── msg/DuburiState.msg       # typed state snapshot for /duburi/state
├── duburi_control/       # MAVLink layer + Duburi facade
│   └── duburi_control/
│       ├── pixhawk.py            # Pixhawk class — arm / mode / RC / setpoints + [MAV ] DEBUG trace
│       ├── commands.py           # COMMANDS registry (single source of truth)
│       ├── motion_easing.py      # smoothstep / smootherstep / trapezoid_ramp
│       ├── motion_writers.py     # shared constants + Writers (lock-aware) + thrust_loop
│       ├── motion_yaw.py         # yaw_snap + yaw_glide (Ch4 rate override)
│       ├── motion_forward.py     # drive_forward_* + arc (Ch5 / Ch5+Ch4 RC override)
│       ├── motion_lateral.py     # drive_lateral_* (Ch6 RC override)
│       ├── motion_depth.py       # hold_depth + prime_alt_hold (one-shot SET_POSITION_TARGET, then ALT_HOLD)
│       ├── motion_vision.py      # align_loop + move_loop (P-on-pixel, gain=speed cap, Outcome codes)
│       ├── heading_lock.py       # background Ch4 yaw-rate streamer (yaw_source-driven)
│       ├── heartbeat.py          # 5 Hz neutral RC override -- prevents FS_PILOT_INPUT disarm
│       ├── vision_verbs.py       # VisionVerbs mixin -- vision_align / vision_move on Duburi (release_yaw aware)
│       ├── duburi.py             # Duburi facade: lock + dispatch + heading_lock + heartbeat owner
│       └── errors.py             # MovementError / MovementTimeout / ModeChangeError
├── duburi_manager/       # ROS2 node, action server, telemetry
│   └── duburi_manager/
│       ├── auv_manager_node.py   # owns MAVLink + /duburi/move ActionServer + VisionState pool
│       ├── vision_state.py       # per-camera Detection2DArray subscriber + bbox_error()
│       └── connection_config.py  # PROFILES + NETWORK constants
├── duburi_planner/       # mission planner: Python client + CLI + mission scripts
│   └── duburi_planner/
│       ├── client.py             # blocking ActionClient wrapper (DuburiClient)
│       ├── duburi_dsl.py         # DuburiMission DSL (duburi.* open-loop verbs + .vision facade)
│       ├── vision_dsl.py         # duburi.vision.align + duburi.vision.move (+ fallback orchestration)
│       ├── model_context.py      # duburi.models registry + ClassRef (auto model+class switch)
│       ├── cli.py                # argparse auto-built from COMMANDS (`duburi` entry)
│       ├── mission.py            # `mission` runner that dispatches into missions/<name>.run
│       ├── missions/
│       │   ├── competition_config.py          # pool-day headings / depths / tuning
│       │   ├── task_{gate,slalom,bin,torpedo,return}.py  # detected()-paradigm task chunks
│       │   ├── task_full_2026.py              # flat combinator (chains all 5 chunks)
│       │   ├── fsm_{slalom,bin,torpedo,return,full_2026}.py  # ★ YASMIN FSM launchers
│       │   ├── {gate_flare_fsm,prequal_fsm,gate_then_bin_fsm}.py  # prior FSM missions (kept)
│       │   ├── {gate_prequal,robosub_prequal,gate_flare_prequal,gate_flare_autonomous}.py
│       │   ├── {robosub_gate_rescue,pool_day_practice,pool_day_torpedo}.py  # pool-day runs
│       │   └── demo_{arc,find_person,heading_lock,move_see,square,pursue}.py
│       └── state_machines/       # YASMIN FSM layer (BUILT) — see fsm-guide.md
│           ├── core/{outcomes,blackboard,vehicle_profile,base_state}.py
│           ├── states/{navigation,vision,utility}.py   # nav: Arm/Disarm/SetDepth/LockHeading/Move*/Turn/Surface; vision: VisionSearch/VisionAlign/VisionMove; utility: Countdown/Pause/LogScore/SetDetector/Fire/StyleRoll
│           └── plans/{gate_flare,prequal,gate_then_bin,slalom,bin_drop,torpedo_fire,return_gate,full_competition}.py
├── duburi_sensors/       # YawSource abstraction (sensors-only, read-only)
│   ├── duburi_sensors/
│   │   ├── factory.py            # make_yaw_source(name) — dvl|bno085|bno085_dvl|mavlink_ahrs
│   │   ├── sensors_node.py       # standalone diagnostic node (no thrusters)
│   │   └── sources/{base,mavlink_ahrs,bno085,nucleus_dvl,nucleus_parser,composite_bno_dvl}.py
│   ├── firmware/esp32c3_bno085.md
│   └── config/sensors.yaml       # yaw_source / bno085_port / nucleus_dvl_* / dvl_auto_connect
└── duburi_vision/        # Camera factory + YOLO11 detector + viz + depth estimation
    ├── duburi_vision/
    │   ├── camera_node.py / detector_node.py / tracker_node.py / depth_estimation_node.py
    │   ├── cameras/{webcam,ros_topic,jetson_stub,blueos_stub}.py
    │   ├── detection/{detector,yolo,gpu,messages}.py
    │   ├── tracking/             # Roboflow OC-SORT/ByteTrack (Tracker ABC) + Kalman smoother
    │   ├── depth/depth_estimation_node.py   # ONNX Depth Anything V2-Small + bbox fallback
    │   └── utils/{check_pipeline,check_thrust}.py
    ├── config/{cameras,detector}.yaml
    ├── models/                   # *.pt weights (gitignored) + committed class-index YAMLs
    └── launch/{vision,vision_dual,video}.launch.py  # 1-cam + 2-cam live + 2-cam dataset-video preset; detector node = duburi_detector_<camera>
```

> **Adding a new command**: add a row in `duburi_control/commands.py` and a same-named method on `Duburi`. The action server, the `duburi` CLI, and the Python `DuburiClient` all pick it up automatically — no other file needs editing.

> **Mission DSL**: prefer
> [`DuburiMission`](src/duburi_planner/duburi_planner/duburi_dsl.py) over
> the raw client when authoring missions. `duburi.move_forward(...)` and
> `duburi.vision.align(...)` / `duburi.vision.move(...)` share one object with
> sticky `duburi.camera` + `duburi.target` context. Vision verbs fall back to
> live `vision.*` ROS params when overrides are unset, so deck-side tuning works
> without editing mission code. Full cookbook + samples:
> [`.claude/context/mission-cookbook.md`](.claude/context/mission-cookbook.md).

> Packages **not** in this repo (despite older context files mentioning them): `duburi_bringup`, `duburi_driver`, `duburi_teleop`, `duburi_mission`. They were aspirational sketches; ignore them when you read `proven-patterns.md` etc.

### 4.2 Data flow (real)

```
[duburi CLI]   ──┐
[mission run]  ──┤
[Python client]──┼──/duburi/move (action goal)──→ [auv_manager_node]
                                                      │
                                                      ├──→ Duburi facade (lock + dispatch via COMMANDS)
                                                      │       ├── motion_yaw     (Ch4 RC rate override ×10 Hz)
                                                      │       ├── motion_forward (Ch5 RC override; arc = Ch5+Ch4 ×20 Hz)
                                                      │       ├── motion_lateral (Ch6 RC override ×20 Hz)
                                                      │       ├── motion_depth   (SET_POSITION_TARGET_GLOBAL_INT ×5 Hz)
                                                      │       └── heading_lock   (Ch4 RC rate override ×50 Hz, background)
                                                      │
                                                      ├──→ Pixhawk ──UDP 14550──→ [BlueOS] ──USB──→ [Pixhawk / ArduSub]
                                                      │                                         telemetry: AHRS2 50Hz, RC 5Hz, BAT 1Hz
                                                      └──→ /duburi/state (DuburiState, on change)

[duburi_sensors.sensors_node]   ←── separate, diagnostic-only, never runs in mission path
```

> Telemetry rates above are explicitly pinned at startup via `MAV_CMD_SET_MESSAGE_INTERVAL` in `auv_manager_node.MESSAGE_RATES` — without this, ArduSub picks defaults (~4 Hz for AHRS2) which silently caps loop tightness.

### 4.3 Node responsibilities

| Node                             | Package         | Owns                                                      |
|----------------------------------|-----------------|-----------------------------------------------------------|
| `auv_manager_node` / `auv_manager` | `duburi_manager` | The single MAVLink connection, `/duburi/move` ActionServer, telemetry publisher, ROS params |
| `sensors_node`                   | `duburi_sensors`| Standalone yaw-source diagnostic — does NOT touch thrusters or arming |
| `camera_node`                    | `duburi_vision` | Camera source → `/duburi/vision/<cam>/image_raw` + `camera_info` |
| `detector_node`                  | `duburi_vision` | Subscribe `image_raw` -> YOLO11 (yolov11n) -> `/duburi/vision/<cam>/detections` + `image_debug` + `classes_filter` |
| `tracker_node`                   | `duburi_vision` | **Roboflow `trackers`** (OC-SORT default, `tracker_type=`) + Kalman smoother; `detections` → `tracks` (stable IDs + coasted boxes during gaps). OC-SORT re-associates the same id after a dropout (vs ByteTrack spawning a duplicate) |
| `depth_estimation_node`          | `duburi_vision` | Monocular proximity (`vis_range`); ONNX Depth Anything V2-Small + bbox-area fallback. `depth:=true`. |
| `vision_display`                 | `duburi_vision` | Mission-control HUD; overlays detections/tracks/vis_range + UI strip. **D** toggles depth inset. |
| `vision_node`                    | `duburi_vision` | In-process camera+detector smoke test (cousin of `sensors_node`) |

There is exactly **one** node that touches `pymavlink` in the live mission path: `auv_manager_node`. The `duburi` CLI, the `mission` runner, and any custom Python script are ROS2 ActionClients of `/duburi/move` -- all live in `duburi_planner`.

---

## 5. MAVLink / ArduSub Patterns (live code)

> Full detail: [`.claude/context/mavlink-reference.md`](.claude/context/mavlink-reference.md) · [`ardusub-canon.md`](.claude/context/ardusub-canon.md).
> Implementation source: `src/duburi_control/duburi_control/pixhawk.py`.

| Call | What it does |
|---|---|
| `pixhawk.arm()` / `pixhawk.disarm()` | Returns `(ok, reason)`; reason is MAV_RESULT name, NO_ACK, or `NOT_ARMED_AFTER_ACK: <pre-arm STATUSTEXT>`. **Client deadline covers the real budget** — the DuburiClient result-backstop is a floor not a ceiling (arm→30 s, disarm→35 s), so a slow-but-legit arm no longer trips a false 12 s "doesn't arm" timeout ([`known-issues.md`](.claude/context/known-issues.md) P1). arm honours a mid-arm abort with a **verified, fail-closed** disarm (P2). |
| `pixhawk.set_mode("ALT_HOLD")` | Polls heartbeat for ACK (SET_MODE gives no direct ACK) |
| `pixhawk.send_rc_override(forward, lateral, throttle, yaw)` | PWM 1100–1900; 1500=neutral; 65535=release |
| `pixhawk.send_rc_override(yaw=pwm)` | Ch4 yaw-rate; ArduSub treats Ch4≠1500 as a pilot yaw-rate command (bypasses its compass-driven heading hold). `send_rc_yaw_only(pwm)` writes only Ch4 (heading-lock path). |
| `pixhawk.set_target_depth(-1.5)` | Negative = below surface; requires ALT_HOLD |
| `pixhawk.get_attitude()` | `{'yaw': deg, 'depth': m, ...}` — AHRS2-backed, cached |
| `duburi.fire(n)` | Fire payload channel n via ESP32 serial (1/2=torpedo, 3/4=dropper); `duburi.payload_ready()` to check |
| `duburi.mission_reset()` | **Call at start of every `run()`.** Stops heading lock, clears `_abort_event`, sends RC neutral, **and re-zeroes the barometer (auto `calibrate_depth`, disarmed-gated)** so depth starts true. Safe before arm (`_UNARM_SAFE`). Prevents state carry-over across back-to-back pool runs. |
| `duburi.calibrate_depth()` | Re-zero the Bar30 at the **surface** (baro ground-pressure, QGC "Calibrate Pressure") so `depth` reads 0 before a dive — fixes the +0.01..0.1 m pre-dive drift. DISARMED-only (surface proxy); reads fresh depth **before/after** and verifies the re-zero took (`[BARO ] pre→post`); >0.30 m pre = refused. Auto-run by `mission_reset`; standalone `ros2 run duburi_planner duburi calibrate_depth`. `MAV_CMD_PREFLIGHT_CALIBRATION` p3=1 → `AP::baro().calibrate()`, ~2.5 s settle, **no reboot**. |

**RC direction (current Duburi hull, pool-verified 2026-06):** Ch4 > 1500 = yaw RIGHT; Ch5 > 1500 = forward; Ch6 > 1500 = strafe RIGHT. (The 2023 reference hull was RC4-reversed — Ch4 > 1500 = LEFT there; polarity is an `RC4_REVERSED`/frame-config property, so re-confirm per hull with a bare `Ch4=1600` check. `heading_lock`/`motion_yaw` are polarity-correct regardless — they derive Ch4 sign from `heading_error` math, never from this label.)
**Yaw settle (`yaw_left/right/turn`):** `motion_yaw._YawPID` closes Ch4 on `yaw_source` (BNO) with a stiction-breaking speed floor that **tapers to 0 across an approach band** (`YAW_APPROACH_BAND_DEG`) so the hull eases into the `YAW_TOL_DEG` (2°) lock instead of limit-cycling on a hard floor. If yaw wobbles/TIMEOUTs on pool day, tune in this order: confirm it declares locked → tighten `YAW_TOL_DEG` for precision → adjust band / `YAW_KI`. See [`known-issues.md`](.claude/context/known-issues.md) (yaw-wobble entry). `heading_lock` is a separate 50 Hz continuous hold.
**Heartbeat:** owned by `auv_manager_node` ROS2 timer — do not roll your own.
**Stream rates:** pinned at startup via `MAV_CMD_SET_MESSAGE_INTERVAL` (AHRS2=50 Hz, RC=5 Hz, BAT=1 Hz).

---

## 6. Control philosophy — ArduSub does the inner loop

ArduSub's onboard 400 Hz stabilizer + EKF3 owns the **inner** loop. **Depth** is
owned entirely by ArduSub (we stream a setpoint). **Yaw is split:** ArduSub's rate
loop closes the *yaw rate* from our Ch4 stick, but the *absolute heading* loop is
closed in Python against `yaw_source` (BNO085/AHRS) — ArduSub's compass is untrusted
inside the aluminum hull, so we drive Ch4 as a rate command and never hand ArduSub
an absolute-attitude setpoint. (There is no `SET_ATTITUDE_TARGET` anywhere in the code.)

> **Parallel BNO→EKF3 feed (live).** Independently of the Ch4 heading loop, when
> `yaw_source` is BNO-based the manager's `_mocap_tick` streams BNO yaw into ArduSub's
> EKF3 at 20 Hz via `ATT_POS_MOCAP` (quaternion built with `math.radians()` —
> unitless on the wire, so no radians caveat). It only takes effect with FC params
> `VISO_TYPE=1` + `EK3_SRC1_YAW=6` on a 2 MB fmuv3 build; the manager verifies these
> at startup and WARNs on mismatch. This is an *EKF correction*, not the heading
> authority — `HeadingLock` (Python Ch4) is still primary. See
> [`future/future-bno-into-ekf.md`](.claude/context/future/future-bno-into-ekf.md).

| Axis      | Setpoint message                  | Loop that closes it           | Our role                       |
|-----------|-----------------------------------|-------------------------------|--------------------------------|
| Yaw       | `RC_CHANNELS_OVERRIDE` Ch4 rate (10 Hz) | ArduSub rate loop + Python heading PID | close heading on yaw_source |
| Depth     | `SET_POSITION_TARGET_GLOBAL_INT` (5 Hz) | ArduSub ALT_HOLD position PID | stream + watch AHRS depth      |
| Forward   | `RC_CHANNELS_OVERRIDE` Ch5 (20 Hz)| open loop (timed thrust)      | shape the thrust envelope      |
| Lateral   | `RC_CHANNELS_OVERRIDE` Ch6 (20 Hz)| open loop (timed thrust)      | shape the thrust envelope      |
| Arc       | `RC_CHANNELS_OVERRIDE` Ch5 + Ch4 (20 Hz, single packet) | open loop | curved car-style trajectory    |
| Heading lock | `RC_CHANNELS_OVERRIDE` Ch4 rate (50 Hz, background) | ArduSub rate loop + Python P-loop | continuous yaw hold across other commands |
| Vision lateral | `RC_CHANNELS_OVERRIDE` Ch6 (20 Hz) | vision loop inside manager | +ex → Ch6 > 1500 → strafe RIGHT (no negation) |
| Vision yaw     | `RC_CHANNELS_OVERRIDE` Ch4 rate (20 Hz) | vision loop inside manager | +ex → yaw toward target (no negation; same polarity as vision lateral — pool-verified 2026-06) |

The two ROS params `smooth_yaw` / `smooth_translate` (both default `false`) optionally shape the *setpoint* (smootherstep / trapezoid_ramp) before it reaches the autopilot — they don't replace the autopilot's inner loop.

> Earlier revisions kept `movement_pids.py` (`DepthPID` / `YawPID`) as a "hot-fix fallback" reference. That file has been removed — ArduSub's inner loop is the only PID in the live path. If you need the math again, see `.claude/context/pid-theory.md` or pull it from git history.

---

## 7. JSF-AV Principles (still apply)

Adapted for our context:

| Principle                       | What it means here                                                  |
|---------------------------------|---------------------------------------------------------------------|
| Single entry / exit             | Each motion helper has one return path; facade is a dispatch table  |
| No dynamic allocation in loop   | RC arrays are reused, no list-comp inside 20 Hz loops               |
| Bounded loops                   | Every `while` has an explicit timeout                               |
| Fail-safe defaults              | On any error → `Duburi.stop()` (active RC neutral) or `pause()` (release) |
| Clear interfaces                | Cross-package surface = `Move.action` + `DuburiState.msg` + `Pixhawk` verbs |
| No global mutable state         | All state lives on `Duburi` / `Pixhawk` instances                   |
| Defensive input validation      | `percent_to_pwm` clamps; CLI argparse rejects out-of-range values   |
| Deterministic timing            | ROS timers + `time.monotonic()`; no `time.sleep` in callbacks       |

---

## 8. ROS2 surface (real, today)

### Action

- `/duburi/move` — `duburi_interfaces/action/Move`
  - One verb per goal; `auv_manager_node.execute_callback` dispatches via the `COMMANDS` registry.
  - See [`.claude/context/ros2-conventions.md`](.claude/context/ros2-conventions.md) for the verb list and field semantics.

### Topic

- `/duburi/state` — `duburi_interfaces/msg/DuburiState`
  - Typed snapshot (`armed`, `mode`, `yaw_deg`, `depth_m`, `battery_voltage`) with `std_msgs/Header`. Missing numerics are `NaN`, missing strings are `''`. Published only when something changes (or every ~1 s as a heartbeat).

### Key ROS params on `auv_manager_node`

> **Default column = `bringup.launch.py` (operator) default.** The node's *own*
> `declare_parameter` defaults differ for two: `mode=auto` and
> `yaw_source=mavlink_ahrs` (a bare `ros2 run duburi_manager start` gets those;
> the launch file overrides to `pool`/`dvl` for pool use). Full param table +
> the split: [`ros2-conventions.md`](.claude/context/ros2-conventions.md).

| Param | Default (launch) | Notes |
|---|---|---|
| `mode` | `pool` | `pool`\|`sim`\|`auto`\|`laptop`\|`desk` (see §3); **node default `auto`** |
| `yaw_source` | `dvl` | `dvl`\|`bno085_dvl`\|`bno085`\|`mavlink_ahrs` — also drives VehicleProfile.auto(); **node default `mavlink_ahrs`** |
| `dvl_auto_connect` | `true` | Background retry loop; `dvl_connect` verb for manual override |
| `nucleus_dvl_host` | `192.168.2.201` | DVL TCP host; port `9000`, password `nortek` |
| `bno085_port` | `auto` | ESP32-C3 HWCDC port; `auto` = VID/PID scan (303a:1001); explicit path skips scan |
| `payload_port` | `auto` | CH340 payload board; `auto` = VID/PID scan (1a86:7523); explicit path skips scan |
| `smooth_yaw` / `smooth_translate` | `false` | Enable smootherstep/trapezoid shaping |

### Yaw source selection

| `yaw_source`   | Heading from | Position (DVL dist) | Recommended for          |
|----------------|--------------|---------------------|--------------------------|
| `mavlink_ahrs` | ArduSub AHRS | none                | bench / Gazebo sim       |
| `bno085`       | BNO085 IMU   | none                | pool without DVL         |
| `dvl`          | Nucleus AHRS | Nucleus DVL         | pool with DVL (heading + position in one) |
| `bno085_dvl`   | BNO085 IMU   | Nucleus DVL         | pool when BNO heading preferred + DVL position |

> DVL sources connect automatically at startup when `dvl_auto_connect:=true`. The `dvl_connect` verb still works as a manual override.
> **Heading lock stays ACTIVE during `move_forward_dist` / `move_lateral_dist`** — the lock owns Ch4 (yaw rate) while DVL drives Ch5/Ch6. This keeps the AUV on-heading during distance moves.

> Older context files reference `/duburi/arm`, `/duburi/depth_cmd`, `/duburi/attitude`, `Attitude.msg`, `RCOverride.msg`, `VehicleState.msg`. **None of these exist.** Single action + single state topic + ROS params is the entire surface.

### Vision-driven verbs — TWO verbs (`vision_align` + `vision_move` on `/duburi/move`)

> Full DSL + verb reference: [`command-reference.md`](.claude/context/command-reference.md) · [`client-and-dsl-api.md`](.claude/context/client-and-dsl-api.md).
> **Reading the result (where/how a verb finished) + mid-hold fire + live feedback:
> [`vision-results.md`](.claude/context/vision-results.md) — read this before writing a vision mission.**
> Vision architecture: [`vision-architecture.md`](.claude/context/vision-architecture.md).
> FSM state wrappers: [`fsm-guide.md`](.claude/context/fsm-guide.md) §4.

The 2026-06 rewrite replaced the 9-verb API with **exactly two** pixel-native verbs.
Engine: `motion_vision.align_loop` / `move_loop`. Source of truth for signatures:
[`vision_dsl.py`](src/duburi_planner/duburi_planner/vision_dsl.py).

| DSL method | Action verb | What it does |
|---|---|---|
| `vision.align(target, lat=, yaw=, depth=, fwd=, fwd_mode=, err=, duration=, gain=, lat_gain=, yaw_gain=, depth_step=, brake=, hold=, fire=, fire_t=, fire_gap=, lock_on=, settle=, fire_pass=, hold_heading=, surge_sign=, max_depth_m=, depth_ceiling=, camera=, fallback=)` | `vision_align` | Centre target on the named axes; each value is a **signed pixel offset** from centre (`0`=centre). At least one of lat/yaw/depth. `hold=`s turns it into an **active station-keep**: keeps correcting on-target for `hold` s (fights water inertia for a torpedo/dropper shot) before exiting; counts against `duration` (budget `duration ≥ approach + hold`). **`fwd=`** (% fill, `fwd_mode=`area/width/height) adds a **forward range-hold axis**: align ALSO drives forward to that standoff fill and HOLDS it, so **one verb** does forward-standoff + lat/depth centering + station-keep + mid-hold fire (the unified **torpedo standoff shot**). The forward term is **one-sided** (drives forward while too far, neutral at/past standoff — never reverses, no reverse-kick/ramming), and the fire is gated on reaching the standoff too; `fwd` unset = no forward axis (lat/yaw/depth-only, e.g. a coarse board centre). **Depth axis** has no `%` cap — its rate is **`depth_step=`** (m/update, 0.02 slow .. 0.10 coarse); depth steps the ArduSub setpoint at 5 Hz and **freezes inside the deadband** so there's no z-wobble. **`fire=`** (int or list, 1/2=torpedo 3/4=dropper) + **`fire_t=`** (s into the hold) fire the payload **mid-hold while still correcting** — gated on alignment (no off-target shot) **and on `is_new_frame`** (fires on the tick a new LIVE box lands — FPS-robust, never a tracker-coasted/predicted box or a frozen detector's re-read stale frame), non-blocking, needs `fire_t < hold`. **`fire=[a,b]` fires a LIST** spaced **`fire_gap=`** s apart (default `FIRE_GAP_S=1.0`) — the solenoid launcher misfires if two go together; single-channel unaffected. **`camera='downward'`** (bottom cam) **rotates the frame AND remaps the kwargs** so `fwd` is always fore/aft and `depth` always the real depth axis: `lat`→Ch6 strafe (same as forward), **`fwd`→Ch5 SURGE** fore/aft (signed px, image-Y, `0`=centre), **`depth`→DEPTH DESCENT** (a fill %, measured by `fwd_mode`; `depth_step` logic, one-sided-deeper). So the forward-cam `depth`/`fwd` meanings **swap** on downward: a bin drop reads `align('fire', camera='downward', lat=0, fwd=0, depth=30, fwd_mode='height', …)` (lat+fwd centre over the bin, depth=30% descends). Centring axes on downward are `lat`+`fwd`; `depth` is the optional approach (a `depth`-only call raises). Physically identical to the old `lat/depth/fwd` downward form — pure kwarg remap, `align_loop`+wire+**Ch5 sign** all unchanged (a previously-verified `surge_sign` stays valid; the DISARMED check just confirms you're driving surge with the new `fwd=` kwarg). **`surge_sign=`** (+1/-1) flips fore/aft for the mount (**verify DISARMED**); **`max_depth_m=`** (deep floor, must be <0 for the descent) + **`depth_ceiling=`** (shallow surface-guard — alignment can't surface the hull) bound the descent. Auto-switches the live detector + HUD to downward. **Full axis-flip table: [`downward-camera.md`](.claude/context/downward-camera.md)**; hardware: [`dual-camera-setup.md`](.claude/context/dual-camera-setup.md). **`fire_pass=True`** fires anyway at command end if the strict lock never fired, provided the target was seen live+recently (guaranteed partial-points shot). **`hold_heading=True`** widens the heading-lock deadband for the hold (yaw-released path) so the launcher heading holds steady (no terminal yaw jitter). See [`vision-results.md`](.claude/context/vision-results.md) §4. **`lock_on=True`** = continuity lock: steer to the box **nearest the last centre** (not the largest) so a 2nd hole / spurious box can't steal the aim on a close-in shot — see [`precision-alignment.md`](.claude/context/precision-alignment.md). |
| `vision.move(target, fwd=, mode=, maintain=, hold=, err=, duration=, gain=, lat_gain=, brake=, fallback=)` | `vision_move` | Drive forward until bbox fills `fwd`% (`mode`=area/width/height). `maintain`=±px lateral offset; never re-centres yaw/depth. |

- **`gain` is a hard max-speed cap** (% thrust), not a target speed — the AUV never exceeds it.
- **Per-axis caps** `lat_gain`/`yaw_gain` (align) and `lat_gain` (move's `maintain` strafe) override `gain` on one axis; **unset = inherit `gain`, NOT disable** (to drop an axis, omit `lat`/`yaw`/`depth`). **Depth has no `%` cap** — its rate is `depth_step` (m/update). Use a low `yaw_gain` for slow, stable micro-alignment of a 20 kg hull against a small/distant target (e.g. `align('hole', yaw=0, lat=0, gain=25, yaw_gain=10)`). The yaw spin-up floor (`VISION_YAW_MIN_PCT`) only engages when the bbox is large (close: `VISION_YAW_FLOOR_FILL`) and is now **tapered** to 0 at the deadband edge (mirrors `heading_lock`/`motion_yaw`) so it eases in instead of relay-slamming — far-field yaw stays pure-proportional so it can't limit-cycle/wobble.
- **Inertial arrival brake** (`brake=`, **on by default**): on arrival the hull would otherwise coast on water inertia off the planned position, throwing off the next mission step. The verb reverse-kicks the translational axes to bleed that momentum — `align` brakes lateral; `move` brakes forward+`maintain` on a **fill-stop** arrival. **Yaw/depth never brake** (Ch4 is a rate ArduSub bleeds; depth is ArduSub hold). The kick scales to a trailing EMA of the exit velocity and is **self-gating**: a gently-converged lock that ramps down into the band usually exits with ~0 momentum and is **not** kicked. A *fast snap-in* (high lateral drive then an abrupt centre) can still cross the gate — so **on the torpedo fire-from-lock path pass `brake=False`** (no benefit when firing, and it removes any 0.2 s pre-shot nudge). **PASS-THROUGH (`move(fwd=None)`) and abort/loss exits never brake.** `brake=False` to coast; `brake_gain` scales the kick. `VISION_BRAKE_GAIN`/`VISION_BRAKE_MIN_PCT` are pool-tunable. (Default-on shifts the stopping point of pre-existing vision missions that were tuned assuming coast — re-verify standoffs.)
- **Never-fail contract:** neither verb raises; the server always returns `success=True` with an outcome code in `Move.Result.final_value` (`ALIGNED`=0, `LOST`=1, `TIMEOUT`=2, `NO_CAMERA`=3, `ABORTED`=4). The DSL returns a `VisionResult` (truthy only on `ALIGNED`); a server/setup error surfaces as non-fatal `FAILED`.
- **Rich result — branch on WHERE/HOW it finished (the hybrid vision+control paradigm).** `VisionResult` carries, on success AND failure: `x_px`/`y_px` (**signed** px of the target from frame **centre** at the last seen frame; `+x`=ended right, `+y`=ended below; `NaN`=never seen), `saw_target` (bool), `last_err_px` (residual from goal), `fill` (move bbox fill), `elapsed_s`, `status`. **Recovery sign matches `align` itself** — `x_px>0` (target right) → `move_right`; `x_px<0` → `move_left`. **Always check `saw_target` before reading `x_px`** (`NaN<threshold` is silently False → a never-seen target slips a guard). Full contract + worked recovery patterns + pitfalls: [`vision-results.md`](.claude/context/vision-results.md). `bool(res)` is unchanged (back-compat).
- **Live feedback:** during a verb, `Move.Feedback.err_x_px`/`err_y_px` stream the live signed target-from-centre px at ~2.5 Hz (`ros2 topic echo /duburi/move/_action/feedback`; `NaN` when no vision verb / no fresh frame) so you can watch convergence in real time. [`vision-results.md`](.claude/context/vision-results.md) §5.
- **Precision terminal alignment (close-in robustness, all opt-in/off by default):** `align(lock_on=True)` (continuity lock — kills last-moment misclassification), three deck ROS params — `vision.range_gain_floor` (softens lat/depth gain as the bbox fills → stops the 20 kg hull overshooting up close; `1.0`=off, `~0.3`=gentle), `vision.ctrl_conf` (control-side conf floor, distinct from the detector's `conf`), `vision.ki_lat` (lateral-only integral, nulls a steady current during the hold; enable **after** damping) — and the **per-call** `align(settle=<px>)` **settle gate** (only declare aligned once the hull is in-band **and** barely moving, so `align` ends *settled* on target like `vision.move`'s held lateral instead of exiting mid-pass and coasting off; keyed on error velocity so a steady current doesn't block it — that's `ki_lat`'s job). `settle` is per-call (not a deck param) and **for coarse exit-and-move-on aligns only — never the terminal fire-lock**, since the mid-hold `fire` rides the same stable-frame counter and a too-tight `settle` can suppress the shot. At the hole, **drop the `yaw` axis** and let `heading_lock` hold Ch4 (no vision-yaw wobble). **Three per-call terminal knobs (2026-07-01, D12) for the fire-lock:** `align(hold_heading=True)` widens the heading-lock deadband for the hold (1°→3°) so the launcher heading holds steady instead of micro-correcting sub-deg noise (the terminal yaw jitter — use it on the yaw-dropped hole-lock); `align(depth_step=<m>)` sets the per-update depth-setpoint resolution (0.02 slow .. 0.10 coarse) — depth steps at 5 Hz and **freezes inside the deadband** so ArduSub settles (no z-wobble); `align(fire_pass=True)` fires the payload at command end even if never fully aligned, provided the target was seen live+recently (a guaranteed partial-points shot). The **fire itself is gated on `is_new_frame`** (fires on the tick a new live box lands — FPS-robust yet never on a frozen/coasted box). Full guide + pool runbook + "what NOT to do": [`precision-alignment.md`](.claude/context/precision-alignment.md).
- **Ch4 / yaw arbitration (2026-06-29):** a vision verb writes Ch4 **only when `yaw` is a requested align axis**. `align` without a `yaw` axis (and **all** `move`) take the `release_yaw` path (`send_rc_translation`, lateral-only) and never touch Ch4 — so the verb never commands yaw the operator didn't ask for, and never fights a live `heading_lock`. **The align-yaw jitter** during a lat/depth align was `heading_lock`'s hard min-PWM floor relay limit-cycling against the lateral-strafe yaw moment; fixed by **tapering** the lock floor (mirrors the `motion_yaw` `ab2014f` fix). **Don't "fix" it by releasing the lock** — that hands yaw to ArduSub's untrusted hull compass; keep the BNO lock. See [`known-issues.md`](.claude/context/known-issues.md) D7/D8.
- **`err=<px>` deadband is honest, not literal-zero:** a **small positive** `err` is the tight knob (`err=8`); **`err=0` means "use default / `vision.err_px` param"** (rosidl `0==unset` live-tuning, *not* zero tolerance — explicit 0 and omitted are indistinguishable on the wire). Effective deadband is floored at `MIN_ALIGN_ERR_PX` (≈5px) so an over-tight `err` can't perpetually TIMEOUT; it's printed at align start and stated in the outcome (`aligned (N/Mpx)`). The detector's always-on `[ offset … ] '<class>' bearing (live)` line is **raw offset telemetry**, NOT the verb's verdict — don't conflate them. [`known-issues.md`](.claude/context/known-issues.md) D9.
- **`fallback`** = mission-authored search `fn(duburi)` / `fn(duburi, should_stop)`; runs on target loss, then the verb re-enters — all inside `duration`.
- Control path always reads `/detections` (tracker `/tracks` feeds the HUD only; no `--tracking` flag).

Gains are live-tunable (apply on the NEXT goal): `ros2 param set /duburi_manager vision.kp_yaw 80.0`.
Key vision ROS params (all on `/duburi_manager`): `vision.kp_lat`/`kp_yaw` (60.0), `vision.kp_depth` (0.05), `vision.kp_forward` (200.0), `vision.lost_grace_s` (1.0), `vision.frame_fill_default` (95.0), `vision.align_stable_frames` (3.0 — **distinct in-band detections**, not loop ticks, so one lucky frame at low FPS can't declare aligned or arm the fire); precision knobs (off by default) `vision.range_gain_floor` (1.0), `vision.ki_lat` (0.0), `vision.ctrl_conf` (0.0); **downward/depth tunables** (moved off per-`align` kwargs) `vision.surge_sign` (**−1**, PERMANENT downward Ch5 fore/aft polarity — missions omit `surge_sign`), `vision.max_depth_m` (**0.0**=off; a `<0` value is the deep floor **and** enables the downward fill→depth descent), `vision.depth_ceiling` (**0.0**=off→engine `_MIN_DEPTH_M` surface guard) — a mission sets the last two **once at start** via `duburi.set_vision_param('max_depth_m', −1.6)` instead of on every call, and `0.0` defaults keep the FORWARD torpedo align unchanged — plus the **per-call** `align(settle=<px>)` settle gate (coarse aligns only; see [`precision-alignment.md`](.claude/context/precision-alignment.md)). Deck defaults live in [`vision_tunables.py`](src/duburi_manager/duburi_manager/vision_tunables.py).

**Tracking + gap-bridging coast (Roboflow `trackers`).** `tracker_node` runs OC-SORT (default) / ByteTrack via the Roboflow `trackers` lib behind the `Tracker` ABC, publishing stable ids + coasted (Kalman-predicted) boxes on `/tracks`. **`vision.coast_s` (default `0`=OFF)** lets the control loop steer on the coasted box of the **locked target id** for up to `coast_s` after a real detection drops — so a brief YOLO flicker doesn't lose a torpedo-hole lock or drift the hull off a slalom pipe. **Opt-in, pool-gated** (it reverses an earlier fix; see [`known-issues.md`](.claude/context/known-issues.md) D10). Anti-bug invariants: a live `/detections` box ALWAYS overrides a coast; coast authority decays by **true detection-age** (not message age); a coasted box is conf-exempt **only for the locked id**. **Three distinct conf gates** (don't conflate): detector `conf` (what YOLO emits) → tracker `track_activation_threshold`/`high_conf_det_threshold` (spawn-id vs two-stage association) → control `vision.ctrl_conf` (what the loop steers on; the coast is exempt for the locked id). **Coast timeout ladder:** `_freshness` (0.4s) < `vision.coast_s` (~0.8) < `vision.lost_grace_s` (1.0) < tracker `max_predict`/buffer wall-time (the 4th rung — keep `max_predict` ≥ `coast_s` in frames or the coast truncates early).
**Vision queries** (client-side cache reads, distinct from the two action verbs; each pumps the node before answering — the default camera is subscribed eagerly so the first call never false-negates): `duburi.detected('gate', stale_after=1.0)` — "seen within the last `stale_after` s?" (True/False, **case-insensitive**). detected()/wait_for() use a **per-class last-seen recency window** (`stale_after`, default 1.0 s), NOT just the single latest raw frame — so a class that flickers out of individual `/detections` frames at low FPS still counts as present until the window lapses (the reacquire-side analogue of the control loop's `lost_grace_s`; the HUD looks continuous because it overlays Kalman-smoothed `/tracks`, while these queries read raw `/detections`). `duburi.wait_for('gate', timeout=8)` — block until seen/timeout (loop-free acquire). `duburi.where('gate')` → `'left'`|`'center'`|`'right'`|`'unknown'` (+ `where_offset` for signed `[-1,+1]`) — **where() reads the current frame** (bearing must be live, never a remembered spot). An `if detected()` runs once — a moving search needs a `while`. All three work inside a vision `fallback`.
`duburi.models(gate='gate_flare_medium_100ep')` — model registry; `duburi.models.gate.gate` returns `ClassRef` (auto-switches model+class when passed as `target`). **Model identity is the STEM** (`.pt` basename): a `ClassRef`/`set_model('<stem>')`/`use('<stem>')` works on a **single-model launch** (`vision.launch.py model:=<stem>` — no `models:=` needed; switching *to the loaded model* is a no-op, any *other* name is a clear "relaunch to switch" reject) AND on a **registry launch** (`models:=` accepts the key OR the stem). A registry is now **resilient** — a model whose `.pt` is missing is SKIPPED with a loud ERROR (pipeline stays up), not a fatal crash; use `models:=` only for genuine mid-mission multi-model switching (prefer bare stems so key==stem). See [`known-issues.md`](.claude/context/known-issues.md) D14.

**Detection FPS (Jetson Orin Nano):** the detector prefers a TensorRT `<stem>.engine` over the `<stem>.pt` automatically (`yolo._resolve_model_path`); raw PyTorch @640 is ~3-4 Hz (inference-bound), TensorRT FP16 is ~20-30 Hz (nano/small) / ~10-15 Hz (medium). Build engines **on the Jetson** (device + JetPack-version locked): `ros2 run duburi_vision export_engine --all` — confirm the `[YOLO ] backend=TensorRT engine` log. Also run `sudo nvpmodel -m 0 && sudo jetson_clocks` (MAXN; ~2× alone — `bringup_check` warns if not set). On a dev box without an engine it falls back to `.pt` transparently. **A `.engine` is transparent to the model-identity fix** (D14/D15): identity is the extension-less **stem**, so `model:=`/`models:=`/ClassRef/`set_model('<stem>')` all match; and live `conf`/`classes`/`max_det` tuning **does** apply to an engine (per-`predict()`, not baked). **Only `imgsz`/`half` are export-baked** — `imgsz:=` re-scales the `.pt` fallback only; re-export the engine (`export_engine --all --imgsz <N>`) to change it. **Keep each `<stem>.yaml` sidecar beside the exported `<stem>.engine`** — class labels come from it (a missing sidecar on an engine with no embedded `names` → empty allowlist → silent `[]` every frame). The debug overlay is skipped when no viewer is subscribed (`viewer:=false`). **Control/FPS coupling:** the 20 Hz vision loop **freshness-decays** the translational command (lat/fwd, not yaw/depth) by `sample.age_s` — full authority on a fresh frame, decaying to neutral when blind — so low/variable FPS can't make it blind-drive on a stale bbox. Raising FPS (TensorRT) is the primary fix; this is the per-frame guard.

> **Jetson Python deps (JetPack 6.2) — pin or the vision launch dies.** This stack
> needs **`numpy<2`** (`1.26.4`): ROS Humble `cv_bridge` + system `cv2` are NumPy-1.x
> ABI (numpy 2 → `_ARRAY_API not found`, every node crashes). Do **not** install pip
> `opencv-python*` — they shadow the GUI-capable system OpenCV (headless → `cv2.namedWindow`
> "rebuild with GTK" kills `vision_display`). For OC-SORT, install **`trackers==2.4.0
> --no-deps`** — the `2.5.0` PyPI wheel is a broken 9.7 kB dud with no module (its
> `numpy>=2` pin is a red herring; 2.4.0 runs fine on numpy 1.26.4). Full symptoms +
> one-shot recovery: [`known-issues.md`](.claude/context/known-issues.md) §E1–E3.

**Logging (per-logger levels, not a quiet flag):** the launch files pin each node's *own* logger to `info` while leaving the **process default at `warn`** — so framework/`rcl`/`rmw` "gibberish" is silenced but every Mongla log shows. The manager keeps all its telemetry (`[STATE]`/`[ARDUB]`/`[RC ]`/`[ACT]`) at info always (no `mission_quiet` — plain `ros2 run duburi_manager start` shows it too). The **always-on operator alignment line** is owned by the **detector node** (`detector_node._log_alignment`): `[ align lat=<px> depth=<px>px ] (cx,cy) align ['class'] center -> (0,0)` — emitted continuously (throttled ~0.5 s) for the **currently-loaded class** whenever it's detected, **regardless of whether a vision verb is running**. `lat`=bbox-centre x offset from frame centre, `depth`=y offset. The per-verb `align_loop`/`move_loop` copies are at debug (detector owns the live line); `move` still prints its control-specific `[ move fill=…% lat=…px ]` feedback at info. Full framework/debug output: `--log-level debug` on the relevant node.

```bash
ros2 run duburi_manager bringup_check          # network + serial + Jetson power preflight
ros2 run duburi_vision vision_check            # topic-only health probe
ros2 run duburi_vision vision_thrust_check     # detection → RC echo (disarmed safe)
ros2 run duburi_vision export_engine --all     # build TensorRT engines (ON THE JETSON)
ros2 launch duburi_vision mission_web.launch.py  # ★ mission console: 2-cam streams + detections + live control, auto-opens browser
```

> **Mission console (`mission_web`)** — the pool-day browser surface. One command starts camera(s) +
> detector(s) + `web_video_server` (MJPEG video pipe, `:8080`) + the `mission_web` node (console + SSE data,
> `:8090`) and auto-opens `http://localhost:8090`. **`cameras:=both`** (default → vision_dual) shows both
> `image_debug` streams side-by-side; **`cameras:=forward|downward`** launches the one-camera `vision.launch.py`
> and the console shows a single panel (a box with only ONE camera never crashes on an absent second device).
> Boxes are burned in server-side (frame-synced, zero browser overlay); panels show a live detection table
> (class · conf · dx/dy px · fill% · vis_range), per-class counts, `/duburi/state`, the latched-`active_camera`
> **mission-camera indicator**, and live control (active-camera switch, model dropdown, conf slider, class chips,
> pause/resume). Control writes the **same surface the DSL writes** — `SetParameters` on `/duburi_detector_<cam>`
> + the latched `active_camera` publish + pause-others/resume-target — so UI and a running DSL mission stay in
> sync. The console **polls** the detector params it reflects (`active_model`/`conf`/`models`/`paused`/`classes`
> via `get_parameters`, 1 Hz) rather than the `classes_filter` topic — that topic is VOLATILE, so a
> durability-mismatched/late-join sub gets nothing; polling is join-order-proof. "STREAM NOT AVAILABLE" (absent
> detector **or** web_video_server down) vs "PAUSED" (present, not inferring) vs live is derived from
> `get_node_names` + the polled `paused`. **Robust to a missing `web_video_server`** (apt pkg, not a repo dep):
> the launch degrades to console-only with an `apt install` hint instead of failing. Full command matrix
> (single/double × single/multi-model × dataset-video): JETSON_SETUP §5 Option D.
> Node: `duburi_vision/web/mission_web_node.py` (+ pure helpers `web/dashboard_state.py`, SPA `web/static/`).

**Operator debugging / practice tooling (all OFF the mission path — see [`foxglove-and-bags.md`](.claude/context/foxglove-and-bags.md)):**

```bash
source scripts/pool_session.sh gate_am         # SOURCE in every terminal — pins DUBURI_RUN_DIR + ROS_LOG_DIR (one folder/run)
ros2 launch duburi_manager bringup.launch.py mode:=pool yaw_source:=bno085 vision:=true foxglove:=true  # vehicle + vision + Foxglove (ws://<ip>:8765)
scripts/pool_record.sh record gate_am          # rosbag (MCAP) a run → the pinned folder (replay offline to tune)
scripts/pool_record.sh replay <bag-dir>        # play a recorded run back (Foxglove/vision_display against it)
scripts/pool_record.sh list                    # list recorded runs + recent scorecards
```
> Full per-run workflow (which terminal runs what, live Foxglove + offline replay-to-tune):
> [`foxglove-and-bags.md`](.claude/context/foxglove-and-bags.md) §0. `bringup.launch.py` is the
> `ros2 run duburi_manager start` equivalent that also wires vision + Foxglove; the bare
> `start` has no `foxglove` arg (launch the bridge yourself if you use it).

Scorecards auto-write to `DUBURI_RUN_DIR` (default `~/duburi_runs`) as `<mission>_<ts>.json`
(mission + ISO timestamp + git SHA + per-verb phases) on every mission exit.

The **ground-station viewer is Lichtblick** (offline-safe Foxglove Studio fork — no login wall
at the venue), connecting to `ws://<jetson-ip>:8765`; the dev box also runs `foxglove_bridge`
for local-sim dry practice. Dev-box install + connect steps: `foxglove-and-bags.md` §4.

---

## 9. ArduSub modes reference

| Mode      | Use case in this stack                                              |
|-----------|---------------------------------------------------------------------|
| `MANUAL`  | Raw RC override, arm/disarm                                         |
| `STABILIZE` | Attitude-stabilized; `SET_ATTITUDE_TARGET` is interpreted as a *rate* (so we avoid it for absolute yaw) |
| `ALT_HOLD` | The only mode we use during a mission. Depth-hold + absolute yaw setpoints both work. |
| `POSHOLD` | XY position hold via DVL/EKF3. Requires Nortek BlueOS extension + ArduSub params `EK3_SRC1_POSXY=3`, `EK3_SRC1_VELXY=5`, `VISO_TYPE=1`. See [`dvl-reference.md`](.claude/context/dvl-reference.md) §POSHOLD. |
| `GUIDED`  | Waypoint following from GCS — not used today                        |
| `SURFACE` | Emergency surface — manual fallback only                            |

Typical mission sequence: `MANUAL` → `arm` → first `set_depth` engages `ALT_HOLD` → mission verbs (`yaw_left`, `move_forward`, ...) → `disarm`.

---

## 10. Reference codebases (study these for patterns, not for package layout)

| Location                                  | Era              | Lessons                                                   |
|-------------------------------------------|------------------|-----------------------------------------------------------|
| `Reference CodeBase/2023/`                | RoboSub 2nd 2023 | Core pymavlink patterns, heading PID, depth PID           |
| `Reference CodeBase/Robosub-2025-Duburi/` | RoboSub 8th 2025 | YASMIN FSM, DVL integration                                |
| `Reference CodeBase/ardusub-interface/`   | BumblebeeAS      | ROS2 + behavior trees, setpoint-based control             |
| `Reference CodeBase/BareMinimum/`         | Internal         | Minimal working pymavlink                                  |
| `Reference CodeBase/` (team archives)     | BRACU Duburi     | Joy → ROS2 → pymavlink bridge; standalone mission scripts; time-based movements |

> **The 2023 codebase is the ground truth for proven MAVLink patterns.** Names like `duburi_driver` etc that appear in `proven-patterns.md` come from those eras — *not* from this workspace.

---

## 11. Development workflow

### Step 1: Sim first

```bash
# Bring up sim (in docker terminals — see §3)
cd ~/Ros_workspaces/duburi_ws
./build_dubomini.sh
source install/setup.bash
ros2 run duburi_manager start --ros-args -p mode:=sim
```

> **`build_dubomini.sh` mirrors device-local models + missions into the tree
> first, then builds.** The competition YOLO weights and the personal test
> missions (`rakib_*`) are kept **out of git** (weights by extension; missions via
> per-clone `.git/info/exclude`) and live in source-of-truth folders **outside**
> the repo — `~/models` → `src/duburi_vision/models/` and `~/missions` →
> `src/duburi_planner/duburi_planner/missions/`. So after a fresh `git clone`,
> one `./build_dubomini.sh` restores them with nothing to copy by hand. The sync
> is additive (never deletes) and soft-skips a missing source folder (dev box).
> Override the sources with `DUBOMINI_MODELS_SRC` / `DUBOMINI_MISSIONS_SRC`.
> (Formerly `build_duburi.sh` — renamed, same two-step interfaces-first build.)

### Step 2: Verify connectivity

```bash
ros2 topic echo /duburi/state            # Should show armed=false, mode=MANUAL, yaw, depth, battery
ros2 node info /duburi_manager           # ActionServer should be listed
```

### Step 3: Test control via CLI

```bash
ros2 run duburi_planner duburi arm
ros2 run duburi_planner duburi set_depth --target -0.5
ros2 run duburi_planner duburi yaw_right --target 90
ros2 run duburi_planner duburi turn --target 90      # absolute heading, direction auto
ros2 run duburi_planner duburi move_forward --duration 5 --gain 80
ros2 run duburi_planner duburi arc --duration 4 --gain 50 --yaw_rate_pct 30
ros2 run duburi_planner duburi lock_heading --target 0 --timeout 120
ros2 run duburi_planner duburi unlock_heading
ros2 run duburi_planner duburi disarm
```

### Step 3b: DVL + vision verbs (pool only)

```bash
# DVL auto-connects (dvl_auto_connect:=true). Manual: duburi dvl_connect
ros2 run duburi_planner duburi move_forward_dist --distance_m 2.0 --gain 60
# Centre the gate (yaw + lateral), then drive forward until it fills 80% of frame:
ros2 run duburi_planner duburi vision_align --camera forward --target_class gate \
    --axes yaw,lat --err_px 40 --gain 30 --duration 20
ros2 run duburi_planner duburi vision_move --camera forward --target_class gate \
    --fwd_fill 80 --mode area --gain 35 --duration 20
```

### Step 4: Run a mission

```bash
ros2 run duburi_planner mission --list
# detected()-paradigm task chunks (standalone or chained):
ros2 run duburi_planner mission task_gate          # gate chunk
ros2 run duburi_planner mission task_full_2026     # full 5-task detected-paradigm run
# YASMIN FSM (per-task or full sequence):
ros2 run duburi_planner mission fsm_slalom         # standalone slalom FSM
ros2 run duburi_planner mission fsm_bin            # standalone bin-drop FSM
ros2 run duburi_planner mission fsm_torpedo        # standalone torpedo FSM
ros2 run duburi_planner mission fsm_return         # standalone return-gate FSM
ros2 run duburi_planner mission fsm_full_2026      # full 5-task YASMIN FSM (recommended)
# Prior FSM missions (kept for backward compat):
ros2 run duburi_planner mission gate_flare_fsm     # FSM gate + flare
ros2 run duburi_planner mission prequal_fsm        # FSM gate-only prequal
ros2 run duburi_planner mission gate_then_bin_fsm  # FSM gate → bin drop
ros2 run duburi_planner mission gate_flare_autonomous  # detected()-paradigm fallback
```

### Step 5: Vision sanity

```bash
ros2 launch duburi_manager bringup.launch.py vision:=true
ros2 run duburi_vision vision_check --camera forward --require-class gate
ros2 run duburi_vision vision_thrust_check --camera forward --duration 4
ros2 param set /duburi_detector_forward classes "gate,flare"   # live class switch
```

---

## 12. Environment & paths

```bash
# Docker env
ROS_DOMAIN_ID=42
GZ_SIM_SYSTEM_PLUGIN_PATH=~/stuff/ardupilot_gazebo/build
GZ_IP=127.0.0.1

# Workspaces
~/Ros_workspaces/duburi_ws        # OUR codebase (this workspace)
~/Ros_workspaces/duburi_ws/sim    # the simulator — nested colcon workspace, same repo
~/Ros_workspaces/colcon_ws        # legacy bluerov2_gz sim (superseded by sim/; DO NOT MODIFY)

# Tools (out of workspace; override with ARDUPILOT_ROOT / ARDUPILOT_GAZEBO_ROOT)
~/stuff/ardupilot/            # ArduSub SITL
~/stuff/ardupilot_gazebo/     # Gazebo-ArduPilot bridge plugin
```

---

## 13. Safety rules (non-negotiable)

1. **Always have a disarm path** — Ctrl-C on the manager triggers `Duburi.stop()` + `disarm()`.
2. **Cooperative abort** — `cancel_callback` sets `command_active=False` and calls `duburi.request_abort()` which signals `_abort_event`; every motion loop checks this flag once per tick and exits early. Safety verbs (`disarm`, `stop`, `surface`) bypass the `command_active` gate and signal abort simultaneously so they always execute.
3. **Heartbeat must keep ticking** — owned by the manager's ROS2 timer; nothing in the action callback may block long enough to break it.
4. **Neutral on startup** — RC defaults to 1500 (not 65535) until a movement is active.
5. **Pool test checklist** — propellers clear, tether on, topside can ping the Jetson.
6. **Autonomous mission** — timer-delayed start (run code, wait N seconds, remove tether).
7. **DVL offset** — `dvl_depth_match = 0.78` (calibrated value from 2025 competition; will move into `duburi_sensors` when the DVL driver lands).

---

## 14. Context files (in `.claude/context/`)

**RoboSub 2026 (read first — competition status & TDR reconciliation):**

| File                            | Contents                                                            |
|---------------------------------|---------------------------------------------------------------------|
| `development-board.md`          | **★ START HERE — central dashboard: phase status, open work, bug/fix log, phase-2 tickets, doc map.** |
| `robosub-2026-audit.md`         | Full-stack audit + TDR⇄code gap matrix (G1–G12) + P0.1 Decision Record + P0/P1/P2 plan |
| `robosub-2026-roadmap.md`       | Phase-1 schedule (Gate/Return/search ≈800 pt) + **Phase-2 committed build tickets** (FSM/Dubomini/IVC/tasks) |
| `scouting/`                     | Competitor/reference-team scouting notes (e.g. `bumblebee-2025.md`) |

**API & verbs (start here):**

| File                            | Contents                                                            |
|---------------------------------|---------------------------------------------------------------------|
| `launch-combinations.md`        | **★ "so we never fail" master reference** — EVERY launch/run command with all args + combinations: control node (modes/yaw_source), `vision`/`vision_dual` (single/dual, single/multi-model, by-path device, dataset video), `mission_web` console, CLI verbs, mission runner, TensorRT engines. Scoped to the tested competition path |
| `command-reference.md`          | **Every verb** on `/duburi/move`: CLI, Python facade, DSL, MAVLink, lock modes, distance metrics |
| `client-and-dsl-api.md`         | `DuburiClient`, `DuburiMission` DSL, `vision.*` verbs, `duburi.detected()` |
| `vision-results.md`             | **★ Read before a vision mission** — `VisionResult` finish-state (`x_px`/`y_px`/`saw_target`/`fill`/`elapsed`), hybrid vision+control recovery patterns, mid-hold fire (`fire`/`fire_t`), live `err_x_px` feedback, do's & don'ts |
| `precision-alignment.md`        | **★ Close-in robustness** — kill last-moment misclassification (`lock_on` continuity lock + `ctrl_conf`) and hold a 20 kg hull steady (`range_gain_floor` + `ki_lat`); phased COARSE→APPROACH→TERMINAL pattern, yaw→`heading_lock` at the hole, pool runbook, what NOT to do |
| `detected-paradigm.md`          | **`duburi.detected()` deep reference** — mechanics, rules, orbit trap, errors, testing, canonical templates |
| `mission-cookbook.md`           | Mission DSL cookbook — working principles + 10 ready-to-steal samples |
| `testing-guide.md`              | Every test: unit, bringup, mission smoke, in-water checklist        |
| `ros2-conventions.md`           | ROS2 coding conventions + complete 28-verb command reference table   |

**ArduSub & MAVLink:**

| File                            | Contents                                                            |
|---------------------------------|---------------------------------------------------------------------|
| `ardusub-canon.md`              | **First-principles** ArduSub: modes, depth cascade, yaw rate loop, failsafes |
| `ardusub-reference.md`          | ArduSub-specific parameters, modes, quirks (quick-list)             |
| `mavlink-reference.md`          | MAVLink catalogue + per-call audit + `[MAV <fn> cmd=verb]` DEBUG trace |
| `heading-lock.md`               | Heading-lock state diagram, motion interaction, failure modes       |
| `axis-isolation.md`             | First-principles theory: sharp vs curved turns, settle/pause        |

**Vehicle, hardware, sim:**

| File                            | Contents                                                            |
|---------------------------------|---------------------------------------------------------------------|
| `../docs/JETSON_ONESHOT_15W.md` | **★ Backup-Jetson one-shot prep (15W Orin Nano 8GB, no WiFi)** — ordered, idempotent, two-agent runbook (network/remote/cameras/deps/build/**TensorRT engines**/power+clocks/verify). The engine is mandatory at 15W (`.pt`=3-4 Hz); `nvpmodel -m 0` is this board's max (NOT MAXN); measure sustained FPS on the desk to pick imgsz (640 default, 512 fallback) |
| `vehicle-spec.md`               | **Canonical** Duburi 4.2 spec + TDR-vs-implementation delta         |
| `hardware-setup.md`             | Pool setup, BlueOS, network topology                                |
| `dual-camera-setup.md`          | **★ Jetson agent** — 2× identical Blue Robotics USB cams: by-path port-stable identity, USB-2 480 Mbps/MJPEG bandwidth, udev aliases, `vision_dual` `device_path` |
| `downward-camera.md`            | **★ the axis flip** — why/how `vision.align` kwargs remap on `camera='downward'` (`lat`=Ch6, `fwd`=Ch5 surge, `depth`=fill→descent); canonical calls, surge-sign DISARMED check, depth bounds, migration note |
| `sim-setup.md`                  | **Legacy** sibling-`colcon_ws` bring-up — superseded by `sim/`       |
| [`../sim/README.md`](sim/README.md) | **★ Simulator** — operator cold start, terminal by terminal      |
| [`../sim/.context/INDEX.md`](sim/.context/INDEX.md) | Simulator doc index (CONTRACT / OPERATOR / AUDIT / WORLD_EDITING / DATASETS) |
| `sensors-pipeline.md`           | `duburi_sensors` design rules + BNO085 calibration model            |
| `dvl-reference.md`              | Nortek Nucleus1000 protocol, packet catalog, POSHOLD ArduSub setup  |
| `dvl-integration.md`            | DVL + BNO085 integration notes + composite source design            |
| `pool-day.md`                   | Pool-day checklist and session workflow                             |
| `foxglove-and-bags.md`          | **★ Operator tooling (off mission path)** — Foxglove telemetry (`foxglove:=true` + FPS/offline gates), `pool_record.sh` rosbag record/replay, mission scorecards → `~/duburi_runs`. Plus the "what we did NOT integrate" scouting decision |
| `remote-access.md`              | **★ Ground-station remote access (off mission path)** — smooth, drop-proof workflow that replaces laggy xrdp: mosh+tmux (terminal, mission survives GUI death), Foxglove/`web_video_server` (vision, no desktop), NoMachine (full desktop), polkit `.pkla` (kills the password popups), emergency recovery without a reboot. Install: `tools/setup_remote_access.sh` |
| `known-issues.md`               | Tracked code bugs from the 2026-04/05 audits (all FIXED). Current cross-cutting state → `robosub-2026-audit.md` |

**Method & design theory:**

| File                            | Contents                                                            |
|---------------------------------|---------------------------------------------------------------------|
| `pid-theory.md`                 | PID tuning notes — **REFERENCE**, not the live path                 |
| `proven-patterns.md`            | Patterns from 2023/2025 codebases — names are **historical**        |
| `yaw-stability-and-fusion.md`   | Yaw drift research; cross-links to `sensors-pipeline.md`            |
| `mission-design.md`             | YASMIN FSM design reference (now built — see fsm-guide.md)          |
| `fsm-guide.md`                  | **★ FSM user guide** — YASMIN fundamentals, VehicleProfile, state library, pool-day workflow, adding new tasks |
| `fsm-vision-missions.md`        | **★ Vision-guided mission design** — first-principles detection model, 6 search patterns, DVL/timed table, full pick+drop worked example, gain tuning, checklist |
| `vision-architecture.md`        | `duburi_vision` file map, topic contract, GPU contract, viz layers  |
| `vision-roadmap.md`             | v1–v4f done (detection, tracking, Kalman, vision verbs, depth pipeline); v5 real hw cams queued |
| `depth-estimation.md`           | Depth Anything V2-Small ONNX node: params, topics, EMA smoothing, HUD integration, test |
| `video-testing.md`              | **Full guide**: video_file source, sim+video workflow, playback controls, mission replay |

**Archived / future work** (in `future/`):

| File                            | Contents                                                            |
|---------------------------------|---------------------------------------------------------------------|
| `future/goals.md`               | Original TDR task checklist (archival)                              |
| `future/future-registry-shrinkage.md` | Parked: COMMANDS registry refactor ideas                    |
| `future/future-bno-into-ekf.md` | Parked: BNO085 velocity integration into ArduSub EKF3              |

---

## 15. Claude automations (`.claude/agents`, `.claude/skills`, `.claude/hooks`)

Project-local Claude Code automations, versioned with the repo and shared with the team.

**Subagents** (`.claude/agents/*.md`) — dispatch via the Agent/Task tool:

| Agent | Use after / for |
|-------|-----------------|
| `mavlink-reviewer`       | editing `duburi_control/` — checks mode preconditions, RC directions, rate pins, heartbeat, disarm safety |
| `mission-reviewer`       | editing `missions/` — `detected()` guard, two-verb vision (`align`/`move`) fallbacks, duration budgets, disarm-in-finally |
| `doc-verifier`           | auditing external-API usage (pymavlink, ultralytics, supervision, cv2) vs current online docs |
| `context-doc-sync`       | flagging stale claims in `.claude/context/*.md` + CLAUDE.md vs `src/` |
| `robosub-task-architect` | designing a new RoboSub 2026 task (mission + detection + DSL verbs) |
| `vision-model-reviewer`  | reviewing YOLO11 train/detect configs, dataset balance, thresholds |

**Skills** (`.claude/skills/<name>/SKILL.md`) — invoke as `/<name>`:

| Skill | Invocation | Purpose |
|-------|-----------|---------|
| `pool-day`     | both      | Interactive in-water preflight (bringup_check, topic rates, armed=false gate) |
| `add-command`  | user-only | Scaffold a new `/duburi/move` verb (commands.py + Duburi method + test) |
| `new-mission`  | user-only | Scaffold a mission from the `detected()`-paradigm template |
| `train-model`  | both      | YOLO11 fine-tune workflow for a new detection task |
| `verify-docs`  | user-only | Run `doc-verifier` across a package, summarize API drift |

**Hooks** (`.claude/hooks/`, wired in `.claude/settings.json` via `$CLAUDE_PROJECT_DIR` so they work on any checkout):

| Hook | Event | Behavior |
|------|-------|----------|
| `block_install.py` | PreToolUse  | Blocks edits to the colcon-generated `install/` tree (exit 2) |
| `py_check.sh`      | PostToolUse | `py_compile` syntax check on edited `.py` (advisory) |
| `pkg_test.sh`      | PostToolUse | Runs the matching `test_<name>.py` for an edited source file (targeted, fast, advisory) |

## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

Rules:
- For codebase questions, first run `graphify query "<question>"` when graphify-out/graph.json exists. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).

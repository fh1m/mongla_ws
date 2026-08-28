# Mongla — `duburi_ws/sim`

Gazebo Harmonic + ArduSub SITL + operator web lab for **Mongla / Duburi 4.5**.
The MAVLink and camera surface the autonomy stack in [`../src`](../src) expects,
without a pool.

**One repo, two colcon workspaces.** `duburi_ws/` is the autonomy stack; `sim/`
(here) is the simulator. They build and test independently — `sim/COLCON_IGNORE`
keeps the root `colcon build` at exactly six autonomy packages — and they are
refereed against each other by
[`../src/duburi_manager/test/test_sim_contract_drift.py`](../src/duburi_manager/test/test_sim_contract_drift.py),
which reads the launch files, model SDF and ArduSub params below and fails if
they stop agreeing with autonomy.

| Start here | |
|------------|---|
| **Operators** | this README → [`.context/QUICKSTART.md`](.context/QUICKSTART.md) |
| **Terminal-by-terminal cold start** | [`../README.md`](../README.md#-simulator--gazebo--ardusub-sitl) |
| **Agents** | [`.context/INDEX.md`](.context/INDEX.md) → [`.context/HANDOFF.md`](.context/HANDOFF.md) |
| **Autonomy integrators** | [`.context/CONTRACT.md`](.context/CONTRACT.md) · [`.context/INTEGRATION_DUBURI_WS.md`](.context/INTEGRATION_DUBURI_WS.md) |

> **Packaging (settled 2026-08-27).** This tree used to be an unversioned sibling
> called `duburi-sim_ws` and every doc here told you not to `git init`. That is
> over: the sim now lives **inside** the `duburi_ws` repo at `sim/`, with history.
> It is also published standalone as
> [`fh1m/duburi-sim_ws`](https://github.com/fh1m/duburi-sim_ws) for sim-only work.
> **`duburi_ws/sim/` is canonical**; the standalone repo is a mirror of it. Land
> changes here. See [`.context/FUTURE_MERGE.md`](.context/FUTURE_MERGE.md) for
> what was decided and why.

---

## Operator path — cold start → mission

### 0. Once per machine

```bash
# ROS Humble + Gazebo Harmonic already installed in the auv-ros2 container
cd ~/Ros_workspaces/duburi_ws
pip install -r sim/requirements.txt      # PyYAML, numpy, Pillow for the world gens

./build_dubomini.sh                      # autonomy FIRST -- sim includes its launch files
cd sim && ./build_sim.sh                 # then the simulator

# optional timeseries UI
sudo apt install ros-humble-plotjuggler-ros
```

> Use `./build_sim.sh`, not a bare `colcon build`. `sim/COLCON_IGNORE` stops the
> *root* build from picking up six Gazebo packages, but colcon checks that marker
> against the base path too, so `cd sim && colcon build` ignores itself and builds
> nothing. `build_sim.sh` passes `--base-paths src` to step past it. Deleting the
> marker to "simplify" this re-contaminates the root build.

Every terminal below starts from the same two sources, **autonomy first**:

```bash
source /opt/ros/humble/setup.bash
source ~/Ros_workspaces/duburi_ws/install/setup.bash
source ~/Ros_workspaces/duburi_ws/sim/install/setup.bash
export GZ_IP=127.0.0.1
```

### 1. Simulator (Terminal 1)

```bash
ros2 run duburi_sim_bringup duburi_sim stop    # always first
ros2 run duburi_sim_bringup duburi_sim sim     # GUI; or: sim --headless
```

Wait for **`JSON received`**. AUV sits at **x ≈ −11.8** (start zone).

### 2. Autonomy stack (Terminal 2)

```bash
export DUBURI_WS=~/Ros_workspaces/duburi_ws
ros2 run duburi_sim_bringup duburi_sim stack --no-vision   # first bring-up
#   heading-only, no DVL:  duburi_sim stack yaw_source:=mavlink_ahrs
# later, with YOLO weights present:  duburi_sim stack
```

> **`flight_controller:=pixhawk`** — `stack.launch.py` passes this through. It is
> **required on the `srot` branch** and **inert on `main`**, which declares no such
> argument. Inert, not an error: `IncludeLaunchDescription.execute()` raises only
> for *missing required* arguments; extra keys become launch configurations nobody
> reads, with no log line. So a renamed or branch-only launch argument fails
> silently in both directions — which is precisely why
> `test_sim_contract_drift.py` asserts on it instead of trusting the launch to
> complain.

### 3. Prove the loop (Terminal 3)

```bash
ros2 run duburi_sim_bridge contract_check   # 4 topics >=5 msgs, 640x480, + ground truth
ros2 run duburi_sim_bringup duburi_sim smoke  # arm -> set_depth -1 -> move_forward 8s
```

> **`mavlink_check` must run with the stack DOWN.** It binds UDP 14550 itself, so
> against a live manager it either fails or silently steals the autonomy link.
> ```bash
> ros2 run duburi_sim_bringup duburi_sim stop   # or just Ctrl-C Terminal 2
> ros2 run duburi_sim_bridge mavlink_check
> ```

### 4. Operator lab (optional)

```bash
ros2 run duburi_sim_bringup duburi_sim lab
# open http://localhost:28765   (port: cat /tmp/duburi-$USER/lab_port.txt)
```

> The lab binds **`127.0.0.1`** by default — it is unauthenticated and its API can
> arm thrusters. To reach it from a topside laptop, prefer an SSH port-forward;
> `DUBURI_LAB_HOST=0.0.0.0` is the explicit opt-in if you really want it on the
> network.

| Tab | Use |
|-----|-----|
| **Operate** | Cams, D-pad teleop (TCP 5763), arm, turbidity, record→zip |
| **World** | Start/restart/stop course, spawn/move props, custom model zip |
| **Datasets** | Clip list with wall duration + `fps_actual`, download zip |

### 5. Run a mission (autonomy verbs, against the sim)

With sim + stack up:

```bash
ros2 run duburi_planner mission --list
ros2 run duburi_planner mission sim_shakedown         # end-to-end loop check
ros2 run duburi_planner mission gate_flare_prequal    # or your mission id

# or drive manually
ros2 run duburi_planner duburi arm
ros2 run duburi_planner duburi set_depth --target -1.0
ros2 run duburi_planner duburi move_forward --duration 5 --gain 60
ros2 run duburi_planner duburi disarm
```

> **Verifying the vision path needs an explicit pass criterion — both failure
> modes are silent.** `duburi_sim stack` defaults to `model:=gate_rescue_repair`,
> whose `.pt` weight is **not in git** (`*.pt` is gitignored; `build_dubomini.sh`
> mirrors it from `~/models`) and whose `gate_rescue_repair.yaml` class sidecar
> may also be absent. A missing weight is loud; a **missing sidecar is not** — the
> class allowlist comes up empty and the detector publishes `[]` every frame
> forever. So do not conclude "vision works" from a clean launch. Require all
> three in the detector log:
>
> 1. the expected model stem (`[YOLO ] ... gate_rescue_repair`),
> 2. a **non-empty** class allowlist,
> 3. the always-on `[ align lat=… depth=… ]` line appearing with a gate in frame.
>
> The contract gate in step 3 deliberately runs `--no-vision` so it stays
> meaningful without weights.

**`sim_shakedown`** is the one-command proof the loop works: arm, hold depth,
drive out, drive back, surface, disarm. The two legs are symmetric, which *is*
the return-to-origin mechanism (no position feedback — the same file runs on the
real vehicle). Measure the residual against ground truth:

```bash
ros2 topic echo /duburi/sim/ground_truth --once     # before, and again after
```

Two measured runs, 5 s legs @ 55 % at −1.2 m: **0.266 m** and **0.200 m**
horizontal, of which along-track was only 0.037 m and 0.011 m. The structure is
the point — symmetric timed legs retrace along-track almost exactly, and
cross-track heading drift dominates.

> The pool is **1.6 m deep** (`spec/arena.yaml`, floor at z = −1.6 in all three
> worlds); a deeper target bottoms out. And `surface()` will not confirm in sim —
> `AHRS2.altitude` is offset from truth (0.33 m at the surface, ~0.16 m at depth)
> while ArduSub controls on EKF3, so the hull surfaces but the readback never
> reaches 0.00. Same offset makes `mission_reset`'s baro re-zero refuse. Numbers:
> [`.context/TROUBLESHOOTING.md`](.context/TROUBLESHOOTING.md).

Mission design and YOLO models live in **`../src`** — this workspace only supplies
physics, cameras and the lab.

### 6. Datasets for vision

```bash
# CLI
ros2 run duburi_sim_bridge record_cameras --duration 20 --fx --frames --labels \
  --label gate_approach

# or Operate → ● record → ■ stop (zip downloads)
```

**Verify a clip before training on it.** `record_cameras` buffers frames in RAM
and drops PNG/label writes on a full queue, desyncing indices without erroring —
so an existing directory proves nothing. Frames on disk must equal `meta.json`'s
`counts`, and `ffprobe` duration must match `duration_s`:

```bash
cd datasets/<run> && python3 -c "
import json,os; m=json.load(open('meta.json'))
for c,n in m['counts'].items():
    f=len(os.listdir(f'frames/{c}')); l=len(os.listdir(f'labels/{c}'))
    print(c, n, f, l, 'OK' if f==n==l else 'MISMATCH')"
ffprobe -v error -show_entries format=duration -of csv=p=0 front.mp4
```

### 7. Timeseries / 3D tools

```bash
ros2 run duburi_sim_bringup duburi_sim plotjuggler   # state + GT
# Foxglove: see duburi_ws/.claude/context/foxglove-and-bags.md
```

### 8. Shutdown

```bash
# Ctrl-C lab/stack terminals, then:
ros2 run duburi_sim_bringup duburi_sim stop
# stop now also kills lab_server / record_cameras / bridges / prop_manager
```

---

## Helper commands

| Command | What |
|---------|------|
| `duburi_sim stop` | Kill sim + stack + lab + bridges |
| `duburi_sim sim` | Gazebo + ArduSub + camera bridge |
| `duburi_sim sim --headless` | No GUI |
| `duburi_sim sim course:=sauvc26_final` | Other course |
| `duburi_sim stack --no-vision` | Manager only |
| `duburi_sim stack` | Manager + vision on sim front cam |
| `duburi_sim smoke` | arm → depth −1 → surge 8 s |
| `duburi_sim lab` | Operator UI |
| `duburi_sim plotjuggler` | PlotJuggler + sim layout |

Full flags: [`.context/COMMAND_REFERENCE.md`](.context/COMMAND_REFERENCE.md).

## Contract (do not drift)

| Surface | Value |
|---------|-------|
| Autonomy MAVLink | UDP **14550** |
| DVL (native gz sensor) | `/duburi/sim/dvl/{velocity,altitude}`, `yaw_source=sim_dvl` |
| Lab teleop RC | TCP **5763** |
| Cams | `/duburi/sim/{front,bottom}_camera/image_raw` **640×480** |
| GT | `/duburi/sim/ground_truth` |
| Lab HTTP | `DUBURI_LAB_PORT` default **28765**, bound to **127.0.0.1** |

## Packages

| Package | Role |
|---------|------|
| `duburi_sim_description` | Vehicle SDF / hydro |
| `duburi_sim_worlds` | Pool, props, courses |
| `duburi_sim_bringup` | Launches + `duburi_sim` CLI |
| `duburi_sim_bridge` | ros_gz, FX, recorder, checks |
| `duburi_sim_scenarios` | Runtime props |
| `duburi_sim_web` | FastAPI + React lab |

## Doc map

| Doc | Audience |
|-----|----------|
| [`.context/INDEX.md`](.context/INDEX.md) | Agent orientation |
| [`.context/HANDOFF.md`](.context/HANDOFF.md) | How the lab got built, why, next steps |
| [`.context/OPERATOR.md`](.context/OPERATOR.md) | Full operator guide |
| [`.context/WORLD_EDITING.md`](.context/WORLD_EDITING.md) | Props / custom assets |
| [`.context/DATASETS.md`](.context/DATASETS.md) | Recording / fps_actual |
| [`.context/PLOTJUGGLER.md`](.context/PLOTJUGGLER.md) | Timeseries |
| [`.context/AUDIT.md`](.context/AUDIT.md) | Known bugs |
| [`.context/TESTING.md`](.context/TESTING.md) | Verification gates |

## Credits

Vehicle: [bluerov2_gz](https://github.com/clydemcqueen/bluerov2_gz) (MIT).
Props: [sauvc26-world](https://github.com/auv-amarine/sauvc26-world) (MIT).
Meshes: [Blue Robotics](https://bluerobotics.com/).

---

## DVL

The vehicle carries Gazebo's **native** DVL (`gz-sim 8` ships one), so
`move_forward_dist` and friends close a real position loop in sim instead of
dead reckoning. `yaw_source=sim_dvl` is the default for `duburi_sim stack`:
heading still comes from MAVLink AHRS, position from the DVL.

```bash
ros2 topic echo /duburi/sim/dvl/velocity --once     # body-frame m/s
ros2 topic echo /duburi/sim/dvl/altitude --once     # bottom-track range
```

Four things about it are counter-intuitive enough that each one produced a
plausible-looking sensor that was quietly wrong — the sensor must sit on
`base_link`, the DVL frame is rotated −90° from body, the integrator must use
sim time rather than wall clock, and `ros_gz_bridge` cannot carry the message at
all. All four are written up in
[`.context/DVL_AND_SONAR.md`](.context/DVL_AND_SONAR.md), which also explains
why **sonar is not available** in Gazebo Harmonic and what to use instead.

---

## Full stack, terminal by terminal (GUI + DVL + RViz + lab)

Every terminal starts with the same three lines, **autonomy sourced before the
sim**:

```bash
source /opt/ros/humble/setup.bash
source ~/Ros_workspaces/duburi_ws/install/setup.bash
source ~/Ros_workspaces/duburi_ws/sim/install/setup.bash
export GZ_IP=127.0.0.1
```

### T1 — Gazebo with the GUI

```bash
ros2 run duburi_sim_bringup duburi_sim stop      # ALWAYS first: one sim only
ros2 run duburi_sim_bringup duburi_sim sim       # GUI (drop --headless)
```

Wait for **`JSON received`**. The GUI is where the **DVL beams** are drawn — four
lines from the hull to the floor. Headless shows nothing, which is why the same
beams are also published as RViz markers.

### T2 — the autonomy stack, with the DVL

```bash
export DUBURI_WS=~/Ros_workspaces/duburi_ws
ros2 run duburi_sim_bringup duburi_sim stack --no-vision
```

Defaults to `yaw_source=sim_dvl` — AHRS heading + DVL position. Look for
`[DVL  ] sim DVL subscribed to /dvl/velocity`. Add
`yaw_source:=mavlink_ahrs` to run without a DVL, in which case the `*_dist`
verbs **refuse** rather than dead-reckon.

### T3 — RViz

```bash
ros2 run duburi_sim_bringup duburi_sim rviz
```

Brings up `robot_state_publisher`, the `odom -> base_link` broadcaster and RViz
with the saved config. **Gazebo shows what is true; RViz shows what the vehicle
believes and can see** — which is why two poses are drawn:

| display | meaning |
|---|---|
| `pose TRUTH (Gazebo)` — axes | where the hull actually is |
| `pose BELIEVED (stack)` — orange arrow | where the stack thinks it is |

They separate vertically by the AHRS2 depth offset (measured 0.64 m in one run).
That gap *is* the bug class this simulator exists to expose.

Also shown: the robot model, the TF tree, DVL beams (green locked / red not),
DVL altitude, both camera feeds, and a ground-truth track.

### T4 — the operator lab

```bash
ros2 run duburi_sim_bringup duburi_sim lab
# http://localhost:28765     port: cat /tmp/duburi-$USER/lab_port.txt
```

Mode selector, yaw teleop (q/e), ground-truth vs believed depth, connection
health, dataset integrity badges.

### T5 — drive it

```bash
ros2 run duburi_sim_bridge contract_check        # prove the surface first
ros2 run duburi_planner mission sim_shakedown    # arm -> depth -> out -> back

# individual verbs
ros2 run duburi_planner duburi arm
ros2 run duburi_planner duburi set_depth --target -0.8
ros2 run duburi_planner duburi move_forward_dist --distance_m 2.0 --gain 55
ros2 run duburi_planner duburi arc --duration 6 --gain 60 --target_yaw 90
ros2 run duburi_planner duburi disarm

# measure any verb against ground truth
ros2 run duburi_sim_bridge verb_audit --group heading
```

### Live tuning

RViz displays; it does not tune. Gains stay where they already are:

```bash
ros2 param set /duburi_manager vision.kp_lat 80.0
ros2 param list /duburi_manager
```

### Shutdown

```bash
ros2 run duburi_sim_bringup duburi_sim stop
```

> **Reset between audit runs.** The pool spans x = ±12.5 and the hull drifts into
> a wall; once pinned, nothing moves and every verb looks dead.

---

## Courses

`courses/*.yaml` + `gen_world.py --all`. Six today:

| course | what it is for |
|---|---|
| `pool_empty` | bare pool, no props — hydrodynamics and step response |
| `sauvc26_qualification` | the qualification gate, ~10 m out |
| `sauvc26_final` | the full arena: gate, orange flare, four drums, three bump flares |
| `task_navigation` | gate + flare only, clearer water — drill the gate approach |
| `task_target_acquisition` | the four-drum target zone alone |
| `task_localization` | the three bump flares alone, in fog — the hardest detection |

The per-task courses exist so a failure is attributable: with the whole arena in
frame you cannot tell a control problem from the detector locking onto the wrong
prop. They also run at different turbidity, so perception is exercised across
conditions rather than one preset.

Prop placement follows the rulebook distances. Where the rulebook gives a **zone**
rather than a point — the orange flare, the bump flares, the drum order — one
legal arrangement is baked in and you vary it at runtime rather than editing YAML:

```bash
ros2 run duburi_sim_scenarios props list
ros2 run duburi_sim_scenarios props add sauvc_drum_blue drum_x 8.0 1.0
ros2 run duburi_sim_scenarios props move flare_red -2.0 4.0
ros2 run duburi_sim_scenarios props remove drum_x
```

Any registered prop, any pose, live — the same catalogue the World tab in the lab
drives. Adding a new prop is a builder plus one `PROPS` entry in
`duburi_sim_worlds/scripts/prop_library.py`; that single registration lights it up
in the model generator, the world generator, the ROS spawn service, the CLI and
the web catalogue at once.

## Vision in sim

```bash
ros2 run duburi_sim_bringup duburi_sim stack        # BOTH cameras + detectors
ros2 run duburi_sim_bringup duburi_sim stack --no-vision
```

Gives `/duburi_detector_forward` on the sim front camera and
`/duburi_detector_downward` on the bottom camera, correctly labelled so missions
and the vision verbs resolve `/duburi/vision/<name>/*` exactly as they do on the
vehicle.

> `vision:=true` started **nothing at all** until 2026-08-28 — a launch-scope leak
> silently disabled the whole include. Older notes telling you to run
> `--no-vision` predate the fix. See
> [`.context/TROUBLESHOOTING.md`](.context/TROUBLESHOOTING.md).

Useful arguments (all forwarded to `vision_dual.launch.py`):

| arg | default | note |
|---|---|---|
| `dwn_classes` | `fire,blood` | **`bin_fire_blood.pt` has no `bin` class** |
| `device_cls` | `cuda:0` | set `cpu` on a host without CUDA, or the detector dies |
| `viewer` | `false` | HUD on one camera; `f`/`d` switches |

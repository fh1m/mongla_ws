# duburi-sim_ws

Gazebo Harmonic + ArduSub SITL + operator web lab for **Mongla / Duburi 4.5**.
Drop-in MAVLink/camera surface for sibling autonomy [`../duburi_ws`](../duburi_ws).

| Start here | |
|------------|---|
| **Operators** | this README → [`.context/QUICKSTART.md`](.context/QUICKSTART.md) |
| **Agents (Claude / Cursor)** | [`.context/INDEX.md`](.context/INDEX.md) → [`.context/HANDOFF.md`](.context/HANDOFF.md) |
| **Autonomy integrators** | [`.context/CONTRACT.md`](.context/CONTRACT.md) · [`.context/INTEGRATION_DUBURI_WS.md`](.context/INTEGRATION_DUBURI_WS.md) |

> **Packaging:** this tree is intentionally **not** a git repo yet. The `duburi_ws`
> agent will `git init` / merge / submodule later — see [`.context/FUTURE_MERGE.md`](.context/FUTURE_MERGE.md).
> Do **not** `git init` here unless the user asks.

---

## Operator path — cold start → mission

### 0. Once per machine

```zsh
# ROS Humble + Gazebo Harmonic already installed in the auv-ros2 container
cd ~/Ros_workspaces/duburi-sim_ws
pip install -r requirements.txt   # PyYAML, numpy, Pillow for world gens
colcon build --symlink-install
source /opt/ros/humble/setup.zsh && source install/setup.zsh

# sibling autonomy (required for stack / smoke / arm / missions)
cd ~/Ros_workspaces/duburi_ws && colcon build --symlink-install

# optional timeseries UI
sudo apt install ros-humble-plotjuggler-ros
```

### 1. Simulator (Terminal 1)

```zsh
cd ~/Ros_workspaces/duburi-sim_ws
source /opt/ros/humble/setup.zsh && source install/setup.zsh
export GZ_IP=127.0.0.1

ros2 run duburi_sim_bringup duburi_sim stop    # always first
ros2 run duburi_sim_bringup duburi_sim sim     # GUI; or: sim --headless
```

Wait for **`JSON received`**. AUV sits at **x ≈ −11.8** (start zone).

### 2. Autonomy stack (Terminal 2)

On `duburi_ws` branch **`srot`**, SITL still needs **`flight_controller:=pixhawk`**
(the helper already forces this).

```zsh
source /opt/ros/humble/setup.zsh
source ~/Ros_workspaces/duburi_ws/install/setup.zsh
source ~/Ros_workspaces/duburi-sim_ws/install/setup.zsh
export DUBURI_WS=~/Ros_workspaces/duburi_ws GZ_IP=127.0.0.1

ros2 run duburi_sim_bringup duburi_sim stack --no-vision   # first bring-up
# later, with YOLO weights:  duburi_sim stack
```

### 3. Prove the loop (Terminal 3)

```zsh
# same sources as T2
ros2 run duburi_sim_bringup duburi_sim smoke
ros2 run duburi_sim_bridge contract_check
ros2 run duburi_sim_bridge mavlink_check
```

### 4. Operator lab (optional)

```zsh
ros2 run duburi_sim_bringup duburi_sim lab
# open http://localhost:28765  (or cat /tmp/duburi_lab_port.txt)
```

| Tab | Use |
|-----|-----|
| **Operate** | Cams, D-pad teleop (TCP 5763), arm, turbidity, record→zip |
| **World** | Start/restart/stop course, spawn/move props, custom model zip |
| **Datasets** | Clip list with wall duration + `fps_actual`, download zip |

### 5. Run a mission (from `duburi_ws`)

With sim + stack up:

```zsh
# list missions
ros2 run duburi_planner mission --list

# example — gate practice (needs vision stack for YOLO missions)
ros2 run duburi_sim_bringup duburi_sim stack          # with vision
ros2 run duburi_planner mission gate_flare_prequal    # or your mission id

# or drive manually via lab / planner CLI
ros2 run duburi_planner duburi arm
ros2 run duburi_planner duburi set_depth --target -1.0
ros2 run duburi_planner duburi move_forward --duration 5 --gain 60
ros2 run duburi_planner duburi disarm
```

Mission design / YOLO models live in **`duburi_ws`** — this repo only provides
physics, cameras, and the lab.

### 6. Datasets for vision

```zsh
# CLI
ros2 run duburi_sim_bridge record_cameras --duration 20 --fx --frames --labels \
  --label gate_approach

# or Operate → ● record → ■ stop (zip downloads)
# verify: ffprobe duration ≈ meta.duration_s
```

### 7. Timeseries / 3D tools

```zsh
ros2 run duburi_sim_bringup duburi_sim plotjuggler   # state + GT
# Foxglove: see duburi_ws/.claude/context/foxglove-and-bags.md
```

### 8. Shutdown

```zsh
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
| Lab teleop RC | TCP **5763** |
| Cams | `/duburi/sim/{front,bottom}_camera/image_raw` **640×480** |
| GT | `/duburi/sim/ground_truth` |
| Lab HTTP | `DUBURI_LAB_PORT` default **28765** |

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
| [`.context/HANDOFF.md`](.context/HANDOFF.md) | What Cursor built, why, next steps |
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

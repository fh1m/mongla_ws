# Quick start

> ℹ **Absorbed 2026-08-27.** This workspace is no longer the sibling tree
> `Ros_workspaces/duburi-sim_ws`; it lives inside the `duburi_ws` repo at
> `duburi_ws/sim/` and is under version control. Paths below have been
> updated; any remaining "sibling" phrasing is historical.

Minimal copy-paste path. Details: [OPERATOR.md](OPERATOR.md), [COMMAND_REFERENCE.md](COMMAND_REFERENCE.md).

## Prerequisites

- ROS 2 Humble + Gazebo Harmonic
- Built `duburi_ws/sim` (`colcon build --symlink-install`)
- Built sibling `duburi_ws` (for stack/smoke)
- ArduSub SITL + ArduPilot Gazebo plugin discoverable (`ARDUPILOT_ROOT` / `ARDUPILOT_GAZEBO_ROOT` if not in default paths)
- Display for GUI: `DISPLAY` + readable `XAUTHORITY` (helper auto-picks mutter cookie)

## Terminal 1 — simulator

```zsh
cd ~/Ros_workspaces/duburi_ws/sim
source /opt/ros/humble/setup.bash
source install/setup.bash
export GZ_IP=127.0.0.1

ros2 run duburi_sim_bringup duburi_sim stop
ros2 run duburi_sim_bringup duburi_sim sim
```

Wait for log `JSON received`. In Gazebo, look at the **−x wall** (x ≈ −11.8).

Headless: `duburi_sim sim --headless`.

## Terminal 2 — autonomy stack

```zsh
source /opt/ros/humble/setup.bash
source ~/Ros_workspaces/duburi_ws/install/setup.bash
source ~/Ros_workspaces/duburi_ws/sim/install/setup.bash
export DUBURI_WS=~/Ros_workspaces/duburi_ws
export GZ_IP=127.0.0.1

ros2 run duburi_sim_bringup duburi_sim stack --no-vision
```

## Terminal 3 — prove it

```zsh
# same sources as terminal 2
ros2 run duburi_sim_bringup duburi_sim smoke
ros2 run duburi_sim_bridge contract_check
ros2 run duburi_sim_bridge mavlink_check
```

## Optional — operator lab

```zsh
ros2 run duburi_sim_bringup duburi_sim lab
# open http://localhost:28765  (or DUBURI_LAB_PORT / /tmp/duburi_lab_port.txt)
```

## Optional — mission (needs stack; vision for YOLO missions)

```zsh
ros2 run duburi_planner mission --list
ros2 run duburi_sim_bringup duburi_sim stack          # with vision when ready
ros2 run duburi_planner mission <mission_id>
```

Missions and weights live in `duburi_ws`. Full operator path including datasets and
PlotJuggler: root [README.md](../README.md).

## One-sim rule

Always `duburi_sim stop` before a new `sim`. Two sims or two managers on UDP 14550
→ `Lost manual control`, missing AUV, weird depth.

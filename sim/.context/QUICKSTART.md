# Quick start

> ℹ **Absorbed 2026-08-27.** This workspace is no longer the sibling tree
> `Ros_workspaces/mongla-sim_ws`; it lives inside the `mongla_ws` repo at
> `mongla_ws/sim/` and is under version control. Paths below have been
> updated; any remaining "sibling" phrasing is historical.

Minimal copy-paste path. Details: [OPERATOR.md](OPERATOR.md), [COMMAND_REFERENCE.md](COMMAND_REFERENCE.md).

## Prerequisites

- ROS 2 Humble + Gazebo Harmonic
- Built `mongla_ws/sim` (`colcon build --symlink-install`)
- Built sibling `mongla_ws` (for stack/smoke)
- ArduSub SITL + ArduPilot Gazebo plugin discoverable (`ARDUPILOT_ROOT` / `ARDUPILOT_GAZEBO_ROOT` if not in default paths)
- Display for GUI: `DISPLAY` + readable `XAUTHORITY` (helper auto-picks mutter cookie)

## Terminal 1 — simulator

```zsh
cd ~/Ros_workspaces/mongla_ws/sim
source /opt/ros/humble/setup.bash
source install/setup.bash
export GZ_IP=127.0.0.1

ros2 run mongla_sim_bringup mongla_sim stop
ros2 run mongla_sim_bringup mongla_sim sim
```

Wait for log `JSON received`. In Gazebo, look at the **−x wall** (x ≈ −11.8).

Headless: `mongla_sim sim --headless`.

## Terminal 2 — autonomy stack

```zsh
source /opt/ros/humble/setup.bash
source ~/Ros_workspaces/mongla_ws/install/setup.bash
source ~/Ros_workspaces/mongla_ws/sim/install/setup.bash
export MONGLA_WS=~/Ros_workspaces/mongla_ws
export GZ_IP=127.0.0.1

ros2 run mongla_sim_bringup mongla_sim stack --no-vision
```

## Terminal 3 — prove it

```zsh
# same sources as terminal 2
ros2 run mongla_sim_bringup mongla_sim smoke
ros2 run mongla_sim_bridge contract_check
ros2 run mongla_sim_bridge mavlink_check
```

## Optional — operator lab

```zsh
ros2 run mongla_sim_bringup mongla_sim lab
# open http://localhost:28765  (or MONGLA_LAB_PORT / /tmp/mongla_lab_port.txt)
```

## Optional — mission (needs stack; vision for YOLO missions)

```zsh
ros2 run mongla_planner mission --list
ros2 run mongla_sim_bringup mongla_sim stack          # with vision when ready
ros2 run mongla_planner mission <mission_id>
```

Missions and weights live in `mongla_ws`. Full operator path including datasets and
PlotJuggler: root [README.md](../README.md).

## One-sim rule

Always `mongla_sim stop` before a new `sim`. Two sims or two managers on UDP 14550
→ `Lost manual control`, missing AUV, weird depth.

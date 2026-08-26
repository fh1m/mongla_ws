# PlotJuggler (timeseries monitoring)

> ℹ **Absorbed 2026-08-27.** This workspace is no longer the sibling tree
> `Ros_workspaces/duburi-sim_ws`; it lives inside the `duburi_ws` repo at
> `duburi_ws/sim/` and is under version control. Paths below have been
> updated; any remaining "sibling" phrasing is historical.

Desktop timeseries tool for AUV state / GT while the sim runs.
Product: [plotjuggler.io](https://plotjuggler.io/) · ROS plugins:
[plotjuggler-ros-plugins](https://github.com/PlotJuggler/plotjuggler-ros-plugins).

## Install (Humble)

```zsh
sudo apt update
sudo apt install ros-humble-plotjuggler-ros
```

## Launch with sim layout

```zsh
source /opt/ros/humble/setup.bash
source ~/Ros_workspaces/duburi_ws/sim/install/setup.bash
# sim + stack already running
ros2 run duburi_sim_bringup duburi_sim plotjuggler
# alias: duburi_sim pj
```

Layout file: `duburi_sim_bringup/config/plotjuggler_sim.xml` (installed under share).

In PlotJuggler: start **ROS2 Topic Subscriber**, then drag:

- `/duburi/state` (armed, depth, yaw, battery, …)
- `/duburi/sim/ground_truth` (pose)

## Division of labour

| Tool | Best for |
|------|----------|
| **PlotJuggler** | Fast multi-plot timeseries, transforms, CSV export |
| **Foxglove / Lichtblick** (`duburi_ws`) | 3D, images, bags — see `duburi_ws/.claude/context/foxglove-and-bags.md` |
| **Lab Operate** | Cams + teleop + record clips |

## Tips

- Do not bind a second MAVLink consumer on UDP 14550; PJ uses ROS topics only.
- For offline analysis, bag `/duburi/state` + GT (roadmap: lab MCAP button in `TESTING_SUITE.md`).

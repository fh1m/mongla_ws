# PlotJuggler (timeseries monitoring)

> ℹ **Absorbed 2026-08-27.** This workspace is no longer the sibling tree
> `Ros_workspaces/mongla-sim_ws`; it lives inside the `mongla_ws` repo at
> `mongla_ws/sim/` and is under version control. Paths below have been
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
source ~/Ros_workspaces/mongla_ws/sim/install/setup.bash
# sim + stack already running
ros2 run mongla_sim_bringup mongla_sim plotjuggler
# alias: mongla_sim pj
```

Layout file: `mongla_sim_bringup/config/plotjuggler_sim.xml` (installed under share).

In PlotJuggler: start **ROS2 Topic Subscriber**, then drag:

- `/mongla/state` (armed, depth, yaw, battery, …)
- `/mongla/sim/ground_truth` (pose)

## Division of labour

| Tool | Best for |
|------|----------|
| **PlotJuggler** | Fast multi-plot timeseries, transforms, CSV export |
| **Foxglove / Lichtblick** (`mongla_ws`) | 3D, images, bags — see `mongla_ws/.claude/context/platform/foxglove-and-bags.md` |
| **Lab Operate** | Cams + teleop + record clips |

## Tips

- Do not bind a second MAVLink consumer on UDP 14550; PJ uses ROS topics only.
- For offline analysis, bag `/mongla/state` + GT (roadmap: lab MCAP button in `TESTING_SUITE.md`).

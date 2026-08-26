# Mongla Gazebo lab (`duburi-sim_ws`)

**Canonical sim docs live in the sibling workspace**, not in this file:

→ [`../../duburi-sim_ws/.context/INDEX.md`](../../duburi-sim_ws/.context/INDEX.md)

(Absolute typical path: `~/Ros_workspaces/duburi-sim_ws/.context/INDEX.md`.)

> [`sim-setup.md`](./sim-setup.md) describes the **legacy** BlueROV / older Gazebo path.
> For current Mongla SITL + operator lab, prefer the sibling `.context/` above.

## Why sibling?

`duburi-sim_ws` is a drop-in Gazebo Harmonic + ArduSub SITL + web lab that speaks the
same MAVLink/camera contract as the pool vehicle. Packages are **not** merged into
`duburi_ws` yet (see sim `FUTURE_MERGE.md`).

## Bring-up (srot + SITL)

On branch **`srot`**, SITL is still ArduSub — the stack must use **`flight_controller:=pixhawk`**.
The sim helper already does that via `stack.launch.py`.

```bash
# terminal 1 — physics
source /opt/ros/humble/setup.bash
source ~/Ros_workspaces/duburi-sim_ws/install/setup.bash
export GZ_IP=127.0.0.1
ros2 run duburi_sim_bringup duburi_sim stop
ros2 run duburi_sim_bringup duburi_sim sim

# terminal 2 — autonomy against sim
source /opt/ros/humble/setup.bash
source ~/Ros_workspaces/duburi_ws/install/setup.bash
source ~/Ros_workspaces/duburi-sim_ws/install/setup.bash
ros2 run duburi_sim_bringup duburi_sim stack --no-vision

# checks / lab / timeseries
ros2 run duburi_sim_bringup duburi_sim smoke
ros2 run duburi_sim_bringup duburi_sim lab          # http://localhost:28765
ros2 run duburi_sim_bringup duburi_sim plotjuggler  # apt: ros-humble-plotjuggler-ros
```

## Contract (do not drift)

| Surface | Value |
|---------|-------|
| Autonomy MAVLink | UDP **14550** (manager `udpin`) |
| Lab teleop RC | TCP **5763** (does not steal 14550) |
| Front / bottom cams | `/duburi/sim/{front,bottom}_camera/image_raw` **640×480** |
| Ground truth | `/duburi/sim/ground_truth` |
| Lab HTTP | `DUBURI_LAB_PORT` default **28765** |

Vision missions still expect the usual forward camera remap from the sim stack launch.

## Datasets → vision

Operator lab Operate tab → record (fx / frames / labels) → `duburi-sim_ws/datasets/` → zip.
YOLO train handoff stays in this repo’s vision docs; sim only produces GT clips.

## Tooling split

| Tool | Use |
|------|-----|
| Lab Operate / World | Cams, teleop, props, record |
| PlotJuggler (`duburi_sim plotjuggler`) | Timeseries `/duburi/state`, GT |
| Foxglove ([foxglove-and-bags.md](./foxglove-and-bags.md)) | 3D, images, bags |

## Deeper reading

| Topic | Doc |
|-------|-----|
| Operator path | `duburi-sim_ws/.context/OPERATOR.md` |
| Commands | `duburi-sim_ws/.context/COMMAND_REFERENCE.md` |
| Contract detail | `duburi-sim_ws/.context/CONTRACT.md` |
| World / props | `duburi-sim_ws/.context/WORLD_EDITING.md` |
| PlotJuggler | `duburi-sim_ws/.context/PLOTJUGGLER.md` |
| Integration notes | `duburi-sim_ws/.context/INTEGRATION_DUBURI_WS.md` |

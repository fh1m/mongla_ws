# Mongla Gazebo lab (`duburi-sim_ws`)

**Canonical sim docs live in the sibling workspace**, not in this file:

→ [`../../duburi-sim_ws/.context/INDEX.md`](../../duburi-sim_ws/.context/INDEX.md)

Also read the Cursor-agent handoff (what was built, why, packaging):

→ [`../../duburi-sim_ws/.context/HANDOFF.md`](../../duburi-sim_ws/.context/HANDOFF.md)

(Absolute: `~/Ros_workspaces/duburi-sim_ws/.context/`.)

> [`sim-setup.md`](./sim-setup.md) describes the **legacy** BlueROV / older Gazebo path.
> For current Mongla SITL + operator lab, prefer the sibling `.context/` above.

## Packaging (for the duburi_ws agent)

- `duburi-sim_ws` is a **sibling** tree and currently has **no `.git`**.
- **You** own `git init` / submodule / subtree when integrating — see sim
  [`FUTURE_MERGE.md`](../../duburi-sim_ws/.context/FUTURE_MERGE.md).
- Do **not** invent a third packaging story; do **not** expect a sim GitHub remote yet.
- Open integration docs PR: https://github.com/fh1m/duburi_ws/pull/8

## Why sibling?

`duburi-sim_ws` is a drop-in Gazebo Harmonic + ArduSub SITL + web lab that speaks the
same MAVLink/camera contract as the pool vehicle. Packages are **not** merged into
`duburi_ws` yet.

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
export DUBURI_WS=~/Ros_workspaces/duburi_ws
ros2 run duburi_sim_bringup duburi_sim stack --no-vision

# checks / lab / timeseries / mission
ros2 run duburi_sim_bringup duburi_sim smoke
ros2 run duburi_sim_bridge contract_check
ros2 run duburi_sim_bringup duburi_sim lab          # http://localhost:28765
ros2 run duburi_sim_bringup duburi_sim plotjuggler  # apt: ros-humble-plotjuggler-ros
ros2 run duburi_planner mission --list
# with vision: duburi_sim stack && ros2 run duburi_planner mission <id>
```

Operator cold-start → mission cheat sheet:
[`../../duburi-sim_ws/README.md`](../../duburi-sim_ws/README.md).

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
MP4 duration matches wall time (`fps_actual`). YOLO train handoff stays in this repo’s vision docs.

## Tooling split

| Tool | Use |
|------|-----|
| Lab Operate / World | Cams, teleop, props, record |
| PlotJuggler (`duburi_sim plotjuggler`) | Timeseries `/duburi/state`, GT |
| Foxglove ([foxglove-and-bags.md](./foxglove-and-bags.md)) | 3D, images, bags |

## Deeper reading

| Topic | Doc |
|-------|-----|
| Cursor handoff | `duburi-sim_ws/.context/HANDOFF.md` |
| Operator path | `duburi-sim_ws/.context/OPERATOR.md` + root README |
| Commands | `duburi-sim_ws/.context/COMMAND_REFERENCE.md` |
| Contract detail | `duburi-sim_ws/.context/CONTRACT.md` |
| World / props | `duburi-sim_ws/.context/WORLD_EDITING.md` |
| PlotJuggler | `duburi-sim_ws/.context/PLOTJUGGLER.md` |
| Integration notes | `duburi-sim_ws/.context/INTEGRATION_DUBURI_WS.md` |
| Known bugs | `duburi-sim_ws/.context/AUDIT.md` |

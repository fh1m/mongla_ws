# Integrating with `mongla_ws`

> ℹ **Absorbed 2026-08-27.** This workspace is no longer the sibling tree
> `Ros_workspaces/mongla-sim_ws`; it lives inside the `mongla_ws` repo at
> `mongla_ws/sim/` and is under version control. Paths below have been
> updated; any remaining "sibling" phrasing is historical.

Sibling layout (v0.1, **do not move packages yet**):

```text
Ros_workspaces/
  mongla_ws/          # autonomy
  mongla_ws/sim/      # this simulator
```

## What autonomy expects

See [CONTRACT.md](CONTRACT.md). Short form:

- MAVLink on **14550** as pixhawk/ArduSub
- `/mongla/move` + `/mongla/state` from manager
- Optional vision on `/mongla/sim/front_camera/image_raw` as camera `forward`

## Source overlay

```zsh
source /opt/ros/humble/setup.bash
source "$MONGLA_WS/install/setup.bash"
source ~/Ros_workspaces/mongla_ws/sim/install/setup.bash
export MONGLA_WS  # absolute path
export GZ_IP=127.0.0.1
```

## Branch notes

### `mongla_ws` `main`

Pixhawk-era defaults. `mongla_sim stack` works as documented in README.

### `mongla_ws` `srot` (current Mongla hardware branch)

Default FC is **USB SROT**. For Gazebo you **must** use pixhawk profile:

```zsh
# Preferred — already sets flight_controller:=pixhawk
ros2 run mongla_sim_bringup mongla_sim stack --no-vision
```

Manual equivalent:

```zsh
ros2 launch mongla_manager bringup.launch.py \
  mode:=sim flight_controller:=pixhawk yaw_source:=mavlink_ahrs \
  dvl_auto_connect:=false vision:=false
```

Omitting `flight_controller:=pixhawk` on `srot` will hunt for a USB board and fail SITL.

## Vision remap

`stack.launch.py` launches vision with:

- `camera:=forward` (detector name matches missions)
- `topic:=/mongla/sim/front_camera/image_raw`

Do **not** use `camera:=sim_front` unless you also rename mission detector expectations.

```zsh
ros2 launch mongla_vision vision.launch.py camera:=forward \
  topic:=/mongla/sim/front_camera/image_raw \
  model:=gate_rescue_repair classes:=gate
```

## Lab needs `MONGLA_WS`

Arm/disarm and timed cmds shell out to:

```text
ros2 run mongla_planner mongla …
```

with `MONGLA_WS` on `PATH`/overlay. Set explicitly if sibling discovery fails.

## What not to do

| Anti-pattern | Why |
|--------------|-----|
| Second bind on 14550 | Breaks arm / FS |
| Point lab teleop at 14550 | Fights manager; use 5763 |
| Rely on `sim-setup.md` BlueROV/`colcon_ws` | Obsolete vs this WS |
| Assume world hot-swap | Lab restart = full stop/start |

## Suggested one-liner for `mongla_ws` docs (optional paste)

> **Simulator:** use sibling workspace `mongla_ws/sim` (Gazebo Harmonic + ArduSub).
> Docs: `mongla_ws/sim/.context/INDEX.md`. On branch `srot`, always
> `flight_controller:=pixhawk` (or `mongla_sim stack`). Prefer that over
> `.claude/context/sim-setup.md` (legacy).

Do not apply that paste in `mongla_ws` unless the user asks — this file is the
canonical integration note for agents working from the sim side.

## Developing autonomy against sim

1. Keep sim+stack running in dedicated terminals.  
2. Iterate missions/planner in `mongla_ws` only.  
3. Cameras: subscribe contract topics; optional FX for domain tests.  
4. Use lab for dataset clips; use planner for scripted motions.  
5. Gate scoring: `gate_transit_check` + GT.

## Future packaging

See [FUTURE_MERGE.md](FUTURE_MERGE.md) — sibling remains default until an
explicit merge/subtree/submodule decision.

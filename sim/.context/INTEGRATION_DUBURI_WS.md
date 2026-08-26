# Integrating with `duburi_ws`

> ℹ **Absorbed 2026-08-27.** This workspace is no longer the sibling tree
> `Ros_workspaces/duburi-sim_ws`; it lives inside the `duburi_ws` repo at
> `duburi_ws/sim/` and is under version control. Paths below have been
> updated; any remaining "sibling" phrasing is historical.

Sibling layout (v0.1, **do not move packages yet**):

```text
Ros_workspaces/
  duburi_ws/          # autonomy
  duburi_ws/sim/      # this simulator
```

## What autonomy expects

See [CONTRACT.md](CONTRACT.md). Short form:

- MAVLink on **14550** as pixhawk/ArduSub
- `/duburi/move` + `/duburi/state` from manager
- Optional vision on `/duburi/sim/front_camera/image_raw` as camera `forward`

## Source overlay

```zsh
source /opt/ros/humble/setup.bash
source "$DUBURI_WS/install/setup.bash"
source ~/Ros_workspaces/duburi_ws/sim/install/setup.bash
export DUBURI_WS  # absolute path
export GZ_IP=127.0.0.1
```

## Branch notes

### `duburi_ws` `main`

Pixhawk-era defaults. `duburi_sim stack` works as documented in README.

### `duburi_ws` `srot` (current Mongla hardware branch)

Default FC is **USB SROT**. For Gazebo you **must** use pixhawk profile:

```zsh
# Preferred — already sets flight_controller:=pixhawk
ros2 run duburi_sim_bringup duburi_sim stack --no-vision
```

Manual equivalent:

```zsh
ros2 launch duburi_manager bringup.launch.py \
  mode:=sim flight_controller:=pixhawk yaw_source:=mavlink_ahrs \
  dvl_auto_connect:=false vision:=false
```

Omitting `flight_controller:=pixhawk` on `srot` will hunt for a USB board and fail SITL.

## Vision remap

`stack.launch.py` launches vision with:

- `camera:=forward` (detector name matches missions)
- `topic:=/duburi/sim/front_camera/image_raw`

Do **not** use `camera:=sim_front` unless you also rename mission detector expectations.

```zsh
ros2 launch duburi_vision vision.launch.py camera:=forward \
  topic:=/duburi/sim/front_camera/image_raw \
  model:=gate_rescue_repair classes:=gate
```

## Lab needs `DUBURI_WS`

Arm/disarm and timed cmds shell out to:

```text
ros2 run duburi_planner duburi …
```

with `DUBURI_WS` on `PATH`/overlay. Set explicitly if sibling discovery fails.

## What not to do

| Anti-pattern | Why |
|--------------|-----|
| Second bind on 14550 | Breaks arm / FS |
| Point lab teleop at 14550 | Fights manager; use 5763 |
| Rely on `sim-setup.md` BlueROV/`colcon_ws` | Obsolete vs this WS |
| Assume world hot-swap | Lab restart = full stop/start |

## Suggested one-liner for `duburi_ws` docs (optional paste)

> **Simulator:** use sibling workspace `duburi_ws/sim` (Gazebo Harmonic + ArduSub).
> Docs: `duburi_ws/sim/.context/INDEX.md`. On branch `srot`, always
> `flight_controller:=pixhawk` (or `duburi_sim stack`). Prefer that over
> `.claude/context/sim-setup.md` (legacy).

Do not apply that paste in `duburi_ws` unless the user asks — this file is the
canonical integration note for agents working from the sim side.

## Developing autonomy against sim

1. Keep sim+stack running in dedicated terminals.  
2. Iterate missions/planner in `duburi_ws` only.  
3. Cameras: subscribe contract topics; optional FX for domain tests.  
4. Use lab for dataset clips; use planner for scripted motions.  
5. Gate scoring: `gate_transit_check` + GT.

## Future packaging

See [FUTURE_MERGE.md](FUTURE_MERGE.md) — sibling remains default until an
explicit merge/subtree/submodule decision.

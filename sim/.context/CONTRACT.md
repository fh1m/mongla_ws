# Drop-in contract (sim ↔ `mongla_ws`)

> ℹ **Absorbed 2026-08-27.** This workspace is no longer the sibling tree
> `Ros_workspaces/mongla-sim_ws`; it lives inside the `mongla_ws` repo at
> `mongla_ws/sim/` and is under version control. Paths below have been
> updated; any remaining "sibling" phrasing is historical.

This workspace must look like the real vehicle to `mongla_ws`. If you change
any row below, update `contract_check` / `mavlink_check` and this file.

## MAVLink

| Item | Contract |
|------|----------|
| Transport | ArduSub pushes `udpclient:127.0.0.1:14550` |
| Autonomy bind | `udpin:0.0.0.0:14550` (manager profile `sim`) |
| Secondary | UDP **14551** for GCS / MAVProxy only |
| Frame | `vectored_6dof` (8 thrusters) — `sub-6dof.parm` + overlay |
| Required traffic | HEARTBEAT, ATTITUDE, AHRS2, BATTERY_STATUS (see `mavlink_check`) |
| Manager mode | `mode:=sim`, **`flight_controller:=pixhawk`** |

**Do not** attach a second client that steals the only 14550 consumer, or run two
managers. Lab teleop intentionally uses **TCP 5763**, not 14550.

Verify:

```zsh
ros2 run mongla_sim_bridge mavlink_check
```

## ROS control / state (from `mongla_ws`)

| Surface | Name |
|---------|------|
| Action | `/mongla/move` (`mongla_interfaces/action/Move`) |
| State | `/mongla/state` |
| CLI | `ros2 run mongla_planner mongla …` |

These come from the stack, not the sim bridge. Sim only needs a healthy ArduSub
link so manager can arm and command.

## Cameras (sim bridge)

| Topic | Spec |
|-------|------|
| `/mongla/sim/front_camera/image_raw` | 640×480, rgb8/bgr8 |
| `/mongla/sim/bottom_camera/image_raw` | same |
| `/mongla/sim/front_camera/camera_info` | matching |
| `/mongla/sim/bottom_camera/camera_info` | matching |
| `/mongla/sim/{front,bottom}_camera/image_fx` | optional degraded feed (`underwater_fx`) |
| `/mongla/sim/ground_truth` | Pose of vehicle in world (recorder / scoring) |

Vision launch (via `stack.launch.py`) remaps:

- `camera:=forward`
- `topic:=/mongla/sim/front_camera/image_raw`

so the detector node is `/mongla_detector_forward` (missions expect `forward`,
not `sim_front`).

Verify:

```zsh
ros2 run mongla_sim_bridge contract_check
```

## Lab-only surfaces (not required for autonomy)

| Surface | Notes |
|---------|-------|
| HTTP lab | `:28765` FastAPI + static UI |
| Teleop | `MONGLA_TELEOP_ENDPOINT` → TCP 5763 RC override |
| Props services | `/mongla/sim/props/{spawn,move,delete}` via `prop_manager` |
| Datasets | `datasets/<label>_<stamp>/` on disk |

Autonomy can ignore these; dataset collection uses them.

## Source order for a full session

```text
humble → mongla_ws/install → mongla_ws/sim/install
```

`stack.launch.py` includes `mongla_manager` / `mongla_vision` from `mongla_ws`.

## Compatibility matrix

| `mongla_ws` branch | How to run against this sim |
|--------------------|-----------------------------|
| `main` | `mongla_sim stack` (pixhawk/SITL native) |
| `srot` | Same — **must** pass `flight_controller:=pixhawk` (already in `stack.launch.py`). Bare `ros2 run mongla_manager start` defaults to USB SROT and will miss SITL. |

## Legacy doc warning

`mongla_ws/.claude/context/sim-setup.md` describes an older BlueROV / `colcon_ws`
path. Prefer **this** `.context/` (now `mongla_ws/sim/.context/`) for all new work.

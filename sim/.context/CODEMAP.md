# Code map — path → responsibility

> ℹ **Absorbed 2026-08-27.** This workspace is no longer the sibling tree
> `Ros_workspaces/mongla-sim_ws`; it lives inside the `mongla_ws` repo at
> `mongla_ws/sim/` and is under version control. Paths below have been
> updated; any remaining "sibling" phrasing is historical.

Workspace root: `mongla_ws/sim/`. Paths relative to that root unless noted.

## Bring-up / CLI

| Path | Role |
|------|------|
| `src/mongla_sim_bringup/scripts/mongla_sim` | Operator helper: `stop|sim|stack|smoke|lab` |
| `scripts/mongla-sim` | Thin wrapper → installed script |
| `src/mongla_sim_bringup/launch/sim.launch.py` | Gazebo server (+GUI) → wait IMU → ArduSub → bridge |
| `src/mongla_sim_bringup/launch/stack.launch.py` | `mongla_manager` (`mode:=sim`, `flight_controller:=pixhawk`) + optional vision |
| `src/mongla_sim_bringup/launch/wait_for_gazebo.py` | Gate until vehicle IMU exists |
| `src/mongla_sim_bringup/config/mongla_sub.parm` | ArduSub overlay (FS/GPS/sim) |
| `src/mongla_sim_bringup/config/mongla_sub_extnav.parm` | ATT_POS_MOCAP path variant |
| `src/mongla_sim_bringup/config/gui.config` | Gazebo GUI ImageDisplay panels |

## Description / worlds

| Path | Role |
|------|------|
| `src/mongla_sim_description/models/mongla_heavy/` | Vehicle model + `configs.yaml` + generator inputs |
| `src/mongla_sim_description/scripts/generate_model.py` | SDF generation |
| `src/mongla_sim_description/models/mongla_heavy/HYDRODYNAMICS.md` | Hydro coeff notes |
| `src/mongla_sim_worlds/spec/arena.yaml` | SAUVC geometry source of truth |
| `src/mongla_sim_worlds/courses/*.yaml` | Course layouts (`pool_empty`, `sauvc26_*`) |
| `src/mongla_sim_worlds/worlds/*.world` | Generated Gazebo worlds |
| `src/mongla_sim_worlds/models/` | Prop SDF models |
| `src/mongla_sim_worlds/scripts/gen_world.py` | Regenerate worlds from courses |
| `src/mongla_sim_worlds/scripts/prop_library.py` | Prop catalog helpers |

## Bridge / sensors / datasets

| Path | Role |
|------|------|
| `src/mongla_sim_bridge/launch/bridge.launch.py` | ros_gz image + GT bridge + `underwater_fx` |
| `src/mongla_sim_bridge/config/underwater_fx.yaml` | Default turbidity/FX params |
| `src/mongla_sim_bridge/mongla_sim_bridge/underwater_fx.py` | `image_raw` → `image_fx` |
| `src/mongla_sim_bridge/mongla_sim_bridge/record_cameras.py` | Dataset recorder (mp4, frames, YOLO GT) |
| `src/mongla_sim_bridge/mongla_sim_bridge/gt_labels.py` | Project course props → YOLO boxes |
| `src/mongla_sim_bridge/mongla_sim_bridge/contract_check.py` | Camera/GT contract vs `mongla_ws` |
| `src/mongla_sim_bridge/mongla_sim_bridge/mavlink_check.py` | UDP 14550 message rates |
| `datasets/` | On-disk runs (`<label>_<stamp>/`) — usually gitignored content |

## Scenarios / props

| Path | Role |
|------|------|
| `src/mongla_sim_scenarios/mongla_sim_scenarios/prop_manager.py` | gz-transport spawn/move/delete services |
| `src/mongla_sim_scenarios/mongla_sim_scenarios/cli.py` | `props list|add|move|remove` |
| `src/mongla_sim_scenarios/mongla_sim_scenarios/prop_catalog.py` | Library + anchors |
| `src/mongla_sim_scenarios/mongla_sim_scenarios/gate_transit_check.py` | GT gate-passage scorer |

## Operator lab (web)

| Path | Role |
|------|------|
| `src/mongla_sim_web/mongla_sim_web/server.py` | FastAPI lab_server (sim/record/props/datasets/teleop) |
| `src/mongla_sim_web/mongla_sim_web/teleop.py` | pymavlink RC override on TCP 5763 |
| `src/mongla_sim_web/mongla_sim_web/ros_bridge.py` | Lab ROS node: cams JPEG, state, FX params |
| `src/mongla_sim_web/mongla_sim_web/script_runner.py` | YAML move scripts + recorder |
| `src/mongla_sim_web/launch/lab.launch.py` | Launch wrapper for `lab_server` |
| `src/mongla_sim_web/scripts/*.yaml` | Dataset collection move scripts |
| `src/mongla_sim_web/frontend/` | React + Vite source |
| `src/mongla_sim_web/static/` | Built assets served by FastAPI |

## Docs / agent entry

| Path | Role |
|------|------|
| `.context/` | **Canonical** operator + developer + audit docs |
| `README.md` | Thin index → `.context` |
| `CLAUDE.md` / `AGENTS.md` | Agent bootstrap → `.context/INDEX.md` |
| `requirements.txt` | Python deps for asset regen only |

## External (not in this repo)

| Path / artifact | Role |
|-----------------|------|
| `../mongla_ws` | Autonomy (`mongla_manager`, planner, vision) |
| `stuff/ardupilot` (or `ARDUPILOT_ROOT`) | ArduSub SITL binary |
| `stuff/ardupilot_gazebo` (or `ARDUPILOT_GAZEBO_ROOT`) | `libArduPilotPlugin.so` |

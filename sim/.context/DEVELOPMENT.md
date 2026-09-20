# Development guide (humans and agents)

> ℹ **Absorbed 2026-08-27.** This workspace is no longer the sibling tree
> `Ros_workspaces/mongla-sim_ws`; it lives inside the `mongla_ws` repo at
> `mongla_ws/sim/` and is under version control. Paths below have been
> updated; any remaining "sibling" phrasing is historical.

## Agent bootstrap

1. Read [INDEX.md](INDEX.md) then [CODEMAP.md](CODEMAP.md).
2. Obey hard rules in INDEX (one sim, pixhawk on srot, prefer this `.context`).
3. Before claiming green, run checks in [TESTING.md](TESTING.md).
4. Do not edit `.cursor/plans/*.plan.md` unless asked.
5. Do not force-push / amend git unless user rules allow.

## Build

```zsh
cd ~/Ros_workspaces/mongla_ws/sim
source /opt/ros/humble/setup.bash
# optional first: source sibling autonomy if linking against it at build time
colcon build --symlink-install
source install/setup.bash
```

Package-selective:

```zsh
colcon build --packages-select mongla_sim_web mongla_sim_bridge --symlink-install
```

Python packages use `setup.py` console_scripts. With `--symlink-install`, edits under
`src/*/mongla_*/*.py` often apply without full rebuild; still rebuild after
`setup.py` / resource / static changes.

## Lab frontend

```zsh
cd src/mongla_sim_web/frontend
npm install
npm run build          # writes to src/mongla_sim_web/static/
```

Vite dev proxy targets `127.0.0.1:28765` (`frontend/vite.config.js`).

After static change:

```zsh
colcon build --packages-select mongla_sim_web --symlink-install
```

UI aesthetic: Unauthorized Engineering / Mongla — `#F2F2F0` / `#111111`, Roboto Mono,
UE logo. Keep mission-control density; avoid generic dashboard chrome.

## Regenerating models / worlds

Needs `requirements.txt` (PyYAML, numpy, Pillow) for generators:

```zsh
src/mongla_sim_description/scripts/generate_model.py ...
src/mongla_sim_worlds/scripts/gen_world.py --all
```

Arena SoT: `src/mongla_sim_worlds/spec/arena.yaml`. Courses: `courses/*.yaml`.

## Optional desktop tools

```zsh
sudo apt install ros-humble-plotjuggler-ros
ros2 run mongla_sim_bringup mongla_sim plotjuggler
```

See [PLOTJUGGLER.md](PLOTJUGGLER.md). Foxglove remains in `mongla_ws` for 3D/images.

## Where to change what

| Goal | Start here |
|------|------------|
| Ports / lab APIs | `mongla_sim_web/server.py`, `teleop.py` |
| Bring-up order | `bringup/launch/sim.launch.py`, `stack.launch.py` |
| CLI helper | `bringup/scripts/mongla_sim` |
| Camera contract | `bridge/launch/bridge.launch.py`, `contract_check.py` |
| Recorder / meta | `bridge/record_cameras.py`, `gt_labels.py` |
| FX defaults | `bridge/config/underwater_fx.yaml` |
| Props / assets | `scenarios/prop_manager.py`, `cli.py`, lab `/api/assets/upload` |
| Course layout | `worlds/courses/*.yaml` + `gen_world.py` |
| PlotJuggler layout | `bringup/config/plotjuggler_sim.xml` |

## Verification culture

| Claim | Prove with |
|-------|------------|
| Cams OK | `contract_check` |
| MAVLink OK | `mavlink_check` |
| Control OK | `mongla_sim smoke` or teleop GT delta |
| Lab OK | browser Operate/World/Datasets + `/api/health` |
| Record OK | `meta.json` + `fps_actual` + ffprobe ≈ `duration_s` + zip 200 |

## Out-of-scope traps

- True Gazebo world hot-swap
- Merging packages into `mongla_ws` without [FUTURE_MERGE.md](FUTURE_MERGE.md)
- Training YOLO in this repo
- Using `mongla_ws` `sim-setup.md` as current truth
- Embedding PlotJuggler inside the web lab

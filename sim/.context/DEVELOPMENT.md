# Development guide (humans and agents)

> ℹ **Absorbed 2026-08-27.** This workspace is no longer the sibling tree
> `Ros_workspaces/duburi-sim_ws`; it lives inside the `duburi_ws` repo at
> `duburi_ws/sim/` and is under version control. Paths below have been
> updated; any remaining "sibling" phrasing is historical.

## Agent bootstrap

1. Read [INDEX.md](INDEX.md) then [CODEMAP.md](CODEMAP.md).
2. Obey hard rules in INDEX (one sim, pixhawk on srot, prefer this `.context`).
3. Before claiming green, run checks in [TESTING.md](TESTING.md).
4. Do not edit `.cursor/plans/*.plan.md` unless asked.
5. Do not force-push / amend git unless user rules allow.

## Build

```zsh
cd ~/Ros_workspaces/duburi_ws/sim
source /opt/ros/humble/setup.bash
# optional first: source sibling autonomy if linking against it at build time
colcon build --symlink-install
source install/setup.bash
```

Package-selective:

```zsh
colcon build --packages-select duburi_sim_web duburi_sim_bridge --symlink-install
```

Python packages use `setup.py` console_scripts. With `--symlink-install`, edits under
`src/*/duburi_*/*.py` often apply without full rebuild; still rebuild after
`setup.py` / resource / static changes.

## Lab frontend

```zsh
cd src/duburi_sim_web/frontend
npm install
npm run build          # writes to src/duburi_sim_web/static/
```

Vite dev proxy targets `127.0.0.1:28765` (`frontend/vite.config.js`).

After static change:

```zsh
colcon build --packages-select duburi_sim_web --symlink-install
```

UI aesthetic: Unauthorized Engineering / Mongla — `#F2F2F0` / `#111111`, Roboto Mono,
UE logo. Keep mission-control density; avoid generic dashboard chrome.

## Regenerating models / worlds

Needs `requirements.txt` (PyYAML, numpy, Pillow) for generators:

```zsh
src/duburi_sim_description/scripts/generate_model.py ...
src/duburi_sim_worlds/scripts/gen_world.py --all
```

Arena SoT: `src/duburi_sim_worlds/spec/arena.yaml`. Courses: `courses/*.yaml`.

## Optional desktop tools

```zsh
sudo apt install ros-humble-plotjuggler-ros
ros2 run duburi_sim_bringup duburi_sim plotjuggler
```

See [PLOTJUGGLER.md](PLOTJUGGLER.md). Foxglove remains in `duburi_ws` for 3D/images.

## Where to change what

| Goal | Start here |
|------|------------|
| Ports / lab APIs | `duburi_sim_web/server.py`, `teleop.py` |
| Bring-up order | `bringup/launch/sim.launch.py`, `stack.launch.py` |
| CLI helper | `bringup/scripts/duburi_sim` |
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
| Control OK | `duburi_sim smoke` or teleop GT delta |
| Lab OK | browser Operate/World/Datasets + `/api/health` |
| Record OK | `meta.json` + `fps_actual` + ffprobe ≈ `duration_s` + zip 200 |

## Out-of-scope traps

- True Gazebo world hot-swap
- Merging packages into `duburi_ws` without [FUTURE_MERGE.md](FUTURE_MERGE.md)
- Training YOLO in this repo
- Using `duburi_ws` `sim-setup.md` as current truth
- Embedding PlotJuggler inside the web lab

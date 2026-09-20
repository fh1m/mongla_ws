# Command reference

> ℹ **Absorbed 2026-08-27.** This workspace is no longer the sibling tree
> `Ros_workspaces/mongla-sim_ws`; it lives inside the `mongla_ws` repo at
> `mongla_ws/sim/` and is under version control. Paths below have been
> updated; any remaining "sibling" phrasing is historical.

Every operator/dev command, env var, port, and launch arg for `mongla_ws/sim`.
Source truth: `src/mongla_sim_bringup/scripts/mongla_sim`, launch files, `setup.py`
entry points, `server.py`.

## Source / overlay order

```zsh
source /opt/ros/humble/setup.bash
source "$MONGLA_WS/install/setup.bash"          # autonomy (when using stack/smoke/lab arm)
source ~/Ros_workspaces/mongla_ws/sim/install/setup.bash
export GZ_IP=127.0.0.1
export MONGLA_WS="${MONGLA_WS:-$HOME/Ros_workspaces/mongla_ws}"
```

`mongla_sim` auto-sources humble + this install + sibling `MONGLA_WS` when set.

Alias (after install sourced):

```zsh
# equivalent forms
ros2 run mongla_sim_bringup mongla_sim <cmd>
# or installed script name from PATH after sourcing install
```

---

## `mongla_sim` helper

| Command | Behavior |
|---------|----------|
| `stop` | SIGKILL leftover sim/stack/gz/ardusub/manager/vision (via `/proc` match) |
| `sim` | `stop` then `ros2 launch mongla_sim_bringup sim.launch.py gui:=true` |
| `sim --headless` | Same with `gui:=false` |
| `sim --gui` | Force GUI |
| `sim …` | Extra args forwarded to `sim.launch.py` (e.g. `course:=sauvc26_final`) |
| `stack` | Kill prior stack only; `stack.launch.py vision:=true` |
| `stack --no-vision` | Manager only (recommended first bringup) |
| `smoke` | Wait `/mongla/state` → arm → `set_depth -1` → `move_forward 8s` → echo GT |
| `lab` | Ensure `prop_manager`; `ros2 launch mongla_sim_web lab.launch.py` on `MONGLA_LAB_PORT` |
| `plotjuggler` / `pj` | Desktop PlotJuggler + `config/plotjuggler_sim.xml` (needs apt package) |

Examples:

```zsh
ros2 run mongla_sim_bringup mongla_sim stop
ros2 run mongla_sim_bringup mongla_sim sim
ros2 run mongla_sim_bringup mongla_sim sim --headless course:=pool_empty
ros2 run mongla_sim_bringup mongla_sim stack --no-vision
ros2 run mongla_sim_bringup mongla_sim plotjuggler
ros2 run mongla_sim_bringup mongla_sim smoke
ros2 run mongla_sim_bringup mongla_sim lab
```

---

## `ros2 launch`

### `mongla_sim_bringup sim.launch.py`

| Arg | Default | Meaning |
|-----|---------|---------|
| `course` | `sauvc26_qualification` | World stem under `mongla_sim_worlds/worlds/<course>.world` |
| `vehicle_name` | `mongla` | Must match course YAML |
| `ardusub` | `true` | Run ArduSub SITL |
| `bridge` | `true` | Camera + GT bridge (+ FX) |
| `gui` | `true` | Separate `gz sim -g` client |
| `mavproxy` | `false` | MAVProxy on UDP 14551 |
| `ardusub_params` | `mongla_sub.parm` | Overlay under bringup `config/` |
| `home` | `22.4820,89.5860,0.0,0` | lat,lon,alt,heading |
| `verbose` | `2` | Gazebo `-v` |

```zsh
ros2 launch mongla_sim_bringup sim.launch.py course:=sauvc26_final gui:=false
```

### `mongla_sim_bringup stack.launch.py`

Hard-wires manager: `mode:=sim`, `flight_controller:=pixhawk`, `yaw_source:=mavlink_ahrs`, `dvl_auto_connect:=false`, manager `vision:=false`.

| Arg | Default | Meaning |
|-----|---------|---------|
| `vision` | `true` | Launch `mongla_vision` on sim front cam |
| `image_topic` | `/mongla/sim/front_camera/image_raw` | Vision input |
| `model` / `models` / `active_model` / `classes` / `viewer` | gate defaults | Passed to `vision.launch.py` |

Vision uses `camera:=forward` so detector name matches missions.

```zsh
ros2 launch mongla_sim_bringup stack.launch.py vision:=false
```

### Other launches

```zsh
ros2 launch mongla_sim_bridge bridge.launch.py
ros2 launch mongla_sim_web lab.launch.py
```

---

## `ros2 run` console scripts

### `mongla_sim_bridge`

| Executable | Purpose |
|------------|---------|
| `contract_check` | ≥5 msgs on front/bottom `image_raw` + `camera_info`; note GT |
| `mavlink_check` | HEARTBEAT/ATTITUDE/AHRS2/… rates on `udpin:0.0.0.0:14550` |
| `record_cameras` | Dataset recorder (see flags below) |
| `underwater_fx` | Standalone FX node (normally via bridge launch) |

```zsh
ros2 run mongla_sim_bridge contract_check
ros2 run mongla_sim_bridge mavlink_check
ros2 run mongla_sim_bridge record_cameras --duration 15 --fx --frames --labels
ros2 run mongla_sim_bridge record_cameras \
  --cameras front --label gate_approach --course sauvc26_qualification \
  --fx --frames --labels
```

**`record_cameras` flags**

| Flag | Default | Notes |
|------|---------|-------|
| `--duration` | `0` | Seconds; `0` = until SIGINT |
| `--fps` | `20` | Hint only; MP4 uses measured `fps_actual` |
| `--frames` | off | Dump PNG frames (async queue) |
| `--labels` | off | YOLO GT from course props (async queue) |
| `--fx` | off | Subscribe `image_fx` instead of raw |
| `--cameras` | `front,bottom` | Comma list |
| `--course` | `sauvc26_qualification` | For labels + meta |
| `--label` | course name | Output dir prefix |
| `--outdir` | `<ws>/datasets` | Parent directory |
| `--script-id` | empty | Recorded in meta |

Ready/stop markers: prints `recording <dir>`, writes `.ready`, on exit prints `wrote <dir>` + `meta.json` with `fps_actual`.
MP4 playback duration ≈ wall `duration_s` (see [DATASETS.md](DATASETS.md)).

### `mongla_sim_scenarios`

| Executable | Purpose |
|------------|---------|
| `prop_manager` | Services under `/mongla/sim/props/*` |
| `props` | CLI: `list` / `add` / `move` / `remove` |
| `gate_transit_check` | Score GT passage through gate |

```zsh
ros2 run mongla_sim_scenarios prop_manager --ros-args -p world:=sauvc26_qualification
ros2 run mongla_sim_scenarios props list
ros2 run mongla_sim_scenarios props add sauvc_qual_gate gate_a 0 0 --z -1.5 --yaw 0.1
ros2 run mongla_sim_scenarios props move gate_a 1 0 --z -1.5 --yaw 0
ros2 run mongla_sim_scenarios props remove gate_a
```

Note: `props list` lists **catalog models**, not live spawned instance names.
Lab tracks instances via `GET /api/props/instances` — see [WORLD_EDITING.md](WORLD_EDITING.md).

### PlotJuggler

```zsh
sudo apt install ros-humble-plotjuggler-ros   # once
ros2 run mongla_sim_bringup mongla_sim plotjuggler   # alias: pj
```

Layout: `mongla_sim_bringup/config/plotjuggler_sim.xml`. Details: [PLOTJUGGLER.md](PLOTJUGGLER.md).

### `mongla_sim_web`

```zsh
ros2 run mongla_sim_web lab_server
# or: ros2 launch mongla_sim_web lab.launch.py
```

---

## Autonomy smoke (from `mongla_ws`)

```zsh
ros2 topic echo /mongla/state --once
ros2 run mongla_planner mongla arm
ros2 run mongla_planner mongla disarm
ros2 run mongla_planner mongla set_depth --target -1.0
ros2 run mongla_planner mongla move_forward --duration 8 --gain 60
ros2 run mongla_planner mission gate_prequal
```

---

## Environment variables

| Variable | Default / behavior |
|----------|-------------------|
| `MONGLA_WS` | Sibling `../mongla_ws`; required for `stack` / `smoke`; lab arm/disarm |
| `MONGLA_LAB_PORT` | `28765`; server tries `preferred … preferred+49` if busy |
| `MONGLA_LAB_HOST` | `0.0.0.0` |
| `MONGLA_TELEOP_ENDPOINT` | `tcp:127.0.0.1:5763` |
| `MONGLA_SIM_SCRIPTS` | Override lab move-script YAML directory |
| `GZ_IP` | Prefer `127.0.0.1` (gz-transport discovery) |
| `GZ_SIM_SYSTEM_PLUGIN_PATH` | Prepended with ArduPilot Gazebo plugin `build/` |
| `GZ_SIM_RESOURCE_PATH` | Package hooks for models |
| `DISPLAY` | Default `:0` in helper |
| `XAUTHORITY` | Auto mutter Xwayland cookie if unset |
| `HOST_UID` | For `/run/user/$HOST_UID/.mutter-Xwaylandauth.*` |
| `ARDUPILOT_ROOT` | Dir containing `build/sitl/bin/ardusub` |
| `ARDUPILOT_GAZEBO_ROOT` | Dir containing `build/libArduPilotPlugin.so` |

---

## Ports / endpoints

| Port | Protocol | Owner / use |
|------|----------|-------------|
| **14550** | UDP | ArduSub → autonomy (`udpclient`); manager `udpin:0.0.0.0:14550` |
| **14551** | UDP | GCS / MAVProxy secondary |
| **5763** | TCP | Lab teleop `RC_CHANNELS_OVERRIDE` (pymavlink) |
| **9002** | JSON FDM | ArduSub ↔ Gazebo ArduPilot plugin |
| **28765** | HTTP | Operator lab (`lab_server`) |

---

## Config / asset paths

| Path | Role |
|------|------|
| `src/mongla_sim_worlds/courses/*.yaml` | Course definitions |
| `src/mongla_sim_worlds/worlds/*.world` | Gazebo worlds |
| `src/mongla_sim_worlds/spec/arena.yaml` | Arena geometry SoT |
| `src/mongla_sim_bringup/config/mongla_sub.parm` | SITL param overlay |
| `src/mongla_sim_bridge/config/underwater_fx.yaml` | FX defaults (turbidity 0.45 …) |
| `src/mongla_sim_web/scripts/*.yaml` | Lab move scripts |
| `datasets/` | Recorded runs |

### Courses

| Course id | Typical use |
|-----------|-------------|
| `sauvc26_qualification` | Default; start zone + qual gate |
| `sauvc26_final` | Full final arena |
| `pool_empty` | Bare pool / hydro |

---

## Lab HTTP API (summary)

See [LAB_API.md](LAB_API.md) for bodies and semantics.

| Method | Path |
|--------|------|
| GET | `/api/health`, `/api/sim/status`, `/api/course`, `/api/vehicle/state` |
| POST | `/api/sim/start`, `/api/sim/restart`, `/api/sim/stop` |
| POST | `/api/vehicle/arm`, `/disarm`, `/cmd`, `/teleop` |
| GET/POST | `/api/fx` |
| GET | `/api/cameras/{front\|bottom}/mjpeg`, `/jpeg` |
| POST | `/api/record/start`, `/api/record/stop` |
| GET | `/api/record/status`, `/api/scripts`, `/api/scripts/status` |
| POST | `/api/scripts/run` |
| GET | `/api/props/catalog`, `/api/props/list` |
| POST | `/api/props/spawn`, `/api/props/remove/{name}` |
| GET | `/api/datasets`, `/api/datasets/{id}/zip` |

---

## Asset regeneration

```zsh
# from workspace root, with requirements.txt installed for regen
src/mongla_sim_description/scripts/generate_model.py \
  src/mongla_sim_description/models/mongla_heavy/model.sdf.in \
  src/mongla_sim_description/models/mongla_heavy/model.sdf \
  src/mongla_sim_description/models/mongla_heavy/configs.yaml
src/mongla_sim_worlds/scripts/gen_world.py --all
# list courses: gen_world.py --list
```

## Frontend rebuild (lab UI)

```zsh
cd src/mongla_sim_web/frontend
npm install
npm run build    # → ../static/
cd ../../..
colcon build --packages-select mongla_sim_web --symlink-install
```

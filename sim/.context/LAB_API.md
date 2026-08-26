# Lab HTTP API

Server: `src/duburi_sim_web/duburi_sim_web/server.py` (`lab_server`).
Default bind: `DUBURI_LAB_HOST`:`DUBURI_LAB_PORT` (28765, auto-bump if busy).

Static UI from package `static/` (built from `frontend/`).

## Health / status

### `GET /api/health`

Returns teleop snapshot + sim job snapshot (`ok`, `teleop`, `sim`).

### `GET /api/sim/status`

Rich status for the Operate panel:

- Process flags: `gz`, `ardusub`, `manager`, `lab_ros`
- `teleop`, `restart` / `sim` job (`running`, `phase`, `course`, `active_course`, `error`, `log_tail`)
- `link`: `{gz, sitl, mav, cams, teleop}`
- `state`: armed, mode, yaw, depth, battery (from manager/lab node)
- `ground_truth`: x,y,z
- `fx`, `cameras`, `use_fx`, `active_course`

Phases include: `idle`, `queued`, `stopping`, `starting_sim`, `waiting_ready`,
`prop_manager`, `starting_stack`, `ready`, `stopped`, `error`.

## World / sim control

### `POST /api/sim/start` body `{course, gui, stack}`

Start if sim down. 409 if already running (use restart).

### `POST /api/sim/restart` same body

Stop → start. Polls gz+ardusub up to ~90s, then restarts `prop_manager` with
`-p world:={course}`, optionally restacks.

### `POST /api/sim/stop`

Stop sim job (and related bringup).

### `GET /api/course`

`{courses: [...], active_course, note}` — note reminds that course switch is restart.

**Semantics:** not Gazebo hot-reload. UI labels it “restart / switch”.

## Vehicle

| Method | Path | Notes |
|--------|------|-------|
| GET | `/api/vehicle/state` | Snapshot |
| POST | `/api/vehicle/arm` | `duburi_planner duburi arm` via `DUBURI_WS` |
| POST | `/api/vehicle/disarm` | same |
| POST | `/api/vehicle/cmd` | `{cmd, duration?, gain?, target?, timeout?}` |
| POST | `/api/vehicle/teleop` | `{fwd,lat,up,yaw,gain?}` — axes −1…1 |
| GET | `/api/vehicle/teleop` | TeleopStreamer status |

Teleop endpoint: `DUBURI_TELEOP_ENDPOINT` default `tcp:127.0.0.1:5763`.

## FX / cameras

| Method | Path |
|--------|------|
| GET/POST | `/api/fx` — turbidity, backscatter, blur_sigma, noise, vignette, enabled, use_fx_feed |
| GET | `/api/cameras/{front\|bottom}/mjpeg` multipart stream |
| GET | `/api/cameras/{front\|bottom}/jpeg` single frame |

Turbidity may be set **>1** (UI slider 0–2). Defaults in `underwater_fx.yaml` start at 0.45.

## Record

### `POST /api/record/start`

```json
{
  "name": "gate_approach",
  "cameras": ["front", "bottom"],
  "fx": true,
  "frames": true,
  "labels": true,
  "duration": 0,
  "course": ""
}
```

Empty `course` → lab `active_course`. Spawns `record_cameras` in a new process group.

### `POST /api/record/stop`

Waits for `.ready` marker (up to 45s) before SIGINT process group; parses `wrote …`
or falls back to newest matching dataset dir; waits for `meta.json`.

Returns `{ok, record_dir, stdout, code}`. UI derives zip id from `record_dir` basename.

### `GET /api/record/status`

`running` + last meta (label, course, cameras, …).

## Scripts

| Method | Path |
|--------|------|
| GET | `/api/scripts` |
| POST | `/api/scripts/run` `{script_id, label?, use_fx?, frames?, labels?}` |
| GET | `/api/scripts/status` |

YAMLs under `src/duburi_sim_web/scripts/` (or `DUBURI_SIM_SCRIPTS`).

## Props / assets

| Method | Path | Notes |
|--------|------|-------|
| GET | `/api/props/catalog` | Library + custom models `{id, anchor}` |
| GET | `/api/props/list` | Legacy CLI catalog dump — prefer `catalog` |
| GET | `/api/props/instances` | Names spawned via lab this session |
| POST | `/api/props/spawn` | `{model, name, x, y, z?, yaw?}` |
| POST | `/api/props/move` | `{name, x, y, z?, yaw?}` |
| POST | `/api/props/remove/{name}` | Also drops from instance tracker |
| POST | `/api/assets/upload` | Multipart zip with `model.sdf` → `models/<id>/` |

Requires `prop_manager` with correct `world:=` (lab restart syncs this).
Custom models may need a sim restart so Gazebo sees `GZ_SIM_RESOURCE_PATH`.
See [WORLD_EDITING.md](WORLD_EDITING.md).

## Datasets

| Method | Path |
|--------|------|
| GET | `/api/datasets` | Newest-first by `meta.utc_start` (skips dirs without `meta.json`) |
| GET | `/api/datasets/{run_id}/zip` | Streaming zip of run directory |

Zip of large runs is synchronous — can block the event loop for many seconds.

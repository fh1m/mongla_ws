# Handoff — Cursor agent → next Claude / duburi_ws agent

> ℹ **Absorbed 2026-08-27.** This workspace is no longer the sibling tree
> `Ros_workspaces/duburi-sim_ws`; it lives inside the `duburi_ws` repo at
> `duburi_ws/sim/` and is under version control. Paths below have been
> updated; any remaining "sibling" phrasing is historical.

**Origin:** built 2026-08-26; absorbed into the `duburi_ws` repo 2026-08-27.  
**Audience:** next Claude Code / Cursor agent continuing sim work, and the
`duburi_ws` agent integrating autonomy against this sibling.

## What this repo is

A **sibling** colcon workspace that provides:

1. Gazebo Harmonic pool + Duburi-class AUV (BlueROV2 Heavy proxy)  
2. ArduSub SITL (JSON FDM)  
3. ros_gz camera/GT bridge + underwater FX  
4. Operator web lab (teleop, World props, dataset record)  
5. Drop-in MAVLink/camera **contract** so `duburi_ws` missions run unmodified  

Autonomy code stays in `../duburi_ws`. **Do not merge packages** until the user
asks — recipe only in [FUTURE_MERGE.md](FUTURE_MERGE.md).

## Git status (important)

`duburi_ws/sim` is **under git**, inside the `duburi_ws` repo (2026-08-27). It previously had none; see [FUTURE_MERGE.md](FUTURE_MERGE.md) for the decision.

- The `duburi_ws` agent will initialize / submodule / subtree when integrating.  
- Land changes **here**; `fh1m/duburi-sim_ws` is a published mirror, not the source.  
- Tracked-by-docs ignore list: root [`.gitignore`](../.gitignore) (`build/`,
  `install/`, `datasets/`, `node_modules/`, …).

Open PR on autonomy side (docs only):  
https://github.com/fh1m/duburi_ws/pull/8 — branch `docs/duburi-sim-sibling-pointer`.

---

## What we built (and why)

### Product split

| Layer | Owner | Why |
|-------|-------|-----|
| Physics + SITL + cams | `duburi_ws/sim` | Gazebo/ArduPilot deps stay off every autonomy checkout |
| Manager / planner / vision / missions | `duburi_ws` | Same binaries as the pool vehicle |
| Operator lab | `duburi_sim_web` | FastAPI+React local tool, not embedded PlotJuggler/Qt |

### Design decisions (pros / cons)

| Decision | Pros | Cons | Keep? |
|----------|------|------|-------|
| Sibling workspace (not in-tree) | Clear boundary; independent colcon | Dual `source install`; newcomers miss overlay | **Yes** until FUTURE_MERGE |
| Teleop RC on TCP **5763**, manager on UDP **14550** | No GCS/manager fight | Easy to miswire a second GCS to 14550 | **Yes** |
| Course switch = stop→start (~90s) | Correct Gazebo world; reliable | Operators expect hot-reload | **Yes** v0.2 |
| FX on `image_fx`, raw stays clean | Contract + training can diverge | Must check `--fx` when recording | **Yes** |
| MP4 encode at `fps_actual` | Wall-clock playback matches take | Lower fps when frames+labels on | **Yes** (was broken before) |
| Lab preview prefers **raw** | Smooth MJPEG under turbidity | FX not visible until record checkbox | Documented |
| PlotJuggler desktop, not in browser | Real timeseries tool | Extra apt package | **Yes** |
| `flight_controller:=pixhawk` in `stack` | Required for `srot` + ArduSub SITL | Surprising if someone expects USB SROT in sim | **Yes** |
| Hold lab listen socket (`uvicorn fd=`) | Beats Cursor/Electron port steal | Slightly unusual bind path | **Yes** |
| No git in this tree yet | Avoid premature history before merge plan | No PR review on sim itself | Until user asks |

### Major fixes shipped (v0.2 freedom suite)

1. **MP4 ~1s bug** — buffered frames; encode with `count/duration_s`; async PNG/labels.  
2. **MJPEG** — ~33 ms poll, JPEG ~82, skip stale frames, raw preview default.  
3. **World freedom** — move/instances/yaw, custom model zip upload, catalog refresh.  
4. **PlotJuggler** — `duburi_sim plotjuggler` + layout XML + docs.  
5. **Stop orphans** — `duburi_sim stop` also kills lab / recorder / bridges / prop_manager.  
6. **Turbidity** UI default aligned to **0.45**.  
7. **Zip** written to tempfile then `FileResponse` (less RAM spike).  
8. **Record stop** returns `meta`; UI warns on `counts==0`.  
9. Deep `.context/` encyclopedia + operator README path to missions.

---

## Codebase structure (health)

```text
duburi_ws/sim/
  README.md              # operator cold-start → mission
  AGENTS.md CLAUDE.md    # agent entry
  .context/              # canonical docs (INDEX first)
  src/
    duburi_sim_description/
    duburi_sim_worlds/
    duburi_sim_bringup/   # duburi_sim CLI, sim/stack launch
    duburi_sim_bridge/    # cams, FX, record_cameras, checks
    duburi_sim_scenarios/ # props
    duburi_sim_web/       # lab API + frontend/ → static/
  datasets/              # gitignored runtime clips
  build/ install/ log/   # colcon; gitignored
```

Structure is **OK** for sibling packaging. Do not rename packages or topics
without bumping [CONTRACT.md](CONTRACT.md).

---

## Anticipated next work (priority)

For **sim Claude agent**:

1. Optional: gz entity query for baked-in props (instances today = lab-tracked).  
2. Lab MCAP bag button (roadmap [TESTING_SUITE.md](TESTING_SUITE.md)).  
3. Gate transit score overlay in Operate.  
4. Turbidity schedule presets / domain randomization.  
5. Pytest smoke for lab APIs (AUDIT D3).  
6. Done 2026-08-27: absorbed into `duburi_ws` as `sim/` (see FUTURE_MERGE).

For **`duburi_ws` Claude agent**:

1. Merge/review [PR #8](https://github.com/fh1m/duburi_ws/pull/8).  
2. Run missions against sibling sim; keep vision remap on `forward`.  
3. Done: FUTURE_MERGE executed as a nested workspace; this tree is versioned
   or absorb it — do not invent a third packaging story.  
4. Train/handoff from `datasets/` clips stays in vision docs here.

---

## How to continue in one screen

```zsh
# read
less ~/Ros_workspaces/duburi_ws/sim/.context/INDEX.md
less ~/Ros_workspaces/duburi_ws/sim/.context/AUDIT.md

# bring up
ros2 run duburi_sim_bringup duburi_sim stop
ros2 run duburi_sim_bringup duburi_sim sim
ros2 run duburi_sim_bringup duburi_sim stack --no-vision
ros2 run duburi_sim_bridge contract_check

# after UI edits
cd src/duburi_sim_web/frontend && npm run build
colcon build --packages-select duburi_sim_web --symlink-install
```

Hard rules: one sim; prefer this `.context/` over `duburi_ws` `sim-setup.md`;
never put teleop on 14550; never `git add -A` (the repo has protected paths); never edit
`.cursor/plans/*.plan.md` unless asked.

## Voice for the next agent

You are taking over a **working** Mongla Gazebo lab built by the Cursor agent.
Prefer surgical fixes (AUDIT open items) over re-architecture. Prove claims with
[TESTING.md](TESTING.md). When talking to the `duburi_ws` agent, point them at
this file + [INTEGRATION_DUBURI_WS.md](INTEGRATION_DUBURI_WS.md) + PR #8.

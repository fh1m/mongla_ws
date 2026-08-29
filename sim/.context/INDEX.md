# `duburi_ws/sim` — agent / operator index

> ℹ **Absorbed 2026-08-27.** This workspace is no longer the sibling tree
> `Ros_workspaces/duburi-sim_ws`; it lives inside the `duburi_ws` repo at
> `duburi_ws/sim/` and is under version control. Paths below have been
> updated; any remaining "sibling" phrasing is historical.

**Read this first.** Gazebo Harmonic + ArduSub SITL for Mongla AUV (Duburi 4.5).
Nested inside [`duburi_ws`](../..); autonomy lives in [`../../src`](../../src). MAVLink/camera surface so autonomy
does not need sim-specific forks.

## 5-minute orientation

| Fact | Detail |
|------|--------|
| Workspace root | `Ros_workspaces/duburi_ws/sim` |
| Autonomy sibling | `Ros_workspaces/duburi_ws` (`DUBURI_WS`) |
| Helper CLI | `ros2 run duburi_sim_bringup duburi_sim <cmd>` |
| Default course | `sauvc26_qualification` (AUV at x ≈ −11.8) |
| MAVLink autonomy | UDP **14550** (manager binds `udpin`) |
| Lab teleop RC | TCP **5763** (does not fight 14550) |
| Operator UI | HTTP **28765** (`DUBURI_LAB_PORT`, auto-fallback) |
| Cameras | `/duburi/sim/{front,bottom}_camera/image_raw` 640×480 |
| One-sim rule | Always `duburi_sim stop` before a new `sim` |

Canonical bring-up:

```zsh
duburi_sim stop → duburi_sim sim → duburi_sim stack --no-vision → duburi_sim smoke
# optional: duburi_sim lab  →  http://localhost:28765
```

## Doc map (where to go)

| Doc | Audience | Use when |
|-----|----------|----------|
| [QUICKSTART.md](QUICKSTART.md) | Anyone | Copy-paste first session |
| [HANDOFF.md](HANDOFF.md) | Next agent / duburi_ws agent | What Cursor built, why, packaging, next work |
| [OPERATOR.md](OPERATOR.md) | Operators | Full stack, courses, lab, failure modes |
| [COMMAND_REFERENCE.md](COMMAND_REFERENCE.md) | Everyone | Exact CLI / env / ports / launch args |
| [ARCHITECTURE.md](ARCHITECTURE.md) | Devs / agents | Process graph, packages, data flow |
| [CONTRACT.md](CONTRACT.md) | Autonomy integrators | Topics / ports / sizes that must not drift |
| [INTEGRATION_DUBURI_WS.md](INTEGRATION_DUBURI_WS.md) | `duburi_ws` devs | `srot` vs `main`, pixhawk, vision remap |
| [LAB_API.md](LAB_API.md) | Lab / UI | FastAPI routes, World restart semantics |
| [PHYSICS.md](PHYSICS.md) | **Physics** | **★ T200 thrust curve from real Blue Robotics data, water current, RTF, what is validated and what is not** |
| [VISION_AND_DATASETS.md](VISION_AND_DATASETS.md) | **Vision / data** | **★ Every camera/model combination, all 13 courses, making new courses+props, Gazebo-labelled and hand-labelled datasets, OpenCV, running missions** |
| [ROBOSUB_AND_ACOUSTICS.md](ROBOSUB_AND_ACOUSTICS.md) | **Competitions** | **★ RoboSub 2026 props/courses, the two-competition arena spec, and the hydrophone both competitions need** |
| [SIM_VISION_TRAINING.md](SIM_VISION_TRAINING.md) | **Vision / data** | **★ Terminal by terminal: sim -> vision pipeline -> auto-labelled dataset -> trained sim-native model -> vision verbs back in sim** |
| [DATASETS.md](DATASETS.md) | Vision / data | `record_cameras`, meta.json, YOLO labels |
| [DEVELOPMENT.md](DEVELOPMENT.md) | Devs / agents | Build, frontend, regen, agent rules |
| [TESTING.md](TESTING.md) | QA | contract/mavlink/smoke/lab checklists |
| [TROUBLESHOOTING.md](TROUBLESHOOTING.md) | Operators | Lost control, black cams, orphans |
| [DVL_AND_SONAR.md](DVL_AND_SONAR.md) | Native Gazebo DVL: how it is wired, the four traps, and why sonar is not available |
| [DAVE_AND_VEHICLES.md](DAVE_AND_VEHICLES.md) | Project DAVE assessment (Jazzy-only, do not adopt) + other AUV models |
| [WORLD_EDITING.md](WORLD_EDITING.md) | Operators | Prop spawn/move, custom assets, limits |
| [PLOTJUGGLER.md](PLOTJUGGLER.md) | Operators | Desktop timeseries vs Foxglove |
| [TESTING_SUITE.md](TESTING_SUITE.md) | Maintainers | Lab amenity roadmap |
| [DUBURI_WS_PR.md](DUBURI_WS_PR.md) | Maintainers | What the autonomy integration PR contains |
| [AUDIT.md](AUDIT.md) | Maintainers | Known bugs, design debt, priorities |
| [FUTURE_MERGE.md](FUTURE_MERGE.md) | Maintainers | Sibling → `duburi_ws` / separate repo |
| [CODEMAP.md](CODEMAP.md) | Agents | Path → responsibility |

## Hard rules for future agents

1. **Do not** start a second sim or second manager on UDP 14550.
2. On `duburi_ws` **`srot`**, stack must use `flight_controller:=pixhawk` (already set by `stack.launch.py`).
3. Prefer this `.context/` over `duburi_ws/.claude/context/sim-setup.md` (legacy BlueROV/`colcon_ws`).
4. Lab “course switch” = stop → start (not Gazebo hot-reload).
5. Verify claims with commands in [TESTING.md](TESTING.md) before asserting green.
6. Do not edit Cursor `.cursor/plans/*.plan.md` unless the user asks.
7. This tree is **under git** inside `duburi_ws` (2026-08-27). Land changes here; `fh1m/duburi-sim_ws` mirrors it. Stage by path — never `git add -A`
   (see [FUTURE_MERGE.md](FUTURE_MERGE.md), [HANDOFF.md](HANDOFF.md)).

## Packages (one line)

| Package | Role |
|---------|------|
| `duburi_sim_description` | Vehicle SDF / hydro configs |
| `duburi_sim_worlds` | Pool, props, course YAML → `.world` |
| `duburi_sim_bringup` | `sim` / `stack` / `duburi_sim` helper |
| `duburi_sim_bridge` | ros_gz cams/GT, FX, recorder, checks |
| `duburi_sim_scenarios` | Runtime prop spawn/move/delete |
| `duburi_sim_web` | FastAPI + React operator lab |

- [`JOYSTICK.md`](JOYSTICK.md) — fly the AUV with a gamepad, from the
  browser or the lab host; mapping, feel, and the watchdog trap.
- [`SCORING.md`](SCORING.md) — style points, the coin flip, gate side and
  the SAUVC flare sequence, scored from ground truth.
- [`BNO085.md`](BNO085.md) — virtual BNO085 board, so `yaw_source:=bno085`
  (what the vehicle flies) runs in sim; datasheet-grounded drift.
- [`FAULTS.md`](FAULTS.md) — inject a DVL dropout, camera loss, MAVLink
  loss, battery sag or a dead thruster, so the recovery paths run.
- [`PAYLOAD.md`](PAYLOAD.md) — virtual payload board: `fire()` and the
  mid-hold `align(fire=…)` shot in sim, through a PTY the real driver opens.

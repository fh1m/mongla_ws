# What the `duburi_ws` integration PR contains

The autonomy-side PR is **docs + pointers only**. Sim packages stay in sibling
`duburi-sim_ws` (see [FUTURE_MERGE.md](FUTURE_MERGE.md)).

**Live PR:** https://github.com/fh1m/duburi_ws/pull/8  
**Branch:** `docs/duburi-sim-sibling-pointer`

Built by the **Cursor agent**; intended reader is the next **Claude / duburi_ws agent**.
Full narrative: [HANDOFF.md](HANDOFF.md).

## Files typically touched on `duburi_ws`

| Path | Change |
|------|--------|
| `.claude/context/duburi-sim.md` | Canonical pointer + packaging (no git on sim) + handoff link |
| `CLAUDE.md` | Prefer sibling sim `.context` over legacy `sim-setup.md` |
| `README.md` | “Drive in Mongla Gazebo lab” section: `duburi_sim` + pixhawk |
| `.claude/context/sim-setup.md` | Banner: legacy BlueROV path; redirect for Mongla SITL |

## Content the PR must document

1. Sibling layout: `Ros_workspaces/duburi-sim_ws` next to `duburi_ws`
2. **`duburi-sim_ws` has no `.git`** — autonomy agent initializes when merging
3. On branch **`srot`**, stack uses `flight_controller:=pixhawk` (SITL is ArduSub)
4. Helper: `ros2 run duburi_sim_bringup duburi_sim {stop,sim,stack,smoke,lab,plotjuggler}`
5. Contract: MAVLink UDP **14550**, cams `/duburi/sim/{front,bottom}_camera/image_raw` 640×480
6. Lab URL: `http://localhost:${DUBURI_LAB_PORT:-28765}` (teleop RC on TCP **5763**)
7. PlotJuggler + dataset loop + mission CLI path

## Not in that PR

- Moving `duburi_sim_*` packages into `duburi_ws`
- Changing autonomy control code for sim
- Embedding PlotJuggler in the web UI
- Creating a sim GitHub remote

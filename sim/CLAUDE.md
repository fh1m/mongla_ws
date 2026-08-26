# CLAUDE.md — Duburi simulator workspace

You are working in **`duburi-sim_ws`** (Gazebo Harmonic + ArduSub SITL + operator lab).

## Mandatory first read

1. [`.context/INDEX.md`](.context/INDEX.md) — orientation + hard rules  
2. [`.context/HANDOFF.md`](.context/HANDOFF.md) — prior Cursor work, design, next tasks  
3. [`.context/CODEMAP.md`](.context/CODEMAP.md) — where code lives  
4. Task-specific doc from the INDEX table (OPERATOR / LAB_API / CONTRACT / …)

## Hard rules

- **One sim only.** `ros2 run duburi_sim_bringup duburi_sim stop` before a new `sim`.
- Sibling autonomy: `../duburi_ws`. On branch **`srot`**, stack must use
  `flight_controller:=pixhawk` (`stack.launch.py` already does).
- Prefer **this** `.context/` over `duburi_ws/.claude/context/sim-setup.md` (legacy).
- Lab “switch course” = stop → start, not Gazebo hot-reload.
- Do **not** edit Cursor `.cursor/plans/*.plan.md` unless the user asks.
- Do **not** `git init` this tree unless the user asks (packaging owned by
  `duburi_ws` agent — [`.context/FUTURE_MERGE.md`](.context/FUTURE_MERGE.md)).
- Verify before claiming green — see [`.context/TESTING.md`](.context/TESTING.md).
- Known issues: [`.context/AUDIT.md`](.context/AUDIT.md).

## Canonical commands

```zsh
duburi_sim stop | sim | stack --no-vision | smoke | lab | plotjuggler
ros2 run duburi_sim_bridge contract_check
ros2 run duburi_sim_bridge mavlink_check
```

Full encyclopedia: [`.context/COMMAND_REFERENCE.md`](.context/COMMAND_REFERENCE.md).
Operator cold-start → mission: root [README.md](README.md).

## Packaging stance (v0.2)

Keep sibling workspace. **No git remote here yet.** Merge/submodule recipe only in
[`.context/FUTURE_MERGE.md`](.context/FUTURE_MERGE.md) — do not move packages
unless the user explicitly requests it.

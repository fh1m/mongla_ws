# CLAUDE.md — Duburi simulator workspace

You are working in **`duburi_ws/sim`** (Gazebo Harmonic + ArduSub SITL + operator lab)
— a nested colcon workspace inside the `duburi_ws` repo, not a sibling tree.

## Mandatory first read

1. [`.context/INDEX.md`](.context/INDEX.md) — orientation + hard rules  
2. [`.context/HANDOFF.md`](.context/HANDOFF.md) — how the lab was built, design, next tasks  
3. [`.context/CODEMAP.md`](.context/CODEMAP.md) — where code lives  
4. Task-specific doc from the INDEX table (OPERATOR / LAB_API / CONTRACT / …)

## Hard rules

- **One sim only.** `ros2 run duburi_sim_bringup duburi_sim stop` before a new `sim`.
- Autonomy is the **parent** workspace: `../src`. Build autonomy first
  (`../build_dubomini.sh`), then `./build_sim.sh` here — never a bare
  `colcon build`, which self-ignores via `COLCON_IGNORE`.
- `flight_controller:=pixhawk` is **required on `srot`, inert on `main`**
  (`stack.launch.py` passes it either way; extra launch args are silently
  dropped, so neither branch complains).
- Prefer **this** `.context/` over `duburi_ws/.claude/context/sim-setup.md` (legacy).
- Lab “switch course” = stop → start, not Gazebo hot-reload.
- **This tree is under git** (inside `duburi_ws`, 2026-08-27). The old
  "do not `git init`" rule is retired — see
  [`.context/FUTURE_MERGE.md`](.context/FUTURE_MERGE.md) for what was chosen.
- Changes land **here**; `fh1m/duburi-sim_ws` is a published mirror, not the source.
- Never stage the repo's protected paths — `git add` by path, never `-A`.
- Verify before claiming green — see [`.context/TESTING.md`](.context/TESTING.md).
- Known issues: [`.context/AUDIT.md`](.context/AUDIT.md).

## Canonical commands

```bash
duburi_sim stop | sim | stack --no-vision | smoke | lab | plotjuggler
ros2 run duburi_sim_bridge contract_check
ros2 run duburi_sim_bridge mavlink_check
```

Full encyclopedia: [`.context/COMMAND_REFERENCE.md`](.context/COMMAND_REFERENCE.md).
Operator cold-start → mission: root [README.md](README.md).

## Packaging stance (settled 2026-08-27)

Absorbed into `duburi_ws` as the nested workspace `sim/`. **Not** a submodule and
**not** a subtree: `sim/` is committed as plain files, so there is no pin and no
second push target to keep in sync. `duburi_ws/sim/` is canonical;
[`fh1m/duburi-sim_ws`](https://github.com/fh1m/duburi-sim_ws) mirrors it for
sim-only work. Do not move packages into `../src` — the root build must stay at
exactly six autonomy packages.

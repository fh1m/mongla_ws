# AGENTS.md — agent entry for `duburi_ws/sim`

## Purpose

Gazebo Harmonic + ArduSub SITL simulator and operator lab for Mongla / Duburi,
a nested colcon workspace inside the `duburi_ws` repo (autonomy lives in `../src`).

## Read first

| Order | File |
|-------|------|
| 1 | [`.context/INDEX.md`](.context/INDEX.md) |
| 2 | [`.context/HANDOFF.md`](.context/HANDOFF.md) — how the lab was built + next steps |
| 3 | [`.context/CODEMAP.md`](.context/CODEMAP.md) |
| 4 | [`.context/COMMAND_REFERENCE.md`](.context/COMMAND_REFERENCE.md) |
| 5 | [`.context/AUDIT.md`](.context/AUDIT.md) before large changes |

## Do

- Follow one-sim rule; use `duburi_sim` helper.
- Build with `./build_sim.sh`, never a bare `colcon build` (it self-ignores).
- Keep MAVLink contract on UDP 14550; lab teleop on TCP 5763.
- Rebuild `src/duburi_sim_web/frontend` → `static/` after UI edits.
- Document new commands in `.context/COMMAND_REFERENCE.md`.
- Prefer surgical AUDIT fixes over re-architecture.

## Don't

- Start a second Gazebo/ArduSub/manager on the same ports.
- Point teleop or a second GCS at 14550 in a way that steals the manager.
- Treat `duburi_ws/.claude/context/sim-setup.md` as current.
- Move packages into `../src`. The root `colcon build` must stay at exactly six
  autonomy packages; `COLCON_IGNORE` here is what guarantees it.
- `git add -A`. Stage by path — the repo has protected files that must never be committed.

## Prove it

`contract_check`, `mavlink_check`, `duburi_sim smoke`, lab `/api/health` —
details in [`.context/TESTING.md`](.context/TESTING.md).

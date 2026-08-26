# AGENTS.md — agent entry for duburi-sim_ws

## Purpose

Gazebo Harmonic + ArduSub SITL simulator and operator lab for Mongla / Duburi,
drop-in against sibling `duburi_ws`.

## Read first

| Order | File |
|-------|------|
| 1 | [`.context/INDEX.md`](.context/INDEX.md) |
| 2 | [`.context/HANDOFF.md`](.context/HANDOFF.md) — prior Cursor agent work + next steps |
| 3 | [`.context/CODEMAP.md`](.context/CODEMAP.md) |
| 4 | [`.context/COMMAND_REFERENCE.md`](.context/COMMAND_REFERENCE.md) |
| 5 | [`.context/AUDIT.md`](.context/AUDIT.md) before large changes |

## Do

- Follow one-sim rule; use `duburi_sim` helper.
- Keep MAVLink contract on UDP 14550; lab teleop on TCP 5763.
- Rebuild `src/duburi_sim_web/frontend` → `static/` after UI edits.
- Document new commands in `.context/COMMAND_REFERENCE.md`.
- Prefer surgical AUDIT fixes over re-architecture.

## Don't

- Start a second Gazebo/ArduSub/manager on the same ports.
- Point teleop or a second GCS at 14550 in a way that steals the manager.
- Treat `duburi_ws/.claude/context/sim-setup.md` as current.
- **`git init` this workspace** or create a GitHub remote without an explicit user ask
  (`duburi_ws` agent owns packaging — [`.context/FUTURE_MERGE.md`](.context/FUTURE_MERGE.md)).
- Merge packages into `duburi_ws` without an explicit ask.
- Edit plan files under `.cursor/plans/` unless asked.

## Prove it

`contract_check`, `mavlink_check`, `duburi_sim smoke`, lab `/api/health` —
details in [`.context/TESTING.md`](.context/TESTING.md).

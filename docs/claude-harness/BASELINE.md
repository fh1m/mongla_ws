# Harness baseline — 2026-09-28

Phase A of the rebuild brief: **measure before changing**. Numbers here are the
state found on 2026-09-28, with the two fixes of `DECISIONS.md` already applied
where noted, so later passes have something to beat.

## Config topology, and the trap in it

⛔ **`$HOME` is not `/home/fh1m`.** It is
`/home/fh1m/Envs/dockers/auv-ros2`, so the live config tree is
`/home/fh1m/Envs/dockers/auv-ros2/.claude/` and
`/home/fh1m/.claude/` is a **stale August copy that nothing loads**. Its
`settings.json` still lists ~30 ECC hooks and reads as if it were live.

⚠ Anyone debugging this environment must check `$HOME` first. Reading the wrong
tree gives a confident, completely wrong picture — it cost the first twenty
minutes of this pass.

| | path |
|---|---|
| live global | `/home/fh1m/Envs/dockers/auv-ros2/.claude/` |
| **stale decoy** | `/home/fh1m/.claude/` — ignore |
| project | `Ros_workspaces/mongla_ws/.claude/` |

## Counts

| | before | after D1/D2 |
|---|---|---|
| registered hooks | **40** | **6** |
| — from disabled plugins | 34 | 0 |
| — project's own | 4 | 4 |
| enabled plugins | 7 | 7 |
| registered marketplaces | 15 | 12 |
| global `settings.json` hooks | 0 | 0 |

The 6 that remain: `block_install`, the graphify grep hint, `py_check`,
`pkg_test` (this repo) and 2 from `ros2-engineering-skills`.

## Enabled plugins, and what each is for

| plugin | keep? |
|---|---|
| `superpowers` | brainstorming / TDD / debugging disciplines — overlaps `/go` and `/prove`; **ablation candidate** |
| `context7` | library docs retrieval — a real external capability, no native equivalent |
| `security-guidance` | — |
| `frontend-design` | rarely relevant to this repo; **ablation candidate** |
| `claude-hud` | statusline / observability |
| `ros2-engineering` | domain skills for the actual work |
| `caveman` | output compression, user-chosen |

## Project instruction surface

| file | lines |
|---|---|
| `CLAUDE.md` | ~250 |
| `.claude/context/` | the book — progressive disclosure, loaded on demand |
| `measured-bars.md` | 104 entries, 802 rows |

⭐ The project side is already close to the shape the brief asks for: a compact
`CLAUDE.md` that points at on-demand context, rather than one giant always-on
file. The global side was the problem.

## What is NOT yet measured

⚠ Stated so the gaps are not mistaken for clean results:

* **fixed context overhead** — needs `/context all` output, which is a slash
  command the operator runs;
* **hook latency** — the 6 remaining hooks have never been timed;
* **startup latency**, before vs after removing 34 hooks;
* **whether the hooks actually stopped firing** — D1 is verified on disk, not
  across a restart.

## Next, in order

1. Confirm across a restart that no ECC/cco/last30days hook fires.
2. Time the 6 remaining hooks; any hook that can freeze a session gets a
   timeout or goes.
3. Ablate `superpowers` and `frontend-design` against real tasks from this
   repo, per the brief's configurations A–F.
4. Boris Cherny primary-source pass → `BORIS-CHERNY-PLAYBOOK.md`.

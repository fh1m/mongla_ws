---
name: go
description: Completion discipline for this repo — pick the strongest oracle, run it, iterate on the failure, then review, simplify and read the diff. Use when finishing a change, or when the prompt ends in "/go".
---

# /go

Append `/go` to a task and it means: **do not come back until evidence says it works.**

This skill is about *finishing*, not about how to implement. Choose the implementation
yourself.

## The loop

1. **Name the success criterion first**, in one sentence, before writing code. If you
   cannot name it, you do not understand the task yet — ask, or go read.
1b. **If the task is a MEASUREMENT, write the scorer before collecting the data,
   and write down what result would falsify the idea.** Measured the hard way:
   of six model-free vision candidates built in one session, the four with a
   pass/fail criterion written down first were each settled in one run; the two
   scored afterwards both produced a confident number that was wrong.
   ⛔ A quality number a method computes from the data it just fit is **not**
   evidence — a vanishing point scored 0.958 on its own metric while jittering
   20 227 px. Cross-check against a second, independent observation or against
   synthetic input whose answer you chose. `tools/tau_from_scale.py --self-test`
   is the pattern, and it corrected its own author before any water.
2. Implement.
3. **Pick the strongest oracle available** from the ladder below. Strongest, not
   cheapest.
4. Run it. Read the whole output, not the exit code alone.
5. Fail → diagnose with `superpowers:systematic-debugging` (root cause before fix), fix,
   **rerun from step 4**. Do not proceed on a fix you have not re-verified.
6. Pass → fresh-eyes review, `/simplify`, read the final diff end to end, rerun the
   oracle once more.
7. State what is still uncertain. There is almost always something.

Completion claims go through `superpowers:verification-before-completion`. Do not
restate its rules here — invoke it.

## The oracle ladder for this repo

Strongest first. Use the highest rung the change can reach.

| Rung | Oracle | When |
|---|---|---|
| 1 | Water / bench run with the board attached | only with a human on the kill switch; nothing here has been in water |
| 2 | `ros2 run mongla_manager bringup_check --srot` | anything touching the board, arming, or bring-up gating |
| 3 | `./build_mongla.sh` then `pytest` (293 test files) | any source change — this is the default rung |
| 4 | The sim: `sim/` with `flight_controller:=pixhawk` | control behaviour and verb semantics. **Not** detection thresholds or vision gains — sim imagery is too clean |
| 5 | Replay a recorded bag (`scripts/pool_record.sh`) | perception and tracking changes, against real footage |
| 6 | A targeted `pytest src/<pkg>/test/test_<thing>.py` | a single focused change |

### Guards that must stay green

Run these whenever the change could plausibly touch them:

- `src/mongla_vision/test/test_no_capability_is_built_and_unreachable.py` — the repo's
  oldest and most expensive defect class. Anything new must be imported by something
  that runs, or carry a written reason naming the consumer it waits for.
- `src/mongla_control/test/test_doc_drift.py` — fails on any doc stating a stale
  constant. Edited `CLAUDE.md` or `.claude/context/`? Run it.
- `python3 tools/gen_reference.py` — regenerates the command reference; a test fails if
  it drifts.
- `tools/orphan_sweep.py`, `tools/topic_wiring_sweep.py`, `tools/launch_param_sweep.py`
  — reachability, topic wiring, launch-parameter coverage.

### New thresholds

Any shipped constant goes in `.claude/context/measured-bars.md` with the method, the
conditions and the bar it must clear. Tests read that file. A number with no
measurement behind it is not ready to ship.

## What does not count as verification

- A message count. `ESC_STATUS` read 958/958 frames of exactly 0 with nothing attached.
- A plausible value where a real one should be. Absence renders `--` or `NaN`, never
  `0.0`.
- An in-air move. At ~0 m depth, target and measurement agree by accident.
- A test that has never failed against a real defect. Break it deliberately, watch it
  fail, restore it.
- "The build passed." The build is rung 3 of 6.

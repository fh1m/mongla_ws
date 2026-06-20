# Development Board — Mongla / Duburi (RoboSub 2026)

> **Single source of truth for status, open work, bugs, and fixes.** Start here.
> Detail lives in the linked docs; this board is the dashboard, not a duplicate.
> **Last updated:** 2026-06-19 · **Competition:** July 11, 2026.
>
> Three-state model (used everywhere): **✅ BUILT & TESTED** · **🟦 COMMITTED (phase-2, not built)** · **✏️ CORRECTED**.

---

## 1. 2026 commitment (P0.1 — decided 2026-05-31, tech lead)

Full TDR is the committed target. Reconciliation Decision Record: [`robosub-2026-audit.md`](robosub-2026-audit.md) §6.

| # | Decision |
|---|----------|
| 1 | **Dual vehicle COMMITTED** — Duburi 4.5 (built, phase-1) + Dubomini 2.0 (🟦 phase-2). |
| 2 | **YASMIN FSM COMMITTED** ✅ **BUILT** (commit 4a94231, 2026-06-03). `detected()` scripts kept as proto / unit-test / FSM-fallback layer. |
| 3 | **IVC COMMITTED** (🟦 phase-2) — dual-vehicle dependency. |
| 4 | **YOLO11** is the detector (✏️ corrects TDR's YOLO26). |
| 5 | Doc strategy: code-truth + committed phase-2 annex; never claim unbuilt as running. |

---

## 2. Phase status at a glance

**✅ Phase 1 — BUILT & TESTED (single-vehicle Duburi, ~800 pt):**
control / MAVLink / ArduSub core · `detected()` reactive missions · YOLO11 + ByteTrack/Kalman + monocular depth (30 fps) · Gate / Return / search-align · DVL packet parser.

**✅ YASMIN FSM layer — BUILT (2026-06-03, commit 4a94231):**
`state_machines/` (VehicleProfile + DuburiState + state library + gate_flare + prequal plan builders).
Drop-in missions `gate_flare_fsm` + `prequal_fsm` + `gate_then_bin_fsm`. 34 tests, all green.
Full guide: [`fsm-guide.md`](fsm-guide.md). Works for both Duburi 4.5 and Dubomini 2.0 (auto-detected).

**✅ Competition mission architecture — BUILT (2026-06-19 → 2026-06-20, commit cec7f37):**
5-chunk competition run (detected-paradigm): `missions/task_{gate,slalom,bin,torpedo,return}.py` + `task_full_2026.py` combinator.
5 YASMIN FSM plan builders: `state_machines/plans/{slalom,bin_drop,torpedo_fire,return_gate,full_competition}.py` + launchers `missions/fsm_{slalom,bin,torpedo,return,full_2026}.py`.
New FSM states: `TurnState`, `ApproachState`, `VisionLockFireState`, `FireState`, `StyleRollState`.
`downward_cam=True` DSL arg on `vision.home/hold/vision_lock_fire` (auto-sets camera + kp_forward polarity).
Demo files renamed `demo_arc/find_person/heading_lock/move_see/square/pursue.py`.
`competition_config.py` for pool-day constants. Gate/return runnable today; slalom/bin/torpedo blocked on model training.

**🟦 Phase 2 — COMMITTED, NOT YET IMPLEMENTED** (tickets: [`robosub-2026-audit.md`](robosub-2026-audit.md) §6 P2 · schedule: [`robosub-2026-roadmap.md`](robosub-2026-roadmap.md) "Phase 2"):
Dubomini control path ([`vehicle-spec.md`](vehicle-spec.md)) · IVC transport · Slalom / Bins / Torpedo / Octagon FSM plan builders · stepper grabber · underwater preprocessing.
**Competition mission chunk scripts are authored (phase-1) — model training for slalom/bin/torpedo is the remaining phase-2 blocker for these tasks.**

---

## 3. Open work (live)

| Pri | Item | State | Detail |
|-----|------|-------|--------|
| P0 | TDR reconciliation (P0.1) | ✅ decided | audit §6 Decision Record |
| P1 | Underwater preprocessing (G5) | 🟦 open | audit §6.5 |
| P1 | Vision-control SITL smoke test (arm→dive→yaw→disarm) | 🟦 open | audit §6.6 / §4 |
| P1 | Path-marker follower + `drop_marker` (ESP32-serial) | 🟦 open (serial contract ✅ built — `PayloadDriver` + `fire` verb + `vision_lock_fire`; path-marker FSM state still needed) | audit §6, `project_payload_actuation` memory |
| P1 | `detector_node.py` `paused` param implementation | ✅ BUILT (2026-06-20) — `declare_parameter('paused', False)`, skip in `_infer_loop` after dequeue, `_on_parameter_change` handler | plan §gap-1 |
| P1 | `duburi_dsl.py` `pause_detector`/`resume_detector` methods | ✅ BUILT (2026-06-20) — subprocess `ros2 param set` rail, camera→node mapping fwd/dwn | plan §gap-1 |
| P1 | `full_mission.launch.py` (competition dual-cam launch) | ✅ BUILT (2026-06-20) — both detectors `paused:=True`, gate_rescue_repair fwd model | plan §launch |
| P2 | Model training: `slalom_red_pipe.pt`, `bin_fire_blood.pt`, `torpedo_blood_hole.pt` | 🟦 open — blocks slalom/bin/torpedo chunks from live pool testing | models/README.md §Competition |
| P2 | YASMIN FSM (core + gate/prequal/bin plans) | ✅ BUILT (4a94231) — [`fsm-guide.md`](fsm-guide.md) |
| P2 | YASMIN FSM (slalom/bin/torpedo/return/full_2026 plans) | ✅ BUILT (cec7f37) — `state_machines/plans/` + `missions/fsm_*.py` |
| P2 | Dubomini control path · IVC · Octagon/path-marker plans · grabber | 🟦 committed build tickets | audit §6 P2 |
| cont. | 800-line files (`duburi.py` 833, `auv_manager_node.py` 795) | watch | audit §3.8 |
| P1 | manager goal-acceptance / abort gating | ✅ tested (`dispatch_policy`, 13 tests) — full `execute_callback` live-node path is integration-only, not unit | audit §4 |

---

## 4. Fix log (this audit cycle, 2026-05-30 → 31)

All landed on `main`, tests green. Commits: `9276aae` · `c508579` · `7838286` (+ this docs reconcile).

| Sev | Fix |
|-----|-----|
| 🔴 | `vis_approach` crashed every call (missing `Move.action` field `target_vis_range`) — field added, rebuilt. |
| 🟠 | `vis_approach` drove forward forever when depth node offline — `vis_range<=0` now suppresses forward + warns (`_forward_decision`). |
| 🟠 | Clock-mixing — `thrust_loop` + pixhawk deadlines → `time.monotonic()`. |
| 🟠 | Disarm safety — mission runner `stop()`+`disarm()` on unhandled exception. |
| 🟠 | Abort-interruptible settle/brake — `_interruptible_sleep`, 50 ms abort poll (safety-stop latency). |
| 🟠 | Lateral-sign test gap (`1801fe2` class) — `_lat_pct`/`_yaw_pct` extracted + opposite-polarity test. |
| 🟡 | Battery NaN sentinel; `vision.use_tracks` per-goal snapshot/restore. |
| 🔵 | Banner `MONGLA · DUBURI AUV MANAGER`; `motion_vision` yaw docstring. |
| chore | Deleted stray `missions/mission.py`; `.graphifyignore`; gitignore tool artifacts. |

| 🟠 | Manager goal-acceptance/abort gating extracted to pure `dispatch_policy.goal_acceptance` (safety-verb bypass rule) + unit-tested. |

**New test coverage (audit cycle):** `motion_vision` 13 · `connection_config` 16 · `nucleus_parser` 14 · `motion_writers` 4 · `dispatch_policy` 13.

**New test coverage (FSM, 2026-06-03):** `test_fsm_states` 31 (VehicleProfile · navigation · vision · utility states · plan builders).

**Bug fixes (2026-06-20, commit cec7f37):** `move_forward(duration=...)` → positional (5 files); `lock_heading(target=...)` → positional (task_full_2026 + LockHeadingState); `unlock_heading()` → `release_heading()`; `time.time()` → `time.monotonic()` in arc loop.

**Suite (per-package): control 80 · planner 34 · manager 29 · sensors 14 · vision 29 = 186.**

---

## 5. Bugs / known issues

- Tracked historical bugs (all FIXED): [`known-issues.md`](known-issues.md).
- Current cross-cutting findings (severity-tagged, file:line): [`robosub-2026-audit.md`](robosub-2026-audit.md) §3.
- No open 🔴 in phase-1 code. Phase-2 risk is build-execution, not control quality.

---

## 6. Doc map (where detail lives)

| Doc | Purpose |
|-----|---------|
| **this board** | live status / open work / fixes / bugs — start here |
| [`robosub-2026-audit.md`](robosub-2026-audit.md) | full audit, TDR⇄code gap matrix (G1–G12), Decision Record, P0/P1/P2 |
| [`robosub-2026-roadmap.md`](robosub-2026-roadmap.md) | phase schedule + Phase-2 committed tickets |
| [`known-issues.md`](known-issues.md) | tracked bug history |
| [`vehicle-spec.md`](vehicle-spec.md) | hardware + TDR-vs-impl delta (Dubomini, sensors, payload) |
| [`mission-design.md`](mission-design.md) | YASMIN FSM design reference (now built) |
| [`fsm-guide.md`](fsm-guide.md) | **FSM user guide** — fundamentals, VehicleProfile, state library, adding tasks, pool-day workflow |
| [`fsm-vision-missions.md`](fsm-vision-missions.md) | **Vision-guided mission design** — detection model, 6 search patterns, DVL/timed table, pick+drop worked example, gain tuning |
| `CLAUDE.md` §15 | Claude automations (agents/skills/hooks) |

> **Maintenance:** when a phase-2 ticket starts, flip its row 🟦→in-progress here and in audit §6. When a bug is fixed, add a §4 row. Keep the three-state honesty — never mark unbuilt as built.

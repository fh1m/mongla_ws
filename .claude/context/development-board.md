# Development Board — Mongla / Duburi (RoboSub 2026)

> **Single source of truth for status, open work, bugs, and fixes.** Start here.
> Detail lives in the linked docs; this board is the dashboard, not a duplicate.
> **Last updated:** 2026-06-24 · **Competition:** July 11, 2026.
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
New FSM states: `TurnState` (nav) + `VisionSearchState` / `VisionAlignState` / `VisionMoveState` (vision).
Vision verbs select the camera via `camera=` (or sticky `duburi.camera`); a downward camera auto-flips the `vision_align` depth-axis polarity.
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
| P1 | Path-marker follower + `drop_marker` (ESP32-serial) | 🟦 open (serial contract ✅ built — `PayloadDriver` + `fire` verb; path-marker FSM state still needed) | audit §6, `project_payload_actuation` memory |
| P1 | `detector_node.py` `paused` param implementation | ✅ BUILT (2026-06-20) — `declare_parameter('paused', False)`, skip in `_infer_loop` after dequeue, `_on_parameter_change` handler | plan §gap-1 |
| P1 | `duburi_dsl.py` `pause_detector`/`resume_detector` methods | ✅ BUILT (2026-06-20) — subprocess `ros2 param set` rail; all 6 detector helpers now derive node uniformly as `/duburi_detector_<camera>` via `_detector_node()` (2026-06-24) | plan §gap-1 |
| P1 | Vision launch consolidation → `vision.launch.py` (1-cam) + `vision_dual.launch.py` (2-cam) | ✅ BUILT (2026-06-24) — replaced 5 scattered launches; detector node = `duburi_detector_<camera>`, quiet (`--log-level warn`), detectors `paused:=true` by default in dual | plan §launch |
| P1 | `vision.move('gate')` pass-through (`fwd=None`) | ✅ BUILT (2026-06-24) — `fwd_fill<=0` sentinel → `move_loop` drives until target seen-then-leaves-frame + commit overshoot; never-seen → LOST | this plan |
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
| 🟡 | Battery NaN sentinel; per-goal vision-param snapshot/restore. |
| 🔵 | Banner `MONGLA · DUBURI AUV MANAGER`; `motion_vision` yaw docstring. |
| chore | Deleted stray `missions/mission.py`; `.graphifyignore`; gitignore tool artifacts. |

| 🟠 | Manager goal-acceptance/abort gating extracted to pure `dispatch_policy.goal_acceptance` (safety-verb bypass rule) + unit-tested. |

**New test coverage (audit cycle):** `motion_vision` 13 · `connection_config` 16 · `nucleus_parser` 14 · `motion_writers` 4 · `dispatch_policy` 13.

**New test coverage (FSM, 2026-06-03):** `test_fsm_states` 31 (VehicleProfile · navigation · vision · utility states · plan builders).

**Bug fixes (2026-06-20, commit cec7f37):** `move_forward(duration=...)` → positional (5 files); `lock_heading(target=...)` → positional (task_full_2026 + LockHeadingState); `unlock_heading()` → `release_heading()`; `time.time()` → `time.monotonic()` in arc loop.

**Vision stability Phase 2 (2026-06-23, commit 0a3d8e1):**
| Fix | What changed |
|-----|-------------|
| 🔴 Depth surfacing clamp | `motion_vision._MIN_DEPTH_M = -0.2` — vision depth loop can never command shallower than 0.2m; uses `min()` (negative-down). Hardware safety constant, not a ROS param. |
| 🟠 Mission state carry-over | New `mission_reset` verb: stops heading lock, clears abort event, RC neutral. Added to `_UNARM_SAFE`. All `run()` functions call it first. `cancel_callback` now also calls `unlock_heading()`. |
| 🟢 pool_day_torpedo.py | Practice mission: board detect → coarse align → fine hole lock → `fire`. |

> ⚠️ **Superseded by the 2026-06-24 two-verb rewrite (below):** the per-verb tuning this cycle added to the old 9-verb API — slalom lat-priority in the old `vision_track_axes`, the old `vision_lock_fire` `speed`/`h_frac_close` fields, and the removed `vision.stable_lock_s` param — was **removed** when vision collapsed to `vision_align` + `vision_move`. The depth clamp and `mission_reset` fixes above still stand; `pool_day_torpedo.py` now drives the board with `vision.align`/`vision.move` + `fire`.

**Vision two-verb rewrite + cross-subsystem harmony (2026-06-24):**
The 9-verb vision API (`vision_align_yaw/lat/depth`, `vision_align_3d`, `vision_hold_distance`, `vision_lock_fire`, `vision_acquire`, `look_around`; DSL `vision.find/home/turn/slide/hover/approach/track/scan/hold`) collapses to **two pixel-native verbs**: `duburi.vision.align(target, *, lat/yaw/depth=signed-px, err=40, duration=20, gain=30, fallback)` + `duburi.vision.move(target, *, fwd=95% fill, mode=area/width/height, maintain, hold, …)`.
| Sev | Fix |
|-----|-----|
| ✅ rewrite | Two verbs only. `gain` is a hard **max-speed cap** (% thrust), not a target speed. Misses are **non-fatal** — the server always returns `success=True` with an outcome code in `Move.Result.final_value` (`ALIGNED`=0 / `LOST`=1 / `TIMEOUT`=2 / `NO_CAMERA`=3 / `ABORTED`=4); search is a mission-authored `fallback(duburi[, should_stop])`. Control loops read `/detections` only (no `--tracking`). `detected()` is now case-insensitive. |
| ✅ params | Removed `vision.deadband`/`lock_mode`/`depth_anchor_frac`/`distance_metric`/`target_bbox_h_frac`/`stable_lock_s`/`h_frac_close`/`proximity_min_scale`/`speed`/`use_tracks`. New `/duburi_manager` params: `vision.kp_lat`=60, `vision.kp_yaw`=60, `vision.kp_depth`=0.05, `vision.kp_forward`=200, `vision.lost_grace_s`=1.0, `vision.frame_fill_default`=95, `vision.align_stable_frames`=3. |
| 🔴 surface() deadlock | `Duburi.lock` `threading.Lock`→`RLock` so the safety verb `surface()`→`set_depth()` re-entry can't hang (commands stay serialized; RLock only re-admits the same thread). |
| 🟠 heading-lock lifecycle | `HeadingLock` gained an `on_exit` callback: on timeout/auto-release it clears `Duburi._heading_lock` and releases the heartbeat hold. `disarm()` now stops/joins an active lock + releases the heartbeat before disarming. FSM `LockHeadingState` passes a long `lock_timeout` (task/hold duration, not the FSM state timeout); FSM `DisarmState` calls `release_heading()` first. |
| 🟠 vision/lock harmony | When a heading lock owns Ch4 and `yaw` is **not** an align axis, `vision_align` uses a `release_yaw` path (lateral via `send_rc_translation`, Ch4 left to the lock) instead of writing `yaw=1500` and fighting the lock. |
| 🟠 ALT_HOLD + cleanup | `vision_move` calls `_ensure_alt_hold('vision_move')` at entry. Manager exception cleanup uses lock-aware `duburi._writers().neutral()` (not raw `send_neutral()`). |
| 🟠 DSL never-die | `vision_dsl._orchestrate` catches `MoveFailed`/`MoveRejected`/`Exception` and returns a non-fatal `VisionResult(False,'FAILED',…)` so a bad camera / ALT_HOLD reject / disarmed never aborts a mission. |
| 🟡 NO_CAMERA gating | Vision verbs return `NO_CAMERA` until `vision_state.info_seen()` (camera_info seen) so pixel math is never mis-scaled on a non-640×480 stream. New throttled "boxes present but none match class" warning kills silent "sees but doesn't move". |

**Suite (per-package): control 80 · planner 34 · manager 29 · sensors 14 · vision 29 = 186.**

**Pre-competition completion audit (2026-07-01) — full-stack review + surgical fixes:**
Three parallel subsystem audits (control/vision/sensors+missions) + hand-verification.
**No fundamental math bugs** — heading-wrap, PWM, easing, pixel normalization all verified
correct. Two agent "CRITICALs" were false (disarm DOES release the heading lock; the mission
runner DOES stop+disarm on any exception). Fixes landed:
| Sev | Fix |
|-----|-----|
| 🟠 | **Fire-freshness guard** — the mid-hold `on_locked` torpedo/dropper fire now requires a LIVE, FRESH sample (`not coasted and age_s ≤ VISION_FRESH_FULL_S`). Never fires on a tracker-coasted (predicted) box or a frozen detector's stale frame. |
| 🟠 | **Distinct-detection stable gate** — `align_stable_frames` (and the settle gate) now count real detections, not 20 Hz loop ticks (`_FRAME_EPS_S`, keyed on `now - sample.age_s`). At low FPS one lucky frame can no longer declare ALIGNED or arm the fire. Backward-compatible (age_s≈0 = every tick a new frame). Duration budgets re-verified (smallest align 4 s ≫ ~1 s worst-case declare). |
| 🟠 | **Abort-interruptible settle** — `motion_forward.py` arc + DVL-dist and `motion_lateral.py` DVL-dist settle sleeps → `_interruptible_sleep(dur, abort_fn)` (cancel/safety-verb no longer waits out the settle). |
| 🟠 | **Mission runner hardening** — `mission.py` now calls `unlock_heading()` on BOTH failure paths (Ctrl-C `_abort_sequence` + the `except`), covering all 16 missions uniformly (no per-mission try/finally churn needed). |
| 🟠 | **`select_device('auto')` crashed on the Jetson** — `gpu.py:52` split `req` (`'auto'`, no colon) instead of `_DEFAULT` (`'cuda:0'`) → `IndexError` on any CUDA host, so `device: auto` in `detector.yaml` was an instant detector crash on the competition hardware. One-line fix (parse the index from `_DEFAULT`); `test_gpu` now green. |
| 🟡 | `task_gate` / `pool_day_practice` rescue-align gained `fallback=creep_forward`; redundant arc neutral removed; DVL open-loop-fallback now WARNs (silently-wrong distance visible). |
| — | **Dropped (non-bugs after verification):** the coast class-filter "fix" (the `and class_id` conjunct is intentional — lets a class-less predicted box coast on the already-matched track_id); per-mission try/finally on 5 non-primary missions (runner backstop + disarm-releases-lock already cover them). |

**Suite after audit: control 212 · manager 46 · planner 137 · (vision unchanged).**

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

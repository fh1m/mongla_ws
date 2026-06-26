# RoboSub 2026 — Full-Stack Audit & Action Plan

> **Date:** 2026-05-30 · **Competition:** July 11, 2026 (~42 days out)
> **Auditor pass:** code + logic + loops + control + MAVLink + ArduSub + vision + planning, cross-checked against the submitted TDR (`TDR26_BRACU_Duburi.pdf`) and the in-repo `robosub-2026-roadmap.md`.
> **Scope:** this is a **plan**, not a build. It triages; it does not implement FSM / IVC / Dubomini / new task missions.

---

## 0. Readiness verdict (one line)

**Phase-1 (single-vehicle Duburi) control + MAVLink + perception-plumbing is competition-grade for Gate / Return / search-and-align (~800 pt). The committed 2026 target (per the P0.1 decision below) is the full TDR: DUAL-vehicle, YASMIN-FSM-sequenced, IVC-coordinated, YOLO11 (TDR's YOLO26 corrected), 7-task system — of which Dubomini / IVC / FSM / the extra tasks are COMMITTED phase-2 build tickets with zero-or-partial code today. The headline risk is now schedule/build-execution on phase 2, not a control-quality gap and no longer a docs-vs-intent gap (reconciled 2026-05-31).**

P0.1 is **DECIDED** (see §6 + the Decision Record). Docs now read in three states: BUILT (phase 1) · COMMITTED-NOT-BUILT (phase 2) · CORRECTED (YOLO26→YOLO11). Central status: [`development-board.md`](development-board.md).

---

## 1. How to read this file

- **Severity:** 🔴 CRITICAL (wrong/unsafe in water) · 🟠 HIGH (bug or real gap) · 🟡 MEDIUM (maintainability/robustness) · 🔵 LOW (polish).
- Each finding has a **file:line anchor** and a **fix sketch**. Pick one at a time → focused PR.
- Findings already closed during this pass are marked **✅ FIXED THIS PASS**.
- Companion docs: tracked-bug history is `known-issues.md`; task schedule is `robosub-2026-roadmap.md`; this file is the cross-cutting audit + the TDR reconciliation.

---

## 2. TDR ⇄ Code gap matrix (the headline)

The TDR is the *submitted, aspirational* document. The audit's job is to surface where running code diverges from it. Per the workspace precedence rule (*"if anything contradicts the actual package layout in `src/`, the package layout wins"*) the **code is ground truth**; the TDR claims below that code does not back are **gaps to close or claims to soften**, not features to assume.

| # | TDR claim (§) | Code reality | Severity | Disposition |
|---|---------------|--------------|----------|-------------|
| G1 | **Dual vehicle**: Duburi 4.5 (primary) + Dubomini 2.0 (agile, 8-thruster, no DVL, no manipulators), parallel run (§I) | Single-vehicle stack. No Dubomini config, frame, thruster map, or control path. `mode`/profiles all single-vehicle. | 🔴 | **DECIDED: dual is COMMITTED (phase 2).** Dubomini profile / frame / thruster map / control path are build tickets — zero code today. Phase-1 Duburi is what runs. → `development-board.md`. |
| G2 | **Finite-State Mission Planner** in ROS2, states navigation/perception/manipulation/recovery, IVC handoff as a transition (§II.C.3) | No FSM. `duburi_planner/state_machines/` is **empty** (reserved). Missions are imperative `detected()`-paradigm Python scripts. | 🟠 | **DECIDED: YASMIN FSM is COMMITTED (phase 2)** for robust fail-safe autonomy; build ref `mission-design.md`, home `duburi_planner/state_machines/`. `detected()` scripts are **retained** as the prototyping / unit-test / FSM-fallback layer the FSM wraps — NOT removed. |
| G3 | **Inter-Vehicle Communication (IVC)** acoustic modem, release signal folded into FSM, bounded-window fallback (§I.D, §II.B.5, App D 2.4) | **Absent entirely** from the codebase — no IVC node, no transport, no release-signal transition, no fallback timer. | 🔴 (dual-vehicle dep) | **DECIDED: IVC is COMMITTED (phase 2)** — hard dependency of the dual-vehicle run, zero code today. Transport + FSM release-signal transition + bounded-window fallback are build tickets. |
| G4 | **Ultralytics YOLO26**, NMS-free, replaces YOLOv11 (§II.C.2) | Code is **YOLO11** (`yolov11n` "ROBOSUB tested ★"). `yolo.py:44` tags YOLO26 as *"previous family — backwards compat."* Commit `efcf9de` deliberately moved docs 26→11. | 🟠 | **DECIDED: YOLO11 is the committed detector — TDR's YOLO26 line is CORRECTED to YOLO11** (battle-tested, 30fps verified). YOLO26 stays only as a legacy/backwards-compat reference. |
| G5 | **Underwater CV preprocessing** (color-cast/haze correction ahead of detection) (§II.C.2, App D 3.3) | No preprocessing stage. Pipeline is camera→detector→tracker(→depth). `draw.py` is overlay-only. | 🟠 | Real perception gap for turbid pool water. Add a preprocess node/stage or scope the claim. |
| G6 | **Vision-guided torpedo** with own ESP32-S3 + camera + 6 thrusters, terminal self-alignment, fires at 0.46 m (§II.A.3) | No torpedo terminal-guidance code in this repo (it would be torpedo-side firmware). Carrier-side: only `set_servo_pwm` exists, and no torpedo mission/verb. | 🟡 | Torpedo MCU firmware is out of this repo's scope; but the **carrier-side fire verb + board-opening detection are missing**. Confirm torpedo firmware lives elsewhere; add the carrier hooks. |
| G7 | **Stepper grabber** (Evil Claw), step-count jaw control, scooping arc (§II.A.4) | `pixhawk.set_servo_pwm(aux, pwm)` is a **single-PWM servo** command — cannot drive a stepper (needs step/dir or a position interface). No grabber verb. | 🟠 | The TDR's 2026 grabber is not addressable by the current payload API. Need a stepper interface (likely via the Actuation Board, not Pixhawk AUX) + a `grab()` verb. |
| G8 | **Solenoid dropper**, high-side MOSFET switching, Actuation Board telemetry (§II.A.5, II.B.3) | `set_servo_pwm` can pulse an AUX line, but no `drop_marker` verb, no Actuation-Board telemetry ingest. | 🟡 | Add `drop_marker()` verb (roadmap already sketches it). Telemetry path is hardware-side. |
| G9 | **Simulation in ROS2 + Unreal Engine 5** (§III.B, App D Fig 25) | Code/sim is **Gazebo** (`bluerov2_gz`, ArduSub SITL). No UE5 integration in-repo. | 🟡 | Either UE5 work is external, or the TDR overstates. Gazebo is what `sim-setup.md`/CLAUDE.md document. Reconcile. |
| G10 | **Sensors**: Duburi compass/IMU/DVL = Nortek Nucleus 1000; Dubomini = VectorNav VN-200 (App A). Heading = fused inertial+DVL (§II.C.1). | Heading fusion **matches** (`bno085_dvl` / `CompositeBnoDvlSource`, Nucleus DVL). BUT primary inertial heading source in code is **BNO085** (ESP32-C3), not Nucleus AHRS or VN-200. App A omits BNO085; App D §3.2 *does* test "BNO085 IMU bias and drift." | 🟡 | Internal TDR inconsistency (App A vs App D). Code uses BNO085 — keep, and align App A. Confirmed by CLAUDE.md "Why BNO085 instead of VN200." |
| G11 | **Tasks**: Gate, Gate-with-style, Slalom, Torpedo, Bin, Octagon, Return Home (Table I) | Missions exist for: Gate, search/align, orbit, prequal. **No** Slalom, Torpedo, Bin, Octagon missions or detection models. "flare" is a **2025-era** target still in code, not in the 2026 TDR task list. | 🔴 (coverage) | Roadmap targets 0/1/6 + path markers (~800 pt) — far less than the TDR's 7-task dual-vehicle claim. Slalom/Bin/Torpedo/Octagon are unbuilt. |
| G12 | "**Path markers** (orange)" between tasks | Roadmap sketches HSV tracker (`_path_follow.py`) — **not yet built**. Not a TDR task per se but in the run narrative. | 🟡 | Build the downward-cam HSV follower (roadmap has the code). |

**Net:** the codebase is the *Duburi-4.2-derived single-vehicle* stack the roadmap honestly describes. The TDR is one strategic tier above it. **The single most important action is to decide, with the team, which document is the 2026 commitment** — then make the other consistent (§6).

---

## 3. Codebase audit by dimension

### 3.1 Logic & correctness

| Sev | Finding | Anchor | Fix |
|-----|---------|--------|-----|
| 🔴 | **`vis_approach` crashed on every invocation** — verb registered with field `target_vis_range` that did not exist on `Move.action`; `fields_for()` does `getattr(request,'target_vis_range')` → `AttributeError`, action aborts. The whole v4f monocular-depth approach verb was dead-on-arrival on the wire. | `commands.py:263`, `vision_verbs.py:186`, `duburi_dsl.py:816` vs `Move.action` | ✅ **FIXED THIS PASS** — added `float32 target_vis_range` to `Move.action`, rebuilt `duburi_interfaces`. `test_commands::test_every_field_is_on_move_goal` now green. **End-to-end consumption verified by reading**: `VisionState` subscribes `/vis_range` (`vision_state.py:115`), populates `sample.vis_range` by matched-detection index (`:215`), and `motion_vision._distance_size` returns it for `metric=='vis_range'` (`:561`) → `distance_error = target − vis_range` (`:358`). It is genuinely wired to the depth value, NOT a mislabeled bbox-height verb. |
| 🟠 | **`vis_approach` drives forward forever if the depth node is offline.** With no `/vis_range` publisher, `sample.vis_range` defaults `0.0`; `distance_error = 0.65 − 0 = +0.65` stays positive every tick → continuous forward thrust until `duration`. A valid *detection* is present so `on_lost` never trips. No guard distinguishes "node down" from "far target". | `vision_state.py:215`, `motion_vision.py:358` | Treat `vis_range==0.0` as "no depth signal" → refuse forward / fail fast, or require a `/vis_range` liveness check in preflight. |
| 🟠 | **Stray `missions/mission.py`** is a stale, fully-commented-out copy of `pursue_demo` (its docstring says "pursue_demo"). `discover()` registers it as a phantom mission named **`mission`**, and it carries a `lock_heading(degrees=0.0)` call whose kwarg may not match the DSL. Confusing duplicate of both `demo_pursue.py` (renamed from `pursue_demo.py`) and the real runner `duburi_planner/mission.py`. | `missions/mission.py` | Delete it. `demo_pursue.py` is the live version. |
| 🟠 | **Missions have no `try/finally` disarm.** `gate_flare_autonomous` only calls `_surface_and_disarm` on its own success/early-return paths; an exception inside `vision.align`/`move_forward_dist` exits the mission with the sub **armed and station-keeping** (heartbeat + heading-lock keep it failsafe-quiet). Relies on operator Ctrl-C → manager `_emergency_stop`. | `gate_flare_autonomous.py:61`, others | Wrap mission bodies in `try/finally: _surface_and_disarm`. Manager-side emergency stop is the backstop, not the primary. |
| 🟡 | **`vision.use_tracks` side-effect — MOOT (removed 2026-06-24).** At audit time a single `tracking=True` goal flipped a node param that poisoned later `VisionState` builds. The two-verb rewrite **removed** the `use_tracks` param and the `--tracking` flag entirely — control loops now read `/detections` only, so there is no tracks toggle left to leak. | `auv_manager_node.py` | None — param deleted by the rewrite. |
| 🔵 | `demo_pursue` (the keeper) relies on sticky `duburi.target`, so its vision verb and `fallback` search need no explicit `target=`. Confirm the sticky-context default is set (it is, via `duburi.target = TARGET_CLASS`). | `demo_pursue.py` | OK; note for mission authors. |

### 3.2 Loops & timing

| Sev | Finding | Anchor | Fix |
|-----|---------|--------|-----|
| 🟠 | **Clock-mixing residue.** The B1 hardening migrated `motion_yaw`/`motion_depth` to `time.monotonic()`, but the **forward/lateral drive loop** still uses wall-clock: `thrust_loop` `started_at=time.time()` / `elapsed=time.time()-started_at`, and `brake_kick_then_settle`. An NTP step mid-drive distorts the timed-thrust envelope. | `motion_writers.py:112,115` | Replace `time.time()`→`time.monotonic()` in `thrust_loop` + brake/settle helpers. |
| 🟡 | **Pixhawk arm/disarm/set_mode/wait_ack deadlines are wall-clock.** Same class as B1; lower impact (short ops) but inconsistent with the rest of the stack. | `pixhawk.py:174,205,231,249` | Migrate these deadlines to `monotonic` for consistency. |
| 🟡 | **Loop sleeps are blocking `time.sleep`.** Abort is checked at loop-top each tick (20 Hz → ~50 ms latency), acceptable. But the sleep is not abort-interruptible; a long settle (`SETTLE_SEC=1.2`) ignores abort for its duration. | `motion_writers.py:139,166` | If tighter abort latency is wanted, replace `time.sleep` with an `Event.wait(timeout=...)` that the abort signals. |
| 🔵 | Telemetry/HUD throttles use `time.time()` — fine (display, not control). | `auv_manager_node.py:648` | No action. |

### 3.3 Control (the strong core)

- **Yaw** (`motion_yaw.py`): rate-override on Ch4 with a real PID (`_YawPID`, Kp/Ki/Kd, anti-windup, sign-flip integral reset, min-speed clamp for T200 stiction), monotonic deadlines, stale-source hold, abort checks. `yaw_glide` sweeps a smootherstep setpoint then PID-locks. **Verdict: strong.** Only nit: `_YawPID` Kd is per-sample (no ÷dt) — fine because `YAW_RATE_HZ` is fixed and documented.
- **Depth** (`motion_depth.py`): pure setpoint-streamer over ArduSub's 400 Hz loop — no stacked Python PID. ALT_HOLD I-term prime, ramp-with-tracking to avoid reverse-chase, brake zone, monotonic. **Verdict: strong, textbook.**
- **Heading lock** (`heading_lock.py`): Ch4-only override (`send_rc_yaw_only`) so concurrent DVL Ch5/Ch6 moves are not clobbered; `SOURCE_DEAD_S=0.5` (B8) caps uncontrolled spin to ~22°; thread-safe retarget/suspend/resume. **Verdict: strong.**
- **Axis decomposition** (`motion_writers.Writers`): lock-aware writer bundle keeps per-axis modules ignorant of lock state. Matches the TDR's "independent per-axis modules, concurrent commands compose" claim — **this part of the TDR is true.**
- **Sign conventions (current hull, corrected 2026-06)**: Ch4>1500=yaw RIGHT, Ch6>1500=strafe RIGHT — vision yaw is **no longer negated** (matches lateral; the old `-ctrl*kp_yaw` drove away from target, pool-observed). The 2023 reference hull was RC4-reversed (Ch4>1500=LEFT); polarity is per-hull (`RC4_REVERSED`/frame config), so confirm with a bare `Ch4=1600` check on pool day. `heading_lock`/`motion_yaw` are polarity-correct regardless (sign from `heading_error` math).

### 3.4 MAVLink

- **`pixhawk.py`** is the sole pymavlink owner; reader-thread rule enforced; cached reads only. COMMAND_ACK handling (clear→send→wait) is correct; SET_MODE polls heartbeat (no ACK) correctly; autopilot-vs-loopback heartbeat filtering (`_autopilot_heartbeat`) is a nice touch. Per-frame `[MAV <fn> cmd=<verb>]` trace is excellent for pool debugging. **Verdict: strong.**
- 🟡 **Battery sentinel unguarded**: `get_battery` returns `voltages[0]/1000` with no check for the MAVLink "unknown" sentinel (65535 → 65.5 V). HUD/[STATE]/`/duburi/state` will show garbage if ArduSub reports unknown. Anchor `pixhawk.py:463`. Fix: treat `0xFFFF` as `None`/NaN.
- 🟡 **AUX servo ≠ stepper** (see G7): `set_servo_pwm` is fine for solenoid dropper / simple servo torpedo trigger, but the TDR's stepper grabber needs a different interface.

### 3.5 ArduSub

- Modes, failsafe model, and message-rate pinning are correct and well-documented. `MESSAGE_RATES` pins AHRS2@50/BATTERY@1/RC@5 (matches CLAUDE.md). Heartbeat daemon (`heartbeat.py`) explicitly guards `FS_PILOT_INPUT` with 5 Hz neutral override — and now logs send failures at ERROR (B9). **Verdict: strong.**
- 🔵 No `SET_MESSAGE_INTERVAL` for SYS_STATUS; battery comes from BATTERY_STATUS@1 Hz — fine. Any POSHOLD-dependent manoeuvre needs the Nortek BlueOS extension + EK3 params (documented in `dvl-reference.md`); verify on the actual vehicle.

### 3.6 Vision

- **Model**: YOLO11 (`yolov11n` default for pretrained; `gate_flare_medium_100ep` for prequal). YOLO26 demoted to backwards-compat. See G4.
- **Pipeline**: camera_node → detector_node → tracker_node (ByteTrack + Kalman, `_class_map` pruning per B7) → depth_estimation_node (`vis_range`, Depth Anything V2-Small ONNX + bbox-area fallback). HUD `vision_display`. Closed loop runs INSIDE the manager (`motion_vision`) so vision and control never fight for thrust.
- **Control loops verified by reading** (`motion_vision.py`, `align_loop` + `move_loop`): per-tick P-on-pixel error with `gain` as a hard max-speed clamp; `align_loop` centres on lat/yaw/depth (each at a signed pixel offset), `move_loop` drives forward to a bbox fill ratio. **Sign conventions (corrected 2026-06)**: neither yaw nor lateral negates — both `+ctrl*kp` on the same `ex` (Ch4>1500=yaw RIGHT, Ch6>1500=strafe RIGHT on the current hull). The earlier `-ctrl*kp_yaw` negation (from the inherited "Ch4>1500=LEFT" 2023 label) drove the AUV away from the target — pool-observed and fixed. **Verdict: well-built.**
- ✅ ~~**Stale module docstring in `motion_vision.py:42`** shows the *pre-fix* un-negated `yaw_pct = clamp(ex * Kp_yaw)`; the live code negates.~~ **SUPERSEDED 2026-06:** the un-negated form is now the *correct* live form — the negation was the bug (drove away from target, pool-observed). Docstring and code both un-negated; consistent.
- 🟠 **No underwater preprocessing** (G5) — biggest perception gap for turbid water.
- 🟡 **Monocular `vis_range` is extra, not in the TDR.** It is a reasonable standoff cue but the TDR ranges via DVL/torpedo pose. Don't over-rely on monocular depth for firing distance; DVL is the TDR's ranging story.
- 🟡 **Vision verbs have no unit tests** — `motion_vision.py` (621 lines) and `vision_verbs.py` (445) are the most competition-relevant control code and have zero direct test coverage. Sign errors here (cf. the lateral-sign fix `1801fe2`) are exactly what bites in the pool.

### 3.7 Planning

- The `detected()`-paradigm (`gate_flare_autonomous.py`) is genuinely good: short open-loop search steps, **safety budgets** (`MAX_*`) prevent infinite loops, class-filter restore avoids the "orbit trap", `on_lost='hold'` rides out flicker, DVL distance moves with open-loop fallback. This is solid reactive autonomy.
- 🟠 **No FSM** (G2) — contradicts the TDR; defensible per roadmap.
- 🔴 **Task coverage** (G11): Slalom/Bin/Torpedo/Octagon unbuilt; path-marker follower unbuilt. The realistic 2026 envelope is Gate + Return + search/align ≈ 800 pts, not the TDR's 7-task dual-vehicle run.
- 🟠 **No disarm-in-finally** (§3.1) across missions.

### 3.8 Design decisions (mostly good)

- ✅ Single MAVLink owner; registry-driven dispatch (one row + one method to add a verb); axis-decomposed motion; ArduSub owns the inner loop; lazy per-camera VisionState pool; cooperative abort via `_abort_event`; emergency-stop on SIGTERM/Ctrl-C. These are the right calls and are executed cleanly.
- 🟡 **Two files exceed the 800-line guideline**: `duburi.py` (833), `auv_manager_node.py` (795). Both cohesive but at the cap. Consider extracting the vision-state pool / DVL-auto-connect from the node, and the facade's vision mixin is already split — keep watching.
- ✅ ~~🔵 **Nomenclature drift inside code**: manager banner says `DUBURI AUV MANAGER` while `_KILL_BANNER` says `MONGLA EMERGENCY STOP`.~~ **FIXED** — startup banner now `MONGLA · DUBURI AUV MANAGER` (project Mongla · vehicle Duburi), consistent with the kill banner.

> **Payload-actuation correction (G6/G8, §5.9):** the dropper + torpedo are **NOT** on Pixhawk MAVLink AUX (`set_servo_pwm`) as the docs assumed. Real/intended path is a **separate ESP32 over USB serial** (PySerial → 4 GPIO → relays); Jetson sends string commands (e.g. `release_torpedo_one`). So `drop_marker`/`fire_torpedo` need a new ESP32 *actuator* serial client (mirror the `duburi_sensors` BNO085 CDC reader), not a servo-PWM verb. Payload build **parked** by owner pending the serial contract (port/baud/command tokens/GPIO map). Fix CLAUDE.md §5.9 + roadmap when payload is un-parked.

---

## 4. Tests

**State after this pass: 95 passed, 0 failed, 0 errors** (`duburi_control` + `duburi_vision` + `duburi_planner`). Fixed this pass:

| Test | Was | Fix |
|------|-----|-----|
| `test_commands::test_every_field_is_on_move_goal` | failed — `target_vis_range` not on `Move.Goal` | **code fix** (added field; rebuilt interfaces) — was a real runtime bug, not test-lag |
| `test_missions_smoke` | collection error — imported dead `NAMES` | rewrote to current `discover()` API |
| `test_heading_lock` ×5 | `FakePixhawk` missing `send_rc_yaw_only` | added stub method + refreshed docstring |
| `test_factory::test_builders_known_keys` | stale expected set | added `video_file` builder |
| `test_depth_estimation::test_onnx_model` | pytest treated helper arg as a fixture | renamed `_check_onnx_model` (CLI-only), kept `main()` |

**Coverage gaps (do not meet the 80% target):**

- 🟠 **No tests for vision control** (`motion_vision`, `vision_verbs`) — highest-risk untested code.
- 🟠 **No integration/E2E** — no SITL smoke test that arms→dives→yaws→disarms against a fake/SITL MAVLink. `test_missions_smoke` only checks import + signature.
- 🟡 `test_depth_estimation` is a report-style harness (no asserts on the collected tests) — weak signal.
- 🟡 No tests for `auv_manager_node` dispatch/abort, ~~`connection_config.resolve_mode`~~, DVL parser (`nucleus_parser`). **`resolve_mode`/`resolve_profile` covered** — `src/duburi_manager/test/test_connection_config.py` (16 tests). **`nucleus_parser` covered** — `src/duburi_sensors/test/test_nucleus_parser.py` (14 tests). **Manager goal-acceptance/abort gating covered** — `dispatch_policy.goal_acceptance` extracted (safety-verb bypass rule) + `test_dispatch_policy.py` (13 tests). Only the full `execute_callback` live-node path remains (integration/SITL, not a clean unit) — tracked on the board, not a unit-test gap.

---

## 5. New Claude setup review (`.claude/agents|hooks|skills`, `.understand-anything`)

**Verdict: sound, well-engineered, correctly wired.** Reviewed as artifacts (not spawned).

- **Hooks** (`settings.json` wires them via `$CLAUDE_PROJECT_DIR`):
  - `block_install.py` (PreToolUse Edit|Write): exit-2 blocks edits under `/install/`. Correct.
  - `py_check.sh` + `pkg_test.sh` (PostToolUse): `pkg_test.sh` src-shadows the installed copy via `PYTHONPATH` and is advisory (always exit 0). Correct, and explains the source-vs-install resolution. 🔵 Note: `pkg_test.sh` greps the pkg from `src/<pkg>` — a non-`src/` edit is ignored (intended).
- **Agents** (6): `mavlink-reviewer`, `mission-reviewer`, `doc-verifier`, `context-doc-sync`, `robosub-task-architect`, `vision-model-reviewer`. 🟡 `vision-model-reviewer` is described as "YOLO11 train/detect" — keep that wording aligned with whatever G4 resolves to.
- **Skills** (5): `pool-day`, `add-command`, `new-mission`, `train-model`, `verify-docs`. Consistent with the registry-driven architecture.
- **`.understand-anything/`**: generated knowledge graph (157 files / 222 nodes, commit `8c978cb`, 2026-05-09). Tool artifact, regenerable, untracked. No action — just know it can go stale after refactors.

---

## 6. Prioritized action plan (for RoboSub 2026)

### P0 — decide & de-risk (this week)
1. ✅ **TDR reconciliation decision (P0.1)** — **DECIDED 2026-05-31 (tech lead).** Gates G1/G2/G3/G4/G11. See the **Decision Record** below; docs now use the three-state model (built / committed-phase-2 / corrected). Central tracking → [`development-board.md`](development-board.md).

   > **Decision Record — 2026-05-31 (tech lead):**
   > 1. **Dual vehicle COMMITTED** (Duburi 4.5 + Dubomini 2.0). Phase-1 single-vehicle Duburi is what's built; Dubomini is a phase-2 build ticket. (G1)
   > 2. **YASMIN FSM COMMITTED** (phase-2) for robust fail-safe autonomy. `detected()` scripts are **kept** as the prototyping / unit-test / FSM-fallback layer the FSM wraps. (G2)
   > 3. **IVC COMMITTED** (phase-2) as a hard dependency of the dual-vehicle run. (G3)
   > 4. **YOLO11 is the committed detector** — TDR's YOLO26 line **corrected** to YOLO11. (G4)
   > 5. Doc strategy: **code-truth + committed phase-2 annex** — state what ships today; mark dual/FSM/IVC/tasks as committed-not-built; never claim them running.
2. ✅ **Disarm-in-finally** (🟠 §3.1) — **FIXED**: done at the runner (`mission.py` `except Exception` now does best-effort `stop()`+`disarm()`). Scope: covers every **runner-launched** mission on the **abort/exception** paths (the KeyboardInterrupt path already did). NOT covered: a mission that completes normally without its own disarm, or a script run via raw `DuburiClient` outside the runner. Uses `stop()`+`disarm()` (disarm → positive-buoyancy surface), not `_surface_and_disarm` (no explicit `set_depth(0)`).
3. ✅ **Clock-mixing residue** in `thrust_loop` (🟠 §3.2) — **FIXED**: `started_at`/`elapsed` now `time.monotonic()` (`motion_writers.py:112,115`). Also migrated the 4 pixhawk arm/disarm/mode/wait_ack deadlines to monotonic (🟡 §3.2).
4. ✅ **Delete stray `missions/mission.py`** (🟠 §3.1) — **FIXED**: `git rm`'d (was a stale commented-out pursue-demo dup; live `demo_pursue.py` retained).

### P1 — close the believable-points gap (next 2–3 weeks, before pool days)
5. **Underwater preprocessing stage** (🟠 G5) — color/haze correction ahead of the detector; biggest perception ROI for turbid water.
6. **Vision-control tests** (🟠 §4) — at minimum sign / error-band unit tests for `motion_vision` error→Ch mapping; a SITL arm→dive→yaw→disarm smoke test.
7. **Path-marker follower** + **`drop_marker` verb** (🟡 G8, G12) — roadmap already has the code; unlocks Bins.
8. ✅ **YOLO26 decision** (🟠 G4) — **DECIDED: YOLO11** (committed). TDR corrected; YOLO26 legacy-only.
9. ✅ **Battery sentinel guard** (🟡 §3.4) — **FIXED**: `get_battery` now maps `0xFFFF` mV voltage and `-1` current to `math.nan` (`pixhawk.py:463`).

> **Also fixed this pass** (cheap items beyond the P0 trio):
> - 🟠 §3.1 `vis_approach` forever-forward — `motion_vision.py` now treats `vis_range<=0` (depth node offline) as no-signal: forward thrust suppressed + throttled warning, instead of phantom-far thrust to timeout. Logic extracted to pure `_forward_decision()` and **unit-tested**.
> - 🟡 §3.1 `vision.use_tracks` permanent flip — first fixed by snapshot+restore around the goal, then made **moot by the 2026-06-24 two-verb rewrite**, which removed the `use_tracks` param and the `--tracking` flag entirely (control reads `/detections` only).
> - 🔵 §3.6 stale `motion_vision.py:42` docstring — yaw formula sync'd to live code. **NOTE 2026-06:** the sign was later *re-corrected* to **un-negated** (`+ctrl*kp_yaw`) — the negation (from the inherited 2023 "Ch4>1500=LEFT" label) drove the AUV away from target on the current hull (pool-observed). See `feedback_yaw_direction` memory.
>
> **Partial close of §4 "no vision-control tests":** new `test_motion_vision.py` (9 tests) pins the yaw sign + vis_range guard via the extracted helpers. The full `align_loop` / `move_loop` + `vision_verbs` paths still need an integration/SITL test.
> Tests: **75 green** (duburi_control + duburi_planner) after rebuild — run with `pytest -p no:anyio` (the `colcon test` entrypoint hits a pre-existing `anyio`/`_pytest.scope` plugin error in this env; unrelated to these changes).

### P2 — Phase-2 build tickets (COMMITTED per the 2026-05-31 decision)
10. **YASMIN FSM** in `duburi_planner/state_machines/` (G2) — wraps the `detected()`/DSL verbs as states; build ref `mission-design.md`. Hosts the IVC release-signal transition.
11. **Dubomini control path** (G1) — profile / frame / `vectored_6dof` thruster map / param-set / mode; VN-200 sensor source.
12. **IVC** transport + FSM handoff + bounded-window fallback (G3).
13. **Slalom** mission + pipe detection (roadmap Option A classical first); **Bins**/**Torpedo**/**Octagon** + path-markers (G11/G12).
14. **Payload actuation** — ESP32-serial dropper/torpedo client (`drop_marker`/`fire_torpedo`), NOT Pixhawk AUX (see `project_payload_actuation` memory); **stepper grabber** via Actuation Board, `grab()` verb (G7, Octagon dep).

### Continuous
14. Keep `known-issues.md` honest — it currently says "backlog empty"; the `vis_approach` crash postdated it. Add a "post-v4f audit" section pointing here.
15. Watch the 800-line files (`duburi.py`, `auv_manager_node.py`).

---

## 7. Bottom line

- **Tests:** synced to current code; **95 green**; one genuine runtime bug (`vis_approach`) found and fixed.
- **Control / MAVLink / ArduSub:** genuinely strong, competition-hardened. Minor clock + sentinel cleanups.
- **Vision:** good plumbing; missing underwater preprocessing; YOLO11 not YOLO26; monocular depth is a bonus, not the TDR's ranging story.
- **Planning:** excellent reactive `detected()` autonomy for Gate/Return (now the proto/test/fallback layer); YASMIN FSM committed for phase 2; Slalom/Bin/Torpedo/Octagon are committed-not-built; disarm-in-finally fixed.
- **On track for RoboSub 2026?** **Phase-1 (single-vehicle Duburi, ~800 pt) — yes**, the core is competition-grade. **Full TDR (dual + FSM + IVC + 7 tasks) — committed but unbuilt**; that is now a **phase-2 build-execution** effort (P2 tickets §6), not a docs-vs-intent gap. P0.1 is reconciled (2026-05-31): docs state built vs committed-phase-2 honestly. Live status → [`development-board.md`](development-board.md).

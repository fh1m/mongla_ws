# Bug register — every defect in mongla_ws, open and fixed

> **This file is the SINGLE tracker for code defects in `mongla_ws`.**
> It replaces the bug content previously spread across `known-issues.md`,
> `ROADMAP.md`, `ROADMAP.md`, `water-owed.md` and
> `srot-pre-dive-gates.md`. See §6 for exactly what was migrated, what was
> deliberately left in place, and why.
>
> Produced by the system-wide audit of 2026-09-08, post the `srot`→`main` merge
> (`6db956a`). Scope: controls, vision, planner, sensors, managers, plus the
> three sibling repos.
>
> **STATUS: 42 of 48 fixed (2026-09-08).**
> B01, B02, B03, B05, B09, B10, B21 (the first SROT-path batch) · B16, B22, B23,
> B27 (vision/tooling) · B18, B30 (the srot vision axes) · B25, B26, B29 — found
> while fixing the others. Each landed with a test **verified to fail without the
> fix**.
>
> **No open item is reachable on the live srot flight path.** That claim is
> checked mechanically by `tools/srot_reachability.py`, which walks CALLS from the
> real srot entry verbs — **not** by grep and not by which modules are imported.
> Both of those gave wrong answers in the same pass (B12 false positive, B18 false
> negative), and chasing the contradiction is what surfaced **B30**, a live
> default-on `AttributeError` in the vision arrival brake. Re-run that tool before
> ever writing "not on this path" in here.
>
> The remaining open items are ArduSub-path (B06, B07, B08, B12, B13, B15, B17,
> J01 — preserved on the `pixhawk` branch), DVL (B04, B11, B14, B19, B20 — not
> fitted), FSM (J02, J03 — never run), or docs-only (B24, whose doc half is done
> and whose code half is B06).
>
> **Caution: B28 is the one srot-path item still open, and only half of it is ours.** The
> host guard is fixed and tested; the firmware fix is upstream PR
> srot-control-board#15, source-verified but **not bench-demonstrated** — the board
> refuses to arm without the thruster pack.
>
> **Caution: J02 is inert only because J03 is true.** `SurfaceState` swallows the failure
> of `set_depth(0.0)` and returns `SUCCEED` unconditionally — a failed ascent
> reported as success. Fix it **before** the FSM is ever switched on for a run.

## How to read this

Severity is about *consequence on the vehicle*, not about how hard the fix is.

| Grade | Meaning |
|---|---|
| **CRITICAL** | Can hurt the hull or hide a condition that can. Fix before water. |
| **HIGH** | Wrong control output, or a wrong number reported as right. |
| **MEDIUM** | Correct today, wrong under a reachable condition. |
| **LOW** | Real but bounded; style/robustness debt with a named trigger. |

`REPRODUCED` means a runnable check was written and executed in this pass, and
the output is quoted. Everything else is read from source and cited by line.

**The single most useful sentence in this document:** in the three worst
findings the *analysis was already correct and the wiring was not* — a leak
reporter that is never registered (B01), a calibration flag no caller consults
(B04), and a guard disarmed by its own caller (B05). Reviewing for "is this
logic right?" would have passed all three.

---

## 1. CRITICAL

### B-89: five missions moved the board before arming it `READ 2026-11-07` **FIXED 2026-11-07**

**What.** The board refuses every SROT_MOVE while disarmed (firmware
`mav_commands.cpp`, "SROT_MOVE refused: arm first", on their `main` since
`139d943`), and `pause` is a SROT_MOVE. `pool_day_practice`, `sauvc_full`,
`sauvc_navigation`, `sauvc_qualification` and `task_full_2026` all took their
tether window as `pause(N)` before `arm()` -- correct on the old backend, where
`pause` was a host sleep, and a refusal on this one. Each would have failed at
its first line, surfaced and disarmed, at the pool.

**Fix.** The tether window is `countdown()`, on the host: monotonic deadlines
per tick (a `sleep(1)` per tick ran ~2 ms long each, measured-bars §68), and a
refusal on an ssh session outside tmux or screen, because pulling the tether
kills that session and the mission with it. Guard:
`test_no_board_verb_before_arm.py` reads every mission's calls and fails on a
board verb before `arm()`; injection-verified against the old missions.

### B-90: a kill switch engaged mid-run left the vehicle armed `READ 2026-11-07` **FIXED 2026-11-07 (host)**

**What.** The kill switch cuts thruster power on the second board; the
control board treats its state as display-only (`arming.h`) and stays armed,
still running the move. Turning the knob back powers the ESCs into whatever
the board is commanding, under a diver's hands. Upstream issue #12 asks the
board to refuse arming when killed; nothing on either side acted on a kill
DURING a run.

**Fix.** The manager disarms on the edge to ENGAGED while armed
(`node_support.kill_watch`, polled with the mission watchdog at 2 Hz). Only a
True reading acts: `None` is "no second-board link". Disarm aborts the
mission's verb, so its own cleanup runs. The firmware should do this itself,
at 500 Hz; added to #12. Test: `test_mission_keepalive.py`, injection-verified.

### B53 — every payload channel the missions use is REFUSED by this board `MEASURED 2026-09-22, re-read 2026-10-01` **MAP REMOVED 2026-10-01** — channels await the wired payload
**`missions/competition_config.py` vs the board's `SERVOn_ROLE` params**

Read live off the board, all 16 channels, twice each (retry on timeout):

```
FIREABLE (SWITCH):  9, 10, 12, 13, 14, 15, 16
SERVO (must not drive): 1, 2, 3, 5, 6, 7, 8
NONE (unroled): 4, 11
```

What the missions believe (`competition_config.py:60,189`, `pool_day_practice.py:63,65`):

> `1=torpedo_1, 2=torpedo_2` · `3 = dropper_1, 4 = dropper_2`

**Every one of those is SERVO or NONE on this board.** Resolved by AST — five real
`fire()` call sites, not grep guesses:

| call site | channel | role | result |
|---|---|---|---|
| `sauvc_target_acquisition.py:88` | 3 | SERVO | `FIRE_REJECTED_ARM` |
| `task_bin.py:96` | 3 | SERVO | `FIRE_REJECTED_ARM` |
| `pool_day_practice.py:178` | 1 | SERVO | `FIRE_REJECTED_ARM` |
| `pool_day_practice.py:206` | 3 | SERVO | `FIRE_REJECTED_ARM` |
| `demo_dual_camera.py:68` | 3 | SERVO | `FIRE_REJECTED_ARM` |

**Warning: If this board's roles are representative, every drop and every torpedo scores
zero** — the dropper and target-acquisition tasks in full.

**The guard works.** `fire()` fails closed: it refuses a SERVO channel rather
than writing `DO_SET_SERVO` to the on-board arm, and refuses an unreadable role
rather than guessing. Nothing here silently no-ops. That is the design working
exactly as intended, and it is why this was findable on a bench.

**Caution: TWO HYPOTHESES, AND THIS ONE IS NOT OURS TO CLOSE.**

1. **The mission channel map is wrong** and should point at 9/10/12–16.
2. **This bench board's `SERVOn_ROLE` params are simply unconfigured.** "1–8 servo,
   9–16 switch" is exactly the folklore `preflight_roles`' own docstring says the
   full read exists to retire — and this board almost matches it, with two
   unexplained holes at 4 and 11. A default or partial config would look like this.

The roles are **firmware state set in Bondor**, not ours, so the answer is an
operator/GCS fact, not a code fact. Recorded rather than "fixed", because
repointing five missions at channels nobody has confirmed would be swapping one
unverified map for another.

**To close it:** on the competition hull, run the channel read and compare. If the
hull agrees with this board, the missions move; if it disagrees, this bench board
gets configured and nothing in `src/` changes.

**2026-10-01 — the invented map is gone, and an unassigned channel is loud.** The
user confirmed nothing is wired to the payload yet, and the bench board re-read
identically (SWITCH 9,10,12–16; SERVO 1,2,3,5–8; unroled 4,11). So instead of
swapping one unverified map for another:

- `competition_config.py` is the **one** place channels are assigned:
  `TORPEDO_1/2_CHANNEL`, `DROPPER_1/2_CHANNEL`, all **0 = NOT ASSIGNED**, with the
  board's live read written beside them. `BIN_`/`SAUVC_DROPPER_CHANNEL` are aliases.
  Five missions that hard-coded 1 or 3 now read those names.
- **Warning: A silent drop was found on the way.** `vision_verbs._parse_channels` dropped
  anything outside 1..16 — so `align(fire=0)` would have fired *nothing* with no
  error. Out-of-range numbers now pass through, and `fire()` refuses each with a
  reason that lands in the verb's payload outcome.
- `fire(0)` refuses as **"payload channel NOT ASSIGNED"**, and every refusal (0,
  SERVO, disabled) now names the board's own SWITCH channels from the roles read at
  bring-up — the operator gets the answer, not just the "no".
- `test_missions_never_hard_code_a_payload_channel.py` fails on a literal channel
  in any `fire(...)` / `fire=` or any `*_CHANNEL = <n>` outside the config.

**To close fully:** wire the payload to SWITCH channels, set the four names, and
update `test_the_channels_are_unassigned_until_someone_confirms_them`.

---

## 2. HIGH

### B-91: the floor's yaw bound was applied on arrival, against the heading now `READ 2026-11-07` **FIXED 2026-11-07**

**What.** `flow_node` published `floor_grid_deg` and `lane_heading_deg` as bare
`std_msgs/Float32`: no stamp. `localization_inputs._bound_yaw` snapped the
current filter yaw to that angle and applied the update at arrival. With
`retrodict:=true` every other aiding measurement is replayed at the instant it
describes, so these two alone compared a frame taken some time ago with the
heading of now: during a turn the bound pulled yaw toward where the hull was,
and folded that into `_yaw_offset_deg`. With `retrodict` off (the default)
every measurement lands on arrival, so this one was no worse than the rest.

**Fix.** Both topics are `mongla_interfaces/FloorHeading`, a header stamped
with the camera frame plus `heading_deg`. `_bound_yaw` passes that stamp to
`_apply`, and the snap is computed inside the replayed update against the yaw
*at the stamp*; the offset fold uses the change the replay made.

**Measured** (bench, synthetic): a 10 deg/s turn, grid seen with 0.3 s
latency, replay on. Judged on arrival the filter ended 1.55 deg off the board;
replayed, 0.01 deg. `test_a_late_floor_heading_is_judged_against_the_yaw_of_its_frame`
holds it and fails (by the 1.55 deg) when the stamp is dropped. The in-water
size of the effect is still not measured (`workbench/POOL.md`).

**Found by** the `std_msgs` inventory (bead `jxt`). The other eight primitive
topics stay: `mission_alive` and `distance_control` are a liveness signal and a
command whose arrival time is the point; `localization/motion` is a latched
state label; `distance_traveled` is read as a start-to-stop difference;
`flow_quality` and the three `video_*` topics are operator and bench
diagnostics.

### B-88: flow_node held a height from before the depth stopped `READ 2026-11-07` **FIXED 2026-11-07**

**What.** B10 gave the old `distance_estimation_node` a clock on its height:
a depth older than 2 s gave no height, so a stalled link refused frames
instead of scaling them by a stale number. `flow_node` replaced that node on
the vehicle and never had the fix. It kept `_last_height` from the last depth
it saw, for ever, and every velocity it published is proportional to that
height. Its debug topic also published an absent height as `0.0` (B05's rule,
on the output side).

**Fix.** `flow_node._fresh_height` recomputes the height every frame from a
depth no older than `HEIGHT_STALE_S` (2.0 s, `flow_constants.py`), aged from
the board's own measurement instant; absent, the frame is refused. The debug
topic reports NaN. The old node is deleted (git history) and every launch
starts `flow_node` with `flow:=true`. Test: `test_distance_height_gate.py`,
injection-verified.

### B-84: a pre-empted verb freed the gate its pre-emptor held `READ 2026-11-07 (cloud audit)` **FIXED 2026-11-07**

**What.** `disarm` pre-empts a running `move_forward`: it claims the command gate
and signals an abort. The move's loop then exits, and its `finally` called
`release()`, which cleared **whatever** held the gate. The gate was empty while
`disarm` was still executing, so an ordinary verb could start beside it. Two
`stop`s back to back had the same shape, because a verb name cannot say which
claim is which.

**Fix.** A claim carries a token. `confirm()` hands it to the goal that reached
`execute_callback` (once: a second goal for the same claim is refused, not run
unguarded), and only that token can `release()`. A cancel on an idle gate no
longer starts a drain nobody would end. An unknown verb is rejected in
`goal_callback`, before it can claim anything; it used to hold the gate for
`COMMAND_GATE_STALE_S`. Tests: `test_command_gate.py`,
`test_unknown_verbs_never_claim_the_gate.py`, both injection-verified.

### B-85: confirmations read off messages from before the command `READ 2026-11-07 (cloud audit)` **FIXED 2026-11-07**

pymavlink keeps the last message of each type forever, and four places took
that cache as the board's answer:

| where | what it believed | now |
|---|---|---|
| `set_mode` | a cached heartbeat already in the mode, or a dead link's last one, confirmed the change at once | only a heartbeat heard *after* the command |
| a move's "did a failsafe cut it short?" | with no heartbeat after the ACK, judged on the one before it: ARMED and AUTO by construction | **fails, unverified** -- the manager asks for 10 Hz, so silence is the news |
| `VFR_HUD` depth | no age at all, beside an age-gated ATTITUDE | absent once older than `_LINK_STALE_S` |
| `get_mode` | a minutes-old heartbeat's mode, as current | `UNKNOWN` |

The payload ACK wait also moved from the wall clock to the reader's monotonic
stamp; the Pi has no RTC and the first NTP sync steps it. Tests in
`test_srot_fc.py`, `test_srot_move_respects_surface.py`,
`test_stale_telemetry_is_absent.py`. The test fakes gained what a real board
does: a 10 Hz heartbeat, and arrival stamps.

### B-86: arm proceeded when the board would not say its firmware revision `READ 2026-11-07 (cloud audit)` **FIXED 2026-11-07**

The "warn but proceed" rule for a silent board was decided when the floor was
rev 2 and the cost of an old board was a coasting `stop`. The floor is now
**rev 10**, because below it the board's yaw sense is inverted and every loop
closed on yaw diverges. `read_behaviour_rev` already asks three times over six
seconds, so "no answer" means firmware older than `REQUEST_MESSAGE` or a link
too poor to arm on. Arm now refuses; connect still only warns;
`allow_fw_behaviour_mismatch:=true` overrides, loudly.

### B-87: a surface nobody saw was reported as reached, and the abort promised one `READ 2026-11-07 (cloud audit)` **FIXED 2026-11-07**

`surface()` returned `success=True` after its timeout with the hull still at
depth. It now fails, saying the board is still ascending; SURFACE stays engaged.
The mission's Ctrl-C banner printed "Mongla will surface on positive buoyancy"
and a three-second countdown that waited on nothing. This hull's net buoyancy
has never been measured, and nothing on that path commands SURFACE (on purpose:
the thrusters go off). It now says what was done. `_try_step` printed `[OK]` for
a disarm the board *refused*, because the refusal is returned, not raised.

**Also from that audit, fixed:** tracker association is order-independent
(best pair first; `cascade.py`, deferred module); a flow calibration that fails
to load now takes the whole no-calibration path, focal scaling included; the
lock node counts and logs frames it cannot decode; `bringup_check` waits on the
monotonic clock; unreachable lines in `hailo.py` deleted; docstrings that named
functions that do not exist. **Read and rejected:** "`manual()` records a NaN
axis as a known zero" -- the NaN is *sent* as 0, so 0 is the demand in force.

### B-83: the tracker shipped a 2.0 s lost-track buffer while the ledger said 5.0 s `READ 2026-11-06` **FIXED 2026-11-06**

**What.** `measured-bars.md` sized the lock ladder's last rung at `track_buffer`
= 5.00 s, covering 100 % of measured gaps against a bar of the p99 (2.418 s),
and quoted the value from `config/tracker.yaml` (150). Nothing loaded that
file: it was keyed `tracker_node`, while the vehicle's trackers are
`mongla_tracker_forward` / `_downward`, and no launch passed it. The node
shipped its own default, **60**. Roboflow keeps a lost track for
`int(frame_rate/30 * track_buffer)` frames, so 60 is **2.0 s at any rate,
below the bar**: a target hidden 2.0-2.4 s came back under a new id. Two
tests "pinned" the rung by reading the same dead file.

**Fix.** The node ships 150 (5.0 s), the one place the value lives.
`tracker.yaml` is deleted; `test_tracker_settings_are_honest.py` and
`test_bars.py` read the node's own default. A live `ros2 param set` of any
tracker setting except `classes` used to print success and change nothing; it
is now refused with "read once at startup".

**The lesson** is §9's, again: a citation is not a measurement. The ledger and
its test agreed with each other, and neither read what runs.

### B-82: the shipped `gate_sharks.hef` lost a median 0.34 of confidence to a fine-tune that diverged `MEASURED 2026-11-06 (emulator)`. **REBUILT 2026-11-06**; on-chip check owed

**What.** The forward gate detector the vehicle launches by default was
compiled at full optimisation: 1038 frames, GPU, level 2. Its
Quantization-Aware Fine-Tuning diverged (distill losses rose epoch over epoch,
and conv54 stayed pinned at 1.000). On the DFC emulator the INT8 model scores
gate frames a median **0.339** lower than its own float model. Level 0 and
level-2-without-QFT builds of the same graph lose −0.010 and −0.007. At the
shipped bar of 0.45 that turns most real gates into misses. The vehicle showed
it: detection rung 20 % on gate footage, where the `.pt` detects 100 %
(measured-bars §66).

**Why nothing caught it.** #47's guard refuses an optimiser *downgrade*. This
build had none. A HEF that compiles, loads and runs at full speed is not a
result. The repo said so in `hailo_compile.py`'s own closing message, and
the conf sweep on the HEF it asked for was never run.

**Fix.**
- `hailo_compile.py` now scores INT8 against float on held-out frames after
  the optimise, and refuses to compile when the median shift exceeds
  `MAX_INT8_SHIFT` 0.10. It reports fewer than 16 scored frames as UNVERIFIED.
- `--finetune off` disables QFT for a model whose fine-tune diverges.
- The rebuilt HEF measures −0.013 (md5 `3e9e15f0…`, provenance in
  `models/gate_sharks.hef.provenance.txt`).
- Tests are in `test_hailo_compile_refuses_a_downgrade.py`. Both new branches
  bite under injection.

**To close.** With the Pi: sync the model, check the md5 against the provenance,
run `hailortcli parse-hef`, and re-run `tools/composed_lock_harness.py` on the
same gate clip. The detection-rung share must rise well above 20 %.

### B-61: the Hailo detector ran on colour-swapped frames `REPRODUCED 2026-09-25` **FIXED 2026-09-25**

**What.** `letterbox` in `detection/hailo.py` resized the camera's BGR frame
and handed it to the chip as it was. The models are trained on RGB, so the
chip saw red and blue exchanged in every pixel for as long as the backend had
existed. Nothing raised, no frame was dropped and the rate did not change; the
detector answered a different question.

**How it was found.** Two wrong diagnoses came first, a decode stride bug and
then the DFC's calibration cliff, because both fit the symptom. What named it
was the `.pt` model run side by side on identical frames, without Hailo: fed the
same swap, it turned `gate=0.78 @56,246` into `repair=0.92 @0,0`, the HEF's
exact signature, class and origin box included. ultralytics converts BGR to RGB
internally, which is why the `.pt` path always looked healthy and every
measurement taken through it stands.

**Fix.** `letterbox` converts BGR to RGB as it resizes.
`test_hailo_channel_order.py` holds it. Timing figures measured before the
fix carry the caveat in measured-bars (the photon-to-detection row).

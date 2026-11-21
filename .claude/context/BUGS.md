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

**Fix.** The manager disarms on the edge to ENGAGED while armed, polled with
the mission watchdog at 2 Hz. Only a True reading acts: `None` is "no
second-board link". Disarm aborts the mission's verb, so its own cleanup runs.
The firmware should do this itself, at 500 Hz; added to #12. The edge-only
disarm forgot a failed attempt; B-93 replaced it with a latch.

### B-92: arm and disarm were confirmed from a heartbeat older than the command `READ 2026-11-08` **FIXED 2026-11-09**

**What.** `_arm_disarm` sent the command and returned success as soon as
`is_armed()` matched, and `is_armed()` read the last cached heartbeat however
old, with no heartbeat at all reading as disarmed. A disarm on a silent link
returned `(True, 'DISARMED')` without hearing the board; a cached ARMED
heartbeat confirmed a new arm at once; an aborted arm called itself disarmed
on no evidence. GitHub #70.

**Fix.** Arm and disarm are confirmed only by a heartbeat received after the
command (`armed_heard_since`), the rule `set_mode` already followed. Silence
is `UNCONFIRMED`, never success. The abort path re-sends DISARM every 0.5 s for
2.5 s, longer than the board's 1 Hz heartbeat period. `armed_state()` reads
True / False / None, and the watchdogs treat None as possibly armed. Tests in
`test_srot_fc.py`; four fail against the old code.

### B-93: a kill-switch disarm that failed was never retried `READ 2026-11-08` **FIXED 2026-11-09 (host)**

**What.** The B-90 watchdog stored the new kill state before disarming,
called disarm once and logged the result without reading it. A refused or
unanswered disarm left the vehicle armed while the switch stayed engaged, and
every later tick did nothing; turning the knob back would resume the move.
GitHub #71.

**Fix.** `node_support.kill_latch` holds until a fresh heartbeat says
DISARMED, re-sending every 3 s whatever the knob does (B-97 adds the
heartbeat withhold and removes the cap),
and acts on an unknown armed state too. While latched, the action server
refuses every verb but `disarm`, `stop` and `surface`, and health reads
`safety_latch FAILED`. Arming now refuses an UNKNOWN kill switch (no
second-board link) unless `allow_unknown_kill_switch` is set, for the bench
(owner decision 2026-11-09). The board-side latch is upstream (GitHub #83).

### B-94: a mission that died armed got one SURFACE request and was forgotten `READ 2026-11-08` **FIXED 2026-11-09**

**What.** The mission watchdog detached the dead mission, sent one
`surface(wait=False)` and never checked it. A refused or lost request left
the vehicle armed at depth, and the manager's healthy heartbeat kept the
board's own 5 s link-loss failsafe from firing. GitHub #72.

**Fix.** `node_support.mission_recovery` latches the recovery: SURFACE is
re-requested every 2 s, three times, each judged by a heartbeat heard after
the first request. Still unconfirmed, the heartbeat is withheld so the board's
failsafe surfaces the vehicle (owner decision 2026-11-09). It resolves when the
board reports SURFACE or DISARMED; until then ordinary verbs are refused and
health reads `safety_latch FAILED`. A manager restart no longer clears
either latch (B-97).

### B-95: an aborted torpedo shot could be revived by the next verb `READ 2026-11-08` **FIXED 2026-11-09 (host)**

**What.** `vision_align`'s fire thread polled the shared abort event, which
the next verb clears at its own entry. An abort that landed in the gap
between two shots, followed quickly by any verb, let the second shot leave.
After the align returned, `stop` and `surface` cancelled nothing, because
there was no running verb for the action gate to abort. GitHub #73.

**Fix.** `Mongla._abort_generation` counts aborts and is never reset;
`request_abort`, `stop` and `surface` bump it. A fire sequence captures it
when scheduled and cancels every channel not yet dispatched once it moves,
reported as `chN:CANCELLED`. The check runs again after the gap, right before
the write. A sequence nobody aborted still outlives its align. A write
already blocked inside `payload.fire` cannot be recalled.

### B-96: `settle=` was accepted by every board verb and dropped at the wire `READ 2026-11-08` **FIXED 2026-11-09**

**What.** The DSL and the facade forwarded `settle`, and `_relative_turn`
put it on the last leg, but `SrotMotion.move` never read it. The board ends
a turn or a dive on one sample inside its error band, so the next leg
started while the hull was still turning. GitHub #86.

**Fix.** After a successful ACK, `settle=N` sends a board HOLD
(`MOVE_HOLD`, the `pause` mapping) for N s through the same path (owner
decision 2026-11-09). An abort brakes it, SURFACE refuses it, a hold the
board cuts short fails the verb, and the result says "held N s", never
"settled": it is time held, not a measured stop. A bad value is refused
before anything is sent.

### B-97: an unconfirmed kill disarm went quiet, and a restart cleared every latch `READ 2026-11-09` **FIXED 2026-11-09 (host)**

**What.** B-93's kill latch stopped re-sending after ten tries and then did
nothing, while the manager's heartbeat stayed healthy, so the board's own
link-loss failsafe never fired and an armed move could continue. Separately,
both latches lived only in the manager's memory: restarting it cleared them
while the board could still be armed and moving. GitHub #98.

**Fix.** `node_support.kill_latch` withholds the heartbeat on its third
unconfirmed try (`KILL_MAX_TRIES`, about 6 s) and keeps re-sending DISARM
every 3 s after that, so the board's link-loss failsafe ends the move.
`node_support.boot_latch` decides on the first fresh heartbeat after start-up:
a board heard ARMED latches the new manager, refusing every verb but
`disarm`, `stop` and `surface` until the board reports DISARMED; no heartbeat
yet also refuses. The latch is decided when a goal arrives, not only on the
watchdog tick, or the manager's own first arm would read as an armed board.
The board-side kill latch is upstream (srot-control-board#41).

### B-108: a fix from a camera with no CameraInfo claimed calibrated precision `READ 2026-11-09` **FIXED 2026-11-10**

**What.** Before a camera's CameraInfo arrived (start-up, a replay without
`camera_info`), `range_to` used one in-water focal from the FOV table and
`bearing_to` read the box offset linearly across the FOV. A flat port has no
single focal: for a gate 4 m ahead and 1.4 m across, the range read 6.1 %
short and the bearing 1.6 deg off. `fix_from_prop` still published the
sigma of a calibrated reading, and `floor_range` centred a principal row that
sits 70 px off centre on the forward camera, a metre at 3 m.

**Fix.** Without CameraInfo the camera matrix is built from the camera's
measured FOV and runs through the same `rectifier_for` port model as a
calibrated one, so only the principal point is a guess. `fix_from_prop` adds
that guess's 1.264 deg (measured-bars, the forward camera entry) across the
line of sight and says `uncalibrated` in its log line, `place_prop` adds it to
the bearing term, and `floor_range` refuses. Both cameras are calibrated and
`vision_pi` passes both files, so this runs only while CameraInfo is absent.
`test_an_uncalibrated_camera_is_modelled_not_trusted.py` checks ray-traced
truth; a single focal, no widening, no refusal and no bearing term each fail it.

### B-109: tests in tools/ never ran, and the calibration solver's truth check had broken `READ 2026-11-10` **FIXED 2026-11-10**

**What.** CI collects `src/*/test` and `test/` only. Three test files sat in
`tools/`: two passing (21 tests, the free-decay and thruster-response fits) and
`test_fov_solve.py`, the calibration solver's only ground-truth check, which
had broken when `fov_solve.py` became a shim and lost the internals it called.
A further 94 bench tests sit in `tools/bench/control_bench/`, also uncollected.
Separately, 34 tools were named nowhere else in the repository.

**Fix.** The three moved where CI collects them; the solver check now imports
`mongla_vision.calibration.solver` and runs (36 s), and faking the pose spread
fails it. `tools/README.md` indexes every tool from its own docstring
(`tools/ci/gen_tools_index.py`), and `test/test_every_tool_is_indexed.py`, in the
integrity gate, fails a stale index, a tool with no description, or a test file
outside the collected tree. The control bench was the one named exemption;
CI now fetches the firmware at the commit in
`tools/bench/control_bench/FIRMWARE_PIN`, builds the bench from it and runs
its 94 tests in the ROS job, and the wire drift guard reads the same checkout
instead of skipping (#132).

### B-105: process teardown counted zombies as survivors `REPRODUCED 2026-11-09 (CI container)` **FIXED 2026-11-09**

**What.** `stop_tree` and `stop` in `tools/_record_lib.sh` asked `kill -0` whether
a process was still running. `kill -0` succeeds on a zombie, and wherever
orphans are not reaped -- a container whose pid 1 is not an init, which is the
dev box's docker and CI -- every process `stop_tree` killed stayed a zombie. It
then waited out its whole grace period and reported dead processes as
SURVIVORS. The two tests that guard teardown carried the same check, and they
failed on both ROS CI distros. On the vehicle, systemd reaps orphans, which
hid it.

**Fix.** `running` in the library treats a zombie as dead, falls back to
`kill -0` when `/proc` cannot be read (unsure means alive), and every check in
the library uses it. The stop-tree test asks the library's own `running`, so
the test covers that function too. Checked against a real zombie:
`kill -0` reported it alive and `running` reported it dead.

### B-101: a pose of one prop could clear a shot at another `READ 2026-11-09` **FIXED 2026-11-09**

**What.** `TargetPose` had no field saying which target it was solved for.
`square_within`, `obliquity_deg` and `tool_offset_px` checked only `ok` and
age, so for up to a second after a mission switched from the gate to the
torpedo board, the gate's fresh, square pose cleared the board's firing gate
and supplied its tool-range correction (#76). Walking the rest of the chain
found two neighbours on the same field: the fuse node voted poses of different
targets into one answer, and `anchor_on('gate')` took any fused pose and wrote
the gate's surveyed bearing against it, putting the world heading off by the
angle between the two faces.

**Fix.** `TargetPose.target_label` carries the class the producer OBSERVED
(the anchor's enrolment label, not the class asked for), copied by PnP on
success and refusal. The fuse node starts a new vote on a new label. Every
pose gate takes the class it acts on, and `vision_align` binds its closures to
`target_class`; an unlabelled pose cannot clear a named shot. `anchor_on`
passes the course's `detect_class`, and a pose of another prop is refused with
that reason. `test_pose_belongs_to_the_target.py`: four cases fail without the
gate and the vote reset.

### B-98: the calibration guide wrote a file wherever a request pointed `READ 2026-11-09` **FIXED 2026-11-09**

**What.** The guide's `/apply` built the destination file name from the
request's `applies_to`, and the medium from the source file's contents, with
no check. The guide listens on every interface with no login, so a profile of
`../../x` wrote a calibration JSON outside the calibration folder; with the
check disabled the test really wrote one in `/tmp`. Found by CodeQL's first
analysis (py/path-injection), with a ReDoS in `tools/ci/topic_wiring_sweep.py`
and an unchecked file name in `tools/site/upload_sink.py`.

**Fix.** `solver.check_profile` accepts a plain name (letters, digits, `_`,
`-`) and a medium of `air` or `water`, and both naming functions
(`calibration_filename`, `capture_dir`) call it, so no path is built from
anything else. `/apply` refuses before it reads the camera. The console stays
open by decision (#66); what it may write is now bounded.

### B-99: the ladder fallback steered on any class `READ 2026-11-09` **FIXED 2026-11-09**

**What.** With no live box of the requested class, `bbox_error(class, lock_s>0)`
returned the first `/lock` box without looking at what it was. `/lock` names
the rung (detection, follow, anchor) in its hypothesis and carried no class at
all, so a ladder pinned or stale on prop B steered an align asked for prop A
(#74). The existing test checked that the ladder's own class changed, not that
the consumer refused a wrong one.

**Fix.** `lock_node` puts the class it follows in `Detection2D.id`: the anchor's
enrolment label on the anchor rung, the last real detection's class on the
others. `_lock_sample` refuses a box whose id is not the requested class, and
an empty id, which cannot be verified. With no class asked, the single-target
fallback is unchanged. `test_lock_fallback_checks_identity.py` runs the real
publisher, subscriber callback and `bbox_error`; four of its cases fail on the
old consumer.

### B-100: a new target class kept the old target's box, follower and pose `READ 2026-11-09` **FIXED 2026-11-09**

**What.** A mission switching the ladder's class (through `classes_filter` or
a live `target_class`) changed `_cls` and the committed width, and nothing
else. The last detection box, the follower's points, the anchor's pose and
label and the world memory all stayed, so until a detection of the new class
arrived the ladder kept publishing the old target, its range now computed with
the new class's width (#75). A ladder tick already running at the switch could
also publish, or write the old anchor label, after it.

**Fix.** `_switch_target` bumps a generation and forgets all per-target state
under the lock; the anchor's bank keeps its references, which are labelled by
class. Each tick carries the generation it started under, and a tick that
finishes under a newer one publishes nothing and clears what it wrote. The
same class re-delivered (the filter is latched) is not a switch. With B-99, a
box published in the instant between the check and the publish still names
the old class and is refused by the consumer.

### B-102: a stale bearing kept the hull turning `READ 2026-11-09` **FIXED 2026-11-09**

**What.** `align` scaled lateral and forward by the box's authority and sent
yaw at full strength whatever its age, on the argument that the board's
heading hold absorbs a rate demand (#77). It does the opposite: a rate demand
from a bearing 0.8 s old, re-sent every tick, keeps turning the hull past a
target it already faces, and a blind frame still yawed at the cap. The test
that pinned the old behaviour asserted full yaw on a 0.30 s-old frame.

**Fix.** `_yaw_command` takes the same authority as the other axes and applies
it before the stiction snap, so a fading demand stays producible down to the
board's floor and then goes to exactly zero, instead of falling into the band
where the board produces nothing. At a healthy frame rate authority is 1.0 and
nothing changes.

### B-103: the ladder's authority came from the tracker's knob, and a box nothing steered on could end a verb `READ 2026-11-09` **FIXED 2026-11-09**

**What.** Every ladder box was marked coasted and so decayed on the tracker's
`coast_s` (#78): with `lock_s=1, coast_s=0` a healthy follower box could not
move the hull at all, and the default `coast_s=0.8` cut the ladder's own 2.5 s
horizon to 0.8 s. The ladder also published rung trust times authority as its
score, so a carried detection at an underwater score of 0.3 steered at 30 %
on a lock it trusted fully. Neighbours on the same chain: a zero-authority
box, or a Kalman prediction, sitting in band advanced `align`'s stable-frame
counter toward ALIGNED, and a predicted box at the fill ended a fill-stop
`move` as reached.

**Fix.** `Sample.source` says which of detector, tracker and ladder made the
box. A ladder box takes the ladder's authority (now what `/lock` publishes;
rung trust is a gate, as a detector score is) times the age of its own frame,
independent of `coast_s`. `_is_evidence` decides what may end a verb: a box
with authority left that is not a prediction. Anything else holds the stable
count and holds forward drive at the fill without completing the arrival.
`test_vision_authority_by_source.py` drives both loops; 14 of its 24 cases fail
on the old code.

### B-104: a gate that vanished for any reason was reported as passed `READ 2026-11-09` **FIXED 2026-11-09**

**What.** A pass-through `move` reported ALIGNED ("passed through") once a gate it
had seen left the frame and the commit ran (#79). Losing a gate is not seeing
the hull cross it: a far gate blinking out in turbidity, one sliding out
sideways and a camera outage all scored the same pass. Neighbours on the same
path: the lock ladder kept the gate "present" for up to its 2.5 s horizon after
the detector lost it, so the commit started late (pass-through already
refused the tracker's coast for this reason, but not the ladder). And both
navigation missions ran their blind transit on `not passed`, which once the
verdict is honest would stack a second open-loop leg on a commit that had
already driven past the gate.

**Fix.** The verdict comes from the last live box. A gate the hull is entering
surrounds the camera axis and runs off the frame: ESTIMATED_CROSSING, `ok`.
Anything else: COMMIT_UNVERIFIED, not `ok`. Neither is ALIGNED, and the reason
says which box decided it. The commit stays bounded by `duration`, and a
commit cut short still gets a verdict. Pass-through ignores the ladder as it
ignores the coast. `VisionResult.committed` tells a mission a commit leg was
driven, and the blind transit runs only without one.
`test_pass_through_crossing.py` replays the five sequences the issue names,
the DSL mapping and both missions; breaking the verdict, the blind-leg guard
or the ladder exclusion fails 7, 2 and 1 of its cases.

**Compromise, and the data that would remove it.** This is an estimate from
one camera. Confirming a crossing needs distance travelled after the loss
compared against the range at the loss, and the chain already has that
data: the downward camera's flow velocity (worst error 1.09 cm over 30 cm)
reaches `/mongla/odom`. TRAVERSAL_CONFIRMED is left for that, tracked on #79.

### B-106: flow published velocity at the calibrated sigma from an uncalibrated scale `READ 2026-11-09` **FIXED 2026-11-09**

**What.** A missing, unreadable, wrong-camera or wrong-aspect calibration made
`flow_node` fall back to one focal length and the frame centre, the state
measured at a 3.08 % axis asymmetry, and then publish velocity and distance
with the calibrated 3.6 % scale sigma. Localization fused it as normal and
reported `aided`. Neighbours on the same path: `Intrinsics.from_json` scales x
and y separately, so a calibration on frames of another aspect ratio (a crop)
stretched one axis with no message; a file calibrated in water would have had
the port corrected twice; and `vision.launch.py` started `flow_node` with no
calibration at all while its own `camera_node` resolved one from the profile.
GitHub #80.

**Fix.** `flow_setup._calibration_problem` names what is wrong (none passed,
unreadable, no `camera_matrix`, a medium other than air, an `applies_to` that
does not name the camera or profile) and `_aspect_problem` adds a crop at the
first frame. Any of them refuses velocity and distance through `_scale_ready`,
unless `allow_uncalibrated:=true`, which runs on the focal parameters, adds
`FLOW_UNCALIBRATED_SCALE_SIGMA_FRAC` (0.0308, ledger-bound) to the sigma and
warns. A latched `/mongla/vision/<camera>/flow_calibration` says
`calibrated` or `uncalibrated: <why>`, and localization reports
`aided (flow uncalibrated)` from it. With no path, flow binds the calibration
from its `profile` through `calibration/binding.py`, the lookup `camera_node`
already made, and both launch files pass the profile.
`test_flow_calibration_is_valid_or_refused.py` covers each case through the
real node; removing the gate, the aspect check, the binding check, the sigma
term or the aiding branch fails 7, 1, 2, 1 and 1 cases.

### B-107: "never been in water" erased the lineage, and a false water claim passed every check `READ 2026-11-09` **FIXED 2026-11-10**

**What.** The README, the site, the wiki home and CLAUDE.md said "This vehicle has never been
in water" and "WATER: nothing", which was true of the current platform and read as true of
the whole project: the Duburi stack this repository began as competed at RoboSub 2026, the
author's vision stack at RoboSub 2025, and the SROT board itself was in water in the Dubomini
hull. In the other direction, no required check caught a planted line falsely saying the
current SROT and Kutum platform had completed water trials. GitHub #87.

**Fix.** `docs/evidence/water.json` holds every time a vehicle in the lineage was in water:
platform, what ran, how well the code is identified, the evidence and whether it is still
active. `lineage.md` is generated from it by `tools/ci/gen_lineage.py`. A status belongs to the
platform it was earned on and is never inherited. Every public page now names the platform
it means. `test_water_claims_have_a_scope.py`, in the integrity gate, validates the
registry, keeps the page generated, and fails an unscoped "never in water" or a current
platform put in water without a CURRENT_WATER event backed by a recording. Planting either
line in the real README or wiki fails it.

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

### B-123: a Ctrl-C interrupted every node's teardown, inside the Hailo's release `SEEN ON KUTUM 2026-11-19` **FIXED 2026-11-19**

**What.** The first full `vision_pi` run on Kutum after the second read,
stopped the way an operator stops it, printed a KeyboardInterrupt traceback
from every Python node (the detector, both trackers, four pose solvers, both
pose fuses), and each exited -2, which the launch reported as a death with
"restart 1/3". A terminal Ctrl-C reaches every process twice (the terminal
signals the process group and `ros2 launch` forwards its own); the first
stopped the spin, the second landed inside `destroy_node()`. In the detector
it landed inside the Hailo VDevice's `__exit__`, part-way through releasing
the chip the next start needs. The same run showed every launch warning that
the tracker's `high_conf_det_threshold` (0.6) was clamped to the detector's
0.45: a clamp the shipped defaults caused themselves, printed as a WARN on
every start, which teaches an operator to read past warnings.

**Fix.** `mongla_runtime.spin.deaf_to_ctrl_c()` is the first line of
every node's teardown in `mongla_vision` and of the three localization nodes
the vision launch starts; the launch still escalates to SIGTERM and SIGKILL
if a teardown hangs. The tracker's threshold defaults to NaN, meaning the
detector's floor, and only an operator's value above the floor warns. Guards:
`test_ctrl_c_twice_is_a_clean_stop.py` and `test_tracker_settings_are_honest.py`,
both injection-verified. On Kutum, a process-group SIGINT now ends all nine
processes "finished cleanly" with no traceback and no clamp warning, and the
next start brings both detectors back (56.8 and 15.0 Hz). The manager,
planner and Batighor mains have the same shape and are left for their own
audits; the manager's Ctrl-C is its disarm path.

### B-126: a mission read a stopped localiser's last pose and last motion verdict forever `READ 2026-11-20` **FIXED 2026-11-20**

**What.** `pose()` cached the first `/mongla/odom` it heard and, once one existed, neither
pumped for a newer one nor asked its age. The localiser restarts three times and then stays
down, so every later `pose()` answered with where the hull had been. `motion()` reads a
latched topic published on change, so a dead localiser's last `'ok'` read forever.

**Fix.** Both answer None unless the odometry STAMP moved within `ODOM_MAX_AGE_S` (0.5 s,
five periods of the 10 Hz timer), the rule the lock's loop closure already used; the bound
now lives once, beside the timer, in `localization_publish.py`. Guard:
`test_a_stopped_localiser_gives_no_pose.py`, a real publisher with a frozen stamp;
injection-verified. BUILT, NEVER FLOWN.

### B-127: a vision placement froze itself into the course as a tape survey `READ 2026-11-20` **FIXED 2026-11-20**

**What.** `place_prop(commit=True)` wrote through `course_survey.survey`, which always set
`measured: true`. A measured prop is only ever checked, never overwritten, so one placement
from a drifted pose became permanent, and the next run's placements could only log that
they disagreed with it.

**Fix.** `survey(measured=...)`: the CLI's tape survey writes true; a placement writes
false, with its sigma in the note, so a later placement refines it. Guard: the commit test
in `test_pose_and_prop_fix.py`, which asserted the defect before.

### B-129: on a headless Kutum the viewer took the whole vision launch down `REPRODUCED 2026-11-20` **FIXED 2026-11-20**

**What.** `vision_dual` defaulted `viewer:=true` and shuts the launch when the viewer exits
(on a desk, `q` quits everything). Kutum has no screen: the viewer aborted (exit -6) and
cameras, detectors and trackers stopped 5.5 s after start, cleanly, so the log read like an
operator Ctrl-C. Every footage run there ended before a frame was detected.

**Fix.** `viewer` defaults to whether there is a LOCAL screen (`launch_defaults.has_screen`:
a DISPLAY of the form `:N`, or WAYLAND_DISPLAY), in both vision launch files; `viewer:=true`
typed still wins. An `ssh -X` display (`localhost:10.0`) is not local: it goes over the
tether, and pulling the tether would take the viewer down with it. And only a CLEAN viewer
exit (code 0, the operator pressing `q`) ends the launch (`viewer_quit_shuts_launch`); a
viewer that crashes takes only itself. Guards: `test_a_headless_launch_starts_no_viewer.py`,
`test_only_a_clean_viewer_quit_ends_the_launch.py`, both failing on the old code. Re-run
on Kutum with the viewer off: forward 24.9 Hz of `gate` detections on recorded footage.

### B-131: a stack started over ssh died with the tether `REPRODUCED 2026-11-20` **FIXED 2026-11-20**

**What.** A run is tetherless: start over the tether, count down, pull it. A process started
in the foreground of an ssh shell, outside tmux, is hung up when the session goes.
Reproduced on Kutum: a process in an interactive ssh shell died when the ssh client was
killed; the same process inside tmux lived. Nothing in the stack knew or said so, and the
countdown went ahead.

**Fix.** `mongla_runtime.tether.tether_bound()` says why a process would stop ('' when it
would not: no ssh, tmux/screen, nohup, or no controlling terminal). `spin()` declares it on
every node as the read-only parameter `tether_bound` and warns at start-up; the DSL
`countdown()` asks every `mongla*` node and refuses while any is bound, naming it. Checked
on Kutum: the ssh shell reads bound, a tmux shell reads ''. Guards:
`test_a_process_knows_if_the_tether_kills_it.py` and the countdown tests in
`test_no_board_verb_before_arm.py`.

### B-132: the localiser said "aided" before any input, and a never-thrust hull waited 3 s to ZUPT `REPRODUCED 2026-11-20` **FIXED 2026-11-20**

**What.** Replaying the real bench-rest recording: the node logged "aiding restored" before
its first IMU sample (the cold covariance read as an observation), and the first ZUPT came
only after the 3 tau coast settle, although nothing had been commanded since start, so
there was nothing to coast out of: 0.71 m dead-reckoned in the first 2.7 s (1x replay),
0.08 m with the fix.

**Fix.** No aiding verdict until an input has been fused; the settle applies only after a
non-zero demand has been seen. Guards in `test_localization_node.py`
(`test_aiding_is_not_reported_before_any_input`,
`test_a_hull_never_thrust_is_settled_from_the_start`), both injection-verified. Numbers:
measured-bars 99.

### B-133: the floor drift bounds held a constant residual at zero `REPRODUCED 2026-11-20` **FIXED 2026-11-20**

**What.** `_on_floor_grid`'s docstring says the residual between heading and grid is
constant; `snap_to_grid` was handed the raw image angle and so pinned that residual at
zero. It is zero only when the camera mount, the image-angle convention (a lane along
the hull reads 90) and the feature's pool direction cancel. Rendered through the real
detectors: every lane read along the pool's x axis was refused, and under a 37 deg mount
the grid was refused too. The existing tests fed the filter its own yaw as the "image"
angle, so they measured agreement and could not see it.

**Fix.** The first floor read after each anchor records the residual; later reads bound
drift against it, and a reference is not counted as a correction. Truth test
`test_a_rendered_floor_bounds_drift_under_any_mount` (lane and grid), failing with the
residual held at zero.

### B-134: the uplink tick stalled the manager's telemetry for 10 s `READ 2026-11-20` **FIXED 2026-11-20**

**What.** `_vision_uplink_tick` called `_vision_state_for`, whose first build waits up to
10 s for a detection, on `timer_group`, shared with `telemetry_tick`, `_health_tick` and
`board_link_tick`. Setting `vision_uplink_camera` froze `/mongla/state` for 10 s. The
heartbeat thread was not affected.

**Fix.** The uplink builds its VisionState with `preflight=False`; vision verbs keep the
warm-up. Guard `test_the_uplink_builds_without_the_preflight_wait`.

### B-135: the accelerometer-bias prior was 17 sigma off the board's measured bias `REPRODUCED 2026-11-20` **FIXED 2026-11-20**

**What.** `RIEKF` used one prior, `P0_bias = 0.01`, for both biases. The ledger's static
IMU table measures this board's accelerometer bias at 0.167 m/s^2, so the filter could not
learn it, and every ZUPT corrected the same biased velocity again. On the real bench
recording, replayed at 1x on Kutum, a still hull walked 15.5 cm in 90 s.

**Fix.** `P0_accel_bias` is its own argument; the node passes `ACCEL_BIAS_PRIOR = 0.2`.
Walk 0.94 cm on the same replay, and no worse in motion on the truth study (measured-bars
99). Guard `test_the_accel_bias_prior_covers_the_measured_bias`, failing at 0.01.

### B-136: one fix in, the filter was certain and wrong, and refused the next correct fix `REPRODUCED 2026-11-20` **FIXED 2026-11-20**

**What.** Flow's sigma is per-sample noise; integrated, it grows like a square root of time.
A flow scale error grows with distance. With nothing modelling the second, the position
sigma after a 0.15 m fix stayed at 0.153 m across 15 m of 5 %-scale flow while the estimate
drifted 0.5-0.7 m, and the chi-square gate refused the next correct fix 3 times in 5
(`tools/studies/localization_candidates.py`, against exact truth). `go_to` would have
arrived at the wrong place reporting 0.15 m.

**Fix.** `odometry_scale_sigma`: the horizontal position sigma is held at no less than
sqrt(sigma_at_last_fix^2 + (k x distance since)^2), with k = `FLOW_SCALE_SIGMA` = 0.036,
the worst slide in measured-bars 13 (first shipped as 0.035, a hand-rounded second copy of
mongla_vision's `FLOW_SCALE_SIGMA_FRAC`; one constant now, `test_one_scale_error.py`). 15 of 15 accepted. Guard
`test_scale_error_floor.py`, failing with the floor removed. Not covered: a sloping floor's
scale error (bead es5.7).

### B-137: a refused position fix was silent `READ 2026-11-20` **FIXED 2026-11-20**

**What.** `_on_fix` returned without a word when the gate or the replay horizon refused a
fix. `fix_from_prop` had already reported success, so a mission believed it had fixed
position, and the counter was the only place the truth showed.

**Fix.** A `fix_refused` counter in the diagnostic line (`fix=accepted/offered`) and a
warning naming the fix, the estimate and both sigmas. Guard
`test_a_refused_fix_is_counted_and_said`.

### B-138: a zero quaternion was fused as a level hull `READ 2026-11-20` **FIXED 2026-11-20**

**What.** `_R_from_quat` returned the identity for a quaternion of zero length, and the
node fused it at the board's 0.5 deg attitude sigma: a level hull facing the boot heading,
from a message that carried no attitude at all. Found by line coverage; no test had sent one.

**Fix.** It returns None for a zero or non-finite quaternion, and the sample is skipped
like one whose covariance says "no orientation". Guard
`test_a_zero_quaternion_is_no_attitude_not_a_level_hull` (injection-verified).

### B-139: a hull pushed along with the gyro quiet read as still `REPRODUCED 2026-11-20` **FIXED 2026-11-20**

**What.** The ZUPT's accelerometer veto took the spread of |a|, the magnitude, chosen
because tilt cannot change it. Neither, to first order, can a horizontal push: +-0.4 m/s^2
across moves |a| by 0.008, so a hull shoved along a line with the gyro quiet was declared
still and its real velocity zeroed. The branch had never run in a test.

**Fix.** The spread is taken per axis on the body's own acceleration, gravity removed with
that sample's board attitude. A/B on the real bench recording on Kutum (zupt on, 240 s):
232 and 232 ZUPTs, still-hull walk 9.3 and 9.4 mm, so no cost at rest. Guard
`test_a_hull_shaking_without_turning_is_not_still`, which failed against the old code.

### B-140: the published test counts froze when two packages arrived `REPRODUCED 2026-11-20` **FIXED 2026-11-20**

**What.** A full run publishes `docs/stats.json` only when the suites it ran equal
`published_counts.SUITE_NAMES`. `mongla_runtime` and `mongla_batighor` came with tests and
were never added, so from then on no run counted as full and nothing was published. The
guard kept comparing the README and the site with that frozen file and passed: the pages
said 4 742 tests long after the suite held more than 5 300.

**Fix.** Both suites are listed. Guard `test_every_package_with_tests_is_a_published_suite`
fails when a package's tests are not a published suite; it failed with the two names removed.

### B-130: the old-host guard matched `mongla.localization()` `READ 2026-11-20` **FIXED 2026-11-20**

**What.** `test_kutum_has_one_name` grepped a fixed string, so the DSL call
`mongla.localization()` read as the retired host name.

**Fix.** It matches the name with no identifier character after it; a file naming the
real old host still fails it (injection-verified).

### B-128: course and frames files silently dropped what they did not recognise `READ 2026-11-20` **FIXED 2026-11-20**

**What.** `bearing: 90` for `bearing_deg: 90` loaded as "bearing unset", and the anchor
refused later naming the right key; `downwards_cam:` loaded as "not measured" and a taped
lever arm was never applied; a quoted `measured: "false"` read as True.

**Fix.** Both loaders refuse an unknown key (naming the nearest known one) and a
non-boolean `measured`. Guard: `test_a_misspelt_course_key_is_refused.py`, four of seven
failing on the old code.

### B-124: the localiser counted a refused measurement as fused, and could stay silent while unaided `READ 2026-11-20` **FIXED 2026-11-20**

**What.** `LocalizationInputsMixin._apply` dropped the update's own answer and
returned True for anything it handed the filter. A flow sample the chi-square
gate rejected, a depth or yaw the B-56 observability gate refused, a ZUPT or a
model aid the filter rejected: each was counted as fused. The diagnostic line
over-reported every channel, and the one operator warning that matters, "NO
VELOCITY AIDING", is computed from the flow and ZUPT counters, so a stream of
rejected flow kept it silent while nothing aided velocity.

**Fix.** `_apply` returns False when the update returned False (or the sample
was older than the replay horizon); the answer kept is the first run's, since
a later replay may run the same update again. Every counter, and the floor
grid and lane line, count on it. Guard:
`test_a_refused_update_is_not_counted_as_fused.py`, four of its five tests
failing on the old code, with and without retrodiction. BUILT, NEVER FLOWN.

### B-125: the fused target pose published zeros for what it never fused `READ 2026-11-20` **FIXED 2026-11-20**

**What.** `pose_fuse_node` filled yaw, range and the spread and left every
other float of `TargetPose` at its default 0.0. Batighor plots the fused
`bearing_deg` and `reproj_px`, so the operator saw a bearing of exactly zero on
every fused pose, and a yaw and range of zero whenever the fuse was undecided.
Missions were safe: `dsl_heading` reads only a pose with `ok` set.

**Fix.** `fused_message` sets every float the fuse does not compute to NaN.
Guard: `test_fused_pose_says_nothing_it_did_not_fuse.py`, which also fails if
a float is added to the message without a decision here; injection-verified.

### B-122: loop closure could not start, and the anchor pose put the object's origin outside the target `READ 2026-11-19` **FIXED 2026-11-19**

**What.** Two defects found by the second line-by-line read of `mongla_vision`.
`loop_closure:=true` raised in the lock's setup on every camera, because two
mistakes sat one behind the other: it read `self._camera`, which nothing sets
(the node keeps `self._cam`), and behind that it subscribed to odometry with
`_qos.sensor()`, which the qos module does not define. Every closure test read
source text or drove the policy object alone, so none ever stood the node up
with the switch on: the capability was built, tested and unreachable. And the
anchor's metric pose built its object points as `(ref - centre) * width_m /
w_px` with the centre and width of the reference ROI, but the reference points
are in full-frame pixels, so any ROI not at the origin shifted the object
plane by its offset and every `pnp_node` range and bearing was wrong by it.

**Fix.** The setup reads `self._cam` and subscribes with depth 10;
`pose.patch_object_points` takes the ROI's own centre in frame pixels and is
what `lock_publish` calls (the old `target_pose`, used only by tests, is
gone). Guards: `test_lock_node_stands_up.py` stands the node up with the switch
on, on both cameras, and walks a whole closure through `consider` to the fix
message; `test_anchor_pose.py` places an offset ROI and recovers the true
range. Still BUILT, NEVER FLOWN: the real matcher has never closed a loop on
the vehicle, and any earlier claim that loop closure works predates this.

### B-120: the localiser published a pose before it had any input `READ 2026-11-19` **FIXED 2026-11-19**

**What.** `/mongla/odom` is published from a 10 Hz timer. Before the first
input it carried the filter's initial state, the origin, stamped `now()`: a
confident pose describing nothing, with a stamp that moved, so the lock's
freshness gate (measured-bars section 91) passed it, and with the board clock
mapped the manager's position uplink would have forwarded it.

**Fix.** No input, no odometry. Guard:
`test_localization_node.py::test_no_input_publishes_no_odometry`.

### B-119: two values the vision stack computed and nothing read `READ 2026-11-19` **FIXED 2026-11-19**

**What.** The lock kept the frame-edge assessment in `_vis_state` and only a
test read it; the warning the operator sees printed the state and the margin
but not the bearing the test checked, so the corrected bearing (the grid
scaling fixed beside B-118) reached no one. And `xfeat_hailo` kept the HEF's
reliability head as `last_reliability`, read by nothing since it was added.

**Fix.** The warning prints `bearing=` and the tests read it from the
warning; `_vis_state` is gone. The reliability head now ranks the keypoints on
both backends, as upstream XFeat does (B-121).

### B-121: the anchor's XFeat was not upstream's, and semi-dense was scored by its own fit `READ 2026-11-19` **FIXED 2026-11-19**

**What.** Three silent departures from upstream `detectAndCompute`: the top-k
ranked by keypoint probability without the reliability head; descriptors
sampled bilinearly from the raw map instead of bicubically from the
normalised one; keypoints kept on the last row and column. Together they cost
the anchor 28 % of its correct matches against known homographies on warped
frames, and 8 % on real pairs seconds apart (measured-bars section 92). Separately,
`anchor_semi_dense` was justified by 2.1-2.6x MAGSAC inliers, a count the fit
produces from the matches it is judging; against the truth it fits the view
change half as often as sparse.

**Fix.** The shared `keypoints_and_descriptors` follows upstream exactly
(16 of 16 real frames, descriptors to 8.8e-7), and both backends pass the
head. Bank files move to version 2, so a bank of the old descriptors is
refused and relearnt. Semi-dense is removed. Numbers and the rejected
alternatives: measured-bars section 92.

**Guards.** `test_xfeat_is_upstream.py` (upstream's own answer, recorded;
injection-verified for each behaviour); the study's `--self-test`.

**How it survived.** Every earlier XFeat number was an inlier count, which
rises with any change that makes matches agree with each other, right or not.
The comment that justified bilinear compared it only with nearest.

### B-141: `?layout=` opened a personal Default on a browser's first visit `REPRODUCED 2026-11-19` **FIXED 2026-11-20**

**What.** `http://kutum:8000/?layout=chain` in a browser profile that had never
opened Batighor showed a personal "Default" layout; the organisation's chain
layout was listed and had to be opened by hand. A second visit opened it.

**Cause.** Upstream, Lichtblick 1.30.0 `CurrentLayoutProvider`: the `layout=`
name is matched against the layouts the app already holds, and it waits only
if the layout manager is busy. `LayoutManagerProvider` syncs only while the
tab is visible. A link opened in a background tab on a fresh profile has no
sync running, so the organisation's layouts are absent, the match fails and a
personal "Default" is made and selected. The sync stores the layouts when the
tab is shown, which is why the second visit works. A foreground tab on
localhost, even with 150 ms of emulated latency, did not reproduce it.

**Fix.** `serve.FIRST_VISIT_HEAD`, served in the app page: when the named
layout is not in the app's layout database at load, it waits for the sync to
store it and loads the page once more (a session flag stops a loop; 15 s cap).
A browser that already holds the layout is untouched. The database is opened
only after the app has created it, because opening it first would create it
empty and the app's upgrade would never make its store.

**Measured** (fresh headless Chrome profile, the link opened in a background
tab, then shown; Kutum's Lichtblick 1.30.0 served from this tree): without the
fix the app selected a personal Default; with it the page reloaded once and
opened chain. The unused personal Default is still created by the first load.

**Guards.** `test_serve_and_fetch.py`: the page carries the script inside its
head, and the script never opens the database before the app has made it.

### B-142: every test launch left a parameter file in /tmp, forever `REPRODUCED 2026-11-20` **FIXED 2026-11-20**

**What.** The dev box's /tmp, a 7.8 GB RAM disk, filled to 100 % and broke
every command that wrote a temporary file. It held 784,505
`launch_params_*` files, about 3 GB of tmpfs pages for 163 MB of YAML, written
from 2026-11-10 on. Kutum held 628.

**Cause.** launch_ros writes the parameter dict of every `Node` to
`NamedTemporaryFile(prefix='launch_params_', delete=False)` and nothing
removes it. The suites render every launch file and switch combination, so one
file of them leaves 1,055.

**Fix.** The root `conftest.py` gives each session its own TMPDIR, which every
launch subprocess inherits, and removes it when the session ends. pytest's own
temp root is pinned first (`PYTEST_DEBUG_TEMPROOT`), so `tmp_path` keeps its
place and its retention.

**Measured.** `test_every_launch_file_renders.py`: 1,055 new files in /tmp
without the fix, 0 with it, and no session directory left behind.

**Guards.** `test/test_launches_leave_nothing_in_tmp.py`.

### B-143: a run with no floor started, and navigated on the IMU alone `READ 2026-11-21` **FIXED 2026-11-21**

**What.** Flow needs the height above the floor to turn pixels into metres,
and the vehicle measures only its own depth. With no floor depth set, flow
refused every interval, localization ran on the IMU alone, and the mission
started and reported nothing wrong. `bringup_check` warned, and the warning
scrolled past. On a sloping floor one typed depth was a scale error of up to
0.72 m in 1.7 m (1.28 to 2.00 m over 17.5 m). The venue's numbers were spread
over three places: the deck file, the course and the geometry override.

**Fix.** The course file is the venue's one file (`missions/course-file.md`):
pool depth and range, floor soundings, tile size, lane lines, the depth floor
and every prop's measured width. Bringup takes them from the deck's `course:`
and refuses a deck file that repeats one. Flow follows the surveyed floor
under the hull (`flow_math.floor_depth_at`), but only once localization has
an anchored heading and a prop position fix. It publishes its height source,
latched, and the mission runner refuses to start on `none`
(`--allow-no-height` for a bench). An unset depth floor is printed at every
mission start.

**Measured.** Synthetic slope, position known to 1 m: RMS height error under
0.05 m against up to 0.72 m for one typed depth. The first bar, worst case
under 0.10 m, missed at 0.107 m; the test says why.

**Guards.** `test_floor_depth_follows_the_slope.py`,
`test_a_run_without_a_floor_does_not_start.py`,
`test_the_course_is_the_venues_one_file.py`,
`test_course_floor_reaches_flow.py`, and the course cases in
`test_mission_keepalive.py`. Each was injection-verified.

### B-144: a lane line in view gave the tile grating a confident wrong height `REPRODUCED 2026-11-21` **FIXED 2026-11-21**

**What.** Flow takes its height from the tile grating before every other
source. One dark lane line across the tiles carries more spectral energy than
the whole grid, and the grating read it instead: rendered tiles with a lane
read a period near 100 px whatever the truth, up to 146 % wrong, and refused
most of the rest. Every tiled pool has lane lines. Its tests drew tiles only,
so nothing failed. A second, smaller fault sat under it: the three-point
parabola that placed the peak read a lane-free floor at 0.34 % median, 0.9 %
worst, and a lane's band dented the peak so the parabola fit the dent.

**Fix.** `tile_grating._blank_thick_dark` fades dark stripes (darker than
0.45 of the floor's median and at least four times longer than wide) to the
floor's median before the FFT; a dark cell, a vignette corner or a round
shadow is left alone, because fading those made a gradient pass as a grating.
The peak is placed by the power centroid of its main lobe (`_centroid`).

**Measured** (measured-bars section 101). Lanes 0.3x to 0.8x the tile period:
95-100 % answered, median 0.12-0.27 %, worst 1.59 %. Lane-free, 780 floors:
0 wrong, 0 refused, median 0.023 %. Real final-run footage: unproven, see the
section (its lane witness and its fisheye lens could not grade it).

**Guards.** `test_tile_grating.py`: the lane cases, the answer rate and the
peak accuracy. Injection: the old module fails 15 of them, the new one
without fading 20, fading any dark blob fails a negative control.

### B-148: the floating PM1 pin reached /mongla/state through get_battery() `REPRODUCED 2026-11-22` **FIXED 2026-11-22**

**What.** On Kutum with the real board (USB power, nothing on GPIO36), the
state strip and Batighor's PM1 plot read 1.39, 2.47, 13.77, 14.60 V within a
minute. `battery_trusted` already condemns a pin whose median step exceeds
2 V, but only `get_batteries()` asked it; the manager builds the state from
`get_battery()`, which read the raw cache.

**Fix.** `get_battery()` asks the same guard, so the unwired pin is NaN (`--`).
That was not enough on the day: the pin now holds a value for seconds between
jumps (60 s on Kutum, one sample a second: 41 changes, sitting at 13-14 V like a
pack, dropping to 1.39-5.86 V and back), so its median step was 0 V and the
median passed it. The guard also counts jumps past the 2 V bar in its window:
three condemn it (`BATTERY_MAX_JUMPS`); a pack never makes one and a single
glitch makes two. Condemned, a line stays condemned until a whole window passes with no
jump: the pin's quiet spells (1.39 V for 14 s) had let it back in.

**Guards.** `test_battery_plausibility.py::test_the_battery_on_the_state_topic_is_guarded_too`
and `::test_a_pin_that_holds_a_value_between_jumps_is_not_trusted` (today's
sequence), each failing without its half of the fix.

### B-147: under replay the composed lock starved both detectors `REPRODUCED 2026-11-21` **FIXED 2026-11-21**

**What.** On Kutum's bench (`replay:=true`, two looped clips at 30 fps,
detectors unpaused), Batighor's boxes trailed the video and the lock flickered
"silent". Measured with a foxglove client for 15 s: forward detections 2.7 Hz
and downward 0.4 Hz with the lock composed, against 22.9 and 16.2 Hz with
`lock:=false`. py-spy put every thread of the vision process in a wait for a
frame; the executor thread sat idle. Under replay the detector, the lock and
the flow each subscribed to `image_raw` themselves (the detector by keeping
`direct_feed` false). A stand-alone check showed the cost of readers in one
process: three nodes subscribed to the same 640x480 image on a multi-threaded
executor received 8.7 Hz each where one received 21.7. Ruled out: the chip
(`anchor_hz` 0.5 against 4.5 changed nothing; measured-bars 95), and the
executor kind (the classic one gave 4.8 Hz).

**Fix.** `_ReplayFeed` in `detector_dual_node.py`: one reader per camera,
decoded once and handed to the same `_Tee` a live camera feeds, and the
detector is `direct_feed` in both modes. Replay now exercises the live
by-reference path. After it, the same bench: forward detections 10.9 Hz,
downward 12.0, the lock 11.8 and 13.5 Hz. Guards:
`test_replay_reads_each_image_once_and_feeds_everything_by_reference` and
`test_the_detector_is_fed_by_reference_live_and_under_replay`, both
injection-verified.

**What was left was not replay.** The bar was within 20 % of the lock-off
rates (18.3 and 13 Hz), and forward still failed. A GIL profile of the composed
process (`py-spy --nonblocking --gil`) put a fifth of its held-GIL samples in
the anchor rung: XFeat's keypoint and descriptor extraction and the bank's
matching, in NumPy, in the process both detectors run in. With only
`lock_anchor` off, the same bench reached 21.8 to 22.0 Hz forward and 20.9 to
22.7 Hz downward, at the lock-off rate, and 12.5 to 14.6 Hz with it on again
(A-B-A, measured-bars 104). So replay no longer costs anything a live camera
does not. The anchor's cost is the same on the vehicle, and is its own item
(bead `mongla_ws-7ue`). The earlier "ruled out: the chip, `anchor_hz` 0.5
changed nothing" was measured while the extra readers held the rate at 2.7 Hz,
where nothing else could show.

**Also seen.** A blocking `py-spy record` (it stops the process to sample)
across the whole launch was followed within a minute by the Hailo-8 leaving
the PCIe bus (`Channel 2 hw state out of sync`, then `Device disconnected while
opening device` on every open). No undervoltage, 57 C. Recovered without a
reboot: `echo 1 | sudo tee /sys/bus/pci/devices/0000:01:00.0/remove`, then
`echo 1 | sudo tee /sys/bus/pci/rescan`. Use `py-spy record --nonblocking` on
anything that holds the chip; that run kept it.

### B-146: a typed course drew its map but set none of its venue `REPRODUCED 2026-11-21` **FIXED 2026-11-21**

**What.** `bringup.launch.py` read the venue (pool depth, tile size, the depth
floor `shield_max_depth_m`) only from the deck's `course:`. Typed as
`course:=bench_demo` with no deck entry, the argument reached Batighor's
course node, so the map drew that pool, while the manager started with
`shield_max_depth_m` 0: no depth floor, under a map that showed one. Seen on
Kutum's bench: the course node logged the course, the manager logged
`[SHLD ] no depth floor`.

**Fix.** `launch_defaults.typed_value('course')` wins over the deck's
`course:`, as every typed argument does, and the venue comes from the course
it names. `test_a_typed_course_brings_its_venue_too` (injection-verified:
fails with the deck-only read). On the bench afterwards: `venue from course
bench_demo ... shield_max_depth_m=1.8`, `[SHLD ] depth floor 1.80 m`.

### B-145: a declaration was refused as a live change, and five kinds of node died at start `REPRODUCED 2026-11-21` **FIXED 2026-11-21**

**What.** rclpy runs a node's on-set-parameters callbacks when a parameter is
first declared, not only when it is changed. `spin` declares `tether_bound` on
every node after the node is built, and every callback that refuses the names
it cannot apply live (`read once at startup`) refused that declaration too. On
Kutum, `bringup.launch.py replay:=true batighor:=true` against the fake board:
every tracker, PnP and pose-fusion node raised `InvalidParameterValueException`
at start and respawned forever, so the lock ladder had no tracks and nothing
reached Batighor's overlay. Every unit test passed, because none started a node
through `spin`.

**Fix.** `mongla_runtime.tether.changes(node, params)` keeps only the names
already declared. Every registered callback (`live_params.apply_live`, the
camera, the tracker, the detector, the lock ladder, the manager) drops
declarations first and returns success when nothing is left, so it neither
refuses a declaration nor tries to apply one (`apply_live` raised `KeyError`
on it). A real change to a fixed name is still refused.

**Guards.** `mongla_runtime/test/test_declaring_is_not_changing.py`: a node that
refuses fixed names survives `tether.declare`; a real change is still refused;
every callback in `src/` that says `read once at startup` goes through
`params = changes(` (found by its `add_on_set_parameters_callback` registration,
not by its wording: the camera said `read once when the camera opens`); and
`apply_live` survives a declaration. Injection: reverting the camera and
`apply_live` fails both of the last two.

### B-118: the anchor rung's box was in the backend's 320x240 grid on a 640x360 frame `READ 2026-11-18` **FIXED 2026-11-18**

**What.** The anchor resizes every frame to its backend grid (XFeat, 320x240)
before it finds features, and `Anchor.snap` scales the detector's box into that
grid. `lock_anchor._anchor_box` built the anchor rung's box from the warped
corners and never scaled it back. The ladder, the follower and `/lock` work in
camera pixels (640x360 on the vehicle, a different scale and aspect), so on the
vehicle the anchor rung published a target at x=600 as x=300, its disagreement
check against the follower compared boxes in two grids (so it tripped and cut
authority whenever both ran), and its reseed restarted the follower on the
wrong patch. `tools/studies/pipeline_visual_check.py` scaled the corners
correctly, so every offline anchor measurement was right and the node was not.
No test fed the node's box path a frame of a size other than the backend's.

**Fix.** `_anchor_box` multiplies the corners by frame size over backend size.
Guarded by `test_anchor_box_is_in_camera_pixels.py` (a real `Anchor`, a
stand-in backend with XFeat's resize contract, enrolled and located on one
640x360 frame; with the scaling removed the box enrolled at x 440-600 comes
back at 220-300). Same pass: the lock's field-of-view warning fed `assess` the
backend `K` beside full-frame pixels, so its bearing angles were off by the
same ratio (`test_lock_visibility_bearing.py`).

**On Kutum, 2026-11-18.** The Sonix downward camera, still, at 640x360: a box
snapped and located on the next frame through the shipping `_anchor_box`, on
the Hailo-8 HEF and on the ONNX path, two boxes each. Bar set before the run:
every edge within 10 px. The release before the fix returned (400, 100, 600,
300) as (200, 67, 300, 200), worst edge 300 px, on both backends; the fixed
release returned it within 1.7 px (HEF) and 2.4 px (ONNX), and the left box
within 2.5 and 4.0 px. Inliers 75-97. Still BUILT, NEVER FLOWN: the rung's
behaviour in water is not measured.

### B-117: every loop closure was refused as a self-closure, because the guards read the newest place, not the matched one `READ 2026-11-18` **FIXED 2026-11-18**

**What.** `lock_inputs._place_tick` fed `loop_closure.consider` the age of the
most recent enrolment (`now - _places._last_t`) and the distance from it
(`travel_since`). Places are taken every 5 s or 1 m, so at recognition time the
age was at most about 5 s against a 30 s bar and the travel under 1 m against
1.5 m: every closure was refused as "would close on itself", whatever the
vehicle swam. The reference sigma was the filter's current one, with a typed
1.0 when the covariance was missing. Nothing ever exercised the node's
arguments, only `consider` with hand-picked ones, so the path looked complete
and could not fire ("never fired, not once" in first-pool-verification section 6).

**Fix.** `PlaceLog` keeps every place it took with its time, position and
sigma, and tracks the furthest the vehicle has been from each (not the path
length, which grows with the filter's jitter while holding station). The node
asks `since(ref_position)` for the place the bank matched, and takes no place
while the filter publishes no sigma. A bank loaded from disk has no history and
refuses. Guarded by `test_loop_closure_refuses.py::test_a_revisit_of_an_old_place_closes`
(an 80 s loop with truth known by construction; the old expressions refuse it
with "reference is 4.5s old") and
`test_loop_closure_is_actually_wired.py::test_the_guards_read_the_matched_place`
(fails with the old line put back). Still BUILT, NEVER FLOWN: a closure needs
water, a working barometer and the floor height.

### B-116: Batighor on Kutum served no layout and no font, and a panel half-installed stayed so `READ 2026-11-18` **FIXED 2026-11-18**

**What.** Kutum builds with `--symlink-install`, so every installed layout and
theme file is a link back into the workspace (`src/mongla_batighor/`, and the
fonts into `docs/assets/fonts/`). `serve.index()` drops any file whose real path
leaves the directory it serves, so on the vehicle every `/layouts/<view>.json`
and every font answered 404: an operator's browser and the desktop app got no
view. The bench never saw it, because the tests built a plain tree. Then
`tools/pool/batighor_desktop.py` died on the font 404 between a panel's
`package.json` and its `extension.js`, and every later run skipped the
directory because it existed, so the calibrate view read "Unknown panel type"
for good.

**Fix.** A link may land in the colcon workspace the package was built from
(found from where its own `package.xml` link lands); any link out of that
workspace still stays out. The desktop tool builds each panel beside its
target and renames it in whole, and an incomplete directory is moved to the
attic and rebuilt. Guarded by
`test_serve_and_fetch.py::test_a_symlink_install_serves_its_layouts_and_nothing_else`
and
`test_batighor_desktop_holds_each_view_once.py::test_a_panel_is_whole_or_absent_and_a_half_one_is_repaired`
(both injection-verified). Seen and verified on Kutum: 404, then 200 for the
layouts and fonts, and all ten views shot from the desktop app.

### B-115: one target width outlived the class it was measured for `READ 2026-11-18` **FIXED 2026-11-18**

**What.** `target_width_m` (launch arguments `target_width_m_forward` and
`target_width_m_downward`, and a `ladder.yaml` key) set one width for the
lock's whole run, and it won over the per-class table. A mission switches
target class on every vision verb (gate, then a bin), so after the first
switch the 6-DoF pose scaled the new prop by the old prop's width: a
confident range off by the ratio of two props, with nothing logged.
`_retarget_width` re-resolved the table on a switch only when the parameter
was unset, so the one case an operator had measured was the one that broke.

**Fix.** The parameter, both launch arguments and the ladder key are gone.
The width is `width_for(class)` for every class the lock follows; a measured
prop goes in `~/.mongla/target_geometry.yaml` under its class, where it
cannot outlive a switch. Guarded by
`test_lock_follows_the_mission_class.py::test_a_measured_width_follows_its_class_through_a_switch`
(injection-verified: keeping the old width on a switch fails it).

### B-114: the tracker's coast windows were frame counts, so their length moved with the detection rate `READ 2026-11-18` **FIXED 2026-11-18**

**What.** Both tracker windows were frames. The Roboflow buffer lasts
`int(frame_rate / 30 * track_buffer)` update calls and the Kalman expiry
`ceil(max_predict_s * frame_rate)` calls, with `frame_rate` a hand-set
parameter (20 by default, 30/32 from the Pi launch). Detections arrive at 30 to
58 Hz depending on load, so the documented 5.0 s lost-track window was 1.7 to
3.3 s on the vehicle. Offline, `tools/studies/tracker_ab.py` took the
container's nominal rate, which is 1000 on every Cockpit recording (gate.mkv:
6142 frames over 191.8 s, 32 fps), so every tracker comparison on those clips
measured gaps and coasts 31x off.

**Fix.** Both windows are seconds on the frames' capture stamps: the wrapper
drops a coasted box `track_buffer / 30` s after its last real detection and the
smoother expires a track after `max_predict_s` s. `frame_rate`,
`frame_rate_warn_ratio`, `fwd_frame_rate` and `dwn_frame_rate` are deleted, and
the laptop launch no longer overrides `track_buffer` to 30 (1.0 s, below
`max_predict_s`). `tracker_ab.py` reads frame times from the container's
packets. Guards: `test_roboflow_tracker.py::test_the_coast_window_is_seconds_on_capture_time_at_any_rate`
and `test_coast_ladder_ordering.py`, both injection-verified.

**How it survived.** A rate warning compared the measured rate with the
parameter and asked the operator to fix the parameter; it fixed the symptom
twice (2026-09-09 among them) and left the unit wrong. A tracker with a short
coast still publishes and looks healthy.

**Not settled by this.** On gate.mkv with real times, presence is 25.7 %
detector-only and 34.6 % (OC-SORT, shipped) to 58.8 % (ByteTrack) tracked. That
ranks nothing: presence counts predicted boxes, so a tracker that coasts past
the gate's departure scores higher. Ranking needs a clip labelled with when
the gate is truly in view.

### B-113: a lapsed DHCP lease renumbered Kutum's Ethernet under a running stack `READ 2026-11-12` **FIXED 2026-11-12** (in the repo; on Kutum at the next `setup.sh --system`)

**What.** With the stack and Batighor running, the bench router let eth0's
lease lapse; networkd dropped 192.168.0.102 and took 192.168.0.105 33 s later.
Every session to the old address went with it: the desktop app sat on
"Waiting for image messages", and ssh to the stable-looking address said no
route to host. The ROS graph on the Pi was untouched. An operator on
`mongla.local` reconnects to the new address; a session typed by address, or
a tether whose DHCP server restarts mid-run, does not.

**Fix.** `critical: true` in `kutum/system/netplan/60-kutum-eth.yaml`, which
netplan 1.1.2 writes as `KeepConfiguration=true`: networkd never drops an
address a DHCP server gave, so a lapse adds a new address beside the old one
instead of replacing it (checked with `netplan generate` on Kutum).
`doctor.sh` reports the installed file differs until setup runs again.

**How it survived.** Each earlier dropout was explained by the Wi-Fi hopping
between access points, and eth0 was noted as the stable address. It was
stable only while the router kept renewing.

### B-112: the detector treated a HEF's compiled score threshold as a floor nothing could lower `READ 2026-11-11` **FIXED 2026-11-11**

**What.** B-58 found a model compiled at the stock 0.200 and concluded that
only a recompile could lower it. Every note since repeated that: the detector
clamped `assoc_conf` to the compiled value, warned that a lower `conf` could
not be had, and a stock graph could not run at the 0.10 to 0.15 this stack
operates at. Nobody had asked the runtime. The NMS of these graphs runs in
libhailort on the host, and `set_nms_score_threshold` on the model's output
replaces the value at configure time.

**Measured** (measured-bars 79): on a graph compiled at 0.200, 323 boxes
came back below 0.200, all 18 boxes at or above it were unchanged, and the
inference took 0.16 % longer.

**Fix.** `HailoDetector._lower_floor` configures any detection graph compiled
above `EMIT_FLOOR` (0.05) at `EMIT_FLOOR`, and `_floor` is what the clamp and
the warning now read. A refusal keeps and reports the compiled value. The
floor and the overlap threshold are also live parameters (`nms_floor`,
`nms_iou`), and a live `iou`, which nothing read after a model was built and
which was accepted in silence, is now rejected with the parameter that works.
`test_hailo_arbiter.py` holds it: the call is made, before the configure, only
when needed, and a refusal is not reported as success.

**How it survived.** The compiled value was read from `parse-hef`, a lower
host `conf` changed nothing, and both facts fitted "it is baked". The check
that would have refuted it was one call on the runtime.

### B-110: a process or camera that died mid-run stayed dead for the run `READ 2026-11-10` **FIXED 2026-11-10**

**What.** No process bringup starts had a restart. A detector that `os._exit`s
after repeated inference failures, a flow node that raised, or a manager that
crashed stayed down until someone relaunched the stack by hand, which nobody
can do with the hull in the water. A USB camera that dropped off the bus was
worse: the v4l2 mailbox kept polling a dead descriptor, `is_healthy()` went
False, and the camera node logged `BAD` every 2 s and never tried to reopen
the device. Both cameras and both detectors share one process on the vehicle
(`detector_dual_node`), so restarting that process would have blinded the
healthy camera as well.

**Fix.** The owner set the policy: restart 2 s after the death, three times,
then stay down and say so; the mission keeps running and nothing acts on the
loss alone. `mongla_runtime/launch_respawn.py` implements it once,
because Humble's launch has no restart limit. Every process in `bringup`,
`vision_pi` and `vision` uses it except the operator display, and
`test/test_every_launched_process_restarts.py` fails a bare node.
A lost camera is reopened in place by its own capture thread
(`mongla_vision/cameras/reopen.py`) under the same three-attempt policy, so
the other camera never stops. The vehicle refuted the first version of that:
with the forward lead pulled, three attempts were spent in 6 s on a device
node that did not exist, and the camera stayed lost after the lead went back.
Now an attempt counts only while the device exists, and a reopen that comes
back as the OpenCV fallback instead of the mailbox is refused. Re-run on the
vehicle: back at 30.0 Hz, 0 dropped, after ~55 s out (measured-bars §69). Restarting the manager is safe because of
B-94/B-97: a new manager that hears ARMED latches. A restarted localization
publishes in `odom` until it is re-anchored, which `pose()` refuses.
Tests run real processes under a real `LaunchService`. Injection checks:
removing the limit, restarting a clean exit, reopening a video file, skipping
the source swap, and a bare node in any of the three launch files each fail.

### B-111: nothing below the mission stopped a dive, and the aiding feed failed open `READ 2026-11-10` **FIXED 2026-11-10** (host half)

**What.** Two gaps the same shape (issue #52). A mission with a sign or units
error could command any depth up to the 10 m move envelope, and the only thing
that would refuse it was the process that made the mistake. And
`/mongla/localization/aiding`, the estimator's own "do not act on this
position" signal, reported `aided` whenever its observability gate raised or
was missing, so the one case where the estimator could not vouch for itself
read as healthy.

**Fix.** `shield_max_depth_m` on the manager (`node_support.depth_shield`):
past the floor for one second while armed, it aborts, surfaces and holds the
same latch a dead mission does until the board confirms. It is off until the
deck sets it, the manager says so at start-up, and `bringup_check` warns. The
aiding feed fails closed. The board half, where a floor belongs, is
[upstream ask Y](upstream/pr-y-a-floor-below-the-mission.md); the host does
not send the `AIDED` flag until the board reads it.

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
stamp; the wall clock steps when network time first syncs (the Pi 5 RTC
sets it at boot, but not to the millisecond). Tests in
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

### B-82: the shipped `gate_sharks.hef` lost a median 0.34 of confidence to a fine-tune that diverged `MEASURED 2026-11-06 (emulator)`. **REBUILT 2026-11-06**; **ON-CHIP CHECK DONE 2026-11-19**

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
run `hailortcli parse-hef`, and re-run `tools/studies/composed_lock_harness.py` on the
same gate clip. The detection-rung share must rise well above 20 %.

**Closed on Kutum, 2026-11-19.** The md5 on the vehicle matches the provenance
(`3e9e15f0...`), `hailortcli parse-hef` reads the 3-class YOLOv8 NMS graph, and
`composed_lock_harness.py` on archive `gate.mkv` (60 s, conf 0.45, forward
detector and ladder composed on one VDevice) held the **detection rung 100 %**
of the time, against 20 % with the diverged build; 29.8 Hz published, 0
XFeat calls raised. The run also logged the lock's "camera looks degraded, 8/8
references collapsed together" warning; its cause was not investigated.

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

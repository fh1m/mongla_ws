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
outside the collected tree. The control bench is the one named exemption: it
compiles the firmware and needs that checkout in CI (#132).

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
loss alone. `mongla_localization/launch_respawn.py` implements it once,
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
run `hailortcli parse-hef`, and re-run `tools/studies/composed_lock_harness.py` on the
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

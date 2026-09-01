# Scoring — what a practice run was worth

The sim could show you a run. It could not tell you what the run **scored**, so
the parts of the rulebook that are pure judging — and that a mission has to be
written around — were never exercised.

`duburi_sim_bridge/scoring.py` watches ground truth and keeps the score against
the competition the running course belongs to. It is read-only, so it can never
affect the run it is watching.

```bash
ros2 topic echo /duburi/sim/score          # live JSON, 2 Hz
ros2 param set /scoring run start          # also automatic on arm
ros2 param set /scoring run stop           # scores the time bonus, writes the card
ros2 param set /scoring coin flip          # take the coin flip
ros2 param set /scoring flare_sequence "[yellow,red,blue]"   # SAUVC task 4
```

The **`score` page in the lab UI** (`duburi_sim lab`) is the readable version:
every rulebook line item, what it is worth, whether it was earned, and the
evidence for each.

## The tables are data — `rulebook.py`

Both scoring tables live in `duburi_sim_bridge/rulebook.py`, one entry per
published line item with its citation. The scorer says HOW a thing is detected;
that file says WHAT it is worth, so a rules update is a one-file edit.

| | rulebook maximum | reachable in sim |
|---|---|---|
| SAUVC 2026 | 310 | 230 (74 %) |
| RoboSub 2026 | 21800 | 14700 (67 %) |

**Both numbers are always shown.** A total that quietly counts points the sim
cannot award reads like a competition result and is not one.

Sources: <https://sauvc.org/rulebook/> and the RoboSub handbook's
[4.2 Autonomy Challenge Scoring](https://robonation.gitbook.io/robosub-resources/section-4-scoring-and-awards/4.2-autonomy-challenge-scoring).
SAUVC publishes its numbers; RoboSub's are on that page and nowhere in the
handbook PDF, which is why an earlier search concluded they were unpublished.

### What is NOT modelled, and why

Marked in the table and struck through in the UI rather than scored as zero —
"we did not do this" and "this cannot be done here" are different facts.

| Item | Worth | Why not |
|---|---|---|
| Octagon object handling (6 items) | 4900 | **No manipulator on the sim vehicle.** The octagon, table and four collectibles all exist as props; nothing can pick one up. This is the highest-scoring RoboSub task. |
| Inter-vehicle communication | 1000 | Dubomini and IVC are committed phase-2 work with no code today. |
| Magnetically-activated bin lights | 1000 | Not in the handbook text, so there is no rule to implement against. Inventing one gives a confidently wrong number. |
| SAUVC target reacquisition | 60 | No manipulator. |
| Size / weight bonuses, spec violations | — | Properties of the hull, not of the run. |

Where a rule exists but the published text gives no number — how "correct
sequence" is judged, the coin-flip mechanics — the value is a **parameter with
our default, labelled as ours**. Same discipline as the style-point split.

## Geometry comes from the course

The launch used to pass only the world name, so **every geometry parameter ran
at its `robosub26_full` default whatever course was loaded** — the scorer
watched for a gate transit at x = −5 while the gate was elsewhere, and reported
"not transited" for a run that went through it. Nothing logged, because nothing
was wrong: it looked exactly where it was told to.

`_adopt_course_geometry` now reads the course yaml and the arena spec back and
takes the gate, board, bin and pool from them. An explicitly-set parameter
still wins.

```
[SCORE] geometry from sauvc26_final: gate x=3.5 board x=? pool 25.0x16.0x1.6 m
[SCORE] scoring sauvc26_final against SAUVC 2026
```

**There is no runtime competition identity in a `.world`.** `competition:` is a
course-yaml key the generator consumes and never writes out, so the scorer
reads the course file back and applies the generator's own fallback chain
(`competition` → `pool` → `sauvc`). The frontend used to prefix-match the
course name; that workaround is gone.

## Penalties — edge-triggered, from ground truth

SAUVC deducts 2 for touching the gate and 5 per touch of the pool bottom or a
wall, and aborts a run at 10 s cumulative contact or 5 discrete touches. None
of it was detected before — the one part of its table a practice run can lose
points on.

Detected by proximity to the pool shell rather than a contact sensor: there is
no contact sensor in the world template, and one would report every physics
step. **A penalty lands once when the hull arrives at the wall, not sixty times
a second while it sits there** — that distinction is the difference between a
−5 and a −300. Measured: two touches over 8.0 s of contact gave exactly two
penalties.

The SAUVC floor slopes (1.6 m centre, 1.2 m ends), so the floor test follows
`floor_edge_depth` rather than a flat number.

## The run clock

Both time bonuses need one, and a run does not begin when the node does — so it
**starts when the vehicle arms** and stops when it disarms. Waiting for an
operator to remember a parameter would mean every scorecard's time bonus was
quietly wrong.

- SAUVC: `(900 − run) × 0.03`, needs two tasks.
- RoboSub: whole minutes remaining plus fractional seconds, `× 100`.

On stop the card is written to **`DUBURI_RUN_DIR`** (default `~/duburi_runs`),
the same tree the autonomy mission scorecards use, so a practice run's two
halves land under one timestamp. Everything used to be in memory, and a restart
— which is also how you change course — erased the run you had just done.

## Style points

Handbook p. 32, verbatim:

> "Teams can gain extra points by passing through the gate with 'style'. For
> every 90° change in orientation, the AUV increases the accumulated points.
> However, returning to the last previous orientation won't count. I.e. an AUV
> that rolls 90° and then back to 0° would not get points. Roll and Pitch are
> worth more than Yaw."

That second sentence is the whole difficulty: a roll to 90° and back is *two*
90° changes and scores *one*, so the scorer remembers the previous quadrant per
axis, not just the current one.

| manoeuvre | score |
|---|---|
| barrel roll 0 → 90 → 180 → 270 → 0 | **8** (4 × 2) |
| oscillating 90 ↔ 0, four times | **+2**, with 3 returns rejected |

A vehicle must reach within 25° of the new cardinal before the change counts —
without it, rolling to 60° and stopping scores a full 90°, since the quadrant
boundary sits at 45°.

RoboSub pays +100 per 90° of yaw and +200 per 90° of roll/pitch, capped at 8
each; the *relative* weights inside our own style total are still ours, because
the handbook says roll and pitch are worth more without publishing the split.

## Gate side, coin flip, flares, shots

Unchanged in substance and now feeding the card:

- **Gate side** — the side taken is the role for the run. First crossing scores
  the gate, the second scores Return Home.
- **Coin flip** — heads places the AUV parallel to the gate, tails with its
  tail facing it, with a few degrees of deliberate slop because
  "approximately" is the rulebook's own word. A mission that hard-codes its
  start heading fails from here.
- **Flares** — a flare counts when its golf ball leaves the cup, the same
  physical event the judges watch. Order is the entire task.
- **Shots** — graded `through` / `past_board` / `miss`, with the opening and
  the range **at the moment of firing**, banded against the two standoffs. The
  distance bonus is additive on a scoring shot: a miss earns nothing however
  far away it was fired from, because the band rewards a hard shot, not a
  distant one.

### Two bugs worth remembering

**Polling gave a false pass.** The first flare scorer shelled out to
`gz model -m <ball> -p` once per flare inside a 0.5 s timer. Three blocking
subprocesses together outlast the timer, so detections came out in *polling*
order rather than the order the balls fell: red, then yellow, then blue was
scored as yellow, red, blue and reported a **clean pass** — on the one thing
the task grades.

**The obvious fix does not work.** A gz-transport subscription to
`dynamic_pose/info` fails because `Pose_V`'s repeated `Pose` field will not
resolve in the Python bindings' descriptor pool, thrown inside gz's own
callback thread where it prints a traceback and is otherwise swallowed — a
subscription that looks alive and delivers nothing. It reads one long-lived
`gz topic -e --json-output` instead.

---

## Round 13 — the two task graders could not award their tasks

Asked whether torpedo-through-hole and bin-drop scoring actually worked, both
were **tested rather than trusted**, and both were broken. Neither logged
anything, because nothing *was* wrong: the scorer was looking exactly where it
had been told to look.

### Torpedo: a perfect shot graded a miss

Fired dead-centre on a real opening from exactly 1.00 m, physically through the
board:

```
[SCORE] TORPEDO PAST_BOARD -- fired from 1.00 m (far)
```

`board_openings` was a hand-typed **two**-opening default, left behind by the
2026 four-opening rewrite, and **nothing derived it from the geometry**:

| | scorer (before) | `prop_library.torpedo_openings()` |
|---|---|---|
| count | 2 | **4** |
| radii | 0.10 / 0.065 | 0.07 / 0.0475 |
| frame | absolute world z | plate-relative (±0.132) |

`'large' if idx == 0 else 'small'` also cannot label a board with **two** large
openings and **two** small; sizes now come from `torpedo_layout()`'s own `kind`.

### Bins: no crate was inside the scored footprint

```
[SCORE] DROPPER OUTSIDE_BIN -- fired from 0.65 m (near)
```

`bin_x`/`bin_y` are the bins **model origin**, but four crates hang **±0.52 m**
off the pipeline, and the box was 0.61 × 0.305 — an older rule; the 2026
CleverMade crate is **0.335 square**. Half-extent in y was 0.1525, so every
crate sat 3.4× outside it, and the only place that scored was the origin itself:
open water on the pipework **between** the crates.

### Both now come from the module that built the props

`prop_library` is installed into the worlds package share, so the scorer imports
it rather than keeping a second copy of the numbers — a second copy is precisely
how this happened. `_adopt_course_geometry` derives `board_openings`,
`board_opening_kinds`, `bin_targets` and `bin_size` at startup, and **says so
loudly** if it cannot; the fallback defaults no longer describe a board.

`_check_bin_lights` had the same disease and is fixed with it: it used a
**crate** dimension (`bin_size[0]`) to place detectors along the **pipeline**,
so detectors sat ±0.15 m from the centre instead of ±0.325 m of the 1.30 m
pipe run. It reads `pipeline_span` now. This changes RoboSub bin-light totals.

### Measured after

```
[SCORE] TORPEDO THROUGH -- fired from 1.02 m (far), opening large
[SCORE] DROPPER IN_BIN  -- fired from 0.65 m (near)      # crate sr_a
[SCORE] DROPPER IN_BIN  -- fired from 0.54 m (near)      # crate rescue_a
[SCORE] DROPPER OUTSIDE_BIN -- fired from 0.00 m (near)  # pipework: the
                                                         # negative control,
                                                         # and the only place
                                                         # that used to score
```

### Two frame traps, and two rclpy traps

- **Plate-relative → world z.** `torpedo_openings()` is ±0.132 about the plate
  centre; the grader compares absolute z. The stale default's −1.10 / −1.40 were
  close enough to the real rows to look plausible.
- **Board yaw.** At yaw = π (how every course places it) plate +y maps to world
  −y. A y-sign error is **invisible on a centred shot**, so `test_the_board_yaw_
  flips_the_side_an_opening_is_on` checks it directly rather than leaving it to
  an end-to-end run that cannot see it.
- **An empty-list parameter default infers BYTE_ARRAY in rclpy.** Setting a
  double array over it then does nothing, silently — `ros2 param get` answers
  `Byte values are: []`. The derived openings were discarded that way on the
  first run of this fix. Defaults are `[0.0]`.
- **`set_pose` with no `orientation` leaves the hull at whatever heading it
  drifted to.** Three of four end-to-end shots flew parallel to the board at
  yaw 89.9° and read exactly like a scorer fault. (Pinning the hull at 20 Hz
  through the shot is *also* wrong — it punts the round as it spawns, the
  muzzle-clearance trap in PAYLOAD.md.)

`test_score_geometry.py` covers all four openings, a 1 mm miss at each rim, all
four crates, the pipework between them, both frame conversions, and the guard
for the whole class — **the scorer's opening count must equal
`len(torpedo_openings(spec))`**, so a spec change fails loudly instead of being
ignored. Four defects were reintroduced to confirm the tests bite.

---

## Round 14 — the rig, and the two more scorer bugs it found

Round 13 closed with one of four openings confirmed live. Building a real rig to
close that gap found **two further scorer defects**, both of which had been
silently mis-grading shots that physically went through.

### `score_check` — a rig that reports its OWN failures as its own

`ros2 run duburi_sim_bridge score_check [--task torpedo|bins]`. It drives the
`duburi` CLI by subprocess (never importing `duburi_ws` — `gate_transit_check`
sets that rule), **verifies its own preconditions from ground truth before
firing**, and reports `RIG FAILED` distinctly from a scorer verdict. That
distinction is the whole point: in round 13 a hull sitting at yaw 89.9 deg was
indistinguishable from a broken grader.

### Bug 1 — the first observation of every shot was thrown away

`_score_shots` did `if last is None: continue`, so plane-crossing detection only
began at the **second** sample. The timer runs at 0.5 s and a torpedo covers the
0.6 m to the board in ~0.35 s, so the round was usually already past the plane
before there was anything to compare against — both samples on the far side, no
sign change, `miss`. Traced off the pose stream, two rounds travelled **+3.458 m
and +3.428 m** in x, from −0.600 clean through the board at x = 0, while the
scorer called both misses.

Fixed by seeding the track with `fired_from`, the vehicle's own position at the
trigger, which is always on the near side and was **already in the record**.

### Bug 2 — the crossing was recorded where the round was NEXT SEEN

With detection fixed, shots became `past_board` instead: the crossing point was
taken as the first sample *after* the plane, which at 1.7 m/s is most of a metre
downrange, by which time the round has fallen out of the opening it passed
through. Measured **0.29–0.36 m** of apparent miss on shots that went cleanly
through. Fixed by interpolating between the straddling samples to x = board_x.

### Where it stands, honestly

| task | result |
|---|---|
| **bins** | **5/5** — all four crates `in_bin`, plus the pipework negative control `outside_bin`. Automated and repeatable. |
| **torpedo** | **1/4** on a clean world. Both scorer bugs above are fixed and independently justified, but the rig still cannot place the hull precisely enough for four consecutive shots. |

The remaining shortfall is **the rig, not the scorer**. `fired_from` shows the
hull climbing between placement and trigger (−1.056 → −1.005, i.e. 51 mm high
against a small opening of radius 47.5 mm) and drifting further on each
successive shot: ALT_HOLD is correctly returning the vehicle to its last
*commanded* depth after each teleport.

**Commanding `set_depth` at the placement depth was tried and REVERTED** — it
re-engages ALT_HOLD, which moves the hull while the shot is being set up, and
the run went 1/4 → 0/4. Recorded because the reasoning was sound and the
measurement disagreed.

The real fix is to stop teleporting and **fly the hull to each firing point with
the stack's own verbs**, which is also the only version that exercises what a
mission actually does. That is next round's work. `score_check` reports the
shortfall rather than hiding it, and exits non-zero.

### Also, the muzzle lead

A round does not leave from the hull's centre: `payload_sim` spawns it
`muzzle_down_m` (0.05 m) below, so aiming the hull at an opening fires **low by
more than a small opening's radius** (0.0475 m). The rig reads that parameter
from the node rather than retyping it, and adds the gravity drop. A real mission
carries the same lead in its `align(depth=)`.

### Round 15 — flying the rig, and why a torpedo run needs vision

`score_check` gained `--fly`, which reaches each firing point with the stack's
own verbs (`set_depth`, `lock_heading`, `move_forward_dist`,
`move_lateral_dist`) instead of placing the hull. It is **opt-in**, and the
default is still a direct placement, because *verifying the grader* and
*verifying the mission* are two different questions and round 13 showed what
conflating them costs.

**The flying mode produced a real mission finding.** The distance verbs close on
the DVL to `dvl_tolerance`, default **0.1 m**. A small opening's radius is
**0.0475 m** and a large one's **0.070 m**, so a verb that has done its job
perfectly still leaves the muzzle outside the hole — measured, flown shots land
~0.14 m off the aim point and a 1.0 m standoff command came out at 0.86 m.

That is not a defect in the verbs or the rig. **It is the reason `vision_align`
exists**: dead reckoning gets the vehicle to the board, and vision has to put
the round through the hole. Worth stating because a torpedo mission written as
"fly to the standoff and fire" cannot work, and nothing previously said so.

### Still open: only the first shot of a run scores

Torpedo stands at **1/4**, bins at **5/5**. Shots 2–4 are graded `miss` about
2 s after the trigger — the signature of a round that never left the tube.
**Two hypotheses were tested and both were wrong:**

- *"consecutive shots lose their launch impulse"* — **disproved.** Four
  consecutive shots from a standalone harness travelled 4.113 / 3.816 / 3.722 /
  3.863 m. The launch path is sound.
- *"the shots are too close together"* — **disproved.** A 20 s cooldown changed
  nothing, and was removed rather than left in to slow the rig for no benefit.

What still differs between the harness that works and the rig that does not is
that the harness holds a live gz-transport pose subscription for its whole run,
while the rig shells out to `ros2 topic echo` per verdict. That is the next
thing to test. It is written down rather than guessed at, because two confident
guesses have already been wrong here.

**This is a rig limitation, not a grader one.** Every opening is covered by unit
tests against the same geometry the mesh is cut from, and opening 0 passes
end-to-end on every run. `score_check` exits non-zero rather than hiding it.

### A crash nobody had hit

`ros2 run duburi_planner duburi --help` died with `TypeError: %c requires int or
char`. argparse interpolates help strings (`self._get_help_string(action) %
params`), and one help string in `commands.py` had a bare `% cap` where the two
others in the same file were correctly escaped `%%`. Nothing caught it because
no test runs `--help`.

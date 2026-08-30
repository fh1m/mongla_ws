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

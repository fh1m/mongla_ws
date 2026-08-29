# Scored elements — what a practice run was worth

The sim could show you a run. It could not tell you what the run **scored**,
so the parts of the rulebook that are pure judging — and that a mission has to
be written around — were never exercised.

`duburi_sim_bridge/scoring.py` watches ground truth and keeps the score. It is
read-only, so it can never affect the run it is watching.

```bash
ros2 topic echo /duburi/sim/score          # live JSON
ros2 param set /scoring coin flip          # take the coin flip
ros2 param set /scoring flare_sequence "[yellow,red,blue]"   # SAUVC task 4
```

## Style points

Handbook p. 32, verbatim:

> "Teams can gain extra points by passing through the gate with 'style'. For
> every 90° change in orientation, the AUV increases the accumulated points.
> However, returning to the last previous orientation won't count. I.e. an AUV
> that rolls 90° and then back to 0° would not get points. Roll and Pitch are
> worth more than Yaw."

That second sentence is the whole difficulty. A roll to 90° and back to 0° is
*two* 90° changes and scores *one*, so the scorer has to remember the previous
quadrant per axis, not just the current one.

Measured against a real vehicle:

| manoeuvre | score |
|---|---|
| barrel roll 0 → 90 → 180 → 270 → 0 | **8** (4 × 2) |
| oscillating 90 ↔ 0, four times | **+2**, with 3 returns rejected |

A vehicle must also reach within 25° of the new cardinal before the change
counts. Without that, rolling to 60° and stopping scores a full "90° change",
because the quadrant boundary sits at 45°.

Roll and pitch are worth 2, yaw 1. The handbook says roll and pitch are worth
more but does not publish the split, so those are **our** numbers and they are
parameters rather than constants — a scorecard that looks official while
inventing its own weights is worse than one that says which weights it chose.

## The coin flip

Handbook p. 32: heads places the AUV "approximately parallel to the gate";
tails places it "with its tail approximately facing the gate (the AUV is
backward)". It is worth extra points, and a mission that has quietly
hard-coded its start heading fails the moment it is taken up.

```bash
ros2 param set /scoring coin flip     # or force 'heads' / 'tails'
#   [SCORE] COIN FLIP: HEADS -- vehicle placed at yaw 86.3 deg.
#           A mission that hard-codes its start heading will fail from here.
```

A few degrees of slop are added deliberately, because "approximately" is the
rulebook's own word.

## Gate side

"The AUV chooses a marine animal by passing under a specific side", so the side
taken **is** the role for the rest of the run. Each transit is recorded with
the side and the depth:

```
[SCORE] GATE TRANSIT on the port side at -1.20 m (-1.00 m off centre)
        -- that is the role for this run
```

## SAUVC flare sequence

Task 4 hands the team a colour order topside; the vehicle must bump the flares
in it. Order is the entire task — three hits in the wrong sequence is not a
partial pass. A flare counts as bumped when its golf ball leaves the cup, the
same physical event the judges watch.

```
[SCORE] FLARE RED bumped (1 of 3) -- OUT OF ORDER, expected yellow
```

The sequence is settable at **runtime**, because that is when the real one
arrives — reading it once at construction made `ros2 param set` report success
and do nothing.

### Two bugs worth remembering

**Polling gave a false pass.** The first version shelled out to
`gz model -m <ball> -p` once per flare inside a 0.5 s timer. Three blocking
subprocesses together outlast the timer, so detections came out in *polling*
order rather than the order the balls fell: knocking red, then yellow, then
blue was scored as yellow, red, blue and reported as a **clean pass** — on the
one thing the task grades.

**The obvious fix does not work.** A gz-transport subscription to
`dynamic_pose/info` fails because `Pose_V`'s repeated `Pose` field will not
resolve in the Python bindings' descriptor pool (`No message class registered
for 'gz.msgs.Pose'`), thrown inside gz's own callback thread where it prints a
traceback and is otherwise swallowed — a subscription that looks alive and
delivers nothing. Importing `pose_pb2` first, at module scope, does not help.
It reads one long-lived `gz topic -e --json-output` instead: one consistent
snapshot per message, and no descriptor pool at all.

## Not modelled

**Magnetically-activated bin lights.** They are not in the 2025 handbook, so
there is no rule text to implement against, and inventing one would produce a
confidently wrong number.

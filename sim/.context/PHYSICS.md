# Physics: what is modelled, what is measured, and what is not

The governing constraint is that **practice in the sim should transfer to the
pool, and the sim should present the same difficulties at the same time**. A sim
that is easier than water trains the wrong reflexes.

Every number on this page came from a live run or from Blue Robotics' published
data. Where something is an estimate, it says so.

---

## The T200 thrust curve

**The problem.** ArduPilotPlugin can only apply an affine map,
`(normalised_pwm + offset) * multiplier`. So the simulated thruster was
perfectly linear in PWM, symmetric forward and reverse, with no deadband and no
spin-up. Against Blue Robotics' own performance data at 16 V a straight line is
wrong by **12.24 N — 24 % of full thrust** — and the error is largest at low
command, exactly where alignment and station-keeping live.

**The fix.** `duburi_sim_bridge/t200_curve.py` sits in the command chain:

```
ArduSub --PWM--> ArduPilotPlugin --cmd_thrust_linear--> t200_curve
        --cmd_thrust--> Thruster plugin --> joint force
```

It recovers the PWM ArduSub asked for and republishes the thrust a real T200
makes at that PWM. Fitted from
`T200-Public-Performance-Data-10-20V-September-2019.xlsx`:

| Volts | Deadband (µs) | Max fwd | Max rev | rev/fwd |
|---|---|---|---|---|
| 12 | 1464–1536 | 36.4 N | −28.6 N | 78.5 % |
| 14 | 1468–1532 | 44.4 N | −34.5 N | 77.7 % |
| 16 | 1472–1528 | 51.4 N | −39.9 N | 77.6 % |
| 18 | 1472–1528 | 59.0 N | −45.0 N | 76.3 % |
| 20 | 1476–1528 | 65.9 N | −49.4 N | 74.9 % |

Fit quality against the source data: **mean error 0.20 N, max 1.77 N at 16 V**,
where the linear model's max error was 12.24 N. Voltage rows are interpolated,
so a sagging pack loses thrust smoothly rather than in 2 V steps.

### Where it actually changes behaviour

This is the part worth understanding, because the naive expectation is wrong.
Measured A/B, same course, **fresh world for each arm**:

| Command | Linear | T200 curve |
|---|---|---|
| `--gain 30` | 0.340 m/s plateau | **0.342 m/s** — identical |
| `--gain 15` | travelled 0.37 m | **travelled 0.00 m** |

At usable command the two agree, because **ArduSub closes a loop**: it sees the
vehicle is slow and asks for more PWM until it gets the speed it wants. Reduced
thrust per PWM is compensated away.

At small command it *cannot* compensate — below the deadband there is no PWM
that produces any thrust at all, and the vehicle simply does not move.

So: **transit verbs are unaffected; fine alignment and station-keeping are.**
Those are precisely the behaviours that used to look better in sim than in
water. If a hold that worked in sim now hunts, that is the simulator starting to
tell the truth.

> A trap worth recording: an early measurement suggested the curve made the
> vehicle travel *further*, which is impossible. The runs were sharing one sim
> and the vehicle started each from wherever the last one left it, sometimes
> against a wall. Distance depends on start pose; **speed plateau does not**.
> Restart the world per arm and compare speeds.

### Spin-up

A first-order lag, default τ = 0.15 s, applied on **measured elapsed time**. An
earlier version assumed a fixed 5 ms command interval; the commands actually
arrive at ~3400 Hz, which made the lag about 7× too slow and would have changed
again on different hardware. `thruster_tau:=0` disables it.

### Not modelled

Bollard-pull data is thrust with the vehicle *held still*. It does not capture
thrust falling as inflow rises — the Thruster plugin's own `wake_fraction` /
`alpha_1` / `alpha_2` cover that and are not yet set. No battery sag, no
prop-wash interaction between thrusters.

---

## Water current

A real pool is not still: filtration circulates it, other vehicles stir it. The
AUV is pushed off station continuously, and `vision.ki_lat` — a lateral integral
term — exists specifically to null that. **With zero current it had nothing to
fight**, so every hold was being graded against an easier problem than the pool
poses.

`duburi_sim_bridge/water_current.py` publishes a steady set plus slow gusting.
Gazebo's Hydrodynamics plugin folds it into the relative-velocity term of the
Fossen model, so the current is applied by the *same* hydrodynamics that produce
drag rather than as a force bolted on beside them.

```bash
ros2 launch duburi_sim_bringup sim.launch.py current_speed:=0.08 current_heading:=45
ros2 param set /water_current speed 0.12      # live, no restart
```

Measured: **0.12 m/s current drifts the vehicle 1.374 m in 40 s** while holding
depth, along the current heading.

> **The topic is bare `/ocean_current`, not `/model/<name>/ocean_current`.**
> The plugin's `<namespace>` element makes the namespaced form look right and
> the docs imply it. It is wrong. `gz topic -i` shows the namespaced topic with
> a publisher and no subscriber, and the bare one with a subscriber and no
> publisher — which is why the first attempt produced 6 mm of drift in 40 s
> instead of 1.374 m. Everything started, the topic existed, the numbers looked
> right, and the two halves were not connected. **`gz topic -i` is what
> distinguishes "publishing" from "being heard".**

Gusting is deliberately slow (0.08 Hz, ~12 s period). A real pool's disturbance
is slow drift, and a controller can meaningfully fight it; high-frequency noise
would add jitter no controller can track and would teach nothing.

Default speed is **0** — a course that silently gained a current would make
every tuned duration wrong with no visible cause.

---

## Real-time factor

The sim ran at RTF 0.37–0.65 — about half real time, so a 45 s recording took
90 s. Bisected live:

| Configuration | RTF |
|---|---|
| `pool_empty`, no ArduSub | 0.59–0.71 |
| `pool_empty`, **with** ArduSub | **1.000** |
| `sauvc26_final`, with ArduSub | 0.37–0.65 |
| `sauvc26_final` with **props stripped**, with ArduSub | **1.000** |

**ArduSub's lock-step is not the problem** — it paces the sim correctly. The
cost is props, and specifically their **visuals, not their collisions**:

- Cutting collision shapes 101 → 37 changed the RTF **not at all**.
- Halving the drums' draw calls took it to **0.71–0.91**.

The four drums were **169 of the course's 191 visuals**, because each carried a
20-segment wall *and* a 20-segment interior liner. The liner is now one
cylinder; it only ever existed to make the interior read as the drum's colour,
which one cylinder does as well as twenty boxes.

Every visual is paid **four times** — two cameras plus two bounding-box cameras
all render the scene. A drift test caps each course's prop-visual count.

Collision shapes were still worth cutting (101 → 37) for solver headroom, and
the drum wall keeps a coarse 4-box collision ring so a dropped ball still cannot
escape sideways.

---

## Validation status

`step_response.py`, `thruster_rig.py` and `thruster_survey.py` exist and are
well built. What is actually measured:

| Quantity | Status |
|---|---|
| Surge terminal velocity | **0.650 m/s measured vs 0.661 predicted, 1.6 %** |
| Sway, heave terminal velocity | not measured |
| Rotational axes | not measured |
| Settling time (the only observable of added mass) | not measured |

`HYDRODYNAMICS.md` states the rotational added-mass terms carry **30–100 %
estimation error**, because the vehicle violates the estimating method's
assumption that length exceeds width — the Heavy is 20 % wider than long.

## The fidelity limit, stated plainly

**The sim vehicle is a BlueROV2 Heavy, not the Duburi 4.5 hull** (`model.sdf.in`
line 12). Every hydrodynamic coefficient describes a different body; only the
`vectored_6dof` thruster frame is shared. No coefficient tuning changes that.

What this work can do — and does — is make the *character* of the difficulty
right: current pushes you off station, small commands do nothing, reverse is
weaker than forward, rotation is lightly damped. The behaviours that survive
those survive water.


---

## Solver tuning: measured, and the answer was "leave it alone"

The `<physics>` block now exposes a `<dart>` sub-block — collision detector and
solver type — where before there was nothing below step size and RTF.

`bullet` + `dantzig` looked like the obvious choice: this world is primitive
boxes and cylinders with few simultaneous contacts, which is the direct LCP
solver's best case. Measured over 443 samples on `sauvc26_final`:

| Configuration | median RTF | mean |
|---|---|---|
| DART defaults | **0.393** | 0.374 |
| `bullet` + `dantzig` | 0.373 | 0.349 |

A 5 % regression, inside the run-to-run noise. Consistent with everything else
here: **the sim is render-bound, not solver-bound**, so tuning the solver tunes
the part that is not the bottleneck. Reverted to defaults, with the knobs left
exposed so the next person re-measures rather than re-guesses.

> RTF is noisy enough that a single reading proves nothing — the same
> configuration gave 0.26 and 0.59 minutes apart. Take a distribution.

## Validation: all three translational axes, for the first time

Full results in
[`duburi_sim_description/models/duburi_heavy/RESULTS.md`](../src/duburi_sim_description/models/duburi_heavy/RESULTS.md).

| Axis | Predicted | Measured | Error |
|---|---|---|---|
| surge | 0.661 m/s | 0.658 m/s | 0.5 % |
| sway | 0.571 m/s | 0.563 m/s | 1.3 % |
| heave | 0.423 m/s | 0.395 m/s | 6.6 % |

Before this, **surge alone** was the entire validation of the hydrodynamic
model. `drag_survey.py` measures each axis and appends to a committed results
file, so a coefficient change is diffed against measurements instead of
re-derived.

Two traps found while building it, both of which produce a plausible wrong
number rather than an error:

- **`t200:=false` is required.** The T200 node publishes to the same
  `cmd_thrust` topic the test rig commands. With both running the rig fights a
  second publisher and reads ~half the thrust it asked for — surge came back at
  0.330 against 0.661, a 50 % error that looks exactly like bad drag
  coefficients.
- **The pool has 12.5 m of runway.** A 25 s run at 0.66 m/s needs 16.5 m. The
  vehicle hits the wall, stops dead, and the tail average returns half of a
  terminal velocity. `step_response.py`'s own 25 s default does this, which is
  why its recorded 1.6 % result could not be reproduced until the settle time
  was cut. `drag_survey.py` now refuses such a run and reports distance
  travelled.

---

## What else could be modelled, in value order

Everything below is available in the installed stack — no new dependencies.

### 1. Sensor noise, from ArduSub SITL (free, one config file)

ArduSub already simulates its own sensors and we set almost none of the noise
parameters. `SIM_BARO_RND` is set; `SIM_ACC_RND`, `SIM_GYR_RND`, `SIM_MAG_RND`
and `SIM_DRIFT_SPEED` are not. That means **EKF3 in sim is fed cleaner data than
it will ever see on the vehicle**, so heading drift — the single most persistent
real-world problem this project has, and the reason `yaw_source` exists at all —
does not happen in sim. This is the highest realism-per-effort item left.

### 2. Battery sag (`SIM_BATT_VOLTAGE` + the T200 curve)

The T200 curve is already voltage-interpolated: 36 N at 12 V against 66 N at
20 V. Feeding it a sagging pack voltage would make thrust fall through a run
exactly as it does in the pool, where a late-mission manoeuvre is measurably
weaker than the same manoeuvre at the start.

### 3. Thruster wake (`wake_fraction`, `alpha_1`, `alpha_2`)

The Thruster plugin supports velocity-dependent thrust and we set none of it.
The T200 data is bollard-pull — thrust with the vehicle held still — so thrust
currently does not fall as inflow rises. This is why a real vehicle accelerates
more slowly toward its top speed than the sim does.

### 4. Marine snow (`particle-emitter`, plugin installed)

The visual cue `underwater_fx` cannot fake, because it degrades uniformly.
Suspended particulate gives a detector something range-dependent to cope with.

### 5. Per-pixel attenuation (`rgbd_camera`, sensor installed)

Attenuation is currently uniform by vehicle depth, so a far wall is no hazier
than the near floor. One `rgbd_camera` emits colour *and* depth, so no second
parallel camera is needed. Noted here since 2026-08-28 and still the largest
known gap in the image pipeline.

### 6. Rotational drag validation

The rotational added-mass terms carry a stated **30–100 % error** and are the
numbers most likely to be wrong. Hard to measure: a free vehicle fired on one
thruster rotates far more than it translates, and body-axis readings decouple
within a second. Would need a short-pulse method like `thruster_survey.py` uses.

### Deliberately not pursued

- **Solver tuning** — measured above, no gain.
- **Wave forces on the hull.** The Gerstner surface is visual-only. Adding
  surface forces would matter for a surface vessel; an AUV at depth feels
  almost none of it, and the octagon task is the only time the vehicle surfaces.
- **CFD-quality flow.** The current model reproduces the disturbance, not its
  cause. Anything more is a research project, not a competition simulator.


---

## Realism additions (2026-08-29)

### Sensor noise — the highest-value item, and it was one config file

`SIM_ACC_RND`, `SIM_GYR_RND`, `SIM_MAG_RND` and `SIM_DRIFT_SPEED` were all
unset: only the barometer had any noise. **EKF3 in sim was being fed cleaner
data than it will ever see on the vehicle**, so heading drift — the most
persistent real-world problem this project has, and the entire reason
`yaw_source` exists — simply did not happen in simulation. A heading loop tuned
against a noiseless compass is tuned against a problem the pool does not pose.

Values are ArduPilot's own SITL defaults for a consumer-grade IMU, which is what
a Pixhawk 2.4.8 carries. `SIM_DRIFT_SPEED` is the one that makes a heading
estimate walk away over a mission rather than staying put.

### Battery sag

`SIM_BATT_VOLTAGE 16.0` (4S nominal) with `BATT_MONITOR 4`. The T200 curve is
already voltage-interpolated — 36 N at 12 V against 66 N at 20 V — so a sagging
pack makes a late-mission manoeuvre measurably weaker than the same manoeuvre at
the start. `FS_BATT_ENABLE` stays 0: a sim run must not be aborted by a failsafe
mid-experiment.

### Thruster wake

`wake_fraction 0.2`, `alpha_1 1.0`, `alpha_2 -0.3` on all eight thrusters. Blue
Robotics' data is **bollard pull** — thrust with the vehicle held still — so
without this the propeller made the same force at 0.7 m/s as at rest. It does
not: inflow rises, angle of attack drops, thrust falls. This is why a real
vehicle creeps toward its top speed while the sim snapped to it.

### Marine snow

A `particle_emitter` sized to the pool, 60 particles/s, drifting slowly down.
`underwater_fx` degrades every pixel by the *vehicle's* depth, so it cannot
produce anything range-dependent and nothing moves between frames. Particulate
occludes, drifts, and gives a detector something to be robust to that fog
cannot. `scene.snow: 0` turns it off. Measured cost: **none** (6.42 Hz with,
6.58 Hz without — inside the noise).

### Per-pixel attenuation — built, and OFF by default

`underwater_fx` now accepts a range image and attenuates **along the path the
light actually travelled**, so a wall 20 m away fades while the floor 1 m below
stays sharp. Verified: with a synthetic range image, near/far differ by 124 grey
levels where uniform attenuation gives them identical values.

Two things had to be calibrated or fixed:

- **`ATTEN_RGB` is an open-ocean coefficient and far too strong for a pool.**
  Applied raw, red fell to 0.004 of its value at 20 m and the far wall went
  black. `RANGE_ATTEN 0.22` and a `RANGE_FLOOR` keep the far field
  visible-but-degraded, which is the point — a detector must find props at range
  through worse imagery, not be handed a frame it cannot work with.
- **`always_on 0` does NOT disable a Gazebo sensor.** The topics were still
  published and the render pass still paid. The generator now **omits the whole
  sensor block** unless `configs.yaml` sets `range_cameras: true`.

It is off by default because it is genuinely expensive — two extra render passes
took the cameras from 12 Hz to 4 Hz on the SAUVC final course. Turn it on for a
perception experiment; `underwater_fx` falls back to uniform attenuation when no
range image arrives, so the sim is correct either way, just less faithful.

> `gz topic -l` still lists `/front_range` and `/bottom_range` when the sensors
> are stripped — `image_bridge` advertises them regardless. Do not read their
> presence as evidence the sensors are running; check for data.

### The process leak, again

Six orphans were found alive from earlier runs — four `t200_curve`, two
`hydrophone`, one for over 16 hours — because the nodes were added to
`sim.launch.py` and not to `duburi_sim stop`'s kill list. Exactly the
`dvl_bridge` leak from 2026-08-28, repeated with the new nodes. A single `stop`
then reaped **62** leftover processes.

The symptom is never "a process leaked". It is "the sim got slow", and it sends
you looking for the wrong thing — this time it cost a full round of frame-rate
A/Bs against snow and range cameras that were both innocent. **Any new node in
`sim.launch.py` belongs in that list.**


## Collision detection — `bullet`, and why not DART's own

**DART's built-in collision detector does not support several primitive pairs,
and silently generates no contact for them.** With `collision_detector: dart`
the server logs, once per pair per step:

```
[DARTCollisionDetector] Attempting to check for an unsupported shape pair:
[CylinderShape] - [BoxShape]. Returning false.
```

"Returning false" means exactly what it says. The hull's collision shape is a
**box** and every pipe prop — gate legs, slalom pipes, flare poles, path
markers — is a **cylinder**, so the vehicle drove through all of them.

Measured on `robosub26_full`, identical thrust driven straight at a gate leg
versus the same push into open water:

| detector | into the leg | open water | surge through the leg |
|---|---|---|---|
| `dart`   | 1.739 m | 1.715 m | flat 0.656 m/s — no contact at all |
| `bullet` | 2.362 m | 3.586 m | 0.658 → 0.303 m/s at x = −5.26 |

Predicted geometric contact is x = −5.246 (leg at −5.0, pipe radius 0.017,
hull half-length 0.229). The `bullet` arm decelerates at −5.26. The DART arm
holds cruise speed the whole way through. The unsupported-pair error count goes
8 → 0.

**This overturns an earlier decision recorded in `gen_world.py`.** `bullet` +
`dantzig` measured ~5 % slower than `dart` over 443 RTF samples and `dart` was
selected on that basis. The comparison was real and it measured the wrong
quantity: a faster simulator that does not collide is not a cheaper trade-off,
it is the wrong answer. The sim is render-bound anyway, so the 5 % is not where
the time goes.

**If you change this setting, re-run the collision A/B, not an RTF sample.**
A one-armed version of that test is not enough either — the first run of it
"stopped short of the leg" and the control arm showed it had simply run out of
travel budget.

### Consequence: the RoboSub gate is now solid

Working cylinder contacts plus a surface-hung gate (below) means the clear
water under the frame is 1.6–2.1 m — about 0.5 m. A sim mission that transited
the gate at 0.8 m depth passed through the legs before and will now hit them.
That is correct behaviour, not a regression.

## Thruster wake — applied, and small at pool speeds

`thrust_coefficient` and `alpha_1`/`alpha_2` are **mutually exclusive** in
`gz-sim-thruster-system`. The model set both, so the velocity-dependent term
was ignored:

```
The [alpha_2] value will be ignored as a [thrust_coefficient] was also
defined through the SDF file.
```

Removing the eight `<thrust_coefficient>` tags takes that warning **8 → 0**,
which is the proof the parameter is now consumed. `alpha_1 = 0.02` preserves
the old static coefficient, so bollard pull is unchanged and only the speed
falloff is new.

**Do not read this as "wake matters here."** `Ct = alpha_1 + alpha_2·J` with
`alpha_2 = −0.012` is a ~6 % thrust reduction at the advance ratios a 0.1 m
propeller sees at pool speeds, i.e. ~3 % in terminal velocity. Measured surge
went 0.658 → 0.649 m/s (1.4 %) and sway 0.563 → 0.559 (0.7 %) — and the
open-water arm of the collision A/B spread 0.652–0.661 m/s *within a single
run*, so the sway figure is inside noise and the surge figure is barely outside
it. **The warning count is the evidence; the velocity delta is not.**

Unlike the T200 curve, these coefficients are **modelled, not measured** — Blue
Robotics publish bollard-pull data only, which by definition contains no speed
dependence. `alpha_2` comes from open-propeller theory and puts zero thrust at
J = 1.67.

---

## Props that react — dynamic, self-righting, and damped

Until now **every prop except the balls was `<static>true</static>`**. A static
body still *generates* contact, so the vehicle stopped dead at a flare — it
just has no mass in the solver and cannot be pushed. That reads as "the sim has
collisions" and is not the same thing.

The knockable set is now dynamic: the three SAUVC bump flares and the RoboSub
slalom sets. Heavy structures (gate frame, torpedo board, bins, drums) stay
static but got back the collisions they were missing.

### Net-negative and top-buoyant, not "buoyant and moored"

The obvious design — make it buoyant and tether it — does not work. **A
net-buoyant free body just accelerates upward until it hits the surface**,
because buoyancy supplies no restoring force in *translation*. What works is
the opposite pairing:

    net weight NEGATIVE          -> it presses on the floor and stays put
    centre of BUOYANCY above
    centre of MASS               -> it rights itself from any tilt

Those are compatible: dense low ballast, near-buoyant volume above it. The
flare, computed from its own geometry rather than chosen:

| | value |
|---|---|
| mass / displacement | 0.640 kg / 0.292 kg |
| net weight in water | **0.348 kg down** |
| centre of mass | z = 0.0425 m |
| centre of buoyancy | z = 0.2721 m — **0.230 m above** |
| righting couple | **0.513 · sin θ N·m**, positive at every angle |

It also rights from **flat**: the CoM sits 0.0425 m up standing and 0.060 m up
lying on its side, so gravity alone has no barrier, and the buoyancy couple is
pure gain on top.

### A dynamic model must be WELDED

`prop_library.py` had **zero `<joint>` elements**, because `<static>true</static>`
welds every link to the world implicitly. Drop that and an 11-link flare is
**eleven free bodies** that fly apart on the first step, silently. Every
dynamic prop now carries fixed joints (`weld_all`).

### Without drag it rings forever

Only the vehicle carried hydrodynamics, so a knocked prop moved like a body in
*air*. Measured on the first working version:

| | peak tilt | at t = 24 s |
|---|---|---|
| no drag | 87.6° | **still swinging 25°** |
| Fossen drag on the pole | 15.4° | **at rest by t = 3 s** |

The golf ball comes off either way (0.664 m drop with drag), but a flare that
never stops moving ejects balls spontaneously afterwards. Coefficients come
from the prop's own dimensions (`rod_drag`), not from taste: transverse
`½ρC_dA`, rotational `½ρC_d·d·L⁴/4`. Added mass is deliberately absent, as on
the vehicle — it belongs in `<fluid_added_mass>` and setting both double-counts.

Props share the **bare `/ocean_current`** with the vehicle, so a hydrodynamics
-enabled prop now feels the pool current too. At 0.05 m/s that is a ~0.7° lean.

### Two traps this closes

**The registry flag and `<static>` are set in two places.** `meta["dynamic"]`
only decides the buoyancy whitelist; whether a body moves is decided by
`model(..., static=)` in the build function. Disagree and nothing errors — a
prop marked dynamic but built static gets buoyancy applied to something that
cannot move, and one built dynamic without the flag gets **no buoyancy at all**
and sinks. `build_props` now refuses on either mismatch (verified by flipping
one flag and confirming the generator stops).

**DART's own collision detector silently returns false for cylinder-box**, and
the flare's base disc on the pool floor is exactly that pair. If anyone reverts
`collision_detector` to `dart` for the measured 5 %, **every flare falls
through the floor.**

### Collisions restored

Links the hull was driving through: the gate role signs and its red divider
(305 mm and 610 mm boards hanging in the gate mouth — without them the gate is
one wide opening and side-selection is not a task), the bin role panels, and
the **entire bins pipework**, which was scenery on the argument that nothing in
the task pushes the frame. True of the task, false of the vehicle, which was
descending straight through it. An approach that only works because the sim
lets the hull occupy the structure is an approach that fails in the pool.

### Contact surfaces

Nothing in this tree set `<friction>` before — fine while every prop was static
and could not slide, load-bearing the moment one can. Ballast discs carry
µ = 0.8 in **both** the `<ode>` and `<bullet>` blocks, since the world runs
DART with bullet's collision detector and which one reads the value is not
worth guessing at.

## Gain is monotonic submerged, and NOT at the surface

Measured 2026-08-30 through the lab's own HTTP teleop path, fresh world per
arm (start pose identical to 0.01 m every time), 9 s window after a 5 s
spin-up, `set_depth -0.8` + `lock_heading` so displacement is path length:

| gain | PWM | thrust/thruster | measured | drag-limited prediction |
|---|---|---|---|---|
| 25 % | 1600 | 5.94 N | 0.109 m/s | 0.109 (reference) |
| 55 % | 1720 | 20.84 N | 0.222 m/s | 0.204 |
| 100 % | 1900 | 53.21 N | **0.369 m/s** | 0.326 |

Monotonic, and slightly above `v ∝ √F` because the hull is still accelerating
inside the window — terminal speed is approached asymptotically. Heading held
92–93° throughout, so path length and displacement agree to 1 mm.

**The same A/B at the surface is non-monotonic**: unstabilized (MANUAL, no
depth hold) at z ≈ −0.2 m, the numbers were 0.112 / 0.220 / **0.077** m/s —
full gain *slower than quarter* gain, repeatably. Nothing is wrong with the
gain chain. The hull top is at the waterline in graded buoyancy with no
attitude stabilization; pitch and yaw moments grow with speed², nothing
corrects them, and the vehicle plows and curves instead of translating. Yaw
drift grew monotonically with gain (−0.01 → 0.18 → 0.29 m of lateral
displacement), which is the tell.

Two lessons for any future speed measurement here:

- **Measure in the configuration the vehicle flies** — submerged, ALT_HOLD,
  heading locked. A surface MANUAL number is real, but it is a measurement of
  attitude divergence, not of thrust.
- **Straight-line displacement is not speed.** It collapses when the hull
  curves, so a fast vehicle going the wrong way reads as a slow one. Log path
  length beside it, or lock the heading so the two agree.

The full-gain chain itself is verified end to end and clips nowhere: gain 1.0
→ PWM 1900 = `MOT_PWM_MAX` = the model's `<servo_max>` = ArduPilot's
`RC5_MAX` = 53.21 N, the T200's full published thrust at 16 V. The UI's own
thrust readout mirrors `t200_curve` to within 0.5 percentage points across the
whole slider.

## Props hinge the way they are actually moored (2026-08-31)

Two reported bugs, one root cause each, and neither was a collision problem.

**The RoboSub gate's role markers would not move.** The gate was
`<static>true</static>` — `model()` defaults to static and `robosub_gate` never
opted out. Round 6 gave the boards collision, so the hull stopped dead against
them, but a static model welds every link to the world and no amount of
collision makes a static link move.

It is now dynamic with the **frame pinned**: `joint("gate_mooring", "world",
"top_bar")` holds the moored 3 m structure exactly where the course put it — it
is far heavier than the hull, and a gate that drifted would move the geometry
the scorer measures against. The two 305 mm boards and the 610 mm divider hang
on revolute hinges about the bar's own axis, so they swing fore and aft, the
direction a hull transiting along x actually pushes them.

**The restoring force is the FASTENING, not gravity**, and that took three
measurements to get right:

| restoring model | knocked to | 15 s later | 21 s later |
|---|---|---|---|
| weight only, damping 0.02 | −56.9° | — | −5.3° (still moving) |
| weight only, damping 1.5 | −54.4° | −41.3° | — (worse) |
| **spring k=3.0, damping 0.4** | **−22.7°** | **0.0°, still** | — |

Raising damping made it *slower*, which is the tell that the missing term was a
spring and not a damper: 0.08 kg of net weight over a 0.15 m lever is 0.12 N·m,
and quadratic drag is worth almost nothing at the velocities that produces. A
real vinyl print is zip-tied flat and springs back from its fastening, so that
is what is modelled — and the board can then weigh what corrugated plastic
actually weighs (0.15 kg) instead of being made artificially heavy.

`top_bar` displacement through all of this: **0.0000 m**. The mooring holds.

**The three slalom pipes moved as one.** They were `weld_all`-ed into a single
rigid body, deliberately — "shoves the set rather than scattering three loose
poles". The handbook says otherwise ("moored at different heights to the floor,
and floating vertically", i.e. individually), and so did the pool.

Un-welding alone is **not** the fix, and the measurement said so immediately: a
free pipe took the hit, travelled 2.29 m in 3.7 s and then diverged the solver
outright. A moored pipe is not a free body. Each pipe is now a self-righting
inverted pendulum — anchor disc welded to the world, pipe on a **universal**
joint at its base (two axes, because a hull can brush a pipe from any bearing
and a single hinge yields along one heading and stands rigid along the other),
with 0.44 kg of buoyancy above the hinge standing it upright.

Measured after: centre pipe knocked to **26.6°**, home by **7.3 s**, residual
< 0.004 m and < 0.5°. Left and right pipes: **exactly 0.0000 m and 0.0°
throughout.** It can be pushed over and it cannot be pushed away.

### Two traps in measuring any of this

- **`gz.msgs.Entity` has no `parent` field.** A wrench addressed as
  `entity {name: "sign" parent {name: "gate"}}` fails to parse — and `gz topic`
  still exits **0**. No force is applied and the prop reads as "does not move".
  Entity names must be fully scoped: `gate::sign_survey_repair`.
- **Link names in `dynamic_pose/info` are unscoped and repeat across models** —
  three `pipe_centre` entries, one per slalom set. Key on the numeric `id`, or
  you will measure a different set than the one you pushed. Necessary but **not
  sufficient**: those poses are also **model-relative**, and the model frame
  rides the canonical link — see *Thruster wash on props* below.

## Thruster wash on props — DEMONSTRATED (2026-08-31)

`wake_fraction` is **not** this, and the name invites the confusion: it reduces
a thruster's own thrust as the hull's speed raises inflow at its blades. That is
a self-effect on the vehicle and applies **no force to anything else**. Nothing
in the simulator made a moving hull disturb its surroundings — which did not
matter while every prop was welded to the world, and started to the moment they
were not.

`thruster_wash` models the hull as a momentum jet pointing astern, and pushes
dynamic props inside its cone with a persistent wrench that is cleared when they
leave. The run this note asked for two rounds running — **park the hull upstream
of a prop and hold thrust** — has now been done. Hull pinned 0.7 m ahead of
`slalom_1` on `rs_task_slalom`, held at the node's own reported **+103.52 N** of
net forward thrust for 25 s:

| arm | peak pipe deflection |
|---|---|
| wash **off** | 0.001° |
| wash **on** | **7.331°** (all three pipes) |

All three pipes move equally because `_prop_xy` returns the **model's** position
for every one of its links, so they share one cone test and one force. That is
the documented simplification, not an artifact.

It is **ON by default as of round 12** (`wash:=false` to disable). The
regression the previous round asked for was run: the scorer has **no
prop-displacement rule** — its contact penalties are vehicle-versus-pool, and
the only `moved` threshold in `scoring.py` is a fired round coming to rest — so
a deflected pipe cannot move a score. `contract_check` satisfied and `smoke` OK
with it on, the node reports **16 prop positions loaded** on `robosub26_full`,
and RTF is unchanged: **median 0.0132 both ways** over ~370 samples per arm.
(Read the median, not the mean — the means were 0.0195 off vs 0.0335 on, which
is startup transient, and this is exactly the render-bound signal that one
reading cannot resolve.)

`targets` still lists **only the slalom pipes** — gate markers and flares will
not swing, whatever the cone reaches.

### The measurement frame is the trap, not the physics

**Do not measure this off `/world/<w>/pose/info`.** It reports a link's pose
**relative to its model frame**, and that frame rides the model's **canonical
link** — the first link authored, which for the slalom prop is `pipe_left`. So:

- push `pipe_left` and it reads **0.00° forever** while `pipe_centre` and
  `pipe_right` report *its* counter-rotation and appear to swing;
- push all three and every reading **collapses to ~0 precisely because it is
  working** — they swing together, and the frame swings with them.

Both symptoms were chased separately for most of a round, as "one pipe is
welded" and "multi-link wrenches don't stack". They were one artifact. The tell
was that two different bodies read an **identical** 49.48° — two things reading
exactly alike are usually one thing, not two. It was settled by watching the
**model entity's own pose**, which rotated 49.48° while its canonical link's
link-relative pose stayed at 0.00. `dynamic_pose/info` uses the same convention
and does **not** save you; compose model × link, or watch the model.

Measured along the way, and each one worth keeping:

- **Persistent wrenches on different links coexist fine.** Centre and right held
  49.9° and 55.6° simultaneously. There is no per-entity limit and no burst
  problem; the apparent one was the frame.
- **Publish rate does not matter** for a persistent wrench: 10 Hz gave 51.0°,
  5 Hz gave 50.0°, same force. It is stored until replaced or cleared.
- **Four equal thruster commands are a pure YAW, not full ahead.** The A/B rig
  drove exactly that and the node correctly reported **0.00 N** of jet. Forward
  is t1/t2 astern, t3/t4 ahead. `net_body_thrust` sums as a **vector** for this
  reason and `test_thruster_wash.py` pins it — a scalar sum of magnitudes would
  have claimed 146 N of wash off a vehicle going nowhere.

Three earlier bugs, all fixed, all of which produced a node that ran, logged and
pushed nothing:

- **The first version used hull speed as the jet speed.** Wrong quantity: wash
  is set by the thrusters, not by how fast the hull travels, and a vehicle
  holding station against a current has zero speed and full wash. Worse, the
  gate stayed `if speed > 0.05`, so the parked-hull experiment this very section
  kept asking for would have measured nothing.
- **`gz.transport13.Node` has no `publish` method.** You `advertise()` a topic
  to get a `Publisher` and publish on that; `node.publish(...)` raises
  AttributeError inside the timer callback, where it is swallowed. **Two of the
  three A/B runs were measuring a node that had never published a byte** — "the
  node is running and logging" is not evidence it is doing anything.
- **`msg.entity.type = 2  # LINK`.** In `gz.msgs.Entity` **2 is MODEL and LINK
  is 3**, so every wrench addressed a model named `slalom_1::pipe_centre`, which
  does not exist, and ApplyLinkWrench dropped it in silence. Separated from "the
  prop is stiff" by putting the same wrench on a known-free body: the
  collectible flew, the pipe did not.

## Round 16 — the hull is ours, and it has a gripper mount (2026-09-01)

### The livery, and the check that it is real

The vehicle visual is a BlueROV2 Heavy `.dae` (MIT, from `bluerov2_gz`) and it
carried **the vendor's own colours**, so every render and every operator view
showed somebody else's vehicle. `configs.yaml` now has a `livery:` block and the
template puts an SDF `<material>` on the hull and on all eight thruster visuals:
brushed 5083 aluminium `[0.62, 0.63, 0.65]` for the hull, anodised deep teal
`[0.09, 0.26, 0.30]` on the thrusters, emissive applied **per channel** for the
reason round 12 established — a flat grey lift is a desaturation term.

**DECLARED IS NOT RENDERED, so this was A/B'd against pixels.** This tree has
caught four separate cases where gz accepted a declaration and nothing changed
(`<scene><fog>`, particle emitters, a hand-written shader, a missing normal
map). The check spawns a second visual-only hull in front of the vehicle and
photographs it with the vehicle's own front camera:

| hull colour | patch RGB |
|---|---|
| aluminium `[0.62, 0.63, 0.65]` | `[123, 134, 143]` |
| forced red `[0.90, 0.05, 0.05]` | `[153, 39, 41]` |

**33.8 % of the frame changed, max delta 214.** An SDF `<material>` does
override a DAE's embedded material, so the livery needs no mesh edit.

Stated plainly because it would be easy to imply otherwise: **this changes no
dataset.** The vehicle is semantic label 0 (background) and is never a detection
target. It is an operator-view and RViz change.

### The gripper: geometry, mass and trim — nothing commands it

Modelled on the Blue Robotics Newton Subsea Gripper from its published
datasheet (62 mm jaw, 303.2 mm, 36 mm body, 524 g air / 267 g submerged, 1.6 s
open-to-close, PWM 1100–1900). **There is no public URDF or Gazebo model of this
part** — Blue Robotics' own forum, May 2024 — so it is authored here the way
every prop in this tree is authored from published figures.

`gripper: enabled: false` by default. When on: three links, two revolute jaws
with `JointPositionController` on a shared topic, and matching URDF geometry.
When off it is **textually stripped** from the SDF, not merely disabled — that
is how the range cameras cost 12 Hz → 4 Hz while claiming to be off.

**The buoyancy trap, and the arithmetic error it caught.** The collision box is
derived — `collision_z = (mass + buoyancy_adjustment) / (bx·by·ρ)` — and
`buoyancy_adjustment` is the **net** figure, displaced minus mass. So adding the
gripper's mass already adds an equal displacement implicitly, and the correction
is only the part it fails to displace: `0.524 − 0.257 = 0.267 kg`, which is its
submerged weight, as it must be. Adding the displacement *on top* of that double
counted and put the vehicle at **+0.624 kg net** — over-buoyant, from a part
that sinks. Caught by checking the derived net against the intended +0.1 rather
than by trusting the arithmetic.

Measured after: **+0.0999 kg net with the gripper off, +0.0999 kg with it on.**
`trim_kg` is the foam a real team bolts on, and it keeps the flight model —
fitted to a measured 0.95 m/s top speed — from changing under the vision work.
Set it to `0.0` to fly the untrimmed vehicle and watch it sink, which is also a
legitimate thing to want.

**Top speed is UNMEASURED with the gripper fitted.** It is off by default and
the trim keeps net buoyancy identical, so the flown model is unchanged; a
`drag_survey.py` A/B belongs to the round that turns it on.

### Why there is no DetachableJoint yet

`DetachableJoint` is the right mechanism — DART will not hold a grasped body
reliably by contact, and Harmonic's version supports attach *and* re-attach over
topics. But **the plugin names its child model in the SDF, at load time**, and
what a gripper grabs is not known until it grabs it. A first draft wrote
`<child_model>__model__</child_model>`, which compiles and attaches the jaw to
**the vehicle itself** — a self-attachment that is at best inert and at worst
something the solver fights, on the hull the hydrodynamics are fitted to. It was
removed rather than left in looking finished.

What lands with it is a runtime node that creates the joint against the model
actually being grasped, the way `payload_sim` spawns a projectile. Until then
**nothing commands the gripper**, and the octagon's object-handling points stay
`NOT_MODELLED` in `rulebook.py` — 5,100 RoboSub and 60 SAUVC. This round is
geometry, mass, trim and two working jaw controllers, and it is not a scoring
change.

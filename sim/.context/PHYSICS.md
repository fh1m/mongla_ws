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

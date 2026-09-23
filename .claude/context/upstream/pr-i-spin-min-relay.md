# PR I — `MOT_SPIN_MIN` makes the actuator a relay, and mixed-axis commands bend the thrust DIRECTION

**Target:** `srot-control-board` · **Rank: above #4, alongside #10 (and sequenced behind it).**

This is not a bug report. `MOT_SPIN_MIN` is doing exactly what you designed it to
do, for a reason you measured in water. It is a report that the *shape* of the fix
has a consequence in the stabilisation path that we can now quantify, and a
proposal that keeps the break-away guarantee while removing the consequence.

---

## 1. What we found

`mixer::oneToDshot()` (`src/control/mixer.cpp`) is the single funnel every control
output passes through — pilot stick, attitude PID, depth PID, AUTO movement
primitive and companion `MANUAL_CONTROL` alike, via the one mixing site at
`task_control_loop.cpp:897-898`:

```cpp
if (t < 0.005f) { ... return NEUTRAL_3D; }              // motor STOPPED
float thr    = thstExpo(t, expo);
float shaped = spin_min + (1.0f - spin_min) * thr;      // <-- the floor
```

With this hull's **live** parameters (read back over MAVLink, manager stopped,
2026-09-07: `MOT_SPIN_MIN 0.15`, `MOT_THST_EXPO 0.65`, `PILOT_EXPO 0.30`,
`PILOT_SPEED 1.0`, `JS_GAIN_DEFAULT 1.0`):

| per-motor demand `t` | DShot value | **% of throttle band** |
|---|---|---|
| 0.004 | 1048 | **0 %** (neutral) |
| **0.005** | **1210** | **16.18 %** |
| 0.01 | 1221 | 17.31 % |
| 0.02 | 1242 | 19.43 % |
| 0.10 | 1373 | 32.55 % |
| 1.00 | 2047 | 100 % |

**There is a step at `t = 0.005` from 0 % to 16.18 % of the throttle band.** The
first 0.5 % of demand is a cliff worth 16 points of output; the next 19.5 % of
demand is worth 28 more.

> *Implied, on assumptions we did not take from your source and are not asking you
> to accept:* with a T200 at ≈5.25 kgf and four horizontals contributing at 45°,
> that is ≈10.7 N — ≈0.54 m/s² on a 20 kg hull. **The band percentages above are
> the numbers we stand behind**; the newton figure depends on frame geometry the
> mixer matrix does not encode (it is a normalised ±1 demand mix, not geometry).

## 2. The consequence we care about, stated plainly

A break-away floor applied to a **continuous stabilisation demand** turns the
actuator into a **relay**, and a relay inside a feedback loop limit-cycles.

⚠ **RETRACTED IN PART, 2026-09-23 — see the second addendum.** This section
originally went on to claim the relay *explains* the close-in oscillation we had
been chasing. We have since closed the loop on a plant and measured it: the
magnitude quantisation on its own limit-cycles at about **a tenth of a degree
peak-to-peak**, which is roughly 3.5 mm at 2 m range and far too small to be the
behaviour we were describing. The magnitude argument is real but it is *minor*,
and we would rather hand you the correction than have you find it.

**§3 is the ask, and it survived the same scrutiny intact.** It is a direction
error, not a magnitude error, and it is large.

One null result the magnitude mechanism does still explain: we shipped a
host-side range-dependent gain softener (`vision.range_gain_floor`), measured it,
found it did nothing, and left it disabled. It could not have worked — **the
quantisation is downstream of every gain we own.** Reducing host gain does not
reduce output; it only moves commands across the cliff less often.

## 3. The part that is worse, and is the actual reason we are writing

The above is a *magnitude* problem. Mixed-axis commands have a **direction**
problem.

Your horizontal rows pair up. For a `lat`+`yaw` command:

```
m1 = +(yaw + lat)      m3 = +(lat − yaw)
m2 = −(yaw + lat)      m4 = −(lat − yaw)
```

so `m1,m2` see `|yaw+lat|` and `m3,m4` see `|lat−yaw|` — **the two pairs cross the
0.005 deadband at different commands.** When `|lat − yaw| < 0.005`, m3 and m4 stop
and only the m1/m2 pair runs. That pair's yaw and lat coefficients are equal, so:

```
achieved_yaw = (+1·T₁ − 1·T₂)/4 = T₁/2        (T₂ = −T₁)
achieved_lat = (+1·T₁ − 1·(−T₁))/4 = T₁/2
⇒ achieved yaw/lat is EXACTLY 1.0, whatever was requested.
```

The horizontal group has **two attractor directions, ±45°**, and snaps to them
inside a band. Measured across a yaw sweep at 2 % lateral:

| yaw stick | commanded | achieved | error |
|---|---|---|---|
| 0.60 % | 23.20° | 5.04° | −18.15° |
| **0.80 %** | 29.74° | **6.72°** | −23.02° |
| **1.00 %** | 35.53° | **45.00°** | +9.47° |
| 1.40 % | 45.00° | 45.00° | 0.00° |
| **2.00 %** | 55.00° | **79.42°** | **+24.42°** |

**Worst direction error 25.6°, and a 38° discontinuous jump between the 0.80 % and
1.00 % rows** — a 0.2 % change in command. The snap band at that lateral demand is
1.0 % of stick wide, i.e. 71 % of the lateral demand itself.

A gain error still pushes the right way. **A direction error that jumps
discontinuously as the controller sweeps its own axis ratio cannot be stabilised
by any choice of gains**, and a vision align sweeps exactly that ratio as it
converges.

## 4. What we are NOT proposing

- **Not lowering `MOT_SPIN_MIN`.** Your water test logged
  `"Thruster N STALLED: dshot=146 rpm=0"` at 0.02. Props have stiction; the floor
  is necessary and we are not asking you to reintroduce a measured failure.
- **Not time-domain dither.** We considered it and rejected it on your own
  numbers: a 2 ms pulse at the 500 Hz loop rate is far shorter than a T200's rotor
  time constant, so the prop never spins up and the dither delivers no average
  thrust. A dither slow enough to break away (≈2-3 Hz) sits inside the hull's own
  dynamics.

## 5. What we propose: make the floor HYSTERETIC

Stiction is a **break-away** phenomenon — starting a stopped prop costs more than
keeping a turning one turning. So the floor is only needed when the prop is
stopped, and **you already measure that**: per-thruster RPM crosses the Pico link
every cycle into `g_state.thrusters.rpm[]`, where it currently feeds only stall
detection and telemetry.

```
prop stopped  (|rpm| below a small threshold)  → apply MOT_SPIN_MIN   (break away)
prop turning                                   → no floor; allow small demands
```

This keeps the floor exactly where you proved it necessary and removes it exactly
where precision matters — during a hold, when the thrusters are already running.

### ⛔ The fallback is not optional, and it sequences this behind #10

On our vehicle RPM has been **exactly 0 in 958/958 recorded `ESC_STATUS` frames**.
A floor gated on `rpm > threshold` is therefore a **no-op on any hull whose
bidirectional-DShot telemetry is not working** — which per `ROADMAP.md:734` needs
ESCs in 3D mode plus a ~2.2 kΩ pull-up each. And if telemetry works on *some*
thrusters, an axis whose four motors disagree about their floor produces **a
torque nobody asked for** — a new failure mode introduced by the fix.

So the rule must be, per thruster:

> **no valid RPM → keep the floor.** Fail to today's behaviour.

That makes it safe to merge before the ESC-telemetry situation is resolved, and it
makes **presence-on-the-wire (#10) the precondition** for this doing anything on
this hull. The two changes belong together.

### Your own code already agrees with the premise

`thrust_trim.cpp` sets `MIN_LEARN_DUTY = 0.15f` with the comment:

> *"Below this |duty| the measurement is dominated by the ESC's own break-away
> behaviour rather than the thrust curve, so learning there teaches nonsense."*

That is the same observation from the other side: **below the floor the output is
not the thrust curve.** `thrust_trim` responds by refusing to learn there. The
proposal above is the stabilisation path making the same distinction.

## 6. What we are doing on our side regardless

Not waiting on you:

- treating the actuator as **quantised** rather than continuous, and recording the
  cliff (0.714 % stick) as a hard bar;
- **not commanding lat and yaw simultaneously in the terminal phase** — which our
  own precision-alignment guidance already recommended for a different reason, and
  which turns out to avoid the ±45° attractor entirely, since a single-axis
  command keeps all four horizontals at equal magnitude;
- refusing to emit commands inside the dead zone, rather than believing a
  correction was made.

Happy to run any experiment on the hull that would help — we have the board on the
bench and the parameter read is scripted.

---

## 7. A SKETCH, not a patch

Filed as an issue rather than a PR because this is an architectural decision in
your stabilisation path and it is yours to make — the same call we made on #11.
We have no ESP32 toolchain, so nothing below is compiled. It is here so the
proposal is concrete rather than a paragraph.

```cpp
// mixer.h — oneToDshot gains a "this prop is already turning" hint.
//   spinning == false  -> today's behaviour exactly (floor applied)
// so every existing caller that cannot know is correct by construction.
int16_t oneToDshot(float norm, int8_t dir, bool spinning = false);

// mixer.cpp — inside oneToDshot, replacing the unconditional lift:
    float spin_min = constrain(g_params.mot_spin_min, 0.0f, 0.9f);
    // Break-away only. A prop already turning does not need the floor, and
    // applying it there is what makes small demands unreachable.
    if (spinning) spin_min = 0.0f;
    ...
    float shaped = constrain(spin_min + (1.0f - spin_min) * thr, 0.0f, 1.0f);

// mixer.cpp — toDshot takes the measured rpm and decides per thruster.
void toDshot(const float norm[NUM_THRUSTERS], const int8_t dir[NUM_THRUSTERS],
             const int16_t rpm[NUM_THRUSTERS], uint8_t present,
             bool armed, int16_t dshot[NUM_THRUSTERS]) {
    for (int m = 0; m < NUM_THRUSTERS; ++m) {
        if (!armed) { dshot[m] = DSHOT_DISARMED; continue; }
        // ⛔ FAIL TO TODAY'S BEHAVIOUR. No presence bit for this thruster means no
        // trustworthy rpm, so the floor STAYS. Without this the change is a no-op on
        // any hull without working bidir DShot -- and worse, a PARTIAL telemetry set
        // would give one axis four motors that disagree about their floor, which is a
        // torque nobody commanded.
        const bool tlm  = (present & (1 << m)) != 0;
        const bool spin = tlm && (abs((int)rpm[m]) > (int)g_params.mot_spin_rpm);
        float n = norm[m] * thrust_trim::gain(m, norm[m] * (float)dir[m]);
        dshot[m] = oneToDshot(constrain(n, -1.0f, 1.0f), dir[m], spin);
    }
}
```

One new parameter, `MOT_SPIN_RPM` — the rpm above which a prop counts as already
turning. It wants to sit above telemetry noise and below the slowest genuine
running speed; `RPM_MIN_TGT` (30) is the natural starting value, and defaulting it
to 0 would disable the whole behaviour, which is a safe default if you want it
opt-in.

**A caveat we should raise rather than let you find:** removing the floor from a
turning prop means a demand can now decay smoothly toward zero and the prop will
eventually stop — at which point the floor returns and the next demand is a step
again. So the hysteresis wants a small dwell (keep treating it as spinning for
~100-200 ms after rpm drops) or it will chatter at the boundary. That is the one
part of this we would want to see measured in water rather than reasoned about.

---

## Addendum, 2026-09-22 — the table above is no longer derived. It is measured.

Everything above §2 was computed from your source. On 2026-09-22 we measured it
on the live board and it holds.

**Method.** Nothing attached to any ESC, armed, STABILIZE, `MANUAL_CONTROL` held
at twenty per second on one axis at a time, and the per-motor result read off the
**RP2350's own USB console** (`src/pico/main.cpp:405`) rather than the MAVLink
wire — which is the only place `out` exists today. Eleven demand levels, four
horizontal motors averaged, `.claude/context/workbench/data/mixer_ladder.json`.

**A decoding trap worth stating, because it cost us the first reading.** The two
DShot 3D bands each count upward from their own floor, exactly as your comment at
`mixer.cpp:131` warns. Read as a signed range around 1048, motor 1 at a 0.02
demand looks like −819 when the truth is −181, the mixer comes out asymmetric,
and the heave row grows a 3 % imbalance that does not exist. Your comment is
correct and we needed it.

**The result.** Transcribing your chain — `PILOT_EXPO` at
`task_control_loop.cpp:161`, the ±1 matrix at `mixer.cpp:26`, `thstExpo` at
`mixer.cpp:15`, the floor at `mixer.cpp:127`, the bands at `mixer.cpp:136` — and
predicting each commanded level:

| axis demand | measured, of 999 | your chain predicts | error |
|---|---|---|---|
| 0.02 | 181 | 181.6 | −0.4 |
| 0.10 | 282 | 282.1 | −0.4 |
| 0.30 | 463 | 463.6 | −0.4 |
| 0.60 | 689 | 689.4 | −0.6 |
| 1.00 | 996 | 999.0 | −3.0 |

**Worst error 3 counts of 999 — 0.30 % — and that one is at saturation.**
Everywhere else it is under one count. The firmware does exactly what its source
says, and §1's table is confirmed: the smallest demand we could command already
produced 18.1 % of the band.

**Two things this changes on our side, so you know what we did with it.**

1. We now carry a host-side model of your chain (`actuation_model.py`), so the
   companion can finally ask for an output it will actually get. It **refuses**
   any request below the floor rather than rounding it up, because silently
   delivering three times what was asked is worse than saying no.
2. It confirms the null result in §2. `vision.range_gain_floor` could not have
   worked, and we have stopped looking for the explanation elsewhere.

**The mixer matrix, also measured, since it bears on §3.** Driving one axis at a
time and differencing the +demand and −demand runs (which cancels whatever the
attitude loop was holding — M6 sat at +190 and M7 at −811 throughout) reproduces
`M[8][6]` exactly, negated by `FRAME_REVERSE = 1`:

```
        M1     M2     M3     M4  |  M5   M6   M7   M8
fwd  +1.00  +1.00  -1.00  -1.00  |   0    0    0    0
lat  -1.00  +1.00  -1.00  +1.00  |   0    0    0    0
 up      0      0      0      0  | +1   +1   +1   +1
yaw  -1.00  +1.00  +1.00  -1.00  |   0    0    0    0
```

Entries are ±1 with no geometry in them, as you say. We are **not** asking you to
change that here — it is our allocation problem, and it is on our side of the
line. We mention it only so §3's "the mixer is a normalised demand mix, not
geometry" is on the record as measured rather than read.

**What would have made this unnecessary:** `out` on the MAVLink wire. We had to
open a second USB cable to the Pico to see a number the board computes every
tick. That is PR B §`ControlState.out_*`, and this session is the strongest
argument for it we have — a companion cannot close any loop around an output it
cannot observe.


---

# Addendum, 2026-09-23 — we closed the loop, and it corrects our own §2

We built a Fossen 6-DOF plant and flew your control code on it:
`stabilize` → allocator → `oneToDshot` → plant → gyro → back in, with
`MOT_SPIN_MIN` inside the loop on every tick. Validated against the exact
analytic properties the equations of motion must have (`νᵀC(ν)ν == 0` to 1e-12,
terminal speed `√(F/q)`, the small-angle roll period), not against another model.

## 1. ⛔ The magnitude argument is smaller than we said

Yaw hold, your gains read off the board, steady state over the last 2 s of a
10 s hold:

| commanded step | with `MOT_SPIN_MIN` | floor removed |
|---|---|---|
| 30° | 0.1055° pk-pk | 0.0043° |
| 10° | 0.1051° | 0.0001° |
| 3° | 0.1051° | 0.0001° |
| 0.5° | 0.1050° | 0.0000° |

The floor does cause a limit cycle, and its amplitude is constant regardless of
step size — but **a tenth of a degree is 3.5 mm at 2 m**, well inside the pixel
tolerances our vision servo works to.

Swept across everything we have not measured: the result is **independent of drag
to 1e-6** (the cycle lives near zero velocity, where quadratic damping vanishes),
and moves **70×** across the plausible thrust and mass band. So it is an order of
magnitude, not a number, and we are not quoting it to three figures.

**Conclusion we are handing you against our own interest: the steady-state
pointing cost of `MOT_SPIN_MIN` does not justify this issue on its own.**

## 2. ⭐ But §3 does, and here it is again in demand space

The original §3 table was in stick units, which made it depend on
`PILOT_YAW_RATE`. Re-measured in **demand space**, where it depends on nothing
but your mixer, at a fixed `lat = 0.02`:

| yaw demand | commanded | achieved | error | m3,m4 |
|---|---|---|---|---|
| 0.006 | 16.70° | 3.61° | −13.09° | 2/2 live |
| 0.014 | 34.99° | 8.48° | **−26.51°** | 2/2 live |
| **0.020** | 45.00° | **45.00°** | 0.00° | **0/2 — both stopped** |
| 0.030 | 56.31° | 79.68° | **+23.37°** | 2/2 live |

**Worst direction error 26.5°. A 36.5° step between two adjacent rows**, from a
0.006 change in demand. At `yaw = lat` exactly, `|lat − yaw| = 0`, the m3/m4 pair
stops entirely, and the achieved direction snaps to exactly 45° — the attractor.

A gain error still pushes the right way. **A direction error that jumps
discontinuously as the controller sweeps its own axis ratio cannot be stabilised
by any choice of gains**, and a vision align sweeps exactly that ratio as it
converges.

## 3. ⚠ And this is specific to `vectored_6dof` — which sharpens the ask

Our closed-loop run used the **five-thruster CAD hull** through a geometric
allocator: orthogonal thrusters, each axis on its own pair, so the ±45° pairing
that produces the snapping **cannot occur there at all**. That is why the
magnitude effect was all that was left to measure.

So §3 is a property of the *eight-thruster vectored frame*, and it lands on the
competition vehicles rather than on the hull now in CAD. It is still worth fixing:
that frame is what the firmware ships, and it is what anyone else running Hengla
on a vectored hull will hit.

## 4. ⭐ Hysteresis still fixes it — and now for the better reason

The proposal in §5 is unchanged and its justification is stronger. During a hold
every thruster is already turning, so a hysteretic floor keeps all four alive;
none drops out; the two pairs never cross the deadband at different commands; and
the direction snapping **has no mechanism left**. Hysteresis was proposed to fix
the magnitude relay. It fixes the direction attractor too, and that is the one
that matters.

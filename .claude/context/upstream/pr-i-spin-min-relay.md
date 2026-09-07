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

That is not a tuning complaint — it predicts, and explains, the close-in
behaviour we have been chasing for three rounds: the hull oscillating when the
target is near and corrections are small, and a torpedo shot that cannot be held
steady. It also explains a null result we had already banked and could not
account for: we shipped a host-side range-dependent gain softener
(`vision.range_gain_floor`), measured it, found it did nothing, and left it
disabled. It could not have worked — **the quantisation is downstream of every
gain we own.** Reducing host gain does not reduce output; it only moves commands
across the cliff less often, which is a lower-duty relay.

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

🤖 Generated with [Claude Code](https://claude.com/claude-code)

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

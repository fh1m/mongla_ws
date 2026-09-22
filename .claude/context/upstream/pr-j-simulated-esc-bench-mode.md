# PR J — a simulated-ESC bench mode, so the whole stack can be exercised without thrusters

**Target:** `srot-control-board` (Pico, `src/pico/main.cpp`) · **Rank: below PR F and PR C.**
This buys *bench throughput*, not a capability in the water. It should never outrank
putting a real measurement on the wire.

---

## 1. What we are asking for

One opt-in mode in which, **for a thruster the Pico can see is absent**, it
synthesises the telemetry that thruster would have produced from the value it
just clocked out — instead of reporting nothing.

Guarded by a parameter that defaults to OFF, and marked in the telemetry so no
consumer can ever mistake it for a measurement.

## 2. Why it is nearly free — the model is already in your file

`src/pico/main.cpp:242` already runs the forward direction of exactly this model:

```cpp
float lvl = g_rpm_ff_a * (float)target;      // feedforward (both modes)
```

with `ff_a` documented in `shared/thruster_link_proto.h:72` as "feedforward slope
(norm-throttle per rpm)" and defaulting to `0.00025f` — about 4 000 rpm at full
throttle. The simulation is that line read backwards:

```
rpm_sim = out_norm / ff_a,  clamped to max_rpm,  through a first-order lag
```

`ff_a`, `idle_rpm`, `max_rpm`, `filt` and `slew` are already members of the
config struct you push down the link (`thruster_link.cpp:48-58`). Nothing new
has to be modelled, parameterised or tuned — this is the plant that `motor_tune`
already fits, run in reverse.

## 3. What it unblocks on our side, concretely

Everything below is blocked today for want of a spinning motor, not for want of
understanding:

| blocked thing | why the sim clears it |
|---|---|
| **the display's thruster indicators** | they render `rpm`, which is permanently 0 on the bench, so the panel cannot be exercised or demonstrated at all |
| **`motor_tune`'s plant fit** | it fits on per-motor RPM telemetry, which on this vehicle has never been non-zero. With no RPM the plant fit has no plant, and the mode cannot be rehearsed before the one day it must work |
| **`ESC_TELEMETRY_*` consumers, end to end** | we have never once seen a non-zero rpm arrive over MAVLink, so every decoder, unit conversion and display path downstream of it is **written but never executed** |
| **our thruster-health detector's positive class** | PR F gives us *missing vs present*. The sim gives us a *spinning* signature to write the detector against before a real thruster exists |
| **RPM-mode control, rehearsed** | `rpm_mode` and the RPM PI loop are real code that has never closed on a real number |

We measured the current bench state on 2026-09-22: `pres=0`, `rpm=0`,
`n≈998` per 500 ms window, on all eight. Your own comment at `main.cpp:393`
documents this as the correct reading for "nothing coming back at all".

## 4. The part we care about more than the feature

⛔ **A simulated number must be impossible to mistake for a measured one.**

This is the failure mode our own codebase is worst at — the recurring defect on
our side is a plausible value standing in for an absent measurement, and a sim
mode is a machine for manufacturing exactly that. If it ships without a marker,
then one day someone reads 3 200 rpm off a bench log, writes a threshold against
it, and that threshold goes to a competition.

So we would rather have it **flagged and off by default** than convenient:

1. **A parameter, default 0.** `TL_SIM_ESC`, or whatever fits your naming. Off is
   the shipped state, and it should be refused while armed — we cannot detect
   water, so arming is the strictest interlock available.
2. **Only for absent thrusters.** If `TL_ST_PRESENT` is set, report the real
   telemetry always. The sim must never overwrite a measurement.
3. **Marked on the console**, so the 2 Hz line says so — e.g. `sim=1` in the
   header, and a `SIM` marker on any motor whose numbers are synthetic.
4. **Marked on the wire.** This is the one that matters to us, because the
   console is not what a mission reads. A status bit, or the `cnt[4]` field that
   PR F is already proposing to use, so a companion can refuse a synthetic
   reading rather than trusting it. **If only one of these four lands, we want
   this one** — we would rather have the sim with no console marking than on the
   wire with no marking.
5. **Loud at startup.** One `STATUSTEXT` at boot when the parameter is non-zero.
   A vehicle that is lying about its thrusters should say so every time it
   powers on.

If marking it on the wire is awkward, we would rather you **decline this PR**
than ship it unmarked. An unmarked sim is worse than no sim: it converts a
bench convenience into a competition-day fault with a plausible number attached.

## 5. What we will do with it

Write the thruster-health detector and exercise the `ESC_TELEMETRY` path against
synthetic data, then **re-run both against a real thruster on the bench** before
either is trusted. The sim is scaffolding for code that has never executed — it
is not evidence about the vehicle, and nothing measured under it will enter
`measured-bars.md`.

---

*Evidence in this document: `src/pico/main.cpp:52,242,393-405,487-491`,
`shared/thruster_link_proto.h:72-74`, `src/drivers/thruster_link.cpp:48-58`, read
on 2026-09-22; bench state measured the same day with the board powered and
nothing attached to any ESC.*

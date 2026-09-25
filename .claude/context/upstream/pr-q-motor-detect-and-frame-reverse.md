# PR Q — ⛔ CRITICAL: MOTOR_DETECT and `FRAME_REVERSE` cannot both be right, and detect wins

**Target:** `srot-control-board` · **File as an ISSUE** (the fix is theirs to choose)
**Status:** ⏳ NOT SENT — read-only GitHub access this session (403 on issue creation).

**This outranks everything else in this batch.** Two features that are each
individually correct, that were added separately, and that neither one mentions.
On this hull — where `FRAME_REVERSE = 1` — running MOTOR_DETECT to completion leaves
the vehicle with **every axis inverted in every closed-loop mode**.

Read from source only. ⚠ **Not reproduced on hardware** — it needs a board, water,
and thrusters free to rotate. It is a short read and we would rather be wrong in
public than quiet.

---

## 1. The two features

**`FRAME_REVERSE`** (`config.h:514`, rev 7) negates all six axis demands
immediately before the mixer. `task_control_loop.cpp:893`:

```cpp
if (g_params.frame_reverse > 0.5f) {
    roll = -roll; pitch = -pitch; yaw = -yaw;
    thr  = -thr;  fwd   = -fwd;   lat = -lat;
}
mixer::mix(roll, pitch, yaw, thr, fwd, lat, norm);
```

The rationale in `config.h` is right and we agree with it: flipping all eight
`MOT_n_DIRECTION` looks equivalent but in practice an operator flips the ones that
*look* wrong, lands on a non-uniform set, and destroys the mixer's torque
patterns. Negating the demands flips every axis while leaving each pattern intact.

**MOTOR_DETECT** (`calibration.cpp:319`) pulses each thruster, projects the gyro
onto that motor's expected angular direction from `mixer::motorAngular()`, and
**composes** the sign into the stored calibration (`calibration.cpp:404-407`):

```cpp
int8_t agree = (peak > 0) ? 1 : -1;
int8_t cur = g_state.cal.motor_dir[s_motor_idx];
if (cur == 0) cur = 1;
g_state.cal.motor_dir[s_motor_idx] = (int8_t)(cur * agree);
```

The compose-rather-than-overwrite fix is also right, and its comment derives it
correctly: it converges to `c'·p = s` from any starting state, in one pass,
idempotently.

## 2. Why they cannot both hold

Grep the whole tree for the parameter:

```
$ grep -rn "frame_reverse\|FRAME_REVERSE" src/ include/ | grep -v 'params.cpp:.*{ "'
src/tasks/task_control_loop.cpp:893     <- the ONLY place in the control path
src/comms/params.h:70                  (declaration)
src/comms/mav_stream.cpp:707,737,742    (telemetry only)
include/config.h:514,529                (docs + default)
```

**One site.** And it is inside the `else` branch that reaches the mixer. The two
paths that drive a single motor for calibration are both *above* it:

- `task_control_loop.cpp:818` — the motor-test override:
  `dshot[in.test_motor] = mixer::oneToDshot(in.test_throttle, in.dir[in.test_motor]);`
- `task_control_loop.cpp:826` — MOTOR_TUNE.

Neither applies `FRAME_REVERSE`. And `mixer::motorAngular()` returns the raw `M`
column, which knows nothing about it either.

So, writing `s` for a thruster's intrinsic sign, `p` for `MOT_n_DIRECTION`, `c` for
`cal.motor_dir` and `R = -1` when `FRAME_REVERSE = 1`:

| path | effective motor polarity vs what `M` predicts |
|---|---|
| MOTOR_DETECT's stimulus | `c · p · s` |
| MOTOR_DETECT converges to | `c · p · s = +1` |
| **flight, `FRAME_REVERSE = 1`** | `R · c · p · s = **-1**` |

**MOTOR_DETECT drives the vehicle to a configuration that is correct only when
`FRAME_REVERSE = 0`.** With it set, a successful detect guarantees every axis
responds backwards in STABILIZE, DEPTH_HOLD and AUTO.

## 3. Why we think this has already happened once

Three things in the two trees line up:

- `calibration.cpp:355` records that **on 2026-08-07, in water, MOTOR_DETECT came
  out wrong for all four vertical thrusters**, and the fix attributed it to the
  degenerate `dom = argmax|e[a]|` (correct, and a real second bug — `M` gives every
  vertical `(±1, ±1, 0)`, so `|roll| == |pitch|` and `dom` never left 0).
- `task_control_loop.cpp` records that on the same day, handing an armed hull to a
  closed-loop controller straight after detect **flipped the vehicle in water the
  instant detect finished** — which is why detect now ends DISARMED in MANUAL.
- Our own `CLAUDE.md` carries: *"the `[-1] × 8` motor directions restored on
  2026-08-06 are no longer the intended configuration and would cancel it."*

`[-1] × 8` is exactly what a uniform `agree = -1` across all eight motors produces.
Our documentation already knows that state cancels `FRAME_REVERSE`; what neither
tree says is that **MOTOR_DETECT re-creates it automatically, by design, every
time it is run on a `FRAME_REVERSE = 1` hull.**

## 4. The same gap makes Motor Test lie

`task_control_loop.cpp:811` explains that the override applies `in.dir[]` rather
than `+1` because *"Motor Test is the tool people use to verify thruster
orientation, so it must show what the vehicle will actually do."*

With `FRAME_REVERSE = 1` it shows the **opposite** of what the vehicle will do. So
the one tool an operator would use to catch §2 confirms the wrong answer, and an
operator who dutifully follows the "re-arm to verify axes" instruction verifies
against a reversed reference.

## 5. What we suggest (their call)

Whatever the fix, the invariant to hold is: **the polarity a calibration routine
measures through must be the polarity flight uses.**

1. ⭐ **Cheapest, and it fixes Motor Test in the same line.** Apply `FRAME_REVERSE`
   inside `mixer::oneToDshot()`, or pass an already-reversed direction to it, so
   every single-motor path inherits it. One multiply, three call sites.
2. **Or** fold it into `motorAngular()`, so detect's expected direction is
   reversed too and it converges to `R·c·p·s = +1`. Fixes detect, leaves Motor
   Test lying.
3. **Or** refuse: have MOTOR_DETECT abort with a STATUSTEXT when
   `frame_reverse > 0.5`, saying the two are incompatible. Least code, and it
   never silently mis-calibrates.
4. **Or** make `FRAME_REVERSE` a mixer-matrix property (negate `M` once at boot),
   after which everything that reads `M` — `mix()` *and* `motorAngular()` — is
   consistent for free. This is our preference and it composes with
   [PR P](pr-p-the-allocator-they-asked-for.md), where the matrix is already built
   at boot from a table.

**Whichever is chosen, a test that asserts the two polarities agree is the thing
that keeps it fixed** — this is a cross-feature invariant, so it will not stay
fixed by review alone.

## 6. ⚠ What this is not

Not a claim that either feature is wrong. `FRAME_REVERSE` is the right knob and
compose-not-overwrite is the right detect. It is a claim that **no code path knows
about both**, and that the vehicle configuration in question is the one our hull
actually runs.

---

*Read at `task_control_loop.cpp:793-900`, `calibration.cpp:319-425`,
`mixer.cpp:137-152`, `config.h:514-529`. Not bench-verified.*

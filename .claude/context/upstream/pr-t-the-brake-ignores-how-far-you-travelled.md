# PR T — the brake's impulse does not depend on the leg's duration, so short legs reverse

**Target:** `srot-control-board` · **File as an ISSUE**
**Status:** ⏳ NOT SENT — read-only GitHub access this session.

`PH_BRAKE` applies a fixed reverse thrust for a fixed time, both derived from the
**commanded** cruise speed and neither from how long the vehicle actually
travelled. With the shipped defaults, any leg shorter than **≈0.21 s** ends up
moving **backwards**, and a 3 s leg has only **4.6 %** of its momentum removed.

Computed from their source and defaults. ⚠ Not water-verified — it predicts a
displacement, and that needs a pool.

---

## 1. The asymmetry is visible inside the one file

`movement.cpp`, cruise → brake (line 188):

```cpp
if (now - s_start_ms >= s_dur_ms) { s_phase = PH_BRAKE; s_start_ms = now;
                                    s_brake_ms = brakeMs(s_speed); }   // COMMANDED
```

`movement.cpp`, `abort()` (line 145):

```cpp
s_brake_ms = brakeMs(s_cur_speed);                                     // ACHIEVED
```

and `PH_BRAKE` itself (line 194):

```cpp
d.fwd = -s_uf * g * s_speed;        // magnitude from the COMMANDED speed
d.lat = -s_ul * g * s_speed;
```

`abort()` brakes on the speed the vehicle reached; normal completion brakes on the
speed it was asked for. Only one of those can be right, and the ramp
(`MOVE_ACCEL = 2.5`/s) means they differ for every leg shorter than `v/2.5`
seconds.

## 2. The impulse, with `MOVE_ACCEL 2.5`, `MOVE_BRAKE_GAIN 0.55`, `MOVE_BRAKE_K 0.60`

Forward impulse over a leg of `T` seconds at commanded `v`, with ramp `a`:

    T ≥ v/a :  I_f = v·T − v²/(2a)
    T < v/a :  I_f = a·T²/2

Brake impulse — note what it does *not* contain:

    I_b = g · v · (k · v) = g·k·v²        ⛔ no T at all

At `v = 0.4` that is `I_b = 0.0528` for every leg, whether it ran 0.1 s or 100 s.

| leg `T` | forward impulse | brake impulse | brake / forward |
|---|---|---|---|
| 0.10 s | 0.0125 | 0.0528 | **422 %** ⛔ net reversal |
| 0.21 s | 0.0528 | 0.0528 | **100 %** — the crossover |
| 0.50 s | 0.168 | 0.0528 | 31 % |
| 1.00 s | 0.368 | 0.0528 | 14 % |
| 3.00 s | 1.168 | 0.0528 | **4.6 %** |

So the same verb is a violent reversal at one end and a rounding error at the
other, and nothing on the vehicle notices because there is no velocity estimate.

⛔ **Short legs are not a corner case for us.** Precision alignment and the
approach to every prop are exactly where sub-second legs are issued, and they are
where being pushed backwards is least recoverable.

## 3. It is also the wrong SHAPE

For a drag-limited body the momentum to remove approaches a constant once the
vehicle reaches terminal speed, so a correct brake is roughly **∝ v with a roughly
constant duration**. The current form is `∝ v²` with duration `∝ v` — wrong in both
factors, and independent of the one variable (`T`) that actually determines how
much momentum exists.

## 4. The fix

The momentum proxy is already being integrated for free: `PH_CRUISE` maintains
`s_cur_speed`. Accumulate it —

```cpp
s_impulse += s_cur_speed * dt;        // one multiply-add per tick in PH_CRUISE
```

— and brake against **that** instead of against `s_speed`: run the brake at the
same magnitude the ramp used and end the phase when the accumulated reverse
impulse reaches `brake_frac · s_impulse`, or time it as
`brakeMs = k · s_impulse / (g · s_cur_speed_at_entry)`. Either makes the brake a
function of the distance travelled, makes short legs proportionate, and makes long
legs actually brake.

⚠ `MOVE_BRAKE_K` and `MOVE_BRAKE_GAIN` change meaning under this, so it wants a
`PARAM_DEFAULTS_VER` bump and a note — and honestly it wants the free-decay
measurement our [PR O](pr-o-thruster-step-response.md) asks for, because the right
brake fraction is a drag question. **A momentum-proportional brake with a guessed
coefficient still beats a duration-blind one with a measured coefficient**, so we
would take this change before that measurement rather than after.

⭐ Cheap check that needs no water: assert monotonicity — that net impulse over
(cruise + brake) never changes sign as `T` decreases. That is the property being
violated, and it is a pure function of these two phases.

---

*Read at `movement.cpp:140-200`, `config.h:591-596`. Impulse table computed from
their defaults; not water-verified.*

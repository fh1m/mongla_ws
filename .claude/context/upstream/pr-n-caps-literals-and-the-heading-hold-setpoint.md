# Ask N — the caps, the literals, and the one function we cannot reach

**To:** the firmware team (Rakibul Islam), on `srot-control-board`.
**Shape:** one headline PR (§2) that is small and high value, then an audit (§3–§5).

**Why now.** We built `tools/control_bench`: your `mixer.cpp`,
`attitude_control.cpp`, `feedforward.cpp`, `thrust_trim.cpp` and
`depth_control.cpp` compiled natively on a host and driven from Python, with no
reimplementation anywhere — the shim provides seven Arduino symbols and a
settable `millis()`, nothing else. It is validated against live-board DShot
captures to under one count of 999.

That means we can now say what a control change *does* before asking for it, and
everything below is measured rather than argued.

⚠ **One correction we owe you up front.** Our first run of these numbers used
`config.h` defaults. The board does not run the defaults — `DEF_PILOT_YAW_RATE`
is `45.0` and the vehicle reads back `160.0`, so every yaw figure we computed was
low by 3.556×. Everything in this document is now taken against parameters read
off the board over MAVLink on 2026-09-23, and that capture is checked in beside
the results. Where a number here disagrees with something we sent earlier, this
one is right.

---

## 1. The measurement everything below refers to

Host yaw stick → `attitude::stabilize` → `mixer::mix` → `oneToDshot`, sustained
command, live-board gains (`PILOT_EXPO 0.30`, `PILOT_YAW_RATE 160`,
`ATC_RAT_YAW_P 0.18`, `ATC_RAT_YAW_IMAX 0.222`, `MOT_SPIN_MIN 0.15`,
`MOT_THST_EXPO 0.65`):

| yaw stick | yaw torque | smallest non-zero motor output |
|---|---|---|
| 2.80 % | 0.000000 | **0.00 %** |
| 2.86 % | 0.010369 | **17.42 %** |
| 5.00 % | 0.018140 | 19.12 % |
| 20.0 % | 0.073725 | 28.83 % |
| 100 %  | 0.506651 | **70.67 %** |

Two facts fall out, and they are the spine of this document:

- **The yaw axis is 17.42 % → 70.67 %, and zero below. A range of 4.06 : 1.**
  There is no such thing as a small yaw correction on this vehicle.
- **The step at 2.856 % is a cliff, not a ramp.** It is `fabsf(yaw_stick) > 0.02f`
  in `attitude_control.cpp:87` meeting `MOT_SPIN_MIN` in `mixer.cpp:110`.

---

## 2. ⭐ THE HEADLINE ASK: let the host set the heading-hold target in STABILIZE

**`attitude::holdYaw(float yaw_rad)` already exists**, is declared in
`attitude_control.h:30`, and is already called in the flight path from
`task_control_loop.cpp:238` when `md.yaw_lock_valid`. It is proven code.

⛔ **But it is only reachable from AUTO.** In STABILIZE — which is the only mode
that honours `MANUAL_CONTROL`, and therefore the only mode our visual servo can
run in — nothing can call it. The hold target is whatever heading the hull
happened to be on when the stick last centred.

**What we are asking for:** a MAVLink command that calls `attitude::holdYaw()`
while in STABILIZE. A `MAV_CMD` with one float (absolute heading in radians, or a
delta — your call, we will take either), a handler in `mav_commands.cpp`, and the
call. We think that is a handler and one line.

**Why it is worth more than anything else on this page.** Today a sub-cliff yaw
correction is impossible: below 2.856 % of stick we get nothing, and at 2.856 %
we get a 17.42 % lurch. With `holdYaw` reachable, a 2° correction becomes *"move
the hold target by 2°"* and your own angle-P (`ATC_ANG_YAW_P = 6.0`) generates
the torque — which is a **continuous** function of heading error, so it is not
subject to the stick gate at all. Precision alignment stops being a series of
lurches and becomes a servo.

⚠ **It also fixes something we are currently doing to you.** Our vision servo
floors small yaw demands to 5 %, which takes your heading hold away and replaces
it with a 19 % kick. We have just changed our side to command a true zero instead
of a sub-cliff value, so the board keeps holding — but the *right* answer is for
us to be able to move the target you are holding to.

---

## 3. Literals that we believe should be parameters

Each of these is a compile-time constant that changes control behaviour and
cannot be read or set over the link. We are not asking for new behaviour — only
for the existing number to become visible and settable, so that the host can
model the board without hardcoding a second copy of it.

| where | literal | why it matters to us |
|---|---|---|
| `attitude_control.cpp:87` | `fabsf(yaw_stick) > 0.02f` | this single literal is the whole yaw floor. We now carry it as `actuation_model.STABILIZE_YAW_STICK_GATE`, with a test that parses your source to keep the copies equal. A parameter would delete that test |
| `mixer.cpp:102` | `if (t < 0.005f)` — the centre gap | the second half of the cliff. Also determines what `MOT_SPIN_ARM` applies to |
| `attitude_control.h:17-18` | `MAX_YAW_RATE = 2.5f`, `MAX_ACRO_RATE = 4.0f` | `MAX_YAW_RATE` is only a fallback when `PILOT_YAW_RATE <= 0`, so it is nearly harmless. `MAX_ACRO_RATE` is **the** ACRO scale and has no parameter at all |
| `attitude_control.cpp:42-44` | the `0.5f` IMAX fallback | when `ATC_RAT_*_IMAX` is 0 the loop integrates to 0.5, which is not "off" — it is a different, larger limit. It cost us a bench divergence; it could cost somebody a tune |
| `config.h:382` | `PID_D_FILT_HZ = 20.0f` | per-instance `_fltd` exists in `pid.h`, but nothing sets it from a parameter |
| `mav_commands.cpp:39` | `GAIN_MIN/MAX/STEP`, `GAIN_LOW/HIGH` | see §4 — this one is a correctness trap for us, not a tuning knob |

**We are happy to write any of these ourselves as a PR** if you would rather
review than author. Say which and we will open it.

---

## 4. ⚠ `JS_GAIN` is a silent 2× on every axis, and we cannot see it

`mav_commands.cpp:736` scales all four pilot axes:

```c
g_state.control.sp_yaw = (r / 1000.0f) * gain;
```

`gain` is `s_gain_live`, **runtime only**, stepped 0.10–1.00 by joystick button,
lazily adopting `JS_GAIN_DEFAULT` on first use. Your own comment says so.

For a human pilot this is exactly right. For us it means **every threshold in §1
doubles whenever the live gain is 0.5**, and a `param_set` of `JS_GAIN_DEFAULT`
does not change the live value — it only changes the next power-on. We read the
board today and it is at `1.0`, so we are fine *now*; we have no way to be sure
we will be fine next time except by reading `NAMED_VALUE_FLOAT 'GAIN'` and
refusing to fly if it is not 1.

**Ask:** either a `MAV_CMD` that sets the live gain directly, or — simpler and
arguably more correct — **do not apply `JS_GAIN` to frames from an onboard
computer** (compid 191). A companion's demand is already a computed number; it
has no ergonomics to soften. A human's joystick and an autonomy stack are not
the same input and probably should not share a scale.

---

## 5. The caps, honestly sorted

**We are not asking you to remove safety.** Three of these are guards and we want
them kept; naming which is which is the point of the list.

| cap | where | our position |
|---|---|---|
| `MOT_SPIN_MIN` clamped to `[0, 0.9]` | `mixer.cpp:110` | ⭐ **keep, and note that 0 is already reachable** — we had assumed otherwise. When the custom thruster lands (ask M) we may want `0`, and you have already allowed it. No change needed |
| per-group saturation scaling | `mixer.cpp:56-70` | ⛔ **KEEP.** Your worked example — forward 1.0 with yaw 0.5 costing a third of roll/pitch authority — is correct, and the block-diagonal split is the right fix. Ask K asks you to keep this even while changing the frame |
| PID `_outmax = 1.0` | `pid.h:123` | **keep.** It is the physical limit of a normalised demand; raising it would only push the clipping one stage later |
| `MOVE_CRUISE_MAX = 0.80` | `config.h:591` | **already a parameter**, and we clamp to a hardcoded copy of the *default* on our side. That asymmetry is ours to fix, not yours — flagged here only so you know that raising the board's value alone does not make our autonomous moves faster |
| `ATC_RAT_*_IMAX` | parameters | **keep**, but see §3 — the zero-means-0.5 fallback is the part that surprises |
| `PILOT_YAW_RATE` has no lower clamp | `movement.cpp:42-50` | your guard is already there and correct. Mentioned only for completeness |

---

## 6. What we will do on our side regardless

- ✅ **done today:** the visual servo no longer floors yaw to 5 %. Below the
  2.856 % gate it now commands an exact zero, so your heading hold stays
  engaged instead of being pushed aside by a demand the board declines.
- ✅ **done today:** the bench carries the board's parameters by MAVLink name,
  and a test refuses to let an attitude-cascade result be quoted from
  `config.h` defaults again.
- **next:** if §2 lands, the servo's yaw axis moves onto the hold setpoint and
  `VISION_YAW_MIN_PCT` disappears from our code entirely.

---

## 7. Related asks

- [`pr-m-thruster-requirements-from-control.md`](pr-m-thruster-requirements-from-control.md)
  — the hardware half of the same problem: the 17.42 % floor exists because of
  the thruster, and §6.1 there argues propeller sizing is the cheapest lever.
- [`pr-i-spin-min-relay.md`](pr-i-spin-min-relay.md) — the original measurement
  of the relay behaviour.
- [`pr-k-the-mixer-is-for-a-different-hull.md`](pr-k-the-mixer-is-for-a-different-hull.md)
  — the frame, and the request to keep the saturation scaling through it.
- [`pr-b-telemetry-budget.md`](pr-b-telemetry-budget.md) — the achieved wrench,
  which is what would let us see a saturation instead of inferring one.

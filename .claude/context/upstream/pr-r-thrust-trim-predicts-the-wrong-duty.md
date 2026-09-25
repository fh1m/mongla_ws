# PR R — `thrust_trim` learns against a prediction that ignores the shaping it applies

**Target:** `srot-control-board` · **File as an ISSUE**
**Status:** ⏳ NOT SENT — read-only GitHub access this session.

`THR_TRIM_EN = 1` does not normalise thrust. With the shipped defaults it drives
**every** thruster gain to its `-25 %` clamp on **every** dive, and calls that a
calibration. Latent today only because `THR_TRIM_EN` defaults to 0 — which is also
why it has never been caught.

Computed from their own source and their own defaults. ⚠ Not bench-verified: it
needs thrusters, which is our G2.

---

## 1. The two numbers that must describe the same thing, and do not

`thrust_trim.cpp:74-80` forms a ratio of measured RPM to predicted RPM:

```cpp
float meas = fabsf((float)rpm[i]);
float pred = rpm_full * fabsf(d);        // d = the NORMALISED DEMAND
...
float ratio = meas / pred;
s_k[i][band] += step * (1.0f / ratio - s_k[i][band]);
s_k[i][band] = constrain(s_k[i][band], 1.0f - lim, 1.0f + lim);
```

`d` is `nrm[]` — `g_state.thrusters.norm[]`, the mixer's normalised output
(`task_dshot_rmt.cpp:188`). But the duty the ESC actually receives is what
`mixer::oneToDshot()` produces from that same value, after **three**
transformations (`mixer.cpp:96-139`):

```cpp
thr    = thstExpo(t, expo);                               // inverse thrust curve
shaped = spin_min + (1.0f - spin_min) * thr;              // deadband lift
// (and the MOT_BAT_V_* voltage feedforward, when a pack voltage exists)
```

So `pred` is computed from the **pre-shaping** demand while `meas` is the RPM
produced by the **post-shaping** duty. The trim's job is to absorb pack droop and
prop-to-prop variation; what it actually absorbs first is `mixer.cpp`'s own
shaping.

## 2. What that costs, with `MOT_THST_EXPO = 0.65` and `MOT_SPIN_MIN = 0.15`

Both are the shipped defaults (`config.h:464`, `config.h:474` — the latter set to
0.15 from the water test that logged `Thruster N STALLED: dshot=146 rpm=0`).
`shaped = 0.15 + 0.85 · thstExpo(d, 0.65)`:

| demand `d` | `pred`/full | actual duty | `meas/pred` | learned `1/ratio` | after the ±25 % clamp |
|---|---|---|---|---|---|
| 0.10 | 0.100 | 0.326 | 3.255 | 0.307 | **0.750** |
| 0.20 | 0.200 | 0.445 | 2.226 | 0.449 | **0.750** |
| 0.30 | 0.300 | 0.542 | 1.808 | 0.553 | **0.750** |
| 0.50 | 0.500 | 0.701 | 1.402 | 0.713 | **0.750** |
| 0.70 | 0.700 | 0.832 | 1.189 | 0.841 | 0.841 |
| 1.00 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |

(taking RPM ∝ duty, which is the model the module's own header assumes.)

**The error is one-sided.** `thstExpo` is convex and `spin_min` is a floor, so
`shaped(d) ≥ d` for all `d < 1` — the prediction is *always* low, so the learned
gain is *always* below 1. `MIN_LEARN_DUTY = 0.15` restricts learning to `|d| ≥ 0.15`,
which is precisely the band where the error is worst.

Net effect of enabling the feature: **every thruster de-rated 25 %**, in both
bands, within `THR_TRIM_TAU` of arming. The module header promises *"20 % for 5 s
travels the same distance on a full or flat pack"*; what it delivers is 20 %
becoming 15 %.

## 3. Two smaller things in the same call

**The learned gain is dead in RPM closed-loop mode.** With `RPM_LOOP = 1`,
`task_dshot_rmt.cpp:104-110` sends `tgt[i] = nrm[i] * rpm_full` and `out[]` — the
only thing `toDshot()` (and therefore `thrust_trim::gain()`) touches — is never
sent. But `allow` (line 176) does not exclude `rpm_mode`, so the trim keeps
learning and its gain is applied to a discarded array. Harmless, and it means a
number is being maintained that cannot reach a motor.

**In that mode `pred` is exactly the target**, so the ratio measures the *Pico's
tracking error* rather than a thrust curve — a different quantity again, learned
into the same variable.

⚠ **Separately, and worth a decision of its own:** with `RPM_LOOP = 1` the whole of
`MOT_THST_EXPO`, `MOT_SPIN_MIN` and the `MOT_BAT_V_*` feedforward is bypassed,
because none of it is in the RPM path. Closing on RPM makes the voltage
feedforward redundant (correctly), but it also makes command→thrust quadratic
again — the thrust linearisation `MOT_THST_EXPO` exists to provide is simply
absent. That may be intended; it is not written down anywhere.

## 4. The fix, and why it is one line of plumbing rather than new maths

The honest prediction is the duty that was actually sent, and `mixer.cpp` is the
only place that knows it. Either:

1. ⭐ Have `oneToDshot()` publish the `shaped` value it computed (a
   `float mixer::lastShaped(int m)`, or fill a `shaped[NUM_THRUSTERS]` out-param on
   `toDshot()`), and feed **that** to `thrust_trim::update()` in place of `nrm`.
   One truth, one copy, and it keeps working when the shaping changes.
2. Or replicate `thstExpo` + `spin_min` inside `thrust_trim`. Works, and it is a
   second copy of the shaping — which is how these two drifted apart.

Then exclude `rpm_mode` from `allow`, so a gain is only learned where it can be
applied.

⭐ **And whichever way: a test that drives a demand ladder through
`oneToDshot()` and asserts the learned gain stays at 1.0 for an ideal thruster.**
That is the property the module claims, it is checkable on a host with no
hardware, and it would have failed on day one.

---

*Read at `thrust_trim.cpp:44-94`, `mixer.cpp:96-139`, `task_dshot_rmt.cpp:100-190`,
`config.h:464-484`. Table computed from their defaults; not bench-verified (needs
thrusters — our G2).*

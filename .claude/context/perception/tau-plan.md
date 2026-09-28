# tau = scale ÷ expansion rate — the plan

> **Status:** measured, NOT wired. Section 62 of `measured-bars.md` has the
> numbers. This file is the route from there to a shipped rung.

## Why this one and not the others

Six model-free candidates were built and measured in one pass (sections 58, 60,
61). Five were rejected outright. This is the survivor, and the reason is
structural rather than lucky:

| candidate | why it died |
|---|---|
| colour back-projection | loses to a decoy box on 69 % of frames |
| NCC template matching | 0.891 peak, +0.005 margin over its own second best |
| ECC photometric | converges on 17.6 % of the frames LK cannot carry |
| vanishing point | 20 227 px jitter while its own quality metric read 0.958 |
| attenuation prior | sign flips per class across clips |
| cross-model ensemble | +43.2 pts against +54.0 for the better model alone |
| **tau from scale** | **nothing wrong with the quantity — the estimator is noisy** |

That distinction is the whole point. The others answer the wrong question or
answer it with no information. tau answers the right question, from a number we
already compute, and the only problem is variance — which is a filtering
problem, and filtering problems are closed by ground truth.

## What it is

`tau = s / (ds/dt)`. Seconds until the target's apparent size goes to infinity,
i.e. until contact. **The metres cancel**, which is why it works on a monocular
vehicle with no DVL — and why `time_to_contact.py`'s stated blocker ("needs
metres we do not have") was the wrong blocker for nine months.

## ⭐ We compute the scale in FOUR places and use none of them for this

| source | field | rate | independent of |
|---|---|---|---|
| follower | `FollowResult.scale` | ~50 Hz | — |
| anchor | `AnchorPose.scale` | 4.5 Hz | the follower (frame-to-reference) |
| flow | `PlanarMotion.scale_rate` | 30 Hz | both (whole-field, downward) |
| detector | box height | ~30 Hz | all three (no tracking at all) |

Four estimates of one quantity, three of them already cross-checked against
each other for other purposes. That is the redundancy Bumblebee reflex 1 asks
for, and it is already on the vehicle.

## The measurement that stops it shipping

600 frames of a real approach, tau from the detector's box height:

| | |
|---|---|
| tau p50 | 2.5 s |
| positive tau | 299 of 600 frames |
| frame-to-frame \|Δtau\| p50 | **2.4 s** |
| frame-to-frame \|Δtau\| p90 | **27.2 s** |

The swing is the size of the value. And half the frames give a *negative* tau,
correctly — the target is receding — so "not approaching" is a state the
consumer must handle, not a large number.

⚠ **Do not fix this by smoothing and shipping.** `anchor/geometry.py` already
paid that bill: a 31-frame rolling median took the plane tilt's p90 swing from
39° to 1.3°, and two independent snaps of the same board still disagreed by
17.5° at p90. **Smoothing bought smoothness, not accuracy.** A filtered tau
would look excellent on a plot and be wrong in the same reference-dependent way.

## ⭐⭐ The scorer is already written, and it changed the problem statement

`tools/tau_from_scale.py --self-test` runs the ranking against synthetic
sources whose bias we chose, so a bug in the scorer cannot be mistaken for a
bias in a sensor on the day — when re-running costs a pool session. It found
two things before any water:

| synthetic source | bias | slope | tau jitter p90 |
|---|---|---|---|
| perfect | −0.00 | −1.00 | 0.04 s |
| **scale biased +20 %** | **−0.00** | **−1.00** | **0.04 s** |
| scale noise 2 % | −2.23 | −0.66 | **23.28 s** |
| wrong exponent (r^−1.15) | −0.96 | −0.87 | 0.03 s |

⭐ **A constant scale bias scores IDENTICAL to perfect.** tau is
scale-invariant, so a source that is 20 % off everywhere cancels exactly. **No
calibration is needed and none can help** — which is precisely why this is the
right quantity for a vehicle with no DVL, and why it does not inherit the
flow path's dependence on a height nobody measures.

⭐⭐ **So only two things can go wrong, and the run distinguishes them:**

* **NOISE.** 2 % scale noise produces **23.3 s** of tau jitter here, against
  the **27.2 s** p90 measured on real footage. **Our problem is therefore
  roughly 2 % scale noise** — which makes the open question quantitative and
  small: *how many frames of averaging bring 2 % down far enough*, and does the
  vehicle have that many before contact.
* **SLOPE.** A wrong exponent — apparent size not going as 1/range, which is
  what a tilted or partly-occluded prop gives — reads −0.87 instead of −1.00,
  and **no amount of filtering fixes it** because the scale itself is wrong.
  This is the failure mode that must not be smoothed over.

## The run that closes it

**One approach at a known constant speed from a known start range.** That makes
true tau a straight line falling at exactly −1 s per second, which turns every
open question into a fit:

1. park at a measured range from a fixed prop (tape on the deck, prop on the
   bottom);
2. command a constant surge, no depth or yaw changes;
3. record the bag with the detector, the follower and the flow all running;
4. true `tau(t) = (R0 − v·t) / v`, a line with slope −1.

Then, offline and against that line:

* **which of the four scale sources is least biased** — they can finally be
  ranked rather than merely compared, which is the "truth test, not an
  agreement test" rule this repo already holds itself to;
* **how much filtering the variance actually needs**, as a fit rather than a
  taste;
* **whether the bias is reference-dependent** — run it twice from different
  start ranges and check the two agree, which is the two-snap control that
  caught the tilt estimator;
* **where the negative-tau boundary sits** in practice.

⚠ The whole run is a few minutes of pool time and needs no new hardware. It is
the cheapest item on `first-pool-verification.md`.

## What ships after it

`time_to_contact.py` gains the scale-based entry point beside its existing
metric one, and its deferral in the reachability register closes. The consumer
is the approach/standoff path — but note that `approach.py` also stays deferred
for a *different* reason (no controller acts on BACK_OFF yet), so closing tau
does not by itself put anything on the control path. **Two doors, both need
opening, and neither is the other.**

## What NOT to do

⛔ Do not wire tau to a control law before the run. A time-to-contact that is
wrong by a factor of two commands a brake at the wrong moment, and the failure
is a collision rather than a missed detection.
⛔ Do not take tau from the follower's scale on a clip where the detector is
healthy. Measured: `Follower.scale` reads exactly 1.0000 at p10, p50 and p90
across 600 frames, because a continuous detector re-seeds every frame and
`step()` never runs. **Any follower-scale measurement must be taken across a
real detection gap.**

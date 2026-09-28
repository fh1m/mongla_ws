---
name: prove
description: Try to falsify a change rather than review it — build the input that breaks it. Use after an implementation passes, before trusting it, or when asked to "prove this works".
---

# /prove

Ordinary review asks *is this good code?* This asks a different question:

> **Assume it is wrong. Find the case that shows it.**

You have failed at `/prove` if you finish without having tried to break anything.

## Method

1. **State the claim precisely.** "`approach_band` returns the band" is not a claim.
   "For every frame with a detection, it returns a band whose width is ≤ X at range Y"
   is.
2. **List what the claim assumes.** Each assumption is an attack surface.
3. **Build the input that violates one.** Write it as a test. Run it.
4. **Rank what you found** by whether it can happen on the vehicle, not by how clever it
   is.
5. Found nothing? Say exactly what you tried and what remains unprobed. An honest empty
   result is fine; a silent one is not.

## Where this codebase actually breaks

Attack these first — every one is a defect that has already happened here:

- **The plausible number standing in for an absent measurement.** The recurring defect.
  What does the function return when the sensor is missing, the detection list is empty,
  the frame is stale? If the answer is a number rather than `NaN`/`--`, that is the bug.
- **Absence vs zero.** `0.0` where the truth is "unknown". 958/958 ESC frames read 0
  with nothing attached.
- **A verb that reports success while the vehicle does nothing.** `set_mode` is
  best-effort on this wire — a silent refusal looks exactly like success. Does the
  caller verify the mode took?
- **Sensor dropout and recovery.** Detection gaps, a lock ladder falling through, an
  unhealthy barometer denying every move verb while `VFR_HUD` still reports a depth.
- **Multiplexed and cached messages.** `BATTERY_STATUS` is instanced and pymavlink
  caches per msgid — sampling the slot alternates between two packs. Seven
  `NAMED_VALUE_FLOAT` names burst inside one tick and the single slot keeps only the
  last.
- **Timing and ordering.** Late measurements. Frame age. A 500 Hz board against a 30 Hz
  camera. Anything that assumes messages arrive in the order they describe.
- **Sign and frame conventions.** Depth is negative below the surface. `FRAME_REVERSE`
  negates all six axis demands before the mixer. Rev 10 inverted yaw. Which hull is
  this number about — the 8-thruster competition vehicle, or the 5-thruster CAD?
- **Reachability.** Is the new code imported by anything that runs? A module with
  correct code, passing tests, and no consumer is the most expensive bug in this repo.
- **Thresholds without measurement.** Is the constant in `measured-bars.md` with a
  method, or did someone pick it?

## Ways to actually falsify

Prefer a mechanism that produces a counterexample over one that produces an opinion:

- A property test over the input range, not one hand-picked case.
- Replay a real bag, including the ugly footage — not sim imagery.
- Cross-check two independent estimators against a case where truth is known.
  **Comparing a new estimator to the incumbent measures agreement and cannot rank
  them.**
- Injection-verify the guard: break the thing deliberately, watch the test fail, restore
  it. A test that has never failed against a real defect is not a guard.
- Differential: same input, two code paths, compare.

## Output

Findings ranked by whether they can happen in water, each with the concrete input that
triggers it and the observed wrong value. Not a list of concerns.

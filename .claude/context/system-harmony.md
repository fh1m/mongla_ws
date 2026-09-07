# System harmony — the four rules, the timing budget, and the settled negatives

> Round 37 item 7, written 2026-09-07. **Every number here is measured on
> this hardware and cited to its round.** Where a number is assumed rather
> than measured, it says so.

The subsystems on this vehicle are not weak individually. They were
**uncoupled** — each holding a private, differently-timestamped opinion, with
six thresholds sized against each other rather than against one measured
quantity. This file is what they now agree on.

---

## 1. TIME IS THE SUBSTRATE — and we shipped this bug three times

A measurement carries its **capture instant** or it does not enter the
system. Not a convention: a type.

We have shipped the same defect three times, and each time it produced a
plausible number rather than an error:

| where | what happened |
|---|---|
| `camera_node` | `header.stamp = now()` at PUBLISH, so every freshness gate — **including the torpedo fire gate** — was blind to queueing |
| `vision_state._on_detections` | `_latest_stamp = time.monotonic()` on ARRIVAL, so `age_s` measured time-since-message, not time-since-shutter |
| the `.tlog` writer | a little-endian timestamp round-trips **successfully** — pymavlink scans forward and recovers, so the only symptom is every time wrong by decades |

**The rule:** pass the stamp through, never resample it. When a clock domain
changes (wall ↔ monotonic), convert **once**, in one place, and guard: a
negative or absurd age falls back with a WARN rather than silently.

**Measured consequence of getting it right:** `age_s` went from reading ~0
(meaningless) to a real ~32 ms floor, which is what made `VISION_FRESH_FULL_S`
re-derivable from the observed detection interval instead of from "one frame
at 10 Hz", a rate this vehicle has not run at for two rounds.

---

## 2. THE CONTROL PATH STAYS OUT OF THE EXECUTOR

ROS 2 pub-sub is **nondeterministic by construction** — callback order is
nondeterministic within a single executor, and priority inversion in the
default executor is documented (TU Dortmund / UC Berkeley, arXiv 2606.09203).

That is the formal statement of what we measured as a **76 ms jitter
artifact** in round 25 — which was itself a harness error: control running as
*threads inside the vision process*. As separate processes, the real
arrangement, it is **sd 0.03 ms**. That error would have inverted the srot
recommendation.

So: camera → detector is a **direct Python handoff in one process**
(round 32), not a ROS hop, and the estimator follows the same shape. rclpy
has **no intra-process comms** — that is rclcpp only — so
`ComposableNodeContainer` buys nothing here and is a settled negative below.

---

## 3. HEALTH IS AN OUTPUT, AND **UNKNOWN IS NOT OK**

`duburi_manager/health.py`: every subsystem reports
`(state, evidence, age)` — never a bare bool — in four states:

```
OK       reporting, and within its bar
DEGRADED reporting, outside its bar, still usable
UNKNOWN  NOT reporting. WORSE than degraded: nothing is watching.
FAILED   reporting, and unusable
```

**`UNKNOWN` ranking below `DEGRADED` is the whole design.** It is the rule
that caught, on real hardware:

- `BARO_HEALTH = 3` read as a health *score* when 3 means **not initialised**
  — a fault code mistaken for a good number;
- the ESC gate, which must report UNKNOWN with no telemetry rather than OK;
- `DEPTH_ERR`/`DEPTH_OUT`/`WTEMP` absence, which is *no data*, never zero.

A motionless bench reports **exactly 0.0**, and so does a dead sensor.

---

## 4. THE TIMING BUDGET, photon → thruster

Every row measured on this vehicle. Cite the round before reusing one.

| stage | measured | where |
|---|---|---|
| capture → frame available (mailbox) | **16.9 ms** staleness (vs **396 ms** on a plain queue) | round 30 |
| photon → detections, end to end | **18.0 ms** median | round 32 |
| detector, Hailo-8, standalone | **80.9 Hz** | round 26 |
| detector, through the ROS graph | **53.9 Hz** | round 26 |
| control loop, srot | **49.86 Hz** soaked 90 s, 0.00 % late ticks | round 26 |
| `ATTITUDE` feedback | **50 Hz** (firmware floor `RATE_MIN_MS` = 20 ms) | round 26 |
| `MANUAL_CONTROL` uplink | ≥ 10 Hz required; authority ramps to zero between `MANUAL_FRESH_MS` 1000 and `MANUAL_DECAY_MS` 1500 | firmware |
| board inner loop | **500 Hz** | firmware |
| serial link | **18.8 %** of 11520 B/s | round 26 |

**Two clocks, and only one of them is usable.** The board's own clock is
**perfect** — `time_boot_ms` period 100.000 ms, **sd 0.000, p2p 0.000** over
200 samples. Host arrival of those same samples is **sd 4.741 ms, p2p
30.85 ms**. So *arrival* jitter is a property of the link and the host, not
the vehicle: map board time to host time (`ClockMap.to_host`, **sd 0.528 ms**
vs arrival's 6.402 ms) rather than timestamping on receipt.

**The ceiling, and why 50 Hz ships.** The loop saturates at **70.1 Hz** with
every other resource idle — Pi CPU 15 %, uplink 18 %, board load 0.0, drops
0, errors 0. Inference is 12.17 ms of a 14.2 ms loop. 50 ships because the
sweep is single-process while the real stack is two processes across DDS,
which measured 53.9 Hz.

---

## 5. SETTLED NEGATIVES — measured, not argued

Recorded so they are not re-proposed. Each has a number or a citation.

| rejected | why |
|---|---|
| **`ComposableNodeContainer`** | rclpy has no intra-process comms (rclcpp only); composed Python nodes still traverse rmw. Reduces process count, **not latency**. |
| **Async Hailo inference** | measured: host code is already at **100 % of the chip** — 98.0 Hz vs `hailortcli --hw-only` 97.9. |
| **`engine=nn_core`** (on-chip NMS) | supports YOLOv5 / SSD / CenterNet only. YOLO11 uses `meta_arch=yolov8`. **Structurally unavailable**, not a tuning choice. |
| **YUYV to skip the decode** | 640×360 YUYV fits USB 2.0 only at ~60 fps, raising the age floor 4.8 → 16.7 ms. Net loss. |
| **Zero-copy** | buffer copy is **13.1 µs** against a **2450 µs** decode: 0.5 %. |
| **Factor graphs (`fuse`, GTSAM)** | **4.5 ms/iter vs 0.06 ms** filtering. We need real time, not loop closure. |
| **UKF** | "substantial equivalence to EKF on real data" despite better theory. |
| **`robot_localization`** | no IMU bias estimation, static covariances. |
| **A global position estimate** | **unobservable** with IMU + baro and no velocity sensor. Target-relative only. The easiest wrong number this project could produce, and the hardest to notice. |
| **An IMM filter bank** | wrong tool: our targets are static; the motion to model is our own. |
| **Enhancement before the detector** | measured **−29 points** (round 35); on the gate it destroyed 95 % of detections. Candidate on the **matcher** path only — a different consumer, and that is not a contradiction. |
| **XFeat for the FLOW rung** | **33.1 ms vs 8.0 ms** for LK on one core of this Pi. It stays the ANCHOR rung at 5–10 Hz. |
| **Feed-forward two-view (DUSt3R/MASt3R/VGGT)** | the accuracy frontier, categorically out of budget on a Pi 5. |
| **Deep single-image calibration** | median focal error 4–27 %; we need ~1 %. |

---

## 6. THREE OBSERVABILITY LIMITS, so they are not rediscovered

1. **Horizontal position is unobservable.** Our depth *is* absolute (Bar30)
   and our yaw *is* absolute when `YAW_REF == 2`, so our unobservable set is
   smaller than generic VIO — but global position still drifts without
   bound. **Target-relative only.**
2. **Bias observability needs excitation**, and station-keeping is our most
   common state and worst estimation case. **Accel bias is not estimated**;
   gyro bias only behind an excitation gate.
3. **NIS/NEES are chi-squared only for an ALREADY-TUNED estimator.** They are
   a **falsifier**, never a certificate.

---

## 7. THE METHOD RULES THAT EARNED THEIR KEEP

Every one of these was paid for with a wrong answer that looked right.

- **Always score the trivial model.** A whole-frame box scored **100 %** at
  IoU 0.3 where the real detector scored 73.6 % and looked healthy; at IoU
  0.5 the detector (9.7 %) was **worse than the null** (15.3 %). Four lines.
- **A test that can only see a defect smaller than itself does not test for
  it** — a 0.5 s leak test on a 0.5 s window.
- **Verify a guard by injecting the defect it exists for.** A `grep` for
  `_srot_drive(` stayed green through exactly the rename it guarded.
- **Compare what SHIPS**, not a parallel implementation of it. An A/B on the
  bare library would have reported that tracking buys nothing.
- **Split the aggregate before believing it.** A pooled 16× was clip
  composition; a sampled sweep reported 307/307 by arithmetic.
- **Least squares never refuses.** Gate a fit on excitation AND a held-out
  score or it returns confident nonsense.
- **A correctly identified mechanism does not make the obvious fix for it
  correct.** Both de-rotation fixes were refuted by their own measurements.
- **Absence is not zero.** A motionless bench and a dead sensor both report
  exactly 0.0.

## Related

- [`measured-bars.md`](measured-bars.md) — every bar, its measurement, its guard
- [`camera-and-calibration.md`](camera-and-calibration.md) — intrinsics, the library, in-water mode
- [`water-owed.md`](water-owed.md) — what only a pool can close
- [`srot-integration.md`](srot-integration.md) — the board contract

# The ladder register — everything built, and whether it actually runs

> **One page, checkable.** Every capability built for the lock ladder across
> the 2026-09-23/25 rounds, its evidence, and its REAL wiring state read from
> `test_no_capability_is_built_and_unreachable.py` rather than from memory.
>
> ⛔ The repo's oldest and most expensive defect is a module that is finished,
> tested, and imported by nothing. Ten of the fourteen below are in that state
> **on purpose**, each with a named blocker. A row here is a debt with a name.

## Scoreboard

| | count |
|---|---|
| WIRED — imported by something that runs | **4** |
| DEFERRED — built, tested, reachable by nothing | **10** |

---

## WIRED

| module | what it does | evidence |
|---|---|---|
| `anchor/bank.py` | checkpoint bank; signature shortlist, eviction by inlier yield, contamination guard refusing enrolment when health says CAMERA | 92–100 % trust-bar clearance cross-run |
| `anchor/health.py` | fouled lens vs changed world; `trend()` anticipates a fall before it happens | `TREND_FALL 0.70`, compares halves not slope |
| `anchor/loop_closure.py` | place recognition; centre-displacement offset, not origin | bar 100 inliers, `MIN_TRAVEL_M 1.5` |
| `tracking/lock_state.py` | the ladder state machine, authority decaying from last real detection | gap p50 0.155 s / p90 0.651 s / p99 2.418 s |

Plus, in the nodes themselves: the camera **mailbox** (16.9 ms stale vs 396 ms
queued), the follower's **RANSAC similarity fit** (carries SCALE, which the
approach controller reads), replay mode, and — new — the **acting bar** below.

---

## DEFERRED, with the blocker

| module | what it would give | blocked on |
|---|---|---|
| **`tracking/visibility.py`** ⭐ | **the FOV guard. There is no field-of-view constraint anywhere in `mongla_control`, so perception can be perfect and the lock still dies because the vehicle turned away** | a consumer in the vision verbs: yaw/lateral demand scaled by `Visibility.scale`, and only the component making the bearing WORSE. Control change ⇒ needs a pool day |
| `tracking/cascade.py` | association cascade; expensive appearance stage only on detections motion cannot explain, with a MEASURED ego-motion stage no published tracker has | `tracker_node` calls its tracker library directly; inserting a cascade touches the association hot path, rate must be measured on the vehicle |
| `tracking/reid.py` | the 179-switch fix, using XFeat already loaded at 701 FPS instead of a 15–25 ms Re-ID net | crop descriptors at track birth/death |
| `tracking/presence.py` | the rung BELOW detection (track-before-detect) | **B-59 — may never wire.** The band below the bar is confident hallucination, not faint signal |
| `continuity.py` | gap distribution analyser | offline only; its constants are a person in air |
| `time_to_contact.py` | τ with ego-velocity subtracted | no approach/standoff controller exists |
| `approach.py` | standoff band, back-off included | the band is measured; the controller that acts on it is not built |
| `flow/scale_check.py` | flow-height vs baro-height cross-check | the barometer reports "not initialised" on this hull — the comparison has one side |
| `detection/confidence_calibration.py` | turbidity → calibrated confidence | gain is DECLARED, not measured; needs a labelled per-venue set |
| `draw_strip.py` | — | superseded UI, 471 dead lines. Delete or revive deliberately |

---

## What these rounds actually changed on the vehicle

| change | measured on | result |
|---|---|---|
| **acting bar** `act_conf` 0.45 in `lock_node` | 3 Mirpur clips × 1 200 frames | gate-on-empty-water 90.8 % → 46.8 %, real gate held at 98.9 % |
| **pre-dive floor gate** in `bringup_check` | the 4 deployed HEFs | FAIL naming `gate_rescue_repair` 0.200 and `yolov11n` 0.200 |
| **HEF recompiled** at `nms_scores_th 0.05` | `hailortcli parse-hef` | `Score threshold: 0.050`, 3 classes — verified on the artifact |
| **`tools/hailo_compile.sh`** | — | the compile is now a standing capability, not a thing rebuilt and thrown away |
| **`tools/negative_clip_check.py`** | — | closes the hole that hid B-59: no bar may be set from positive frames alone |
| **`tools/data_root.py`** | `df` on `/tmp` | 5 defaults across 4 tools moved off the RAM disk that ate Round 7 |

---

## Dead ends, recorded so they are not re-derived

Three runtime false-positive rejectors were tried against B-59 and **all three
failed** (full table in `BUGS.md`):

- **temporal persistence** — the literature's "false positives are isolated in
  time" does not transfer: ours persist 400 frames of 400, because the camera
  stares at the same empty water;
- **box geometry** — area 0.285 real vs 0.601 and 0.131 hallucinated, straddling
  both sides;
- **edge energy inside the box** — 6.69 real vs 6.82 and 9.40 hallucinated,
  i.e. *backwards*. Turbid water is not smooth; what a human reads as "nothing
  there" is semantic, not photometric.

## The honest state of "never lose the target"

The ladder is **strong after acquisition** — follow, anchor, place memory, and
now an acting bar that stops it acquiring garbage. It is **weak at the two
ends**:

1. **below detection** — closed as a dead end for now (B-59);
2. **outside the frame** — `visibility.py` exists and is not wired, so nothing
   stops the vehicle turning away from a target it is successfully tracking.

⭐ **(2) is the largest remaining gap in the whole pipeline** and the only one
whose module is already written and measured.

---

## Work left — the running list

> Kept HERE because it has been lost twice in conversation. Anything picked up
> gets struck from this list in the same commit that lands it.

### Blocking, in order

1. ✅ **B-60 RESOLVED by model selection.**
   `pendrive_2/final/models/gate/weights/best.pt` scores **100 % positive and
   0.0 % across 1 200 negative frames from 3 venues**, at every threshold.
   The shipped model fires on 100 % of a gate-free clip at 0.15. Remaining:
   its top box on `gate.mkv` is still slightly off structure; it carries
   `gate, shark, shaw_fish` and NOT `rescue`/`repair`, so promoting it changes
   the mission surface; and it must be compiled at >= 1024 calibration frames
   before it flies.
2. ✅ **The archive was searched.** `tools/model_inventory.py`: **208 `.pt`,
   96 unique after content dedup, 29 with a gate class.** 96/96 have never
   been measured against absent-prop footage — that column is now tracked.
3. **`rescue` / `repair` / `sauvc_sim` / `bin_fire_blood` still unmeasured**
   for the same defect.
4. ✅ **B-61 — ROOT CAUSE FOUND AND FIXED: BGR/RGB.** The Hailo input path
   never converted `cv2`'s BGR to the RGB the model was trained on, so the
   chip ran on colour-swapped pixels since the backend was written. Proved by
   reproducing it on the `.pt` with no chip involved: the same swap turns
   `gate=0.78` into `repair=0.92 @0,0`. ⚠ **Two wrong diagnoses came first**
   (decode stride; the DFC calibration cliff — a real defect, fixed
   separately, but not this one). **Every HEF-path measurement must be
   re-taken**: 80.9/53.9 Hz, the murky INT8 table, "INT8 costs 0.08". The
   `.pt` numbers (B-59, B-60) are unaffected.

   ~~B-61 — cause found, fix in flight.~~ The HEF returned `repair` 0.86 with
   an impossible box (`y2 < y1`, `x1 > 1.0`) because the calibration set was
   258 frames against the DFC's 1024 cliff, which silently drops AdaRound and
   QAT. The decode is EXONERATED by a raw-buffer dump. `hailo_compile.py` now
   refuses below 1024 and stages frames as uint8. **The 1024-frame rebuild
   must reproduce the `.pt`'s classes before the HEF path is trusted again.**

### Wired half-way

5. **`visibility.py`** — `assess()` is wired advisory; **`WorldTarget`,
   `project_world_target`, `region_conf_bar` are reached by TESTS ONLY.** That
   is the object-permanence half: what lets a target leaving frame become a
   navigation problem rather than a forgotten one. ⚠ The reachability register
   cannot see this — it checks modules, not capabilities.

   ⛔ **The blocker, measured 2026-09-25, is two missing signals — not a
   missing consumer.** `WorldTarget.observe()` needs `vehicle_xy`, `yaw_deg`,
   `bearing_deg` and `range_m`. `lock_node` has the first (`_odom_xy`) and can
   compute the third (`bearing_from_pixel`). It has **neither yaw** (never
   subscribed; yaw lives on `/mongla/state`) **nor an absolute range** — only
   a RELATIVE scale from the anchor's reference-patch width, which is metric
   only if the patch's true width is known.
   The module refuses a guessed range by design, and it is right to: *"a
   remembered position built from a guessed range would send the vehicle
   confidently to the wrong place."* So wiring it means either subscribing
   `/mongla/state` for yaw plus establishing one metric width per prop, or
   deriving range from `_altitude_m()` for downward-camera targets only.
   **Do not wire it by inventing a range.**
6. **FOV demand-scaling** — measured and advisory only; scaling waits on water.

### Deferred, unchanged

7. `cascade.py` · `reid.py` · `presence.py` (B-59/60 may retire it) ·
   `continuity.py` · `time_to_contact.py` · `approach.py` ·
   `flow/scale_check.py` · `confidence_calibration.py` · `draw_strip.py`
   (delete or revive)
8. **Never started:** Round 11 (homography → yaw/altitude/obliquity) ·
   Round 14 / POS_HOLD without DVL · downward place-bank preloading ·
   Hailo async API (13.4 → ~7 ms) · `track_id` on `/detections` (its absence
   makes every id-switch number meaningless) · `geometric_allocation.py` triage
9. **`measured-bars.md` §1 must be re-derived** once a model passes
   `negative_clip_check.py` — every bar there came from positive frames alone.

---

## Re-ID: the blocker was misdiagnosed, and the real one is architectural

`tracking/reid.py` has been deferred on the note *"XFeat is ALREADY loaded at
701 FPS … waits on tracker_node supplying crop descriptors at track birth and
death"*. Fixing B-62 was supposed to unblock it. It does not.

⛔ **`tracker_node` has no images.** It subscribes to exactly two topics —
`detections` and `camera_info` — and nothing else. It owns track identity and
has never seen a pixel. Re-ID needs a *crop descriptor*, so the missing
ingredient was never XFeat's cost; it is that the node holding the identity
cannot compute an appearance at all.

**Three ways out, and none is free:**

1. **Subscribe images in `tracker_node`.** Puts a second full-resolution
   subscriber on the hot path and duplicates the camera mailbox the detector
   already owns. ⚠ Measured elsewhere in this repo: a queue-bound consumer sees
   396 ms of staleness against 16.9 ms on the mailbox, so this is the option
   most likely to be quietly wrong.
2. **Compute descriptors in the DETECTOR**, which already holds the frame and
   the boxes, and publish them alongside the detections. Costs a wire format
   and bandwidth, but the frame is already in hand and correctly stamped.
3. **Move identity into `lock_node`**, which has frames and XFeat today. ⛔ But
   `lock_node` tracks ONE target by design, and re-ID exists for the
   multi-object case — this would be re-architecting around the tool we happen
   to have.

⭐ **(2) is the honest one**, and it is the same shape as a decision this repo
already made: the detector owns the frame, so anything needing pixels and
boxes together belongs where both already are. It is a wire-format change and
needs its rate measured before it ships.

**Status: still DEFERRED, with a corrected and more specific blocker.** The
old note was wrong about what was missing, which is worth more than the fix
would have been — `track_id` on `/detections` (also outstanding) is a
prerequisite for any of the three.

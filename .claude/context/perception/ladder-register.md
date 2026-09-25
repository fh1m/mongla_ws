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

1. **B-60 — the `gate` class is not a gate detector.** Worst mistake **0.92 on
   `bin_front_#1.mp4`**, higher than the 0.89 it gives the real gate. No bar
   separates them. Fix is retraining with hard negatives
   (`tools/mine_hard_negatives.py`, 160 frames mined so far).
2. **Evaluate the OTHER models.** 66 `.pt` on this box, **32 with a gate
   class**, including single-class `duburi_nano_250` / `duburi_medium_100`
   never tested. A better model may already exist — cheaper than retraining.
3. **`rescue` / `repair` / `sauvc_sim` / `bin_fire_blood` unmeasured** for the
   same defect.
4. **HEF vs `.pt` class disagreement** — on `gate.mkv` the `.pt` says
   `gate(0)` at 0.78–0.89 while both HEFs say `repair` 150–262×. Either INT8
   damage or a class-order mismatch. **Unresolved.**

### Wired half-way

5. **`visibility.py`** — `assess()` is wired advisory; **`WorldTarget`,
   `project_world_target`, `region_conf_bar` are reached by TESTS ONLY.** That
   is the object-permanence half: what lets a target leaving frame become a
   navigation problem rather than a forgotten one. ⚠ The reachability register
   cannot see this — it checks modules, not capabilities.
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

# The ladder register — everything built, and whether it actually runs

> **One page, checkable.** Every capability built for the lock ladder across
> the 2026-09-23/25 rounds, its evidence, and its REAL wiring state read from
> `test_no_capability_is_built_and_unreachable.py` rather than from memory.
>
> **Warning:** The repo's oldest and most expensive defect is a module that is finished,
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
| **HELD rung** (`lock_state.Rung.HELD`, `lock_frames._held_from`) | a weak box near a lock seen twice replaces the follower's guess in a gap: **3.7 px vs 4.6 px** (held-out) against the next strong detection | **ON by the owner's decision** (`assoc_conf` 0.10). Misses its safety bar where the gate model mistakes the torpedo board for a gate, +1.5 points (measured-bars section 72) |

Plus, in the nodes themselves: the camera **mailbox** (16.9 ms stale vs 396 ms
queued), the follower's **RANSAC similarity fit** (carries SCALE, which the
approach controller reads), replay mode, and — new — the **acting bar** below.

---

## DEFERRED, with the blocker

| module | what it would give | blocked on |
|---|---|---|
| **`tracking/visibility.py`** | **the FOV guard. There is no field-of-view constraint anywhere in `mongla_control`, so perception can be perfect and the lock still dies because the vehicle turned away** | a consumer in the vision verbs: yaw/lateral demand scaled by `Visibility.scale`, and only the component making the bearing WORSE. Control change ⇒ needs a pool day |
| `continuity.py` | gap distribution analyser | offline only; its constants are a person in air |
| `flow/scale_check.py` | flow-height vs baro-height cross-check | the barometer reports "not initialised" on this hull — the comparison has one side |
| `draw_strip.py` | — | superseded UI, 471 dead lines. Delete or revive deliberately |

---

## What these rounds actually changed on the vehicle

| change | measured on | result |
|---|---|---|
| **acting bar** `act_conf` 0.45 in `lock_node` | 3 Mirpur clips × 1 200 frames | gate-on-empty-water 90.8 % → 46.8 %, real gate held at 98.9 % |
| **pre-dive floor gate** in `bringup_check` | the 4 deployed HEFs | FAIL naming `gate_rescue_repair` 0.200 and `yolov11n` 0.200 |
| **HEF recompiled** at `nms_scores_th 0.05` | `hailortcli parse-hef` | `Score threshold: 0.050`, 3 classes — verified on the artifact |
| **`tools/bench/hailo_compile.sh`** | — | the compile is now a standing capability, not a thing rebuilt and thrown away |
| **`tools/studies/negative_clip_check.py`** | — | closes the hole that hid B-59: no bar may be set from positive frames alone |
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

**Key point: (2) is the largest remaining gap in the whole pipeline** and the only one
whose module is already written and measured.

---

## Work left — the running list

> Kept HERE because it has been lost twice in conversation. Anything picked up
> gets struck from this list in the same commit that lands it.

### Blocking, in order

1. **B-60 RESOLVED by model selection.**
   `pendrive_2/final/models/gate/weights/best.pt` scores **100 % positive and
   0.0 % across 1 200 negative frames from 3 venues**, at every threshold.
   The shipped model fires on 100 % of a gate-free clip at 0.15. Remaining:
   its top box on `gate.mkv` is still slightly off structure; it carries
   `gate, shark, shaw_fish` and NOT `rescue`/`repair`, so promoting it changes
   the mission surface; and it must be compiled at >= 1024 calibration frames
   before it flies.
2. **The archive was searched.** `tools/bench/model_inventory.py`: **208 `.pt`,
   96 unique after content dedup, 29 with a gate class.** 96/96 have never
   been measured against absent-prop footage — that column is now tracked.
3. **`rescue` / `repair` / `sauvc_sim` / `bin_fire_blood` still unmeasured**
   for the same defect.
4. **B-61 — ROOT CAUSE FOUND AND FIXED: BGR/RGB.** The Hailo input path
   never converted `cv2`'s BGR to the RGB the model was trained on, so the
   chip ran on colour-swapped pixels since the backend was written. Proved by
   reproducing it on the `.pt` with no chip involved: the same swap turns
   `gate=0.78` into `repair=0.92 @0,0`. **Two wrong diagnoses came first**
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

5. **`visibility.py`** — `assess()` is wired advisory and `region_conf_bar`
   is wired in `lock_frames.py` with the lock's own centre as the prior. The
   world-frame memory (`WorldTarget`, `project_world_target`) was written by
   the downward lock and never read; it was deleted on 2026-11-18, with the
   lock node's `/mongla/state` subscription and its never-subscribed IMU roll
   handler, which existed only to feed it.
6. **FOV demand-scaling** — measured and advisory only; scaling waits on water.

### Deferred, unchanged

7.
   `continuity.py` ·
   `flow/scale_check.py` · `draw_strip.py`
   (delete or revive)
8. **Never started:** Round 11 (homography → yaw/altitude/obliquity) ·
   Round 14 / POS_HOLD without DVL · downward place-bank preloading ·
   Hailo async API (13.4 → ~7 ms) · `geometric_allocation.py` triage

   **`track_id` is NOT missing** — checked 2026-09-25.
   `tracker_node.py:432` sets `d2.id = str(td.track_id)` on `/tracks`, and
   `check_tracker.py` reads `d.id` (the field `vision_msgs/Detection2D`
   actually has; there is no `tracking_id`). The backlog entry claiming "its
   absence makes every id-switch number meaningless" was stale. What is
   genuinely absent is an id on **`/detections`** — the DETECTOR's output has
   no track identity, because association happens downstream in
   `tracker_node`. That is by design, and it is why re-ID cannot be fed from
   the detections topic as it stands.
9. **`measured-bars.md` §1 must be re-derived** once a model passes
   `negative_clip_check.py` — every bar there came from positive frames alone.

---

## Re-ID and the association cascade: deleted, nothing needed them

`tracking/reid.py` and `tracking/cascade.py` were built to cut identity
switches and deferred because `tracker_node` has no images. Measured against
how the vehicle actually selects a target (measured-bars section 88), the
shipped tracker switches identity about once every 30 s on the bins and never
on the gate, and no consumer pays for it: align re-locks on the next real
detection whatever its id, and the coast follows the locked id only between
real detections. Both modules were deleted on 2026-11-18. A consumer that needs
an identity to survive a long gap is the reason to rebuild them; they are in
the git history.

## The ROS-graph sweep — `/target_pose_fused` is produced and nobody reads it

`tools/ci/topic_wiring_sweep.py` (new, 2026-09-25) looks for the half of the §9
defect that `orphan_sweep.py` cannot see: a topic published with no in-graph
reader, or subscribed with no in-graph producer. That is the shape of the yaw
gap — no import was missing, no test failed, and the capability did not exist.

Five rows; four resolve cleanly and **one is real**:

| topic | verdict |
|---|---|
| `/mongla/esc_rpm` | diagnostic, CLI and Foxglove read it |
| `/mongla/localization/aiding` | diagnostic |
| `/target_pose` | false alarm — `pnp_node` builds the name in a VARIABLE, invisible to a source scan |
| **`/target_pose_fused`** | **Warning: published by `pose_fuse_node`, read by nothing** |

**Key point: What is being wasted.** `pose_fuse_node` subscribes every estimator's
single-frame answer and publishes the one the support agrees on — a fused,
outlier-rejected pose, with `pose_cluster`'s decision rule behind it. Every
consumer today reads the RAW `/target_pose` instead: one frame's answer from
whichever estimator happened to produce it.

**Caution: This was a deliberate split**, and the node's docstring defends it: a
consumer wanting the latest reading and one wanting the settled reading are
different consumers, and collapsing them would remove the ability to compare
an estimator against its own fused output. So the topic existing unread is
NOT automatically wrong.

**What is wrong is that no consumer chose.** The fused topic was built for the
control path — the settled answer is what you steer on — and the control path
never switched. Do not flip it blind: the fusion adds latency by
construction (it waits for support), and `vision_verbs` acts on a freshness
decay. The measurement to make first is **fused-vs-raw latency against
accuracy on a recorded bag**, which is exactly what publishing both was meant
to enable.

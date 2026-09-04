# Never losing the lock: what was measured, and what it changed

> Every number here came from real RoboSub 2025 competition footage (20 GB,
> 33 clips) run through the models trained on that same footage (25 archived
> training runs), plus live verification on the Pi 5 + AI HAT+. Where a claim
> is reasoning rather than measurement it says so.

## 0. The gap that made this round necessary

Six constants protect a target lock — the freshness ramp, `coast_s` (0.8),
`lost_grace_s` (1.0), `_STALE_LIMIT_S` (1.0), the Kalman `max_predict_s` (1.5),
the tracker's `track_buffer` (5.0 s). All are sized from detection **rate** and
from each other's ordering. **None was sized from how long a real gap lasts,
because nothing had ever measured one.** `check_tracker.py` came closest and
collapsed it to a scalar: a 5 % "predicted ratio" is one 400 ms blackout or a
hundred single-frame flickers, reported identically.

`duburi_vision/continuity.py` is the missing ruler. Pure functions over
observations — no ROS, no camera, no model — so a live topic, a bag and an
offline replay are all scored the same way.

## 1. THE HEADLINE: the tracker emitted nothing on real water

`high_conf_det_threshold` gates **track creation**. Below it no track is ever
started, so `/tracks` is empty, so `vision.coast_s`, the Kalman smoother and
the continuity lock all have nothing to work with. The shipped value was 0.6,
inherited from pedestrian benchmarks where scores run high.

Real underwater scores, 319 detections across the gate approach:

```
  p10 0.167   p50 0.258   p90 0.439   max 0.640
  fraction at or above 0.6:  0.6 %
```

| `high_conf_det_threshold` | tracker presence |
|---|---|
| 0.60 — **shipped** | **0.0 %** |
| 0.40 | 13.9 % |
| 0.25 | 17.0 % |
| 0.15 | 45.1 % |

End to end, same cached detections, tracker the only variable:

| arm | presence | gaps > `lost_grace_s` |
|---|---|---|
| detector only | 15.5 % | **6** |
| tracker as shipped | 0.0 % | — |
| tracker clamped to the detector floor | **45.1 %** | **1** |

**Six mission-aborting losses become one.** Nothing ever raised anything,
because a tracker with no tracks still publishes an empty array and every node
looks healthy — the same shape as the truncated-coast bug in
`test_tracker_rate.py`.

The fix is structural: the gates are **clamped to the detector's live `conf`**,
wired through all three launch files. A detection the detector chose to publish
must be allowed to start a track. It is a ceiling, never an assignment.

Live confirmation: `/tracks` now publishes at **51.95 Hz**. It published
nothing before.

## 2. Three defects in the code whose job is to survive a gap

**A real detection could not revive an expired Kalman filter.**
`is_expired()` reads `predict_streak`; `predict_streak` resets **only** inside
`smooth()`; the old `continue` skipped `smooth()`. `prune()` only drops filters
whose id is *absent*. So once a filter expired while the backend still held the
id, a returning target was suppressed **for ever** — it could never reach the
one call that would clear the condition suppressing it. At shipped config the
Kalman expires at 30 frames and the backend holds an id for 100: a **70-frame
dead zone**. The gate approach has 53 gaps in 57 s, four past 1.5 s.

The suite could not see it because the one test that could
(`test_roboflow_tracker.py:107`) **reimplements** the node's loop — it tested a
copy, and the copy did not have the bug. The loop is `kalman_pass()` now,
imported by both.

**`_last_real` was never evicted.** Unbounded growth, and a recycled tracker id
inherited the previous object's sighting timestamp — a coast could start from a
sighting belonging to something else.

**`close()` was silently partial.** One `try` around five teardowns, and the
fourth referenced an attribute that no longer exists.

## 3. The model is not the problem — the FOOTAGE is

All 25 archived training runs report **mAP50 = 0.995**. Three models on video
of the competition they were trained for:

| model | video presence |
|---|---|
| bin | 100.0 % |
| octagon | 92.4 % |
| gate | **1.5 %** |

My hypothesis — "stills-trained models fail on video" — is **disproved by the
octagon**: 96.7 % recall on labelled stills against 92.4 % on its own video, a
4-point gap. The difference is measurable in the frames:

```
  gate_back.mkv   Laplacian variance  321   brightness 170.8   contrast 27.7
  bin.mkv         Laplacian variance 1180   brightness 170.3   contrast 35.4
```

**3.7× blurrier at identical brightness.** Motion blur plus washed-out
underwater contrast — the regime an AUV spends its run in, and the one a
dataset of still frames does not contain. Confirmed at the source: not one of
the 25 training configs uses blur, rotation or perspective augmentation
(`degrees: 0.0`, `perspective: 0.0`, HSV and mosaic only). The models were
never shown a blurred frame.

## 4. CLAHE: 5× the detections, and what it costs

600 frames of the gate approach at conf 0.15:

| preprocessing | presence | mean score |
|---|---|---|
| none | 10.7 % | 0.226 |
| unsharp mask | 38.0 % | 0.329 |
| CLAHE (LAB, clip 2) | 56.3 % | 0.401 |
| **CLAHE (YUV, clip 3)** | **56.2 %** | **0.410** |

And **no cost where it is not needed**: bin 100 → 100 %, octagon 100 → 100 %
with a *higher* mean score (0.570 → 0.634). YUV over LAB for the same result at
half the price — 3.78 ms vs 7.07 on the Pi.

The honest cost, measured on the vehicle with one variable:

| | detections | age med | CPU |
|---|---|---|---|
| CLAHE off | **79.1 Hz** | 21.13 ms | 48.5 % |
| CLAHE on | 50.3 Hz | 22.00 ms | 60.5 % |

−36 % of rate for 5× the detections on hard footage. **Off by default**;
`preprocess:=clahe` enables it. That trade is the operator's to make, and it
depends on water clarity on the day.

## 5. Method notes that cost something to learn

**Run inference once, then compare.** `tools/tracker_ab.py` caches detections
so every tracker sees byte-identical input. A rosbag cannot do this:
`pool_record.sh` excludes `image_raw`, and `ros2 bag play` is wall-clock paced,
so the pipeline drops a different frame subset each run.

**Compare what ships, not the library.** The first A/B called the bare
`OCSORTTracker`. Our `RoboflowTracker.update()` adds the coasted rows — that
*is* the coast layer. Measuring the library would have reported that tracking
buys nothing.

**A gap is measured from the LAST SIGHTING**, not the first missing frame,
because that is what the control loop compares against (`coast_s` is
`monotonic() - _last_real[id]`). Measuring from the first miss under-reports
every gap by a frame period, in the direction that flatters us.

**The unit tests could not catch a launch-type bug.** ROS 2 launch coerces the
literal `'off'` to boolean `False` before `declare_parameter` sees it →
`InvalidParameterTypeException` → the whole composed process dead at startup.
Ten tests passed; hardware found it in one run. The launch default is `'none'`
now.

## 6. Still open

- **Ego-motion compensation.** [EMAP](https://arxiv.org/abs/2404.03110) cuts ID
  switches **73 %** on OC-SORT — our tracker — by decoupling camera motion from
  object trajectories. We publish body rates at 50 Hz (`/duburi/imu_rates`) and
  `rotation_flow_px()` already computes the image shift. Not yet connected.
  **Recorded so it is not re-derived: an IMM filter bank is the wrong tool** —
  our targets are static props and all apparent motion is ours.
- **Low-confidence second association.** `hailo.py:569` discards the 0.05–0.15
  boxes the chip already computed. ByteTrack's finding is that those are the
  occluded and motion-blurred ones, worth +1–10 IDF1 across nine trackers.
- **Camera exposure.** Both cameras are on auto with
  `exposure_dynamic_framerate=1`, so in dark water the driver buys brightness
  with blur *and* a lower frame rate. Blur ≈ angular rate × exposure.
- **Retraining with blur augmentation.** Out of scope this round by decision.
  §3 says the models are good and the footage is hard, so this is the
  source-level fix and the archived runs give a clean baseline to beat.
- **The remaining 25 s gap** on the gate approach is genuine absence — the
  camera is not pointed at the prop. No threshold recovers that; it is mission
  geometry.

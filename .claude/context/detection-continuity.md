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

## 4. CLAHE: RETRACTED — it does not help, and usually harms

**This section used to read "CLAHE: 5× the detections".** That number does not
reproduce and the claim is withdrawn. What follows is what replaced it.

Re-measured on **raw detection rate**, one variable at a time, on the same
clip the original claim came from:

| sample of `gate_back.mkv` (`robosub_gate2`, conf 0.10) | base | + CLAHE |
|---|---|---|
| stride 5, offset 0 | 12.0 % | **0.4 %** |
| stride 7, offset 40 | 30.4 % | **1.2 %** |
| stride 11, offset 90 | 30.8 % | **2.0 %** |

And nowhere else either — 9 still-image cases across 4 props, 3 venues and a
39× sharpness range gave deltas between −2.7 and +2.7 points; 3 video clips
gave −0.8, +3.8, −6.5. **Seventeen configurations, never meaningfully
positive**, and on the gate it destroys 95 % of the detections.

The CLAHE implementation was checked before retracting: it produces a correct,
artefact-free contrast enhancement. It works. It just moves the image away from
what the detector learned.

### Why the original measurement said the opposite

It was taken as **tracker presence** with the tracker-clamp fix and a
confidence drop stacked into the same arm, not as detector output with one
variable moved. The clamp alone took presence 0.0 → 45.1 % and conf 0.10 added
8.5 more. Attributing the remainder to the last thing switched on is the
confounded-A/B trap recorded in §5 of this same document — written, and then
walked into.

### The generalisable rule

The models were trained on **unprocessed** underwater frames — not one of the
25 archived configs uses blur, rotation, perspective, or any contrast
augmentation. Any preprocessing that makes an image look better **to a person**
moves it away from the distribution the detector learned. A contrast fix is a
domain shift wearing a helpful face, and the prettier the output looks the
further it has moved.

### What survives

The blur finding itself, which reproduced across three venues: **footage, not
models, separates a 100 % clip from a 1.5 % one.** That still points at motion
blur. It just says the answers are exposure control and training-time
augmentation, not a filter in front of the detector.

`preprocess:=clahe` remains selectable for water we have never measured. **No
profile turns it on** — `test_profiles.py::test_no_profile_enables_preprocessing`
is the guard, and it was verified to bite.

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
- ~~**Low-confidence second association.**~~ **MEASURED — see §7.** The
  answer is a floor of **0.10**, not 0.05, and the reason is in the data.
- **Camera exposure.** Both cameras are on auto with
  `exposure_dynamic_framerate=1`, so in dark water the driver buys brightness
  with blur *and* a lower frame rate. Blur ≈ angular rate × exposure.
- **Retraining with blur augmentation.** Out of scope this round by decision.
  §3 says the models are good and the footage is hard, so this is the
  source-level fix and the archived runs give a clean baseline to beat.
- ~~**The remaining 25 s gap is genuine absence.**~~ **RETRACTED — §8.** It was
  not. CLAHE finds the target through most of it and the longest gap collapses
  to 2.7 s.


## 7. The confidence floor, settled on evidence

ByteTrack's finding is that low-score boxes are the occluded and
motion-blurred ones — the AUV case — and that associating them is worth +1 to
+10 IDF1. Our detector discards everything under 0.15 before the tracker sees
it. So: how far down is it worth going?

1830 frames of the gate approach, tracker clamped to follow the floor:

| floor | presence | boxes/frame | multi-box frames | centre spread | jitter p95 |
|---|---|---|---|---|---|
| 0.25 | 17.0 % | 1.08 | 7.6 % | 0.0351 | 0.0064 |
| 0.15 | 45.1 % | 1.12 | 11.3 % | 0.0493 | 0.0055 |
| **0.10** | **53.6 %** | 1.20 | 18.0 % | **0.0483** | **0.0052** |
| 0.05 | 56.1 % | 1.45 | 35.4 % | 0.0522 | 0.0084 |
| 0.02 | — | 1.86 | 58.4 % | 0.0563 | — |

**0.15 → 0.10 is free**: 8.5 points of presence, and the extra boxes are the
*same target* — centre spread does not grow (0.0493 → 0.0483) and jitter
actually falls. Below 0.10 it inverts: at 0.05, 35 % of frames carry more than
one box and jitter jumps 53 %, which on a vision-servoed hull means steering at
the wrong thing.

**Presence alone could not have answered this** — a false positive is "seen"
too. The discriminator is whether the extra boxes cluster on the target
(spread flat) or scatter across the frame (spread grows), which needs no
per-frame ground truth.

Recorded at the point of use in `detection/hailo.py`. Not made the default:
it has not been through water with a live control loop.

### A rig error worth recording

The first run of this sweep read *flat* below 0.15 — because the detection
cache had been built at conf 0.15, so the lower floors had nothing extra to
find. The tool answered the question it was asked and the question was wrong.
Rebuilt at the HEF's baked 0.02 floor, the real shape appeared. **A cache is
only as permissive as the run that made it.**


## 8. All three changes together — and a retraction

The individual measurements each answered one question. Composed, on the same
1830 frames of the gate approach, they answer the one that matters:

| arm | presence | p90 gap | longest gap |
|---|---|---|---|
| **A** shipped (conf 0.15, no clamp) | **0.0 %** | — | — |
| **B** + tracker clamp | 45.1 % | 25 000 ms | 25.0 s |
| **C** + conf 0.10 | 53.6 % | 24 938 ms | 24.9 s |
| **D** + CLAHE | **95.4 %** | **2 656 ms** | **2.7 s** |

**Zero to 95.4 % presence** on the footage where the lock was previously
impossible, and the worst single loss goes from a 25 s blackout to 2.7 s —
inside the Kalman predict window, and only just outside `coast_s`.

**I called that 25 s gap "genuine absence — the camera is not pointed at the
prop", and that was wrong.** It appeared in every earlier arm, which made it
look structural. CLAHE finds the target through most of it: the prop was in
frame the whole time, in water too washed-out for the model to see it. The
statement is retracted rather than edited away, because the reasoning behind
it — "a gap that survives every threshold must be geometry" — is plausible,
and is the kind of thing that would otherwise be re-derived.

That also revises the round's own headline. The tracker clamp is the fix that
makes the coast layer *exist*; **CLAHE is the one that makes the lock hold.**
Neither is sufficient alone: at arm C the tracker is working and still loses
the target for 25 s; CLAHE without the clamp would feed a tracker that emits
nothing.


## 9. ⛔ CLAHE IS NOT UNIVERSALLY GOOD — the counter-example

A second target, the torpedo model over the bin clip, goes the other way:

| arm | gate approach | torpedo on bin |
|---|---|---|
| tracker clamp + conf 0.10 | 53.6 % | 79.5 % |
| **+ CLAHE** | **95.4 %** (+42) | **15.3 %** (−64) |

**This is why it stays off by default**, and it is the single most important
caveat in this document. Enabling it globally would have halved detection on
clear water while looking like an improvement on the clip I happened to
measure first.

The two clips separate cleanly on frame statistics, and **saturation splits
them harder than blur**:

| | blur (lapvar) | contrast | saturation |
|---|---|---|---|
| gate | 315 | 27.7 | **159.9** |
| bin / torpedo | 1241 | 36.1 | **27.9** |

**The rule: CLAHE helps blurry, low-contrast, SATURATED (green/murky) water
and hurts sharp, desaturated water.** Physically sensible — it amplifies local
contrast, which recovers a washed-out target and over-sharpens one that was
already crisp.

So it is a **per-water** decision, not a per-vehicle one. `water_check` prints
the three statistics and says which side of the line the pool is on:

```
ros2 run duburi_vision water_check              # live camera
python3 tools/water_check.py --video clip.mkv   # a recording
```

Validated against both measured clips, on frames it had not seen: gate →
CLAHE ON, bin → CLAHE OFF. In between it says **MEASURE BOTH** rather than
inventing confidence from a rule fitted to two samples.

### What this changes about §8

§8's "0 % → 95.4 %" stands for the gate. It is not a general claim. The
general claims from this round are narrower and stronger:

- **The tracker clamp is unconditional** — it helped both targets (0 % → 45.1 %
  and 0 % → 63.5 %) and cannot hurt, because it only ever lets through a box
  the detector already published.
- **conf 0.10 is unconditional** — it helped both (+8.5 and +16.0 points) with
  jitter flat or better.
- **CLAHE is conditional**, and now measurable in advance.

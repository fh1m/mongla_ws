# First pool day — what to verify, in order

> Everything below was built or measured **on a bench, on archive footage, or
> on a person in a room**. Nothing in this file has been in water. The order is
> chosen so that a failure early invalidates the tests after it — do not
> reorder to get to the interesting part.

**Record every run.** `scripts/pool_record.sh record --full <label>` and
`tools/record_session.sh`. A number nobody can replay is a number nobody can
check, and three results were retracted this week for exactly that.

⛔ **`/tmp` is a RAM disk on the dev box.** Bags land in `MONGLA_RUN_DIR`
(`~/mongla_runs`), never `/tmp`.

---

## Gate 0 — before the vehicle enters the water

These are not tests, they are permissions. A NO here ends the day.

| # | check | command | pass |
|---|---|---|---|
| 0.1 | board talks, firmware rev at or above the floor | `ros2 run mongla_manager connect --watch` | `behaviour rev` ≥ `FW_BEHAVIOUR_REV_REQUIRED` |
| 0.2 | **barometer healthy** | same screen | **NOT** "not initialised" |
| 0.3 | leak dry, kill clear | same screen | `leak dry`, `kill switch clear` |
| 0.4 | subsystem grade | `ros2 run mongla_manager bringup_check --srot` | exits 0 |
| 0.5 | propellers clear, human on the kill switch | — | stated aloud |

⛔ **0.2 gates most of this document.** An unhealthy barometer refuses
`DEPTH_HOLD`/`AUTO`/`PATTERN`, and since `SROT_MOVE` enters `AUTO` **every move
verb is denied** — the vehicle arms and does not move. It also means no
altitude, so **loop closure cannot fire at all** (§39). On the bench on
2026-09-24 it read *not initialised*.

---

## 1 — The depth loop, closed, for the first time (G1)

**Why first:** every move verb on srot enters `AUTO`, which closes the depth
loop. Until this passes, §2 onward cannot run at all.

Follow the two armed bench checks in
[`srot-integration.md`](srot-integration.md). ⛔ An in-air move is **not**
partial validation — at ~0 m, target and measurement agree.

**Pass:** commanded depth held within a stated band, no runaway, `DEPTH_OUT`
present. **Record:** depth, demand and mode for the whole run.

---

## 2 — Does the detector see competition props in this water?

Everything above the detector assumes it fires. Our recall has swung
**29.2 / 72.7 / 68.3 %** across venues, so this is measured per venue, not
assumed.

- run `vision_pi.launch.py` with `paused:=false` ⛔ (the launch default is
  `true`, which consumes frames and publishes nothing — the defect that cost a
  day; it now warns)
- point at each prop at 1 m, 2 m, 3 m, 5 m
- **record:** `/detections` and `image_raw` for every approach

**Pass:** the props the mission needs are detected at the ranges the mission
needs. **Report:** per-prop, per-range detection rate and confidence.

---

## 3 — The gap distribution, in water (closes §50)

⭐ **The measurement the whole lock ladder is sized on, and it has only ever
been taken on a person in a room.**

Same runs as §2 — no extra work, just the analysis:

```
tools/continuity_from_bag.py <bag> --label pool_<prop>
```

**Compare against the air recording:** 338 gaps, p50 116 ms, p90 1.12 s,
p99 3.04 s, max 7.64 s.

**Then, and only then**, the A/B in `config/ladder_candidate.yaml`: run the
same approach twice, once on `shipped` and once on `candidate`, and score
`lost` occupancy from `/lock`. **Pass for the candidate:** lower `lost` with no
increase in false holds. ⛔ If it is not better in water, the shipped values
stay — the candidate exists to be refuted.

---

## 4 — The lock ladder on real props

Air numbers to beat, from the live run (§36): `detection 48.1 % · follow
15.5 % · anchor 11.2 % · lost 25.2 %`, anchor firing in 42 % of windows.

**Pass:** the anchor rung carries real gaps on a prop, i.e. `anchor > 0` in a
meaningful fraction of windows and `lost` below the air figure.
⚠ Do not expect the air numbers — a prop that leaves the frame is a harder
target than a person who walks back in.

---

## 5 — The checkpoint bank on props that repeat

- enrol on approach, then leave and return
- **Pass:** the bank re-identifies the same prop; `identity_ok` fires at the
  40-inlier bar (§25) and does **not** fire on a different prop.
- ⛔ **The known failure to hunt:** a preloaded bank said "torpedo" while
  looking at the gate (§22). Point at the *wrong* prop deliberately and confirm
  it refuses.

---

## 6 — Loop closure: the first fix that ever reaches the filter

⛔ **Never fired. Not once, anywhere.** Needs the downward camera, a working
baro (0.2) and `floor_height` from the tile grating.

```
~/.mongla/loop_closure.yaml     enabled: true
```

- swim a loop that genuinely revisits a spot, 30 s+ apart and 1.5 m+ travelled
- **Pass:** a closure fires **at the revisit and nowhere else**, with sigma
  composed from the stored checkpoint's own uncertainty.
- ⛔ **Watch for the opposite failure:** `/mongla/odom` jumping to a place the
  vehicle never was. A wrong fix is **believed** — the filter shrinks its
  covariance around it (§39) — so a single false closure fails this test.
- **Record:** `/mongla/localization/fix`, `/mongla/odom`, and the lock log's
  `loop closure N:` lines.

⚠ Bar derived on archive: **zero** false closures at 100 inliers across six
cross-venue pairs (§41). If water produces any false closure at 100, raise the
bar rather than keeping it.

---

## 7 — The flow scale cross-check (closes §44, unwires an orphan)

The two heights — tile grating vs `pool_depth_m − |depth|` — have **never been
compared**, because the barometer has never worked at the same time as the
grating.

- measure the real pool depth with a tape, set `pool_depth_m`
- **Pass:** the two agree within 20 %. ⛔ If they disagree, the module reports
  it and **does not pick a winner** — `pool_depth_m` is the one nobody
  measures. Resolve it with the tape, not with code.

---

## 8 — Fouled lens vs changed world (§ health)

**Declared thresholds, never tested against a real fouled port.** The one
deliberate-damage test in this document, and the cheapest.

- get a clean lock, then **smear the port** (silt, a thumbprint, a bubble)
- **Pass:** the log says `the CAMERA looks degraded, not the scene`
- then **occlude the target instead**, port clean
- **Pass:** it says `the VIEW changed, the camera is fine`

⛔ If both read the same, the separator is not a separator and its thresholds
(`COLLAPSE_FRACTION`, `FOULED_FRACTION`) need measuring rather than declaring.

---

## 9 — Things that only exist as numbers until someone swims them

| item | what water decides |
|---|---|
| **XFeat INT8 on the chip** (§38) | 19/20 columns cleared the bar on archive. Re-run the murky table on *this* pool's footage |
| **640 vs 320 pose precision** (§42) | 2.74× more precise on a synthetic warp. Does it survive real range and turbidity? |
| **Downward camera as a DVL** | verified to 1.09 cm over 30 cm **in air**. No in-water velocity number exists at all |
| **Approach band** (§46) | confidence peaks mid-range on archive. Does the peak sit at the same apparent size in this water? |
| **Fine-tuned XFeat** | only if the checkpoint sweep says it wins on the held-out venue |

---

## What to bring back

1. **Every bag.** `~/mongla_runs/`, copied off the vehicle before it is powered
   down.
2. **The `continuity_from_bag.py` output** for each prop.
3. **A yes/no per section above** — not a narrative.
4. ⭐ **Any number that contradicts this file.** Three results were retracted
   this week; the ones that hurt were the ones nobody went looking to refute.

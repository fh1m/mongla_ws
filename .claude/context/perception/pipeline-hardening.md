# The vision→control pipeline: what is verified, and what could still bite

> Every number here was measured on the vehicle — Pi 5 + AI HAT+ (Hailo-8),
> two cameras, the srot board on `/dev/ttyUSB0` — at `8e92d59`. Where a claim
> is reasoning rather than measurement it says so. Nothing here is inherited
> from a datasheet or a comment.

## 0. The state at the last sanity check

Whole stack live: two cameras and two detectors in ONE process, the manager
talking to the real srot board. **0 errors in either log.**

```
detections            77.9 Hz    age  med 21.72   p95 27.87   max 31.58 ms
image_raw (viewers)    9.5 Hz    age  med 11.43   p95 16.38   max 18.09 ms
detection interval               med 13.38   p95 18.08   max 23.13 ms
chip efficiency       783 inferences / 10 s, frame age at infer-start
                      11.5 ms mean / 18.4 max, 0 decoded-but-never-inferred
CPU                   detector ~51 %, manager ~12 %, of 400 %
srot                  13.55 V, depth +1.18 m, leak dry, kill clear
tests                 1237 passed, 7 skipped, 0 failed (on the Pi)
```

Where the 18.0 ms of pipeline goes, budget closing to 0.02 ms:

```
capture -> pump store    4.04 ms    USB + kernel; the hardware floor
decode (MJPEG)           1.90 ms    cv2.imdecode, releases the GIL
inference                9.85 ms    chip 8.34 + 1.51 ms host
                       ------
                        18.01 ms    at 77 Hz
```

Chip ceiling, measured: **97.92 FPS / 8.34 ms HW latency**. We run at 79 % of
it. The remaining 1.51 ms of host work is letterbox + buffer copy + NMS decode.

## 1. The design rules that came out of this, in priority order

**Any C extension called at rate must be checked for GIL behaviour before
anything else in the process is blamed.** The blocking Hailo call held it for
its whole 10 ms and the symptom was a *camera* that looked 13.8 ms stale. Two
hypotheses about the camera died first. `tools/gil_audit.py` audits the whole
set; run it after any dependency change.

**A fixed-rate consumer polling an asynchronous producer wastes half a period.**
This appeared at three layers of one chain — the camera's publish timer
(8.4 ms), the decode behind the rate gate, and the control loop's
`time.sleep(1/50)` against 77 Hz detections (~6.5 ms). Reaction time is the
SUM, so each was worth removing separately. Anywhere a `sleep` faces a queue,
look again.

**Derive from the sensor, cap by physics.** Freshness thresholds are computed
from the observed pipeline latency and detection interval — 50 ms full / 200 ms
zero on the forward camera, 100 / 317 on the downward, with nothing configured
per-camera. But derivation alone is unsafe: a pipeline stuck at 300 ms argues
itself into MORE trust the worse it gets. Full authority is capped at 100 ms in
absolute time (~6.5 cm of travel at cruise) and the observed pipeline is
trusted only up to that cap.

**A measurement without a control is not a measurement.** The first GIL probe
reported a median of 0.000 ms for a *known* GIL holder — a spinning sampler
competes for the lock it measures. Both controls print first and the run is
void if they do not separate.

**Report percentiles, not maxima.** The live check first read a 154.64 ms worst
pipeline age — one outlier in 2231 samples. p99 is 29.58. A threshold chosen to
satisfy a max is chosen by noise.

## 2. Bottleneck permutations — status and what would catch each

| # | scenario | status | evidence / what would catch it |
|---|---|---|---|
| 1 | GIL held by a hot C call | **closed** | `gil_audit.py`: everything hot frees it; `cv_bridge` holds but is 0.1 % duty on the viewer feed |
| 2 | chip fed a stale frame | **closed** | `0 decoded-but-never-inferred`, logged every 10 s; WARNs if not |
| 3 | Hailo SRAM exhaustion | **closed** | configure-once; `test_swapping_does_not_RECONFIGURE_the_graph` asserts ≤2 configures over 24 swaps |
| 4 | both detectors unpaused | **mitigated** | 4.15 ms swap each way, ~74 Hz across both; the backend WARNs after 20 swaps. Missions pause the unused camera |
| 5 | slow / degraded camera | **closed** | thresholds derive; verified live on a 27.6 Hz and a 73.4 Hz camera, both sensor-limited |
| 6 | pipeline stall (300 ms) | **closed** | full authority capped absolutely; `test_a_slow_pipeline_does_NOT_earn_full_authority` |
| 7 | QoS mismatch on any link | **closed** | one shared table + `test_qos_contract.py` computes pairings and scans for hand-rolls |
| 8 | a camera unplugged mid-mission | **OPEN** | the capture loop now logs loudly instead of dying silently, but the *detector* just stops receiving. See §3 |
| 9 | chip stall / `job.wait` timeout | **OPEN** | 1 s budget, then an exception into `_infer_loop`'s handler. Never exercised. See §3 |
| 10 | Pi thermal throttling | **OPEN** | never soaked longer than ~90 s. See §3 |
| 11 | memory growth over a mission | **OPEN** | never measured beyond a few minutes. See §3 |
| 12 | srot link saturation | **closed by headroom** | 18.8 % of 115200 at 79 Hz; baud is fixed in firmware |
| 13 | PCIe transfer limit | **closed** | already Gen 3; x1 is the Pi's physical connector, not a setting |

## 3. What is still open, and the cheapest thing that would close it

**A camera disappearing mid-mission (#8).** The composed detector is fed by
reference; if the camera thread dies, nothing arrives and nothing says so. The
`direct_feed` fallback only fires at startup. *Cheapest fix:* extend the
existing 10 s efficiency timer — zero inferences with the detector unpaused is
already an anomaly worth a WARN, and the counter exists.

**A chip stall (#9).** `wait_for_async_ready` / `job.wait` carry a 1 s budget,
then raise into `_infer_loop`'s `except`, which logs and continues. Never
exercised. *Cheapest fix:* the fault-injection pattern from the sim rounds —
force a timeout and confirm the node survives and says so, rather than
discovering it in water.

**Thermal and duration (#10, #11).** Longest continuous run this session was
~90 s. A competition run is minutes. *Cheapest fix:* a 20-minute soak with the
whole stack, recording detection rate, `vcgencmd measure_temp`, RSS, and the
efficiency counters. Nothing else needs building — every instrument exists.

## 4. Explicitly rejected, with the reason

**Pipelining the decode.** Now that the GIL is free the camera *could* decode
the next frame during inference, taking the period from 11.8 ms toward 9.85
(~101 Hz). It would make detections OLDER — the decoded frame waits a whole
inference before its own inference starts, ~9.85 ms of age to buy ~20 % of
rate. **Control pays latency, not throughput.**

**Raising the image topic above 10 Hz.** It is a viewer feed now; the control
path does not read it. At 40 Hz it cost the composed process measurable
contention (detection age 33.50 → 31.47 ms when dropped to 10).

**A second VDevice, or the HailoRT scheduler.** Measured: `HAILO_OUT_OF_PHYSICAL_DEVICES`
and a SIGSEGV respectively. One device, two graphs, taking turns.

## 5. What only water can answer

- `dt` is measured per pass now, which slightly tightens the continuity-lock
  gate. Correct, but it changes terminal-alignment feel.
- The freshness thresholds are ~2× tighter on the forward camera than the
  fixed values they replace. Safer, and untested against a real dropout.
- Every number above is with the hull dry and stationary. Motion blur, turbid
  water and a moving target change detection rate, and the thresholds derive
  from detection rate.

# Mongla vision systems: a whole-pipeline Hailo-8 + Raspberry Pi 5 research audit

**Prepared:** 2026-11-23  
**Evidence status:** Literature / repository audit and experimental plan, **not new hardware measurements**. No new commands were executed on Kutum during this report. Existing experimental numbers are attributed to the older recorded runs; all new ideas are *UNTESTED*.  
**Purpose:** To extend the current vision stack *without throwing away measurements, reintroducing disproved enhancements, or confusing FPS with autonomy performance.*

## Executive decision

The next major opportunity is **mission-quality perception per watt and per millisecond**, not a bigger detector. Optimize the information the controller receives: target identity/presence, angle and metric geometry, trustworthy motion estimates, frame usability, observation freshness, and uncertainty. A model that produces a useful observation at the right moment is more valuable than a faster model whose output changes no decision.

The highest-value untested capabilities are: (1) a **floor-trackability/flow-health map** that predicts where visual odometry becomes unsafe; (2) compact **geometric landmark or bearing heads** that eliminate ambiguity in box-based mission alignment; (3) **small, context-aware vision models** compiled to one Hailo context; and (4) **perception scheduling based on expected information gain**. Test these against current implementations, not idealized baselines.

The most useful reusable artifact is a **capture-to-HEF-to-mission evaluation harness**: all videos, image transforms, numerical errors, neural outputs, observation timestamps, and controller decisions share a reproducible timeline.

## A. Existing facts that must be respected

Before new code, read the actual sources:

- [Vision architecture](../../.claude/context/perception/vision-architecture.md).
- [Full vision code audit](../../.claude/context/packages/mongla_vision/audit.md).
- [Model production, source holdouts and retractions](../../.claude/context/perception/model-pipeline.md).
- [Hailo end-to-end measurements](../../.claude/context/perception/hailo-vision.md).
- [Hailo performance frontier](../../.claude/context/perception/hailo-frontier.md).
- [Underwater vision measured across real footage](../../.claude/context/perception/underwater-vision.md).
- [Prior SOTA vision review](../../.claude/context/sota/vision.md).

**Measured on earlier Kutum hardware runs, not during this audit:**

| Result | Interpretation |
| --- | --- |
| YOLO11n custom HEF ~95–98 FPS hardware-only, 3 contexts | Close to the compiled graph's ceiling, not a general ceiling for Hailo-8 |
| Model Zoo YOLOv8n ~415 FPS hardware-only, one context | Large architecture+allocation difference; not a matched-accuracy comparison |
| Blocking inference -> async reduced measured capture-to-detection 28.06 -> 18.01 ms | Fixes GIL interference; not increased three-context hardware throughput |
| 2026-11-20 real composed graph, both detectors active: forward ~25.5 Hz; downward ~25.8 Hz | Two graphs compete for one chip. Camera rates and detector rates are not interchangeable |
| Newer live camera modes | Fantech interim forward camera rate depends on lighting; Sonix downward mono global-shutter reaches far higher rates |
| Underwater refraction | Existing Pinax optics solution already documents substantial geometry error if flat port is ignored |
| Flow | Already includes feature quality checks, caustic filters, gyro logic, rejection gates and an authority-aware estimator interface |
| Generic enhancement | **Rejected and removed.** 17 tested configurations were not beneficial; gate recall fell 30.4% -> 1.2% in one result |
| Newest documented source holdout | Corrected label-map comparison on 2026-11-21: YOLOv8n 0.882 mAP50 versus YOLO11n 0.872 (one seed, one held-out team) |
| Segmentation | Quantized-domain mask decoder is implemented; host mask extraction still costs per object |
| Field status | Current SROT + Kutum + Mongla-hull configuration **has not yet completed a water trial** |

The model-production workflow **already** requires real-source holdouts, verified empty-water negatives, deduplication, class-label audits, deployment rollback and observed Hailo device accuracy. Do not replace it with a weaker generic training tutorial. The previous holdout initially identified a dramatic flare problem that was **retracted** when different teams' word “pole” was found to mean the flare. Label semantics are part of the system.

## B. The complete causal chain

~~~
light source + water + optical port + lens + exposure
 -> sensor, shutter, sensor firmware
 -> USB UVC or CSI + V4L2/libcamera + original timestamp
 -> decoder / RGB or mono conversion / resize / distortion correction / crop
 -> frame-health checks and information scheduling
 -> neural or classical observations
 -> Hailo runtime + memory + inference + output wire format
 -> host NMS / segmentation decode / embeddings / landmarks
 -> temporal association and lock ladder
 -> refraction-aware geometry / optical flow / localization
 -> controller's belief, stale-observation authority and mission decisions
 -> embedded actuation -> movement -> new light and observations
 -> recorded failure -> sample selection -> labels -> training -> compiler
~~~

Every boundary can create a false sense of correctness. Measure **source capture time** to **action decision time**, not simply process-to-process FPS.

### B1. The optical front end

- A wet camera is **not** an ordinary calibrated air camera. Keep the current flat-port/refraction correction and validation. Pose/metric distance claims need underwater calibration and uncertainty.
- Record camera **serial, mode, gain, exposure, auto-exposure state, housing/port, illumination, water clarity, blur, and color cast** with each video. Exposure blur may dominate any CNN improvement.
- Shutter type and frame rate are separate: a 120 Hz stream with long exposure can still produce unusable features.
- Test whether bounded exposure or controllable illumination improves target localization without creating additional backscatter. The current Fantech ignored attempted manual exposure control; don't prescribe a nonfunctional setting.
- Raspberry Pi 5 PiSP is excellent for *supported CSI/libcamera* camera workflows. The existing **USB UVC** cameras do not automatically pass through that CSI ISP. The Hailo-8 is a neural accelerator, not a general image signal processor.
- Original sensor signal quality cannot be recreated reliably by a restoration model after it is lost.

### B2. Data transport and Pi memory

Illustrative **uncompressed** RGB buffer arithmetic, not achievable sensor claims:

| Frame format | Raw bytes per frame | One 120 FPS stream |
| --- | ---: | ---: |
| 640x360 RGB | 691,200 | 82.9 MB/s |
| 640x640 RGB | 1,228,800 | 147.5 MB/s |
| 1280x720 RGB | 2,764,800 | 331.8 MB/s |

Two full RGB 720p/120 FPS streams cost ~664 MB/s **per additional full-frame copy**. Compression may reduce USB traffic while increasing CPU decode cost. Determine actual MJPEG/YUYV/raw mode, driver timestamps, decode stages and NUMA/cache effects rather than infer performance from advertised FPS.

The existing V4L2 latest-frame mailbox prevents large stale queues. Preserve it as the reference. Evaluate any GStreamer alternative against kernel timestamps and actual detector age. A DMABUF allocation in one library does **not** establish true camera-to-Hailo zero-copy; current public Hailo community reports point to CPU-visible buffers and Hailo DMA at the device boundary.

Pi 5 4 GB and 8 GB versions share the CPU/GPU class; more RAM provides headroom, not higher clocked per-core compute. The Pi should execute deterministic geometry, light image statistics, flow point vetting, sensor fusion, time bookkeeping, mission scheduling, logging and safe recovery. The Hailo should execute supported, compute-heavy neural graph portions.

### B3. Frame preprocessing — each stage must prove itself

The fixed-order stages to audit are: capture/decode; Bayer/YCbCr/RGB/mono conversion; RGB/BGR channel order; color range; resizing/letterbox; port-aware correction; geometric ROI; Hailo-specific mean/scale and quantization; output coordinate unprojection. Record every conversion and memory copy. A single silent swap of RGB/BGR or x/y changes the robotics action and can escape ordinary smoke tests. Existing Mongla documentation already caught both categories of defect.

Do **not** restore generic CLAHE before detection. It was explicitly tested and removed. Only attempt underwater-specific enhancement under a *new predeclared failure condition*, with a held-out downstream task metric. An attractive image and PSNR/SSIM alone are insufficient.

## C. Every additional visual observation worth considering

The goal is broader than detection, depth, stereo and XFeat. Each task below is a **candidate**, not an assertion that a suitable HEF or measured Mongla implementation exists.

| Output / capability | Why a robot might need it | Candidate execution | Verdict / test |
| --- | --- | --- | --- |
| **Image usability / health**: blur, saturation, underexposure, caustic severity, dynamic texture | Refuse optical velocity or target fixes when evidence is invalid | Pi statistics first; learned classifier only if necessary | P0: measure incremental value over existing checks |
| **Floor trackability map** | Select patches supporting stable, nonaliasing flow; reject caustics and blank floor | Gradient/structure tensor on Pi; small segmentation CNN candidate | **P0: strongest untested navigation head** |
| **Scene-change and motion/texture separation** | Distinguish actual motion from moving highlights and water particles | Multi-frame Pi CV; compact CNN if baseline fails | P1: test on independently shifted real floor imagery |
| **Gate opening / traversable aperture segmentation** | Steering through open region, not box center | Existing masks or light semantic segmenter | P1 if mission consumer demonstrably benefits |
| **Target landmarks / keypoint heatmaps** | Geometry of gate corners, drum rim, flare endpoints, mat corners | Small Hailo head or Pi regression | **P1: measure bearing, pose error and success** |
| **Direct target bearing/elevation + confidence** | Control needs angular demand more directly than a bbox | Shared detector backbone + small head | P1: requires accurate refraction-aware labels |
| **Target-presence confirmation** | Reject faint/ambiguous false gates before committing | Tiny ROI classifier | P1: reduce false actions without missing target |
| **Out-of-distribution or novel-scene warning** | Flag unfamiliar mission objects or water regime | Embedding distance, ensemble or tiny CNN | P1: explicit unknown-object holdout required |
| **Semantic floor/wall/surface segmentation** | Safety, range interpretation, boundary avoidance | BiSeNetV2 / PP-LiteSeg / other compact head | P2: requires classes with action consumers |
| **Object appearance embedding** | Distinguish lookalikes, aid rare reacquisition | Compact CNN, TinyCLIP selective | P2: prior ReID was removed because unused; prove new value |
| **Scene/place recognition** | Candidate loop-closure/re-localization trigger | Sparse embedding with geometric confirmation | P2: periodic pool tiles alias; embeddings alone are not pose |
| **Video anomaly / obstruction** | Lens occlusion, particles, stuck camera, frame corruption | Temporal statistics then compact model | P1 if current health checks miss incidents |
| **Event-conditioned ROI** | Spend pixels on distant prop while preserving situational coverage | Existing hysteresis range crop / tracking | P1: preserve reacquisition and off-axis targets |
| **Neural restoration/denoising** | Possible improvement on specifically degraded images | Small image-to-image CNN | P3: existing generic preprocessing was harmful |
| **Super-resolution on distant target ROI** | Recover apparent target size | ESPCN/SPAN family, selectively | P2 only if end-to-end localization improves; can hallucinate |
| **Dense learned optical flow** | Alternative to sparse LK in hard texture | Hailo compiled CNN if possible | P3: bandwidth/latency likely worse than existing LK |
| **Learned temporal track / STARK** | Reacquire through an occlusion | Hailo zoo STARK-Lightning | Existing STARK study must be audited first; no duplicate |
| **Optical fiducial or blinking-signature recognition** | Specialized task markers | Pi spectral/temporal detection | Only if rules define a marker |
| **Open-vocabulary visual similarity** | New props and annotation proposals | TinyCLIP/SigLIP/teacher model | Best OFFLINE until proven in domain |
| **OCR / text / symbolic signs** | Only if tasks include readable text or codes | PaddleOCR or classic markers | Low priority for current missions |
| **Sonar/camera agreement** | Acoustically constrained range and semantic verification | Sensor fusion when sonar exists | Future hardware/data dependency |

**Novelty direction:** A compact visual head that says **“this pixel region is geometrically safe for optical-flow velocity”** could be more useful than another object detector. It is *not proven original*. The first contribution is a public, realistic benchmark demonstrating the effect on underwater navigation.

## D. What Hailo-8 can actually run, as of 2026-11-23

The **Hailo Model Zoo v2.20.0** release (September 18, 2026) is a **models-only** update for Hailo-8/Hailo-8L based on v2.19.0. It retains **Dataflow Compiler 3.34.0** and **HailoRT 4.24.0**. This matches Kutum's recorded runtime family. Do not accidentally install v5.x intended for other chips.

The official H8/8L update lists **SPAN/SPANF super-resolution; DDRNet, PP-LiteSeg and BiSeNetV2 semantic segmentation; ByteTrack; YOLO26 semantic/instance segmentation; YOLO11 OBB; PaddleOCR-v6; YamNet; MiVOLO; STARK-Lightning**. A listed model proves the vendor has compiled a variant, **not** that it serves Mongla's domain, latency requirements or concurrent scheduler.

**Model Zoo reference FPS applies to its published host, PCIe, input dimensions and benchmark mode, not Pi5 Gen3 x1.** One instructive counterexample: an ESPCN x2 grayscale super-resolution model is listed at 1164 FPS on its tiny input, while Real-ESRGAN x2 at 512x512 is listed at only 1.9 FPS. “Hailo can do super-resolution” is not a performance specification.

Check outputs, tensor shape, network contexts, precision and downstream consumer *before* downloading or training each model. Models designed for people, street scenes and text need domain adaptation for underwater gates, flares, drums and textured floors.

## E. How to get more useful work out of one Hailo-8

### E1. Graph and representation design

- **Do not treat 26 TOPS as attainable application throughput.** The architecture has compute, control and memory allocation constraints. Context switching can dominate a nominally small CNN.
- Current three-context YOLO11n versus one-context YOLOv8n is compelling, but the **corrected source holdout already selected YOLOv8n**. Further testing must be tied to a new question (e.g., geometry head, 2-camera concurrency, low-light class failure).
- Compare **single small graph with multiple useful heads** against **separate HEFs** only after computing their real context allocation and output-transfer cost.
- Consider an explicit two-stage pipeline only if its *expected* inference cost and target miss rate improve: inexpensive presence score -> more expensive precise geometry on selected frames.
- Reduce resolution or use a tracked ROI only while preserving full-FOV search/reacquisition. The existing range-crop test suggests a conditional win, but not a universal one.
- Structured channel pruning/width scaling can reduce real dense compute; simply zeroing weights in an otherwise unchanged tensor generally does not guarantee NPU speedup.
- A more capable **offline** teacher can distill appearance or landmark knowledge into a small H8-compatible CNN. Fine-tuning from a prior pool is already part of Mongla's workflow; don't retrain everything from scratch.

### E2. Quantization and compiler behavior

For every candidate preserve: exact checkpoint SHA, ONNX graph, color channel/order, input resize/normalization, representative calibration manifests, DFC and HailoRT versions, model script and *applied automatic script*, HEF SHA, network context count, layer precision, warnings, output format and on-chip numerical parity.

Candidate levers: architecture change; measured resolution/crop alternatives; structured pruning; teacher-student distillation; more representative calibration; selective 16-bit activations; supported 4-bit weights; allocator/compiler context settings. Test one factor at a time. DFC 3.x APIs and specific optimization options must be verified in the pinned installed version; **the presence of a keyword in another DFC generation is not evidence it will compile here**.

Don't claim a successful new optimization on compile success alone. Require **independent held-out accuracy on actual silicon** and per-frame action-quality checks.

### E3. Hailo runtime scheduling

The existing async change fixed a GIL/camera starvation problem. It did not increase the Hailo throughput of the multi-context detector. The repo also documented chip-memory exhaustion after repeated network configure, service overhead, and packed-versus-strided output surprises. Retain those regression tests.

New experiment: compare **fixed-rate**, **max-throughput**, **deadline-aware** and **state-uncertainty-aware** scheduling of forward detection, downward detection and XFeat/geometry-related models. Optimize sample age and actual AUV task success, not raw aggregate FPS. The scheduling policy must know HEF context count, measured worst-case runtime, the mission's active camera and the estimator's need for a new observation.

### E4. Pi-side software execution

Use CPU for: timestamps and queue policy, calibration and optics, Shi–Tomasi/LK if it is already efficient, affine/projective geometry, PnP, Kalman and factor-graph fusion, feature vetting, confidence/mission logic, low-cost quality measures and telemetry. Use Hailo for supported dense learned tensors where **the actual downstream gain exceeds compiler and scheduling costs**.

Monitor p50/p95/p99 **photon-to-decision** age and the age of the actual frame that drives control. An inference that returns 10 ms after submission may still describe a 100-ms-old image. Do not use large inference batches for a control loop without modeling their added age.

The USB cameras do not automatically benefit from the Pi 5 CSI PiSP. Verify camera formats and CPU copy path. True Hailo DMA-BUF “zero-copy end to end” is not documented in the public Pi 5 integration; benchmark actual copies before claiming it.

## F. Make model creation the cheapest part of the loop

Mongla already has a rigorous [model pipeline](../../.claude/context/perception/model-pipeline.md). Extend it rather than replace it.

### From recorded video to a deployed HEF

1. **Raw capture and manifest.** Preserve untouched clips with venue/date, physical camera, calibration and housing, lighting, sensor mode, frame timestamps, depth/IMU when available, mission stage and source license.
2. **Source inventory.** Detect corrupt frames, repeated frames, missing intervals and mislabeled or unknown video sources.
3. **Candidate selection.** Stratify by apparent target size (especially distant first acquisition), viewpoint, turbidity, caustics, blur, lighting, target-present/absent and failure transitions. Use temporal/perceptual hashes to remove near duplicates.
4. **Offline teacher-assisted proposal.** Run a strong desktop model to propose boxes, masks, landmarks and track IDs; propagate annotations over short shots. A teacher output is *not ground truth*.
5. **Human validation in CVAT or equivalent.** Confirm geometry and class semantics, including difficult negatives. Preserve source authors' original labels before canonical mapping.
6. **Source-aware fixed split.** Never randomly separate nearby frames from a single clip into train and test. Reserve entire venues, sessions and teams; remove cross-source duplicates. Keep test labels untouched by model selection.
7. **Predeclare an acceptance bar.** Include early-approach recall, false positives/minute of truly empty water, geometry error, target-selection mistakes and labeling minutes—not mAP alone.
8. **Train.** Begin from a documented baseline, keep multiple seeds, add hard negatives after classified errors. Test fine-tuning, frozen layers, data augmentation, distillation and smaller backbones only after the data gate.
9. **Compile.** DFC 3.34 and H8 targets, with representative calibration images, inspected context count and the applied script.
10. **Validate on device.** FP32 versus quantized-emulator versus H8 tensors; held-out class/size/scene metrics and per-stage latency.
11. **Deploy atomically and rehearse rollback.** HEF plus class/metric sidecar together.
12. **Harvest failures.** Every missed object, false approach, invalid flow, impossible pose or stale observation should become a replayable example.

### A useful experimental design: human-minute efficiency

At fixed labeling time, compare uniform video sampling, diversity sampling, detector disagreement, and explicit failure-driven selection. Measure **held-out mission utility per human label-hour**. This tests whether your data-collection pipeline can outperform the strategy of merely obtaining more raw video.

Some of the archive's hardest problems are not neural architecture problems: inconsistent foreign team class conventions, a shortage of far-range objects, periodic tiled-floor aliasing, camera exposure variance, and unrepresentative pool-to-pool transfer. Training efficiency improvements must respect these facts.

## G. Full performance ledger: one schema for every run

For every new model or system experiment, require the following fields:

| Layer | Mandatory recorded evidence |
| --- | --- |
| Optical / camera | camera ID, frame mode, lens/port calibration, exposure, gain, water condition |
| Source frames | capture timestamps, delivered FPS, dropped/stale count, decode path |
| Host preprocess | input color, resize/ROI, per-stage p50/p95/p99, buffer-copy count if available |
| Hailo | runtime/driver/firmware version, PCIe mode, HEF hash, contexts, HW-only rate, actual call latency, scheduler state |
| Postprocess | CPU ms, detection/mask/keypoint count, exact output tensor interpretation |
| Tracking / geometry | identity continuity, feature quality, orientation/bearing errors, validity/rejection counts |
| Fusion / mission | observation age, state covariance, selected target, false commands and completed tasks |
| Environment | elapsed time, CPU load, temperature, power meter method, enclosure, logging load |
| Learning | source holdout, original labels + mapping, training seed, validation manifest, calibration data |
| Claim | baseline, success bar written beforehand, measured result, rejected explanations, accepted/rejected verdict |

A manifest template (all placeholders mean **unmeasured**, never zero):

~~~yaml
experiment: E01-floor-trackability
status: untested
date: null
source_commit: null
model_sha256: null
hef_sha256: null
hailort: "4.24.0"
dfc: "3.34.0"
h8_contexts: null
input_camera: null
camera_mode: null
holdout_manifest: null
p95_observation_age_ms: null
false_flow_acceptances: null
false_target_actions: null
task_success_rate: null
power_W: null
acceptance_bar_written_before_run: false
conclusion: null
~~~

## H. Priority experiments with falsification criteria

| ID | Hypothesis | Baseline | Acceptance criterion | Falsifier / stop |
| --- | --- | --- | --- | --- |
| **E01** | Lightweight floor-trackability / scene-quality observation prevents false downward flow | Existing caustic + feature + refusal logic | Fewer false accepted velocities on held-out real sequences at acceptable extra latency | No improvement over current gates |
| **E02** | Landmark/bearing head improves target approach | Box + optics + known geometry / PnP | Better held-out angular/pose error and mission alignment | More ambiguous geometry, labels or latency |
| **E03** | Tiny ROI confirmation cuts false steering on weak detections | Existing confidence + tracker ladder | False actions decrease with no meaningful loss of distant target acquisition | Excess false rejection |
| **E04** | A more compact H8 network achieves equal mission perception with fewer contexts | Existing v8n family baseline | Same held-out performance within predeclared tolerance; lower p95 age or shared-device utilization | Compiler infeasible, recall drops, runtime output differs |
| **E05** | Better source-balanced calibration reduces INT8 shift | Existing device compile | Improved silicon accuracy on unseen venue across seeds | No retained change |
| **E06** | Scheduling by uncertainty saves inference cycles while improving decisions | Fixed-rate and mission-camera-exclusive policies | Higher mission success / fewer stale measurements per watt | Scheduler overhead or missed rare targets |
| **E07** | Shared multi-task heads outperform separate graphs | Existing detector/lock/flow combination | Equal geometry and detection under lower worst-case latency | Extra contexts or lower quality |
| **E08** | Task-specific restoration rescues a defined failure subset | Raw frames and **already rejected** generic CLAHE | Held-out mission improvement under same original-frame geometry | Visual attractiveness only or hallucinated features |
| **E09** | Compression / structured pruning saves graph resources | Stable baseline HEF | Lower contexts/time while preserving field recall and calibration | Only disk size decreases |
| **E10** | New H8 v2.20 tracker/segmentation models beat existing tested rungs | Existing STARK studies and lock ladder | Reduced lost-target time at matched false-lock rate and p95 age | No mission-relevant benefit |
| **E11** | Optimized video selection reduces labeling cost | Existing label-contact-sheet workflow | Better held-out recall/false actions per human label-minute | Dataset leakage or no gain |
| **E12** | Silicon/emulator disagreements can cause control mistakes | Existing FP32/HEF validation | Reproducible tensor discrepancy producing a real changed action | Numerical differences without downstream effect |

**Execution priority:** first audit whether E01/E02/E06 are already settled in the current repo. Do only the unmet experiment. No broad implementation sweep and no automatic adoption of pre-trained models.

## I. Practical read-only hardware reconnaissance

Review command help before use and capture all output in a timestamped experiment folder.

~~~bash
uname -a
lscpu
free -h
lsusb -t
lspci -vv
v4l2-ctl --list-devices
v4l2-ctl --device=/dev/video0 --list-formats-ext  # correct node per connected camera
vcgencmd measure_temp
hailortcli fw-control identify
hailortcli parse-hef /absolute/path/to/model.hef
hailortcli benchmark --help
pidstat -rud 1
~~~

Record CPU governor, ambient temperature and actual power at the 5V rail if a valid meter is installed. A software-reported Hailo watt figure is not available on all AI HAT+ configurations. Avoid thermal/stress tests in an uncontrolled sealed hull.

### Safe experiment order

1. Inventory actual HEFs, cameras and existing benchmarks. No new optimization claim.
2. Replay the same real footage through the current published baseline.
3. Introduce **one** candidate observation (E01 or E02) and verify on recorded data.
4. Compare with the original model on the identical Pi and camera feed.
5. Add one closed-loop simulated mission with randomized visibility/dynamics.
6. Only after acceptance, deploy a change to the running system and preserve rollback.

## J. The research contribution worth building

**A mission-aware neural-perception execution framework for low-power autonomous robots**, with evidence that a particular additional observation or schedule reduces closed-loop errors at a fixed energy/latency budget.

This can eventually compare multiple compute backends (Hailo-8, the Pi CPU, another accelerator) and vehicle platforms, but the first credible result should stay small: one underwater navigation failure case, one new observation, one rigorous test and one reproducible improvement.

The novelty claim is **hypothetical** until a literature review and strong matched baselines establish it. A working source repository and carefully documented negative results are more valuable than “world first” wording without evidence.

## K. External sources checked on 2026-11-23

- [Hailo Model Zoo releases, especially v2.20.0 and v2.19.0](https://github.com/hailo-ai/hailo_model_zoo/releases)
- [Hailo public model list by hardware and task](https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/PUBLIC_MODELS.rst)
- [Hailo model optimization](https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/OPTIMIZATION.rst)
- [Hailo-8 runtime source branch](https://github.com/hailo-ai/hailort/tree/hailo8)
- [Hailo-8 driver source branch](https://github.com/hailo-ai/hailort-drivers/tree/hailo8)
- [Ultralytics Hailo export and task boundaries](https://docs.ultralytics.com/integrations/hailo)
- [Raspberry Pi 5 camera software](https://www.raspberrypi.com/documentation/accessories/camera-software.html)
- [Pi 5 ISP explanation](https://www.raspberrypi.com/news/image-processing-on-raspberry-pi-5-our-new-hardware-image-signal-processor/)
- [DMABUF on Pi5 + Hailo-8 discussion](https://community.hailo.ai/t/dmabuf-on-rpi-5-hailo-8/17779)
- [Hailo super-resolution model reference FPS](https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8/HAILO8_super_resolution.rst)
- [FLSea 2026 underwater stereo and visual-inertial datasets](https://doi.org/10.1002/rob.70291)
- [FathomNet marine imagery](https://registry.opendata.aws/fathomnet/)
- [SOVIS sonar-visual real data](https://github.com/weitung/sovis-dataset)
- [2026 underwater enhancement review](https://www.mdpi.com/2313-433X/12/11/526)

## L. Non-goals and integrity rules

- Do not claim that this report benchmarks the physical device.
- Do not silently reintroduce CLAHE, ReID, or unvalidated depth estimates previously removed.
- Do not treat Model Zoo reference FPS as Pi 5 performance.
- Do not treat a HEF compilation, good mAP, or high chip FPS as evidence of an improved mission.
- Do not turn unknown class labels into known negatives without inspecting the original source.
- Do not claim raw frame buffers are “zero copy” without actual memory-path measurement.
- Do not bypass the repo's CI / protected-branch workflow.
- Do not overwrite the prior measurement records with corrected-looking numbers; append retractions and new evidence.

**Decision:** Extend the current stack in the direction of *useful observations, calibrated correctness, and freshness-aware scheduling*, and publish only measured gains that survive source holdouts and end-to-end experiments.

## M. Settled against the repository, 2026-11-24

Section H asked that each experiment be checked against what the repo already measured before
any new work. Each row cites the measurement that decides it, or the bead that holds it open.

| ID | State | Evidence or next step |
| --- | --- | --- |
| E01 floor trackability | **open** | The flow gates exist (caustic and feature checks, turbidity confidence built and off, measured-bars 45); a per-patch trackability map does not. Bead `mongla_ws-3sn`, bar written: fewer bad velocities accepted on the measured 30 cm slides (measured-bars 10), none refused that are good today |
| E02 landmark / bearing head | **open, needs labels** | No corner keypoints anywhere in the stack. Bead `mongla_ws-5h8`; it waits for corner-labelled frames |
| E03 ROI confirmation | **open** | Nothing in the repo. Bead `mongla_ws-di2`, scored on the sealed own-footage check set |
| E04 compact single-context network | **settled** | yolov8n, one context, matches yolo11n after INT8 and runs 3.3x faster on the chip (measured-bars 98); chosen by the owner 2026-11-23 |
| E05 calibration set | **partly** | Every compile already measures INT8 against float on held-out frames and refuses a shift past 0.10 (`tools/bench/hailo_compile.py`, B-82). A source-balanced calibration set is untested; it rides on the next compile of the wide training run |
| E06 scheduling | **partly** | One chip, three networks: XFeat at the anchor's rate costs the detectors nothing (measured-bars 95). Uncertainty-aware scheduling is untested and has no consumer yet |
| E07 shared multi-task heads | **deferred** | No consumer for a second head exists; built now it would ship unreachable |
| E08 restoration | **settled, negative** | Generic enhancement was measured in 17 configurations and removed; a new attempt needs a new, predeclared failure case |
| E09 pruning / compression | **deferred** | The detector is one context at the chip's rate; nothing to buy until a second network needs the room |
| E10 Model Zoo tracker | **settled** | STARK-Lightning on the Hailo-8: 6.7 ms a call and no longer a hold than the follower (measured-bars 96) |
| E11 cheaper labelling | **in progress** | `tools/bench/label_contact_sheet.py`; the auto-label study and its scorer, written before any candidate (`tools/studies/autolabel_score.py`, bead `mongla_ws-qqd`) |
| E12 silicon against float | **partly** | `tools/bench/hailo_int8_shift.py` measures the tensor shift per compile; whether a shift ever changes an action is untested |

Two of the twelve are settled, one is settled negative, four are partly done or in progress,
two are deferred for want of a consumer, and three are open with written bars.

# Perception, sensing and mission: round 2, beyond the existing SOTA dossiers

Scope: this report only covers what is **not** already in `sota/{VERDICT,SOTA-GAPS,vision,localization,planning}.md`, `scouting/bumblebee-2025.md` or the `bumblebee-doctrine` skill. Everything here was done read-only. Bumblebee code was read from fresh `git clone --depth 1` copies of `mission_planner_release`, `filters`, `pose_estimator` and `vision_pipeline` (HEAD 2026-06-20). Hailo figures come from the raw Model Zoo `.rst` tables (DFC v2.19.0). arXiv, robonation.org and bluerobotics.com were blocked, so some claims rest on search snippets and are marked that way.

Labels: **[M]** measured in this tree · **[C]** cited · **[E]** my own estimate or computation, not measured · **[BB]** read from Bumblebee code.

---

## 0. Six findings that change existing conclusions

1. **Our own ESC setting jams the pinger band.** `srot-control-board/docs/ESC_FLASHING.md:99-100` recommends Bluejay PWM at **48 kHz, falling back to 24 kHz**. The SAUVC pinger runs at **45 kHz** (`competition_config.py:164-170`), and RoboSub pingers run at 25–40 kHz. So 24 kHz sits in the RoboSub band with its 48 kHz harmonic just above it, and 48 kHz lands 3 kHz from the SAUVC tone. Bluejay can also run at **96 kHz** [C]. No doc connects the two. We own this firmware setting, so a hydrophone added later would otherwise be fighting noise we chose ourselves.
2. **Bumblebee's acoustics decide one bit.** In `order_by_ping.py:17-18,49-63` [BB], they listen for 10 s, need 3 pings, and then evaluate a half-plane test, `(doa_deg − offset) % 360 < 180`. The result picks between two task orders, and the default when nothing is heard is "not left". The confidence gate is **disabled** (`CONFIDENCE_THREHOLD = -1.0`, `acoustics.py:17`), and the whole acoustics root is **commented out** of the 2024 `mother_c.py:217-218`. They also close the grabber before listening (`order_by_ping.py:21-35`). **So the champion's shipped acoustic requirement is a left/right sign.** Two hydrophones can deliver that. A 4-element array cannot deliver it better.
3. **The fix for the Hailo 98 FPS ceiling is already in the Zoo.** `hailo-vision.md:68-123` [M] shows that multi-context YOLO11 models cap at about 98 FPS, while single-context YOLOv8s runs at 462 FPS. The doc parks the switch as "would have to be justified against accuracy". The Zoo supplies that justification: **yolov8s is 44.6 mAP at 491 FPS, against yolov11n at 39.0 mAP and 185 FPS** [C]. YOLOv8s is both more accurate and faster on this chip. Meanwhile `train-model/SKILL.md:8,27` still fine-tunes `yolo11n.pt`.
4. **The INT8 confidence loss is probably a compile setting, not a property of the chip.** `hailo-vision.md:692-735` [M] measured a 0.08 confidence drop under INT8 and compensated by lowering the runtime threshold. The DFC lowers its optimisation level when it has **fewer than 1024 calibration images or no GPU** [C, Hailo community]. We calibrated on 320 frames (`:777`). The compile log almost certainly shows reduced optimisation, which means no fine-tune and no AdaRound.
5. **"30–300× worse than a DVL" compares us with the wrong DVL.** The 0.01–0.1 % figure in VERDICT is for a high-end DVL-INS. The budget unit, a Water Linked A50, is rated **±1.01 %** as standard (±0.1 % needs a licence), costs **<US$8 000**, and works down to 5 cm altitude [C]. Our 3.4 % is a *scale bias*, and `vision-velocity-and-odometry.md:719` attributes it to height. On that reading, a US$400 altimeter could bring flow within about 1–3× of an A50 [E, needs a falsifier, §2].
6. **"XFeat on a Pi 5: nobody has published it" is already answered in our tree.** It runs at 33.1 ms at 320×240 with the stack up (`sources/vision-velocity-and-odometry.md`, master comparison) [M]. That row should close in VERDICT's "What the field does not know" list.

---

## 1. Cheap acoustics

**Current state.** No hydrophone is fitted (`vehicle-spec.md:42`). Missions refuse to steer on the pinger (`sauvc_target_acquisition.py:5-15`). The ask is recorded as ROADMAP H2 (`ROADMAP.md:164`). The only pinger at stake is SAUVC's 50-point drum, a 45 kHz RJE ULB-362B.

**What others do** [C]:
- USC 2025: 3 hydrophones in an L, an Arduino Portenta H7, and a front end reaching 40 dB SNR.
- UBCO: Aquarian S1 hydrophones, a 6th-order Chebyshev filter, and a Zynq-7020 FPGA doing cross-correlation.
- One team samples simultaneously at 5 MHz.
- Older USC boards used analogue phase comparators, and later moved to a 4-element tetrahedron.
- No RoboSub team publishes a measured bearing error. I searched for one and found nothing.

In a pool, full-window cross-correlation of a reverberant ping gives multiple peaks. The published fix is to use the **first arrival** only (sparse-representation pinger localisation, IEEE 8096292; swimming-pool multipath study PMC8840367).

**Proposal: a two-element array, phase-based, with first-arrival gating, on a dedicated Pico 2.** Not the thruster co-processor, which is safety-critical.
- **Sampling.** The RP2350's internal ADC does 500 kS/s in total [C]. Round-robin over two channels gives 250 kS/s each, which is 5.6 samples per 45 kHz cycle. The inter-channel skew is a fixed 2 µs, which is 32.4° of phase at 45 kHz. It is deterministic, so it is calibrated out once [E]. Four channels would still give 125 kS/s each, above Nyquist for 45 kHz. An external simultaneous-sampling ADC is only needed for a 3-D array.
- **Geometry.** Spacing d ≤ λ/2 = **16.7 mm** at 45 kHz (18.8 mm at 40 kHz) keeps phase unambiguous [E].
- **Noise-limited accuracy.** 1 ms integrated at 250 kS/s and 10 dB SNR gives σφ ≈ 0.02 rad, so δθ ≈ **0.4°** broadside [E]. Multipath, not noise, will set the real number.
- **Multipath window.** For a pinger 5 m away with 3 m of combined depth, the surface bounce arrives **0.55 ms** after the direct path [E]. Gate on onset and use only the first ≈0.4 ms.
- **Ego-noise blanking, which only we can do.** Once the 2 s ping period has locked, the board can cut DShot to zero for about 5 ms around each expected arrival. Bidirectional-DShot RPM also gives the exact harmonics to notch. Bumblebee's equivalent is closing a grabber.
- **BOM** [E]: 2 hydrophones (DIY potted piezo about $10, or Aquarian AS-1 at roughly $150–200 each, price unverified), a dual op-amp band-pass front end (~$20), a Pico 2 (~$5), and two penetrators. Total is **~$60 DIY to ~$450**.
- **Effort.** Front end, 2 days. PIO/DMA capture plus Goertzel phase, 3 days. Pool calibration, 1 session.

**Falsifier.** Put the pinger at 4 known bearings in a pool and record 20 pings each. If the sign is wrong more than 5 % of the time at ≥20° off-axis with thrusters idle, or if the error at 96 kHz ESC PWM is no better than at 48 kHz under thrust, the design fails.

**Priority.** Do the ESC change **first**. It is free, and it is a precondition for everything else here. Then the hardware ask, H2 narrowed to "two elements".

---

## 2. Ranging, altimetry and velocity without a DVL

**Current state.** `pool_depth_m` defaults to NaN and flow refuses without it (`flow_node.py:199`, `bringup.launch.py:184`). Tile height is off (`tile_m = 0.0`, `flow_node.py:262-272`). The SAUVC floor slopes from 1.6 m to 1.2 m.

**The options** [C]:

| sensor | price | what it gives | verdict |
|---|---|---|---|
| **Ping2** 115 kHz, 25° beam, 100 m range, 300 m depth rating | ~£380 | true height above floor; replaces the hand-typed pool depth and fixes the slope | **the best value** |
| Cerulean S500 | $595 | 0.3 m minimum range | alternative |
| **Ping360** 750 kHz, 2°×25° beam, 50 m range | ~€2 750 | scanning image; a 360° sweep takes seconds [E: 180 steps × about 13 ms round trip at 10 m ≈ 2.4 s] | fallback-grade only; too slow for the 2.4 s blackout |
| **DVL A50** | <$8 000 | ±1.01 %, from 5 cm altitude | the honest ceiling; ~20× the cost of a Ping2 |
| Green line-laser triangulation | ~$20 | 0.2–0.4 mm σ at ≤15 cm range, from the literature | short range only, needs a second rigid mount; ROADMAP H3 stays parked |

**Proposal A (hardware, ~$400).** A downward Ping2 whose altitude becomes a filter measurement, `h_floor = z_floor − p_z`. The dialect already carries `DISTANCE_SENSOR` (`VISION_API.md` §0).

**Proposal B (software, now).** Make floor depth a **slow random-walk state** in the RIEKF, not a launch argument:
- It is observed by tile height (`tile_grating.py`) when tiles are visible.
- It is observed by the Ping2 when fitted.
- With neither, it stays a prior with a stated σ, and flow's R grows with that σ instead of refusing outright.
- Scale-from-IMU is observability-limited in cruise, which the existing doc already notes; this route avoids needing it.

**Falsifier.** Do 3 × 30 cm slides at two heights with the Ping2 fitted. If flow's scale bias does not fall from 3.4 % to below 1.5 %, the bias is not height, and the Ping2 buys less than claimed.

**Priority.** A is hardware ask #1. B is software and medium priority.

---

## 3. Detection and pose on Hailo-8

**Current state.** The deployed models are custom YOLO11n, multi-context ×3, at about 98 FPS (`hailo.py:630-639`, `hailo-vision.md:68-123`). Pose is XFeat plus IPPE/SQPnP (`anchor/pose.py`). There are no keypoint models. The Zoo seg models are COCO-only.

**Hailo-8 Model Zoo, batch 1** [C]:

| model | mAP | FPS | contexts |
|---|---|---|---|
| yolov8n | 37.0 | 1036 | single [M] |
| **yolov8s** | **44.6** | **491** | single [M: 462.4 on our Pi] |
| yolov11n / yolov11s | 39.0 / 46.3 | 185 / 111 | multi ×3 [M] |
| yolo26n / s | 40.0 / 47.5 | 155 / 97.8 | multi, presumably [E] |
| **yolov8s_pose** | 59.2 (kpt) | **393** | — |
| yolov8m_pose | 64.3 | 145 | — |
| yolov8n_seg / s_seg | 30.3 / 36.6 | 528 / 107 | n = single [M] |
| scdepthv3 | RMSE 0.48 | 929 | — |

There is no OBB model and no optical-flow model in the Hailo-8 Zoo. CLIP and SigLIP image encoders are present.

**Proposals:**

- **3a. Retrain every task model on YOLOv8s** (software, 1–2 days per model on the existing datasets). Expected: ≈4.7× chip rate (98 → ~460 FPS, measured on the stock model), +5.6 COCO mAP over 11n, and much smaller swap and SRAM residency. That cheapens the dual-camera activation turn-taking (`hailo.py:188-208`).
  - Falsifier: held-out-**session** recall from `tools/model_select.py`. If v8s does not match or beat the current 11n on cross-session recall, keep 11n. A higher FPS number is not a reason to switch.
- **3b. yolov8s-pose with 4–8 keypoints per rigid prop** (torpedo board corners, gate top/bottom, bin rim corners), fed straight to the existing SQPnP/IPPE interval.
  - This is CMU's 2026 route. It needs no template, which XFeat does, and the keypoint visibility flag gives an occlusion state for free.
  - Apply doctrine reflex 2 when labelling: order corners by a second feature so the board's symmetry does not become a flip.
  - ⚠ Quantisation pitfall [E, verify]: 8-bit keypoint regression over 640 px may exceed our 1.55 px PnP noise. Compile the keypoint output layers at 16-bit (`quantization_param(..., precision_mode=a16_w16)` in the `.alls`) and **measure reprojection before and after**.
  - Keep XFeat as the refinement stage and blackout anchor. Doctrine reflex 1: publish both frames.
  - Falsifier: pose p90 yaw on constructed truth, keypoints against XFeat, at 1–3 m. If keypoints are worse than XFeat's 1.40°, they are only an initialiser.
- **3c. Recompile at full optimisation level.** Use ≥1024 in-domain RGB frames, on a GPU box, and assert in CI that the log contains no "Reducing optimization level" warning. This is the G-15 guard broadened to cover the second silent downgrade.
  - Falsifier: if the INT8 confidence shift stays near 0.08, optimisation level was not the cause.
- **3d. Tiled inference during acquisition only.** At v8s rates, 3 tiles × 30 Hz fits easily. This addresses the unmeasured far-range floor (the 10 px COCO number) without a second HEF. Turn it off once the target is locked; `rangecrop.py` already covers close range.
- **Multi-model scheduling.** Keep the measured one-VDevice manual activation. ROUND_ROBIN in one process SIGSEGVed here (`hailo.py:196-199`). What is new is that single-context models change the swap cost, so the 4.15 ms swap figure should be re-measured after 3a.

---

## 4. Underwater training data (augmentation, not preprocessing)

**Current state.** None of the 25 archived configs uses blur, rotation, perspective or contrast augmentation (`preprocess.py:44-45`). There is no synthetic data in the model path.

**Evidence** [C]:
- Physics-informed augmentation for YOLOv12 (arXiv 2506.23505): flip +2.4, PSF blur +1.5, HSV attenuation +1.9, structured erasure +1.1 mAP50, and **−22.4 % small-object false negatives**. This is augmentation at *training* time, so it does not contradict our preprocessing retraction.
- Copy-Paste (Ghiasi, CVPR 2021): **+6.9 box AP at 10 % of COCO**; 75 % of the data with Copy-Paste matches 100 % without.
- Synthetic data plus real barely moves the needle (0.796 vs 0.793 mAP50). Synthetic alone reaches 43 % of real-trained mAP50, as already recorded.

**Proposal** (software, ~2 days): an Ultralytics augmentation recipe plus a copy-paste script.
- Paste mask-cut prop crops from our archive onto **empty real pool frames** from each venue.
- Add depth-dependent blur and colour attenuation.
- Overlay caustic textures on downward frames. This is cheap caustics robustness for bin detection.
- Leave blur and noise mild. The existing robustness sweep found noise costs −75 to −81 %, so do not teach the model to accept noise.

**Falsifier.** Hold out one session and retrain with and without the recipe. If cross-session recall does not improve, the domain gap is not photometric, and active selection (G-16) is the only lever.

---

## 5. Mission layer

**Current state.**
- `resilience.retry/never_fails/selector` (`resilience.py:41,75,99`) is the same vocabulary as Bumblebee's `Retry`, `FailureIsSuccess` and `Selector`. Doctrine reflex 6 applies: behaviour trees add nothing here.
- `run_plan` (`mongla_dsl.py:1129`) orders by `points / worst_case_s` (`run_budget.py:141-153`).
- `Task` carries **no success probability** (`run_budget.py:34-44`). The "expected-value ordering" in `planning.md` is really *points density*.

**Proposal 5a. Put a real expectation into `Task`** (software, half a day).
- Add `p_full` and `p_fallback` as Beta posteriors counted from scorecards across practice runs. Scorecards already record verb outcomes.
- Order by `(p·points)/worst_case_s`.
- This makes pool-day statistics steer competition order. No team in the sweep publishes this, and Bumblebee's trees are hand-ordered.
- Falsifier: replay 10 practice scorecards. If Beta-weighted ordering never changes the plan from points-density ordering, drop it.

**Proposal 5b. A runtime shield on the board, not just a checker on the host.**
- The Simplex / runtime-assurance literature puts an unverified advanced controller beside a small verified reversionary one, with a monitor switching between them (Black-Box Simplex arXiv 2102.12981; NASA RTA framework NTRS 20240007986). ROS-level monitors exist too: ROSMonitoring 2.0 (arXiv 2411.14367) and Reelay for STL.
- Our architecture already has the reversionary controller physically separate: the ESP32 failsafes (SURFACE on 5 s of silence, leak, battery).
- Move 3–4 *mission-independent invariants* there as board parameters:
  - maximum depth;
  - maximum continuous time in AUTO/STABILIZE without a host heartbeat that carries a fresh aiding flag (`localization_node.py:406-434` already publishes `aided/unaided`);
  - a maximum yaw-rate envelope.
- This complements the pre-flight AST checker (G-20, which is static). The shield catches what a static check cannot: a verified mission meeting an unverified world.
- Firmware PR, ~2 days. The falsifier is injection on the bench: kill the aiding topic and watch the board refuse AUTO within the configured window.

**Task timing.** Bumblebee 2026 lengthened settling from 3 s to 15 s and cut retries (already in the doctrine). What is new from the code is the default-when-deaf behaviour: a 10 s listen with a deterministic default order. In our budget terms, the listen is a fixed 10 s cost with no worst-case penalty. With the p-weighting in 5a, it should only happen when p(correct side) × (bonus) > 10 s × (points density of the next task).

---

## 6. Localization beyond the current filter

**Checking the filter against the invariant-EKF literature** (Barrau & Bonnabel, IEEE TAC 2017; Hartley et al., IJRR 2020):
- Depth and position fixes are world-frame, i.e. right-invariant observations. Their H is state-independent in our filter. Correct.
- Flow is a body-frame velocity, a left-invariant observation. Its H = Rᵀ, which depends on the state estimate (`inekf.py:247-262`). That is expected, and the Jacobian is finite-difference-verified.
- Hartley shows the error dynamics A are exactly state-independent apart from the bias terms. So `Φ = expm(A·dt)` is exact and cheap: one 15×15 expm per IMU step, or a closed form. That makes the first-order Euler noted in `localization.md` a cheap upgrade rather than a redesign [C/E].
- Biases break group-affinity ("imperfect IEKF"). Treat the Allan variance (B-6) as the bigger lever.

**Props as landmarks, when the course has not been surveyed** (G-09's failure case).
- `resection.py` and `heading_anchor.py` already do the known-map case, and Bumblebee's heading re-anchor already exists here.
- Beyond that: treat rulebook layout distances as **landmark priors with σ ≈ 1–2 m** and solve a small sliding-window least-squares problem jointly over the last N keyframe poses and ≤10 prop positions. Inputs are bearing, range from PnP, and the heading anchor.
- This is "SLAM with a prior map" at toy scale: roughly 200 variables, not iSAM2 at scale.
- No Pi-class GTSAM or iSAM2 timing exists in the literature (searched; confirms the owed item). My estimate is a sparse LM on this size in the low milliseconds per solve on a Cortex-A76 [E]. Measure with scipy `least_squares` before adding GTSAM as a dependency.
- Falsifier: replay a bag with deliberately wrong course priors (+1 m). If the jointly solved prop positions do not move toward truth, or the pose error does not beat resection-with-wrong-map, drop it.

**Visual-inertial odometry on the downward camera.** Nothing new beyond AQUALOC (it stops rather than drifting). Rejected again. Do not add OpenVINS or ORB-SLAM3.

---

## 7. Ranked top 12

**Software only:**

| # | move | cost | why this rank |
|---|---|---|---|
| 1 | **ESC PWM 96 kHz, or at least off 24/48** before any acoustic work, with a bench check of the thrust and deadband curve | a reflash + 1 bench hour | free, and a precondition for any acoustics; it may also change the G-12 deadband, so re-measure that too |
| 2 | **Retrain task models on YOLOv8s** (3a) | 1–2 days per model | 4.7× chip headroom and higher COCO mAP; frees the chip for pose, seg and tiling |
| 3 | **Recompile at full optimisation level** with ≥1024 frames on a GPU, plus a log guard (3c) | 1 day | the likely cause of the measured 0.08 confidence loss |
| 4 | **yolov8s-pose keypoints → existing SQPnP** (3b), with 16-bit keypoint outputs | 3–4 days + labelling | template-free pose for torpedo, gate and bin; keypoints give geometry and the class gives identity |
| 5 | **Copy-paste + physics augmentation recipe** (§4) | 2 days | +6.9 AP in the low-data regime (cited); our datasets are low-data |
| 6 | **p_success in `Task` from scorecards** (5a) | 0.5 day | turns pool days into competition ordering |
| 7 | **Floor depth as a filter state** (2B) | 2 days | removes the hand-typed constant; ready for the Ping2 |
| 8 | **Board-side runtime shield** (5b) | firmware PR, 2 days | a Simplex-style reversionary controller we already physically own |
| 9 | **Prior-map joint landmark window** (§6) | 3 days | makes an unsurveyed venue usable |

**Hardware asks:**

| # | ask | cost | why |
|---|---|---|---|
| 10 | **Ping2 altimeter** | ~$400 | the binding constraint on flow scale (already the documented verdict); may bring flow to the A50 class at 5 % of the price |
| 11 | **2-element hydrophone on a Pico 2**, with thruster blanking | $60–450 | matches exactly what the champion's code consumes (1 bit); 50 points at SAUVC |
| 12 | Ping360 only if blackout statistics after #2–4 still show p99 > 2 s | ~$2 750 | the sonar fallback remains the documented ceiling; this is the cheapest version, and scan-rate-limited |

The DVL A50 (<$8k) is deliberately left out until #10 is measured.

---

## 8. Where we can surpass Bumblebee

Their control architecture is ArduSub behind one MAVROS setpoint. Their acoustics are a 1-bit half-plane test whose confidence gate is off. Their controls stack, localisation and UKF are withheld.

1. **Board-side visual servo at 500 Hz.** `LANDING_TARGET` ingest is specified (`VISION_API.md`) and not implemented. With v8s at ~460 FPS and board-side gyro de-rotation between frames, image error reaches the loop in about 20 ms and is extrapolated with IMU at 500 Hz. They cannot do this without owning ArduSub's loop.
2. **Acoustics in sync with the actuators.** We can blank thrusters in millisecond windows timed to the ping period, and notch at the exact RPM-derived harmonics. We also choose the ESC's PWM frequency to keep it out of band. Closing a grabber is their version.
3. **Thrusters as sensors.** Bidirectional DShot plus firmware we own gives per-ESC current once PR #4 lands. That enables current-based fault detection and command-velocity aiding (DeepVL-shaped inputs) that no ArduSub team gets per-ESC.
4. **A shield in firmware.** Invariants enforced below the mission on an independent MCU, parameterised, and injection-tested.
5. **Covariance honesty.** They moved from `PoseWithCovarianceStamped` to `PoseStamped`. We publish aiding state and a derived covariance, and could publish floor-depth σ (2B).
6. **Held-out-session model selection** combined with a chip whose single-context headroom (after #2) allows more models in parallel than their Orin plan, per the redundancy doctrine.

---

## Sources

- Bumblebee `mission_planner_release` (robosub24 acoustics, robosub26 mother), cloned: https://github.com/BumblebeeAS/mission_planner_release
- Hailo Model Zoo, Hailo-8 tables (object detection, pose, instance segmentation, depth, zero-shot): https://github.com/hailo-ai/hailo_model_zoo/tree/master/docs/public_models/HAILO8
- Hailo optimisation guidance, ≥1024 images, GPU requirement: https://community.hailo.ai/t/optimiziation-warnings-meaning/9429 · https://community.hailo.ai/t/problem-wigh-gpu-on-hailo-warning-reducing-optimization-level-to-0-the-accuracy-won-t-be-optimized-and-compression-won-t-be-used-because-there-s-no-available-gpu/18264 · https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/OPTIMIZATION.rst · https://docs.ultralytics.com/integrations/hailo
- RP2350 ADC (500 kS/s, 4/8 channels): https://en.wikipedia.org/wiki/RP2350 · https://github.com/steve-m/hsdaoh-rp2350
- RoboSub hydrophone systems: https://robonation.org/app/uploads/sites/4/2025/07/RS25_TDR_Univ-of-Southern-California-RoboSub-SC.pdf · https://robonation.org/app/uploads/sites/4/2025/07/RS25_TDR_Univ-of-British-Columbia-Okanagan-Marine-Robotics-compressed.pdf · https://github.com/cheehieu/usc-auv-passive-sonar · https://github.com/Abhicoder1999/TDOA-Hydrophones · https://github.com/xiongyihui/tdoa
- Pool multipath and first arrival: https://ieeexplore.ieee.org/document/8096292 · https://pmc.ncbi.nlm.nih.gov/articles/PMC8840367/
- Aquarian AS-1 sensitivity: https://www.aquarianaudio.com/AqAudDocs/AS-1_manual.pdf
- Bluejay PWM 24/48/96 kHz: https://github.com/bird-sanctuary/bluejay · https://www.unmannedtechshop.co.uk/blogs/knowledge-base/bluejay-pwm-frequency-guide-esc-settings
- Ping2: https://www.robotshop.com/products/bluerobotics-ping2-sonar-altimeter-echosounder · https://www.carcinus.co.uk/product/ping2-sonar-altimeter-and-echosounder/ · Cerulean S500: https://www2.whoi.edu/site/marinerobotics/wp-content/uploads/sites/32/2020/08/f1.2.Cerulean_Ulmer.pdf
- Ping360: https://docs.bluerobotics.com/ping-viewer/device-settings-ping360/ · https://www.seascapesubsea.com/product/ping360-scanning-imaging-sonar/
- DVL A50: https://www.waterlinked.com/datasheets/dvl-a50 · https://bluerobotics.com/introducing-the-reef-water-linkeds-dvl-a50/
- Laser triangulation: https://www.researchgate.net/publication/358784433 · https://doi.org/10.3390/jmse9010079
- Physics-informed augmentation: https://arxiv.org/pdf/2506.23505 · Copy-Paste: https://arxiv.org/abs/2012.07177 · synthetic+real: https://arxiv.org/html/2609.20680
- Keypoints → PnP: https://www.ultralytics.com/blog/how-to-use-ultralytics-yolo11-for-pose-estimation · https://arxiv.org/pdf/2205.02536
- Invariant EKF: Barrau & Bonnabel, IEEE TAC 62(4) 2017 (https://www.researchgate.net/publication/266560636) · Hartley et al., IJRR 2020 https://arxiv.org/pdf/1904.09251 · https://github.com/RossHartley/invariant-ekf
- GTSAM / iSAM2 (no Pi timing found): https://gtsam.org/tutorials/intro.html · miniSAM https://arxiv.org/pdf/1909.00903
- Runtime assurance: https://arxiv.org/pdf/2102.12981 · https://ntrs.nasa.gov/citations/20240007986 · ROSMonitoring https://github.com/autonomy-and-verification-uol/ROSMonitoring · https://arxiv.org/abs/2411.14367

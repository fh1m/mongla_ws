# SOTA: visual odometry, velocity sensing and monocular depth for DVL-free underwater robots on edge hardware

Compiled 2026-09-22. Every claim carries a source link and a number. Where a number could
not be found in the source, it is listed in **Claims I could NOT verify** at the bottom
rather than guessed.

**Our baseline, for comparison throughout:** sparse Shi-Tomasi + LK + RANSAC similarity on
the downward camera, **12.34 ms/pair** on a Pi 5, de-rotated with IMU, `v = h·(flow_px/dt −
f·ω)/f`. Measured: worst error **1.09 cm** over three 30 cm slides, noise floor
**0.57 mm/s**, scale **103.4 %** of truth, de-rotation takes pure-rotation error
**575.7 → 57.1 mm/s**. Needs a known height. Fails on bare floor, sun caustics, and
rotation-dominated intervals (≥1.128 rad/s yaw).

**The common yardstick.** Papers report accumulated pose drift as *% of trajectory*; ours is
a *velocity* sensor. The bridge is scale bias, which integrates linearly into trajectory
drift while the noise floor integrates as a negligible random walk:

- scale 103.4 % ⇒ **a 3.4 % drift floor** unless something else corrects it;
- 30 cm × 3.4 % = **1.02 cm**, against a measured worst error of **1.09 cm** — our worst-case
  flow error *is* our scale bias, nothing else;
- 0.57 mm/s over a 100 s run random-walks to well under a centimetre.

So **read every "% drift" below against our 3.4 %.**

---

## 1. Learned VO / VIO underwater

### DIVO (McGill, Jul 2026)

[DIVO: Continuous-time DVL-Inertial-Visual Odometry for Unmanned Underwater Vehicles](https://arxiv.org/abs/2607.04615)
— first continuous-time (Gaussian-process) trajectory estimator fusing DVL + stereo + IMU,
and the first underwater odometry system with a **learned frontend**
(**SuperPoint + LightGlue**).

Measured, from the [full paper](https://arxiv.org/html/2607.04615v1):

| Sequence | DIVO position ATE | rotation ATE |
|---|---|---|
| Simulated circle | **0.1148 m** (RMSE 0.1132 m) | 0.0136° |
| EuRoC Vicon Room (sim) | **0.0717 m** (RMSE 0.0673 m) | 0.0163° |
| Quarry Conveyor1 | **0.332 m** | — |
| Quarry Conveyor2 | **0.347 m** | 1.437° |
| Quarry Truck1 | **0.530 m** | 1.870° |
| Quarry Truck2 | **0.233 m** | 0.909° |

Baselines it is compared against: **ORB-SLAM3, OKVIS2-X (± Unimatch), MSCKF-DVIO,
AQUA-SLAM**.

**Hardware: Intel i9-13900H + NVIDIA RTX 2000 Ada (8 GB), running at 8 fps.** That is a
laptop dGPU, not an edge board — and 8 fps is ~125 ms/frame, **10× our 12.34 ms**. It also
**requires a DVL**, which is the sensor we do not have. Relevance to us: the *frontend*
choice (SuperPoint/LightGlue over KLT) is the transferable idea, not the system.

### AQUALOC — the benchmark, and what it shows about classical VO underwater

[The Aqualoc Dataset (arXiv 1809.07076)](https://arxiv.org/abs/1809.07076) — monocular
camera + low-cost MEMS IMU + pressure sensor, on deep-sea archaeological sites and a
harbour; ground truth from offline COLMAP SfM.

The sharpest number in the literature about it is a **failure** number, not an accuracy
number: on AQUALOC sequences, ORB-SLAM3 tracked only **1.426 m of a 16.212 m** trajectory
(seq 6) and **4.665 m of 8.041 m** (seq 7) before losing tracking and resetting to origin
([Attenuation-Aware Weighted Optical Flow, arXiv 2407.13159](https://arxiv.org/pdf/2407.13159)).
*Read that as: feature-based monocular SLAM underwater does not merely drift, it stops.*
Our LK-flow velocity sensor has no map to lose, which is a structural advantage, not a
minor one.

### DeepVL (see §2) as a VIO aid

Same paper reports that fusing its learned velocity into VIO gave equivalent odometry with
**an order of magnitude fewer visual features** — VIO with only **22 features** reached
**2.2 % relative position error**.

| Method | What it measures | Accuracy | Runtime on edge | What it needs that we lack | Verdict vs our LK flow |
|---|---|---|---|---|---|
| DIVO | pose (DVL+stereo+IMU, continuous-time, SuperPoint/LightGlue) | ATE 0.233–0.530 m real; 0.0717–0.1148 m sim | **8 fps on i9-13900H + RTX 2000 Ada** (~125 ms) | a **DVL**, a stereo rig, a discrete GPU | **10× our cost, needs the sensor we don't have** |
| ORB-SLAM3 on AQUALOC | monocular/stereo SLAM pose | tracked **1.426 m of 16.212 m**; **4.665 m of 8.041 m** | — | — | **does not merely drift — it stops** |
| AQUALOC benchmark | — (dataset) | GT from offline COLMAP SfM | — | deep-sea + harbour imagery unlike a pool | useful as a stress set, not as a method |

---

## 2. DeepVL / model-learned velocity (velocity from IMU + thrust, no camera)

[DeepVL: Dynamics and Inertial Measurements-based Deep Velocity Learning for Underwater
Odometry (ICRA 2025, arXiv 2502.07726)](https://arxiv.org/html/2502.07726) ·
[code: ntnu-arl/DeepVL](https://github.com/ntnu-arl/DeepVL)

This is the most directly relevant paper to our situation, because it is explicitly a
*DVL-free, camera-optional* velocity source.

- **Inputs:** 3-axis accel, 3-axis gyro, J motor commands, **battery voltage** (7+J channels).
- **Model:** 3 GRU layers, hidden dim 40, **28 k trainable parameters**; ensemble of **8**
  networks for uncertainty.
- **Training data:** **3 h 40 min** in a laboratory pool + **20 min** in the Trondheim
  Fjord → **≈120 k sequences** of 15 s (300 samples each), ≈10 k validation sequences.
  Training took **~1 hour on an RTX 3080**.
- **Accuracy:** **3.9 % relative position error during full visual blackout**; **2.2 %**
  when fused with a 22-feature VIO; **0.39 m RMSE over 10 m deltas** across **88
  trajectories** of 100–300 m.
- **Runtime: < 5 ms on an NVIDIA Orin AGX (CPU *or* GPU).** A 28 k-parameter GRU is small
  enough that a Pi 5 CPU is plausible — but that is an inference of mine, not a measured
  number in the paper.
- **Platform:** a real custom BlueROV with a 5-camera AlphaSense module, Orin AGX 32 GB,
  BMI085 IMU at 200 Hz, cameras at 20 Hz, speeds 0–0.8 m/s. **It flew.**

**Verdict vs our LK flow:** not a competitor, a **complement** — and closer in accuracy than
it first looks. Using the common yardstick from the top of this file, our flow's
**scale-limited drift floor is 3.4 %**; DeepVL's blackout drift is **3.9 %**. Those are the
same number to within the measurement. DeepVL achieves it **with no camera at all**. The
point of DeepVL is that it **keeps producing a velocity when the camera cannot** — exactly
our bare-floor / caustics / rotation-dominated failure set. Cost: it needs **hours of
labelled velocity data**, which means a DVL or a motion-capture pool to train against. That
is the thing we lack.

Related, same family:
[Diver velocity estimation using inertial measurements and LSTM trained on DVL data](https://www.sciencedirect.com/science/article/pii/S0967066126002625) ·
[A Deep Learning Approach To Dead-Reckoning Navigation For AUVs With Limited Sensor Payloads (arXiv 2110.00661)](https://arxiv.org/pdf/2110.00661)
— same family (learn velocity from proprioception); **I did not extract their numbers**, see
the unverified list.

| Method | What it measures | Accuracy | Runtime on edge | What it needs that we lack | Verdict vs our LK flow |
|---|---|---|---|---|---|
| DeepVL | body-frame velocity + uncertainty from IMU, thrust, battery | **3.9 %** blackout; **2.2 %** w/ 22-feature VIO; **0.39 m RMSE / 10 m** over 88 traj. | **<5 ms on Orin AGX**, 28 k params, 3×GRU(40) | **~4 h of DVL- or mocap-labelled velocity truth** | **complement, not competitor** — same accuracy class (3.9 % vs our 3.4 % floor), **works with the camera off** |

---

## 3. Dense / semi-dense flow vs sparse LK

The question is narrow: **does any learned dense flow beat sparse LK *as a velocity sensor*
at comparable cost on a Pi 5?** The answer from the numbers is no, and the margin is large.

[NeuFlow v2: High-Efficiency Optical Flow Estimation on Edge Devices (arXiv 2408.10161)](https://arxiv.org/abs/2408.10161v2)
is the best edge case in the literature. Its headline: **>20 FPS at 512×384 on a Jetson
Orin Nano** — i.e. **≤50 ms/frame on a 1024-CUDA-core GPU**. That is **≥4× our 12.34 ms**,
on hardware we do not carry. Accuracy: **KITTI-15 EPE 4.33** vs NeuFlow v1's 12.4, claimed
**10×–70× speedup** over SOTA at comparable accuracy. Trained on FlyingThings; evaluated on
Sintel (1024×436) and KITTI-15 (1242×375).

[RAFT (arXiv 2003.12039)](https://arxiv.org/pdf/2003.12039): RAFT-small is **1 M
parameters**; full RAFT is quoted at **~100 ms on a GTX 1080Ti**. The RAFT authors
benchmark on desktop GPU only. [DIFT (arXiv 2306.05691)](https://arxiv.org/pdf/2306.05691)
reports that RAFT's memory footprint makes it impractical on mobile, with **a single
iteration below 0.2 inferences/second** on a mobile platform — i.e. seconds per frame.

Two structural points that the runtime numbers understate:

1. **Dense flow solves a harder problem than we need.** A velocity sensor needs one global
   2-DOF translation + 1 rotation, which RANSAC over ~100 sparse tracks already gives. A
   dense field is ~200 000 vectors of which we throw away all but the consensus.
2. **We already rejected the dense-ish alternative on measurement.** Fourier-Mellin phase
   correlation: **16.87 ms vs 12.34 ms, and 10–30× the error.** That is the same shape of
   result the runtime table predicts for the learned models.

| Method | What it measures | Accuracy | Runtime on edge | What it needs that we lack | Verdict vs our LK flow |
|---|---|---|---|---|---|
| Our sparse LK + RANSAC | body velocity (px→m via height) | 1.09 cm / 30 cm slide; 0.57 mm/s floor; scale 103.4 % | **12.34 ms** Pi 5 CPU | — | baseline |
| NeuFlow v2 | dense flow field | KITTI-15 EPE 4.33 | ≤50 ms @512×384 **Jetson Orin Nano GPU** | a CUDA GPU; Hailo-8 port unproven | **loses** — 4× cost, no velocity-accuracy claim |
| RAFT / RAFT-small | dense flow field | SOTA EPE | ~100 ms on GTX 1080Ti; <0.2 inf/s single iter on mobile | desktop GPU | **loses badly** |
| Fourier-Mellin (ours, rejected) | global translation+rot+scale | 10–30× our error | 16.87 ms | — | already rejected on measurement |

**Verdict:** no dense method in 2025–2026 beats sparse LK as a *velocity sensor* at our cost
on our hardware. If dense flow ever earns a place here it will be on the **Hailo-8**, and no
one has published a flow model compiled for Hailo-8 that I could find (see unverified list).

---

## 4. Monocular metric depth, 2025–2026 — and what water does to it

This is the highest-stakes section for us, because a metric depth model that worked in water
would **remove the known-height requirement** that currently makes our flow sensor refuse.

### The models, on land

| Model | Params | NuScenes AbsRel ↓ | NuScenes δ₁ ↑ | Runtime |
|---|---|---|---|---|
| Depth Anything V2 | — | 0.614 | 31.84 | — |
| Metric3D v2 | — | **0.197** | 93.25 | — |
| UniDepth | — | **0.138** | **93.01** | — |
| [Depth Pro](https://arxiv.org/html/2410.02073v2) | **504 M** | 0.287 | 49.1 | **0.3 s on a V100** for a 1536×1536 (2.25 MP) map |

(NuScenes AbsRel/δ₁ comparison from the
[wildlife metric-depth benchmark, arXiv 2510.04723](https://arxiv.org/html/2510.04723v1);
Depth Pro's own numbers from [arXiv 2410.02073](https://arxiv.org/html/2410.02073v2).)
Depth Pro's own zero-shot spread: Sun-RGBD AbsRel **0.113** / δ₁ **89.0** down to Sintel
AbsRel **0.508** / δ₁ **40.0** — a 4.5× spread across domains *in air*. That variance is the
warning label.

### The models, in water — the decisive benchmark

[Underwater Monocular Metric Depth Estimation: Real-World Benchmarks and Synthetic
Fine-Tuning with Vision Foundation Models (arXiv 2507.02148)](https://arxiv.org/abs/2507.02148)
benchmarked six models zero-shot on **FLSea** (12 288 images, Canyon + Red Sea) and **SQUID**
(57 stereo pairs, 4 scenes).

Zero-shot **AbsRel ↓ / δ₁ ↑**:

| Model | FLSea-Canyon | FLSea-Red Sea | SQUID |
|---|---|---|---|
| ZoeDepth | 1.5907 / 0.2345 | 1.3335 / 0.2109 | 1.3214 / 0.0968 |
| Metric3D V2 (ViT-S) | 1.5331 / 0.0967 | 0.8130 / 0.2136 | 1.3059 / 0.1680 |
| Depth Pro | 0.9858 / 0.1557 | 0.3888 / 0.3772 | **3.2185** / 0.1678 |
| Depth Anything V2 (ViT-S) | 0.3576 / 0.4463 | 0.2569 / 0.4722 | 0.5242 / 0.2054 |
| Depth Anything V2 (ViT-L) | 0.2269 / 0.6363 | 0.2307 / 0.4812 | 0.3390 / 0.2896 |
| **UniDepth V2 (ViT-L)** | **0.1156 / 0.9109** | **0.0932 / 0.9439** | **0.3222 / 0.5201** |

**Read those numbers honestly.** Metric3D v2 and Depth Pro — the two that look strongest in
air — **collapse in water**: Metric3D v2 goes from AbsRel 0.197 on NuScenes to **1.5331** on
FLSea-Canyon (≈8×), and Depth Pro to **3.2185** on SQUID (AbsRel > 3 means the prediction is
wrong by more than 3× the true depth). Only UniDepth V2 (ViT-L) holds up, at
**AbsRel 0.0932–0.1156 and δ₁ 0.91–0.94 on FLSea** — but **0.3222 / 0.5201 on SQUID**, so
even the winner degrades 3× across water types.

Fine-tuning Depth Anything V2 ViT-S on a **synthetic underwater Hypersim** variant (physical
image-formation model) helped but modestly: FLSea-Red Sea **0.2569 → 0.2266** AbsRel,
δ₁ **0.4722 → 0.6170**; SQUID **0.5242 → 0.4465**, δ₁ **0.2054 → 0.3204**; FLSea-Canyon
AbsRel actually got **worse** (0.3576 → 0.3620).

Also relevant: [Physics-informed Knowledge Transfer for Underwater Monocular Depth Estimation
(ECCV 2024)](https://www.ecva.net/papers/eccv_2024/papers_ECCV/papers/09034.pdf) transfers an
in-air model using a physical underwater image-formation model and a small unlabelled
underwater set — the same recipe, without metric ground truth.

### Speed on *our* NPU — the number that decides it

[Hailo model zoo, HAILO8 depth estimation](https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8/HAILO8_depth_estimation.rst):

| Model | Input | Params | float RMSE | FPS on Hailo-8 (batch 1) |
|---|---|---|---|---|
| `fast_depth` | 224×224×3 | 1.35 M | 0.6 | **2 519** |
| `scdepthv3` | 256×320×3 | 14.8 M | 0.48 | **929** |

929 FPS is **~1.08 ms** — *cheaper than our 12.34 ms flow pair*. There is also a
[`stereonet` stereo model, Hailo-8 only](https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8/HAILO8_stereo_depth_estimation.rst)
with a [C++ reference app](https://github.com/hailo-ai/hailo-apps/blob/main/hailo_apps/cpp/depth_estimation_stereo/README.md).

⚠ **But these are the small self-supervised models, not the foundation models that scored
well underwater.** `scdepthv3` is 14.8 M params; UniDepth V2 ViT-L — the only model that
survived water — is a ViT-Large, roughly 20–25× larger. **I did not find it in the Hailo-8
depth-estimation model list, and I did not audit the whole zoo** — treat "not available on
Hailo-8" as unconfirmed, not established. Either way the shape of the problem stands: the
model that works underwater is not the model we have a published Hailo-8 number for.

### The refraction trap we already measured

Our own calibration gives **46.7° in water vs the datasheet's 63.8° in air** — a **27 %
focal-length error** for any model that assumes an in-air intrinsic. Metric depth models
consume focal length explicitly (UniDepth predicts a camera; Depth Pro estimates focal
length from the image). Feeding an in-air focal to a metric model in water biases *scale*
directly, which is exactly the quantity we would be buying it for.

| Method | What it measures | Accuracy | Runtime on edge | What it needs that we lack | Verdict vs our LK flow |
|---|---|---|---|---|---|
| UniDepth V2 ViT-L | metric depth per pixel | **AbsRel 0.093–0.116, δ₁ 0.91–0.94 (FLSea)**; 0.322/0.520 (SQUID) | ViT-L; **no published Hailo-8 number found** | Hailo compile; in-water focal; validation in *pool* water | **best hope for removing the height requirement — unproven on our chip** |
| Depth Anything V2 ViT-S | relative → fine-tuned metric | AbsRel 0.227–0.524 underwater | ViT-S is the plausible Hailo candidate | fine-tune data; Hailo compile | too coarse for scale today |
| Metric3D v2 / Depth Pro | metric depth | **collapse in water** (1.53 / 3.22 AbsRel) | Depth Pro 0.3 s on a **V100** | a datacentre GPU | **rejected on evidence** |
| `scdepthv3` (Hailo zoo) | relative depth | float RMSE 0.48 (in-air benchmark) | **929 FPS = 1.08 ms on Hailo-8** | metric scale (it is relative); underwater validation | fast enough, but does not give metric height by itself |

---

## 5. Absolute scale without a DVL

Context first, so the rest of this section is read at the right altitude: **a good DVL-INS
dead-reckons at 0.01–0.02 % of distance travelled, and a typical spec is 0.1 % (DRMS) for
straight-line runs** ([Nortek, Validation of a New Generation DVL for Underwater Vehicle
Navigation](https://www.nortekgroup.com/assets/documents/Validation-of-a-New-Generation-DVL-for-Underwater-Vehicle-Navigation.pdf)).
Our flow's 3.4 % scale-limited floor is **30–300× worse than a DVL**. Every option below is
an attempt to buy back some of that factor without the acoustic hardware.

### 5a. Pressure + floor plane (what we do today)

Ours, implicitly: `h` from the Bar30 plus a known pool depth ⇒ height above floor ⇒ metric
scale. Measured: implied height **0.72 / 0.69 / 0.70 m against a 0.72 m tape**, i.e. worst
spread **3 cm on 0.72 m = 4.2 %**.

⚠ **That 4.2 % is not independent confirmation of the 3.4 % scale bias — it is the same
measurement read backwards.** The implied heights are an *output* of the flow pipeline
(invert the scale equation against known motion), so citing them as agreement with the bias
that produced them is circular. One number, two views, not two agreeing measurements.

The real failure mode here is structural: it assumes a **flat, level floor at known depth**.
In a competition pool that is close to true; over any real bottom it is not.

### 5b. Stereo baseline

[On the Accuracy Potential in Underwater/Multimedia Photogrammetry (PMC4570311)](https://pmc.ncbi.nlm.nih.gov/articles/PMC4570311/)
gives the cleanest controlled measurement of what water costs a calibrated camera pair:

| Condition | External precision (X/Y/Z) |
|---|---|
| In air (air-glass-air) | 0.013 / 0.011 / 0.024 mm |
| In water, **no** recalibration | 0.031 / 0.072 / 0.153 mm (**≈5× worse**) |
| In water, **with** camera self-calibration | 0.021 / 0.034 / 0.044 mm (**≈2× worse**) |

Under favourable conditions (shallow, clear, uniform temperature, zero salinity)
"a degradation of the geometric precision still amounts to approximately **a factor two**".
Causes named: refraction shrinking the effective viewing angle and stereo intersection
angles; interface planarity; **dispersion — water's refractive index varies 1.4 % across the
visible spectrum vs 0.008 % in air**; temperature/salinity gradients; a non-planar best-focus
plane.

**This is the strongest independent confirmation of our own 46.7° vs 63.8° measurement**: not
recalibrating in water costs ~5×; recalibrating in water recovers most of it but never all.
Two lessons: (i) our in-water calibration was not optional, and (ii) a stereo pair is a
genuine metric-scale source, at ~2× in-air precision, and it removes the known-height
requirement. Cost: a second synchronised camera and a rigid baseline. Hailo-8 has a
[`stereonet` model, Hailo-8 only](https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8/HAILO8_stereo_depth_estimation.rst).

### 5c. Laser scalers / laser line / structured light

[Automatic scale estimation of SfM-based 3D models using laser scalers in underwater
scenarios (ISPRS J., Istenič et al.)](https://www.sciencedirect.com/science/article/pii/S0924271619302448) ·
[arXiv 1906.08019](https://arxiv.org/pdf/1906.08019) ·
[Scale Accuracy Evaluation of Image-Based 3D Reconstruction Strategies Using Laser
Photogrammetry (Remote Sensing 11(18):2093)](https://mdpi.com/2072-4292/11/18/2093/htm).
Two variants: a **fully unconstrained** method for arbitrary laser setups, and a **partially
constrained** method assuming parallel beams equidistant from the camera. They report that
"results of simplistic methods are **extremely dependent on the perspective angle of the
camera and the degree of misalignment of the lasers**", and propagate uncertainty by Monte
Carlo. **I could not retrieve the scale-error percentages** — both the MDPI page (403) and
the arXiv PDF (image-only) refused extraction. See the unverified list.

Structured light gets you absolute range directly:
[Monocular underwater measurement of structured light by scanning with vibrating
mirrors](https://www.sciencedirect.com/science/article/abs/pii/S0143816623002671) ·
[High-precision underwater 3D measurement for AUV mapping using binocular multi-line
laser](https://www.sciencedirect.com/science/article/abs/pii/S0263224126010171) ·
[A Laser Line Auto-Scanning System for Underwater 3D Reconstruction
(PMC5038807)](https://pmc.ncbi.nlm.nih.gov/articles/PMC5038807/). These are
**reconstruction** systems, not velocity sensors; they add a laser and a scanning mechanism
to the hull, and the papers name **optical attenuation, refractive distortion and centreline
extraction** as the accuracy limits.

### 5d. Differential pressure / flow speedometer (no camera at all)

[Differential Pressure Sensor Speedometer for AUV Velocity Estimation (IEEE JOE)](https://ieeexplore.ieee.org/document/8704939/) ·
[Water velocity sensor estimating sideslip from Bernoulli's law](https://www.sciencedirect.com/science/article/abs/pii/S0029801822015591)
— reported accuracy **0.2° sideslip and 0.015 m/s sway velocity**. Note that is **0.015 m/s
= 15 mm/s**, against our flow's **0.57 mm/s** noise floor — 26× coarser, but it measures
**water-relative** speed, so it works over any bottom, at any height, in the dark. It is a
different measurement (through-water, not over-ground), which is both its weakness (currents)
and its independence.

### 5e. Known-size objects

The competition case: a gate or prop of published dimensions in frame gives range from
apparent size. I found no paper giving a clean accuracy figure for this underwater; it is
folklore in the competition community rather than literature. Listed here for completeness
and flagged as unverified.

| Method | What it measures | Accuracy | Runtime on edge | What it needs that we lack | Verdict vs our LK flow |
|---|---|---|---|---|---|
| Pressure + flat-floor (ours) | height ⇒ flow scale | 3 cm on 0.72 m (**4.2 %**) | free | — | current baseline; assumes flat known floor |
| Stereo baseline | metric range, per pixel | **≈2× in-air precision** with in-water self-calibration; 5× without | `stereonet` on Hailo-8 | 2nd synced camera, rigid baseline, in-water recalibration | **removes the height requirement**; best-supported option |
| Laser scaler | scale factor for SfM | not retrieved | — | laser pair, calibration | promising, numbers unverified |
| Laser line / structured light | absolute 3D range | sub-mm class in lab rigs | scanning hardware | laser + scanner + calibration | over-engineered for a velocity sensor |
| Differential pressure | **water-relative** speed | 0.015 m/s, 0.2° sideslip | trivial | a differential pressure port in the hull | 26× coarser than our noise floor but **immune to every one of our optical failure modes** |
| DVL (for reference) | ground-relative velocity | **0.01–0.1 % of distance** | — | the sensor itself | 30–300× better than anything here |

---

## 6. Event cameras underwater — the literature is thin, and I will say so

One substantive source exists:
[AquaticVision: Benchmarking Visual SLAM in Underwater Environment with Events and Frames
(arXiv 2505.03448)](https://arxiv.org/abs/2505.03448), by Yifan Peng, Yuze Hong, Ziyang Hong,
Apple Pui-Yi Chui and Junfeng Wu. It is, by its own claim, **the first** underwater dataset
containing **both events and frames** for benchmarking underwater visual positioning, with
**ground truth from a motion capture system**. Its stated motivation is that "event cameras
can help mitigate challenges posed by extremely low light or hazy underwater conditions".
Project page: <https://sites.google.com/view/aquaticvision-lias>.

**I could not extract a single quantitative event-vs-frame comparison from it.** The abstract
carries no numbers; the 8.6 MB PDF did not yield legible result tables through my fetch. So
the honest statement is: *as of September 2026 there is one underwater event-camera
benchmark, published May 2025, and no published odometry accuracy number that I was able to
verify comparing event to frame cameras underwater.* There is adjacent frame+event flow work
([Dense Continuous-Time Optical Flow from Events and Frames, arXiv 2203.13674](https://arxiv.org/pdf/2203.13674))
but it is not underwater.

For us this settles it without further research: an event camera is **new hardware, a new
processing stack, and no published underwater accuracy number**. It is not a candidate this
season.

---

## 7. Terrain / floor-texture navigation in a pool (tiles, lane lines, caustics)

**State it plainly: there is almost no literature on navigating from a *swimming-pool*
floor.** "Terrain-aided navigation" underwater means something else entirely — it means
matching bathymetry from sonar. [A review of terrain aided navigation for underwater
vehicles](https://www.sciencedirect.com/science/article/abs/pii/S0029801823011630) lists the
sensors it considers: **DVL, profiling sonar, forward-looking sonar, side-scan sonar,
altimeters, underwater laser scanners** — no camera-over-tiles case. Pool work that exists is
*docking* and *station-keeping*, not odometry.

What the pool literature does give us:

- [An Improved Localization Method for the Transition between AUV Homing and Docking
  (PMC8038153)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC8038153/) — swimming-pool
  localization and docking missions; improved accuracy and update rate (no metric figure I
  could extract).
- [Fast Underwater Optical Beacon Finding and High Accuracy Visual Ranging Based on Deep
  Learning (PMC9611530)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9611530/) — in a
  self-made lab pool and an anechoic pool: **average relative distance error 1.04 %, average
  detection speed 0.088 s (11.36 FPS)**. That is *ranging to a known beacon*, which is §5e's
  known-size-object case with a number attached — **1.04 %**, better than our 3.4 % scale
  floor, but it requires a cooperative beacon.
- [Development and testing of a navigation solution for AUVs based on stereo
  vision](https://www.sciencedirect.com/science/article/pii/S0029801823011411) — pool trials
  where the AUV was **fixed to an immovable frame**, horizontal velocities recorded and
  compared against a **DVL** as reference. This is precisely the bench protocol we used with
  the 30 cm slides, and it is the published-methods answer to "how do you validate a camera
  velocity sensor without going to sea".

### Caustics — our measured failure mode, and what the field does about it

The literature agrees caustics are a first-order problem and that the fix is a **learned
removal network**, not a filter:

- [DeepCaustics: Classification and Removal of Caustics From Underwater
  Imagery](https://www.researchgate.net/publication/325919642_DeepCaustics_Classification_and_Removal_of_Caustics_From_Underwater_Imagery)
  — two CNNs (SalienceNet → saliency map of caustics; DeepCaustics → caustic-free image).
- [Self-Supervised Underwater Caustics Removal and Descattering via Deep Monocular SLAM
  (ECCV 2024)](https://josauder.github.io/backscatternet_causticsnet/) — CausticsNet +
  BackscatterNet, **trained with no ground-truth colour images and no caustics labels**,
  supervised by monocular SLAM on ordinary underwater video. Reports better downstream
  **SfM keypoint matching** than a wide range of methods, evaluated on the **R-CAUSTICS**
  benchmark (**712 image pairs/triplets across 7 scenes**) and on COLMAP reconstruction.
  ⚠ The same source notes that networks trained on R-CAUSTICS **transfer poorly to novel
  field data** — which is why the self-supervised formulation matters.
- [RecGS: Removing Water Caustic with Recurrent Gaussian Splatting (arXiv 2407.10318)](https://arxiv.org/html/2407.10318)
  — decomposes caustics by building 3DGS recurrently with a low-pass filter each iteration.
  Offline, not a real-time option.

**Verdict:** the self-supervised CausticsNet formulation is the only one whose training data
we could actually produce (our own pool video, no labels). But the honest reading of our own
17-configuration preprocessing result is that **image preprocessing has never helped us**,
and caustics removal is image preprocessing. Any trial must be gated on a measured
before/after on tracked-feature survival, not on how the images look.

---

## 8. Fusion and handover when the camera fails

The strongest published answer is not underwater at all:
[SUPER ODOMETRY 2.0 / Resilient odometry via hierarchical adaptation, Science Robotics 10,
eadv1818 (arXiv 2608.25427)](https://arxiv.org/html/2608.25427), Shibo Zhao et al., CMU.

Numbers:

- **Endpoint drift 20 cm over 2 966 m = 0.006 % drift rate** (campus test).
- **Largest ATE 0.184 m over a 600 s run with sensor dropouts.**
- SubT-MRS dataset: **average ATE 0.271 m, 54 % better than the second-best method**;
  position robustness 0.925, rotation robustness 0.940.
- Validated across **200 km and 800 operational hours** on aerial, wheeled and legged robots.
- Its **learning-based inertial odometry** — the last-resort fallback — was trained on
  **>100 hours** of heterogeneous platform data, gives **35.5 % average ATE improvement**
  over platform-specialised models and **41 %** on time-relative trajectory error, and
  **adapts to a new platform from 60 seconds of IMU data** (45 s sufficed for a smoke
  environment).

The architecture is the transferable idea, and it is a four-level ladder, not a switch:
adaptive **feature** selection (visual degradation) → adaptive **state-direction** selection
(geometric degradation) → adaptive **engine/factor** selection (mixed) → **learning-based IMU
odometry** (total degradation). *The IMU is promoted to a peer of the camera, not a
stopgap.*

Underwater, [DeepVL (arXiv 2502.07726)](https://arxiv.org/html/2502.07726) is the same idea
with the underwater twist that thruster commands and battery voltage substitute for the
missing exteroception: **3.9 % relative position error through full visual blackout**, on
**100–300 m** trajectories. That is the closest thing in the literature to a published
"how long can it dead-reckon inside X m of error" for a DVL-free AUV: **≈4 m of error per
100 m travelled, camera off.**

Reference ceiling, again: a **DVL-INS is 0.01–0.1 % of distance travelled**
([Nortek](https://www.nortekgroup.com/assets/documents/Validation-of-a-New-Generation-DVL-for-Underwater-Vehicle-Navigation.pdf)).

| Method | What it measures | Accuracy | Runtime on edge | What it needs that we lack | Verdict vs our LK flow |
|---|---|---|---|---|---|
| Super Odometry 2.0 | full odometry w/ degradation ladder | 0.006 % drift / 2 966 m; ATE 0.184 m over 600 s with dropouts | **not published** | LiDAR; a 100 h multi-platform IMU corpus | the *architecture* transfers; the system does not |
| DeepVL | body velocity from IMU+thrust | 3.9 % blackout; 2.2 % w/ 22 features | **<5 ms on Orin AGX**, 28 k params | hours of DVL/mocap-labelled velocity truth | **the right fallback shape for us** |
| DVL-INS (reference) | ground velocity | 0.01–0.1 % of distance | — | the DVL | the bar nobody here reaches |

---

## 9. Failure detection for VO — how published systems distrust themselves

Four mechanisms appear repeatedly, in rough order of sophistication:

1. **Raw feature count / bias threshold.** [LVI-SAM (arXiv 2104.10831)](https://arxiv.org/pdf/2104.10831)
   declares visual failure "when the number of tracked features falls below a threshold or
   when estimated IMU bias exceeds a threshold"; on failure the visual subsystem
   **re-initialises and informs the lidar-inertial system**. Cheap, and the same check we
   can run on our Shi-Tomasi corner count.
2. **Structural degeneracy, not counts.** The same literature warns that features can be
   *abundant but geometrically degenerate* — "a corridor textured with a repetitive coplanar
   pattern" — so a **structural degeneracy measure is needed to replace raw-count proxies**.

   ⚠ **This warning does *not* straightforwardly apply to us, and it is worth being precise
   about why**, because a tiled pool floor is superficially the textbook degenerate case.
   The warning is about **pose/structure estimation from repetitive texture**: self-similar
   geometry admits wrong data association across widely separated views, and a wrong
   association yields a confident wrong pose. We do not do that. We estimate a **2-DOF
   frame-to-frame translation between consecutive frames 12.34 ms apart**, where the
   inter-frame displacement is far below the tile pitch and association is locally
   unambiguous. That is why our bench data — **1.09 cm worst error over pool slides**, with
   tiles in frame — survives a warning that would sink a SLAM frontend. The degeneracy
   measure becomes relevant to us only if we ever add loop closure or map-relative
   relocalisation over the same floor.
3. **Covariance / Hessian-based confidence with hysteresis.**
   [Super Odometry 2.0](https://arxiv.org/html/2608.25427) assesses visual quality from the
   **Hessian matrix of KLT tracking** and the **SE(2) covariance of intensity residuals**;
   LiDAR quality from **PCA eigenvalue linearity/planarity/scatter weights normalised to
   [0,1]**; and disables a modality when its **contribution to the joint state falls below
   10 % and stays low for 2–4 seconds**. The hysteresis matters as much as the threshold —
   it is what stops a system from chattering between sources.
   *Note the coincidence: their visual-health metric is the **Hessian of KLT**, which is the
   same matrix Shi-Tomasi already computes for corner selection. We are one eigenvalue away
   from a published health metric.*
4. **Learned uncertainty / ensembles.** [DeepVL](https://arxiv.org/html/2502.07726) outputs a
   **3D velocity and its uncertainty**, using an **ensemble of 8 networks** so the estimator
   downstream has a variance to gate on rather than a point estimate.
5. **A second, independent short-horizon estimator as the integrity check.**
   [Pedestrian Dead Reckoning-Assisted VIO Integrity Monitoring (PMC6960658)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6960658/)
   uses "the short-time reliability of PDR to aid visual integrity monitoring" and
   automatically switches between VIO and PDR outputs. The underwater analogue of PDR is
   exactly DeepVL's thrust-plus-IMU model.

| Mechanism | What it catches | Cost | What it needs that we lack | Verdict for us |
|---|---|---|---|---|
| Feature-count / bias threshold | bare floor, blackout | ~0 | — | **already available; catches our textureless case** |
| Structural degeneracy measure | repetitive coplanar texture | small | a defined metric | **not needed for frame-to-frame velocity** (12.34 ms spacing ≪ tile pitch); needed only if we add loop closure |
| KLT Hessian + residual covariance, 10 % for 2–4 s hysteresis | soft, gradual degradation | small | — | **closest published match to our pipeline** |
| Ensemble uncertainty | model-domain novelty | 8× inference | training data | only if we adopt a learned velocity model |
| Independent 2nd estimator (PDR/DeepVL analogue) | *silent wrongness* — the plausible-number failure | a whole model | labelled velocity data | the only mechanism that catches a confident wrong answer |

⚠ The last row is the one our own rules care about. Every mechanism above except #5 detects
*absence* of signal. Our measured caustics failure is **presence of a confident wrong
signal** — caustics tracked as motion. None of the count- or covariance-based checks catch
that, because the tracking is healthy; it is tracking the wrong thing. Only an independent
estimator disagreeing does.

---

## Master comparison

| Method | What it measures | Accuracy | Runtime on edge | What it needs that we lack | Verdict vs our LK flow |
|---|---|---|---|---|---|
| **Our sparse LK + RANSAC** | body velocity | 1.09 cm / 30 cm; **3.4 % scale-limited drift floor**; 0.57 mm/s | **12.34 ms**, Pi 5 CPU | — | baseline |
| DIVO | full DVL-inertial-visual odometry | ATE 0.233–0.530 m on quarry seqs | **8 fps on i9 + RTX 2000 Ada** | a DVL, a dGPU | not portable |
| ORB-SLAM3 underwater | monocular SLAM | tracked **1.426 m of 16.212 m** (AQUALOC seq 6) | — | — | **fails outright** |
| DeepVL | velocity from IMU + thrust | **3.9 %** blackout, 2.2 % w/ VIO | **<5 ms** Orin AGX, 28 k params | ~4 h labelled velocity truth | **complement: same accuracy class, no camera** |
| NeuFlow v2 | dense flow | KITTI-15 EPE 4.33 | ≤50 ms, Orin Nano GPU | CUDA GPU | loses on cost |
| RAFT / RAFT-small | dense flow | SOTA EPE | ~100 ms on 1080Ti; <0.2 inf/s mobile | GPU | loses badly |
| UniDepth V2 ViT-L | metric depth | **AbsRel 0.093–0.116, δ₁ 0.91–0.94 (FLSea)** | **no Hailo-8 number found** | Hailo compile; in-water intrinsics | **could remove the height requirement — unproven on our chip** |
| Metric3D v2 / Depth Pro | metric depth | **AbsRel 1.53 / 3.22 in water** | Depth Pro 0.3 s on V100 | datacentre GPU | **rejected on evidence** |
| `scdepthv3` (Hailo zoo) | relative depth | RMSE 0.48 (in air) | **929 FPS = 1.08 ms on Hailo-8** | metric scale; water validation | fast, but relative |
| Stereo pair | metric range | **≈2× in-air precision** in water w/ self-calibration | `stereonet` on Hailo-8 | 2nd synced camera + rigid baseline | **best-evidenced way off the known-height dependency** |
| Laser scaler / structured light | absolute scale / 3D | numbers not retrieved | scanning hardware | lasers, calibration | over-engineered for velocity |
| Differential-pressure speedometer | **water-relative** speed | 0.015 m/s, 0.2° sideslip | trivial | a pressure port | 26× coarser, but immune to all our optical failures |
| Known beacon ranging (pool) | range to known object | **1.04 % distance error**, 0.088 s/detection | 11.36 FPS | a cooperative beacon | good where a prop is in frame |
| Event camera | — | **no verified underwater number** | — | the camera, a whole stack | not a candidate |
| DVL (reference) | ground velocity | **0.01–0.1 % of distance** | — | the sensor | the bar |

---

## Claims I could NOT verify

Listed so nothing above is read as measured when it is not.

1. **Laser-scaler scale-error percentages.** [Istenič et al., ISPRS J.](https://www.sciencedirect.com/science/article/pii/S0924271619302448)
   and [Remote Sensing 11(18):2093](https://mdpi.com/2072-4292/11/18/2093/htm) — MDPI
   returned **HTTP 403**, the [arXiv PDF](https://arxiv.org/pdf/1906.08019) is image-only.
   I have the method description but **no error number**.
2. **Event-vs-frame odometry accuracy underwater.** [AquaticVision (arXiv 2505.03448)](https://arxiv.org/abs/2505.03448)
   exists and has mocap ground truth, but I extracted **no ATE, no success rate, no event
   camera model or resolution**. Its abstract makes no numerical comparison.
3. **NeuFlow v2 parameter count and GFLOPs.** The paper's own comparison table "does not
   include parameter counts or GFLOPs for any method". The "10×–70× speedup" is the paper's
   claim; I have only the **>20 FPS @ 512×384 on Orin Nano** figure as a hard number, and I
   did **not** confirm a per-frame millisecond figure.
4. **Any optical-flow model compiled for Hailo-8.** I found depth (`fast_depth`,
   `scdepthv3`) and stereo (`stereonet`) in the
   [Hailo-8 model zoo](https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8/HAILO8_depth_estimation.rst)
   but **no optical-flow model**. Absence of evidence here, but I searched for it directly.
5. **Whether DeepVL's 28 k-parameter GRU runs in real time on a Pi 5 CPU.** The paper
   measures **<5 ms on an Orin AGX**. A Pi 5 number does not exist in the source; my "28 k
   params is small enough" remark is an inference, not a measurement.
6. **Underwater-tuned intrinsics inside metric depth models.** I could not find a paper that
   quantifies how much a wrong (in-air) focal length biases metric depth underwater. Our own
   **46.7° vs 63.8°** measurement makes the mechanism certain; the magnitude is unverified.
7. **Super Odometry 2.0's compute hardware and runtime.** "Not specified" in the paper — so
   its 0.006 % drift carries **no edge-feasibility claim** whatever.
8. **Maximum IMU-only dead-reckoning duration for Super Odometry 2.0.** The paper gives no
   steady-state IMU-only drift rate or maximum duration.
9. **Known-size-object range accuracy for competition props.** No paper found. The closest
   number is the **1.04 %** beacon-ranging figure, which uses an *active cooperative beacon*,
   not a passive prop.
10. **DIVO ablation without the DVL.** I did not find a DVL-free variant of DIVO reported, so
    its ATE numbers cannot be read as achievable on our sensor set.
11. **Caustics-removal effect on optical-flow velocity error.** Every caustics paper
    evaluates **SfM keypoint matching / COLMAP reconstruction**, not velocity-sensor error.
    No one has published what caustics removal does to a flow-based velocity estimate.
12. **arXiv 2110.00661 (deep dead-reckoning with limited sensor payloads)** — found and
    linked, numbers not extracted; it is listed as a family member, not as evidence.
13. **AQUALOC ATE RMSE tables.** I have the tracked-length failure figures (1.426/16.212 m,
    4.665/8.041 m) from [arXiv 2407.13159](https://arxiv.org/pdf/2407.13159), but **not**
    the ATE RMSE values themselves.

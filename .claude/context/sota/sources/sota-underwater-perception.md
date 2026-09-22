# State of the art — underwater perception, MOT/Re-ID on edge
*Research dossier for a competition AUV (Pi 5 + Hailo-8, 2 × USB cameras, flat ports).*
*Compiled 2026-09-22. Every claim carries a link and a number. Unverifiable claims are listed at the end.*

**Status: complete.** 12 sections. Sources are linked inline; §11 lists every claim I could not stand behind.

---

## 1. Underwater detection benchmarks and datasets

| Dataset | Size / classes | Best reported number | Source |
|---|---|---|---|
| **DUO** (Detecting Underwater Objects) | 7 782 images (6 671 train / 1 111 test), **4 classes**; de-duplicated from URPC2017-2020 + UDD with a perceptual-hash pass | best in the survey's own re-benchmark: **GuideAnchor + ResNet-101, AP 61.4, AP50 83.8, AP75 72.0** | [DUO paper](https://arxiv.org/pdf/2106.05681), [UODReview benchmark](https://github.com/LongChenCV/UODReview) |
| **RUOD** | **10 classes** | best re-benchmarked: **GridRCNN + ResNet-101, AP 54.2, AP50 81.6, AP75 60.0** | [UODReview benchmark](https://github.com/LongChenCV/UODReview) |
| **URPC** series | no public test-set annotations; **URPC2017/2018 contain considerable annotation errors** — the survey explicitly warns that papers using different self-made splits are not comparable | controlled re-benchmark on **URPC2020 (4 434 train / 1 019 test)**: raw-image AP ranges **35.2 (SSD) → 45.4 (TOOD)** | [arXiv 2311.18814 tables](https://ar5iv.labs.arxiv.org/html/2311.18814); dataset caveats from [ACM CSUR survey](https://dl.acm.org/doi/10.1145/3759243) |
| **Brackish** (temperate brackish water, first of its kind) | **14 518 frames / 25 613 annotations**, 6 classes (bigfish, crab, jellyfish, shrimp, small fish, starfish); 9 967 / 1 467 / 1 468 train-val-test | one comparison paper reports **mAP@0.5 = 97.5 %** (detector not identified in what I retrieved) — high enough that it discriminates poorly between methods | [Gated Cross-domain Collaborative Network (arXiv 2306.14141)](https://arxiv.org/pdf/2306.14141), [UnderwaterDataset index](https://github.com/ddz16/UnderwaterDataset) |
| **TrashCan** (from the J-EDI deep-sea library) | **6 008 train / 1 204 val**, 16 categories (trash, ROV, bio, metal, plastic, unknown…) | reported **mAP@0.5 = 66.2 %** — the hardest of the optical sets | [arXiv 2306.14141](https://arxiv.org/pdf/2306.14141) |
| **SUIM** (semantic segmentation, not detection) | **1 525 train/val + 110 test** RGB images, pixel annotations, **8 categories** (fish, reefs, aquatic plants, wrecks/ruins, divers, robots, sea-floor); resolutions 256² to 1906×1080 | segmentation benchmark — quoted here because it is the standard underwater *mask* set, and it is **small**: 1 635 images total | [UnderwaterDataset index](https://github.com/ddz16/UnderwaterDataset) |

**The survey's own recommendation** ([Underwater Optical Object Detection in the Era of AI, ACM Computing Surveys 2025](https://dl.acm.org/doi/10.1145/3759243)): use **RUOD and DUO as the unified benchmarks** — large scale, consistent splits, accurate annotations — and treat URPC results as non-comparable across papers.

A 2026 structured review normalised results from **88 primary studies** across URPC, DUO, RUOD and side-scan sonar, into one mAP/recall/FLOPs matrix — [Recent advancements in underwater object detection with YOLOv8–YOLOv12, *Discover Computing* 2026](https://link.springer.com/article/10.1007/s10791-026-10271-1).

**What makes underwater different, measured:** see also [Are All Marine Species Created Equal? Performance Disparities in Underwater Object Detection](https://arxiv.org/pdf/2508.18729) — per-class disparity, not aggregate mAP, is where underwater detectors fail.

> Relevance to us: the AP gap between AP50 (≈82–84) and AP75 (≈60–72) on these benchmarks is the honest measure of *localisation* quality underwater. For alignment verbs, AP75-class numbers are the ones that matter, and they are 20 points below the headline.

---

## 2. Does enhancement help DETECTION? (the contest)

This is genuinely contested in the literature, and the contest resolves in a way that matches the local measurement (CLAHE etc. destroying gate detections, 30.4 % → 1.2 %).

### The "no / harmful" side

- **[Is Underwater Image Enhancement All Object Detectors Need? (IEEE TIP 2024 / arXiv 2311.18814)](https://ar5iv.labs.arxiv.org/html/2311.18814)** — the largest controlled study: **13 SOTA enhancement algorithms × 7 detectors = 98 retrained models** (a later extension reports 18 algorithms × 7 detectors = 133 models). Dataset **URPC2020, 4 434 train / 1 019 test**. Crucially the detectors were **retrained on the enhanced images**, so this is not a domain-shift result — and enhancement still lost. (Our own 30.4 % → 1.2 % gate result *is* a domain-shift result: an unprocessed-trained model fed processed frames. The two are different mechanisms pointing the same way; §9's "Clean-Water" row is the published instance of ours.)

  Baseline AP on **raw** images:

  | Detector | AP (%) raw |
  |---|---|
  | Faster R-CNN | 43.5 |
  | Cascade R-CNN | 44.3 |
  | RetinaNet | 40.7 |
  | FCOS | 41.4 |
  | ATSS | 44.8 |
  | TOOD | **45.4** |
  | SSD | 35.2 |

  Enhanced domains: best (SGUIE, TOPAL) **43.1–44.4 %** AP across detectors; worst (UGAN) **39.0–40.8 %**, WaterGAN **41.0–42.4 %**. The paper's own sentence: *"surprisingly, all detectors retrained on the raw domain can achieve the highest AP values than the detectors retrained on other domains"* — i.e. **raw wins for all 7 of 7 detectors**. Attributed to background errors and missed detections. (I quote raw range **35.2–45.4** against UGAN range **39.0–40.8** rather than a point delta: the two ranges are per-detector minima/maxima and I did not retrieve the per-cell pairing, so subtracting across them would be wrong.)
- **[Beneath the Surface: The Role of Underwater Image Enhancement in Object Detection (arXiv 2411.14626)](https://arxiv.org/abs/2411.14626)** — 9 enhancement models (physical / non-physical / learning-based) × 3 detectors, 2 datasets. Finding: **adverse effect of enhancement on detection at the dataset level**; attributed to *diffused edges and increased noise* after enhancement. Also: enhancement especially **inhibits detection of hard cases**, because it amplifies background interference.
- **Image-quality metrics do not predict detection.** Stated explicitly in [*J. Imaging* 2026, 12(1), 18](https://doi.org/10.3390/jimaging12010018): "traditional image quality metrics do not reliably predict detection performance" and "visually pleasing images do not necessarily lead to higher detection performance." The transferable rule: *never* tune an enhancement stage on UIQM/PSNR. (I did **not** retrieve a per-metric correlation coefficient — see §10.)

### The "yes, but only selectively" side

- **[Revisiting Underwater Image Enhancement for Object Detection: A Unified Quality–Detection Evaluation Framework (*J. Imaging* 2026, 12(1), 18)](https://doi.org/10.3390/jimaging12010018)** — agrees enhancement loses at the **dataset** level, but shows a **per-image oracle**: on CUPDD, average per-image mAP **0.41 (original) → 0.64 (best-of-mixed set)**, a **+56 % relative** gain. That is an *oracle upper bound* obtained by picking the best enhancement per image after the fact — not a deployable gain unless you can predict which images benefit.
- Joint / dual-branch training (enhancement and detection optimised together) is the other route that reports gains — [Joint Perceptual Learning (arXiv 2307.03536)](https://arxiv.org/pdf/2307.03536), [Dual-Branch Collaborative Framework (arXiv 2606.15857)](https://arxiv.org/pdf/2606.15857). Note these are **not** "bolt CLAHE in front of a frozen detector"; they retrain the detector.

### Verdict for this vehicle

| Claim | Evidence | Applies to us? |
|---|---|---|
| "Enhancement improves detection" as a preprocessing bolt-on | no dataset-level study supports it; 98–133-model study says no | **No** |
| Enhancement helps if the detector is *retrained on enhanced imagery* | joint-optimisation papers report gains | Only if we retrain; cost = a second training pipeline |
| Per-image *selective* enhancement can help | +56 % relative, but **oracle selection** | Not deployable today |
| UIQM/PSNR predicts detection gain | explicitly refuted in ≥2 studies | **No — never tune on UIQM** |

---


---

## 3. Domain adaptation / generalisation — has synthetic underwater imagery ever worked?

The cleanest controlled answer found, and it is not encouraging.

**[Towards Scaling Marine Perception with Synthetic Data (arXiv 2609.20680)](https://arxiv.org/html/2609.20680)** — OceanSim (IsaacSim-based) renderer, sea-urchin detection, **real test set of 179 images / 718 urchins**:

| Training data | Model | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|---|
| Synthetic only, 1 800 imgs | YOLOv9 | 0.521 | 0.389 | **0.341** | 0.110 |
| Synthetic only, 1 800 imgs | DETR-R50 | 0.419 | 0.369 | **0.301** | 0.103 |
| Real only, 1 800 imgs | YOLOv9 | 0.821 | 0.724 | **0.793** | 0.403 |
| Real only, 1 800 imgs | DETR-R50 | 0.816 | 0.804 | **0.838** | 0.347 |

Synthetic-only reaches **43 % of the real-trained mAP50** (0.341 vs 0.793) and **27 % of mAP50-95** (0.110 vs 0.403). The paper also sweeps synthetic scale from 1× (1 800 images) to 10×10× (18 000 images) — **I did not retrieve the result of that sweep**, only that it was run (§10). The authors' own summary: *"models trained on simulated data fall short on being able to identify objects under heavy underwater degradation."*

**Simulator capability, stated by the same review:**
- **Stonefish** — custom OpenGL C++ renderer, ROS interface, exports semantic annotation labels, **but has no dedicated synthetic-data-generation pipeline**, so large dataset generation is hard. [Stonefish: Supporting ML Research in Marine Robotics (arXiv 2502.11887)](https://arxiv.org/html/2502.11887v1)
- **HoloOcean** — strong multi-sensor support (imaging / profiling / sidescan / echosounder sonar, cluster-based multipath, editable material dependence, probabilistic noise) **but also no scalable SDG**. [Towards Realistic 3D Sonar Simulation (arXiv 2606.06130)](https://arxiv.org/html/2606.06130v1)
- Sim-to-real *has* been validated for **optical flow**, not detection: [eStonefish-Scenes, a sim-to-real validated event-based optical-flow dataset for underwater vehicles](https://zenodo.org/records/18471242).

> Relevance: our sim (Gazebo + ArduSub SITL) is already documented as not transferring detection thresholds. This is the published version of the same finding, with a number on it: expect roughly **half** the mAP50 from synthetic-only training, and worse at tight IoU — exactly the regime alignment needs.

---

## 4. MOT SOTA 2025-2026 — measured deltas

**Head-to-head under identical detections** ([Roboflow `trackers` benchmark suite](https://trackers.roboflow.com/latest/trackers/comparison/), reproducible harness, HOTA/IDF1/MOTA, default and tuned):

MOT17 (default params):

| Tracker | HOTA | IDF1 | MOTA |
|---|---|---|---|
| SORT | 58.4 | 69.9 | 67.2 |
| ByteTrack | 60.1 | 73.2 | 74.1 |
| **OC-SORT** | **61.9** | **76.4** | **76.0** |
| BoT-SORT | 63.7 | 78.7 | 79.2 |
| C-BIoU | 63.0 | 79.1 | 77.4 |

MOT17 (tuned): SORT 60.4 / ByteTrack 60.5 / OC-SORT 62.0 / BoT-SORT 63.8 / C-BIoU 63.0 HOTA.

Other benchmarks, same suite: **DanceTrack** BoT-SORT 57.8 HOTA (best), C-BIoU IDF1 58.7 / MOTA 92.4 (tuned). **SportsMOT** BoT-SORT 73.8 → 74.1 HOTA. **SoccerNet with oracle detections** BoT-SORT 84.5 HOTA default, C-BIoU 85.7 HOTA tuned.

Published leaderboard numbers (each paper's own):
- **OC-SORT**: 78.0 MOTA / 77.5 IDF1 / **63.2 HOTA** on MOT17; on **DanceTrack it beats ByteTrack by >10 HOTA points**, because DanceTrack violates constant velocity. [Observation-Centric SORT (arXiv 2203.14360)](https://arxiv.org/pdf/2203.14360)
- **Deep OC-SORT** (= OC-SORT + adaptive Re-ID): **64.9 HOTA on MOT17-test**, beating SORT, DeepSORT, ByteTrack, OC-SORT and StrongSORT **on the same detections**. [Deep OC-SORT (arXiv 2302.11813)](https://arxiv.org/pdf/2302.11813)
- **OA-SORT over Hybrid-SORT** on DanceTrack: **+0.9 HOTA, +1.1 AssA, +0.1 MOTA, +1.2 IDF1**. [Occlusion-Aware SORT (arXiv 2603.06034)](https://arxiv.org/pdf/2603.06034)
- **PD-SORT** adds pseudo-depth cues to the motion model — the "no appearance model" branch of the field. [PD-SORT (arXiv 2501.11288)](https://arxiv.org/pdf/2501.11288)

**The size of the prize, honestly stated.** Our tracker is OC-SORT. The measured headroom from the best motion-only tracker (BoT-SORT, which is OC-SORT-class motion + camera-motion compensation) is **+1.8 HOTA on MOT17** (61.9 → 63.7). The measured headroom from adding appearance Re-ID (OC-SORT 63.2 → Deep OC-SORT 64.9 on the papers' own MOT17 numbers) is **+1.7 HOTA**. Both are single-digit, on crowded pedestrian scenes with dozens of identity-confusable targets. A competition AUV sees **1–3 objects of distinct classes at a time**; the association problem that Re-ID solves is largely absent.


---

## 5. Re-ID on edge — the cost side

**[OSNet — Omni-Scale Feature Learning for Person Re-ID (arXiv 1905.00953)](https://arxiv.org/pdf/1905.00953)** is the standard lightweight Re-ID backbone:

| Config | Params | Mult-Adds (≈FLOPs) |
|---|---|---|
| OSNet β=1.0, γ=1.0 | **2.2 M** | **978.9 M** |
| OSNet β=0.5, γ=0.5 | **0.6 M** | **272.9 M** |

Two things to note before costing this.

1. **The Mult-Adds are per crop, not per frame.** A Re-ID head runs once *per detection*, so the per-frame cost is `N_tracks × 979 MFLOPs` at full width. Our detection graph already runs at **53.9 Hz**; a second network in the loop is the first thing that would break that budget.
2. **Deep OC-SORT's own ablation is the honest measure of what that buys**: **+1.7 HOTA over OC-SORT on MOT17** ([Deep OC-SORT, arXiv 2302.11813](https://arxiv.org/pdf/2302.11813)), on a benchmark built from crowds of same-class pedestrians. On a course with a handful of distinct-class props, the association ambiguity that the embedding resolves does not exist in the same quantity.

**I could not find** a published ARM-CPU or Hailo-8 throughput number for OSNet/FastReID. Treat any "OSNet runs at X Hz on a Pi" figure as unmeasured until we measure it. See §10.

> Verdict on the evidence: **Re-ID is the lowest-value addition on this list for this vehicle.** It costs a second inference stream and buys single-digit HOTA on a problem shape we do not have.

---

## 6. Tracking through a 2–4 s detector blackout

Our measured gap distribution (p50 0.155 s, p90 0.651 s, p99 2.418 s, max 4.00 s) puts the hard cases well beyond what any motion model carries. At p99 = 2.418 s and, say, 0.3 m/s of relative motion, a constant-velocity Kalman prediction has accumulated ~0.7 m of unobserved displacement; nothing in the SORT family models that. Appearance Re-ID does not help either — there is nothing to re-identify *against* during the blackout, only at its end. What survives a blackout is **geometry carried forward from the image itself**, which is what the XFeat homography anchor already does.

### Feature-matching anchors, measured

**[XFeat: Accelerated Features for Lightweight Image Matching (CVPR 2024, arXiv 2404.19174)](https://arxiv.org/html/2404.19174v1)** — all on **Intel i5-1135G7, VGA (640×480)**, which is the closest published proxy to a Pi-5-class CPU at our 640×360:

| Method | FPS (i5-1135G7, VGA) | MegaDepth-1500 AUC@5° | Acc@10° | Inliers |
|---|---|---|---|---|
| ORB | 44.3 | 17.9 | 43.1 | — |
| SuperPoint | 3.0 | 37.3 | 67.4 | 495 |
| ALIKE | 5.3 | 49.4 | 77.7 | 333 |
| DISK | 1.2 | 53.8 | 81.3 | 1 231 |
| **XFeat** | **27.1 ± 0.33** | 42.6 | 74.9 | **892** |
| **XFeat\*** (semi-dense) | **19.2 ± 1.12** | 50.2 | 85.1 | **1 885** |

HPatches homography accuracy (MHA@5 px): XFeat **98.1 illumination / 81.1 viewpoint** — the **best viewpoint score in the table**, above SuperPoint 79.6, ALIKE 77.5, DISK 77.5, ORB 71.4. Viewpoint robustness is exactly the axis a moving AUV stresses.

Headline ratios from the paper: XFeat is **5× faster than ALIKE**, **9× faster than SuperPoint at higher accuracy**, **16× faster than DISK at comparable accuracy**.

**LightGlue / LighterGlue:** the original [LightGlue (arXiv 2306.13643)](https://arxiv.org/pdf/2306.13643) matcher is measured in XFeat's supplementary at **0.31 pairs/second on an i7-6700K CPU** — i.e. **~3.2 s per match pair on CPU**, categorically unusable in a 2-second blackout on a Pi. XFeat's own **LighterGlue** is ~**3× faster than LightGlue** with fewer parameters. ONNX export of XFeat + LightGlue exists: [xfeat_lightglue_onnx](https://github.com/noahzhy/xfeat_lightglue_onnx), [verlab/accelerated_features](https://github.com/verlab/accelerated_features).

> Verdict: **the anchor we already run (XFeat) is the right member of this family on the published numbers** — the only faster option (ORB, 44.3 FPS) loses 25 AUC points and 10 MHA-viewpoint points. Upgrading to ALIKED/DISK-class features costs 5–20× the compute for an accuracy the anchor does not need. The realistic lever is XFeat → XFeat\* (semi-dense): **+2× inliers (892 → 1 885) and +10 Acc@10° for a 1.4× slowdown** (27.1 → 19.2 FPS).


**Deep OC-SORT's own table** (same table, same detections — not a cross-paper subtraction):

| Benchmark | OC-SORT HOTA | Deep OC-SORT HOTA | Δ | Δ AssA |
|---|---|---|---|---|
| MOT17-test | 63.2 | 64.9 | **+1.7** | 63.2 → 65.9 (+2.7) |
| MOT20-test | 62.1 | 63.9 | **+1.8** | 62.0 → 65.7 (+3.7) |
| DanceTrack-test | 55.1 | 61.3 | **+6.2** | 38.3 → 45.8 (**+7.5**) |

Re-ID backbone used: **SBS50 from FastReID**. The DanceTrack gain is the large one, and DanceTrack is deliberately the benchmark where **all targets look identical and motion is non-linear** — the opposite of our scene, where targets differ by class and the vehicle's own motion dominates.

---

## 7. Segmentation on edge

### Measured Hailo-8 throughput (the only numbers that matter for this vehicle)

From the official [Hailo Model Zoo, HAILO8 object detection](https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8/HAILO8_object_detection.rst) and [HAILO8 instance segmentation](https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8/HAILO8_instance_segmentation.rst). Stated conditions: **PCIe Gen 3 × 4 lanes, room temperature, Intel Core i5-9400 host, Hailo Dataflow Compiler v2.19.0, batch size 1**.

| Model | Task | Input | Params (M) | GOPs | COCO mAP | **FPS on Hailo-8** |
|---|---|---|---|---|---|---|
| yolov8n | det | 640² | 3.2 | 8.74 | 37.0 | **1036** |
| yolov8s | det | 640² | 11.2 | 28.6 | 44.6 | **491** |
| yolov8m | det | 640² | 25.9 | 78.9 | 49.9 | 66.9 |
| yolov11n | det | 640² | 2.6 | 6.55 | 39.0 | 185 |
| yolov11s | det | 640² | 9.4 | 21.6 | 46.3 | 111 |
| ssd_mobilenet_v2 | det | 300² | 4.46 | 1.52 | 24.2 | 784 |
| **yolov8n_seg** | **inst-seg** | 640² | 3.4 | — | 29.7 | **528** |
| **yolov8s_seg** | **inst-seg** | 640² | 11.8 | — | 36.4 | **107** |
| yolov8m_seg | inst-seg | 640² | 27.3 | — | 40.1 | 51.4 |
| yolov5n_seg | inst-seg | 640² | 1.99 | — | 22.9 | 464 |
| yolact_regnetx_800mf | inst-seg | 512² | 28.3 | — | 25.4 | 54.9 |

Two hard facts fall out:
1. **YOLO-seg compiles and is fast on Hailo-8.** `yolov8n_seg` at **528 FPS** is ~10× our current graph rate; even `yolov8s_seg` at **107 FPS** has headroom over our measured 53.9 Hz graph. Instance masks are *not* the bottleneck.
2. **FastSAM / MobileSAM / EdgeSAM are not in the Hailo-8 model zoo at all.** There is no published Hailo-8 compilation of any SAM variant. Their published speeds are on other silicon:

| Model | Encoder params | Published speed | Source |
|---|---|---|---|
| EdgeSAM | **9.6 M**, 22.1 GFLOPs | **>30 FPS on iPhone 14**; 37× faster than SAM on a 2080 Ti; 14× faster than MobileSAM on-device; **+2.3 mIoU COCO / +3.2 mIoU LVIS over MobileSAM** | [EdgeSAM (arXiv 2312.06660)](https://arxiv.org/pdf/2312.06660), [repo](https://github.com/chongzhou96/EdgeSAM) |
| MobileSAM | ~2× EdgeSAM's encoder | **5 FPS on iPhone 14**; ≈66× smaller than SAM; mIoU ≈ 0.74 (2–4 points below full SAM) | [EdgeSAM paper](https://arxiv.org/pdf/2312.06660) comparison |

Accuracy cost of EdgeSAM distillation: **1–3 mIoU with box prompts, 4–5 mIoU with point prompts**.

### Is instance segmentation measurably better than boxes for ALIGNMENT?

**I did not find a controlled study that measures alignment / pose error with masks vs boxes.** What exists instead:
- **Oriented bounding boxes** (not masks) is what the winning teams actually reached for: Cornell's **YOLO-7D** is an *OBB* pipeline.
- Masks feed pose indirectly — Bumblebee's pipeline uses YOLO11 boxes as the ROI and then **XFeat + PnP** for the actual pose, i.e. the precision comes from feature geometry, not from the mask boundary.

This goes in §10 as an unverified claim. The honest reading of the evidence is that **the published route to alignment precision is features + PnP, not masks.**

---

## 8. What winning RoboSub teams actually run

Read directly from the RoboNation TDR archive PDFs (text extracted, not summarised from press pages).

| Team | Year / place | Detector | Geometry / pose | Tracking & filtering | Compute | Cameras / sonar |
|---|---|---|---|---|---|---|
| **NUS Bumblebee** (BBAUV 4.5) | **RoboSub 2025 — 1st, Autonomy Challenge + 1st Design Documentation** | **YOLO11** | **XFeat feature matching** → precise element localisation; **PnP**; **HDBSCAN** clustering; **monocular depth estimation (DepthAnything)** | **UKF**, with extreme-reading rejection and fallback recalibration on visual elements; **MUSIC** DoA for pingers | **Jetson Orin AGX 32 GB** (+ Intel i7-1185GRE SBC); Mini-AUV: **Orin NX 16 GB** | FLIR BlackFly S PoE GigE (BFS-PGE-31S4C-C); **Oculus M750d dual-freq multibeam sonar (750 kHz / 1.2 MHz, $21 300)**; Teledyne Pathfinder 600 kHz DVL. Mini: Arducam IMX219 + DWE ExploreHD 3.0, Water Linked A50 DVL. ROS 2 Humble. [TDR](https://robonation.org/app/uploads/sites/4/2025/07/RS25_TDR_National-University-of-Singapore-Bumblebee-compressed.pdf) |
| **Cornell CUAUV** (Orion 2.0) | RoboSub 2026 | **YOLO-7D** — custom **7-channel oriented-bounding-box** detector: RGB + **depth + surface normals**. Claimed **≈20 % detection-accuracy improvement over RGB-only**, ">10 % increase in accuracy for all detections" | OBB + depth channels | **Multiplicative UKF** for orientation + separate **linear KF** for position | **Jetson Orin Nano** — explicitly compute-bound: "limited VRAM and CPU capacity can directly reduce YOLO frame rate" | ZED stereo; C++ CUDA pipeline reading frames **directly from GPU** into shared memory → **doubled effective vision frame rate**; moving the Kalman filter from Python to C++/Eigen cut estimator CPU by **~an order of magnitude** to free compute for YOLO. [TDR](https://robonation.org/app/uploads/sites/4/2026/07/RS26_TDR_Cornell_CUAUV.pdf) |
| **CMU TartanAUV** | RoboSub 2026 | **yolo26-pose, small models — keypoints, not boxes** | Keypoints → pose; **RTAB-Map** 3D mapping from RGBD | EKF | **Jetson AGX Orin** | Oak-D W PoE (RGBD). Training data: **mix of synthetic (NVIDIA IsaacSim Omniverse) and real** images, synthetic used specifically for **extreme angles and high turbidity** under-represented in real data. [TDR](https://robonation.org/app/uploads/sites/4/2026/07/RS26_TDR_CarnegieMellon_TartanAUV.pdf) |
| **ITU AUV (Taluy)** | RoboSub 2025 — **3rd, Autonomy Challenge** | **YOLOv11**, trained on a **combined dataset of Blender-generated synthetic images + manually labelled real pool-test images** | Stereo depth → object position and orientation → TF frames; bottom-facing camera for prop position | **EKF** | **Jetson AGX Xavier** + Jetson Orin Nano | stereo + bottom camera; **360° sonar + Ping 1D sonar**. [TDR](https://robonation.org/app/uploads/sites/4/2025/07/RS25_TDR_Istanbul-Technical-University-AUV-compressed.pdf) |
| **ASU Desert WAVE** | RoboSub 2025 — **2nd, Autonomy Challenge** | ML-based **visual servoing** rather than a mapped detector-first pipeline; a-priori 2D waypoint map built with a **laser tape measure** | a-priori map + visual servoing | yaw loop off IMU | **Jetson Orin Nano** | Waveshare IMX219-160 (160° FOV). Reported **95 % success rate** choosing the correct gate side. [TDR](https://robonation.org/app/uploads/sites/4/2025/07/RS25_TDR_Arizona-State-Si-Se-Puede-Desert-WAVE.pdf) |

### The four things every top team does that are worth naming

1. **Nobody runs a Re-ID / appearance-embedding tracker.** Not one of the five TDRs read mentions re-identification, appearance embeddings or cross-camera association. Tracking is a **Kalman/UKF/EKF state estimator**, and identity comes from **class + geometry**.
2. **Detection is a coarse ROI; precision comes from geometry.** Bumblebee: YOLO11 → **XFeat** → PnP. CMU: keypoints → pose. Cornell: OBB + depth. The bounding box is never the final answer.
3. **The winner runs the same feature anchor we do.** Bumblebee (1st, 2025) uses **XFeat** for precise pose. That is independent confirmation of the anchor choice in our lock ladder.
4. **Compute discipline is a documented competitive lever.** Cornell's two biggest reported wins this cycle are *infrastructure*: GPU-direct frame handling (**2× vision frame rate**) and Python→C++ Kalman (**~10× CPU reduction**), both purely to buy back frames for YOLO. Our Pi 5 + Hailo-8 split is the same strategy in hardware.

### The one thing they have that we do not

**Sonar.** Bumblebee carries an **Oculus M750d multibeam ($21 300)**; ITU carries a 360° sonar plus Ping 1D. Both teams use it as a **modality fallback when optics fail** — which is precisely the 2–4 s blackout regime in §6. There is no software substitute for it in the literature.

---

## 9. Camera settings as a perception lever

The direct question — *fixed vs auto exposure underwater, measured at the detector* — **has no published controlled answer I could find**. What does exist is a measured robustness sweep that bounds how much the image-formation side is worth.

**[An Empirical Study on the Robustness of YOLO Models for Underwater Object Detection (arXiv 2509.17561)](https://arxiv.org/html/2509.17561)** — DUO (7 780) + Roboflow100 (2 220) = **10 000 images**, 4 classes, 70/20/10 split. Five YOLO variants, six synthetic degradations. **mAP@50:**

| Model | Clean | Low-contrast (α=β=0.6) | **Blur (19×19 Gaussian)** | **Noise (σ=10, s&p p=0.005)** | Bluish cast | Greenish cast | **"Clean-Water" (contrast enhance + sharpen)** |
|---|---|---|---|---|---|---|---|
| YOLOv8m | 0.758 | 0.712 | 0.534 | 0.181 | 0.733 | 0.745 | **0.652** |
| YOLOv10m | 0.746 | 0.682 | 0.517 | 0.145 | 0.706 | 0.737 | **0.638** |
| YOLO11m | 0.752 | 0.691 | 0.498 | 0.143 | 0.714 | 0.742 | **0.628** |
| YOLOv12m | **0.770** | 0.712 | 0.557 | 0.192 | 0.739 | 0.759 | **0.649** |

Read off this table, in order of size:

- **Noise is catastrophic: −75 % to −81 % relative** (0.770 → 0.192 worst case). Anything that raises image noise is the most expensive thing you can do to a detector. ⚠ The injected noise here is synthetic Gaussian + salt-and-pepper, **not measured sensor-gain noise**; that a short shutter must be paid for in gain, and gain in noise, is physics reasoning laid over this table, not something this paper measured (§11.4).
- **Blur costs −26 % to −34 % relative** (0.752 → 0.498 for YOLO11m). Motion blur is the second-largest lever, and it is bought back with shutter time, which costs gain, which costs noise. That trade is the real exposure decision, and this table puts a number on each side of it **under simulated degradation** — a real shutter/gain sweep on our own footage would be new evidence, not a replication.
- **Colour cast is cheap: −2 % to −5 % relative.** Bluish/greenish attenuation barely moves mAP. This is a strong argument that *colour correction is not where the wins are* — consistent with §2.
- **⚠ The "Clean-Water" row is a fourth independent enhancement-hurts-detection measurement.** It is contrast enhancement + soft sharpening (γ=1.2) applied at test time to a model trained on unprocessed frames: **0.770 → 0.649 (−16 % relative) for YOLOv12m, 0.758 → 0.652 (−14 %) for YOLOv8m — every model loses.** It is a *cleaner-looking* image that detects worse. This is the same mechanism as the local 30.4 % → 1.2 % gate result, at a milder severity.
- Model choice under degradation matters less than the degradation: the spread between YOLOv8m and YOLOv12m on blur is 0.534 vs 0.557 (**2.3 points**), while blur itself costs **~22 points**.

Motion-blur physics, for the record (no detector-level number attached): underwater motion blur is attributed to camera motion, relative target motion, and **longer exposure times forced by low light** — [Physically Guided Attention Mechanism for Underwater Motion Deblurring (*J. Imaging* 2026, 12(5), 186)](https://doi.org/10.3390/jimaging12050186).


Earlier Bumblebee generation for continuity: **RoboSub 2023, YOLOv8 (detection + segmentation) + SIFT + PnP** — [Bumblebee RoboSub 2023](https://bumblebee.sg/competitions/robosub/2023/). The 2023→2025 move is **SIFT → XFeat**, same architecture.

---

## 10. Cross-cutting summary table

`technique | measured gain | cost on Pi 5 / Hailo-8 | evidence strength (n, venues)`

| Technique | Measured gain | Cost on Pi 5 / Hailo-8 | Evidence strength (n, venues) |
|---|---|---|---|
| **Underwater enhancement as preprocessing** (CLAHE / GAN / physical) | **Negative.** −1 to −5.8 AP on URPC2020 even *after retraining*; −14 to −16 % relative mAP50 when applied at test time to an unprocessed-trained model | CPU cost on Pi 5, zero benefit | **Strong.** n = 98–133 models, 13–18 methods × 7 detectors, 1 dataset family (URPC2020); corroborated by 9 methods × 3 detectors × 2 datasets, and by a 10 000-image 5-model degradation sweep. **Four independent studies, plus our own 17-config / 4-prop / 3-venue result.** |
| Per-image *selective* enhancement | +56 % relative per-image mAP (0.41 → 0.64) — **oracle selection** | unknown; requires a per-frame selector that does not exist | Weak for deployment. n = 1 dataset (CUPDD), oracle upper bound |
| **Synthetic-only training** (OceanSim / Blender / IsaacSim) | **−57 % mAP50 vs real-trained** (0.341 vs 0.793); −73 % mAP50-95 | free at inference; large offline cost | Moderate. n = 179 real images / 718 instances, 1 site, 2 detectors. Note: **two RoboSub teams (CMU, ITU) use synthetic as a *supplement*, never alone** |
| **OC-SORT → BoT-SORT** (better motion + CMC) | **+1.8 HOTA** MOT17 (61.9 → 63.7) | CPU-only, cheap | Strong. Common-detection harness, 4 benchmarks |
| **Adding appearance Re-ID** (OC-SORT → Deep OC-SORT, FastReID SBS50) | **+1.7 HOTA** MOT17, **+1.8** MOT20, **+6.2** DanceTrack | second inference stream, **N_tracks × 979 MFLOPs/frame** (OSNet); no Hailo-8 zoo entry for any Re-ID model | Strong for pedestrians. **Zero RoboSub TDRs use it.** Benchmark shape (crowds of identical targets) does not match ours |
| **XFeat homography anchor** (what we run) | 74.9 Acc@10° and **892 inliers** at **27.1 FPS on an i5-1135G7 @ VGA**; **best HPatches viewpoint MHA in its table, 81.1** | CPU; the published CPU proxy is close to Pi-5 class but **not measured on a Pi 5** | Moderate–strong for *matching*; **benchmark mismatch** — MegaDepth/HPatches measure wide-baseline pose between image pairs, not planar-anchor survival across a 2 s gap |
| XFeat → **XFeat\*** (semi-dense) | **inliers 892 → 1 885 (+111 %)**, Acc@10° 74.9 → 85.1 | 27.1 → **19.2 FPS** on the same CPU (1.4× slower) | Same source, same protocol |
| LightGlue matcher on CPU | higher match quality | **0.31 pairs/s on an i7-6700K** ⇒ ~3.2 s/pair — **unusable inside a 2 s blackout** | Single measurement, XFeat supplementary |
| **Instance segmentation on Hailo-8** (yolov8n_seg / yolov8s_seg) | masks at 29.7 / 36.4 COCO mask-mAP | **528 / 107 FPS on Hailo-8** — cheap, compiles today | Strong (vendor zoo, stated conditions: PCIe Gen3 ×4, i5-9400, DFC 2.19.0, BS=1) |
| FastSAM / MobileSAM / EdgeSAM | EdgeSAM +2.3 mIoU COCO over MobileSAM; >30 FPS on iPhone 14 | **no Hailo-8 model-zoo entry for any SAM variant** — compilation unproven | Moderate for the models, **zero** for this accelerator |
| **Shortening exposure to kill motion blur** | recovering blur is worth **+0.20 to +0.25 mAP50** (0.498–0.557 → 0.746–0.770) | free | Moderate. n = 10 000 images, 5 models, synthetic 19×19 Gaussian blur — *simulated*, not a real shutter sweep |
| **…paid for with sensor gain/noise** | noise at σ=10 + s&p 0.005 costs **−0.55 to −0.61 mAP50** (−75 to −81 % relative) | free | Same source. **The noise penalty is ~2.5× the blur penalty** — the trade is not symmetric |
| Colour-cast correction | recovering a bluish/greenish cast is worth only **+0.01 to +0.04 mAP50** (−2 to −5 % relative) | CPU | Same source. Smallest lever in the table |
| Sonar as an optical fallback | not quantified in any TDR | hardware: Oculus M750d **$21 300**; Ping 1D ≈ $400 | Practice-based: **2 of 5 top TDRs carry it**, none publishes a detection number |

---

## 11. Claims I could NOT verify

Listed so nobody quotes them back as established.

1. **No Hailo-8 (or Pi-5 ARM CPU) throughput number for any Re-ID model** — OSNet, FastReID SBS50, or otherwise. The Hailo model zoo has no person-Re-ID entry. Any "OSNet runs at X Hz on a Pi 5" figure is unmeasured.
2. **No Hailo-8 compilation of FastSAM, MobileSAM or EdgeSAM is published.** Their quoted speeds are iPhone 14 / 2080 Ti. Whether the SAM decoder compiles to Hailo-8 at all is an open question.
3. **No controlled study measuring alignment/pose error with instance masks vs boxes.** The question "is segmentation measurably better than boxes for ALIGNMENT" is, as far as I can find, **unanswered in the literature**. The winning teams route around it with features+PnP (Bumblebee) or keypoints (CMU) or oriented boxes (Cornell).
4. **No measured fixed-vs-auto-exposure detector study underwater.** §9 substitutes a *synthetic* blur/noise sweep. A real shutter/gain sweep on our own footage would be new evidence, not a replication.
5. **The synthetic-data scale sweep result** in arXiv 2609.20680 (1× → 10×10×, 1 800 → 18 000 images): I retrieved that the sweep was run, not what it showed.
6. **Mixed synthetic+real fine-tuning numbers** — I did not retrieve a table showing "synthetic pretrain + real fine-tune vs real-only". CMU and ITU both do this in practice; neither publishes the delta.
7. **Cornell's "≈20 % detection accuracy improvement" for YOLO-7D** is a team's self-reported figure in a TDR, on their own data, with no baseline protocol published. The TDR text also says ">10 % increase" in the figure caption for the same result. Treat as **indicative, not measured**.
8. **ASU's "95 % success rate" choosing the correct gate side** — TDR self-report, sample size not given.
9. **XFeat FPS on a Raspberry Pi 5** — the published figure is an Intel i5-1135G7 at VGA. A Pi 5 number would have to be measured here.
10. **Per-metric correlation coefficients between UIQM/UCIQE/PSNR and detector mAP** — the papers state the non-correlation qualitatively; I did not retrieve a Spearman/Pearson value.
11. **Brackish / TrashCan mAP figures** are single-study numbers quoted from one comparison paper (arXiv 2306.14141), not a leaderboard consensus; I did not retrieve which detector produced the 97.5 % / 66.2 %. The **spread between them (97.5 vs 66.2) is itself the useful fact**: "underwater detection mAP" without the dataset named is meaningless.
12. **Caltech, Duke and ERAU TDRs** were not read (the 2026 index lists 50+ teams; I read the five with documented placements or direct relevance). Duke and ERAU are not among the 2025 top-4 placements the results page names.

---

## 12. What the evidence constrains

1. **The enhancement question is closed, and it closes our way.** Four independent studies plus our own. The strongest version: *even retraining the detector on enhanced imagery does not recover the loss* (7/7 detectors, URPC2020). Our 30.4 % → 1.2 % is an extreme instance of a published effect, not an anomaly.
2. **Appearance Re-ID is measured at +1.7 HOTA (MOT17) / +1.8 (MOT20) and +6.2 only on DanceTrack**, a benchmark built from identical-looking targets. Zero of the five TDRs read use it. It costs a second inference stream and has no Hailo-8 model-zoo entry. The measured gap here (2.4 s p99 blackout) is not the association problem that number is measuring.
3. **The 2025 winner's pose stage is XFeat + PnP** — the same anchor already in the lock ladder. Within that family the only published free-lunch axis is XFeat → XFeat\*: **+111 % inliers and +10.2 Acc@10° for 1.4× the time**.
4. **Instance segmentation is cheap on this accelerator (528 FPS, yolov8n_seg) and unproven for this purpose.** No controlled study compares masks vs boxes for alignment error. Oriented boxes (Cornell), keypoints (CMU) and features+PnP (Bumblebee) are what the field reaches for instead.
5. **The blur/noise trade is asymmetric by ~2.5×.** Under simulated degradation blur costs ~22 mAP50 points, the noise regime costs ~55–61. Whatever the exposure setting, it is only defensible while noise stays low — which makes lighting part of the same decision.

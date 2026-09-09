---
name: bumblebee-doctrine
description: How the back-to-back RoboSub champions (BumblebeeAS, 2025 AND 2026) actually think, distilled from a full code-grounded read of their 2026 stack. Load before designing or reviewing ANY perception, estimation, control-integration, or mission-composition change in this repo — and before deciding what NOT to build. Reflexes, not recipes: redundancy over tuning, geometry apart from identity, effort is not points, and configure the fallback.
---

# Bumblebee doctrine — how the champions think

Distilled from reading 13 BumblebeeAS repos at 2026 HEAD (commits through
2026-09-08) plus their TDRs. Full evidence: `bumblebee-study/` (outside the
git repo) — `notes/01-18*.md`, `research/01-04*.md`, `SYNTHESIS.md`.

They won RoboSub **2025 and 2026**. Their architecture is better than ours.
Their line-by-line hygiene is not — a publisher leak is live at HEAD, the
correct ICP solver sits dead, 4 of 11 test imports are broken. **Copy the
architecture and the reflexes. Do not copy the hygiene, and do not assume a
thing is good because they shipped it.**

## Our asymmetry — state it before copying anything

They run ArduSub on a Mini-AUV and publish ONE MAVROS setpoint per goal,
then only monitor, because *"ArduSub retains a GUIDED position target until
it is replaced; re-publishing restarts its trajectory planner."*

**We own firmware, board, companion and BOM.** srot runs a 500 Hz loop on a
dedicated core with the IMU in hand. Their outer loop is 500x slower than our
inner loop. So we put their *capabilities* where they structurally cannot
reach — the visual loop at 500 Hz, the tool frame inside the controller —
and we do NOT copy their control architecture, which is a workaround for not
owning the firmware.

## The eight reflexes

**1. Redundancy beats optimisation.** Their 2026 gate runs FIVE pose
estimators in parallel, each publishing its own frame; slalom runs three
(normal, a `_near` variant with `max_reprojection_error` 100.0 instead of
10.0, and a 2000-particle structure filter). *"Capitalize on the strengths of
each individual method and have several fallbacks."*
→ **When you cannot pick a threshold, RUN BOTH and publish to distinct
frames so the consumer chooses.** We tune one number and hope.

**2. Separate GEOMETRY from IDENTITY.** From `auv5/bin.yaml`: every structure
detection (rim + lightbox) is PnP'd; the symbol topic (fire/blood) is
**never** PnP'd — bin walls occlude symbols from oblique views — it only tags
rim points by mask intersection for an **identity posterior**.
→ Pose from the geometrically reliable thing. The symbol answers only
*which one is it*. We use one detection for both and inherit bad geometry.

**3. Resolve a symmetry with a SECOND object, then re-anchor globally.**
Shipped and verified: candidate corners picked by distance to a second
object, sense reversed per class. After the second torpedo shot,
`seq_relocalise` computes `(odom_yaw + torp_yaw − board_yaw) % 2π` into a
global `zero_yaw` — **the board becomes a heading reference for later
tasks.** Directly portable to our boot-relative BNO.
⚠ Their FOG "directed-yaw lock" is a config comment with **no code**. Intent,
not mechanism. Do not cite it as precedent.

**4. Retrain during the competition, by hand.** Torpedo model trained the day
before final standings; 6 slalom versions, 5 bin, 4 torpedo. Their
`pose_auto_labeller_node` is **dead three ways** (`raise NotImplementedError`,
no `super().__init__()`, not in `console_scripts`) — they labelled in CVAT.
→ Venue-water retraining inside a day is their biggest operational edge, and
it is human throughput. Auto-labelling is something they **abandoned**, so
building it is new work and a real differentiator, not a port.

**4b. Free the accelerator by LIFECYCLE.** `yolo_ros_trt/yolo_node.py` is a
`LifecycleNode`: `on_activate` loads the model *then* creates the image
subscription; `on_deactivate` does `del self.model; gc.collect()` and
destroys it. Default `activate_on_start=False` — idle until the mission asks.
→ This is the answer to our Hailo 2-group SRAM ceiling
(`SRAM_MEMORY_FULL` → 98 % CPU → zero detections). Hot-swap keeps models
resident; **release frees the resource.**

**5. One decode, many models.** *"Single multi-model YOLO node: one
subscription + one decode feeding both models"*, per-model conf/iou/mode,
one in `track` while another runs `predict`.

**6. Graceful degradation is CONFIGURED, not coded.** `claw_required: false`
— *"degrades to front-only if absent"*; per-class `freshness_timeout: 0.5`.
→ Fallbacks are parameters, switchable at the pool. Ours are branches.

**7. Write the reason and the failure next to the number.**
`joint_max_rms_px: 15.0` — *"catches both a mislabeled lone hole sliding
meters onto the wrong quadrant AND a fake-hole 4-hole frame 'solving' at
100px+. Honest joint solves run <10px on real masks."*
`n_slots: 72` — *"avoids the clocking-quantization aliasing that made 36
erratic at a few degrees of roll."*
→ Same discipline as our `measured-bars.md`. **We are peers here — keep it.**

**7b. They SIMPLIFIED between two winning years.** 2026 gate: 300 → 167
lines, shark/fish branch deleted, retries 3 → 1, settling **3 s → 15 s**,
`FailureIsSuccess` added on start-vision, TF clustering → pose clustering
time-synced to odometry (`min_poses=4`, 15 s window).
→ Fewer branches, fewer retries, **much longer settling**, more structural
fallbacks, an evidence gate on perception. That is what a team does after
winning.

**7c. EFFORT IS NOT POINTS.** Slalom is 11 files, ~3,000 lines, including a
2000-particle filter — and **no mother tree imports any of it.** The mothers
dead-reckon waypoints past the slalom. Even `slalom_stupid.py` was not run.
→ A team that can build a particle filter still shipped waypoints where
perception was hardest. **Ask what actually runs before admiring what exists.**

**8. What they WITHHOLD is informative.** Missing from every public repo:
`torpedo_hole_pose_estimator_node`, `gate_structure_pose_estimator_node`,
`slalom_structure_pf_node`, the whole controls stack (trajectory planner,
controller, QP thrust allocator), the UKF localisation node, and the 2026
`yolov26_segment` weights.
**But the configs leak the algorithm.** The 2026 torpedo solver reconstructs
as: IPPE → ICP refinement (`icp_iters: 7`) → angular slot binning
(`n_slots: 90`) → ternary visibility gate on `arc_frac` (≥0.7 full → tilt;
0.4–0.7 partial → yaw only; below → skip) → joint 6-DoF solve with RMS gates.
→ Read their parameter comments as a specification.

## Behaviour-tree vocabulary (census over 19.3k lines)

`Sequence(memory=True)` ×182 default · `Retry(num_failures=N)` ×43 around
every service call · `FailureIsSuccess` ×37 (the never-block idiom) ·
`CheckBlackboardVariableValue` ×62, always first child of a Selector's
Sequence · **zero uses of `py_trees.idioms.*`** — every if/else hand-rolled
as `Selector[ Sequence[check, A], B ]`.

- **`memory=False` appears 4 times in ~220 composites**, always a
  re-evaluating guard inside a `Parallel` beside a long-running motion:
  "poll this every tick while my sibling moves".
- **Preemption has no `Timeout` on motion goals.** It is
  `Parallel(SuccessOnOne)[ motion, FailureIsRunning(condition) ]` — a race
  where "object seen" aborts a sweep mid-flight; `SuccessIsFailure(Timer)`
  makes expiry *lose* a race.
- **`cluster_goto` is the reusable perceive→move→verify primitive**: cluster,
  then `Retry(5)[ goto, re-cluster tighter, tf_check, threshold, assert ]`,
  gated at **2.5 cm and 1°** against **both** the tool frame and base_link.
  Strictly better than our `align(settle=)` because it verifies against a
  **re-measured pose**, not the controller's own error.
- Every task tree ends `Selector[ precise_sequence, blind_fire ]` —
  *"fire both torpedoes blindly so the score doesn't go to zero on a
  perception miss."* RoboSub 2026's Core/Advanced/Disruptive rubric now
  scores exactly this.

## What we deliberately do NOT copy

- **Their control architecture** — a slow outer loop into a black box.
- **Heavyweight multi-target tracking** — they built a Stone Soup JPDA UKF
  and a 3-D SORT and **abandoned both**, because props do not move.
- **Generic SLAM** (purpose-built underwater systems are fine; generic land
  VIO is not).
- **Their dropped covariance** — four 2026 commits moved
  `PoseWithCovarianceStamped` → `PoseStamped`. Shipping a correctly-scaled
  covariance is us going *beyond* them.
- **Their circle-correspondence approximation** in the public repo (measured
  up to 185 mm range and 86° normal error at high obliquity) — take the ICP
  version their config implies.
- **Their publisher-per-publish leak**, live at HEAD in the torpedo node.
- **Image enhancement before a detector** — three independent confirmations,
  including their own: `image_brighten_node` is commented out in
  `torpedo.yaml` twice, and our CLAHE study destroyed 95 % of gate detections.

## Use this skill as a checklist

Before shipping a perception/estimation/mission change, answer:

1. Is there a threshold I am guessing? → run both, publish distinct frames.
2. Am I taking pose from something whose geometry I trust? → split identity out.
3. What does this degrade to, and is that a **parameter**? → `Selector[precise, always_act]`.
4. Does the number have its reason and its failure written beside it?
5. Does anything actually CALL this, or am I admiring effort? → walk callers.
6. Can I delete a branch, cut a retry, and lengthen a settle instead?
7. Does it free the Hailo when the task ends?
8. Have I verified, or am I repeating a doc? **Code over docs; measurement
   over code.** This repo's recurring defect is a plausible number standing
   in for an absent measurement.

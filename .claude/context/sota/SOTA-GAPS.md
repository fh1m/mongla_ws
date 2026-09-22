# The gap ledger — every move, ranked

> How to read this: [`README.md`](README.md). Score is **value ÷ (risk × effort)**, each 1–5.
> Value is points on a run sheet or runs survived. A row without a falsifier is not a move, it is
> a wish, and it does not belong here.
>
> **Two kinds of row.** `OURS` = the gap is against our own stated standard, evidenced by
> `file:line` in this tree — no external citation needed. `SOTA` = the gap is against published
> work, and the row carries the citation in its dossier.

---

## 1. Ranked — the whole board

| # | move | kind | dive | value | risk | effort | score | testable before water? |
|---|---|---|---|---|---|---|---|---|
| G-10 | Close the depth-sign gate on the bench with `DEPTH_CMD` | OURS | control | 5 | 1 | 1 | **25.0** | **yes — today, a thumb and a disarmed board** |
| G-01 | Make every FSM plan survive a refused verb (J04) | OURS | planning | 5 | 1 | 1 | **25.0** | yes — executes in the test suite |
| G-03 | Measure `k_n_per_rpm2` on a fitted thruster | OURS | control | 4 | 1 | 1 | **20.0** | needs thrusters (G2), not water |
| G-12 | Measure the ESC deadband; the yaw floor may sit inside it | SOTA | control | 4 | 1 | 1 | **20.0** | yes — one thruster on a bench |
| G-11 | Geometric `B` + one offline pseudo-inverse + **report the scale factor** | SOTA | control | 5 | 2 | 2 | **12.5** | yes — the host half; the report is firmware PR #20 |
| G-13 | Detect a dead thruster on **current**, not RPM | SOTA | control | 4 | 1 | 2 | **10.0** | needs firmware PR #4 + thrusters |
| G-14 | Assign materials in CAD and get a real BG | SOTA | control | 4 | 1 | 2 | **10.0** | yes — a scale and a CAD session |
| G-07 | Measure tool offsets and stop aiming the camera centre | OURS | vision | 4 | 1 | 1 | **20.0** | yes — a tape measure |
| G-05 | Turn `retrodict` on by default after measuring its Pi cost | OURS | localization | 3 | 1 | 1 | **15.0** | yes — already proven correct |
| G-02 | Publish a "do not trust me" signal from the estimator | OURS | localization | 5 | 1 | 2 | **12.5** | yes — replay a bag with flow removed |
| G-09 | Survey the course and fill the position files | OURS | planning | 5 | 1 | 2 | **12.5** | no — needs the venue |
| G-04 | Model the board's `PILOT_EXPO` droop in the vision loop gain | OURS | control | 4 | 2 | 1 | **10.0** | yes — bench, demand vs commanded |
| G-15 | One-call `format="hailo"` compile + **1 024 in-domain calibration images**, with a guard against the silent COCO128 fallback | SOTA | vision | 4 | 1 | 1 | **20.0** | yes — entirely off-vehicle |
| G-16 | **Active selection** of which frames a human labels | SOTA | vision | 5 | 1 | 2 | **12.5** | yes — off-vehicle, on the existing archive |
| G-17 | A **DeepVL-shaped second-opinion velocity model** | SOTA | vision + localization | 5 | 2 | 3 | **8.3** | partly — needs labelled velocity truth |
| G-18 | XFeat → **XFeat\*** semi-dense in the anchor rung | SOTA | vision | 3 | 1 | 1 | **15.0** | yes — bench, on recorded footage |
| G-19 | Gate flow health on the **KLT Hessian** we already compute | SOTA | vision | 4 | 1 | 1 | **20.0** | yes — replay recorded caustics footage |
| G-08 | A model-production pipeline: label → train → compile → verify, timed | OURS | vision | 5 | 2 | 3 | **8.3** | yes — entirely off-vehicle |
| G-06 | Close the loop from `/lock` to control (`vision.lock_s > 0`) | OURS | vision | 4 | 3 | 2 | **6.7** | partly — bench with a printed target |

*(This table grows as each dive lands. Rows are added with their dossier section, never before.)*

---

## 2. The rows, with their evidence and their falsifier

### G-10 — the highest-consequence unvalidated sign in the stack, closable today · `OURS` · control

**The gap.** The board's depth loop **has never run closed** — the firmware says so in its own
words, because the Bar30 was not fitted during development. Its sign **was inverted** and the fix
is argued but unvalidated. **The SURFACE failsafe routes through the same loop**: a leak, a flat
thruster pack or a lost GCS all call `depth::update()`, so if the sign is still wrong the
emergency ascent drives the vehicle *down*
([`srot-architecture.md` §6c](../platform/srot-architecture.md)).

**The move.** The firmware already built the tool and we have never used it: `depth::preview()`
runs the same error expression through a separate **proportional-only** instance and publishes
`DEPTH_CMD`, readable **disarmed, with nothing spinning**. Pressurise the Bar30 port with a thumb
and read the sign:

| thumb says | `DEPTH_CMD` | verdict |
|---|---|---|
| deeper than target | **positive** → ascend | correct |
| shallower than target | **negative** → descend | correct |
| demand moves *away* from target | — | **still inverted — do not dive** |

P-only is deliberate: a full PID fed a constant error winds to the rail and reports "inverted" for
a correct loop.

**The falsifier.** If `DEPTH_CMD` does not move at all under a thumb, the preview path is not
wired the way the firmware document says, and everything above is unproven.

**Why it ranks at the top.** It needs no water, no thrusters and no firmware merge — the three
things currently blocking everything else — and it is the single sign whose error is a vehicle
that dives when it is trying to save itself. It belongs in `bringup_check`.

### G-01 — every FSM plan aborts to SURFACE on srot · `OURS` · planning

**The gap.** All eight FSM plans build a `LockHeadingState`; srot refuses `lock_heading`; the
refusal raises; `base_state.py:44-54` converts any exception to `ABORT`; every plan wires
`ABORT → SURFACE`. The vehicle arms, dives, and surfaces before attempting a task. Full chain and
evidence: [`BUGS.md` J04](../BUGS.md).

**The move.** Consult `profile.has_heading_lock` in `LockHeadingState._run` and skip loudly — the
pattern `_warn_no_distance_move` (`navigation.py:106-125`) already uses for the distance verbs.

**The falsifier.** A test that *executes* each plan's states against
`srot_fc.UNSUPPORTED_VERBS` must fail today and pass after. If it passes today, the chain above is
wrong and this row is withdrawn.

**Risk 1** — it is a guard, not a behaviour change; the verb was never going to work.

### G-02 — the estimator cannot say "I am lost" · `OURS` · localization

**The gap.** No divergence detector, no covariance bound, no reset, no relocalization. The one
signal is a 5 Hz **log line** written after a measured **635 m of drift in 95 s while publishing a
healthy-looking pose** (`localization_node.py:569-601`). `mongla.pose()` can only see the frame
id, never the no-aiding condition.

**The move.** Put the verdict on the wire: aiding state, NIS health, and a covariance bound, so a
mission can refuse to act on a pose the filter does not stand behind — the same rule the firmware
already follows when it renders `--` instead of `0.0`.

**The falsifier.** Replay the real board bag with flow removed after *t*. If the published verdict
does not change before the position error passes 1 m, the signal is useless and the row is
withdrawn.

**Risk 1** — additive; nothing existing consumes it until a mission opts in.

### G-03 — the thrust constant has never been measured · `OURS` · control

**The gap.** `thrust_model.py:67-78`: `k_n_per_rpm2` has **no default on purpose** — neither host
nor firmware carries an absolute N per RPM². The module has zero callers, asserted by a test. Every
force-domain statement the stack could make is blocked behind it.

**The move.** A load cell (or a fish scale and a lever) and the slope of thrust against RPM²
through the origin, at the battery voltage we fly at. Bollard only — the model is explicitly a
bollard relation and **flatters us at cruise**, where K_T falls with advance ratio.

**The falsifier.** If the fitted line does not pass through the origin within its own residual, the
`T = k·n²` form is wrong for this thruster and the model is replaced, not tuned.

**Risk 1, effort 1** — an afternoon, once thrusters exist (gate G2).

### G-04 — a 30 % gain droop nobody models · `OURS` · control

**The gap.** The board applies `demand = ((1−EXPO)·u + EXPO·u³)·SPEED` with `EXPO = 0.30`
(`srot_protocol.py:528-541`). The vision loop's `kp` is tuned against the demand it *sends*, so the
small-signal gain it actually gets is 0.70 of what it thinks — **exactly in the terminal-alignment
regime** — and the cubic makes one `kp` wrong at both ends of the error range.

**The move.** Invert the shaping on the host before the frame leaves, or read `PILOT_EXPO` and
divide it out of the gain. One line, one parameter read.

**The falsifier.** Command a fixed demand ladder on the bench and log the board's echoed demand. If
the measured curve is linear, the parameter is not doing what the document says and the row is
withdrawn.

**Risk 2** — it changes effective loop gain, so it re-opens gain tuning.

### G-05 — retrodiction is built, correct, measured, and off · `OURS` · localization

**The gap.** `retro.py` is verified to 1e-9 against an on-time filter and costs 0.8 ms per sample
60 ms late — and ships `False` because its **Pi cost was never measured** on the vehicle.

**The move.** Measure it on the Pi under the full graph, then flip the default.

**The falsifier.** If the measured cost at our real lateness distribution exceeds one IMU period
(2 ms at the board's 500 Hz, 20 ms at the host's rate), it stays off and the row closes.

### G-06 — the lock ladder publishes and never steers · `OURS` · vision

**The gap.** The whole ladder — LK follower, XFeat anchor, authority ramp sized on 71 real gaps —
publishes `/lock`, and the control loop reads it only when `vision.lock_s > 0`, which ships **0**.
It was staged deliberately: echo, confirm, actuate. The third step never happened.

**The move.** Enable it behind the existing parameter, on the bench, against a printed target.

**The falsifier.** Compare alignment error with `lock_s = 0` and `lock_s = 1.0` across induced
detector blackouts. If the ladder does not reduce terminal error or hold through a p90 gap
(0.651 s), it stays off — and that is a real answer about the ladder, not about the wiring.

**Risk 3** — it hands authority to a coasted estimate; that is exactly the class of failure that
drives a vehicle into a prop.

### G-07 — every tool offset reads zero · `OURS` · vision

**The gap.** `config/tool_geometry.yaml` — every entry `unmeasured: true`, so the aim point is the
**camera centre**, and the file states the consequence: *the miss equals the offset, at every
range*.

**The move.** A tape measure and a rebuild.

**The falsifier.** Fire on a paper target at two ranges. If the miss does not shift by the measured
offset, the geometry model is wrong and the numbers do not help.

### G-08 — model production is a 40-line skill and a manual CLI · `OURS` · vision

**The gap.** No labelling tool, no augmentation, no dataset versioning, no CI, datasets at
hard-coded absolute paths outside the repo, zero compiled `.hef` in the tree, and **the time to
produce a model for a new class is recorded nowhere**. Meanwhile the evaluation side is unusually
strong (`model_select.py` shows a 43-point cross-session recall gap between two models with
identical configs and identical mAP50).

**The move.** Pending dive 2's research — the shape will be "foundation model labels, small CNN
deploys", and the ledger row will name the specific pipeline and its measured human cost.

**The falsifier.** Time the current path once, end to end, on a real new class. If it is already
under an hour of human attention, the pipeline is not the bottleneck and this row shrinks to
"version the datasets".

### G-09 — the course files have no positions · `OURS` · planning

**The gap.** Both course YAMLs ship every prop `measured: false` with no coordinates, so
`goto_prop`, `fix_position` and `anchor_on` **all refuse** — by design, and correctly. But it means
the map-based half of the mission layer is unreachable at an unsurveyed venue.

**The move.** `course_survey` at the venue, which already exists and writes the deck copy.

**The falsifier.** None needed — this is a measurement, not a hypothesis. The risk is that survey
time at the venue is not available, which is a scheduling answer, not a technical one.

### G-11 — allocate in force space, and put the scale factor on the wire · `SOTA` · control

**The gap.** Durham's theorem says no choice of mixer weights makes clip-after-a-fixed-mix exact;
our per-group scale-down is weaker still, and it runs in **demand space**, where the ±1 entries
overstate surge and sway by **1/cos 45° = 41 %**. Separately, the board computes the applied
scale every tick and **never transmits it**, so every published anti-windup design in this family
is unimplementable here. Both in [`control.md` §2.1, §2.3](control.md).

**The move.** A measured geometric `B`, one weighted pseudo-inverse `C = W⁻¹Bᵀ(BW⁻¹Bᵀ)⁻¹`
computed **offline** (online cost: one 5×5 mat-vec, ~25 MACs), and the achieved wrench or the
scale factor reported on telemetry — which is already asked for as firmware PR #20.

**The falsifier.** Command a diagonal wrench that saturates one group and compare the achieved
body acceleration against both allocators. If the pseudo-inverse does not reduce the wrench error,
the geometry we measured is wrong and the row closes on that instead.

**Risk 2** — it changes what every axis actually delivers, so gains move with it.

### G-12 — the yaw floor may sit inside the ESC deadband · `SOTA` · control

**The gap.** Blue Robotics publish a **±25 µs deadband** around 1500. On our ±400 µs scale that is
**±6.25 %** producing exactly zero thrust — and `VISION_YAW_MIN_PCT = 5.0`, which the code itself
labels *"a hardware spin-up assumption, NOT a measured value"*. If the published figure holds for
our ESCs, **the stiction floor commands nothing at all**, and every terminal-alignment yaw
correction below 6.25 % is silently discarded.

**The move.** One thruster, one bench, a PWM ladder, and the RPM (or a current clamp) — find where
motion actually starts.

**The falsifier.** If motion starts below 5 %, the floor is fine and this row closes with a
measured number replacing an assumption, which is still a win.

### G-13 — a dead thruster is invisible on the channel we have · `SOTA` · control

**The gap.** Published AUV thruster fault detection uses **voltage, current and speed**; an EKF
bank identified a two-thruster failure in **0.1 s** on a BlueROV2. We have `ESC_STATUS(291)`
undecodable and **958/958 frames reading exactly 0 RPM with nothing attached** — an RPM detector
here reports a healthy zero for a missing thruster.

**The move.** Firmware PR #4 already decodes per-ESC voltage/current/temperature and discards it;
getting it on the wire makes the cheap detector possible. Cornell's version — comparing *attempted
against actual* movement — needs nothing new at all.

**The falsifier.** Unplug one thruster on the bench and watch the detector. If current does not
separate a spinning thruster from a missing one, the channel is not discriminating and the row
closes.

### G-14 — "roll is passively stable" is an assumption, not a finding · `SOTA` · control

**The gap.** Roll has no actuator, so the whole argument rests on hydrostatics — and
**BG cannot be computed**, because Onshape reports no material assigned to any part. The
literature's own caution is that *smaller AUVs have a relatively small stabilising moment*, and a
published box-hull design quotes a 7.39 cm metacentric height as the thing that makes it work.

**The move.** Assign materials (or weigh the parts) and compute CoG, CoB and BG. Then a bench
incline test on the assembled hull to check the number.

**The falsifier.** If the measured restoring moment is small enough that a thruster wash can roll
the hull past the camera's usable tilt, "leave roll passive" stops being a design choice and
becomes a ballast problem — which is a hardware answer, found by a software measurement.

### G-15 — the compile is one call now, and it fails silently · `SOTA` · vision

**The gap.** Our deployment path is six manual Dataflow Compiler steps with recorded traps.
`model.export(format="hailo")` has existed since **ultralytics 8.4.97** and covers YOLOv8 / YOLO11
/ YOLO26 including custom models. ⛔ It wants **≥1 024 in-domain calibration images**, and **with
none supplied it silently falls back to COCO128** — the exact silent-failure shape this codebase
keeps catching elsewhere. We currently calibrate on 320 real pool frames.

**The move.** Adopt the one-call export, build a ≥1 024-frame in-domain calibration set, and write
the guard **first**: a compile that did not see our frames must fail loudly, not quietly ship a
COCO-calibrated `.hef`.

**The falsifier.** Compile the same weights both ways and compare INT8 mAP on a held-out session.
If the one-call path is not within the zoo's stated 0.6–2.4 mAP INT8 cost, it is not equivalent and
the manual path stays.

### G-16 — which frames we label matters more than how many · `SOTA` · vision

**The gap.** We label whatever was collected. MaskAL, on a domain-shifted robot dataset with three
held-out test sets, reaches **93.9 % of full-data performance from 17.9 % of the data**, against
**81.9 % for random sampling** — a **12-point gap at equal effort**, and **900 actively-sampled
images ≡ 2 300 random ones**. Our own three-octagon-models result (29.2 / 72.7 / 68.3 %
cross-session recall at an *identical* mAP50 of 0.9950) is this same effect, unmanaged.

**The move.** Score the existing archive by model uncertainty, label the top slice, retrain, repeat
— the loop the literature measures, run on footage we already own.

**The falsifier.** Hold out a session. If an actively-selected 20 % does not beat a random 20 % on
that session's recall, the method does not transfer to our domain and the row closes with a real
answer.

**Note:** nothing in this literature measures **held-out-session** recall — every headline is an
i.i.d. split. We already measure it. That is a lead, not a gap.

### G-17 — a second opinion, not a fallback · `SOTA` · vision + localization

**The gap.** Our flow refuses when it *knows* it cannot measure (no height, bare floor, rotation
dominant). It cannot refuse when it is **confidently wrong** — the measured caustics failure, where
tracking is healthy and tracking the wrong thing. Every published health mechanism except an
independent second estimator detects the **absence** of signal, not a wrong one.

DeepVL ([arXiv 2502.07726](https://arxiv.org/html/2502.07726), ICRA 2025) predicts body velocity
from IMU + thruster commands + battery voltage: **28 k parameters, 3×GRU(40), <5 ms on an Orin
AGX, 3.9 % relative position error through a full visual blackout**, 0.39 m RMSE per 10 m across
88 trajectories, flown on a real BlueROV. Our own flow's drift floor is **3.4 %** — the same
accuracy class, **with the camera off**.

**The move.** Train the same shape on our data. We already have the input side (`/mongla/demand`,
IMU, battery) and a partial version of the idea in `command_velocity.py`'s RLS fit.

**The falsifier.** It needs ~4 h of labelled velocity truth, which we do not have. If flow-derived
velocity is the only available label, the model can never disagree with flow usefully — and then
this row collapses to "buy a DVL or a mocap hour", which is a hardware answer.

**Risk 2** — it is additive until something consumes it.

### G-18 — a measured free lunch inside a component we already run · `SOTA` · vision

**The gap.** XFeat\* (semi-dense) against XFeat on the same CPU and protocol: **inliers
892 → 1 885 (+111 %)**, **Acc@10° 74.9 → 85.1**, for **27.1 → 19.2 FPS (1.4× slower)**. Our anchor
runs at 3 Hz by design, so the headroom exists.

**The falsifier.** Replay recorded blackout footage through both. If anchor survival across our
measured p90 gap (0.651 s) does not improve, the extra inliers are not buying what we need.

### G-19 — the health metric is one eigenvalue away · `SOTA` · vision

**The gap.** Our flow health is point-survival fraction. Super Odometry 2.0
([arXiv 2608.25427](https://arxiv.org/abs/2608.25427), *Science Robotics*) gates visual health on
the **Hessian of KLT tracking** — *the same matrix Shi-Tomasi already computes inside our flow
node* — and disables a modality when its contribution stays below **10 % for 2–4 s**, with the
hysteresis mattering as much as the threshold.

**The move.** Publish the Hessian's smaller eigenvalue as a health channel and gate on it with
hysteresis.

**The falsifier.** Replay the recorded caustics clip. If the eigenvalue does not separate the
caustic-tracking interval from good tracking, it is not the discriminator for *our* failure — which
would itself be worth knowing, because it would confirm that only an independent estimator (G-17)
catches a confident wrong reading.

---

## 3. Rejected here, with the reason

*(Populated as the dives reject things. The `ROADMAP.md` §6 list — Ruckig on the ESP32, ADRC/STSMC
for the distance leg, CLAHE, Fourier–Mellin, camera-based flare order, Pinax LUT, Hailo mask
descriptors — stands and is not re-opened.)*

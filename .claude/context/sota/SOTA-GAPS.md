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
| G-01 | Make every FSM plan survive a refused verb (J04) | OURS | planning | 5 | 1 | 1 | **25.0** | yes — executes in the test suite |
| G-03 | Measure `k_n_per_rpm2` on a fitted thruster | OURS | control | 4 | 1 | 1 | **20.0** | needs thrusters (G2), not water |
| G-07 | Measure tool offsets and stop aiming the camera centre | OURS | vision | 4 | 1 | 1 | **20.0** | yes — a tape measure |
| G-05 | Turn `retrodict` on by default after measuring its Pi cost | OURS | localization | 3 | 1 | 1 | **15.0** | yes — already proven correct |
| G-02 | Publish a "do not trust me" signal from the estimator | OURS | localization | 5 | 1 | 2 | **12.5** | yes — replay a bag with flow removed |
| G-09 | Survey the course and fill the position files | OURS | planning | 5 | 1 | 2 | **12.5** | no — needs the venue |
| G-04 | Model the board's `PILOT_EXPO` droop in the vision loop gain | OURS | control | 4 | 2 | 1 | **10.0** | yes — bench, demand vs commanded |
| G-08 | A model-production pipeline: label → train → compile → verify, timed | OURS | vision | 5 | 2 | 3 | **8.3** | yes — entirely off-vehicle |
| G-06 | Close the loop from `/lock` to control (`vision.lock_s > 0`) | OURS | vision | 4 | 3 | 2 | **6.7** | partly — bench with a printed target |

*(This table grows as each dive lands. Rows are added with their dossier section, never before.)*

---

## 2. The rows, with their evidence and their falsifier

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

---

## 3. Rejected here, with the reason

*(Populated as the dives reject things. The `ROADMAP.md` §6 list — Ruckig on the ESP32, ADRC/STSMC
for the distance leg, CLAHE, Fourier–Mellin, camera-based flare order, Pinax LUT, Hailo mask
descriptors — stands and is not re-opened.)*

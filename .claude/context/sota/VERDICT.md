# The verdict — Mongla against the world, in one document

> The four dossiers answer *"what does the field do?"* subsystem by subsystem. This answers the
> question that was actually asked: **where do we stand, and what would make us the best?**
> Every number here is carried from a dossier, and every dossier carries its source.

**The research behind it:** six sweeps, ~95 fetched sources, **2 464 lines** of raw cited dossiers
in [`sources/`](sources/), each with its own "claims I could not verify" section.

---

## The short answer

**We are not behind the field. We are behind ourselves.**

The recurring finding across all four subsystems is not that Mongla uses inferior methods — in
three places it independently arrived at what the 2025 world champion uses — but that **capability
after capability is built, measured, correct, and switched off**, and that **constants nobody
measured are load-bearing.** The nearest competitor's real advantage is not cleverness; it is that
their stack has been in water and ours has not.

| | |
|---|---|
| **Where we are genuinely ahead** | measurement discipline, honest refusal, and a budget/deadline machinery no deployed system in the sweep could match |
| **Where we are level** | the perception anchor, the tracker, the optics, the estimator's mathematical form |
| **Where we are genuinely behind** | anything requiring a sensor we lack (sonar, DVL-class velocity truth), and **anything requiring water** |
| **Where the field is behind everyone** | five questions nobody has published answers to — and we have the bench to answer them |

---

## Subsystem by subsystem

### Control — level in structure, blind in units

| | |
|---|---|
| **Ahead** | 500 Hz on the board, and a host that refuses verbs it cannot honour rather than faking them |
| **Level** | yaw-first priority turns out to be published DP convention; `REVERSE_EFFICIENCY = 0.77` sits within 1–3 % of the vendor's own 0.754–0.787 |
| **Behind** | **allocation.** Durham's theorem proves clip-after-a-fixed-mix cannot be exact for *any* mixer weights; ours is weaker still — a per-group scale in **demand space**, where ±1 entries overstate surge/sway by **41 %**. Both RoboSub teams with published allocators moved to QP or a documented priority ladder years ago |
| **Blocked** | the achieved wrench is computed on the board every tick and **transmitted nowhere**, so *no* published anti-windup design in this family is implementable here, whichever we pick |
| **Unknown** | `k_n_per_rpm2` has no value at all. Every force-domain claim sits behind one load-cell afternoon |
| ⚠ **Honest** | **no AUV study ties loop rate to performance.** Every AUV control paper reached ran at 10–100 Hz; ArduSub's 400 Hz is an in-air inheritance. 500 Hz may be buying headroom rather than stability — [`BENCH.md` B-13](../workbench/BENCH.md) settles it |
| **The opening** | **INDI**: 512 Hz on a real quadrotor, **7× lower gust deviation than PID** (0.21 m vs 1.51 m), needing only an angular-acceleration estimate and a control-effectiveness matrix identifiable from one test flight. **Nobody has taken it underwater** |

### Vision — our retraction is the field's consensus

| | |
|---|---|
| **Ahead** | ⭐ **we measure the thing the field does not.** Nothing in the entire auto-labelling literature measures held-out-**session** recall; every headline is an i.i.d. split. Our `model_select.py` showed three models at an identical mAP50 of 0.9950 scoring **29.2 / 72.7 / 68.3 %** across sessions |
| **Vindicated** | enhancement-as-preprocessing is dead four independent ways — **98–133 retrained models**, raw wins 7 detectors of 7. Our 30.4 % → 1.2 % gate result is an extreme case of a published effect |
| **Level** | the 2025 champion's pose stage is **YOLO11 → XFeat → PnP** — our anchor, independently chosen; their 2023 stack was SIFT. And **nobody** in five top TDRs runs re-identification |
| **Behind** | the **model pipeline**: no labelling tool, no augmentation, no versioning, no CI, datasets at absolute paths outside the repo, zero compiled `.hef` in the tree. The evidenced alternative is exemplar-prompted teacher → active selection → semi-supervised remainder: **93.9 % of full performance from 17.9 % of the data** vs 81.9 % random |
| **Trap avoided** | full auto-labelling costs ~5 mAP and **collapses below 0.10 mAP on long-tail classes** — which is exactly what a torpedo board is. The human stays, on ~18 % of frames |
| **Hardware ceiling** | **sonar.** Two of five top teams carry it as the fallback when optics fail — precisely our 2.418 s p99 blackout — and there is **no software substitute in the literature** |

### Localization — the right mathematics on an unmeasured foundation

| | |
|---|---|
| **Ahead** | retrodiction verified to **1e-9** against an on-time filter; a right-invariant filter whose Jacobians were checked against finite differences; sources that **refuse** rather than guess |
| **Level** | the invariant error formulation is what the fixed-lag smoother literature reaches for too |
| ⚠ **The sharp one** | **NIS/NEES are chi-squared only for an already-tuned filter.** Our `Q` is a nominal diagonal with a position block of **exactly zero** — so a rejection cannot separate an outlier from a mistuned filter, and the five-rejection lockout break with its ×4 inflation rests on that. **An Allan variance is a precondition, not a refinement** |
| **Behind** | **no divergence detector, no covariance bound, no reset, no relocalization.** The only "am I lost" signal is a 5 Hz log line — written after a measured **635 m of drift in 95 s while publishing a healthy-looking pose** |
| **Context** | the factor-graph literature's headline underwater numbers (0.099 m ATE, 87 % better) are **DVL numbers**. A DVL-INS dead-reckons at 0.01–0.1 % of distance; we are at **3.4 %** — 30–300× worse, and nothing closes that without acoustics |
| **The free win** | `height_above_floor = pool_depth_m + depth_m` is **already written**. One untyped venue constant stands between a correct refusal and a working velocity sensor — and pressure-plus-known-floor is one of only four published routes to absolute scale without a DVL |

### Planning — ahead of the field, and unable to prove it

| | |
|---|---|
| **Ahead** | ⭐ **no deployed system the sweep could find** uses expected-value task ordering, and **none has a worst-case admission gate** like `worth_attempting()`. The closest relatives are anytime/contract algorithms — theory, not flying systems |
| **Resolved** | the FSM layer is **retired** — 3 550 lines that never executed, carrying a defect that ended every run one state after DIVE. Behaviour is representation-independent (Iovino et al.), so a behaviour tree would have reproduced it exactly: the defect was wiring, not structure |
| **Behind** | **pre-flight proof.** COLA2 compiles its mission language to a **Petri net with reachability-based safety proofs** on a real AUV; BehaVerify model-checks **100× faster** than its predecessor. We prove nothing — and we **cannot test in water before we fly**, which makes this the gap that matters most |
| **Partly closed** | PlanSys2 replans dynamically; `run_plan` now re-decides between steps from the live clock. Not within a step, and not on a changed world beyond the clock |

---

## The five things that would matter most

Ranked from the ledger, filtered to what actually changes the outcome:

| # | move | why it is on this list |
|---|---|---|
| 1 | **The depth-sign check** ([B-1](../workbench/BENCH.md)) | the SURFACE failsafe routes through a loop that has never run closed and whose sign was inverted once. Costs a thumb and a disarmed board. If it is wrong, the first emergency dives |
| 2 | **Allan variance** ([B-6](../workbench/BENCH.md)) | one overnight run that makes the estimator's entire gate structure mean something |
| 3 | **Geometric `B` + the scale factor on the wire** (G-11) | removes a 41 % error for ~25 multiply-accumulates, and is the precondition for every published anti-windup design |
| 4 | **A model pipeline with active selection** (G-16, G-15) | 93.9 % of full performance from 17.9 % of the labels — and the compile is one call now, with a silent-COCO128 trap to guard first |
| 5 | **An estimator that can say "I am lost"** (G-02) | today a 635 m drift is a log line. Everything downstream trusts a pose nobody vouches for |

**What is deliberately not on this list:** a QP in the 500 Hz loop (10–13.2 ms measured against a
2 ms budget), appearance re-identification (+1.7 HOTA on a problem shape we do not have), and any
IR depth camera (850/940 nm is absorbed within centimetres of water).

---

## What the field does not know either

Five of our open questions are **not literature gaps — they are measurements nobody has
published**. Each one we take is a number that does not currently exist anywhere:

1. **Allan variance for our IMU class** in this application
2. **A Pi-class CPU cost for fixed-lag smoothing** — no paper publishes one
3. **Masks vs boxes for alignment error** — the field routes around the question entirely
4. **Human-minutes per model class** — an empty cell across the whole 2026 auto-labelling literature
5. **XFeat on a Raspberry Pi 5** — published figures are laptop-class CPUs

For a team with a bench and a measurement habit, that is the opportunity, not the obstacle.

---

## How to read the rest

| document | what it holds |
|---|---|
| [`control.md`](control.md) · [`vision.md`](vision.md) · [`localization.md`](localization.md) · [`planning.md`](planning.md) | the full dive per subsystem: what we do at `file:line`, what the best work does with citations, the measured gap, candidate moves with falsifiers, and what was rejected and why |
| [`SOTA-GAPS.md`](SOTA-GAPS.md) | every move ranked by value ÷ (risk × effort), each with the experiment that would prove it wrong |
| [`sources/`](sources/) | the raw research, verbatim, including every claim the sweeps could **not** verify |
| [`../workbench/`](../workbench/README.md) | what is being measured next, and what it said |

**The standing rule:** a row closes on a **measurement**, never on merged code. A result that kills
an idea is a good result and is written down the same way.

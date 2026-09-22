# Planning, the DSL and the mission executive, against the world

> Dive 4 of four. Method and rubric: [`README.md`](README.md). Our side is read out of the tree
> at `file:line`. Their side carries a URL and a number.

---

## 1. What we do today

### 1.1 The DSL is a Python API, not a language

`src/mongla_planner/mongla_planner/mongla_dsl.py` — **3 121 lines, one class**. No lexer, no
parser, no AST, no external mission format. A mission *is* a Python function `run(mongla, log)`.
≈110 public methods: open-loop motion, exactly **two** closed-loop vision verbs
(`vision.align`, `vision.move`), vision queries, metric geometry, course/heading, detector
control, and a `__getattr__` escape hatch to the raw client (`:3104`).

### 1.2 What is genuinely good, and rare

- **Budget** (`run_budget.py`): the clock starts on a successful **arm**, not at script start;
  `remaining_s = total − reserve − elapsed` with a **structurally unspendable reserve**;
  `verdict()` judges on **worst case**, never best, and returns `full / fallback / skip`.
- **Deadlines** (`mongla_dsl.py:1014-1057`): `with task(name, deadline_s=)` cancels the goal
  **in flight** and raises `TaskAbandoned`; nested tasks keep the earlier deadline; `stop`,
  `surface`, `disarm`, `pause` are never blocked by an expired deadline.
- **Fallbacks** (`resilience.py`): `retry` / `never_fails` / `selector`, taking **callables, never
  verb names**, so the mechanism cannot be pointed at `disarm` by configuration (`:165-170`).
  The vision verbs **never raise on a miss**; a `LOST` with a fallback re-enters the same duration
  budget, and the fallback is **interrupted the instant the target reappears** (`:599`).
- **Refusal with a named reason** as the precondition system: `goto_prop` refuses on no course,
  unmeasured position, no fix, or a leg > 12 m — and has **no timed fallback on purpose**,
  because the removed one once drove 2.361 m for a 1.0 m command and reported success.
- **A scorecard** that records every verb, its outcome, the first 8 hex of the goal UUID, the
  budget verdicts, the notes and the per-verb vision state.

### 1.3 What the planning layer was not, before `run_plan`

**No search, no replanning, no executed value-based ordering.**

- The FSM graph (YASMIN 5.x) was built **once before arm** and was static; `full_competition.py`
  branched on `profile` at *construction* time, never at runtime. **The layer is gone** (§1.4).
- `run_budget.by_points_per_second()` had **zero callers**; so did `RunBudget.plan` and
  `projected_points`. ✅ `run_plan(order='by_value')` now calls the first and the machinery is live.
- The entire adaptive layer is: one scalar clock, three `worth_attempting()` yes/no/degraded
  decisions in `sauvc_full.py`, and a fixed expanding-box search whose radius comes from the fix
  residual (`acquire()`, `:1953-2018`).

### 1.4 ⛔ The defect this map found — and the layer it killed

`state_machines/README.md` (now deleted) claimed that a plan leaning on `LockHeadingState`
*"will not fail loudly — those verbs are refused and the plan continues."* **The code did the
opposite.**
`vehicle_profile.has_heading_lock` (False on srot) has **zero callers**;
`LockHeadingState._run` dispatches `mongla.lock_heading()` unconditionally; srot refuses it;
the DSL raises `MoveFailed`; `base_state.py:49-55` converts any exception to **ABORT**; and every
plan wires `ABORT: 'SURFACE'` on `LOCK_HDG`.

**Consequence: on srot, every one of the eight FSM plans aborted straight to SURFACE immediately
after DIVE.** No test covered it — `test_sauvc_srot_port.py` executes only the three SAUVC
*script* missions against `UNSUPPORTED_VERBS`.

✅ **Resolved 2026-09-22 by retiring the layer** (operator decision; J03 had framed exactly this
choice). 3,550 lines deleted. The capability oracle now lives where missions actually run —
`mongla.can(verb)`, reading `srot_fc.UNSUPPORTED_VERBS` at call time — and `run_plan` skips a step
whose `needs=` verb the backend refuses. See §5.

### 1.5 The constants that are still blank

`plans/full_competition.py:37-47`: `slalom_heading`, `bin_heading`, `torpedo_heading`,
`return_heading` are all `None`; `torpedo_depth_m: None  # MUST fill at pool`. Both course files
ship every prop `measured: false` with no coordinates at all.

---

## 2. What the best work does

> Source dossier: [`sources/sota-planning.md`](sources/sota-planning.md). ⚠ The sweep ran with
> **zero web-search budget** (exhausted session-wide) and worked through arXiv, GitHub and
> bibliographic APIs only. The RoboSub/SAUVC TDRs could not be reached at all — that gap is named
> in §6.

### 2.1 Behaviour trees: the honest version, in both directions

- The empirical study of **34 practitioners**
  ([Ghzouli et al.](https://arxiv.org/abs/2609.22404)) is not a sales pitch: BTs are widely used,
  and their real cost is **tooling and guideline immaturity** — people adopt them and then build
  their own conventions.
- [Iovino et al.'s BT-vs-FSM comparison](https://arxiv.org/pdf/2405.16137) reaches the conclusion
  that matters here: **the behaviour is representation-independent** — anything an FSM expresses,
  a BT expresses and vice versa — and the difference is **maintainability as complexity grows**.
  That is a statement about editing, not about capability.
- Verification is real and fast: **BehaVerify** reports **100× faster** BT verification than the
  prior tool, and Ingrand's Fiacre/Tina/Hippo pipeline formally verifies robot behaviour models.

**What this says for us:** a BT would not have made the retired FSM correct — J04 was a
capability check that existed and was never consulted, which is a *wiring* defect that a BT
reproduces just as happily. The published advantage of BTs is editing at scale, and our missions
are a few hundred lines of Python that read top to bottom.

### 2.2 The field executives, and what each actually guarantees

| system | deployed | the guarantee it offers |
|---|---|---|
| **PLEXIL** (NASA) | **7 named deployments** — Curiosity's drill, LADEE, ISS among them | deterministic execution semantics with contingency handling, formally validatable |
| **T-REX** (MBARI) | ported onto a **HUGIN AUV**, validated in a 2017 NTNU-FFI cruise | timeline-based deliberation inside a look-ahead window, on a real AUV |
| **PlanSys2** | ROS 2, IROS 2021, 492★ | PDDL plans compiled **into** behaviour trees — and it **replans dynamically** |
| **COLA2** (Girona) | Ictineu AUV, 98+ citations | ⭐ the mission language compiles to a **Petri net with reachability-based safety proofs** |

**COLA2 is the one worth stealing from.** It is the only architecture found with an actual
mathematical route to a pre-flight safety proof — the thing we need most, because **we cannot test
in water before we fly**.

**PlanSys2 names our sharpest capability gap:** it replans when the world changes. Our missions —
before `run_plan` — decided nothing at runtime.

### 2.3 What nobody could find

- **Decision-theoretic task selection.** Searched four ways: **no deployed field robot** using
  MDP/POMDP or expected-value task ordering turned up. Logged as an absence rather than a
  confirmed negative — but it means our `worth_attempting()` worst-case admission gate is not
  behind the field; it is ahead of anything the sweep could find deployed.
- **A worst-case admission gate like ours.** The closest relatives are classical **anytime and
  contract algorithms** (JAIR 2014, IJCAI 2019) — theory, not a flying system.
- **LTL/STL mission specifications actually flown.** They appear at planning time and were never
  found in the water.

### 2.4 What can be proven before the vehicle is wet

Three routes exist, and they are not exotic:

1. **Model-check the mission as a static artifact** (BehaVerify's approach, 100× faster than its
   predecessor).
2. **Compile it to a structure with proofs attached** — COLA2's Petri net, with reachability
   analysis answering "can this mission reach a state where it is submerged with nothing
   commanding it?"
3. **Verify a narrow safety layer separately** — [Safe-ROS](https://arxiv.org/pdf/2511.14433)'s
   Safety Instrumented Functions: a small, independently-verified watchdog beside an unverified
   autonomy stack.

Route 3 is the cheapest and matches what we already have: `mission.py` guarantees disarm on every
exit path, and the safety verbs bypass the deadline gate.

---

## 3. The gap

| # | what the best work does | what we do | the gap |
|---|---|---|---|
| P-1 | **prove something about the mission before flight** (COLA2's Petri net; BehaVerify) | nothing is proven — a mission is arbitrary Python, checked by running it | we cannot test in water before we fly, which makes this the gap that matters most |
| P-2 | **replan when the world changes** (PlanSys2) | `run_plan` now re-decides *between* steps from the live clock — a first-order version | no re-ordering or substitution **within** a step; no reaction to a changed world beyond the clock |
| P-3 | a separately-verified **safety layer** (Safe-ROS SIFs) | `mission.py` guarantees disarm on every exit path; safety verbs bypass the deadline gate | the guarantees exist but are asserted by tests, not by a verified component |
| P-4 | timeline-based deliberation on a real AUV (T-REX on HUGIN) | linear scripts | we are not trying to schedule a 12-hour survey; this is a ceiling, not a gap |

**And what we are ahead on, stated because it is easy to miss:** no deployed system the sweep
could find uses expected-value task ordering, and none has a **worst-case admission gate** like
`worth_attempting()`. Our budget/deadline/abandonment machinery is unusual, and it was already
here before this dive.

---

## 4. Candidate moves

- **A pre-flight mission checker (P-1).** Walk a mission's AST and answer, before anything gets
  wet: does every path reach `disarm`? Does it call a verb this backend refuses? Does every task
  carry a deadline? Does the declared worst case fit the budget? We already have AST helpers in
  `test/mission_ast.py` written for exactly this kind of question, and `run_plan` makes the cost
  and value of each step **declared data** rather than control flow — which is what makes them
  checkable at all.
- **Keep `run_plan` shallow.** Its whole value is that a mission stays readable. Re-ordering
  within a step, or a planner that generates the step list, would trade that away for a capability
  nobody the sweep could find has deployed.

---

## 5. Rejected, with the reason

| rejected | why |
|---|---|
| **The YASMIN FSM layer** | ⛔ **Retired 2026-09-22.** 3,550 lines that had never executed, carrying J04 — every plan aborted to SURFACE one state after DIVE on srot. The published case for a graph representation is *maintainability at scale* (Iovino et al.), and behaviour is representation-independent; our missions are short, linear and read top to bottom. What the layer promised now lives in the DSL: `can()`, `require()`, `run_plan()`. |
| **Adopting behaviour trees in its place** | Same reasoning, plus: a BT would have reproduced J04 exactly — the defect was a capability check that existed and was never consulted, which no representation prevents. |
| **PDDL / PlanSys2** | The gap it fills is dynamic replanning over a symbolic domain. Our course is five known tasks on a clock; `run_plan`'s per-step re-decision covers the part of that we can use, without a planner, a domain file, or a second execution engine. |
| **LTL/STL mission specs** | Never found flown — planning-time artefacts. Revisit only if the pre-flight checker (P-1) needs a property language, and then as a checking notation, not an execution one. |

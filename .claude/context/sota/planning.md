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

### 1.3 What the planning layer is not

**No search, no replanning, no executed value-based ordering.**

- The FSM graph (YASMIN 5.x) is built **once before arm** and is static; `full_competition.py`
  branches on `profile` at *construction* time, never at runtime.
- `run_budget.by_points_per_second()` exists and has **zero callers**; so do `RunBudget.plan` and
  `projected_points`. Task order is hardcoded in each mission body.
- The entire adaptive layer is: one scalar clock, three `worth_attempting()` yes/no/degraded
  decisions in `sauvc_full.py`, and a fixed expanding-box search whose radius comes from the fix
  residual (`acquire()`, `:1953-2018`).

### 1.4 ⛔ The defect this map found

`state_machines/README.md` claims a plan that leans on `LockHeadingState` *"will not fail loudly —
those verbs are refused and the plan continues."* **The code does the opposite.**
`vehicle_profile.has_heading_lock` (False on srot) has **zero callers**;
`LockHeadingState._run` dispatches `mongla.lock_heading()` unconditionally; srot refuses it;
the DSL raises `MoveFailed`; `base_state.py:49-55` converts any exception to **ABORT**; and every
plan wires `ABORT: 'SURFACE'` on `LOCK_HDG`.

**Consequence: on srot, six of the eight FSM plans abort straight to SURFACE immediately after
DIVE.** No test covers it — `test_sauvc_srot_port.py` executes only the three SAUVC *script*
missions against `UNSUPPORTED_VERBS`.

### 1.5 The constants that are still blank

`plans/full_competition.py:37-47`: `slalom_heading`, `bin_heading`, `torpedo_heading`,
`return_heading` are all `None`; `torpedo_depth_m: None  # MUST fill at pool`. Both course files
ship every prop `measured: false` with no coordinates at all.

---

## 2. What the best work does

*(Pending the research sweep for this dive.)*

---

## 3. The gap

---

## 4. Candidate moves

---

## 5. Rejected, with the reason

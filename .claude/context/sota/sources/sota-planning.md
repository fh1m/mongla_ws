# SOTA: Robust Mission Execution for Autonomous Vehicles

Research pass 2026-09-22. Evidence-only; no implementation plan. Every claim carries a
a titled markdown link to the source and a number or a concrete capability statement. Context for the reader: our
system is a **Python method API (not a language)** — a mission is `run(mongla, log)` calling
~110 methods on one facade class (3 121 lines). It has a **run budget** with worst-case
admission (`full`/`fallback`/`skip`), per-task **deadlines** that cancel in-flight goals
(`TaskAbandoned`), **fallback chains over callables** (never verb names), vision verbs that
**never raise on a miss**, and a **scorecard** of every verb/outcome/goal UUID. It has **no
search, no replanning, no executed value-based ordering** — the FSM graph is built once before
arm and is static; `by_points_per_second()` exists with zero callers.

⚠ NOTE ON METHOD: WebSearch was exhausted early in this session (shared session budget, not
task-specific — hit "200 of 200" before this task's own searches ran). All research past the
first 3 sources uses WebFetch directly against known arXiv IDs, official docs, and GitHub
repos named in the task brief. Where a claim could not be independently located this way, it
is logged in "claims I could NOT verify" rather than asserted.

---

## 1. Behaviour trees

### [Behavior Trees for Robotic Systems: An Empirical Study on Practices and Experiences](https://arxiv.org/abs/2609.22404)
Ghzouli, Horkoff, Strüber, Wohlrab. Technical action research at an automotive company plus a
**34-practitioner survey**. Finding: BTs "improve team communication and the understandability
of robotic decision-making logic" — but the abstract itself does NOT give a number for this
(no measured debugging-time or defect-rate delta was found in the fetched abstract). The paper
is explicit that practitioners face "design and integration decisions, complicated by the lack
of adequate guidelines and tool support" — i.e. BTs cost tooling/guideline maturity, not free.
**This is the paper's own even-handed caveat**: no standard way yet to choose BT node ordering
or verify architecture choices are correct.

### [A formal implementation of Behavior Trees to act in robotics](https://arxiv.org/abs/2502.11904)
Ingrand (LAAS-CNRS). Translates BTs to **Fiacre** (a formal language) → **Tina** (model
checker, offline LTL/CTL verification) and **Hippo** (runtime verification engine). Concrete
capability: BTs written normally can be checked offline against LTL/CTL properties **without
the robotics programmer needing to learn a formal language**, plus online runtime verification
during execution. Demonstrated on two robotics applications. This is the sharpest evidence in
this dive that a BT, unlike an imperative script, has a path to **provable** properties before
flight — because the tree structure is a static artifact a model checker can consume; an
arbitrary Python function is not.

### [Comparison between Behavior Trees and Finite State Machines](https://arxiv.org/abs/2405.16137)
Iovino, Förster, Falco, Chung, Siegwart, Smith (submitted to IEEE T-RO, May 2024). Practical
comparison on a **mobile manipulation task**, both in simulation and on physical robots,
scored across reactivity / modularity / readability / design methodology. Central finding:
**"the robot's behavior during task solving is independent [of] the policy representation"**
— i.e. BT vs FSM produce equivalent robot behavior — but **"maintaining a BT rather [than] an
FSM becomes easier as the task increases in complexity."** This is a maintainability claim,
not a capability claim: for a small mission (our size class, ~dozens of tasks) the paper's own
result says the two are behaviorally equivalent, and the case for BTs strengthens specifically
as scale grows.

---

### [Formalizing Stateful Behavior Trees](https://arxiv.org/abs/2411.14165)
Serbinowska, Robinette, Karsai, Johnson (FMAS 2024, EPTCS 411). Formal result: **Stateful
Behavior Trees with unbounded integer storage are equivalent in computational power to Turing
Machines**; restricted to finitary variable types they are equivalent to finite state
automata. Practical tool: **BehaVerify**, generating Haskell code and nuXmv models for model
checking, reported to **outperform another verification tool by a factor of 100** on its
scalability benchmarks. This is the clearest concrete number in the BT-verification space
found in this dive: a two-orders-of-magnitude tool-performance claim, not a hand-wave.

---

## 2. Field mission executives (PLEXIL, T-REX, RMPL/Enterprise, PlanSys2)

### [PLEXIL](https://en.wikipedia.org/wiki/PLEXIL) — NASA, [github.com/plexil-group/plexil](https://github.com/plexil-group/plexil)
Open-source plan execution language + engine, first appeared **2005**. Confirmed deployments
(named, not estimated): **NASA K10 rover**, **Mars Curiosity rover's percussion drill**, the
**Deep Space Habitat / Habitat Demonstration Unit**, **Edison Demonstration of Smallsat
Networks**, the **LADEE spacecraft**, the **Autonomy Operating System (AOS)**, and
**International Space Station procedure automation** — seven named flight/field systems. What
it guarantees over a hand-written script: PLEXIL plans are a declarative, interchangeable
representation (not tied to one host language), meant specifically so the *same* plan
artifact can be statically analyzed/exchanged across tools — but neither the GitHub README nor
Wikipedia state a formal concurrency or timing guarantee in the material fetched; that would
need the PLEXIL manual itself (not reached in this pass — logged below as unverified).

### T-REX — MBARI, deployed on AUVs
No primary MBARI page was reachable this session (404s, search engines blocked). Corroborating
evidence via [OpenAlex work search](https://api.openalex.org/works?search=NTNU-FFI+Cruise+2017+Hugin+Autonomy+Integration+DUNE+T-REX):
a **NTNU-FFI report, "NTNU-FFI Cruise 2017 – Hugin Autonomy Integration (DUNE, T-REX)"**
(Sture, Wiig, Fossum) states T-REX was integrated as "a dedicated autonomy co-processor on FFI
Hugin HUS" and **"experimentally validated during a joint FFI-NTNU cruise in November 2017"**
— i.e. T-REX's teleo-reactive executive was ported off MBARI's own vehicles onto a HUGIN AUV
and run in water, evidence the architecture is not a single-lab curiosity. No dive count or
hours figure was retrievable from the metadata (full PDF is in the Norwegian Open Research
Archive, not fetched this session).

### [On mixed-initiative planning and control for Autonomous Underwater Vehicles](https://api.openalex.org/works?search=mixed-initiative+planning+control+autonomous+underwater+vehicles+Chrpa+Rajan)
Chrpa, Pinto, Ribeiro, Py, de Sousa, Rajan (IROS 2015, **18 citations**, 86th–98th percentile
for its year). Moves AUV supervision away from a priori waypoint sequences toward
**goal-oriented commanding via a shore-based automated planner**, explicitly to cut operator
cognitive load when running multiple AUVs at once. Reports "preliminary experiments" with
multiple AUVs in water simultaneously; no specific success-rate numbers were retrievable from
the abstract metadata this session.

### PlanSys2 / RMPL-Enterprise
Not reached this session — WebSearch was already exhausted and the specific project pages
(ROS 2 PlanSys2 docs, MIT RMPL/Enterprise) were not tried via direct WebFetch yet. Logged
under "claims I could NOT verify" below; a follow-up pass should hit
`github.com/IntelligentRoboticsLabs/ros2_planning_system` directly.

---

### [PlanSys2 (ros2_planning_system)](https://github.com/PlanSys2/ros2_planning_system)
PDDL-based planner for ROS 2, published at **IROS 2021**. Translates PDDL domain/problem
specs into plans, then executes through **behavior trees** (`plansys2_bt_actions`) with an
**execution monitor** (`plansys2_executor`) that **supports dynamic replanning when conditions
change during execution**. This is the one architecture in this dive that actually closes the
loop our system explicitly does not: PlanSys2 replans; our FSM graph is static once built.
Adoption signal: **492 GitHub stars, 115 forks**, ROS 2 distros Humble through Rolling — i.e.
a maintained, not experimental, piece of ROS 2 infrastructure.

---

## 3. Deliberative planning actually deployed on AUVs

### [COLA2: A Control Architecture for AUVs](https://api.openalex.org/works?search=COLA2+Girona+AUV+control+architecture)
Palomeras, El-Fakdi, Carreras, Ridao (Universitat de Girona), *IEEE Journal of Oceanic
Engineering* vol. 37 no. 4, pp. 695–716, **2012**. **98+ citations, top-10th-percentile**
impact. Three layers: reactive, execution, mission. The mission layer uses a **Mission
Control Language (MCL) that is automatically compiled into a Petri net**, and safety is argued
via the Petri net's **reachability properties**, which compose "while preserving these
qualities" — i.e. COLA2 is the clearest example in this dive of a fielded AUV architecture
that gets a *provable* safety property (reachability) out of its mission representation before
the vehicle is in the water, by construction of the representation itself rather than by
after-the-fact testing. Validated on the **Ictineu AUV** in tank and real underwater
conditions; exact mission-count/hours not retrievable from the abstract metadata this session.

### On mixed-initiative planning and control for AUVs
See topic 2 above (Chrpa, Pinto, Ribeiro, Py, de Sousa, Rajan, IROS 2015) — shore-based
automated planner replacing a priori waypoint sequencing, validated with multiple AUVs
operating simultaneously in water.

### T-REX on HUGIN (NTNU-FFI)
See topic 2 above — MBARI's teleo-reactive executive ported to a Norwegian HUGIN AUV and
"experimentally validated" in a joint cruise, November 2017.

*NTNU's own deliberative-planning AUV work (beyond the T-REX port) and WHOI's planning stack
were not reached this session — logged below as unverified.*

---

### [Runtime Verification and Field-based Testing for ROS-based Robotic Systems](https://arxiv.org/abs/2404.11498)
Caldas, Pinera Garcia, Schiopu, Pelliccione, Rodrigues, Berger. Produces **20 practitioner
guidelines** (8 for developers, 12 for QA), grounded in literature review + mining of ROS
repos + **55 questionnaire responses** across two validation rounds. Core claim: simulation
"cannot realistically deliver solutions to emulate real-world phenomena," so runtime
verification and field-based testing are treated as necessary, not optional, for ROS systems
headed to unpredictable real environments — directly relevant to our own "cannot test in
water before flying" constraint, though this paper is about testing *in* the field, not
proving things *before* it, which is the harder ask we actually have.

### [Safe-ROS: An Architecture for Autonomous Robots in Safety-Critical Domains](https://arxiv.org/abs/2511.14433)
Benjumea, Farrell, Dennis (University of Manchester). Pairs the main control system with
**independent Safety Instrumented Functions (SIFs) implemented as formally-verified cognitive
agents** — i.e. safety logic is factored out into a separate, small, formally checked
component rather than being one property of a large program. Case study: an AgileX Scout Mini
performing autonomous nuclear-facility inspection, one instantiated safety requirement
(stop-on-obstacle) taken end-to-end from formal property → implementation → integration
verification, validated in Gazebo + lab testing. This is architecturally close to our own
"safety verbs never blocked by budgets/deadlines" design — Safe-ROS's contribution is that the
safety layer is *separately verified*, not merely coded to run first.

---

## 4. Decision-theoretic task selection

No deployed AUV or field robot using explicit MDP/POMDP or expected-value task ordering was
located this session (multiple targeted OpenAlex queries returned only tangential results —
path-planning surveys, HRI assembly work, UAV spectrum surveys — none reporting a fielded
robot picking its *next task* by expected value against a clock). This absence is itself
informative given the effort spent (4 distinct queries): **decision-theoretic task selection
for "what to do next given the clock and what's been seen" does not appear to be common
practice in deployed field robots**, at least not discoverable through open bibliographic
search in this pass. This should be read as "not found," not "does not exist" — the
literature may use different vocabulary (e.g. "goal reasoning," "task allocation under
uncertainty") not tried here. Logged more fully under unverified claims below.

---

## 5. Verification before flight — see entries above (BehaVerify/SBT, Ingrand's Fiacre/Tina/
Hippo pipeline, Safe-ROS, COLA2's Petri-net reachability, runtime-verification guidelines).
Collected into a single answer in the closing section below.

---

## 6. Fault tolerance as architecture

### [An integrated diagnostic architecture for autonomous underwater vehicles](https://api.openalex.org/works?search=hierarchical+fault+detection+isolation+recovery+autonomous+underwater+vehicle+field)
Hamilton, Lane, Brown, Evans, Taylor. *Journal of Field Robotics* vol. 24 no. 6, pp. 497–526,
**2007**. Concrete architecture: a **hierarchical ontology model** of the vehicle down to
"least replaceable units," spanning **electrical, mechanical, hydraulic, and computing**
subsystems, diagnosed by a mix of **domain-dependent tools (rulebase, model-based methods)**
and **domain-independent tools (correlator, topology analyzer, watcher)**. The point that
transfers directly: fault detection/diagnosis is explicitly **two separate stages** — a
domain-independent layer that notices something is wrong anywhere in the ontology, and a
domain-dependent layer that pins down what and where — rather than one monolithic per-sensor
check. Where recovery decisions live is not stated in the abstract fetched.

---

## 7. Mission specification languages beyond BTs/PDDL — LTL/STL

Four targeted OpenAlex queries for LTL/STL mission specs **actually flown** on real robots
returned only simulation-validated work: e.g. **"Provably-Correct Task Planning for Autonomous
Outdoor Robots"** (Yoo, 2014 dissertation) and an LTL-based charging-station deployment paper
(Huang et al. 2024) that "validates results through simulation" only. **No source found in
this pass reports an LTL/STL mission specification actually executed on a real deployed
robot** (as opposed to used as an internal planning representation compiled down to something
else, e.g. PlanSys2's PDDL→BT path, or COLA2's MCL→Petri-net path, both of which *are* flown).
This reads as a real gap, worth stating plainly: **temporal-logic mission specs remain a
planning-time/verification-time tool, not an execution-time one, in the sources reachable
here.**

---

## 8. What top RoboSub/SAUVC teams use for mission execution

**Not reachable this session.** RoboNation team design reports (TDRs) are grey literature —
not indexed by OpenAlex/Semantic Scholar/dblp — and require either a working web search or a
known direct PDF URL, both unavailable here: WebSearch hit its session-wide quota (200/200,
exhausted before this task's own searches ran, evidently by earlier turns in this session)
before any query for this task executed, and Bing/DuckDuckGo returned CAPTCHA/corrupted pages
through WebFetch. Direct guesses at `robonation.org` PDF paths and team GitHub repos
(`uf-mil/SubjuGator`, `uwrov/mission-control`) 404'd or returned only repo metadata, not
README content. **This topic is a genuine gap in this pass, not a "nobody does X" finding** —
it needs either a working search tool or a person with the TDR PDFs on hand. Our own
project's `bumblebee-doctrine` skill (internal, not re-verified externally this session)
already encodes a distilled read of BumblebeeAS's 2025/2026 champion stack, which the project
should treat as its actual source on this topic rather than what this file found.

## 9. Time/budget-aware execution

### [Optimal Scheduling of Contract Algorithms for Anytime Problem-Solving](https://api.openalex.org/works?search=anytime+algorithm+contract+algorithm+Zilberstein+deadline+execution)
López-Ortiz, Angelopoulos, Hamel. *Journal of Artificial Intelligence Research*, 2014.
Resolves "an open conjecture" on optimal scheduling of contract algorithms (algorithms that
must complete within a specified time bound given as input) across **m parallel processors**.
Defines the field's standard vocabulary precisely: an **anytime algorithm** has monotonically
increasing utility with execution time and can be interrupted at any point for its
current-best result; a **contract algorithm** is given a time bound up front and guarantees
completion by then, but (classically) produces nothing useful if interrupted early — the
opposite trade-off from an anytime algorithm.

### [Earliest-Completion Scheduling of Contract Algorithms with End Guarantees](https://api.openalex.org/works?search=anytime+algorithm+contract+algorithm+Zilberstein+deadline+execution)
Angelopoulos, Jin, IJCAI 2019. Extends contract-algorithm scheduling with explicit **end
guarantees** — i.e. formal bounds on what is guaranteed to be true when the contract's clock
runs out, not just "it stops."

### Worst-case admission gate — no direct match found
No source located in this pass describes a **pre-mission worst-case admission gate**
structurally equivalent to ours (compute the worst-case cost of every remaining task before
committing, refuse to start if it cannot be paid even in the worst case, with a structurally
unspendable safety reserve). The anytime/contract-algorithm literature is the closest
theoretical relative — it formalizes "how much can I guarantee by a deadline" — but that
literature is about algorithm scheduling (e.g. which of several available algorithms to run
for how long), not about a mission executive deciding whether to *admit* a sequence of
physical-world tasks against a clock with a reserved margin. This appears to be a genuinely
underexplored combination in the sources reachable this session (as also found by the earlier
"Task Scheduling with Mobile Robots" 2025 systematic literature review of **71 papers,
2014–2024**, which discusses scheduling complexity generally but — per its fetched abstract —
does not name deadline-driven abandonment or admission-gate mechanisms as covered topics).

---

## Comparison table

| Approach | What it guarantees | What it costs | Runs on ROS 2 today | Verdict vs. our Python DSL with budgets/deadlines |
|---|---|---|---|---|
| **BehaviorTree.CPP / BT-style trees** | Static tree = analyzable artifact; formal semantics exist (Ingrand's Fiacre/Tina/Hippo; Serbinowska et al.'s BehaVerify, 100x faster than a prior tool) | Tooling/guideline immaturity — Ghzouli et al.'s own practitioners report unclear node-ordering and architecture guidance | Yes (BehaviorTree.CPP is ROS 2-native) | We have static structure (the FSM graph) but not the tree formalism a model checker consumes; our fallback-as-callable design is BT-*reactive*-node-shaped without the BT tooling ecosystem |
| **PlanSys2 (PDDL→BT)** | Domain-independent planning + **dynamic replanning on changed conditions** | Needs a PDDL domain/problem model authored and kept in sync with reality | Yes, maintained (492★/115 forks, IROS 2021) | We have no replanning at all — this is the sharpest capability gap this dive surfaces |
| **PLEXIL** | Interchangeable plan representation flown on 7 named NASA systems (K10 rover, Curiosity's drill, LADEE, ISS procedures, etc.) | Its own DSL/toolchain to learn; concurrency/timing guarantees not confirmed in sources reached | Not natively; would need a ROS 2 bridge | Our budgets/deadlines are bespoke Python, not an interchangeable/portable plan format — PLEXIL trades that portability for its own tooling burden |
| **T-REX (teleo-reactive, MBARI)** | Ported off MBARI's own AUVs onto a HUGIN AUV and "experimentally validated" in water (NTNU-FFI 2017) — cross-institution reuse is real evidence of robustness | Reactive/timeline architecture, own DSL for "reactors"; adoption outside MBARI's own ecosystem appears thin | Not natively (pre-dates ROS 2 as a mainstream target) | Closest philosophical cousin to ours (reactive, deadline-aware) but with actual deliberative planning underneath, which we don't have |
| **COLA2 (Girona)** | Mission language **compiled to a Petri net**, safety argued via **Petri-net reachability properties that compose** — a provable-before-water property from the representation itself | Requires modeling missions in Petri-net-compilable MCL, not arbitrary imperative code | Not directly (pre-ROS 2 lineage, though Girona's stack has since moved toward ROS) | This is the one architecture in this dive with an actual **mathematical safety proof route** from the mission spec — our refusal-with-reason + fallback design is defense in depth, not a proof |
| **MDP/POMDP / expected-value task ordering** | In theory: provably optimal task choice under uncertainty and a clock | Not found deployed on a real field robot in this pass (4 targeted queries, no hit) | Not established as common ROS 2 practice found here | We already know we don't do this (`by_points_per_second()`, zero callers) — the literature search did not surface anyone else doing it live either, so this is not an obviously-missing industry-standard capability |
| **Safe-ROS-style separated SIFs** | Independent, **formally verified** safety functions alongside the main controller | One case study, one safety requirement, lab-validated only | Yes, ROS-based (AgileX Scout Mini demo) | Our "safety verbs never blocked" rule is close in spirit but not formally verified as a separate component |
| **Anytime/contract algorithms (theory)** | Formal completion/utility guarantees by a deadline (JAIR 2014, IJCAI 2019) | Framed around algorithm scheduling, not mission-task admission | N/A (algorithmic theory, not a ROS package) | Our worst-case admission gate is a real-world instance of contract-algorithm thinking applied to task admission — no other deployed system doing exactly this was found |

---

## What can be verified before water

From the sources actually reached this session, three distinct routes to pre-flight proof
exist, none of which our current Python-method mission format uses:

1. **Model-check the plan as a static artifact.** Ingrand's Fiacre/Tina/Hippo pipeline
   translates a BT to a Timed Transition System and checks LTL/CTL properties offline
   ([arXiv:2502.11904](https://arxiv.org/abs/2502.11904)); Serbinowska et al.'s BehaVerify does
   the same via Haskell/nuXmv and is reported **100x faster** than a prior tool
   ([arXiv:2411.14165](https://arxiv.org/abs/2411.14165)). This requires the mission to *be* a
   tree/graph a checker can consume — our mission is an arbitrary Python function, which a
   generic model checker cannot statically enumerate.
2. **Compile the mission to a structure with a mathematical safety property.** COLA2 compiles
   its Mission Control Language to a Petri net and argues safety via reachability properties
   that compose under further composition — proven by construction, not by testing
   ([COLA2, IEEE JOE 2012](https://api.openalex.org/works?search=COLA2+Girona+AUV+control+architecture)).
3. **Separately verify a narrow safety layer, not the whole system.** Safe-ROS formally
   verifies small Safety Instrumented Functions (cognitive agents) independent of the main
   controller and demonstrates end-to-end formal-property → implementation → integration
   verification for one case ([arXiv:2511.14433](https://arxiv.org/abs/2511.14433)).

None of these routes is "run the Python function symbolically" — all three require the mission
(or at least its safety-relevant slice) to exist as a *non-imperative*, analyzable
representation before any proof is possible. Our own deadline/budget/fallback-callable
machinery is a **runtime discipline**, not a pre-flight-provable one, by the same standard
these three routes use.

---

## Claims I could NOT verify

- **PLEXIL's formal concurrency/timing guarantees.** The GitHub README and Wikipedia page
  describe deployment history but not the execution semantics; the PLEXIL user manual itself
  (linked from the repo but not fetched) would be needed.
- **T-REX deployment scale** (dive count, total hours, number of vehicles beyond the one
  MBARI+HUGIN cross-institution data point found). The NTNU-FFI report's full text (Norwegian
  Open Research Archive) was not fetched.
- **RMPL / Enterprise (MIT)** — not reached at all this session; zero sources attempted beyond
  the initial plan.
- **Decision-theoretic (MDP/POMDP) task selection deployed on any real field robot with
  reported numbers** — four targeted bibliographic queries found none; this is reported above
  as an absence, but absence-from-search is weaker evidence than a stated negative from a
  survey paper, and the vocabulary used ("goal reasoning," "task allocation under uncertainty")
  may hide relevant work not tried.
- **What top RoboSub/SAUVC teams (2023–2026 TDRs) actually use for mission execution** —
  entirely unreached; WebSearch was already exhausted before this task began, and grey
  literature (competition TDRs) is not indexed by the bibliographic APIs used as a fallback.
  This is the single largest gap in this dive.
- **LTL/STL mission specs "does anybody fly them"** — reported above as "not found in this
  pass," based on four queries all surfacing simulation-only work. This should be treated as
  weak/absence evidence, not a confirmed negative — the search method (keyword bibliographic
  queries) is bad at finding grey-literature flight reports generally.
- **NTNU and WHOI's own (non-T-REX) deliberative planning stacks** for AUVs — named in the
  task brief, not reached this session.
- **Any numeric hours/dive-count figures** for COLA2's Ictineu AUV validation, PLEXIL's
  per-system flight hours, or T-REX's MBARI-native deployment record — abstracts described
  *that* validation happened, not *how much*.


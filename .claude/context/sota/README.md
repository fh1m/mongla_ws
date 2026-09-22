# Against the world — how Mongla compares to the best work there is

Five documents. Four dossiers, one ledger. They answer one question, subsystem by subsystem:

> **What does the best work on earth do here, what do we do, what is the gap in numbers, and
> what is the cheapest experiment that would close it?**

| | |
|---|---|
| [`control.md`](control.md) | the loops, the allocation, the vehicle model |
| [`vision.md`](vision.md) | perception, and the model-production pipeline |
| [`localization.md`](localization.md) | the estimator, its sources, and what happens when it is lost |
| [`planning.md`](planning.md) | the DSL, the executive, and what "robust mission" means elsewhere |
| [`SOTA-GAPS.md`](SOTA-GAPS.md) | **the ledger** — every gap and every candidate move, ranked |
| [`sources/`](sources/) | the raw research dossiers the sweeps produced, kept verbatim — every link that was actually fetched, every table, and every "claim I could not verify" |

## The bar

**Competition-dominant first.** Be the best competition AUV there is — RoboSub, SAUVC, TAC —
then generalise. Where a field-grade method (ocean, hours, no operator) is the real ceiling, the
dossier says so and says what it would cost, but the ranking currency is **points on a run
sheet** and **runs survived**.

**Software-first.** Every move must run on what exists: one SROT board (ESP32 at 500 Hz +
RP2350), a Raspberry Pi 5 with a Hailo-8, two USB cameras, a BNO085 and a Bar30, over one
USB-C cable carrying MAVLink 2 at 115 200 baud (**11 520 B/s**, measured 21.0 % used). Hardware
appears in a dossier only where the research shows a physical ceiling software cannot cross —
and then with the software alternative it beat, stated.

## The rules this pass runs by

1. **No claim without a source and a number.** A citation is a URL or a DOI that was actually
   fetched. "X is better" without a number is not a finding.
2. **Every candidate move carries its falsifier** — the experiment that would show it does *not*
   help us — written before the move is ranked.
3. **Truth, not agreement.** Comparing a new method against the incumbent measures agreement and
   cannot rank them. Construct the case where truth is known.
4. **Our constraints are part of the comparison.** A method needing a GPU, a DVL or 200 W is not
   "better" for Mongla — it is a different vehicle.
5. **Retractions stay.** Anything rejected here is recorded with the reason so it is never
   re-researched. Same rule as [`ROADMAP.md`](../ROADMAP.md) §6.
6. **Ask whether it can not exist** before ranking it. A wide interface means the abstraction is
   wrong.

## The shape of a dossier

```
## What we do today          file:line, from the tree — never from docs
## What the best work does   cited, with numbers
## The gap                   a number, or "unmeasured — here is the measurement"
## Candidate moves           each with: cost on our hardware · the falsifier · the risk
## Rejected                  with the reason, so it is not re-opened
```

## The ranking rubric

`SOTA-GAPS.md` scores every move by **value ÷ (risk × effort)**:

| term | scale | meaning |
|---|---|---|
| **value** | 1–5 | points on a run sheet, or runs that survive a failure they would otherwise lose |
| **risk** | 1–5 | chance it makes the vehicle worse, or costs a run to find out |
| **effort** | 1–5 | 1 ≈ a day, 5 ≈ a month of work and a measurement campaign |

A move that cannot be tested before the vehicle is in water says so in its own row. A move whose
falsifier cannot be designed does not get a row — it gets a research question.

## What this is not

Not a bug register — [`BUGS.md`](../BUGS.md) owns that. Not a status file —
[`ROADMAP.md`](../ROADMAP.md) owns that. Not a wish list: an item here has a source, a number and
an experiment, or it is not here.

Related: [`measured-bars.md`](../measured-bars.md) (every shipped constant and the measurement
behind it) · [`scouting/`](../scouting/README.md) (what specific competitors do).

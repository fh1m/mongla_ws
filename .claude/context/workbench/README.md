# The workbench — what is being measured, and what it said

> **What you cannot measure you cannot improve.** This folder is the working surface between a
> ranked idea and a shipped constant. Three files, one job each, all **append-only in spirit**:
> a row is never deleted, only ticked and given its number.

| file | holds |
|---|---|
| [`BENCH.md`](BENCH.md) | every test provable **dry** — no water, mostly no thrusters — with its status and its result |
| [`POOL.md`](POOL.md) | one section per pool session: the agenda written **before** travel, and what each item produced |
| [`RESEARCH-OWED.md`](RESEARCH-OWED.md) | the questions the literature sweeps could not answer, precise enough that a future session re-runs exactly what is missing |

## How a row moves

```
ranked in SOTA-GAPS.md  →  scheduled here (BENCH or POOL)  →  RUN  →  a number
        ↓                                                              ↓
   value ÷ (risk × effort)                          measured-bars.md, which tests read
        ↓                                                              ↓
                                     the SOTA-GAPS row closes, citing the number
```

**A row closes on a measurement, never on merged code.** A result that kills the idea is a good
result and is written down the same way — this project keeps its retractions.

## The three states

| state | means |
|---|---|
| `[ ]` **open** | not run |
| `[~]` **blocked** | the run is designed, waiting on hardware, a merge, or a venue — the blocker is named in the row |
| `[x]` **done** | run, with the number in the row and in `measured-bars.md` |

## Rules

1. **Write the expected result before running.** A test whose outcome you cannot state in advance
   is not yet an experiment.
2. **Write the number down the same day.** A number recorded tomorrow is a number measured twice.
3. **Never quote a bench number as a water number.** This stack has three states — BENCH, BUILT,
   WATER — and blurring them is how a team finds out at the venue.
4. **A guard that has never failed is not a guard.** Injection-verify anything that claims to
   protect something.

Related: [`../sota/SOTA-GAPS.md`](../sota/SOTA-GAPS.md) (what is worth doing and why) ·
[`../measured-bars.md`](../measured-bars.md) (what is already measured) ·
[`../ROADMAP.md`](../ROADMAP.md) (status) · [`../BUGS.md`](../BUGS.md) (defects).

# Pool — one section per session, agenda written before travel

> A pool day improvised is a pool day wasted. Each session gets its agenda **before** the van is
> loaded, every item gated on the one before it, and every item produces **a number or a bag**.
> States: `[ ]` open · `[~]` blocked · `[x]` done. Method: [`README.md`](README.md).

## The standing order (every session, in this sequence)

| # | item | gate to continue |
|---|---|---|
| 1 | **Deck checks** — `bringup_check --srot`, the depth-sign check (B-1), arm/disarm, kill switch | all green, or the day becomes a bench day at the poolside |
| 2 | **The two armed depth checks** | ⛔ until these pass, every `SROT_MOVE` verb is denied by the board — nothing below is reachable |
| 3 | **Closed-loop depth hold**, logged | depth holds within a stated band; `DEPTH_ERR` vs depth captured for the deferred near-surface question |
| 4 | **Velocity truth run** — known distances on a line, filmed from outside, several speeds | this is both the flow-scale check and the label set a learned velocity model needs |
| 5 | **Flow validation** at taped range; `floor_range` truth | the numbers agree with tape, or the refusals are correct |
| 6 | **Height sources cross-check** — pressure + floor plane vs tile grid vs any new sensor | three sources that *can* disagree is the point |
| 7 | **Course survey** (`course_survey`) | positions written to the deck copy |
| 8 | **Vision align on a real prop**, then `/lock` actuation behind its parameter | only after everything above |

**Every session ends with**: bags pulled, scorecards filed, `measured-bars.md` rows written **the
same day**, and `capability-map.md` rows moved BENCH → WATER. A number not written down that day
is a number that will be measured twice.

---

## Session 01 — first water  `[ ]` not yet scheduled

**The vehicle has never been in water.** This session exists to close the gate that blocks
everything else, and to come back with the measurements three later rows depend on.

**Hardware:** exactly what exists today — SROT board, Pi 5 + Hailo-8, two USB cameras, BNO085,
Bar30. **No new sensors.**

**Must-close:**
- [ ] The two armed depth checks (unblocks every move verb)
- [ ] First closed-loop depth hold, logged
- [ ] Velocity truth run — the label set
- [ ] Flow scale against tape (the bench rig currently measures 103.4 %)

**Nice-to-have, only if all of the above pass:**
- [ ] `floor_range` at taped range
- [ ] A vision align against any real prop

**Bring:** tape measure; a line and weights for the velocity run; an external camera with a clock
in frame; spare batteries; the laptop with the bags directory pre-created.

**Result:** _(not yet run)_

---

## Template for the next session

```
## Session NN — <the one thing it must achieve>  [ ]

**Hardware on the vehicle:** ...
**Must-close:**  - [ ] ...
**Bring:** ...
**Result:** filled in the same day, with the numbers and the bag names
**Landed in:** measured-bars.md rows ..., capability-map rows moved ...
```

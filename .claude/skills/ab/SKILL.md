---
name: ab
description: Rigorous A/B measurement for a vision or control change in this repo. Use when a claim says a knob, optimisation or fix helps or hurts, before any result is written to the ledger or a commit.
---

# /ab

A claim that a change helps or hurts is not evidence until a run could have shown the
opposite. This skill is the procedure for getting that run.

## Steps

1. **State the hypothesis and the one variable.** Write it in one sentence, with the
   metric and the direction you expect. If two things change, split the test.
2. **Prove the variable is wired through.** Grep for hardcoded overrides of that name
   (a literal `True`, a launch default, a `set` in a file that runs later). Then print
   the effective value from inside the running process. A test whose knob never changes
   behaviour is a null result, and it proves nothing about the change.
3. **Use real data.** Real frame pairs, real clips, real water or bench footage. Synthetic
   warps of one frame are not evidence unless the user asked for them. Note the source in
   the ledger.
4. **Make the harness able to fail.** Run it once on a deliberately broken input (random
   descriptors, a swapped pair, a sign flipped) and confirm it scores badly. Only then
   trust its pass.
5. **Run it repeatedly.** At least 3 runs for each arm, 5 when a gain is small. Report the
   median and the spread. Reject a gain that sits inside the spread.
6. **Wrap every run in a timeout** and record the thread and BLAS settings. The launch
   files pin `OPENBLAS_NUM_THREADS=1`; a run that changes it says so in the result.
7. **Record the verdict, negative results included,** in `measured-bars.md` (the shipped
   constants and their evidence) or in the ledger that the change names. A tested null is
   a finding.

## Subagents

Only when the user asks for one, and always `haiku-researcher`. Use one as a skeptic: give
it the claimed gain and the raw numbers, and ask it to find the reason the gain is false.

## Handoff

For a run that will span a session limit, write `.agent/HANDOFF.md` (under 60 lines): the
runs done, the hypotheses still open, the exact commands that reproduce them, and the next
three steps. This is a handoff note, not a task list; tasks stay in beads.

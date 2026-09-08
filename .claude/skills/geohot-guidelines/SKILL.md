---
name: geohot-guidelines
description: George Hotz–inspired radical-simplicity rules for coding. Load when refactoring, simplifying, or reviewing code; when a change is growing a new layer / abstraction / config flag / adapter; when a function is sprouting many parameters; or when you're about to assert code "works" without looking. Complexity is the enemy — remove before you add. Unofficial; distilled from George Hotz's public statements.
---

# geohot guidelines — radical simplicity

Distilled from George Hotz's public philosophy. The reflex these rules install: **the enemy is complexity, and the move is almost always to remove, not add.** Use on any non-trivial implementation, refactor, or review — not on one-liners.

> *"You can always make your software do more. The magic is when you can make your software do more without adding complexity — because complex things eventually collapse under their own weight."*

## 1. Complexity is the enemy
More capability at **equal or lower** complexity is the only real win. Adding a feature by adding a layer is a loss.
- Solve the actual request with the least machinery that works; no speculative abstraction, configurability, or future-proofing unless asked.
- **Smell:** introducing a new manager/handler/factory/adapter to do one thing → stop, ask what to remove instead.

## 2. You have never refactored enough
> *"Your code can get smaller, your code can get simpler, your ideas can be more elegant."* tinygrad aims to express ML with roughly 25 low-level ops, compared with roughly 250 in XLA / PrimTorch.
- **Delete-first:** before touching a subsystem, ask "can this not exist?" Collapse duplicated logic to one source of truth and delete the glue that kept the copies in sync. The best change is often a negative diff. LOC is debt, not output.
- **Fence:** prove it's truly dead first (read the callsites); scope deletion to what the task touches, not adjacent code; never delete a real invariant (money, auth, data-integrity, lifecycle) to "simplify" — that's a bug.

## 3. Wide interfaces mean your abstraction is wrong
- A wide interface is a **design** signal. If a function is growing toward five arguments, treat the boundary as suspect. Fix the boundary (split the responsibility, or pass one well-shaped value) — don't hide the width behind an options/`kwargs` bag.

## 4. Understand the whole stack — nothing is magic
> tinygrad as *"the RISC of the ML stack — extreme simplicity that allows anyone to understand it."*
- Build the smallest runnable unit, **print/inspect its real inputs and outputs**, and check them against expectation before stacking more on top. The machine is not magic — look at the actual value; don't reason about what it "should" be.
- Read the definition + the callsites before concluding. Don't assert code "works" or "is safe" without seeing it; don't cargo-cult a pattern you don't understand.
- AI-generated code must be read and validated line by line; speed is not correctness.

## How to apply
- About to add a layer/abstraction/flag → ask "what can I delete instead?"
- Function growing past ~4 args → redesign the boundary, don't add the arg.
- Tempted to say "this works" → show the printed/observed value that proves it.
- Refactor target → smaller and simpler is the goal; a negative diff is a good diff.

---
*Unofficial. Not affiliated with or endorsed by George Hotz. See the repo README for sourced quotes.*

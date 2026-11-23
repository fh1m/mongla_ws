# Contributing to Mongla

Thank you for reading the code closely enough to want to change it. This is a vehicle: a
change that looks like a simplification has, more than once, been a guard someone removed.
These are the rules the repository holds itself to, and the checks that hold it.

## Before you write code

- **Say which state a thing is in.** RUNS TODAY (on the bench or on footage), BUILT, NEVER
  FLOWN, or BLOCKED (on hardware, water or a firmware merge). Never blur them; a water status
  belongs to the platform it was earned on.
- **Write the bar before the run.** If a change claims to help, write down the number that
  would prove it wrong, then measure on real data. A quality number a method computes from
  the data it just fit is not evidence.
- **A missing number is NaN, never 0.0.** Rendering absence as zero recreates the defect this
  codebase has paid for most.
- **Nothing ships unreachable.** A module is imported by something that runs, or it names the
  consumer it waits for. A test is not a caller.
- **A missing low-level feature is a pull request to the firmware repository**, not a host
  workaround. We never commit to the firmware, ground-station or flasher repositories, and
  they never commit here.

## Making the change

1. Branch from `main`. One pull request per finding.
2. Change the docs in the same commit as the code they describe. A removal is finished only
   when every file that named the removed thing says what is true now.
3. Every shipped threshold goes in `.claude/context/measured-bars.md`, with the method and the
   conditions; every defect goes in `.claude/context/BUGS.md`.
4. A new guard is injection-verified: break the thing it guards, watch it fail, restore it.
   A test that has never failed against a real defect is not a guard.

## The checks

```bash
./build_mongla.sh && source install/setup.bash
python3 -m pytest -q                       # every suite; a full run also publishes the counts
python3 tools/ci/run_repo_integrity.py     # the integrity gate, as CI runs it
```

Four checks are required and none may be talked past: `ROS humble` and `ROS jazzy` build and
test the whole stack in a clean container; the integrity gate runs every guard, including
ruff (pyflakes rules), shellcheck and actionlint; and the workflow contract checks that
none of the required workflows can be skipped.

## The pull request description

Fill the template. If the change compromises because data is missing, say so under **Compromises** and link the upstream ask.

## Where to start

- [`docs/the-shift.md`](../docs/the-shift.md): why reflexes live on the board and thinking on the Pi.
- [`.claude/context/ROADMAP.md`](../.claude/context/ROADMAP.md): where we are and what is blocked.
- [`.claude/context/BUGS.md`](../.claude/context/BUGS.md): every defect, including the ones we took back.
- Issues labelled `proposal` are open questions with their evidence; Discussions hold the
  measured write-ups.

Questions go to Discussions; a security problem goes through [SECURITY.md](../SECURITY.md),
never a public issue.

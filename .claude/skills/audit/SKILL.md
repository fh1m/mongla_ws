---
name: audit
description: Line-by-line code audit of a named package and everything that depends on it, delivered as one PR per finding. Use when the user asks to audit, review every file, or find bugs and untested code across a package.
---

# /audit

An audit reads the code, not the prose about it. A review of "the package" that reads a
diff, or that trusts a README, is a different job and must be named as one.

## Steps

1. **Restate the scope before reading anything.** List the exact files: every source file
   under the named packages and the packages that import them, with line counts. State the
   base branch. Show the list and wait for the user's OK.
2. **Read the code only.** Docs, READMEs, comments and prior summaries are not sources of
   truth. A claim in a comment is a hypothesis to check against the code path it names.
3. **Look for these, in order of cost to the vehicle:**
   - a parameter declared but never read, or a node name set but ignored;
   - a log line or docstring that claims something the code does not do;
   - a wrong sign, unit or frame, including a physics sign that a test cannot fail on;
   - a race, an unguarded shared state, or a timeout that never fires;
   - a deploy or rsync path that copies the wrong tree;
   - dead, duplicated or unreachable code (`test_no_capability_is_built_and_unreachable.py`
     enforces this for modules; a test is not a caller).
4. **For each finding, write a check that fails on the defect** and confirm it fails. Then
   fix, and confirm it passes and the package suite stays green. Run the check alone and
   the suite together; report both.
5. **One PR per finding.** The description names file:line, the evidence, and the before
   and after test output. Fixes go into PRs, not the local working tree, unless the user
   says otherwise.
6. **Write the coverage record** (`audit/coverage.md` on the audit branch): every file in
   the list marked read, with its findings and PR links. Do not report done while a file
   is unmarked or a finding has neither a PR nor a written reason for not fixing it.

## Subagents

When the list is long, split it into shards of about 1500 lines, one `haiku-researcher`
per shard, each in its own worktree. Only when the user asks for subagents, and never any
other model. A separate skeptic pass tries to falsify each finding before its PR is opened.

## Handoff

After each shard or PR, update `.agent/HANDOFF.md` (under 60 lines): files read, findings
open, PRs opened, and the next three steps. A fresh session must be able to resume from it.

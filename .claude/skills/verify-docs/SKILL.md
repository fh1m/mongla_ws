---
name: verify-docs
description: Check a package's external-API usage against current online docs and report drift. Use before a release/cleanup or after a dependency bump.
disable-model-invocation: true
---

# Verify docs

**Usage:** `/verify-docs <package>`  (e.g. `duburi_control`, `duburi_vision`)

Dispatch the `doc-verifier` agent over `src/<package>/` and summarize API-drift findings.

## Steps

1. Resolve the target: `src/<package>/` (default to all six packages if none given).
2. Launch the `doc-verifier` agent with that scope. It greps external-API call sites
   (pymavlink, ultralytics, supervision, onnxruntime, cv2, rclpy) and compares each
   against current upstream docs (context7 MCP first, then allow-listed WebFetch domains).
3. Summarize the agent's report grouped by library, each finding with `file:line` and the
   authoritative source URL.
4. Do **not** auto-fix. Present drift; let the user choose what to change.

Pair with the `context-doc-sync` agent when you also want `.claude/context/*.md` checked
against the code (different axis: internal docs vs source, not source vs upstream).

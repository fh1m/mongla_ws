#!/usr/bin/env bash
# PostToolUse hook: the instant checks on an edited file. Python: byte-compile,
# then the integrity gate's own lint (ruff F,E9: names that do not resolve or
# are never used). Shell: shellcheck at the gate's level. Each takes tens of
# milliseconds; a workflow gets actionlint the same way. A missing tool is skipped, since the gate runs them anyway.
# Reads the hook JSON on stdin; advisory only (always exits 0).
f=$(python3 -c "import sys,json; print((json.load(sys.stdin).get('tool_input') or {}).get('file_path',''))" 2>/dev/null)
case "$f" in
  *.py)
    if ! python3 -m py_compile "$f" 2>/tmp/mongla_pycheck.err; then
      echo "[Hook] py_compile FAILED: $f"
      tail -5 /tmp/mongla_pycheck.err
    elif command -v ruff >/dev/null 2>&1; then
      out=$(ruff check --isolated --select F,E9 --target-version py310 --no-cache \
            --output-format concise "$f" 2>&1) || { echo "[Hook] ruff:"; echo "$out" | head -12; }
    fi
    ;;
  *.sh)
    if command -v shellcheck >/dev/null 2>&1; then
      out=$(shellcheck -S warning -f gcc "$f" 2>&1) || { echo "[Hook] shellcheck:"; echo "$out" | head -12; }
    fi
    ;;
  */.github/workflows/*.yml)
    if command -v actionlint >/dev/null 2>&1; then
      out=$(SHELLCHECK_OPTS='-S warning' actionlint -no-color "$f" 2>&1) || { echo "[Hook] actionlint:"; echo "$out" | head -12; }
    fi
    ;;
esac
exit 0

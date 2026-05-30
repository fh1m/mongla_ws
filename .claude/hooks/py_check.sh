#!/usr/bin/env bash
# PostToolUse hook: byte-compile an edited Python file for an instant syntax check.
# Reads the hook JSON on stdin; advisory only (always exits 0).
f=$(python3 -c "import sys,json; print((json.load(sys.stdin).get('tool_input') or {}).get('file_path',''))" 2>/dev/null)
case "$f" in
  *.py)
    if ! python3 -m py_compile "$f" 2>/tmp/duburi_pycheck.err; then
      echo "[Hook] py_compile FAILED: $f"
      tail -5 /tmp/duburi_pycheck.err
    fi
    ;;
esac
exit 0

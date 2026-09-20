#!/usr/bin/env python3
"""PreToolUse hook: block Edit/Write targeting the colcon-generated install/ tree.

Edits under install/ are silently overwritten on the next `./build_mongla.sh`, so they
are always a mistake — the real source lives in src/. Reads the hook JSON on stdin and
exits 2 (block) when the target path contains '/install/'.
"""
import json
import sys

try:
    payload = json.load(sys.stdin)
except Exception:
    sys.exit(0)  # malformed payload -> don't block

file_path = (payload.get("tool_input") or {}).get("file_path", "")

if "/install/" in file_path:
    sys.stderr.write(
        f"[Hook] BLOCKED: {file_path}\n"
        "  -> this is in the colcon-generated install/ tree; edits are lost on rebuild.\n"
        "  -> edit the source under src/ instead.\n"
    )
    sys.exit(2)

sys.exit(0)

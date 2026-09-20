#!/usr/bin/env bash
# PostToolUse hook: run the matching pytest file for an edited source file.
#
# Targeted + fast: editing motion_forward.py runs only test_motion_forward.py (if it
# exists); editing a test file runs that test. Advisory only (always exits 0) so it never
# blocks an edit. Prepends src/<pkg> to PYTHONPATH so the EDITED source is imported, not
# the stale colcon-installed copy (build_mongla.sh copies, it does not symlink-install).
#
# Path-independent: resolves the workspace from CLAUDE_PROJECT_DIR (set by the harness for
# hook subprocesses) or from this script's location, so it works for any teammate's checkout.
WS="${CLAUDE_PROJECT_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"

f=$(python3 -c "import sys,json; print((json.load(sys.stdin).get('tool_input') or {}).get('file_path',''))" 2>/dev/null)

# Only react to edits inside this workspace's src/ tree.
case "$f" in
  "$WS"/src/*) ;;
  *) exit 0 ;;
esac

pkg=$(echo "$f" | grep -oP "src/\K[^/]+")
[ -z "$pkg" ] && exit 0

base=$(basename "$f" .py)
case "$base" in
  test_*) testfile="$WS/src/$pkg/test/$base.py" ;;
  *)      testfile="$WS/src/$pkg/test/test_$base.py" ;;
esac
[ -f "$testfile" ] || exit 0   # no matching test -> nothing to run

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null
# shellcheck disable=SC1091
source "$WS/install/setup.bash" 2>/dev/null
# Edited source must shadow the installed copy.
export PYTHONPATH="$WS/src/$pkg:${PYTHONPATH}"

echo "[Hook] pytest $(basename "$testfile")  (src-shadowed)"
# -p no:anyio: a stale anyio pytest11 entrypoint in this env errors at plugin
# load (ModuleNotFoundError: _pytest.scope). Disabling it is safe when absent.
( cd "$WS" && python3 -m pytest -q -p no:anyio "$testfile" 2>&1 | tail -20 )
exit 0

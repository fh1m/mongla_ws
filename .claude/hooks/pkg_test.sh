#!/usr/bin/env bash
# PostToolUse hook: run the matching pytest file for an edited source file.
#
# Targeted + fast: editing motion_vision/ runs only test_motion_vision/ (if it
# exists); editing a test file runs that test. Editing a DOC (or a skill, agent,
# launch file or config) runs the two doc guards instead, because docs drift from
# code silently: the retired-stack sweep and the book's reachability/title checks.
# Advisory only (always exits 0) so it never blocks an edit. Prepends src/<pkg> to PYTHONPATH so the EDITED source is imported, not
# the stale colcon-installed copy (build_mongla.sh copies, it does not symlink-install).
#
# Path-independent: resolves the workspace from CLAUDE_PROJECT_DIR (set by the harness for
# hook subprocesses) or from this script's location, so it works for any teammate's checkout.
WS="${CLAUDE_PROJECT_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"

f=$(python3 -c "import sys,json; print((json.load(sys.stdin).get('tool_input') or {}).get('file_path',''))" 2>/dev/null)

case "$f" in
  "$WS"/install/*|"$WS"/build/*|"$WS"/log/*) exit 0 ;;
esac

# A doc, skill, agent or config: run the doc guards (deterministic, ~5 s, no ROS).
case "$f" in
  *.md|*.yaml|*.yml|*.json|*.html|*.launch.py|*.xml|*.ini|*.sh)
    case "$f" in
      "$WS"/*)
        echo "[Hook] doc guards (retired stack, book reachability and titles, wiki round trip)"
        ( cd "$WS" && python3 -m pytest -q -p no:anyio -p no:cacheprovider \
            test/test_the_retired_stack_stays_retired.py \
            src/mongla_manager/test/test_docs_contract.py \
            test/test_the_wiki_is_built_from_the_book.py 2>&1 | tail -15 )
        ;;
    esac
    case "$f" in *.launch.py) ;; *) exit 0 ;; esac ;;
esac

# Only react to source edits inside this workspace's src/ tree.
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

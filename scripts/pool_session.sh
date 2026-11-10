# pool_session.sh -- pin one run folder for a whole pool session.
#
#   SOURCE this (don't execute) in EVERY terminal of a session so the bag,
#   the mission scorecard, and every node's rcl log land in the SAME folder:
#
#     source scripts/pool_session.sh gate_practice_am
#
#   It exports:
#     MONGLA_RUN_DIR = ~/mongla_runs/<label>     (bags + scorecards live here)
#     ROS_LOG_DIR    = $MONGLA_RUN_DIR/logs       (rcl logs for nodes started here)
#
#   Use the SAME <label> in each terminal → one directory has the whole run.
#   No label re-uses an already-exported MONGLA_RUN_DIR, else makes a
#   timestamped one. Purely additive: without sourcing this, everything still
#   defaults to a flat ~/mongla_runs (unchanged behaviour).
#
# zsh + bash compatible. Sourcing is required (it must mutate YOUR shell env).

# --- refuse to be executed (it would export into a throwaway subshell) -------
_dbr_sourced=0
if [ -n "${ZSH_VERSION:-}" ]; then
  case "${ZSH_EVAL_CONTEXT:-}" in *:file) _dbr_sourced=1 ;; esac
elif [ -n "${BASH_VERSION:-}" ]; then
  (return 0 2>/dev/null) && _dbr_sourced=1
fi
if [ "$_dbr_sourced" -ne 1 ]; then
  echo "pool_session.sh must be SOURCED, not executed:" >&2
  echo "    source scripts/pool_session.sh <label>" >&2
  exit 1
fi
unset _dbr_sourced

# --- resolve the session dir -------------------------------------------------
_dbr_root="${MONGLA_RUNS_ROOT:-$HOME/mongla_runs}"
_dbr_label="${1:-}"
if [ -n "$_dbr_label" ]; then
  # sanitize the label the same way pool_record.sh does (can't escape the root)
  _dbr_label="$(printf '%s' "$_dbr_label" | tr -c 'A-Za-z0-9._-' '_')"
  MONGLA_RUN_DIR="$_dbr_root/$_dbr_label"
elif [ -n "${MONGLA_RUN_DIR:-}" ]; then
  : # keep the one already exported in this shell
else
  MONGLA_RUN_DIR="$_dbr_root/session_$(date +%Y%m%d_%H%M%S)"
fi

export MONGLA_RUN_DIR
export ROS_LOG_DIR="$MONGLA_RUN_DIR/logs"
mkdir -p "$ROS_LOG_DIR"

# --- which software produced this run ----------------------------------------
# A run is evidence only for the code that ran it, and nothing wrote that down
# before: the RoboSub 2026 runs can be tied to a commit only by date (water
# lineage, W-RS26). On Kutum the workspace is a release and carries
# release.json; anywhere else, the commit and whether it was dirty.
if [ -n "${ZSH_VERSION:-}" ]; then
  _dbr_self="${(%):-%x}"
else
  _dbr_self="${BASH_SOURCE[0]}"
fi
_dbr_ws="$(cd "$(dirname "$_dbr_self")/.." && pwd -P)"
{
  echo "workspace: $_dbr_ws"
  echo "recorded: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  if [ -f "$_dbr_ws/release.json" ]; then
    cat "$_dbr_ws/release.json"
  else
    echo "commit: $(git -C "$_dbr_ws" rev-parse HEAD 2>/dev/null || echo unknown)"
    if ! git -C "$_dbr_ws" diff --quiet HEAD 2>/dev/null; then
      echo "dirty: yes (the run used code no commit has)"
    fi
  fi
} >> "$MONGLA_RUN_DIR/software.txt"

echo "▸ Mongla run session pinned:"
echo "    MONGLA_RUN_DIR = $MONGLA_RUN_DIR   (bags + scorecards)"
echo "    ROS_LOG_DIR    = $ROS_LOG_DIR      (node logs)"
echo "    software       = $MONGLA_RUN_DIR/software.txt"
echo "  Source this with the SAME label in every terminal of this run."

unset _dbr_root _dbr_label _dbr_self _dbr_ws

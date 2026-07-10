# pool_session.sh -- pin one run folder for a whole pool session.
#
#   SOURCE this (don't execute) in EVERY terminal of a session so the bag,
#   the mission scorecard, and every node's rcl log land in the SAME folder:
#
#     source scripts/pool_session.sh gate_practice_am
#
#   It exports:
#     DUBURI_RUN_DIR = ~/duburi_runs/<label>     (bags + scorecards live here)
#     ROS_LOG_DIR    = $DUBURI_RUN_DIR/logs       (rcl logs for nodes started here)
#
#   Use the SAME <label> in each terminal → one directory has the whole run.
#   No label re-uses an already-exported DUBURI_RUN_DIR, else makes a
#   timestamped one. Purely additive: without sourcing this, everything still
#   defaults to a flat ~/duburi_runs (unchanged behaviour).
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
_dbr_root="${DUBURI_RUNS_ROOT:-$HOME/duburi_runs}"
_dbr_label="${1:-}"
if [ -n "$_dbr_label" ]; then
  # sanitize the label the same way pool_record.sh does (can't escape the root)
  _dbr_label="$(printf '%s' "$_dbr_label" | tr -c 'A-Za-z0-9._-' '_')"
  DUBURI_RUN_DIR="$_dbr_root/$_dbr_label"
elif [ -n "${DUBURI_RUN_DIR:-}" ]; then
  : # keep the one already exported in this shell
else
  DUBURI_RUN_DIR="$_dbr_root/session_$(date +%Y%m%d_%H%M%S)"
fi

export DUBURI_RUN_DIR
export ROS_LOG_DIR="$DUBURI_RUN_DIR/logs"
mkdir -p "$ROS_LOG_DIR"

echo "▸ Duburi run session pinned:"
echo "    DUBURI_RUN_DIR = $DUBURI_RUN_DIR   (bags + scorecards)"
echo "    ROS_LOG_DIR    = $ROS_LOG_DIR      (node logs)"
echo "  Source this with the SAME label in every terminal of this run."

unset _dbr_root _dbr_label

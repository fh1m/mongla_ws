# shellcheck shell=bash
# _record_lib.sh -- signal handling shared by the session recorders.
#
# ⛔ A backgrounded child of a NON-INTERACTIVE shell inherits SIGINT set to
# IGNORE, and an ignored disposition survives exec -- no later `trap` can undo
# it. Started over ssh or with `nohup ... &`, `ros2 bag record` therefore
# ignores every `kill -INT`, `wait` never returns, and the bag is never
# finalized. The recorder looks like it is working the entire time.
#
# Measured twice on 2026-09-24: a bench record still open 30 minutes after its
# countdown ended, and a replay that had finished playing but would not exit.

# Run a command with the default signal dispositions restored.
run_resettable() {
  python3 -c 'import os, signal, sys
signal.signal(signal.SIGINT, signal.SIG_DFL)
signal.signal(signal.SIGTERM, signal.SIG_DFL)
os.execvp(sys.argv[1], sys.argv[1:])' "$@"
}

# stop <pid> <name> -- INT, then TERM, then KILL. Never blocks forever.
# rosbag2 finalizes (writes metadata.yaml) on INT or TERM; only KILL loses it.
stop() {
  local pid="$1" name="$2" i
  [ -n "$pid" ] || return 0
  kill -INT "$pid" 2>/dev/null || return 0
  for i in $(seq 20); do
    kill -0 "$pid" 2>/dev/null || { echo "  $name stopped cleanly"; return 0; }
    sleep 1
  done
  echo "  $name ignored SIGINT -- sending SIGTERM"
  kill -TERM "$pid" 2>/dev/null
  for i in $(seq 15); do
    kill -0 "$pid" 2>/dev/null || { echo "  $name stopped on SIGTERM"; return 0; }
    sleep 1
  done
  echo "  $name ignored SIGTERM -- SIGKILL (the bag may be unfinalized)" >&2
  kill -KILL "$pid" 2>/dev/null
}

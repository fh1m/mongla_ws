# shellcheck shell=bash
# _record_lib.sh -- signal handling shared by the session recorders.
#
# ⛔ A backgrounded child of a NON-INTERACTIVE shell inherits SIGINT set to
# IGNORE, and an ignored disposition survives exec -- no later `trap` can undo
# it. Started over ssh or with `nohup ... &`, `ros2 bag record` therefore
# ignores every `kill -INT`, `wait` never returns, and the bag is never
# finalized. The recorder looks like it is working the entire time.
#
# Measured three times on 2026-09-24: a bench record still open 30 minutes after
# its countdown ended, a replay that had finished playing but would not exit,
# and -- the expensive one -- a `ros2 launch` that ignored SIGINT and therefore
# ORPHANED its five nodes when it was killed. Those nodes kept the Hailo
# claimed, and the only symptom was HAILO_OUT_OF_PHYSICAL_DEVICES raised an
# hour later by a tool with nothing to do with the vision graph.
#
# So EVERY long-lived child goes through run_resettable, not just the bag
# recorder: `ros2 launch` forwards SIGINT to its nodes, and a launcher that
# cannot receive one cannot forward it.

# ⛔ AND RESETTING THE DISPOSITION IS NOT ENOUGH ON ITS OWN. Measured: with the
# shim in place, `ros2 launch` still did not exit on SIGINT within 20 s, so the
# escalation sent SIGTERM -- which the launcher does NOT forward, orphaning
# every node it started. Signalling the launcher relies on the launcher
# choosing to pass the signal on.
#
# So a long-lived graph is started in its OWN PROCESS GROUP and the GROUP is
# signalled. Then every node receives the signal directly and none of it
# depends on the launcher's cooperation.

# Start a command in its own process group, with default signal dispositions.
# Echoes the pid, which is also the process-GROUP id.
# ⛔ TWO APPROACHES TRIED AND REJECTED, because signalling the LAUNCHER only
# works if the launcher cooperates:
#   1. `run_resettable ros2 launch` -- measured: still did not exit within 20 s
#      of SIGINT, so the escalation sent SIGTERM, which ros2 launch does NOT
#      forward, and its nodes were orphaned anyway.
#   2. `setsid cmd &` to signal the process GROUP -- os.setsid() failed in this
#      context and the shim died at once, so $! named a pid that never existed
#      and stop() silently signalled nothing. Five nodes survived every time.
#
# What works is not asking anyone to forward anything: walk the process tree
# and signal every descendant directly. `ros2 launch` starts each node as its
# own child, so the tree IS the node list.
# ⛔ AND THE TREE MUST BE SNAPSHOT BEFORE ANYTHING DIES. Re-walking `pgrep -P`
# at each escalation looks equivalent and is not: the moment the parent exits,
# its children are REPARENTED to init and the walk can never find them again.
# Children that ignore SIGINT therefore outlive the parent and are never
# signalled a second time. Caught by
# test_stop_tree_leaves_nothing_running.py, which spawns children that ignore
# SIGINT exactly as a non-interactive shell's background jobs do: 3 of 3
# survived. The vehicle had hidden it, because ROS nodes die on the first INT.
descendants() {          # every descendant pid, deepest first
  local pid="$1" kid
  for kid in $(pgrep -P "$pid" 2>/dev/null); do
    descendants "$kid"
    echo "$kid"
  done
}

kill_tree() {            # kill_tree <sig> <pid>...
  local sig="$1" pid
  shift
  for pid in "$@"; do
    kill "-$sig" "$pid" 2>/dev/null || true
  done
}

# stop_tree <pid> <name> -- INT the whole tree, then TERM, then KILL, and
# report SURVIVORS. Never blocks forever.
stop_tree() {
  local pid="$1" name="$2" i tree alive left
  [ -n "$pid" ] || { echo "  $name: no pid recorded"; return 0; }
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "  $name: already gone"
    return 0
  fi
  # Snapshot first. This is the whole fix.
  tree="$(descendants "$pid") $pid"

  kill_tree INT $tree
  # Seconds to wait for a graceful INT before escalating. A real graph needs
  # this long; the test spawns children that ignore SIGINT on purpose, and
  # would otherwise spend the full grace period three times over -- a guard
  # too slow to run habitually is not a guard.
  for i in $(seq "${STOP_TREE_GRACE_S:-20}"); do
    alive=0
    for p in $tree; do kill -0 "$p" 2>/dev/null && alive=1 && break; done
    [ "$alive" = 0 ] && break
    sleep 1
  done
  kill_tree TERM $tree
  sleep 2
  kill_tree KILL $tree

  # Report what SURVIVES, not that the launcher exited -- "launcher stopped"
  # was true during every run that left five nodes holding the Hailo.
  left=0
  for p in $tree; do kill -0 "$p" 2>/dev/null && left=$((left + 1)); done
  local nodes
  nodes="$(pgrep -f '[d]etector_dual_node|[t]racker_node|[l]ock_node' 2>/dev/null | wc -l)"
  if [ "$left" -gt 0 ] || [ "$nodes" -gt 0 ]; then
    echo "  ⛔ $name: $left tree process(es) and $nodes vision node(s) SURVIVE" >&2
  else
    echo "  $name stopped; no surviving processes"
  fi
}

# Run a command with the default signal dispositions restored.
run_resettable() {
  python3 -c 'import os, signal, sys
signal.signal(signal.SIGINT, signal.SIG_DFL)
signal.signal(signal.SIGTERM, signal.SIG_DFL)
os.execvp(sys.argv[1], sys.argv[1:])' "$@"
}

# stop <pid> <name> -- INT, then TERM, then KILL. Never blocks forever.
# rosbag2 finalizes (writes metadata.yaml) on INT or TERM; only KILL loses it.
# stop <pid> <name> [group] -- pass 'group' when the pid leads its own process
# group (i.e. it came from start_group); the whole group is then signalled.
stop() {
  local pid="$1" name="$2" mode="${3:-}" target i
  [ -n "$pid" ] || return 0
  if [ "$mode" = group ] && kill -0 "-$pid" 2>/dev/null; then
    target="-$pid"          # negative pid = the process GROUP
  else
    target="$pid"
  fi
  if ! kill -INT "$target" 2>/dev/null; then
    # Say so. A silent return here reads exactly like a clean stop, which is
    # how a bad pid hid five surviving nodes.
    echo "  $name: nothing to signal at $target (already gone, or wrong pid)"
    return 0
  fi
  for i in $(seq 20); do
    kill -0 "$pid" 2>/dev/null || { echo "  $name stopped cleanly"; return 0; }
    sleep 1
  done
  echo "  $name ignored SIGINT -- sending SIGTERM"
  kill -TERM "$target" 2>/dev/null
  for i in $(seq 15); do
    kill -0 "$pid" 2>/dev/null || { echo "  $name stopped on SIGTERM"; return 0; }
    sleep 1
  done
  echo "  $name ignored SIGTERM -- SIGKILL (the bag may be unfinalized)" >&2
  kill -KILL "$target" 2>/dev/null
}

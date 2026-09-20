#!/usr/bin/env bash
# pool_record.sh -- record / replay / list Mongla pool-run rosbags.
#
# Every run's artifacts live under one folder (default ~/mongla_runs, override
# with MONGLA_RUN_DIR) -- the SAME tree the mission scorecard writes into, so
# after a pool session everything for a run is in one place.
#
#   scripts/pool_record.sh record [label]        # record the debug allowlist
#   scripts/pool_record.sh record --full [label]  # + raw camera images (BIG)
#   scripts/pool_record.sh replay <bag-dir>       # play a recorded run back
#   scripts/pool_record.sh list                   # list recorded runs
#
# Bags are Foxglove-playable and re-runnable offline -- tune detection conf /
# gains / mission timings from a recorded run without burning pool time.
#
# ponytail: a thin wrapper over `ros2 bag record`; the value is the allowlist
# (keeps bags small / off the raw-image tether firehose) + the shared run tree.
set -euo pipefail

RUN_DIR="${MONGLA_RUN_DIR:-$HOME/mongla_runs}"

# Debug allowlist -- what you actually need to review a run. Raw image_raw and
# camera_info are EXCLUDED by default (bandwidth + disk); image_debug carries the
# detection overlay and is enough for most review. Add them with --full.
# `correspondences` is the EVIDENCE a 6-DoF pose was solved from, and it is
# recorded on purpose: a pose alone is a lossy summary, so a bag without it
# can never separate a bad solve from bad evidence. Replay into `pnp_node`
# with a different `max_reproj_px` and the shot is re-solvable. ~8 kB per
# message at the anchor's 3 Hz, which is nothing beside image_debug.
ALLOW_DEBUG='^/mongla/(state|imu_rates|move/_action/(feedback|status)|vision/[^/]+/(detections|tracks|lock|correspondences|target_pose|image_debug|vis_range.*)|vision/[^/]+/distance.*)$'
ALLOW_FULL='^/mongla/(state|imu_rates|move/_action/(feedback|status)|vision/[^/]+/(detections|tracks|lock|correspondences|target_pose|image_debug|image_raw|camera_info|vis_range.*)|vision/[^/]+/distance.*)$'

usage() { sed -n '2,20p' "$0"; exit "${1:-0}"; }

cmd_record() {
  local regex="$ALLOW_DEBUG" label=""
  while [ $# -gt 0 ]; do
    case "$1" in
      --full) regex="$ALLOW_FULL"; shift ;;
      -h|--help) usage 0 ;;
      *) label="$1"; shift ;;
    esac
  done
  label="${label:-run}"
  local ts out
  ts="$(date +%Y%m%d_%H%M%S)"
  # Sanitize the label so it can't escape the run dir or break the path.
  label="$(printf '%s' "$label" | tr -c 'A-Za-z0-9._-' '_')"
  out="$RUN_DIR/bag_${label}_${ts}"
  mkdir -p "$RUN_DIR"
  echo "▸ recording → $out"
  echo "  allowlist: $regex"
  echo "  (Ctrl-C to stop cleanly; bag is finalized on stop)"
  # -s mcap: write MCAP (not the default sqlite3 .db3). MCAP is Foxglove's
  # native format — the bag opens as a local file directly in the Foxglove
  # desktop app WITH embedded schemas (needed for our custom MonglaState msg),
  # so "record a run, drag it into Foxglove" just works. Needs the
  # ros-humble-rosbag2-storage-mcap plugin (exec_depend in mongla_manager).
  # --regex matches topic names; -o sets the output bag dir.
  # --include-hidden-topics: /mongla/move/_action/{feedback,status} are HIDDEN
  # (the _action namespace); without this the regex matches them but bag record
  # still skips them. The regex keeps the set filtered, so we only get the
  # matching hidden topics, not the full _action firehose.
  ros2 bag record -s mcap --regex "$regex" --include-hidden-topics -o "$out"
  echo "▸ saved: $out"
  echo "  replay:  scripts/pool_record.sh replay '$out'"
}

cmd_replay() {
  [ $# -ge 1 ] || { echo "replay needs a bag dir" >&2; usage 1; }
  local bag="$1"
  [ -d "$bag" ] || { echo "no such bag dir: $bag" >&2; exit 1; }
  echo "▸ playing $bag  (Ctrl-C to stop)"
  ros2 bag play "$bag"
}

cmd_list() {
  [ -d "$RUN_DIR" ] || { echo "no runs yet ($RUN_DIR)"; return 0; }
  echo "runs in $RUN_DIR:"
  # Show bags (dirs) newest-first with size; and any scorecards alongside.
  ( cd "$RUN_DIR" && ls -1dt bag_*/ 2>/dev/null | while read -r d; do
      printf '  %-40s %s\n' "$d" "$(du -sh "$d" 2>/dev/null | cut -f1)"
    done )
  local cards
  cards="$(ls -1t "$RUN_DIR"/*.json 2>/dev/null | head -8 || true)"
  [ -n "$cards" ] && { echo "scorecards:"; echo "$cards" | sed 's|^|  |'; }
}

case "${1:-}" in
  record) shift; cmd_record "$@" ;;
  replay) shift; cmd_replay "$@" ;;
  list)   shift; cmd_list ;;
  -h|--help|'') usage 0 ;;
  *) echo "unknown subcommand: $1" >&2; usage 1 ;;
esac

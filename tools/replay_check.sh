#!/usr/bin/env bash
# replay_check.sh -- run the REAL vision graph on a RECORDED session.
#
# This is the regression fixture the stack never had. `record_session.sh`
# captures image_raw; this plays it back into a live detector + tracker + lock
# ladder + checkpoint bank with no camera attached and nobody in the room, then
# scores what came out.
#
#   tools/replay_check.sh <bag-dir> [label]
#
# Why it matters: every vision check before this needed a person standing in
# front of the camera at the moment the check ran, so no result was ever
# repeatable and no change could be compared against another on the same input.
# A recorded session is input that never changes, which is the only way a
# number from today and a number from next month mean the same thing.
set -euo pipefail

BAG="${1:?usage: replay_check.sh <bag-dir> [label]}"
LABEL="${2:-replay}"
RUN_DIR="${MONGLA_RUN_DIR:-$HOME/mongla_runs}"
WS="${MONGLA_WS:-$HOME/mongla_ws}"
[ -d "$BAG" ] || { echo "no such bag: $BAG" >&2; exit 1; }
mkdir -p "$RUN_DIR"

set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

TS="$(date +%Y%m%d_%H%M%S)"
LABEL="$(printf '%s' "$LABEL" | tr -c 'A-Za-z0-9._-' '_')"
LOG="$RUN_DIR/${LABEL}_${TS}.log"
OUT="$RUN_DIR/bag_${LABEL}_${TS}"

# shellcheck source=tools/_record_lib.sh
source "$(dirname "$0")/_record_lib.sh"

cleanup() {
  set +e
  stop "${REC_PID:-}" 'bag record'
  stop "${SYS_PID:-}" 'graph'
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "▸ graph up in REPLAY mode (no camera)"
run_resettable ros2 launch mongla_vision vision_pi.launch.py \
    replay:=true paused:=false lock:=true lock_class:=person \
    fwd_model:=yolov11n >"$LOG" 2>&1 &
SYS_PID=$!

for _ in $(seq 40); do
  sleep 1
  ros2 topic list 2>/dev/null | grep -q 'forward/detections' && break
done

# Record the graph's OWN output to a second bag, so the replay's result is
# itself a durable artifact rather than a number scrolling past in a terminal.
run_resettable ros2 bag record -s mcap --regex \
  '^/mongla/vision/forward/(detections|tracks|lock)$' -o "$OUT" \
  >>"$LOG" 2>&1 &
REC_PID=$!
sleep 3

echo "▸ playing $BAG"
# Only image_raw: everything else in the recorded bag is OUTPUT, and replaying
# a detection alongside the detector that is meant to produce it would score
# the recording instead of the code.
ros2 bag play "$BAG" --topics /mongla/vision/forward/image_raw >>"$LOG" 2>&1
echo "▸ play finished"
sleep 3

cleanup
trap - EXIT INT TERM
sync
echo "▸ replay output: $OUT"
python3 "$WS/tools/bag_gaps.py" "$OUT" || true
echo "  log: $LOG"

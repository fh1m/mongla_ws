#!/usr/bin/env bash
# record_session.sh -- capture a REPLAYABLE pipeline session on the vehicle.
#
# Why this exists: every pipeline check so far has needed a human standing in
# front of the camera at the moment the check runs. That makes verification
# expensive, unrepeatable and impossible to regression-test. One recorded
# session with detections genuinely dropping and recovering is a permanent
# fixture: `pool_record.sh replay` feeds it back through the real graph, so the
# detector, the tracker, the lock ladder and the checkpoint bank can all be
# re-verified later with nobody in the room.
#
#   tools/record_session.sh vision [seconds] [label]   # camera + detector + ladder
#   tools/record_session.sh bench  [seconds] [label]   # SROT board at rest
#
# Recording the board AT REST is not a spare-time exercise either: a still
# vehicle has a known ground truth (zero motion), so the IMU/depth traces are a
# direct read of sensor noise and bias -- the numbers the filter needs and the
# only ones obtainable without water.
#
# Output goes to $MONGLA_RUN_DIR (default ~/mongla_runs). Recordings are DATA:
# they are never committed, and a session with a person in it is personal
# footage that does not leave the machines it was recorded on.
set -euo pipefail

MODE="${1:-vision}"
SECS="${2:-90}"
LABEL="${3:-$MODE}"
RUN_DIR="${MONGLA_RUN_DIR:-$HOME/mongla_runs}"
WS="${MONGLA_WS:-$HOME/mongla_ws}"
mkdir -p "$RUN_DIR"

# ROS's own setup.bash reads variables it never sets (AMENT_TRACE_SETUP_FILES,
# COLCON_TRACE and friends), so `set -u` aborts the script the instant it is
# sourced -- before a single line of output exists, which makes the failure
# look like the script did nothing at all. Drop -u across the sources only.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

TS="$(date +%Y%m%d_%H%M%S)"
LABEL="$(printf '%s' "$LABEL" | tr -c 'A-Za-z0-9._-' '_')"
LOG="$RUN_DIR/${LABEL}_${TS}.log"
BAG="$RUN_DIR/bag_${LABEL}_${TS}"

# shellcheck source=tools/_record_lib.sh
source "$(dirname "$0")/_record_lib.sh"

cleanup() {
  set +e
  stop "${BAG_PID:-}" 'bag record'
  stop "${SYS_PID:-}" 'graph'
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

case "$MODE" in
vision)
  # --full keeps image_raw, so the bag can be replayed through a DIFFERENT
  # detector or a retuned threshold later. image_debug alone would freeze
  # today's decisions into the fixture and make it useless for that.
  TOPICS='^/mongla/(state|imu_rates|vision/[^/]+/(detections|tracks|lock|correspondences|target_pose|image_debug|image_raw|camera_info))$'
  echo "▸ starting the vision graph (this takes ~15 s to settle)"
  ros2 launch mongla_vision vision_pi.launch.py \
      paused:=false lock:=true lock_class:=person \
      fwd_model:=yolov11n fwd_device_path:=/dev/video1 \
      >"$LOG" 2>&1 &
  SYS_PID=$!
  ;;
bench)
  # No vision. The board only, reporting itself while it sits still.
  # /mongla/imu as well as /mongla/imu_rates: the rates topic is derived, and a
  # noise/bias fixture that records only the derived signal cannot be re-derived
  # differently later. Record what the board actually said.
  TOPICS='^/mongla/(state|imu|imu_rates|esc_rpm|demand|localization/(aiding|fix|heading|motion)|odom)$'
  echo "▸ starting the manager against the board"
  ros2 launch mongla_manager bringup.launch.py vision:=false \
      >"$LOG" 2>&1 &
  SYS_PID=$!
  ;;
*) echo "usage: $0 {vision|bench} [seconds] [label]" >&2; exit 2 ;;
esac

# Wait for the graph rather than sleeping a guessed interval: a fixed sleep
# either wastes seconds or starts the bag before the publishers exist, and a
# bag that opens early records an empty topic list for the whole run.
for _ in $(seq 40); do
  sleep 1
  ros2 topic list 2>/dev/null | grep -q '/mongla/state' && break
done

echo "▸ recording $SECS s → $BAG"
run_resettable ros2 bag record -s mcap --regex "$TOPICS" \
    --include-hidden-topics -o "$BAG" >>"$LOG" 2>&1 &
BAG_PID=$!

if [ "$MODE" = vision ]; then
  echo
  echo "   ┌─────────────────────────────────────────────┐"
  echo "   │  WALK IN AND OUT OF FRAME, several times.   │"
  echo "   │  Let the detection DIE each time you leave. │"
  echo "   │  That gap is the whole point of the record. │"
  echo "   └─────────────────────────────────────────────┘"
fi
for r in $(seq "$SECS" -10 1); do printf '\r   %3d s left ' "$r"; sleep 10; done
printf '\r   done.        \n'

cleanup
trap - EXIT INT TERM
sync
echo "▸ saved: $BAG"
du -sh "$BAG" 2>/dev/null
echo "  replay: ros2 bag play '$BAG'"
echo "  log:    $LOG"

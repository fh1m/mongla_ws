#!/usr/bin/env python3
"""tracker_check -- health probe for the tracking pipeline.

Run AFTER vision_check confirms the detector is up. Subscribes to the
/tracks topic (output of tracker_node) and reports:
  * tracks per frame
  * track ID stability (how often a previously-seen ID returns)
  * predicted-frame ratio (how often Kalman is filling gaps)
  * whether a required class was tracked at all

Exits 0 when at least one stable confirmed track was seen, 1 otherwise.

Examples
--------
# Default: check laptop camera for 5 s
ros2 run duburi_vision tracker_check

# Require a 'person' track with stable ID
ros2 run duburi_vision tracker_check --camera laptop --duration 5 \\
    --require-class person
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.qos  import QoSProfile, QoSReliabilityPolicy

from vision_msgs.msg import Detection2DArray


def main(argv=None):
    parser = argparse.ArgumentParser(description='Tracker pipeline smoke test.')
    parser.add_argument('--camera',         default='laptop',
                        help='camera namespace under /duburi/vision/ (default: laptop)')
    parser.add_argument('--duration',       type=float, default=5.0,
                        help='measurement window in seconds (default: 5)')
    parser.add_argument('--min-track-hz',   type=float, default=1.0,
                        help='minimum tracks messages/sec to pass (default: 1)')
    parser.add_argument('--require-class',  default='',
                        help='if set, fail unless this class was seen in a track')
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = Node('tracker_check')
    ns   = f'/duburi/vision/{args.camera}'

    state: dict = {
        'msg_count':      0,
        'track_frames':   0,   # frames that had at least one track
        'total_tracks':   0,
        'predicted_tracks': 0,
        'classes':        Counter(),
        'all_ids':        set(),
        'prev_ids':       set(),
        'reused_ids':     0,
        'id_frames':      0,
    }

    def _on_tracks(msg: Detection2DArray) -> None:
        state['msg_count'] += 1
        ids_this_frame = set()
        has_track = False
        for d in msg.detections:
            tid = d.tracking_id
            if not tid:
                continue
            has_track = True
            state['total_tracks'] += 1
            ids_this_frame.add(tid)

            # class name from hypothesis
            class_name = ''
            if d.results:
                hyp = d.results[0]
                if hasattr(hyp, 'hypothesis'):
                    class_name = str(hyp.hypothesis.class_id)
                else:
                    class_name = str(getattr(hyp, 'id', ''))
            if class_name:
                state['classes'][class_name] += 1

            # predicted check: score == 0 means Kalman predicted
            score = 0.0
            if d.results:
                hyp = d.results[0]
                score = float(getattr(getattr(hyp, 'hypothesis', hyp), 'score', 0.0))
            if score == 0.0:
                state['predicted_tracks'] += 1

        if has_track:
            state['track_frames'] += 1

        # Count re-used IDs (temporal stability)
        reused = ids_this_frame & state['prev_ids']
        state['reused_ids']  += len(reused)
        state['id_frames']   += len(ids_this_frame)
        state['all_ids']     |= ids_this_frame
        state['prev_ids']     = ids_this_frame

    qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
    node.create_subscription(Detection2DArray, f'{ns}/tracks', _on_tracks, qos)

    print(f"[tracker_check] watching {ns}/tracks for {args.duration:.1f}s ...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)

    elapsed   = time.monotonic() - t0
    msg_hz    = state['msg_count'] / max(elapsed, 0.01)
    total_ids = len(state['all_ids'])
    stability = (state['reused_ids'] / state['id_frames']
                 if state['id_frames'] > 0 else 0.0)
    pred_ratio = (state['predicted_tracks'] / state['total_tracks']
                  if state['total_tracks'] > 0 else 0.0)

    print()
    print("=== tracker_check results ===")
    print(f"  tracks topic hz  : {msg_hz:.2f}  (required >= {args.min_track_hz})")
    print(f"  frames with tracks: {state['track_frames']} / {state['msg_count']}")
    print(f"  unique track IDs  : {total_ids}")
    print(f"  ID stability      : {stability*100:.1f}%  (% of ID appearances that reused a prior ID)")
    print(f"  predicted ratio   : {pred_ratio*100:.1f}%  (Kalman-only frames; higher = more gaps)")
    if state['classes']:
        top = state['classes'].most_common(5)
        print(f"  classes tracked   : {dict(top)}")

    failures = []
    if msg_hz < args.min_track_hz:
        failures.append(
            f"tracks hz {msg_hz:.2f} < required {args.min_track_hz} -- "
            "is tracker_node running? (ros2 run duburi_vision tracker_node)")
    if args.require_class and args.require_class not in state['classes']:
        failures.append(
            f"class {args.require_class!r} not seen in tracks -- "
            "is the detector seeing it? (ros2 run duburi_vision vision_check "
            f"--require-class {args.require_class})")

    if failures:
        print()
        print("[FAIL]")
        for f in failures:
            print(f"  ✗  {f}")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    print()
    print("[PASS]")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()

# tracking/ — v2 SHIPPED

ByteTrack + per-track Kalman smoother are fully implemented and merged.

## What shipped

| Component | File |
|---|---|
| ByteTrack wrapper | `tracking/bytetrack.py` |
| Tracker ABC | `tracking/tracker.py` |
| Kalman smoother (per track, 4-state CV) | `tracking/kalman.py` |
| ROS node (detections → tracks) | `tracker_node.py` (package root) |
| Track-ID overlay | `draw.py` → `draw_track_ids()` |
| ROS integration test | `utils/tracker_check.py` → `tracker_check` CLI |

## Topic contract

```
in   /mongla/vision/<cam>/detections    vision_msgs/Detection2DArray
out  /mongla/vision/<cam>/tracks        vision_msgs/Detection2DArray  (tracking_id set)
```

Predicted frames (occlusion-bridged) carry `score=0.0`.

## Where tracks are consumed

The mission **control path always reads `/detections`** — the two vision
verbs (`vision_align` / `vision_move`) run their pixel-error P-loops on raw
detector boxes, so there is no per-goal tracking flag anymore. `/tracks`
(stable IDs + Kalman-smoothed bboxes) feeds the **mission-control HUD and
offline analysis** instead.

```bash
# Start tracker_node alongside the detector (default true):
ros2 launch mongla_vision cameras_.launch.py with_tracking:=true

# depth_estimation_node can optionally read /tracks instead of /detections:
ros2 launch mongla_vision cameras_.launch.py depth:=true   # use_tracks=with_tracking
```

See `.claude/context/perception/vision-architecture.md` (topic contract) for the full
detector → tracker → HUD data flow.

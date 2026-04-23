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
in   /duburi/vision/<cam>/detections    vision_msgs/Detection2DArray
out  /duburi/vision/<cam>/tracks        vision_msgs/Detection2DArray  (tracking_id set)
```

Predicted frames (occlusion-bridged) carry `score=0.0` — `VisionState` and
`vision_state.py` check for this and do not fire `on_lost` on predicted boxes.

## How to enable in missions

```python
# DSL — per-goal tracking:
duburi.vision.lock(target='gate', axes='yaw,forward', tracking=True, ...)

# launch — enable tracker_node:
ros2 launch duburi_vision cameras_.launch.py with_tracking:=true
```

See `.claude/context/mission-cookbook.md` §"Tracking while moving" for full examples.

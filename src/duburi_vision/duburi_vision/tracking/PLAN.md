# tracking/ -- v2 design notes

Goal: persist a stable `track_id` across frames so the planner can say
"chase track 7" instead of "the largest box this frame", and so we don't
lose lock when a fish briefly occludes the target.

## Module layout (when implemented)

```
duburi_vision/tracking/
  __init__.py
  tracker.py        # ABC: update(detections, frame_t) -> list[Track]
  bytetrack.py      # supervision.ByteTrack wrapper (default)
```

## ROS surface

Add a `tracker_node.py` at the package root (NOT in `tracking/`, mirrors
how camera/detector live at root). Topics:

```
in   /duburi/vision/<cam>/detections    vision_msgs/Detection2DArray
out  /duburi/vision/<cam>/tracks        vision_msgs/Detection2DArray  (with tracking_id)
out  /duburi/vision/<cam>/image_debug   sensor_msgs/Image             (track_id labels)
```

The tracked array reuses the `Detection2D.tracking_id` field; the visualizer
gains a `draw_track_id` overlay so each box keeps the same color across
frames.

## Failure modes to handle

- ID swap when two objects of the same class cross paths
- ID loss + re-acquisition (the supervision ByteTrack has `track_buffer`
  for exactly this -- expose it as a ROS param)
- Class drift across frames (planner asked for "person", tracker held a
  bbox that the detector relabeled as "backpack" mid-sequence)

## Proven patterns to lift

- supervision tutorial: https://supervision.roboflow.com/latest/trackers/
- ByteTrack paper: https://arxiv.org/abs/2110.06864
- yolo_ros (mgonzs13) for ROS integration shape

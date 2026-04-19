# filters/ -- v3 design notes

Goal: smooth bbox center per `track_id` so the visual-PID setpoint isn't
chasing per-frame jitter. Outputs feed `duburi_planner.vision_client`.

## Module layout (when implemented)

```
duburi_vision/filters/
  __init__.py
  smoother.py       # ABC: update(track_id, cx, cy, t) -> (cx_hat, cy_hat, vx, vy)
  kalman.py         # 4-state CV model (filterpy or hand-rolled)
```

## State model

Per-track 4-state constant-velocity Kalman:
  x_k = [cx, cy, vx, vy]^T
  Measurement = [cx, cy] (from tracker output)
  Process noise: tuned per-class (a swimming person is wigglier than a torpedo target)

## ROS surface

Either (a) live inside `tracker_node` (cheaper, fewer hops) or (b) be a
separate `kalman_node.py`. Decide once we measure end-to-end latency on
Jetson Orin. Topic:

```
out  /duburi/vision/<cam>/tracks_smooth    vision_msgs/Detection2DArray
```

with the smoothed cx/cy populated and bbox size still raw.

## Bonus: lost-target prediction

When the detector misses a frame, the Kalman keeps predicting forward.
Surface that as `Detection2D.results[0].score = 0.0` so the planner can
see "this is a predicted box, not a real measurement". After N predicted
frames in a row (configurable), drop the track.

## References to lift from

- rlabbe Kalman & Bayesian Filters in Python (book)
- https://github.com/rlabbe/filterpy
- https://github.com/pcdangio/ros-kalman_filter
- https://github.com/AbhinavA10/ROS2-Labs (ROS2 Kalman example)

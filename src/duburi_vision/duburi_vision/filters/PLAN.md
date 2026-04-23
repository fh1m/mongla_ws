# filters/ — v3 SHIPPED (folded into tracker_node)

The per-track Kalman smoother was implemented directly inside `tracker_node.py`
rather than as a separate `kalman_node` or `filters/` module. This reduced
end-to-end latency by one topic hop and proved sufficient on Jetson Orin.

## What shipped

| Component | File |
|---|---|
| 4-state CV Kalman per track | `tracking/kalman.py` |
| Integration into tracker pipeline | `tracker_node.py` (runs after ByteTrack update) |

## Design decisions

- Smoothed cx/cy replace raw values in the outgoing `/tracks` topic.
- Predicted frames (no detector measurement) use the Kalman forward-predict step
  and are marked with `score=0.0` so callers know the box is estimated.
- Process noise is a single tunable; per-class tuning is deferred to v4+.

The `filters/` module directory is kept as an organisational placeholder.
New filter work (e.g. depth/velocity fusion) goes here.

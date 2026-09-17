# duburi_vision — eyes, and everything that keeps them honest

**24k lines · 824 tests · [`src/duburi_vision`](../../../../src/duburi_vision)**

The largest package, because seeing underwater is the hard part. Water bends light, absorbs
red first, moves particles through the frame and throws caustics across the floor. A detector
trained in one swimming pool can fail in a different swimming pool.

So this package is not just "run a neural network". It is: get the *newest* frame, know what
the lens really does, detect, keep hold of the target when the detector blinks, and measure
motion from what the floor does between frames.

---

## The path a frame takes

```
camera ──► detector ──► tracker ──► lock ──► the control loop
   │                                  │
   └──► optical flow ─► velocity, height above floor, heading from the floor
```

### 1. Get the newest frame, not the oldest

A camera driver's queue hands you the *oldest* frame it is holding. Underwater that is a frame
from a vehicle that has since moved. The camera path keeps a mailbox instead — one slot,
newest wins — measured at **16.9 ms** stale against **396 ms** for a plain queue.

### 2. Know what the lens does

A camera behind a flat window is not the camera in the datasheet: water bends every ray, and
the effect grows toward the edges of the frame. Our forward camera is **63.8° in air and
46.7° in water**, measured over 25 views rather than taken from a spec sheet, and every metric
consumer goes through the same refraction correction (`optics.py`).

Skip that and a gate 4 m away reports as 5.15 m — a 29 % error no amount of control tuning
can fix.

### 3. Detect, on a chip built for it

The Hailo-8 runs the network. Measured: **80.9 Hz** standalone, **53.9 Hz** through the live
ROS graph, **18.0 ms** from photon to detection. Segmentation masks are decoded without ever
leaving integer arithmetic — 31.5 ms down to **0.93 ms**, bit-for-bit identical.

The detector picks its backend from the artifact it is given: a Hailo binary, a TensorRT
engine, or plain PyTorch. Same interface, so nothing above it changes.

### 4. Hold the target when the detector blinks

Detectors flicker. Measured over 71 real gaps in real footage, the median gap is 0.155 s and
the 90th percentile 0.651 s. Rather than tune a timeout by feel, there is a ladder — tracker,
then an optical-flow follower, then a feature anchor that once held **175 consecutive frames**
the detector had lost. Each rung is sized against that measured distribution.

### 5. Measure motion from the floor

The downward camera is a velocity sensor. Rotation is subtracted before scaling, because
translation scales with height and rotation does not. Against a tape: three 30 cm slides,
worst error **1.09 cm**, with the implied height coming back 0.72 / 0.69 / 0.70 m against a
0.72 m measurement. When the image cannot support a measurement it reports **nothing** — never
a confident zero.

---

## Nodes

| Node | Publishes | Purpose |
|---|---|---|
| `camera_node` | `image_raw`, `camera_info` | the mailbox camera path; also plays video files |
| `detector_node` | `detections`, `contours`, `image_debug` | inference, plus the always-on operator alignment line |
| `tracker_node` | `tracks` | stable ids, and a predicted box across a short gap |
| `lock_node` | `lock`, `correspondences` | the continuity ladder and the feature anchor |
| `flow_node` | `velocity`, `floor_height`, `floor_grid_deg`, `lane_heading_deg`, `distance_traveled` | everything the downward camera can tell us |
| `depth_estimation_node` | `vis_range` | monocular proximity, off by default |
| `mission_web_node` | — | the pool-day browser console |

## Tools

```bash
ros2 run duburi_vision vision_check          # is the pipeline alive and seeing?
ros2 run duburi_vision calibrate             # measure this camera, in water
ros2 run duburi_vision vision_thrust_check   # detection → thrust, disarmed and safe
ros2 launch duburi_vision mission_web.launch.py   # both cameras in a browser
```

## What is deliberately *not* here

**Image enhancement.** Contrast equalisation before the detector was measured in 17
configurations across four props and three venues and was never positive — on the gate it
destroyed 95 % of detections. It ships off in every profile, and a test keeps it off. The
instinct is strong; the data says no.

## Testing

```bash
python3 -m pytest -q src/duburi_vision/test
```

824 tests, including ones that run on real recorded footage rather than synthetic images, and
one that reads `measured-bars.md` so a documented threshold cannot drift from the shipped
value.

---

Related: [`duburi_localization`](../duburi_localization/README.md) (what consumes the
velocity) · [Capability Map](../../capability-map.md) ·
[`measured-bars.md`](../../measured-bars.md)

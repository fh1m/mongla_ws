# Bumblebee — RoboSub 2025 (1st Place)

**Team**: Bumblebee Autonomous Systems, National University of Singapore (NUS)
**Placement**: 1st place, RoboSub 2025
**Paper**: Bumblebee_Robosub_Paper_2025.pdf (confirmed read)

---

## Their Vision Stack (confirmed from TDR)

```
Camera (4K, auto-exposure)
  │
  ▼
YOLO11  ─── initial detection + class label + bbox
  │
  ▼
XFeat (CVPR 2024, accelerated_features)
  │   keypoint matching: live crop ↔ dock-side template
  │   produces 2D-2D correspondence pairs
  │
  ├──▶  solvePnP  ──▶  6DOF pose (x,y,z + roll,pitch,yaw)
  │                     gate: 1.5m poles × 1.0m width
  │
  ├──▶  HDBSCAN   ──▶  3D point clustering (slalom: 6 poles → ordered waypoints)
  │
  └──▶  DepthAnything V2  ──▶  per-pixel relative depth map (no calibration required)

All above sources fused with DVL via UKF (Unscented Kalman Filter)
  │
  ▼
Final 6DOF pose estimate → motion planner
```

---

## Key Technical Details

### XFeat (arXiv:2404.19174, github.com/verlab/accelerated_features)

- **What it is**: Local keypoint descriptor (position + 64-D feature vector)
- **What it is NOT**: A multi-object re-ID model, a bounding box tracker, a class classifier
- **How Bumblebee uses it**: template image (dock-side) ↔ live bbox crop → 2D-2D correspondences → PnP
- **Speed**: ~27 fps on CPU (x86), ~100 fps on Orin Nano GPU via TensorRT/ONNX. ⚠ Bumblebee's
  own figure, for their Jetson Orin Nano — not our platform. Our companion is the Raspberry
  Pi 5 + Hailo-8 AI HAT (Jetson/TensorRT retired here); Hailo-8 needs its own HEF compilation
  via the Hailo Dataflow Compiler, a different deployment path with unmeasured feasibility.
- **License**: Apache-2.0, pip installable
- **Why not SuperPoint+SuperGlue**: XFeat is 10× faster, single ONNX, no GNN matcher required

### DepthAnything V2 (arXiv:2406.09414, LiheYoung/Depth-Anything-V2)

- **What it is**: ViT-based monocular depth estimation → relative depth map per pixel
- **Calibration**: 1-point calibration (measure distance to object at known range → scale factor)
- **Speed**: Small model (25M params), TensorRT fp16 → ~15 fps on Orin Nano GPU. ⚠ Same
  caveat as XFeat above — this is Bumblebee's Jetson figure, not measured for our Pi 5 +
  Hailo-8 companion.
- **Use case**: Distance estimate without known object geometry (bbox area proxy is fragile)

### HDBSCAN Clustering

- **Why**: Slalom task has 6 poles; YOLO detects "pole" class but doesn't give ordering
- **How**: 3D projected XFeat keypoints → HDBSCAN → cluster centroids = individual pole positions
- **Library**: `sklearn.cluster.HDBSCAN` (scikit-learn, already available in most ROS2 envs)

### UKF Fusion

- **Sources fused**: XFeat+PnP pose + DepthAnything distance + DVL velocity + AHRS heading
- **Why UKF not EKF**: Pose composition is mildly nonlinear; UKF handles it without Jacobians
- **Output**: Full 6DOF world-frame pose at ~30 Hz for the motion planner

---

## What Mongla Does vs. What Bumblebee Does

| Capability | Mongla (current) | Bumblebee (2025 winner) |
|------------|-----------------|------------------------|
| Detection | YOLO11 ✓ | YOLO11 ✓ |
| Tracking | ByteTrack + KF ✓ | ByteTrack (implied) |
| Distance estimate | bbox area proxy ⚠️ | XFeat+PnP (metric) + DepthAnything ✓ |
| Approach angle | unknown ✗ | XFeat+PnP 6DOF ✓ |
| Multi-instance | one bbox per class ⚠️ | HDBSCAN 3D clustering ✓ |
| Sensor fusion | P-controller on err_x/err_y | UKF over all sources |
| Monocular depth | None ✗ | DepthAnything V2 ✓ |

---

## Adoption Plan for Mongla

### P0 — Pre-pool-day: XFeat + PnP Pose Estimator

**Why P0**: Gate-pass with style requires knowing approach angle precisely. Torpedo requires exact board face orientation. XFeat+PnP gives this with ZERO new training data needed (just a dock-side photo).

**What to build**: `mongla_vision/pose/xfeat_pose.py`

```python
class XFeatPoseEstimator:
    def __init__(self, template_path: str, object_points_3d: np.ndarray, K: np.ndarray):
        # Run XFeat on template at init, store descriptors
    def estimate(self, crop_bgr: np.ndarray) -> Optional[PoseResult]:
        # Match → PnP → return (distance_m, yaw_offset_deg, pitch_offset_deg)
        # Returns None if < 8 inliers (fall back to bbox area proxy)
```

**New ROS topic**: `/mongla/vision/<cam>/pose` (`geometry_msgs/PoseStamped`)

**Gate physical geometry** (for solvePnP object_points_3d):
```python
# Standard RoboSub gate: 1.5m poles, 1.0m separation (measure at dock)
GATE_POINTS_3D = np.array([
    [0.0,  0.0, 0.0],   # bottom-left pole
    [1.0,  0.0, 0.0],   # bottom-right pole
    [0.0,  1.5, 0.0],   # top-left pole
    [1.0,  1.5, 0.0],   # top-right pole
], dtype=np.float32)
```

**Templates**: Store in `mongla_vision/templates/gate_front.jpg` etc.

**Dock-side calibration checklist**:
1. Place AUV 1m from gate, centered, camera level
2. `ros2 run mongla_vision capture_template --camera forward --output templates/gate_front.jpg`
3. Confirm template captures both poles and crossbar clearly

### P1 — Competition day: XFeat Re-ID Gallery

**Why P1**: After Part 4 tracking improvements (track_buffer=150), ID stability is much better. Re-ID is the remaining failure mode: turbid water hides gate for >5s → new ID on reappear. XFeat re-ID bridges that gap.

**What to build**: `mongla_vision/filters/reid.py`

```python
class XFeatReID:
    def update(self, track_id: int, crop: np.ndarray) -> int:
        # Returns stable_id (may differ from track_id if gallery match found)
        # Side effect: updates gallery with current crop's XFeat descriptors
```

**Integration**: Called in `tracker_node.py` after ByteTrack assignment, before publishing `/tracks`.

**When to DISABLE** (per Bumblebee's lessons):
- Gate class: left pole and right pole have near-identical XFeat descriptors → false match
- Bin covers: too similar to floor texture → high false positive rate

### P2 — Post-competition: DepthAnything V2

**Why P2**: Useful but not competition-critical given XFeat+PnP already gives metric distance.

**What to build**: `vision_depth_node.py`

```python
# New ROS node: camera/image_raw → /mongla/vision/<cam>/depth_map (sensor_msgs/Image, 32FC1)
# Model: DepthAnything V2 Small (ONNX, TensorRT fp16)
# Calibrate: measure bbox median depth at known distance → scale factor
```

**Jetson Orin Nano feasibility**: 25M params, TensorRT fp16 → ~15 fps concurrently with
YOLO11n. This is a pre-pivot estimate for the retired Jetson platform — do not treat it as
our budget. Our companion is Pi 5 + Hailo-8, which needs its own HEF compile and has not
been re-estimated for this model.

### P2 — Post-competition: HDBSCAN Multi-Instance (Slalom)

**Why P2**: Slalom has 6 poles; YOLO gives individual pole bboxes without ordering.

**What to build**: After XFeat+PnP gives per-pole 3D positions, HDBSCAN clusters them into ordered waypoints.

```python
from sklearn.cluster import HDBSCAN
# Input: list of (x, y, z) from XFeat+PnP per detected pole
# Output: sorted pole positions → slalom waypoints
```

---

## Implementation Notes from Bumblebee Paper

1. **XFeat runs on bbox crops, not full frames** — crop the YOLO detection first, then run XFeat on the crop. This keeps inference fast (128×128 px typical crop vs 1280×720 full frame).

2. **Minimum inlier threshold**: Bumblebee uses 8+ inliers before trusting the pose. Below this, fall back to bbox area proxy.

3. **Template diversity**: Store 3-5 templates per obstacle (front, left-30°, right-30°, underwater-visibility). Match against all, pick the one with most inliers.

4. **ONNX export command** (from XFeat repo README):
   ```bash
   python export_onnx.py --model xfeat --output models/xfeat.onnx
   ```

5. **Pool-day template capture**: Capture templates in the actual pool under competition lighting, not in air. Underwater color cast changes keypoint appearance significantly.

---

## Open Questions

- Does RoboSub 2026 use the same gate geometry? Check official task manual each year.
- Which YOLO model does Bumblebee use with XFeat? Paper says YOLO11 — same as us ✓
- Their DVL model: unknown. We have never fitted a DVL —
  `mongla_sensors/sources/nucleus_dvl.py`/`nucleus_parser.py` target a Nortek Nucleus1000
  protocol, but no Nucleus1000 (or any DVL) has ever been mounted or validated in water.
- Their UKF implementation: custom or ROS2 robot_localization package?

---

## References

- XFeat paper: arXiv:2404.19174 (Guilherme Potje et al., CVPR 2024)
- XFeat code: github.com/verlab/accelerated_features
- DepthAnything V2: arXiv:2406.09414 (Lihe Yang et al.)
- DepthAnything V2 code: github.com/LiheYoung/Depth-Anything-V2
- HDBSCAN: Campello et al. 2013, scikit-learn >= 1.3

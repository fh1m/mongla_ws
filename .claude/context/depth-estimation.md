# Depth Estimation (`duburi_vision/depth`)

## Purpose

Monocular proximity estimation for detected objects. Publishes a
`vis_range` score per detection (0.0 = far, 1.0 = close) so mission
logic and the HUD can react to object distance without stereo cameras
or sonar.

## Modes

| Mode | Condition | Method |
|------|-----------|--------|
| ONNX | `model_path` set + onnxruntime installed | Depth Anything V2-Small |
| Fallback | no model or import failure | `sqrt(w_frac * h_frac)` bbox-area proxy |

Both modes publish identical `vis_range` topics — callers need not
distinguish.

## Node: `depth_estimation_node`

**Package**: `duburi_vision`  
**Executable**: `depth_estimation_node`  
**File**: `duburi_vision/depth/depth_estimation_node.py`

### Parameters

| Param | Default | Notes |
|-------|---------|-------|
| `camera` | `'forward'` | Namespace: `/duburi/vision/<camera>/…` |
| `model_path` | `''` | Absolute path to `.onnx`. Empty = fallback |
| `run_every_n_frames` | `3` | Run ONNX on every Nth frame (skip others) |
| `publish_depth_map` | `False` | `True` → publish `vis_range_map` debug image |
| `invert_depth` | `True` | Negate raw ONNX output before normalisation |
| `use_tracks` | `False` | `True` → subscribe `/tracks` (keeps parallel to tracker order) |

### Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/duburi/vision/<cam>/image_raw` | `sensor_msgs/Image` | in |
| `/duburi/vision/<cam>/detections` or `/tracks` | `vision_msgs/Detection2DArray` | in |
| `/duburi/vision/<cam>/vis_range` | `std_msgs/Float32MultiArray` | out |
| `/duburi/vision/<cam>/vis_range_map` | `sensor_msgs/Image` (float32) | out (optional) |

`vis_range.data` is a list of floats, one per detection in the input
array, in the same order. If the detector sends 3 detections the array
has 3 floats.

### Temporal smoothing

EMA with `alpha=0.40` applied after each detection callback:
```
smooth[i] = 0.40 * raw[i] + 0.60 * prev_smooth[i]
```
Resets when detection count changes (new object appearing). This removes
frame-to-frame jitter from YOLO NMS or ONNX output variation while
staying responsive to real proximity changes (~0.4 s time constant).

## Model: Depth Anything V2-Small

| Property | Value |
|----------|-------|
| Input | `(1, 3, 364, 364)` float32, ImageNet normalised NCHW |
| Output | `(1, 364, 364)` float32 (raw inverse depth) |
| Post-process | negate (`invert_depth=True`), min-max normalise to [0, 1] |
| Mean | `[0.485, 0.456, 0.406]` |
| Std | `[0.229, 0.224, 0.225]` |
| File | `duburi_vision/depth/models/depth_anything_v2_small.onnx` |
| Source | https://huggingface.co/depth-anything/Depth-Anything-V2-Small |

**Re-export from HuggingFace** (if `.onnx` not present):
```bash
pip install optimum[exporters]
optimum-cli export onnx \
    --model depth-anything/Depth-Anything-V2-Small \
    --task depth-estimation \
    --opset 17 \
    depth_anything_v2_small_onnx/
```

The `.onnx` is gitignored (binary, ~50 MB). Copy into
`duburi_vision/depth/models/` or set `model_path:=` to any other path.

## Launch

```bash
# Bbox-area fallback (no model — always works):
ros2 launch duburi_vision cameras_.launch.py depth:=true

# With ONNX model:
ros2 launch duburi_vision cameras_.launch.py depth:=true \
    depth_model:=$(ros2 pkg prefix duburi_vision)/../../src/duburi_vision/duburi_vision/depth/models/depth_anything_v2_small.onnx

# Verify topic:
ros2 topic echo /duburi/vision/logitech/vis_range
```

## HUD integration

`vision_display` subscribes `vis_range` + `vis_range_map` automatically.

- **Bbox border colour**: blue = far → green = mid → red = close
- **Label suffix**: `~0.72 CLOSE` (qualitative + numeric)
- **STATE panel VIS_R row**: `0.72 CLOSE` (green/amber/dim)
- **PROX bar in Zone D**: horizontal fill above altimeter
- **Depth map inset**: top-right of video frame, TURBO colormap.
  Toggle with **D** key (off by default to save CPU).

## Test

```bash
cd ~/Ros_workspaces/duburi_ws
python src/duburi_vision/test/test_depth_estimation.py \
    src/duburi_vision/duburi_vision/depth/models/depth_anything_v2_small.onnx
# Expected: 9 passed, 0 failed
```

The test does not require a live ROS environment — it stubs out all ROS
imports and tests onnxruntime loading, inference shape, and value range
directly.

## Depth health in pipeline_health

`vision_display._build_health()` checks:
```python
'depth': (now - node._last_vis_range_t) < 3.0
```
Green = `vis_range` messages arrived within the last 3 s.
Red = node not running or stalled.

## Implementation notes

- **Why `use_tracks=True` matters**: when the tracker is running,
  `display_node` iterates `display_dets` which comes from `/tracks`.
  If `depth_estimation_node` subscribes `/detections` instead, the
  vis_range index order can diverge (tracker reorders / drops entries).
  Setting `use_tracks=LaunchConfiguration('with_tracking')` in the
  launch file keeps both arrays parallel.

- **`invert_depth=True`**: Depth Anything outputs *inverse* depth
  (large value = close). We negate so that larger values remain "close"
  after the min-max normalisation step.

- **Why EMA not Kalman**: vis_range is a scalar; EMA gives sufficient
  smoothing with a single param and no covariance tuning needed.

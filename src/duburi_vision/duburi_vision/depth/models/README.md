# Depth Model Weights

This folder holds ONNX model weights for the depth estimation node.
Files matching `*.onnx`, `*.pt`, `*.bin`, and `*.pth` are excluded from git
(see `.gitignore` in this directory).

## Supported model: Depth Anything V2-Small

The depth estimation node (`duburi_vision/depth_estimation_node.py`) expects a
Depth Anything V2-Small ONNX export with:

- Input:  `image`  — NCHW float32, shape `(1, 3, 364, 364)`, ImageNet-normalised
- Output: `depth`  — shape `(1, 364, 364)` or `(364, 364)`, relative inverse depth

## Placing the model

Copy or symlink the ONNX file into this directory:

```bash
# Option A — copy
cp /path/to/depth-anything-v2-small.onnx \
   src/duburi_vision/duburi_vision/depth/models/depth-anything-v2-small.onnx

# Option B — symlink from the project model drop location
ln -s /home/fh1m/Envs/dockers/auv-ros2/model.onnx \
      src/duburi_vision/duburi_vision/depth/models/depth-anything-v2-small.onnx
```

## Launching with the model

Pass the full path via the `model_path` ROS parameter:

```bash
ros2 run duburi_vision depth_estimator \
  --ros-args \
  -p camera:=forward \
  -p model_path:=$(pwd)/src/duburi_vision/duburi_vision/depth/models/depth-anything-v2-small.onnx \
  -p run_every_n_frames:=3 \
  -p publish_depth_map:=false
```

If `model_path` is left empty the node runs a bbox-area fallback that requires
no model and still publishes a `vis_range` estimate based on detection size.

## Exporting from HuggingFace

If you need to regenerate the ONNX file from the original weights:

```bash
pip install transformers optimum[exporters] onnxruntime

# Export Depth Anything V2-Small to ONNX
optimum-cli export onnx \
  --model depth-anything/Depth-Anything-V2-Small-hf \
  --task depth-estimation \
  --opset 17 \
  depth_anything_v2_small_onnx/

# The exported model is at:
#   depth_anything_v2_small_onnx/model.onnx
# Copy it here and rename it to depth-anything-v2-small.onnx.
```

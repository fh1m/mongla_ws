# XFeat anchor — Jetson setup runbook (for the on-device agent)

> **Audience:** the agent doing the on-Jetson wiring. This is the *deployment* side of
> the XFeat + LighterGlue anchor/feature-lock system. The code is already on the `lock`
> branch (`duburi_vision/anchor/*`); this doc is how to make it **actually run on the
> Orin Nano at the pool**. Companion: [`anchor-system.md`](anchor-system.md) (architecture).

## 0. TL;DR (what must be true before pool day)

1. `torch` + `torchvision` (JetPack build) importable, CUDA available.
2. **XFeat *and* LighterGlue weights pre-cached** in `~/.cache/torch/hub` **while the
   Jetson still has internet** — the pool has none, and a cold `torch.hub.load` there
   will hang/FATAL. Pre-download = load XFeat **and run one match** (LighterGlue weights
   download lazily on the *first match*, not at load).
3. `numpy<2` (ROS Humble `cv_bridge`/`cv2` ABI — same pin as the rest of the stack).
4. Anchor launches with `anchor:=true` and the XFeat log says `loaded XFeat + LighterGlue
   on cuda:0`.
5. XFeat + YOLO-TRT co-resident within the 8 GB budget at usable FPS.

If all five hold, `duburi.anchor.snap(...)` / `duburi.anchor.align(...)` work.

---

## 1. Why pre-download is mandatory (the #1 pool failure)

`anchor/xfeat.py` loads the model via:

```python
torch.hub.load('verlab/accelerated_features', 'XFeat', pretrained=True, top_k=2048)
```

- On first run this **clones the repo** to `~/.cache/torch/hub/verlab_accelerated_features_main/`
  (XFeat's own weights are bundled in that repo — no separate checkpoint download for XFeat).
- **LighterGlue is different:** it has *no* hub entrypoint and loads its weights from a URL
  **lazily, the first time `match_lighterglue` is called** — i.e. on the first real match,
  not at model load. So "XFeat loaded OK" does **not** mean LighterGlue is cached.

The pool has no internet. If either download is cold there, `anchor_node` logs
`[ANCHOR] XFeatMatcher init FAILED: ... -- pre-download the XFeat/LighterGlue weights on
the Jetson` (or hangs on the LighterGlue fetch mid-mission). **Pre-download both, on the
Jetson, on a network, before you go.**

### 1.1 One-shot pre-download + verify canary

Run this **on the Jetson, with internet**, in the same Python env ROS uses. It loads
XFeat *and* forces one LighterGlue match so both caches populate, then prints device:

```bash
python3 - <<'PY'
import numpy as np, torch
print('torch', torch.__version__, 'cuda', torch.cuda.is_available())
x = torch.hub.load('verlab/accelerated_features', 'XFeat', pretrained=True, top_k=2048)
dev = 'cuda:0' if torch.cuda.is_available() else 'cpu'
x = x.to(dev)
# two textured dummy frames -> forces LighterGlue weights to download NOW
a = (np.random.rand(480, 640, 3) * 255).astype('uint8')
b = np.roll(a, 12, axis=1)
d0 = x.detectAndCompute(a, top_k=2048)[0]; d0['image_size'] = (640, 480)
d1 = x.detectAndCompute(b, top_k=2048)[0]; d1['image_size'] = (640, 480)
mk0, mk1, idx = x.match_lighterglue(d0, d1)   # <-- pulls LighterGlue weights
print('matched', len(mk0), 'points on', dev, '-- XFeat + LighterGlue cached OK')
PY
```

Expected last line: `matched <N> points on cuda:0 -- XFeat + LighterGlue cached OK`.
After this, `~/.cache/torch/hub/` holds the repo + LighterGlue checkpoint and the pool
run is offline-safe. **Do not clear `~/.cache/torch` after this.**

> `TORCH_HOME` overrides the cache dir if you want it on a specific disk
> (`export TORCH_HOME=/home/<user>/.cache/torch`). Keep it on persistent storage.

---

## 2. Python deps (pin or the launch dies)

Same ABI reality as the rest of the vision stack (see CLAUDE.md §8 "Jetson Python deps"):

- **`numpy<2`** (`1.26.4`) — ROS Humble `cv_bridge` + system `cv2` are NumPy-1.x ABI.
  numpy 2 → `_ARRAY_API not found`, every node crashes.
- **`torch` / `torchvision`** — use the **NVIDIA JetPack wheel** for the installed
  JetPack (do NOT `pip install torch` from PyPI — that pulls a CPU/x86 build). Match the
  JetPack version. Verify `torch.cuda.is_available() == True`.
- Do **not** `pip install opencv-python*` — it shadows the GUI-capable system OpenCV
  (breaks `vision_display`).
- XFeat itself needs no pip install — it's pulled via `torch.hub` (§1).

Quick check:
```bash
python3 -c "import numpy, torch, cv2; print('numpy', numpy.__version__); \
print('torch', torch.__version__, 'cuda', torch.cuda.is_available()); print('cv2', cv2.__version__)"
```
Want: `numpy 1.26.x`, torch a `+cuXX`/`tegra` build with `cuda True`, cv2 the system build.

---

## 3. VRAM / compute budget (XFeat beside YOLO-TRT)

- XFeat is **small** (~0.6 M params, 64-D descriptors) — sparse VGA inference is ~150+ FPS
  single-batch on desktop GPUs; on the Orin Nano expect real-time at the anchor node's
  `skip_frames=3` (≈ every 3rd frame). LighterGlue adds a light matching pass.
- The anchor node runs on the **forward** camera during a task (torpedo/gate). Keep
  **one detector live at a time** (the stack already enforces single-live-detector); XFeat
  + one YOLO-TRT engine co-reside in 8 GB comfortably. Two YOLO engines **and** XFeat at
  once is the thing to avoid.
- `sudo nvpmodel -m 0 && sudo jetson_clocks` (MAXN) as for YOLO — `bringup_check` warns if
  not set.
- If FPS is tight, raise `skip_frames` (match less often — the lock still holds between
  matches via the last pose) before dropping YOLO.

---

## 4. Launch + params

Single camera (forward):
```bash
ros2 launch duburi_vision vision.launch.py camera:=forward anchor:=true
```
Dual camera (anchor on both, matches the competition bringup):
```bash
ros2 launch duburi_vision vision_dual.launch.py anchor:=true
```

`anchor_node` params (defaults from the launch files / node):

| Param         | Default    | Meaning |
|---------------|------------|---------|
| `cam`         | `forward`  | Which camera namespace to attach to |
| `device`      | `cuda:0`   | `cuda:0` or `cpu` (falls back to cpu if CUDA absent) |
| `top_k`       | `2048`     | Max keypoints per frame (more = slower, more robust) |
| `min_inliers` | `12`       | RANSAC inliers to declare `LOCKED` (below → `LOST`) |
| `skip_frames` | `3`        | Match every Nth frame (throttle; 0 = every frame) |
| `min_conf`    | `0.1`      | LighterGlue match confidence floor |

Live-tune, e.g.:
```bash
ros2 param set /duburi_anchor_forward min_inliers 16
ros2 param set /duburi_anchor_forward skip_frames 2
```

### Expected healthy logs
- `[ANCHOR] loaded XFeat + LighterGlue on cuda:0 (Orin ...) top_k=2048`
- On snap: `[ANCHOR] reference captured (<N> keypoints)` (N ≥ 16, else it REJECTS a
  low-texture reference — aim at a textured target / get closer).
- Topics up: `/duburi/vision/forward/anchor_error` (Vector3), `anchor_state` (String),
  `anchor_conf` (Float32), `anchor_ref` (Image).

---

## 5. Optional — ONNX / TensorRT for more headroom

The upstream repo has no TRT/ONNX export, but a community port exists:
`https://github.com/noahzhy/xfeat_lightglue_onnx`. Only pursue this if §3 shows XFeat is
starving YOLO of GPU — for a single forward-camera anchor it is usually **not needed**.
If you do: export XFeat (+LighterGlue) to ONNX there, build a TRT engine **on the Jetson**
(device + JetPack locked, same rule as the YOLO engines), and swap the matcher backend.
Leave a note here if you wire it so the next run knows. Keep the torch.hub path as the
tested fallback.

---

## 6. Verify end-to-end (disarmed, on the bench)

1. `ros2 launch duburi_vision vision.launch.py camera:=forward anchor:=true viewer:=true`
2. Point the camera at a textured object. In another shell:
   ```bash
   ros2 run duburi_planner mission <a mission that calls duburi.anchor.snap/align>
   ```
   or drive it manually via the CLI verbs (`vision_anchor_snap` / `vision_anchor_align`).
3. Watch: snap logs `reference captured (N keypoints)`; move the camera — `anchor_error`
   tx/ty track the shift; the HUD reference inset shows green match lines + inlier count.
4. `anchor_conf` rises with a good match; `anchor_state` reads `LOCKED` when inliers ≥
   `min_inliers`.

## 7. Troubleshooting

| Symptom | Cause → fix |
|---|---|
| `XFeatMatcher init FAILED ... pre-download` (FATAL at startup) | Cold cache / no internet → run §1.1 canary on a network; don't clear `~/.cache/torch`. |
| Loads but every match is `LOST`, 0 inliers | Reference was low-texture, or camera on wrong `device` → snap a textured target; confirm `on cuda:0` in the load log. |
| Hang mid-mission on first `anchor.align` | LighterGlue weights never cached (loaded XFeat only) → §1.1 forces a match to cache them. |
| `_ARRAY_API not found` / node crashes | numpy 2 installed → pin `numpy==1.26.4`. |
| `torch.cuda.is_available()` False | PyPI CPU torch installed → install the JetPack torch wheel. |
| Reference `REJECTED -- only N keypoints` | Target too smooth/far → get closer / aim at texture (min 16 kp). |

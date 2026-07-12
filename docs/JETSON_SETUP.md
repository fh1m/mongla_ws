# Jetson / fresh-device setup

Bootstrap a clean Jetson Orin Nano (or any Ubuntu 22.04 dev box) so it
can `colcon build` and run `duburi_manager`. Steps are ordered: each
one assumes the previous one finished cleanly. Skip any section
whose check command already passes -- **the goal is a working build,
not a re-install**.

---

## 0. Prereqs (apt)

ROS2 Humble + the per-package binary deps the workspace needs. Same
list works on Jetson (JetPack 6.0 ships Ubuntu 22.04) and on a desktop
dev box.

```bash
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-vision-msgs \
    ros-humble-cv-bridge \
    ros-humble-rosbridge-server \
    ros-humble-rqt-logger-level \
    python3-opencv \
    python3-pip python3-colcon-common-extensions \
    git curl rsync
```

Source ROS once per shell (or add to `~/.bashrc`):

```bash
source /opt/ros/humble/setup.bash
```

Verify ROS:

```bash
ros2 --help                # prints the ros2 CLI banner
python3 -c "import rclpy"  # no traceback
```

---

## 1. Python deps (pip + Jetson PyTorch)

### 1A. Jetson ONLY -- install torch/torchvision FIRST

The PyPI `torch` wheel is x86_64 + desktop-CUDA only. On Jetson it
falls back to CPU and YOLO inference becomes unusably slow. Install
the JetPack-matched wheels from the NVIDIA index *before*
`pip install -r requirements.txt`:

```bash
# Pick the version that matches your JetPack. JP 6.0 = torch 2.3.x
pip install --no-cache \
    --index-url https://pypi.jetson-ai-lab.dev/jp6/cu122 \
    torch torchvision
```

Verify CUDA is wired:

```bash
python3 - <<'EOF'
import torch
print('cuda:', torch.cuda.is_available(),
      ' device:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU')
EOF
```

If this prints `cuda: False` on a Jetson, **STOP**. The YOLO detector
will run on CPU and you'll burn a pool slot debugging it. Re-install
the JetPack wheel above before continuing.

### 1B. All devices

```bash
pip install -r requirements.txt
```

(Source: [`requirements.txt`](../requirements.txt) at the workspace
root. Lower-bound pinned, no exact pins so security patches keep
flowing.)

If you're on Jetson and get an `opencv-python` build failure, drop
that line from the requirements (the system `python3-opencv` from
step 0 is already installed and shadows it).

### 1C. Jetson ONLY -- exact validated pins (do this, then never think about it)

`requirements.txt` is lower-bound (desktop/CI patch flow). On the Jetson the
vision stack is ABI-sensitive, so apply the on-device-verified pins too:

```bash
pip install --no-deps --force-reinstall -r requirements-jetson.txt
```

This pins **`numpy==1.26.4`** (ROS Humble `cv_bridge` + system `cv2` are NumPy-1.x
ABI; numpy 2 → `_ARRAY_API not found` crashes every node) and **`trackers==2.4.0`**
(the `2.5.0` PyPI wheel is a broken dud with no module; its `numpy>=2` pin is a red
herring). `--no-deps` is required so it can't drag numpy 2 / `opencv-python` back in.

**Never `pip install` OpenCV on the Jetson.** If a stray install pulled it in,
the GUI HUD (`vision_display`) dies on `cv2.namedWindow`. Remove it and fall back
to the system build:

```bash
pip uninstall -y opencv-python opencv-python-headless
python3 -c "import cv2; print(cv2.__version__, cv2.__file__)"   # want /usr/... not ~/.local
```

Full symptom catalogue + one-shot recovery:
[`.claude/context/known-issues.md`](../.claude/context/known-issues.md) §E1–E4.

---

## 2. udev for the BNO085 ESP32-C3 bridge

The BNO chip lives behind an ESP32-C3 USB serial bridge -- the manager
auto-probes `/dev/ttyACM*` for the JSON line `{"yaw": ...}`. To make
the device readable without `sudo`:

```bash
sudo usermod -a -G dialout $USER
# log out + back in OR reboot for the group change to take effect
```

Verify the bridge enumerates after plugging it in:

```bash
ls -l /dev/ttyACM*    # should be group dialout, you in dialout
duburi_sensors_node --ros-args -p source:=bno085 -p port:=auto
                      # should print [BNO ] ready  yaw=...
```

Firmware + wiring contract: [`src/duburi_sensors/firmware/esp32c3_bno085.md`](../src/duburi_sensors/firmware/esp32c3_bno085.md).

---

## 3. MAVLink endpoint (BlueOS or USB Pixhawk)

### Pool / sub: BlueOS UDP

The default profile expects BlueOS to forward the autopilot to UDP
`<jetson_ip>:14550`. Configure that on the BlueOS web UI:

```
Vehicle Setup -> MAVLink Endpoints -> Add
  Type:   UDP Client
  IP:     <jetson static IP, e.g. 192.168.2.69>
  Port:   14550
```

The Jetson side is zero-config: `ros2 run duburi_manager start`
auto-detects the BlueOS endpoint via the `pool` profile (see
`src/duburi_manager/duburi_manager/connection_config.py`).

### Bench: USB Pixhawk (no BlueOS)

Plug the Pixhawk USB into the Jetson and:

```bash
ls -l /dev/ttyACM*      # Pixhawk usually shows as /dev/ttyACM0
ros2 run duburi_manager start --ros-args -p mode:=pixhawk_usb
```

(`mode:=auto` also works -- the resolver probes both UDP and USB.)

### Desk SITL: Gazebo / SITL

```bash
ros2 run duburi_manager start --ros-args -p mode:=sim
```

---

## 4. Build the workspace

From the `duburi_ws` directory:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run the unit tests once before going to the pool -- they take ~5 s
and catch most regressions:

```bash
cd src/duburi_control
python3 -m pytest test/ -v
```

All tests should pass (60 as of this writing).

---

## 5. Smoke test

In one terminal:

```bash
ros2 run duburi_manager start --ros-args -p mode:=auto
                                    # -p debug:=true to see [MAV ...] frames
```

In another:

```bash
ros2 run duburi_manager bringup_check    # GREEN/RED summary
duburi arm
duburi lock_heading
duburi move_forward --duration 2 --gain 40
duburi unlock_heading
duburi disarm
```

If any of those fail, walk through
[`.claude/context/testing-guide.md`](../.claude/context/testing-guide.md)
section 2 -- it covers every common bringup failure with the exact
log line you'll see.

---

## 5b. Watching the vision HUD over VSCode Remote-SSH

> **For pool/competition ops, prefer the layered ground-station workflow** in
> [`../.claude/context/remote-access.md`](../.claude/context/remote-access.md): view vision in
> Lichtblick / `web_video_server` on the laptop (no desktop pixels), drive the Jetson over
> mosh+tmux, and use NoMachine for a full desktop. The `ssh -X` / on-Jetson-display options
> below are the dev-box fallback and are superseded by layers 1–2 there.

**First, the mental model that trips everyone up:** `ros2 launch duburi_vision
vision.launch.py …` runs the nodes **on the Jetson** — your VSCode Remote-SSH
terminal is just a shell *on the Jetson*. So `vision_display`'s OpenCV window is
created **by a Jetson process, on a Jetson display.** Plugging an external monitor
into your **dev laptop does nothing** unless the pixels are routed there. A
headless SSH shell has no `$DISPLAY`, which is the
`cv2.error: Can't initialize GTK backend` you hit. Three real ways to see it:

### Option A — show it on the Jetson's own display (zero install, default)
The Jetson runs a GNOME/Xorg session on display **`:1`**. The `~/.zshrc` guard
(known-issues §E4) auto-sets `DISPLAY=:1` in a headless shell, so a **fresh**
VSCode terminal just works and the HUD opens on the Jetson's screen — view it on a
monitor plugged into the **Jetson**, or over your existing remote-desktop/VNC. All
editing + launching still happen in VSCode (the latency win). If it still errors,
the terminal predates the `.zshrc` edit — open a new one, or `export DISPLAY=:1`.

### Option B — forward the window to your laptop with `ssh -X` (no extra pkg)
To render the actual cv2 HUD **on your laptop monitor**, use X11 forwarding — but
note **VSCode's integrated terminal does NOT forward X11.** Open a *separate*
terminal on your laptop:

```bash
ssh -X duburi-jetson@<jetson-ip>
cd ~/workspaces/duburi_ws && source install/setup.bash
ros2 launch duburi_vision vision.launch.py camera:=forward \
    models:="slalom_red_pipe,gate_rescue_repair,torpedo_blood_hole" \
    classes:=red_pipe,gate,rescue,repair,torpedo,hole,blood conf:=0.60
```

Your laptop needs an X server: Linux = native; macOS = XQuartz; Windows = VcXsrv or
WSLg. `ssh -X` sets `DISPLAY` to a tunnel automatically. Honest caveat: X11
forwarding a full-rate video window over the network is **laggy** — fine for a
glance, poor for tuning. Keep editing in VSCode; use this throwaway terminal only
to watch. (`ssh -Y` if `-X` is blocked by the security extension.)

### Option C — browser stream (best for the laptop; lowest lag) ★
Run the pipeline **headless** (`viewer:=false`, no cv2 window) and serve the
annotated `image_debug` topic over HTTP; VSCode **auto-forwards the port**, so you
open it in your laptop browser — works on any laptop monitor, no X server:

```bash
sudo apt install ros-humble-web-video-server          # one-time
# terminal 1 — pipeline, no GUI window:
ros2 launch duburi_vision vision.launch.py camera:=forward viewer:=false \
    models:="slalom_red_pipe,gate_rescue_repair,torpedo_blood_hole" \
    classes:=red_pipe,gate,rescue,repair,torpedo,hole,blood conf:=0.60
# terminal 2 — web server:
ros2 run web_video_server web_video_server             # serves on :8080
```

Then open in your laptop browser (VSCode forwards `8080` automatically; check the
**Ports** tab):

```
http://localhost:8080/stream?topic=/duburi/vision/forward/image_debug
```

`rqt_image_view` (`sudo apt install ros-humble-rqt-image-view`) is the same idea but
still needs X (Option A/B). For headless laptop viewing, Option C wins.

### Option D — mission console (recommended for pool day) ★★
One command brings up **both cameras + both detectors + web_video_server + a
purpose-built browser console** and auto-opens it. Side-by-side annotated streams,
a live detection table (class · conf · dx/dy px · fill% · vis_range), per-class
counts, vehicle state, the **active-mission-camera** indicator, and live control
(active-camera switch, model dropdown, conf slider, class chips, pause/resume) that
writes the **same ROS surface the mission DSL writes** — so a switch in the UI and a
switch from a running DSL mission stay in lock-step. Click a detection row to copy a
ready `align()` DSL snippet.

```bash
sudo apt install ros-humble-web-video-server          # one-time (video pipe)
ros2 launch duburi_vision mission_web.launch.py       # cameras+detectors+video+console
```

Console on **:8090** (data over SSE), video on **:8080** (web_video_server MJPEG).
On the Jetson NoMachine desktop it opens `http://localhost:8090` automatically. From
a dev-box browser, forward **both** ports (VSCode Ports tab, or NoMachine) and open
`http://localhost:8090`. Add `no_browser:=true` for a headless Jetson.

- **Dataset videos, no hardware** (dev-box end-to-end test):
  `ros2 launch duburi_vision mission_web.launch.py fwd_video:=<gate.mp4> dwn_video:=<bin.mp4>`
- **Registry (UI/DSL model switching):** pass `fwd_models:=a,b,c` (a bare stem
  registers under its own name; the console dropdown lists them).
- Detectors start **live** (`paused:=false`) so both streams show immediately. If the
  GPU is bound, use the console's per-camera Pause or "Make live cam" (exclusive) —
  the same exclusivity the DSL `use_camera` enforces. `paused:=true` starts dark.

Option C (raw `web_video_server` + a hand-built URL) is the underlying mechanism and
still works for a quick single-topic glance; Option D is the full operator surface.

> **"STREAM NOT AVAILABLE" means one of two things** — the camera's detector is not on
> the graph (single-camera run), **or** `web_video_server` isn't up on `:video_port`
> (data panels for that camera keep working, only the video tile is blank). If the
> detection table is live but the video is blank, check the video server, not the camera.
> A **present-but-paused** detector shows "PAUSED — resume to view" instead.

---

## 6. Optional: model weights for YOLO

The YOLO detector auto-downloads the default weight file on first
call. To pre-fetch (e.g. running the pool with no internet):

```bash
mkdir -p ~/.config/Ultralytics
wget https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt \
     -O src/duburi_vision/models/yolov8n.pt
```

Then point the detector at it:

```bash
ros2 run duburi_vision detector_node --ros-args -p model_path:=src/duburi_vision/models/yolov8n.pt
```

---

## 7. What to do if `apt install ros-humble-desktop` fails on Jetson

JetPack 6.0 ships Ubuntu 22.04 so the Humble apt feed works directly.
For older JetPacks (5.x / Ubuntu 20.04), install ROS 2 Humble from
source or use the OSRF prebuilt JetPack image. Source build steps
live at https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
-- this is a several-hour job, plan accordingly.

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
  IP:     <jetson static IP, e.g. 192.168.2.2>
  Port:   14550
```

The Jetson side is zero-config: `ros2 run duburi_manager auv_manager`
auto-detects the BlueOS endpoint via the `pool` profile (see
`src/duburi_manager/duburi_manager/connection_config.py`).

### Bench: USB Pixhawk (no BlueOS)

Plug the Pixhawk USB into the Jetson and:

```bash
ls -l /dev/ttyACM*      # Pixhawk usually shows as /dev/ttyACM0
ros2 run duburi_manager auv_manager --ros-args -p mode:=pixhawk_usb
```

(`mode:=auto` also works -- the resolver probes both UDP and USB.)

### Desk SITL: Gazebo / SITL

```bash
ros2 run duburi_manager auv_manager --ros-args -p mode:=sim
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
ros2 run duburi_manager auv_manager --ros-args -p mode:=auto
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

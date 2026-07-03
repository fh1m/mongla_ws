# Dual-camera setup (Jetson Orin Nano) — forward + downward

> **Audience: the on-Jetson agent / operator.** This is the on-device procedure to run
> the TWO identical Blue Robotics Low-Light USB cameras (forward + downward) reliably.
> The ROS code hooks are already shipped (`camera_node` `device_path` param + forced
> MJPEG + `vision_dual.launch.py` `fwd_device_path`/`dwn_device_path`); this doc is the
> hardware/OS wiring that makes them port-stable and bandwidth-safe.

---

## 1. The camera

Blue Robotics **Low-Light HD USB** (product page:
<https://bluerobotics.com/store/sensors-cameras/cameras/cam-usb-low-light-r1/>,
sensor datasheet: <https://bluerobotics.com/wp-content/uploads/2026/06/IMX323LQ-C.pdf>).

| Spec | Value | Consequence |
|---|---|---|
| Sensor | Sony **IMX322/IMX323** | — |
| Interface | **USB 2.0** UVC webcam | shares one 480 Mbps bus — see §2 |
| Formats | **MJPEG** / H.264 / YUYV | **use MJPEG** for two cams (already forced in code) |
| Max | 1920×1080 @ 30 fps | we run 640×480@30 for detection latency |
| FOV | 80° H × 64° V | matters for the downward alignment px→metres ratio |
| Power | ~220 mA @ 5 V | a powered hub avoids brown-out with two |

**Both units share the same USB VID:PID and (usually) the same/blank serial.** You therefore
**cannot** tell them apart by ID — the ONLY stable discriminator is the physical **USB port**
(`/dev/v4l/by-path/…`). `/dev/videoN` indices reorder across boots and re-plugs, so never
hardcode them.

---

## 2. Bandwidth reality on the Orin Nano (READ FIRST)

The Orin Nano exposes a **single USB 2.0 root hub — 480 Mbps shared** across every USB-2 device.
USB-3 ports do NOT help USB-2 cameras: the xHCI controller hands USB-2 traffic to the legacy
480 Mbps controller, so plugging into a "USB-3" port gives no extra bandwidth for these cams.

- **YUYV (uncompressed) 640×480@30 ≈ 147 Mbps each → two barely fit / three don't.**
- **MJPEG 640×480@30 ≈ 10–30 Mbps each → two fit with huge margin.** ← what we use.

Our `WebcamCamera` already forces `CAP_PROP_FOURCC='MJPG'` before setting resolution
(`cameras/webcam.py`) and logs the negotiated fourcc — confirm `fourcc=MJPG` in the camera_node
log for BOTH cameras. If it logs `YUYV`, the camera ignored the request (rare) — drop resolution
or fps.

Check the topology:
```bash
lsusb -t          # every camera should sit under a 480M root hub; note which ports
lsusb             # confirm both Blue Robotics cams enumerate (same ID = expected)
```
If you later add more USB-2 devices, keep total well under 480 Mbps. Split the two cameras onto
**different physical ports** (ideally different root-hub branches) for headroom + power.

---

## 3. Port-stable identity (the important part)

### 3a. Find each camera's by-path symlink
Plug the FORWARD camera into the port you'll always use for it; likewise DOWNWARD. Then:
```bash
ls -l /dev/v4l/by-path/
# e.g.
#  platform-3610000.usb-usb-0:2.1:1.0-video-index0 -> ../../video0
#  platform-3610000.usb-usb-0:2.2:1.0-video-index0 -> ../../video2
```
Each `…-video-index0` entry is a **capture node pinned to a physical port** — it does NOT move
when indices reorder. Note which port is forward vs downward (unplug one to confirm which
symlink disappears). A UVC camera exposes several nodes per port (`video-index0/1`); **use
`…-video-index0`** (the capture node; `index1` is usually metadata).

> Why not `/dev/v4l/by-id/`? Two identical cams produce the same (or colliding) by-id name —
> not unique. Why not `/dev/videoN`? Reorders across boots. **by-path is the only stable pin.**

### 3b. Launch with the by-path symlinks (no code change needed)
```bash
ros2 launch duburi_vision vision_dual.launch.py \
  fwd_device_path:=/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1:1.0-video-index0 \
  dwn_device_path:=/dev/v4l/by-path/platform-3610000.usb-usb-0:2.2:1.0-video-index0
```
`device_path` (non-empty) overrides the `fwd_device`/`dwn_device` int indices. camera_node logs
`[CAM ] device_path (port-stable) → …` so you can confirm the right node was used. Leave the
paths empty to fall back to `fwd_device:=0 dwn_device:=4` (bench default).

### 3c. (Optional, nicest) udev aliases → fixed friendly names
So the launch line never changes even if you re-cable, mint stable symlinks by port with a udev
rule. Find the port's `KERNELS`/`ID_PATH`:
```bash
udevadm info -q all -n /dev/video0 | grep -E 'ID_PATH=|KERNELS='
```
Then `/etc/udev/rules.d/99-duburi-cameras.rules` (match on the port `KERNELS`, NOT serial —
**both Blue Robotics cams share one serial `...2020032801`, so serial can't tell them apart**).
The `KERNELS` value is the **interface** node (`<hub>.<port>:1.0`), e.g. `1-2.2:1.0` — NOT the
`ID_PATH`/`platform-...` string. `ATTR{index}=="0"` is **required**: each cam exposes 4 video
nodes and *two* report `:capture:` (index0 = the real MJPEG stream, index2 = a metadata node),
so `index==0` picks the stream. This is the **live, deployed rule on THIS Jetson** (2026-07-03,
verified: forward = bottom-LEFT socket = port `1-2.2`; downward = bottom-RIGHT = port `1-2.4`):
```
# Whatever plugs into bottom-LEFT (port 1-2.2) = FORWARD; bottom-RIGHT (1-2.4) = DOWNWARD.
SUBSYSTEM=="video4linux", KERNELS=="1-2.2:1.0", ATTR{index}=="0", SYMLINK+="duburi_cam_forward",  MODE="0666"
SUBSYSTEM=="video4linux", KERNELS=="1-2.4:1.0", ATTR{index}=="0", SYMLINK+="duburi_cam_downward", MODE="0666"
```
```bash
sudo udevadm control --reload && sudo udevadm trigger --subsystem-match=video4linux
ls -l /dev/duburi_cam_*   # expect duburi_cam_forward -> ../video0, duburi_cam_downward -> ../video4
```
Then the competition launch is short and re-cable-proof:
```bash
ros2 launch duburi_vision vision_dual.launch.py viewer:=false paused:=true \
    fwd_device_path:=/dev/duburi_cam_forward dwn_device_path:=/dev/duburi_cam_downward
```
Symlinks auto-recreate on every boot/replug. If you re-cable to different sockets, re-run 3a and
replace the two `KERNELS` values. **NOTE:** ports can shift (seen `2.1/2.3` → `2.2/2.4` across
re-plugs) — always confirm YOUR live `KERNELS` from 3a before trusting the numbers above.

---

## 4. Verify each camera before a run
```bash
# formats + supported modes (confirm MJPG 640x480@30 is listed):
v4l2-ctl -d /dev/v4l/by-path/<fwd>-video-index0 --list-formats-ext | head -40

# quick capture smoke test (GStreamer), per camera:
gst-launch-1.0 v4l2src device=/dev/v4l/by-path/<fwd>-video-index0 io-mode=2 \
  ! image/jpeg,width=640,height=480,framerate=30/1 ! jpegdec ! videoconvert ! fpsdisplaysink -v
```
Run BOTH pipelines at once to confirm the two fit the bus concurrently (watch for
`No space left on device` / dropped frames → bandwidth; switch to MJPEG / lower fps).

---

## 5. In the mission stack (already wired — just confirm)
- `vision_dual.launch.py` brings up both cameras + both detectors (start **paused**) + one HUD.
- A vision verb / `use_camera('downward')` **auto-switches** the live detector (pauses forward,
  resumes downward, only ONE runs → VRAM/bandwidth) and the HUD auto-follows via the latched
  `/duburi/vision/active_camera` topic. Manual HUD keys still work: `f`=forward `d`=downward
  `b`=side-by-side `D`=depth-map.
- Downward frame remap (in the control engine): image-X→Ch6 lateral, image-Y→Ch5 surge fore/aft,
  `fwd`(fill)→depth descent. The hull can't surface during a downward align because ArduSub
  holds `set_depth` (Ch3) and vision only drives the horizontal channels; the `depth_ceiling`
  is a belt-and-suspenders clamp. See `precision-alignment.md` / `command-reference.md`, `task_bin.py`.

> **★ DISARMED axis check — do BOTH axes before any armed bin run.** `surge_sign` only flips
> the fore/aft POLARITY; there is **no axis-swap knob**. The code hard-assumes image-X→Ch6 and
> image-Y→Ch5, so a bottom camera mounted rotated 90° is a CODE change, not a param. Catch it by
> testing both axes disarmed with `vision_thrust_check --camera downward`: (1) target displaced
> FORE/AFT in the image must drive **Ch5** (forward/back) — flip `BIN_SURGE_SIGN` if reversed;
> (2) target displaced LEFT/RIGHT must drive **Ch6** (strafe). If a fore/aft displacement moves
> Ch6 (or vice-versa), the camera is rotated — fix the mount or the code, do NOT run armed.

## 6. `mavlink-camera-manager` — evaluated, NOT adopted
<https://github.com/mavlink/mavlink-camera-manager> is a capable multi-camera streamer, but it
targets MAVLink/GCS video streaming, not our ROS2 `image_raw` + on-Jetson YOLO detection path.
Our `camera_node` (v4l2 + MJPEG + by-path) is lighter and already integrated with the detector/
HUD/vision-verb pipeline. Revisit only if we need MAVLink-advertised streams to a topside GCS.

## 7. Reference threads (Jetson multi-USB-cam)
- <https://forums.developer.nvidia.com/t/running-multiple-usb-cameras-on-jetson-orin-nano-only-2-working-need-all-4/343994> (the 480 Mbps root-hub limit; USB-3 hand-off to legacy)
- <https://forums.developer.nvidia.com/t/how-to-capture-video-simultaneously-with-two-cameras-using-gstreamer/75513>
- <https://forums.developer.nvidia.com/t/recording-concurrently-multiple-streams-from-usb-cameras-to-files-with-python/75048>
- <https://forums.developer.nvidia.com/t/how-to-use-multiple-cameras-on-the-jetson-nano/208902>
- <https://forums.developer.nvidia.com/t/gstreamer-multiple-camera-output/50984>

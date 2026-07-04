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
| Power | **~220 mA @ 5 V** (max) | two = 440 mA — trivial; **current is NOT the problem** (see §2b) |

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

## 2b. The "2nd camera won't enumerate / cannot perform power cycle" failure (READ THIS)

**Symptom (pool 2026-07):** plug in the 2nd camera and it never appears as `/dev/videoN`;
`dmesg` shows *"Cannot enable … cannot perform power cycle"* / reset loops; when it does appear
you get permission/invalid-device errors opening it. One camera alone is always fine.

**It is NOT a current-budget brown-out.** Each cam draws max **220 mA @ 5 V** (440 mA for two) —
the dev-kit ports supply that trivially. A powered hub is *not* required. Two real causes:

1. **USB autosuspend (the enumeration failure).** The Orin Nano's internal hub ships with
   autosuspend ON (`/sys/module/usbcore/parameters/autosuspend = 2`, hub `1-2`
   `power/control = auto`). A UVC camera that gets autosuspended can fail to resume /
   re-enumerate — that's the "power cycle" line. **Fix (Jetson-side, no code):** pin the two
   camera ports to `power/control=on`. Shipped in `~/ESSENTIALS/99-duburi-cameras.rules`
   (deployed by `install.sh`):
   ```
   SUBSYSTEM=="usb", KERNEL=="1-2.1", ATTR{power/control}="on"
   SUBSYSTEM=="usb", KERNEL=="1-2.3", ATTR{power/control}="on"
   ```
   Verify after re-plug: `cat /sys/bus/usb/devices/1-2.1/power/control` → `on`.
   Global belt-and-suspenders: add `usbcore.autosuspend=-1` to the `APPEND` line in
   `/boot/extlinux/extlinux.conf` and reboot (the per-port rule usually suffices, no reboot).

2. **Two 1080p MJPEG streams saturating the one 480 Mbps bus (the streaming failure).** Even
   when both enumerate, opening both at once is bandwidth-heavy and load-heavy.

### Operating rule: ONE camera streaming at a time
We never run both cameras (or both detectors) simultaneously, so don't. Prefer the **single-camera
`vision.launch.py`** per task (one MJPEG stream, one detector, one CUDA context — dodges the
dual-USB *and* the dual-detector OOM). `vision_dual.launch.py` **opens both streams** (the
`paused` flag only gates detector *inference*, not the stream), so use it only on a bench / with a
powered hub. The HUD's old `b` **side-by-side view was removed** — it force-streamed both cameras;
the HUD now shows exactly one camera (`f`/`d` to switch). See §4b for the launch/DSL matrix.

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
verified: both cams in the **UPPER** USB row — forward = upper-LEFT socket = port `1-2.1`;
downward = upper-RIGHT = port `1-2.3`. The bottom row is `1-2.2`/`1-2.4`; the upper ports
enumerate both cameras cleanly whereas a bottom-port attempt only brought up one):
```
# Whatever plugs into upper-LEFT (port 1-2.1) = FORWARD; upper-RIGHT (1-2.3) = DOWNWARD.
SUBSYSTEM=="video4linux", KERNELS=="1-2.1:1.0", ATTR{index}=="0", SYMLINK+="duburi_cam_forward",  MODE="0666"
SUBSYSTEM=="video4linux", KERNELS=="1-2.3:1.0", ATTR{index}=="0", SYMLINK+="duburi_cam_downward", MODE="0666"
```
> The canonical copy of this rule lives at `~/ESSENTIALS/99-duburi-cameras.rules` and is
> deployed by `~/ESSENTIALS/install.sh`. Edit the ports there if you re-cable.
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

## 4b. Which camera runs which model / classes (launch args + DSL)

### At launch — per-camera args on `vision_dual.launch.py`
Each camera has an independent model / class-filter / confidence. Defaults in parentheses:

| Forward cam | Downward cam | Meaning |
|-------------|--------------|---------|
| `fwd_model:=` (`gate_rescue_repair`) | `dwn_model:=` (`bin_fire_blood`) | YOLO model **stem** (resolves `<stem>.engine` in `src/duburi_vision/models/`, `.pt` fallback) |
| `fwd_classes:=` (`gate,rescue,repair`) | `dwn_classes:=` (`fire,blood`) | class filter (comma list; empty = all classes) |
| `fwd_conf:=` (`0.35`) | `dwn_conf:=` (`0.35`) | confidence floor |
| `fwd_device_path:=` | `dwn_device_path:=` | port-stable symlink (see §3c) |

Other args: `paused:=` (`true` — see §4c, **keep it true**), `viewer:=` (`true`), `tracking:=` (`true`),
`anchor:=` (`false`), `device_cls:=` (`cuda:0`).

```bash
# Competition default (gate on forward, bin on downward), headless, detectors paused:
ros2 launch duburi_vision vision_dual.launch.py viewer:=false paused:=true \
    fwd_device_path:=/dev/duburi_cam_forward dwn_device_path:=/dev/duburi_cam_downward

# Different models/classes per camera (e.g. slalom forward, octagon downward):
ros2 launch duburi_vision vision_dual.launch.py \
    fwd_model:=slalom_red_pipe fwd_classes:=red_pipe fwd_conf:=0.4 \
    dwn_model:=octagon         dwn_classes:=octagon  dwn_conf:=0.3 \
    fwd_device_path:=/dev/duburi_cam_forward dwn_device_path:=/dev/duburi_cam_downward

# SINGLE camera only (the memory-safest option for a one-camera task — one detector,
# one CUDA context; cannot hit the dual-detector OOM):
ros2 launch duburi_vision vision.launch.py camera:=forward \
    model:=gate_rescue_repair classes:=gate,rescue,repair \
    device_path:=/dev/duburi_cam_forward
```

### Mid-mission — switching camera / model from the DSL (already wired)
The mission DSL (`DuburiMission`, `duburi.*`) drives it live — **no relaunch**:

```python
duburi.use_camera('downward')            # downward detector LIVE, forward auto-PAUSED, HUD flips
duburi.vision.align('fire', lat=0, depth=0, err=30)
duburi.use_camera('forward')             # back to forward (downward auto-paused)

duburi.set_model('torpedo_blood_hole')   # hot model swap on the current camera (needs models:= registry)
duburi.use('gate', 'gate')               # model + class filter together
duburi.set_classes(['fire','blood'])     # class filter only
duburi.pause_detector('downward')        # manual pause / resume
duburi.resume_detector('forward')
```

A vision verb with `camera='downward'` **auto-activates** that camera too (same guard). `duburi.detected()`,
`where()`, `wait_for()` all read whichever camera is current. Missions: see `task_bin.py` (downward),
`task_gate.py` (forward).

## 4c. ⚠️ Memory guard — the dual-detector OOM (why the Jetson crashed)

`vision_dual` starts **two detector processes**, so there are **two CUDA contexts** (~1 GB each in the
shared unified pool) *regardless of* `paused` — `paused` only stops per-frame **inference**, not the
context/engine load. The crash happens when **both detectors INFER at once** (concurrent GPU work spikes
the pool → `NvMap error 12` / NVML assert / hard lock). Rules that prevent it:

1. **Never launch `paused:=false`.** Keep the default `paused:=true` and make exactly one camera live via
   `duburi.use_camera(...)`. The DSL enforces exclusivity — `_activate_camera` **pauses the previous
   detector before resuming the next**, so only one ever infers ("never two detectors on the Jetson").
   The failure mode is bypassing it: `paused:=false`, or calling `resume_detector` on both without an
   intervening `use_camera`.
2. **One-camera task → use `vision.launch.py camera:=<cam>`** (single detector; the dual-OOM is impossible).
3. **Trim + max before a dual run:** `pkill -f chrome` (or headless), `duburi_max`, and `duburi_clean`
   between runs. The desktop shares the same pool (see `xfeat-setup.md` §3).
4. **System safety net — `earlyoom`** (in `~/ESSENTIALS`): if memory still runs out, earlyoom SIGTERMs the
   heaviest process (a detector) **instead of the kernel hard-locking the whole Jetson** — you lose the
   vision task, not the vehicle. Install: `~/ESSENTIALS/install.sh` (see that README).
5. `anchor:=true` adds XFeat on top — run it with **one** detector, never two + anchor (`xfeat-setup.md` §3).
6. **★ Jetson-agent VRAM measurement (open, from the 2026-07-04 audit — VIS-C1).** The current strategy
   is *operational* (rules 1–4) — both engines stay VRAM-resident regardless of `paused`; nothing unloads
   them. Before trusting a full dual run, **measure it**: `tegrastats` (or `duburi_max` then watch RAM)
   with **both** competition engines loaded + depth if a task uses it — does it OOM at steady state? If it
   comfortably fits (2 nano/small TRT engines are ~1 GB each), no code change is needed. **If and only if it
   OOMs, the correct fix is LAZY-LOAD-ONCE** — don't load a detector's engine until its first `resume`, then
   keep it resident (pays the load cost once, at a natural task transition). **Do NOT implement
   unload-on-pause** — it reloads the engine on every mid-mission camera flip (multi-second stall at exactly
   the wrong moment). fp16 is now the default (`half:=true`, ~½ the VRAM per engine) which widens the margin.

---

## 5. In the mission stack (already wired — just confirm)
- `vision_dual.launch.py` brings up both cameras + both detectors (start **paused**) + one HUD.
  **Caveat:** it OPENS BOTH camera streams (paused gates only detector inference) — see §2b; a
  single-camera task is safer on `vision.launch.py`.
- A vision verb / `use_camera('downward')` **auto-switches** the live detector (pauses forward,
  resumes downward, only ONE runs → VRAM/bandwidth) and the HUD auto-follows via the latched
  `/duburi/vision/active_camera` topic. Manual HUD keys still work: `f`=forward `d`=downward
  `D`=depth-map (the `b` side-by-side key was removed — the HUD streams one camera at a time).
- **Camera-switch settle:** `use_camera` sleeps `_CAM_SWITCH_SETTLE_S` (**1.5 s**, was 0.6 s) after
  a switch so the resumed detector's first live frames + the HUD re-latch land before the next verb
  steers — a switch happens once per task, so the headroom is free. Override per mission with
  `duburi.cam_switch_settle_s = <seconds>` before the switch.
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

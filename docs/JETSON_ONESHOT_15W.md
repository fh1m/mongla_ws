# Backup-Jetson one-shot prep — 15W Orin Nano 8GB, no WiFi (competition eve)

> **Why this exists.** The primary Jetson shorted. The backup is an **Orin Nano 8 GB**
> whose top power mode is **15 W** (no MAXN/"Super" 25 W), and it has **no WiFi card** —
> in the AUV it is reachable only over the Fathom-X **wired tether**. Tomorrow is the
> **last** testing day: this is a **one-shot**, so this runbook is ordered, copy-paste,
> **idempotent** (skip a step whose check already passes), and every step has a **verify
> checkpoint** — do NOT proceed past a failed check.
>
> This does not replace the reference docs — it **sequences** them for the one-shot and
> adds the 15 W-specific parts. Deep detail lives in:
> [`JETSON_SETUP.md`](JETSON_SETUP.md) (deps/build), [`remote-access.md`](../.claude/context/remote-access.md)
> (mosh/NoMachine/polkit), [`dual-camera-setup.md`](../.claude/context/dual-camera-setup.md)
> (camera udev), [`launch-combinations.md`](../.claude/context/launch-combinations.md) (every command).

## Two agents

| Agent | Where | Can do | Cannot do |
|-------|-------|--------|-----------|
| **A** (author) | dev box | Wrote + source-checked this runbook | Touch the board (no Jetson, engines are device-locked) |
| **B** (operator) | **on the Jetson, on the desk**, both cameras + BlueOS/Pixhawk connected, **wired Ethernet with internet** | Run every step, report each ✅/❌ | — |

**Ground rules for Agent B:** run top-to-bottom. After each step, paste the verify output. If a
check fails, STOP and report — do not continue. Everything installs over the desk Ethernet and is
baked to the SSD, so the tether-only AUV run needs no internet.

---

## Step 0 — Discover the board (report all of this back first)

```bash
cat /etc/nv_tegra_release                 # L4T / JetPack version
free -h                                    # confirm ~7.4Gi total (8GB board)
sudo nvpmodel -p --verbose 2>/dev/null | sed -n '1,30p'   # the mode MAP (names+numbers)
sudo nvpmodel -q                           # CURRENT mode
ping -c2 8.8.8.8                           # desk Ethernet has internet?
ls /dev/video* ; ls -l /dev/v4l/by-path/   # cameras enumerated?
```
**✅ Checkpoint:** 8 GB confirmed, JetPack noted, `ping` replies, both cameras appear. **Report the
`nvpmodel -p` mode map** — the highest-performance mode is what Step 6 selects (it is **mode 0** on
Orin; on this non-Super board that is its **top 15 W mode**, which is correct — there is no MAXN).

---

## Step 1 — Network (no WiFi → wired Ethernet)

The board has **one** Ethernet port. Use **DHCP now** (desk internet, for installing), then pin the
**AUV static IP** at the very end (Step 9) before the vehicle run.

```bash
# Now (desk install): DHCP is usually already active. Confirm internet:
ip -4 addr show ; ping -c2 github.com
```
> The AUV internal net (CLAUDE.md §2): Jetson **192.168.2.69/24**, gateway 192.168.2.2, BlueOS
> 192.168.2.1, MAVLink UDP 14550. You set that static IP in **Step 9**, not now.

**✅ Checkpoint:** `ping github.com` replies.

---

## Step 2 — Remote access (mosh + tmux + NoMachine + polkit)

All four layers are installed by the **idempotent** script (safe to re-run). NoMachine + mosh are
already installed per your note; the script wires tmux, the polkit `.pkla`, and NoMachine's Xorg
config. Pass the NoMachine `.deb` path if NoMachine still needs installing.

```bash
cd ~/Ros_workspaces/duburi_ws
tools/setup_remote_access.sh                       # or: tools/setup_remote_access.sh /path/nomachine_*.deb
```
**✅ Checkpoint:** from the **laptop**, `mosh <user>@<jetson-ip>` gives instant echo; `tmux new -s t`
works; NoMachine client connects to the Jetson. Full guide + laptop steps + emergency recovery:
[`remote-access.md`](../.claude/context/remote-access.md). **Live the whole session in `tmux` over
`mosh`** so a GUI wedge never kills a running mission.

---

## Step 3 — Cameras: named udev symlinks (forward + downward)

The two Blue Robotics cams are **identical and share one serial**, so identity is pinned by **USB
port** (`KERNELS`), not serial. The named symlinks (`/dev/duburi_cam_forward` / `_downward`) were
device-local on the burned board — **recreate them here.** The launches **auto-bind** these names,
so once they exist the launch line needs no `device_path`.

**3a — Physical disambiguation (Agent B, hands-on).** Plug both cams into the **upper** USB row.
Unplug ONE and watch which `by-path` entry disappears to learn which socket is forward vs downward:
```bash
ls -l /dev/v4l/by-path/          # note the two entries; unplug one; ls again -> which dropped
# For each cam's video node, read its port KERNELS (the interface node <hub>.<port>:1.0):
udevadm info -q all -n /dev/video0 | grep -E 'KERNELS=="[0-9]' | head -1
```
Decide: **forward = the socket you want as the front camera**; note its `KERNELS` (e.g. `1-2.1:1.0`),
and the downward socket's `KERNELS` (e.g. `1-2.3:1.0`). **Confirm YOUR live values — ports shift
across re-plugs; do not trust example numbers.**

**3b — Write the udev rule** (`ATTR{index}=="0"` picks the real MJPEG stream node; `MODE="0666"`
makes it readable without sudo). Substitute YOUR two `KERNELS`:
```bash
sudo tee /etc/udev/rules.d/99-duburi-cameras.rules >/dev/null <<'RULES'
# forward = upper-LEFT socket; downward = upper-RIGHT. EDIT KERNELS to your live ports (Step 3a).
SUBSYSTEM=="video4linux", KERNELS=="1-2.1:1.0", ATTR{index}=="0", SYMLINK+="duburi_cam_forward",  MODE="0666"
SUBSYSTEM=="video4linux", KERNELS=="1-2.3:1.0", ATTR{index}=="0", SYMLINK+="duburi_cam_downward", MODE="0666"
RULES
sudo udevadm control --reload && sudo udevadm trigger --subsystem-match=video4linux
ls -l /dev/duburi_cam_*
```
**✅ Checkpoint:** `duburi_cam_forward` and `duburi_cam_downward` both exist and point at a
`videoN`. They auto-recreate on every boot/replug. Details + the re-cable procedure:
[`dual-camera-setup.md`](../.claude/context/dual-camera-setup.md) §3.

---

## Step 4 — Python deps + build (over Ethernet, then baked to SSD)

Order matters — the **JetPack torch wheel FIRST**, then the ABI pins. Follow
[`JETSON_SETUP.md`](JETSON_SETUP.md) §1; the exact commands:

```bash
# 4a — Jetson torch/torchvision FIRST (CUDA wheel; PyPI torch is CPU-only on ARM).
#      Match the index to your JetPack (JP6 = cu122). Verify from Step 0's L4T.
pip install --no-cache --index-url https://pypi.jetson-ai-lab.dev/jp6/cu122 torch torchvision
python3 -c "import torch; print('cuda:', torch.cuda.is_available())"   # MUST be True

# 4b — the rest (lower-bound), then the ON-DEVICE exact ABI pins.
cd ~/Ros_workspaces/duburi_ws
pip install -r requirements.txt
pip install --no-deps --force-reinstall -r requirements-jetson.txt     # numpy==1.26.4, trackers==2.4.0
# DO NOT pip-install OpenCV (shadows the GTK system cv2 + drags numpy 2). If it crept in:
pip uninstall -y opencv-python opencv-python-headless 2>/dev/null || true

# 4c — web viewer video pipe (apt pkg, not a repo dep):
sudo apt install -y ros-humble-web-video-server

# 4d — build (mirrors device-local ~/models + ~/missions into the tree, then colcon):
source /opt/ros/humble/setup.bash
./build_dubomini.sh
source install/setup.bash
```
**✅ Checkpoint:**
```bash
python3 -c "import rclpy, cv2, numpy; print('numpy', numpy.__version__, '| cv2', cv2.__version__, cv2.__file__)"
# want: numpy 1.26.4  |  cv2 4.x under /usr (NOT ~/.local)
```
`cuda: True` (4a), numpy `1.26.4`, cv2 from `/usr`. If numpy is 2.x or cv2 is under `~/.local`, see
[`known-issues.md`](../.claude/context/known-issues.md) §E1–E3.

---

## Step 5 — TensorRT engines (the single biggest 15 W FPS lever)

> **⚠ BLOCKER FIRST — do the competition `.pt` weights actually exist on this board?**
> The YOLO weights are **gitignored** and were **device-local (`~/models`)** on the lost board.
> `build_dubomini.sh` mirrors `~/models` → the tree but **soft-skips a missing `~/models`** — on a
> fresh replacement board that folder likely does **not** exist, so the build silently proceeds with
> **zero weights**, `export_engine` builds nothing, and **there is no detector tomorrow.** This is the
> single most likely dead-end — resolve it now, over the desk Ethernet:
> ```bash
> ls -1 ~/models/*.pt src/duburi_vision/models/*.pt 2>/dev/null   # what weights actually exist?
> ```
> - If empty/partial: **copy the operator's competition weights** onto the board (into `~/models/`,
>   then re-run `./build_dubomini.sh`, or straight into `src/duburi_vision/models/`).
> - **Confirm every stem tomorrow's missions load has a `.pt`.** This session only
>   `bin_fire_blood.pt` + `gate_rescue_repair.pt` were confirmed present; **`slalom_red_pipe` and
>   `torpedo_blood_hole` had NO `.pt`** — if a task needs those, source them tonight or that task
>   cannot run regardless of Jetson prep.

The detector auto-prefers a `<stem>.engine` over the `<stem>.pt`. **Raw `.pt` is ~3–4 Hz —
UNUSABLE for vision alignment; the FP16 TensorRT engine is ~5–8× that.** Engines are
**device + JetPack locked**, so they MUST be built **on this board**. Models are **yolo11n** (nano —
right size for 15 W).

```bash
ls -1 src/duburi_vision/models/*.pt            # the weights that will get engines (must be non-empty!)
# Build FP16 engines at imgsz 640 (default; best small-target recall for the torpedo hole):
ros2 run duburi_vision export_engine --all
ls -1 src/duburi_vision/models/*.engine        # one .engine per .pt
```
**✅ Checkpoint:** a `.pt` **and** a matching `.engine` exist for **every model tomorrow's missions
load** (not a vacuous pass over an empty dir), and each `<stem>.yaml` sidecar sits beside it (class
labels come from it). **Keep 640 unless Step 7 says otherwise.**

---

## Step 6 — Power mode + clocks (+ make them survive a reboot)

```bash
# Select the HIGHEST-wattage mode from Step 0's map (mode 0 on Orin = this board's max; its
# top 15W mode, NOT MAXN on a non-Super board — expected). If your map numbers differ, use that.
sudo nvpmodel -m 0           # <- the highest-perf mode number from Step 0
sudo jetson_clocks           # pin clocks to that mode's ceiling
sudo nvpmodel -q             # confirm current = the selected mode
```
**`jetson_clocks` does NOT persist across reboot** (nvpmodel does). If the board is power-cycled and
this is skipped, detector FPS silently halves mid-run. Install a boot service so it's automatic:
```bash
sudo tee /etc/systemd/system/duburi-maxclocks.service >/dev/null <<'UNIT'
[Unit]
Description=Duburi: pin Jetson to top power mode + max clocks at boot
After=multi-user.target
[Service]
Type=oneshot
ExecStart=/usr/sbin/nvpmodel -m 0
ExecStart=/usr/bin/jetson_clocks
RemainAfterExit=yes
[Install]
WantedBy=multi-user.target
UNIT
sudo systemctl enable duburi-maxclocks.service
sudo systemctl start  duburi-maxclocks.service && systemctl is-active duburi-maxclocks.service
```
**✅ Checkpoint:** `nvpmodel -q` shows mode 0; `systemctl is-active duburi-maxclocks` = `active`.
(Path check: `which nvpmodel jetson_clocks` — if they differ from the unit, fix the `ExecStart` paths.)

---

## Step 7 — Measure sustained FPS → lock in imgsz (do the iteration NOW, on the desk)

You cannot iterate in the water — the **desk is the iteration window.** Measure the detector's
**sustained** rate and only drop imgsz if it can't hold the 20 Hz control loop.

```bash
# One camera, headless, detector active (paused:=false), on the real engine:
ros2 launch duburi_vision vision.launch.py camera:=forward viewer:=false \
    model:=gate_rescue_repair classes:=gate,rescue,repair paused:=false &
sleep 8
ros2 topic hz /duburi/vision/forward/detections     # watch ~30s of SUSTAINED rate, Ctrl-C
tegrastats --interval 1000                            # in another pane: watch GPU%, temp (Ctrl-C)
```
**Decision (≈29 Hz @640 expected on 15 W for yolo11n TensorRT):**
- **Sustained ≥ ~20 Hz → keep imgsz 640** (best torpedo-hole recall). Done.
- **Sustained < ~20 Hz** (thermal droop in a warm hull, or a heavier model) → rebuild smaller and
  launch matching:
  ```bash
  ros2 run duburi_vision export_engine --all --imgsz 512
  # then every launch/run adds:  imgsz:=512
  ```
  `imgsz` is **baked into the engine** — the runtime `imgsz:=` must match the export. 512 keeps more
  small-target recall than 416; only go 416 if 512 still can't hold 20 Hz.

**✅ Checkpoint:** sustained detector Hz ≥ 20 at the chosen imgsz; note the temperature isn't
thermal-throttling (`tegrastats` temp stops climbing). **Record the chosen imgsz** — every launch
below uses it.

**Then TEAR DOWN this launch before Step 8** — it holds the cameras open; a second launch on the same
`/dev/duburi_cam_*` collides (busy device / duplicate detector node):
```bash
kill %1 2>/dev/null; pkill -f vision.launch ; sleep 2
fuser /dev/duburi_cam_forward /dev/duburi_cam_downward 2>/dev/null || echo "cameras free"
```

> The 20 Hz control loop **freshness-decays** the translational command by frame age, so it never
> blind-drives on a stale bbox — but that means low FPS directly weakens alignment authority. FPS is
> the lever; that is why the engine + clocks + imgsz above matter.

---

## Step 8 — Verify the full mission surface (bringup + web viewer + DSL switching)

```bash
# 8a — preflight: every section should PASS (K = engines present; L = max-perf mode).
ros2 run duburi_manager bringup_check
# 8b — the pool-day console: both cameras, detections, live control. Auto-opens :8090.
ros2 launch duburi_vision mission_web.launch.py            # add imgsz:=512 if Step 7 chose it
#   -> browser shows BOTH camera panels with boxes burned in, detection table, /duburi/state.
```
Then prove **switching + DSL ↔ UI convergence** (mission authoring depends on it). There is **no
interactive DSL shell** — the DSL verbs (`use_camera`/`set_conf`/`set_model`/`detected`) run inside a
mission; the same switches are exposed by the **web UI** and by raw **ROS params**, and all three
write the *same* surface. Exercise each:

```bash
# 1) FROM THE UI: in the browser console, switch active camera, change the model dropdown, drag the
#    conf slider, toggle a class chip. Each must visibly take effect on the stream.
# 2) The SAME surface via ROS params (what set_conf/set_model write under the hood):
ros2 param get /duburi_detector_forward active_model
ros2 param set /duburi_detector_forward conf 0.50           # console conf readout follows
# (badge-follow only: a one-shot pub shows the console tracking active_camera, but does NOT do the
#  pause-others/resume-target exclusivity that the DSL's use_camera() / the UI switch do — use those
#  two for the real exclusivity test below.)
ros2 topic pub -1 /duburi/vision/active_camera std_msgs/String "{data: downward}"  # UI badge follows
# 3) FROM A DSL MISSION (exercises use_camera/detected/set_model end-to-end):
ros2 run duburi_planner mission --list                       # confirm the registry loads
ros2 run duburi_planner mission demo_dual_camera             # real camera-switch demo mission
```
Also click a detection **row in the web console** → it copies a ready DSL mission block. Switching
from the UI and from a running DSL mission are the same operation (both write the params + latched
`active_camera` above). Full command matrix: [`launch-combinations.md`](../.claude/context/launch-combinations.md).

**✅ Checkpoint:** `bringup_check` all-PASS; both camera streams smooth in the browser with synced
boxes; camera switch, model switch, conf, and `detected()` all work from **both** the DSL and the UI.

---

## Step 9 — Pin the AUV static IP (LAST — after all installing is done)

```bash
# Switch the wired port from DHCP to the AUV static address (nmcli example; adjust connection name):
sudo nmcli con mod "Wired connection 1" ipv4.method manual \
     ipv4.addresses 192.168.2.69/24 ipv4.gateway 192.168.2.2
sudo nmcli con up "Wired connection 1"
ip -4 addr show     # confirm 192.168.2.69
```
**✅ Final checkpoint (in the AUV, over the tether):** laptop can `ping 192.168.2.69`, `mosh` in,
`ros2 run duburi_manager bringup_check` all-PASS, MAVLink heartbeat present, `armed=false`. You are
mission-ready.

---

## 15 W quick-reference (what actually keeps FPS up)

| Lever | Why | Where |
|-------|-----|-------|
| **TensorRT `.engine`** (not `.pt`) | `.pt` = 3–4 Hz (unusable); FP16 engine = ~29 Hz @640 on 15 W | Step 5 |
| **`nvpmodel -m 0` + `jetson_clocks`** | 15 W max mode + pinned clocks; clocks don't survive reboot | Step 6 |
| **imgsz 640 → 512** *only if needed* | smaller = faster but less small-target recall; measure first | Step 7 |
| **8 GB → both detectors resident** | `mission_web` (vision_dual) keeps both models in VRAM; inactive is **paused** (no inference) | Step 8 |
| yolo11n (nano) models | already the right size for 15 W | Step 5 |

**Do NOT** disable tracking to save CPU: `vision.coast_s` defaults to 0.8 s and the control loop
steers on the tracker's coasted box during detection gaps (torpedo-hole lock, slalom-pipe hold) — the
detector is GPU-bound anyway, so tracking (CPU) isn't the bottleneck. **Do NOT** run INT8 engines —
they crater small-target recall (the hole). Depth/optical-flow distance nodes stay **off** (default).

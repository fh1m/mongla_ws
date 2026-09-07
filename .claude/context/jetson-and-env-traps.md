# Jetson / Pi environment traps — dependency pitfalls that take down the vision launch

> Extracted verbatim from the retired `known-issues.md` on 2026-09-08, because
> these are **environment** problems, not code bugs, and `requirements-jetson.txt`,
> `docs/JETSON_SETUP.md`, `docs/JETSON_ONESHOT_15W.md` and `tools/setup_pi_hailo.sh`
> all point at them. Code defects now live in [`BUGS.md`](BUGS.md).

These are **environment** problems, not code bugs — but they take down the whole
vision launch (`vision.launch.py`) with confusing tracebacks, so they live here.
All three were hit on the Orin Nano on the same day; symptom was every node dying
before the camera frame loop started.

> **Live-checked on the Jetson (Orin Nano, JetPack 6.2, py3.10) — 2026-06-30, by Claude.**
> After the three fixes below, `ros2 launch duburi_vision vision.launch.py` was run on
> the actual hardware and came up healthy: all 3 TensorRT engines loaded
> (`slalom_red_pipe` / `gate_rescue_repair` / `torpedo_blood_hole`) and the display
> reported `cam=OK det=OK trk=OK`. `trackers==2.4.0` confirmed importing and building
> `engine=ocsort` on numpy 1.26.4 on-device.

### E1. NumPy 2.x ABI break kills every vision node (`_ARRAY_API not found`)
- **Symptom:** every node (`camera_node`/`detector_node`/`vision_display`) crashes at
  `from cv_bridge import CvBridge` with
  `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.2.6 …`
  → `AttributeError: _ARRAY_API not found`.
- **Root cause:** a user-site `numpy 2.2.6` (`~/.local/lib/...`) shadowed the
  JetPack/ROS system numpy. ROS Humble's `cv_bridge` boost extension **and** the
  system `cv2` are compiled against the NumPy 1.x C-ABI and segfault under 2.x.
  `ultralytics` also pins `numpy<2.0.0`.
- **Fix:** `pip3 install "numpy==1.26.4"` (last 1.x; correct target for this stack).
- **Guard:** do not let any `pip install` pull numpy 2 back. If something forces it,
  reinstall 1.26.4 and that package with `--no-deps`.

### E2. `vision_display` crashes with `namedWindow … rebuild with GTK` (headless OpenCV)
- **Symptom:** `cv2.error: (-2:Unspecified error) The function is not implemented.
  Rebuild the library with … GTK+ … support` at `cv2.namedWindow`, then `exit code -6`.
- **Root cause:** pip `opencv-python-headless` (no GUI) + `opencv-python` were
  installed in user-site and **shadowed** the GUI-capable JetPack system OpenCV.
  The headless wheel wins → no window backend. (These wheels also want numpy≥2,
  compounding E1.)
- **Fix:** `pip3 uninstall -y opencv-python opencv-python-headless` → import falls
  back to the system `cv2` (4.12.0, **GTK3** build, numpy-1.x compatible). Verify:
  `python3 -c "import cv2; print(cv2.__file__)"` should be under `/usr/local/lib` or
  `/usr/lib`, **not** `~/.local`.

### E3. Roboflow `trackers` "unavailable" — the **2.5.0 PyPI wheel is broken**, NOT a numpy pin
- **Symptom:** `tracker_node` logs
  `roboflow trackers unavailable (… Install: pip install trackers); falling back to
  legacy_bytetrack`, even though `pip show trackers` reports it installed.
- **Root cause (the trap):** the **`trackers 2.5.0` wheel on PyPI is a 9.7 kB dud** —
  it ships only `dist-info` metadata + a CLI stub and **contains no `trackers/`
  package**, so `import trackers` → `ModuleNotFoundError`. `--force-reinstall` just
  reuses the same empty wheel. The `numpy>=2.0.2` pin in its metadata is a **red
  herring** — it is purely conservative; the code runs fine on numpy 1.26.4.
- **Fix:** install the **last good release, `2.4.0`** (126 kB, real code), with
  `--no-deps` so it can't drag numpy 2 / `opencv-python` back and re-trigger E1/E2:
  ```bash
  pip3 install --no-deps --force-reinstall "trackers==2.4.0"
  ```
  Verified on numpy 1.26.4: `tr.OCSORTTracker` + `tr.ByteTrackTracker` instantiate
  and run real `update()` calls; the node wrapper builds `engine=ocsort`. So OC-SORT
  (the documented default) **does** work on this Jetson — you do **not** have to
  accept the legacy ByteTrack fallback.
- **Watch:** if a future `pip install` upgrades to `trackers 2.5.0`, the engine
  silently disappears again (broken wheel). Re-pin to `2.4.0 --no-deps`. Re-evaluate
  when Roboflow ships a `>2.5.0` whose wheel actually contains the module.

> **One-shot recovery (all three at once):**
> ```bash
> pip3 install "numpy==1.26.4"
> pip3 uninstall -y opencv-python opencv-python-headless
> pip3 install --no-deps --force-reinstall "trackers==2.4.0"
> ```
> Benign remaining log noise (safe to ignore): numpy "smallest subnormal … is zero"
> UserWarning (aarch64 build quirk), TRT `NvMapMemAlloc … error 12` / "engine plan
> across different models of devices", and the `target=None deprecated` FutureWarning.

### E4. cv2 windows die under VSCode Remote-SSH — `Can't initialize GTK backend` (no `$DISPLAY`)
- **Symptom (distinct from E2!):** launched from a **VSCode Remote-SSH / plain-ssh**
  terminal, `vision_display` crashes at `cv2.namedWindow` with
  `Can't initialize GTK backend in function 'cvInitSystem'` and exit code 1. The
  detector then takes ~15 s to SIGKILL (TensorRT load blocks the SIGINT handler —
  benign). E2 was *headless OpenCV* (no GTK compiled in); **E4 is the opposite** —
  OpenCV *has* GTK, but a headless SSH shell has **no display server** (`$DISPLAY`
  empty), so the GUI has nowhere to draw. (Over the old full remote-desktop session
  it worked because that terminal inherited the desktop's `DISPLAY`.)
- **Root cause:** the GNOME/Xorg session runs on display **`:1`** (owned by the same
  `duburi-jetson` user; socket `/tmp/.X11-unix/X1`). A VSCode Remote-SSH integrated
  terminal starts with `$DISPLAY` unset and never inherits it.
- **Fix (host-local, in `~/.zshrc`):** when `$DISPLAY` is empty, auto-point GUI apps
  at the live local X socket — guarded so it never clobbers a real desktop terminal:
  ```sh
  if [ -z "$DISPLAY" ]; then
      for _d in /tmp/.X11-unix/X*; do
          [ -S "$_d" ] && export DISPLAY=":${_d##*/X}" && break
      done; unset _d
  fi
  ```
  `DISPLAY=:1` alone is enough (it falls back to the valid `~/.Xauthority` cookie).
  Verified on-device: a fresh headless zsh resolves `DISPLAY=:1` and `vision_display`
  opens its HUD without error.
- **Where the window appears:** on the **Jetson's** display `:1` — so you still *view*
  it via remote desktop / VNC, but **all editing + launching happens in VSCode
  Remote-SSH** (the latency win). NOT in the committed `.vscode/settings.json`:
  hardcoding `DISPLAY` there would break a teammate's *local* VSCode (forcing `:1`
  over their real `:0`). To drop remote desktop for *viewing* too, expose the
  annotated `…/image_debug` topic via `web_video_server`/Foxglove (browser over
  VSCode's auto port-forward) — not installed today; future task.

### E5. Payload CH340 has no `/dev/ttyUSB*` after carrier-board/SSD swap — **2026-07-10**
- **Symptom:** `lsusb` shows `1a86:7523 QinHeng Electronics CH340` (payload ESP32
  board is on the bus), but `start … -p payload_port:=auto` logs
  `[PAYLOAD] no port found (auto-detect excluded: set())` and `fire()` becomes a
  log-stub. `ls /dev/ttyUSB*` → nothing (only `/dev/ttyACM0`, the BNO085).
- **Root cause (two independent, both introduced by the 2026-07 carrier-board +
  SSD swap onto a fresh Tegra kernel):**
  1. **Kernel missing the CH341 driver.** The new `5.15.185-tegra` kernel shipped
     with `# CONFIG_USB_SERIAL_CH341 is not set` — no `ch341.ko` anywhere in
     `/lib/modules/$(uname -r)`. The CH340 enumerates on USB but the kernel never
     creates `/dev/ttyUSB*`, so pyserial `list_ports` (and thus payload
     auto-detect) sees nothing. `cdc_acm` (the BNO085 ESP32-C3) is unaffected — it
     needs no vendor driver, which is why the BNO worked and the payload didn't.
  2. **`brltty` steals the CH340.** Ubuntu's braille-display driver claims any
     `1a86:7523` via a udev rule and holds it through `usbfs` (interface driver
     shows `usbfs`), blocking `ch341` even once the module exists — the classic
     Arduino/ESP-on-Ubuntu trap.
- **Fix (durable, idempotent):** run **`tools/install_ch341_driver.sh`**. It purges
  `brltty` + its udev rule, builds `ch341.ko` out-of-tree against the running
  kernel headers (fetches the 5.15 `ch341.c`), `depmod`s it in (so `modules.alias`
  auto-loads it for `1a86:7523` on any port at boot/replug), loads it, and rebinds
  the already-attached board without a physical replug. Re-run after any kernel
  update / SSD reflash. Requires `/lib/modules/$(uname -r)/build`, gcc, make, curl.
- **Verified on-device (Orin Nano, 2026-07-10):** after the script, `/dev/ttyUSB0`
  (`usb-1a86_USB_Serial-if00-port0`, driver `ch341`) appears, `PayloadDriver`
  auto-detects + connects, and `bringup`/`start` re-connect the payload.
- **Self-diagnosing now:** `PayloadDriver.connect()` calls
  `_diagnose_missing_port()` — when the board is on the bus but has no tty node it
  logs the actual cause (`held by brltty (usbfs)` / `no ch341 driver bound` /
  `bound but no node yet`) and points at the script, instead of the old blank
  `no port found`.

---

---

## Forks we evaluated (so we don't revisit)

### `BumblebeeAS/ardupilot_fix` — STALE DUD (evaluated 2026-04)

* **Source:** https://github.com/BumblebeeAS/ardupilot_fix
* **State vs upstream:** **1 commit ahead, 3911 commits behind** `ArduPilot/master`. 0 stars, 0 forks. No CI configured.
* **The single commit** (`xelisce`, 2025-05-23, "hard code variables into file fix, passed all tests"): adds 9 unused declarations to `libraries/AP_DDS/AP_DDS_Client.cpp`. No semantic ArduSub change. No new mode, no new failsafe, no new MAVLink behaviour.
* **Verdict:** nothing to learn or pull. The fork name suggests a fix for something interesting but the diff is non-semantic. Stay on the upstream Sub-stable-V4.5.x branch documented in [`ardusub-canon.md`](./ardusub-canon.md).
* **Re-evaluate when:** the fork's `xelisce` author (or `BumblebeeAS` org) ships a second semantic commit. Until then, do not spend an evening "evaluating" this again.

---

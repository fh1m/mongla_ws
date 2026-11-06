# Environment traps — dependency pitfalls that take down the vision launch

> These are **environment** problems, not code bugs, but each one takes down the whole vision
> launch with a confusing traceback. Code defects live in [`BUGS.md`](../BUGS.md).
>
> E1–E3 were first hit on the old companion computer and are **still live on the Pi**: they
> are Python packaging traps (NumPy ABI, a shadowed OpenCV, a broken wheel), not properties of
> any one board. `tools/setup_pi_hailo.sh` and
> [`pi-hailo-vision-box.md`](pi-hailo-vision-box.md) both point here.

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
  installed in user-site and **shadowed** the GUI-capable system OpenCV.
  The headless wheel wins → no window backend. (These wheels also want numpy≥2,
  compounding E1.)
- **Fix:** `pip3 uninstall -y opencv-python opencv-python-headless` → import falls
  back to the system `cv2` (GTK build, numpy-1.x compatible). Verify:
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
  (the documented default) **does** work on the Pi — you do **not** have to
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
> UserWarning (aarch64 build quirk) and the `target=None deprecated` FutureWarning.

### E4. cv2 windows die over SSH — `Can't initialize GTK backend` (no `$DISPLAY`)
- **Symptom (distinct from E2):** launched from a VSCode Remote-SSH or plain `ssh`
  terminal, `vision_display` crashes at `cv2.namedWindow` with
  `Can't initialize GTK backend in function 'cvInitSystem'`. E2 was OpenCV *without*
  GTK; here OpenCV *has* GTK, but the SSH shell has no display server — `$DISPLAY`
  is empty, so the window has nowhere to draw.
- **Fix (host-local, in `~/.zshrc`):** when `$DISPLAY` is empty, point GUI apps at
  the live local X socket, guarded so a real desktop terminal is never clobbered:
  ```sh
  if [ -z "$DISPLAY" ]; then
      for _d in /tmp/.X11-unix/X*; do
          [ -S "$_d" ] && export DISPLAY=":${_d##*/X}" && break
      done; unset _d
  fi
  ```
  Not in the committed `.vscode/settings.json`: a hard-coded `DISPLAY` breaks a
  teammate's local VSCode.
- **Better on the vehicle:** don't open a window at all. The annotated
  `…/image_debug` topic is served by `web_video_server` and Foxglove
  ([`foxglove-and-bags.md`](foxglove-and-bags.md)), viewable in a browser over
  VSCode's port-forward.

### E5. The SROT board has no `/dev/ttyUSB*` — `brltty` or a missing `ch341`
The board's USB serial is a CH340 (`1a86:7523`), which needs the kernel's `ch341`
driver. Two independent faults leave it on the bus (`lsusb` shows it) with no tty
node, so `find_srot_serial()` finds nothing:
1. **`brltty` steals it.** Ubuntu's braille-display daemon claims every `1a86:7523`
   through a udev rule and holds it via `usbfs`, blocking `ch341`. Ubuntu 24.04
   ships it by default. `tools/setup_pi_hailo.sh` purges it.
2. **The kernel has no `ch341`.** A custom or vendor kernel can ship with
   `CONFIG_USB_SERIAL_CH341` unset; `modinfo ch341` says whether this one has it.
`bringup_check --srot` grades both (section C) and prints the fix.

---


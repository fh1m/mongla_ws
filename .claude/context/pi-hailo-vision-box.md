# The Pi + AI HAT+ vision box — measured, built, and ready for srot over USB

> **Status: the box works.** Ubuntu 24.04 / ROS 2 Jazzy, `colcon build` green on
> all 6 packages, **829 autonomy tests passed / 0 failed**, and
> camera → Hailo-8 → `vision_msgs/Detection2DArray` running at **72.4 Hz on the
> live ROS graph**. Everything below was measured on the hardware, 2026-09-02.
>
> **Nothing in `duburi_ws`'s critical path was changed to achieve this.** The
> Jetson + BlueOS + Pixhawk stack that placed 8th at RoboSub 2025 is untouched.

## Why this box exists

The Jetson Orin is shortaged, price-inflated, fragile, and **one already burned
at RoboSub 2025** — with the entire vision pipeline on it. This is the
diversification path: a Raspberry Pi 5 + AI HAT+ (Hailo-8, 26 TOPS).

## The headline number

Full configuration — **two cameras, two detectors, and a 50 Hz control loop, all
as separate processes**, which is how the real stack runs:

| | measured |
|---|---|
| control-loop jitter | **sd 0.07 ms**, median 20.00 ms, max 21.16, **0 % late** |
| vision, active camera | **85.1 Hz** |
| CPU | **69.6 % idle** (cores 15–37 %) |
| ARM clock | **1.0 GHz** — the governor clocked it *down*; no demand |
| RAM | **291 / 3983 MB** |
| temp | 48 °C, no active throttle |

**A 4 GB Pi 5 runs the complete vision + control workload at 1 GHz with 70 % of
the CPU idle.** Jetson TensorRT on the same model class is 20–30 Hz.

### An error worth keeping, because it would have inverted the recommendation

The first jitter measurement put the control loops in **threads inside the
vision process** and read median 24.96 ms, **max 76.65 ms** — nearly four missed
cycles, which reads as a decisive argument for offloading control to srot.

It was a **harness artifact**: one process, one GIL. `auv_manager_node` and
`detector_node` are *separate processes*.

| control loop | median | max | jitter sd |
|---|---|---|---|
| threads inside the vision process (**wrong model**) | 24.96 ms | **76.65** | 6.38 |
| separate process, beside vision at 79 Hz (**real**) | **20.00 ms** | **20.25** | **0.03** |

CPU was ~20 % busy throughout — it was never compute. **Without that control the
recommendation would have been srot-for-a-reason-that-does-not-exist.**

## main vs srot, measured

| | main (Pixhawk + BlueOS) | srot |
|---|---|---|
| vision on the Pi | 79.2 Hz alone, 76.9 Hz with control | 79.2 Hz |
| control jitter, separate process | sd 0.03–0.07 ms | n/a — off-host |
| **host outbound MAVLink writes** | **80 Hz peak** (50 lock + 20 vision + 5 depth + 5 neutral-RC), one `_tx_lock`, one link | **2 Hz** (heartbeat only) |
| on-board control rate | ArduSub 400 Hz inner; our heading loop 50 Hz in Python | **500 Hz FreeRTOS**, heading/depth/allocation on-board |
| boxes | Pixhawk + BlueOS Pi + USB IMU + vision computer | one ESP32 + one USB-C cable + vision computer |

**srot's advantage is not host CPU — there is plenty.** It is that the host
stops being in any real-time loop: `heading_lock`, `motion_writers`,
`motion_yaw`, `motion_depth` and the 5 Hz neutral-RC heartbeat are **deleted,
not ported**, and 80 Hz of contended outbound writes becomes 2 Hz.

**The catch, stated plainly:** `vision_align` / `vision_move` are in srot's
`UNSUPPORTED_VERBS` today — the exact verbs this box exists to feed. And srot's
GATE 0 (axis config unknown) and GATE 2 (depth loop never closed) are open. So
build the Hailo backend against **main** first, where every verb works, and
carry it to srot when its gates close.

## The real unlock, and it is software

`VISION_LOOP_HZ = 20.0` (`motion_rates.py`). Perception now runs at **~79 Hz**
into a **20 Hz** consumer — **the AI HAT is ~4x faster than the loop consuming
it, so most of its speed is currently invisible.** That 20 Hz was chosen when
perception was 3–4 Hz on a `.pt`.

> ⛔ **CLEARED 2026-09-07 — third doc found asserting this blocker, and the
> other two were corrected the same day.** Both cameras are calibrated and
> the files SHIP: forward fx 851.23 / **73.88° air**, downward fx 1027.87 /
> **63.82° air**, ±0.7°, `calibrateCameraRO`, held-out validated. `K` and `D`
> publish on every frame (measured off the wire on the vehicle). The
> backwards state below — the simulator having a calibrated FOV the vehicle
> did not — is resolved in the vehicle's favour, and the sim's derived 57.7°
> is a *different lens*, not a disagreement.

~~**Blocked on one bench measurement that blocks both main and srot:** camera FOV
does not exist anywhere in `duburi_ws` — no HFOV, no calibration, `K`/`D`
published empty. Pixels cannot become radians without it. (`sim/.context/` has a
derived in-water **57.7°**; *the simulator has a calibrated FOV the vehicle does
not*, which is backwards.)~~

## What was installed, and the two traps that were live

Both of `BUGS.md` §E1–E3's documented traps were **present on this Pi**
and breaking `cv_bridge`:

```
~/.local had numpy 2.5.2   -> shadowed Noble's 1.26.4, which cv_bridge is compiled against
~/.local had opencv-python 5.0.0 -> shadowed the GUI-capable system cv2
```

**Noble ships `python3-numpy` 1.26.4 as the system package**, so on Jazzy the
`numpy<2` constraint is satisfied by the distro — it stops being a manual pin.

The fix, and why each part is needed:

```bash
pip uninstall numpy opencv-python          # let the system packages win
echo 'numpy<2' > ~/duburi_constraints.txt  # or pip silently drags numpy 2 back
pip install -c ~/duburi_constraints.txt 'supervision>=0.18.0' scipy
pip install --no-deps 'supervision==0.26.1' 'trackers==2.4.0'
```

**Three findings behind those lines:**
- **`supervision` 0.30.1 uses `np.long`** (`annotators/core.py:10`) — numpy-2-only.
  0.26.1 is the newest that imports on numpy 1.26.4. `trackers` fails only
  because it imports supervision.
- **`supervision` AND `trackers` both hard-require `opencv-python`**, so pip
  cannot install them without shadowing the system cv2. `--no-deps` is not
  optional — it is the only way. (CLAUDE.md already says this for `trackers`;
  it is equally true for `supervision`.)
- **A bare install that "succeeds" can still be wrong.** Installing supervision
  without the constraint reported OK — because pip had pulled numpy 2.5.2 back
  in, re-breaking `cv_bridge`. Always re-check `numpy.__version__` after any pip
  operation on this box.
- `trackers==2.4.0`'s `numpy>=2.0.2` metadata is a **red herring** — confirmed
  again here, it runs fine on 1.26.4.

`PYTHONNOUSERSITE=1` is **not** a workaround: `hailo_platform` lives in the same
`~/.local`. Fix the packages, not the path.

## Layout

```
~/duburi_ws          the srot branch, built (plain colcon build -- NOT
                     --symlink-install; mixing the two makes CMake try to
                     replace a real directory with a symlink and the build dies)
~/hailo_models       *.hef + the *.yaml sidecars, OUTSIDE the workspace so a
                     rebuild cannot delete them
~/pi_env.sh          source this in every terminal
~/hailo_bench        measurement harnesses
```

**The `.yaml` sidecar is mandatory, not advisory.** A `.hef` carries no class
names, so without it the allowlist is empty and the detector returns `[]` on
**every frame** with a single warning and no error.

Compiled and verified on-device — note class count is nearly free
(2 → 11 classes costs 0.04 ms):

| model | classes | baked threshold | hw FPS |
|---|---|---|---|
| gate_rescue_repair | 3 | 0.05 | 97.6 |
| bin_fire_blood | 2 | 0.05 | 98.2 |
| sauvc_sim | 11 | 0.05 | 97.8 |

## Camera settings, and why

```
640x360 MJPG, CAP_PROP_FPS 210, CAP_PROP_BUFFERSIZE 1
capture thread + preprocess thread, single-slot drop-stale queues
```

- **The frame rate is a SETTING, not a bandwidth limit.** 30 → 30.2 Hz,
  90 → 68.8, **210 → 125.1**. JPEG decode is only 1.5 ms. An auto-exposure
  hypothesis was tested and **killed** — forcing manual exposure across
  50/100/200/400 changed nothing on either camera.
- **640x360 has the same field of view as 1280x720** — measured by patch-matching
  across scales: **scale 1.01, correlation 0.98**. Eyeballing it failed first
  because the scene moved between captures. The network input is a 640x640
  letterbox either way, so 720p was being downscaled to 640x360 anyway: the old
  setting cost half the frame rate for nothing.
- **`fps: 210` is a property of the bench Microdia**, not of the Pi. The second
  test camera (a Fantech webcam) caps at **15 Hz at every resolution**. **The
  vehicle's Blue Robotics cameras are unmeasured — re-measure before trusting
  210 there.**

## Dual camera

Both cameras run with **zero contention** (+0.0 % / +0.1 % vs solo) because they
sit on **separate USB controllers** (bus 002 / bus 004, per `lsusb -t`), and both
have stable `by-path` nodes — so select by `by-path`, never by index, or they
will swap on reboot.

One chip serves both: **85.1 Hz** on the active camera in the latched
`active_camera` arrangement `detector_node` already implements, and **24.3 Hz
per camera** even in the pessimistic both-infer-every-frame case. A **second
`VDevice` is refused** (`HAILO_OUT_OF_PHYSICAL_DEVICES 74`) — two independent
inference *processes* will not work; both HEFs go on one VDevice.

## Before this is flight hardware

**The PSU.** `vcgencmd get_throttled` has read `0x50000` in every sample —
no *currently active* throttle, but bit 16 says under-voltage **has occurred
since boot**, and the boot banner warns the supply cannot deliver 5 A. Tolerable
on a bench; not tolerable on the vehicle's only computer. **Verify the supply.**

## Next

1. Measure camera FOV (bench task, no Pi needed, blocks both main and srot).
2. `detection/factory.py` + `detection/hailo.py` — the seam is mapped; `.hef`
   needs adding at three extension-aware spots, and stem identity plus the
   sidecar lookup are already extension-agnostic.
3. Connect srot over USB and re-run the split with the board closing control.

**Humble and Jazzy must never share a DDS domain** — a Jazzy `ros2 topic list`
can drive a Humble subscriber out of memory (`ros2/rmw_fastrtps#797`). This is
per-vehicle all-or-nothing, never a mixed graph.

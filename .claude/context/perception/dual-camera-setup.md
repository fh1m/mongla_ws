# Two cameras on one vehicle

Forward and downward, both plain USB cameras, both on the Pi. Three things make this harder
than plugging in two webcams, and each has bitten us.

---

## 1. Which camera is which

Two identical cameras enumerate in whatever order the kernel probes them, so `/dev/video0` is
not a name — it is a race. Address a camera by the **USB port it is plugged into**
(`/dev/v4l/by-path/...`), which is stable across reboots and re-plugs, and give each port a
role in the launch profile.

**Warning: Roles have been wrong before.** The global-shutter camera is the **downward** one and the
rolling-shutter camera is **forward**; a udev rule once bound those names backwards, so
`pi_forward` opened the bottom camera. Everything downstream — calibration, field of view,
range — was then attached to the wrong physical lens for nine days.

**Check, do not assume:**

```bash
ls -l /dev/v4l/by-path/                       # which port is which
ros2 run mongla_vision vision_check --camera forward
```

Point something recognisable at one camera and confirm *that* camera's stream moves.

## 2. Bandwidth

Two uncompressed streams do not fit on one USB bus. Both cameras run **MJPEG**, and the
profiles cap resolution and frame rate to what the bus and the decoder can carry. If a camera
silently drops to a lower rate, or one stream dies when the second starts, bandwidth is the
first suspect — try the cameras on separate controllers.

## 3. Freshness beats frame rate

A camera driver's queue hands you the **oldest** frame it is holding. Underwater that is a
frame from a vehicle that has since moved. The camera path keeps a **mailbox** — one slot,
newest wins — measured at **16.9 ms** stale against **396 ms** for an ordinary queue.

Publishing *slower* can therefore give *fresher* detections: the topic is a mailbox too, and
surplus frames are cost without value.

---

## Calibration belongs to the camera, not the role

Each physical camera has its own measured calibration in
`mongla_vision/config/calibration/`, including its field of view in air, measured, and in water, computed by Snell's law
(**63.8° → 46.7°** for the downward camera, 73.9° → 53.6° for the forward one; neither yet
checked in water). A calibration file follows the **lens**, never the role
it is playing — that is the mistake the role swap made expensive.

## Running both

```bash
ros2 launch mongla_manager bringup.launch.py vision:=true      # the vehicle: both cameras
ros2 launch mongla_manager bringup.launch.py vision:=true batighor:=true   # and in a browser
```

Batighor (`http://mongla.local:8000/`) shows each camera's own JPEG with the detections drawn
by the browser, and its helm switches the live camera. On a bench with no board, run
`ros2 launch mongla_batighor batighor.launch.py` beside `vision_dual.launch.py`.

Related: [`camera-and-calibration.md`](camera-and-calibration.md) ·
[`camera-latency.md`](camera-latency.md) ·
[`downward-camera.md`](downward-camera.md) (the axis remap) ·
[`packages/mongla_vision`](../packages/mongla_vision/README.md)

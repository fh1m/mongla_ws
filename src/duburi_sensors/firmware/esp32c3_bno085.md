# ESP32-C3 + BNO085 — Firmware Contract

This is the wire spec the Jetson-side `BNO085Source` expects. The
reference sketch [esp32c3_bno085.ino](esp32c3_bno085.ino) is verbatim
the firmware running on our test bench (verified streaming `{"yaw":...,
"ts":...}` lines into `duburi_sensors.sources.bno085.BNO085Source`).

> **Why no magnetometer?** Eight thrusters + aluminum (Marine 5083)
> hull + battery currents make the inside of the AUV magnetically
> hostile. We use the BNO085's **gyro-integrated rotation vector**
> (`SH2_GYRO_INTEGRATED_RV`) -- gyro-only, magnetometer DISABLED -- and
> let the Jetson capture a one-shot Earth-reference offset from the
> Pixhawk's magnetometer at boot (sub at the surface, clean magnetic
> environment). After that the BNO is pure-gyro: smooth, immune to
> in-hull magnetic interference, and the only drift is the BNO's own
> bias-stabilised gyro (~0.5°/min typical).

## Wire format

One JSON object per line, newline-terminated, UTF-8.

```json
{"yaw": 123.45, "ts": 12345}
```

| Field | Type   | Required | Notes                                                       |
|-------|--------|----------|-------------------------------------------------------------|
| `yaw` | float  | yes      | degrees in `[0, 360)`, **sensor frame**, +CW about gravity. |
| `ts`  | int    | no       | ms since MCU boot — diagnostics only.                       |

`yaw` is **sensor-frame**, not magnetic-north. The Jetson side adds the
locked Earth-reference offset (`pixhawk_yaw - bno_raw_yaw` captured
at boot). Do not try to apply a heading offset in firmware.

Anything else in the JSON object is ignored. Lines that do not start
with `{` are ignored, so `Serial.println("boot ok")` and friends are
free to use during bring-up.

## Transport

| Setting   | Value                                              |
|-----------|----------------------------------------------------|
| Interface | USB CDC (the C3's native USB serial)               |
| Baud      | `115200`                                           |
| Stream    | continuous; host opens the port, then `readline()` |
| Rate      | **~50 Hz** sustained (chip runs internal at 500 Hz; sketch throttles every 20 ms) |

Why 50 Hz?
* Control loops poll yaw at 10 Hz; stale threshold is 250 ms.
* 50 Hz gives ~12 fresh frames per stale window — comfortable headroom
  without saturating USB CDC.

## Sensor mode

The reference sketch enables `SH2_GYRO_INTEGRATED_RV` (defined in the
SH-2 ChipFlow protocol as the gyro-integrated rotation vector). It is
the magnetic-interference-immune mode the AUV expects; do **not** flip
the `#define FAST_MODE` gate to `SH2_ARVR_STABILIZED_RV` for missions
(that mode is mag-fused).

## Wiring (matches the reference sketch)

| BNO085 pin | ESP32-C3 GPIO | Notes                                  |
|------------|---------------|----------------------------------------|
| SDA        | GPIO 4        | I²C data, 400 kHz                      |
| SCL        | GPIO 5        | I²C clock                              |
| RST        | GPIO 3        | optional; tie to 3.3V and use `-1` if you skip |
| VIN        | 3V3 / 5V      | breakout has its own LDO; either works |
| GND        | GND           |                                        |

Mounting orientation **does not matter**: the calibration step on the
Jetson captures whatever rotation exists between the BNO chip and the
AUV body and bakes it into the offset. Mount it **rigidly** though --
vibration on the breakout is the most common cause of jittery yaw.

## Reference firmware

The canonical sketch lives at
[`esp32c3_bno085.ino`](esp32c3_bno085.ino). Build with the Arduino IDE
or `arduino-cli`:

```bash
arduino-cli core install esp32:esp32
arduino-cli lib install "Adafruit BNO08x"
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C3 esp32c3_bno085.ino
arduino-cli upload  --fqbn esp32:esp32:XIAO_ESP32C3 -p /dev/ttyACM0 esp32c3_bno085.ino
```

Snippet of the critical loop body (the full file lives next to this
doc):

```cpp
if (millis() - lastPrint > 20) {
  lastPrint = millis();
  Serial.printf("{\"yaw\":%.2f,\"ts\":%lu}\n",
                ypr.yaw, (unsigned long)millis());
}
```

## Bring-up checklist

1. Flash the sketch. Open Arduino Serial Monitor at 115200 baud.
2. Confirm one JSON line per ~20 ms (`{"yaw":...,"ts":...}`).
3. Slowly rotate the breakout 360° on a flat surface — `yaw` should
   sweep monotonically through `[0, 360)` once per rotation.
4. Hold still for 30 s — `yaw` should drift no more than ~0.25°.
5. Plug into the Jetson and let auto-detect pick the port:

   ```bash
   ros2 run duburi_sensors sensors_node --ros-args \
       -p yaw_source:=bno085 -p bno085_port:=auto
   ```

   You should see:

   ```
   [SENS ] BNO085 auto-detect: probing N candidate(s)...
   [SENS ] BNO085 auto-detect: picked /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit-if00
   [SENS ] BNO085 reader started on /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit-if00 @ 115200
   [SENS ] yaw=...° healthy=True rx_hz=~50
   ```

6. With ArduSub SITL or a Pixhawk attached, switch on the calibration:

   ```bash
   ros2 run duburi_sensors sensors_node --ros-args \
       -p yaw_source:=bno085 -p calibrate:=true \
       -p bno085_port:=auto
   ```

   The first log line must be:

   ```
   [SENS ] BNO085 calibrated  pixhawk=NN.NN°  bno_raw=NN.NN°  offset=±NN.NN°
   ```

   After that, rotating the breakout by hand should rotate the printed
   yaw by the same angle, but reading the same Earth-frame as the
   Pixhawk reported at boot.

## Why `port: auto` (the new default)

The manager's default for `bno085_port` is now `auto`. On startup the
host probes:

1. `/dev/serial/by-id/usb-Espressif*`, `usb-Adafruit*`, `usb-Seeed*`,
   `usb-1a86*` (CH340/CH9102) — these stay stable across reboots.
2. `/dev/ttyACM0..3`, `/dev/ttyUSB0..3` — fallback for boards without
   a recognisable VID/PID symlink.

The first device that delivers a parseable `{"yaw":...}` line within
1.5 s wins, and the path is logged. Pin a specific device with
`-p bno085_port:=/dev/ttyACM0` to skip discovery.

## Definition of done

* Firmware boots in <2 s after USB enumeration.
* Streams ≥40 frames/sec sustained for ≥10 minutes.
* No malformed JSON lines (host parse-error counter stays at 0).
* Survives `bno08x.wasReset()` and continues streaming.
* `yaw` field stays in `[0, 360)`, monotonic under continuous rotation.
* `ros2 launch duburi_bringup mongla.launch.py yaw_source:=bno085` brings
  the sub up with no manual port specification.

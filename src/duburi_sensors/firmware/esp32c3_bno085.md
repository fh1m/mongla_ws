# ESP32-C3 + BNO085 — Firmware Contract

This is the wire spec the Jetson-side `BNO085Source` expects. Any MCU
that ships JSON-line-over-USB-CDC matching the contract below works;
the reference firmware target is an ESP32-C3 (Adafruit QT Py /
Seeed XIAO ESP32-C3 / SuperMini) talking to a SparkFun or Adafruit
BNO085 breakout over I2C.

> **Why no magnetometer?** Eight thrusters + aluminum (Marine 5083) hull + battery
> currents make the inside of the AUV magnetically hostile. We use the
> BNO085's **gyro + accelerometer-only** fusion (`SH2_GAME_ROTATION_VECTOR`)
> and let the Jetson capture a one-shot Earth-reference offset from the
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
| Rate      | **50 Hz target** (20-100 Hz acceptable)            |

Why 50 Hz?
* The control loops poll yaw at 10 Hz, stale threshold is 250 ms.
* 50 Hz gives ~12 fresh frames per stale window — comfortable headroom
  without saturating USB CDC.
* **Avoid exactly 200 Hz.** There is a documented BNO085 firmware bug
  where the chip stops emitting reports after a short period of
  perfect stillness at the 200 Hz update rate (see Adafruit forum
  thread *"BNO085 Behavior After Stillness"*). 50 Hz / 100 Hz are safe.

## Sensor mode

Use `SH2_GAME_ROTATION_VECTOR` only. Do **not** enable the geomagnetic
report and do **not** enable the magnetometer report.

Game rotation vector outputs a 6-axis fused quaternion (gyro +
accelerometer) with the chip's mag explicitly excluded — exactly what
we want.

## Mounting

Mounting orientation **does not matter** for the AUV-side software:
the calibration step captures whatever rotation exists between the
BNO chip and the AUV body and bakes it into the offset. Mount the
breakout however is convenient mechanically.

That said, mount it **rigidly** — vibration on the breakout is the
most common cause of jittery yaw. Foam-tape on a flat surface inside
the dry hull is fine; do not let it dangle off pin headers.

## Reference firmware (Arduino, Adafruit_BNO08x)

Tested on Adafruit QT Py ESP32-C3, BNO085 over I2C with `RST` wired
to GPIO 5 (recommended — lets us recover from chip resets).

```cpp
#include <Adafruit_BNO08x.h>
#include <Wire.h>

#define BNO_RESET_PIN  5            // wire BNO RST -> GPIO5; -1 if unused
#define REPORT_RATE_US 20000        // 50 Hz; do NOT use 5000 us (200 Hz)

Adafruit_BNO08x bno08x(BNO_RESET_PIN);
sh2_SensorValue_t sv;

static void enableReports() {
  // Game rotation vector = gyro + accel fusion, magnetometer DISABLED.
  // This is the only report the firmware enables, on purpose.
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, REPORT_RATE_US);
}

void setup() {
  Serial.begin(115200);
  // ESP32-C3 USB CDC: wait briefly for the host to open the port,
  // but don't block forever — the AUV may power-cycle without USB.
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { delay(10); }

  Wire.begin();                     // default SDA/SCL on the C3 board
  while (!bno08x.begin_I2C()) {
    Serial.println("{\"err\":\"bno085 init\"}");
    delay(500);
  }
  enableReports();
  Serial.println("boot ok");        // ignored by host parser
}

void loop() {
  // BNO085 occasionally resets itself; re-enable reports if so.
  if (bno08x.wasReset()) enableReports();
  if (!bno08x.getSensorEvent(&sv)) return;
  if (sv.sensorId != SH2_GAME_ROTATION_VECTOR) return;

  const float qw = sv.un.gameRotationVector.real;
  const float qx = sv.un.gameRotationVector.i;
  const float qy = sv.un.gameRotationVector.j;
  const float qz = sv.un.gameRotationVector.k;

  // ZYX yaw extraction (rotation about gravity):
  //   yaw = atan2(2(qw*qz + qx*qy), 1 - 2(qy^2 + qz^2))
  float yaw_rad = atan2f(2.0f * (qw * qz + qx * qy),
                         1.0f - 2.0f * (qy * qy + qz * qz));
  float yaw_deg = yaw_rad * 57.29578f;
  if (yaw_deg < 0) yaw_deg += 360.0f;

  // Single line, single allocation, newline-terminated.
  Serial.printf("{\"yaw\":%.2f,\"ts\":%lu}\n",
                yaw_deg, (unsigned long)millis());
}
```

## Bring-up checklist

1. Flash the sketch. Open Arduino Serial Monitor at 115200 baud.
2. Confirm one JSON line per ~20 ms (`{"yaw":...,"ts":...}`).
3. Slowly rotate the breakout 360° on a flat surface — `yaw` should
   sweep monotonically through `[0, 360)` once per rotation.
4. Hold still for 30 s — `yaw` should drift no more than ~0.25°.
5. Plug into the Jetson; confirm with the diagnostic node:

   ```bash
   ros2 run duburi_sensors sensors_node --ros-args \
       -p yaw_source:=bno085 -p bno085_port:=/dev/ttyACM0
   ```

   You should see `[SENSOR] yaw=...° healthy=True rx_hz=~50` in raw
   mode (no Earth reference applied).

6. With ArduSub SITL or a Pixhawk attached, switch on the calibration:

   ```bash
   ros2 run duburi_sensors sensors_node --ros-args \
       -p yaw_source:=bno085 -p calibrate:=true \
       -p bno085_port:=/dev/ttyACM0
   ```

   The first log line must be:

   ```
   [SENSOR] BNO085 calibrated  pixhawk=NN.NN°  bno_raw=NN.NN°  offset=±NN.NN°
   ```

   After that, rotating the breakout by hand should rotate the printed
   yaw by the same angle, but reading the same Earth-frame as the
   Pixhawk reported at boot.

## Definition of done

* Firmware boots in <2 s after USB enumeration.
* Streams ≥40 frames/sec sustained for ≥10 minutes.
* No malformed JSON lines (host parse-error counter stays at 0).
* Survives `bno08x.wasReset()` and continues streaming.
* `yaw` field stays in `[0, 360)`, monotonic under continuous rotation.

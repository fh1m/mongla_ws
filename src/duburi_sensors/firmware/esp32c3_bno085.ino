/*
 * ESP32-C3 + BNO085 -> Duburi yaw source
 * --------------------------------------
 * Reference firmware that ships JSON-line yaw over USB CDC at ~50 Hz.
 * The Jetson-side BNO085Source (duburi_sensors) opens this port,
 * parses one {"yaw":...,"ts":...} line per packet, and (on first run)
 * captures an Earth-reference offset against the Pixhawk's mag-fused
 * yaw. After that the BNO085 is the sole heading source -- gyro drift
 * only, no magnetometer interference inside the hull.
 *
 * Verified hardware:
 *   * MCU    -- ESP32-C3 SuperMini (also tested: Adafruit QT Py C3)
 *   * Sensor -- Adafruit BNO085 breakout, I2C
 *   * Wiring -- SDA=GPIO4, SCL=GPIO5, RST=GPIO3 (any free GPIO)
 *
 * Wire contract owned by ../sensors/bno085.py and documented in
 * esp32c3_bno085.md. Touch this file only if the contract changes.
 */
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// ====== PIN CONFIG ======
#define BNO08X_RESET 3   // any free GPIO; set to -1 if RST is tied to 3.3V
#define I2C_SDA      4
#define I2C_SCL      5

uint32_t lastPrint = 0;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

// ====== SENSOR MODE ======
//
// FAST_MODE selects SH2_GYRO_INTEGRATED_RV (no magnetometer, no fusion
// post-processing -- the chip just streams the integrated gyro
// quaternion at 500 Hz). This is the magnetic-interference-immune mode
// the AUV expects.  Leaving it commented out falls back to ARVR_RV
// (slower, mag-fused) which we DO NOT want inside the hull.
#define FAST_MODE

#ifdef FAST_MODE
sh2_SensorId_t reportType      = SH2_GYRO_INTEGRATED_RV;  // no magnetometer
long           reportIntervalUs = 20000;                  // 500 Hz internal
#else
sh2_SensorId_t reportType      = SH2_ARVR_STABILIZED_RV;
long           reportIntervalUs = 5000;
#endif

// ---- Helpers --------------------------------------------------------
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports...");
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable report");
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk,
                       euler_t* ypr, bool degrees = false) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw   = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll  = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw   *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll  *= RAD_TO_DEG;
  }
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rv, euler_t* ypr,
                         bool degrees = false) {
  quaternionToEuler(rv->real, rv->i, rv->j, rv->k, ypr, degrees);
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rv, euler_t* ypr,
                         bool degrees = false) {
  quaternionToEuler(rv->real, rv->i, rv->j, rv->k, ypr, degrees);
}

// ====== SETUP ======
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nESP32-C3 BNO085 I2C boot");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);                       // fast I2C

  if (BNO08X_RESET >= 0) {
    pinMode(BNO08X_RESET, OUTPUT);
    digitalWrite(BNO08X_RESET, LOW);
    delay(100);
    digitalWrite(BNO08X_RESET, HIGH);
    delay(300);
  }

  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x");
    while (1) delay(10);
  }

  Serial.println("BNO08x found");
  setReports(reportType, reportIntervalUs);
  Serial.println("Reading sensor data...\n");
}

// ====== LOOP ======
void loop() {
  if (bno08x.wasReset()) {
    Serial.println("Sensor reset detected");
    setReports(reportType, reportIntervalUs);
  }

  if (!bno08x.getSensorEvent(&sensorValue)) return;

  switch (sensorValue.sensorId) {
    case SH2_GYRO_INTEGRATED_RV:
      quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
      break;

    case SH2_ARVR_STABILIZED_RV:
      quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      break;

    default:
      return;
  }

  // Throttle the host-facing print to ~50 Hz. The internal sensor loop
  // runs at 500 Hz; printing every event would saturate USB CDC and
  // produce stalls every few seconds.
  if (millis() - lastPrint > 20) {
    lastPrint = millis();
    Serial.printf("{\"yaw\":%.2f,\"ts\":%lu}\n",
                  ypr.yaw, (unsigned long)millis());
  }
}

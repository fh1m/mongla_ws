# Vehicle spec — what is actually fitted

Canonical hardware reference. If this disagrees with the code, the code wins and this page is
the bug. Architecture and reasoning: [The Shift](../../docs/the-shift.md).

---

## The vehicle today

| Part | Fitted | Notes |
|---|---|---|
| Hull | Octagonal, marine 5083 aluminium, in-house | an aluminium hull is why a magnetometer cannot be trusted for heading |
| Frame | vectored, 8 × T200 | M1–M4 horizontal at 45°, M5–M8 vertical |
| ESCs | BLHeli_S reflashed to **Bluejay** | bidirectional DShot — this is what makes RPM telemetry exist |
| Flight controller | **SROT board**, firmware **Hengla** | ESP32 (dual core) + RP2350 Pico |
| Companion | **Raspberry Pi 5 + Hailo-8 AI HAT** | ROS 2 Jazzy on the vehicle |
| Link | one **USB-C** cable, MAVLink 2 @ 115200 | no network, no router, no BlueOS |
| IMU | BNO085, **on the board**, fused up to 400 Hz | read at 500 Hz by the control loop |
| Depth | Bar30 / MS5837, **on the board** | reported as altitude: negative below the surface |
| Leak · kill · water temp | on the board | kill state comes from the power board over ESP-NOW |
| Batteries | **two packs**: electronics (PM1) and thrusters (PM2) | de-multiplex `BATTERY_STATUS` by `id` |
| Cameras | 2 × USB — forward and downward | measured calibration in `duburi_vision/config/calibration/` |
| Radio | LoRa (SX127x) to the Bondor ground station | ⛔ needs an external antenna: an aluminium hull is a Faraday cage |
| Payload | the board's own outputs; `fire(N)` = **board channel N** (1–16) | the board's channel role decides whether it fires |

### Measured, not assumed

| Quantity | Value | Where |
|---|---|---|
| Forward camera field of view | **63.8° air · 46.7° water ±0.7°** | `measured-bars.md` §3 |
| Heading drift at rest | **< 0.01 °/min**, p2p < 0.08° over 8 min | `measured-bars.md` |
| Downward camera as a velocity sensor | 30 cm slides, worst error **1.09 cm** | `measured-bars.md` §13 |
| Link load | **18.8 %** of the serial budget | `measured-bars.md` §4 |

## Not fitted

| Thing | Status |
|---|---|
| **DVL** (Nortek Nucleus 1000) | never fitted, never validated in water. Distance comes from the downward camera; the `*_dist` verbs are refused on srot |
| **External IMU board** (ESP32-C3 + BNO085) | removed 2026-08-01 — the IMU is on the SROT board |
| **Pixhawk / ArduSub / BlueOS** | replaced. Preserved on the `pixhawk` branch; see [`legacy-pixhawk-and-sitl.md`](legacy-pixhawk-and-sitl.md) |
| **Hydrophone** | none. Any task that identifies a prop acoustically is not separable for us |
| **Thrusters, currently** | not mounted for bench work — 958/958 ESC frames read zero, which is why thruster health reads UNKNOWN |

## Two bodies, one codebase

**Duburi 4.5** (primary; grabber / dropper / torpedo) and **Dubomini 2.0** (agile,
manipulator-free) run the same stack. Mongla is the soul; the hulls are bodies. The workspace
name `duburi_ws` and the `/duburi/*` namespace are back-compat and are not renamed.

## Where a number belongs

A measured constant lives in [`measured-bars.md`](measured-bars.md) with its method and its
bar, and tests read that file. A per-camera optical number belongs to the **physical camera**,
not to the role it is playing — the two were swapped once, and a 46.7° figure sat on the wrong
camera for nine days (17 % range error).

Related: [`srot-architecture.md`](srot-architecture.md) ·
[`srot-integration.md`](srot-integration.md) ·
[`pi-hailo-vision-box.md`](pi-hailo-vision-box.md) ·
[`capability-map.md`](capability-map.md)

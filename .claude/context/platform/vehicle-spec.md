# Vehicle spec — what is actually fitted

Canonical hardware reference. If this disagrees with the code, the code wins and this page is
the bug. Architecture and reasoning: [The Shift](../../../docs/the-shift.md).

---

## The vehicle today

| Part | Fitted | Notes |
|---|---|---|
| Hull | **the Mongla hull, 702.0 × 176.1 × 172.1 mm**, from the CAD | see [The body](#the-body-seven-thrusters) below. The octagonal 5083 aluminium hull earlier versions of this page named belongs to the older vehicles |
| Thrusters | **7 on the final hull** (5 in the CAD today) | the board still mixes for 8: see [The body](#the-body-seven-thrusters) |
| ESCs | BLHeli_S reflashed to **Bluejay** | bidirectional DShot — this is what makes RPM telemetry exist |
| Flight controller | **SROT board**, firmware **Hengla** | ESP32 (dual core) + RP2350 Pico |
| Companion | **Raspberry Pi 5 + Hailo-8 AI HAT** | ROS 2 Jazzy on the vehicle |
| Link | one **USB-C** cable, MAVLink 2 @ 115200 | no network, no router |
| IMU | BNO085, **on the board**, fused up to 400 Hz | read at 500 Hz by the control loop |
| Depth | Bar30 / MS5837, **on the board** | reported as altitude: negative below the surface |
| Leak · kill · water temp | on the board | kill state comes from the power board over ESP-NOW |
| Batteries | **two packs**: electronics (PM1) and thrusters (PM2) | de-multiplex `BATTERY_STATUS` by `id` |
| Cameras | 2 × USB — forward and downward | measured calibration in `mongla_vision/config/calibration/` |
| Radio | LoRa (SX127x) to the Bondor ground station | **Warning:** needs an external antenna: an aluminium hull is a Faraday cage |
| Payload | the board's own outputs; `fire(N)` = **board channel N** (1–16) | the board's channel role decides whether it fires |

### Measured, not assumed

| Quantity | Value | Where |
|---|---|---|
| Camera field of view | downward (Microdia global shutter): **63.8° air**, measured; **46.7° water** by Snell, not yet checked in water · forward (Fantech): 73.9° air, 53.6° water the same way | `measured-bars.md` §3; `src/mongla_vision/config/calibration/` |
| Heading drift at rest | **< 0.01 °/min**, p2p < 0.08° over 8 min | `measured-bars.md` |
| Downward camera as a velocity sensor | 30 cm slides, worst error **1.09 cm** | `measured-bars.md` §13 |
| Link load | **18.8 %** of the serial budget | `measured-bars.md` §4 |

## Not fitted

| Thing | Status |
|---|---|
| **DVL** (Nortek Nucleus 1000) | never fitted, never validated in water. Distance comes from the downward camera; there is no distance-closed move verb |
| **External IMU board** (ESP32-C3 + BNO085) | removed 2026-08-01 — the IMU is on the SROT board |
| **Pixhawk / ArduSub / BlueOS** | replaced by the SROT board. The old stack is preserved at the tag `last-with-pixhawk-and-sim` |
| **Hydrophone** | none. Any task that identifies a prop acoustically is not separable for us |
| **Thrusters, currently** | not mounted for bench work — 958/958 ESC frames read zero, which is why thruster health reads UNKNOWN |

## The body: seven thrusters

| count | what it is | state |
|---|---|---|
| **7** | **the final Mongla hull**: the five below, plus two roll thrusters, one on each side. All six axes are actuated | the CAD for the roll pair is not done. Their positions come from it, never from an estimate |
| **5** | the current CAD. Lateral tunnel pair 350.00 mm apart (sway and yaw), vertical tunnel pair 519.00 mm apart (heave and pitch), one axial unit in the nose (surge). Four ⌀84 mm tunnels, fineness 3.99. Roll unactuated | measured off Onshape: `mongla_control/config/hull_geometry.yaml`, mirrored in `geometric_allocation.py`, which refuses a roll demand |
| **8** | the old **Dubomini** vehicle: `vectored_6dof`, 8 × T200, M1–M4 horizontal at 45°, M5–M8 vertical | **still the matrix the firmware mixes for.** `allocation.py` mirrors it, because it predicts what the board will do, for as long as the board does it. [srot-control-board#39](https://github.com/RakibulIslam1/srot-control-board/pull/39) replaces it with a geometry table, where the roll pair is two rows |

So a number that came from a competition run (RoboSub 2023/2025) belongs to an older
vehicle, not to this hull. **Say which hull a number belongs to.**

The earlier vehicles, **Duburi 4.5** (grabber, dropper, torpedo) and **Dubomini 2.0** (agile,
no manipulator), ran the stack this repository grew from. The workspace name `mongla_ws` and
the `/mongla/*` namespace are kept for the test vehicle's tooling and are not renamed.

## Where a number belongs

A measured constant lives in [`measured-bars.md`](../measured-bars.md) with its method and its
bar, and tests read that file. A per-camera optical number belongs to the **physical camera**,
not to the role it is playing — the two were swapped once, and a 46.7° figure (Snell, from the air value) sat on the wrong
camera for nine days (17 % range error).

Related: [`srot-architecture.md`](srot-architecture.md) ·
[`srot-integration.md`](srot-integration.md) ·
[`pi-hailo-vision-box.md`](../kutum/pi-hailo-vision-box.md) ·
[`capability-map.md`](../capability-map.md)

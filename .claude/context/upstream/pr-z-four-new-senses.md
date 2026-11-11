# PR Z — four new senses, on the board's clock

**To:** the hardware team (parts, mounting, wiring) and the firmware team
(drivers, timing, the wire).
**Status:** written 2026-11-11, read against `srot-control-board` `origin/main`
`f1d3ba9`. Filed as four issues, one per sense, so each can be taken alone:
[#78](https://github.com/RakibulIslam1/srot-control-board/issues/78) lasers, [#79](https://github.com/RakibulIslam1/srot-control-board/issues/79) altimeter, [#80](https://github.com/RakibulIslam1/srot-control-board/issues/80) stereo pair,
[#81](https://github.com/RakibulIslam1/srot-control-board/issues/81) hydrophones.
**Order:** none is imposed. Whichever part arrives first is built first; each
section stands alone and names what it needs from the others.

## Why

Mongla has no DVL and no altimeter. Everything it knows about where it is comes
from the board's IMU and depth sensor and from two cameras. Three quantities
are inferred today that could be measured:

| quantity | how we get it today | what it costs us |
|---|---|---|
| height above the floor | a pool-depth prior minus the board's depth, or the tile size | the downward camera's velocity is only as metric as that prior; a sloped floor or an unknown pool is a scale error on every metre travelled |
| range to a prop | the size of its detection box against a known size | wrong for a partly visible or unknown object, and nothing for a wall |
| bearing to a sound source | nothing | an acoustic beacon is invisible to us |

Every published DVL-free underwater system we found pairs its camera with a
**measured** range (sources at the end). We own the board, the firmware, the
hull and the companion, so the range can be measured where it is cheapest and
stamped on the one clock that already times the IMU: the board's.

The pin budget, read from `HARDWARE.md` and `include/config.h`:

| where | free | notes |
|---|---|---|
| ESP32 | GPIO13, GPIO14 | fully free; PR W asks for them as `CAM_TRIG` and `CAM_STROBE` |
| ESP32 | GPIO12, GPIO15 | strapping pins, forced LOW at start-up: safe for an output whose off state is LOW |
| ESP32 | GPIO0 | BOOT strap, must be HIGH at boot |
| ESP32 | ADC1 (34, 35, 36, 39) | all four taken (LoRa IRQ, leak, battery voltage, battery current) |
| ESP32 | VSPI (18/19/23) | shared with LoRa and the SD card; another device needs a chip select |
| ESP32 | I2C bus 1 (25/26, 100 kHz) | OLED and PCA9685; room for another address |
| Pico 2 | everything but GP0/1 (UART), GP6 to GP13 (ESCs), GP15 (e-stop), GP25 (LED) | including its three ADC inputs, GP26 to GP28 |

The ESP32 has no analog input left, so anything that samples a waveform belongs
on the Pico or on its own small microcontroller. You know the board; where we
name a pin it is a proposal.

---

## 1. Twin parallel lasers beside the downward camera

**What it gives:** height above the floor, and with it the scale of every
downward-camera velocity, from the camera we already have.

Two dot lasers, mounted parallel to the downward camera's axis, a known
distance `b` apart. They put two dots on the floor that are always `b` apart
in the world, whatever the height. In the image they are `d = f * b / h`
pixels apart, so `h = f * b / d`.

With the downward camera's measured field of view carried into water (46.7
degrees across 640 px, so `f` is about 741 px) and `b` = 150 mm, which fits
the 176 mm hull:

| height | dot separation | height error for a 0.1 px centroid error |
|---|---|---|
| 0.5 m | 222 px | 0.04 % |
| 1.0 m | 111 px | 0.09 % |
| 2.0 m | 56 px | 0.18 % |
| 3.0 m | 37 px | 0.27 % |

Published laser scalers on ROVs report about 1 % in practice, with beam
parallelism as the limit, not the centroid. A third laser, off the line of the
first two, adds the floor's tilt.

**Hardware ask.**

- Two (or three) **green, 520 nm**, dot modules, 5 mW or less, 12 mm barrel,
  3 to 5 V. Not red: water absorbs 650 nm roughly ten times faster than
  520 nm, and at 3 m the light travels 6 m.
- A rigid mount that holds them parallel to each other and to the camera
  axis, with a way to adjust and lock each one. Parallelism is the whole
  accuracy: 1 mrad of error is 3 mm of baseline error at 3 m, which is 2 %.
- Behind a flat window, or in the camera's own housing.
- One logic-level MOSFET switching all of them from one pin.

**Firmware ask.**

- `LASER_EN` on **GPIO12**. It is forced LOW at start-up, so the lasers are
  off through boot without any code.
- **Strobe with the camera trigger (PR W).** A parameter `LASER_DIV`: the
  lasers are on during every Nth trigger pulse and off for the rest (0 = off,
  the default). The detector and the optical flow then see clean frames, and
  the host knows from `CAMERA_TRIGGER.seq` which frames carry dots.
- **An interlock the board already has the data for:** no laser output above
  a depth of `LASER_MIN_DEPTH` (default 0.3 m), when disarmed, or with the
  barometer unhealthy. Lasers that cannot switch on in air cannot be pointed
  at a person on the deck.

**Our half.** A dot finder on the laser frames (two bright green blobs on a
known line in the image, so the search is one row band), the height, and a
check against the depth sensor and the pool prior. It needs no new message.

**Without PR W** it still works with the lasers on continuously; the cost is
two dots in every frame the detector and the flow see.

---

## 2. An acoustic altimeter

**What it gives:** height in water where no camera sees the floor, and an
independent check on section 1.

Two ways to get it, cheapest first.

**a. A finished underwater ranging module.** The DYP-L04 family (sold for
pool-cleaning robots) is a potted ultrasonic sensor with a UART or RS-485
output: 5 to 24 V, about 20 mA, IP68 to 10 m of water, ranges of 2 to 300 cm
or 5 to 600 cm depending on the version, and a maker-stated accuracy of
±(0.5 cm + 0.5 % of range) inside 2 m. The accuracy is the vendor's figure; we
found one paper that tested it in water and could not read its numbers. Ask
the seller for the UART, 600 cm version.

**b. Our own, from `open_echo`.** Jan Neumann's open-source echo sounder is
built around TI's TUSS4470 ultrasonic driver: it bursts a transducer from 40
to 1000 kHz and returns the echo envelope as an analog voltage. Schematics,
layout and firmware are public. It gives the whole echo, not one number, so a
second return (a prop above the floor) is visible. It needs an analog input
sampled fast, which the ESP32 no longer has, so it would run on its own small
microcontroller or the Pico's ADC and report over UART or I2C.

Start with (a). It is a part and a connector. Keep (b) for when one number is
not enough.

**Hardware ask.** One module pointing straight down, clear of the thruster
wash, at least its blind zone above the hull's lowest point. A second one
pointing forward is the same part again and gives range to a wall.

**Firmware ask.**

- Read it on a spare UART: the Pico has free pins for one; the ESP32's third
  UART would have to use strapping pins.
- Stamp each reading with the **board's** time and send it as MAVLink
  `DISTANCE_SENSOR` (132), `orientation` 25 (downward) or 0 (forward), with
  `min_distance` and `max_distance` set from the module, and
  `signal_quality` 0 when the module reports no echo. No echo must never be a
  distance of zero: suppress the message, as rev 3 does for other values.
- A health bit beside the barometer's, so `bringup_check` can grade it.
- Later, on the board: an altitude hold. Height above the floor held at
  500 Hz is the control mode a floor-following survey wants, and the board is
  the only place that can run it. Not part of this ask.

**Our half.** Decode `DISTANCE_SENSOR` in `SrotFC`, a `height_m` field in
`MonglaState` (NaN when absent), and the filter takes it as the scale for the
downward camera in place of the prior.

**A caution to test first:** thrusters and ESC switching are loud in the
ultrasonic band. Bench the module in a tank with a thruster running before it goes on
the hull.

---

## 3. A forward stereo pair, on the same trigger

**What it gives:** true range to every prop and wall in view, with no learned
network and no assumed size.

PR W asks for one triggerable global-shutter camera. Make it two. USB modules
built on the OV9281 (1 MP, monochrome, global shutter, up to 120 fps) expose a
trigger input (`FSIN`, 3.3 to 5 V, pulse 2 us or longer) and a strobe output.
The InnoMaker U20CAM-9281M is one such; AliExpress carries the same sensor on
similar boards. **Check the listing for the FSIN pads before buying: a module
without them cannot be triggered.**

Wire both `FSIN` inputs to the one `CAM_TRIG` pin. Both sensors then expose at
the same instant, on the board's tick, and the pair is synchronised by
construction. Nothing else in the firmware changes.

Range error for a baseline `b`, focal length `f` in pixels and a matching
error of 0.25 px is `Z^2 * 0.25 / (f * b)`. With `b` = 120 mm and a lens that
gives `f` of about 600 px in water:

| range | error |
|---|---|
| 1 m | 0.3 cm |
| 2 m | 1.4 cm |
| 5 m | 8.7 cm |

**Hardware ask.**

- Two identical modules with identical M12 lenses, about 80 to 90 degrees
  across in water. The stock 148 degree lens is too wide: it spends the pixels
  on the periphery.
- One rigid bar holding both, behind one flat window, baseline as wide as the
  hull allows. The calibration is only valid while that bar does not flex.
- Two USB 2.0 ports on Kutum. Each camera at 640x400 mono is well inside one
  port.

**Firmware ask.** None beyond PR W. If the trigger rate is ever shared with
section 1's lasers, the forward pair ignores `LASER_DIV`.

**Our half.** Calibration of the pair in water, rectification, and matching.
The Hailo-8 has about a third of its time free, and the HailoRT service lets
a second process use it (measured-bars 80), so the matching can run on the
chip or on the CPU, decided by measurement.

---

## 4. Hydrophones

**What it gives:** bearing to an acoustic pinger, which no camera can.

This is the largest of the four and the one to scope before buying anything.

Two hydrophones a known distance apart hear the same ping at slightly
different times; the difference is the bearing. For that the two channels must
be sampled at the same instant and several times faster than the pinger's
frequency.

**Hardware ask.**

- Two hydrophones (a third, off the line, removes the left/right ambiguity).
  Options from cheap to dear: a potted piezo disc (about 20 euros a unit in a
  published low-cost toolkit), or a finished hydrophone.
- A preamplifier and a band-pass filter per channel around the pinger's
  frequency, into the Pico's ADC inputs GP26 and GP27 (GP28 for a third).
- Spacing of half a wavelength at the pinger's frequency or less, so the
  phase difference is unambiguous: at 40 kHz in water that is about 19 mm.

**Firmware ask.**

- Sample two channels on the Pico's ADC with DMA. The RP2350's converter does
  500 thousand samples a second in total, 250 thousand a channel for two,
  which covers a pinger up to about 60 kHz with margin.
- Detect the ping on the Pico (energy in the band), measure the phase or
  time difference there, and send one small message per ping: board time,
  bearing, signal level, and a quality value. Raw samples never cross the
  link.
- A parameter for the pinger's frequency.
- **Check first that the sampling does not disturb DShot.** The Pico's first
  job is the ESCs. If it does, this becomes its own microcontroller.

**Our half.** The bearing as a heading measurement in the filter, and a verb
that turns toward it.

**Prior art:** the open-source `ahoi` acoustic modem (TUHH) shows ranging and
localisation with open hardware at about 600 euros, 400 of it the hydrophone.

---

## What we are not asking for

- A DVL. It is the instrument all of this replaces, at a hundredth of the
  price.
- An infrared time-of-flight or optical-flow chip. Water absorbs infrared
  within centimetres, and those chips are focused for air.
- A depth camera. Structured-light and time-of-flight depth cameras are
  infrared too.

## Sources

- Laser scalers: Istenic et al., "Automatic scale estimation of structure
  from motion based 3D models using laser scalers", arXiv 1906.08019; Hansen
  et al., "Short-range sensor for underwater robot navigation using
  line-lasers and vision", IFAC-PapersOnLine 48(16), 2015.
- Camera, depth sensor and IMU with the scale estimated online:
  https://hal-lirmm.ccsd.cnrs.fr/lirmm-01567463
- `open_echo`: https://github.com/neumi/open_echo
- DYP-L04: https://www.dypcn.com/ultrasonic-underwater-ranging-sensor-product/
- OV9281 trigger: https://www.inno-maker.com/product/u20cam-9281m/ and
  https://docs.arducam.com/UVC-Camera/Appilcation-Note/External-Trigger-Mode/OV9281-Global-Shutter/
- `ahoi` modem: Renner et al., "ahoi: inexpensive, low-power communication
  and localization for underwater sensor networks and AUVs", ACM TOSN 2020.

# The AUV architecture changed — read this before anything else in `srot-*`

> Written by the `srot-control-board` side, for the `mongla_ws` agent. Everything here is a
> statement about hardware and firmware that already exists and has been bench-verified, not a
> proposal. The proposals are in the last section and are labelled as such.

## What changed

The `srot` branch was written against a vehicle that no longer exists. It is an excellent
**bypass** of the Pixhawk path, but it was built on the assumption that the rest of the stack
stayed put. It did not.

**Old (what `srot`, `sensors-pipeline.md`, `vehicle-spec.md` and `dual-camera-setup.md`
still describe):**

```
Pixhawk (ArduSub) ──MAVLink──> Raspberry Pi (BlueOS) ──UDP──> Jetson
   ^                                                             |
   └── sensors on the Pixhawk;  a SEPARATE ESP32-C3 + BNO085 on USB ──┘
```

**New (what is on the bench today):**

```
        ┌────────────────────────── SROT control board ──────────────────────────┐
        │  ESP32 DevKit V1, dual-core FreeRTOS, 500 Hz control loop              │
        │                                                                        │
        │  I2C0: BNO085 IMU (0x4A)   ·   MS5837 Bar30 depth (0x76)               │
        │  I2C1: SH1106 OLED (0x3C)  ·   PCA9685 servo/MOSFET expander (0x40)    │
        │  UART2: RP2350 (Pico 2) thruster co-processor, per-motor RPM loop      │
        │  Attitude + depth hold, 6DOF vectored allocation, arming, failsafes    │
        └───────────────────────────────┬────────────────────────────────────────┘
                                        │
                              ONE USB Type-C cable, 115200
                                        │
                          ┌─────────────▼─────────────┐
                          │   Jetson Orin — GPU only  │
                          │   detector, cameras,      │
                          │   mission logic           │
                          └───────────────────────────┘
```

**No Pixhawk. No Raspberry Pi. No BlueOS. No UDP router. No separate IMU board.**

## The three things that follow from that

### 1. The BNO085 is on the control board now

This is the change with the largest footprint on your side, and it is the one nothing in
`mongla_ws` knows about yet.

**What we can state as fact, and what we cannot.** Fact: the SROT board carries a BNO085 and
fuses it, and the vehicle is wired board→Jetson on one cable. What we *cannot* see from here is
your side of the hull — whether the old ESP32-C3 + BNO085 board is still physically fitted, and
whether anything you have depends on it. So read the table below as "here is what has become a
duplicate **if** that board is gone", not as an instruction to delete code. You can see your
vehicle; we cannot.

The board carries the BNO085 on I2C0 and fuses it into the 500 Hz control loop. It already
publishes fused attitude over MAVLink (`ATTITUDE`, now pinnable to ~55 Hz). It is the *same
sensor part* your `mongla_sensors` package talks to over USB — one physical layer closer to the
thrusters, sampled 10× faster, and on the loop that actually uses it.

What that makes redundant on your side:

| File | Lines | Why it is now duplication |
|---|---|---|
| `mongla_sensors/bno085.py` | ~333 | Reads the same part over a second USB link |
| `mongla_sensors/_discovery.py` | ~155 | Discovers a board that is no longer fitted |
| `mongla_sensors/composite_bno_dvl.py` | — | Fuses a duplicate against the DVL |
| `firmware/esp32c3_bno085.ino` | — | Firmware for a board that no longer exists |
| the 5 s boot calibration + 50 Hz reader thread | — | Cost paid per boot, per tick, for a duplicate |

**We are not asking you to delete these.** Two honest reasons to keep them a while: they are
your fallback if the board's attitude turns out to be wrong in the water, and `yaw_source` is
already a runtime switch, so keeping the code costs nothing but load. What we *are* asking is
that `yaw_source` stop defaulting to a source that assumes hardware that is not fitted, and
that the reader thread not run when nothing consumes it.

The reason to care: on a 15 W Orin, every Hz not spent on the detector is a Hz wasted — and the
detector is **the only thing in this system the board cannot do**. Attitude, depth, allocation,
braking, heading hold and failsafes all moved down. Perception is what is left up top, and it
is the thing that decides whether the vehicle scores.

### 2. The firmware answered nine of your eleven `JETSON_FEEDBACK` items

Round 6 in `srot-control-board/AUDIT.md`, items **R35–R44**, bench-verified on hardware over
COM19. The companion-facing summary is `FIRMWARE_CHANGELOG_FOR_DUBURI.md` in that repo. The
short version:

Your own numbering, so it is checkable — the full per-item reply is
`FIRMWARE_CHANGELOG_FOR_DUBURI.md` §1–§11:

| Your item | Status |
|---|---|
| **§1** move stranded on `IN_PROGRESS` for ever | **FIXED** (R35) — every move reaches exactly one terminal ACK |
| **§2** `MOVE_STOP` applies zero braking thrust | **FIXED** — STOP restores the outgoing leg's axis + speed into `PH_BRAKE` |
| **§3** SURFACE keeps driving the last pilot sticks | **FIXED** — plus a failsafe-gated surface disarm |
| **§4** GCS failsafe not companion-specific | **NOT FIXED — needs your decision**, see below |
| **§5** `SET_MESSAGE_INTERVAL` | **IMPLEMENTED** (511 + 510) |
| **§6** `MV_STATE` off-by-one past 5; dead progress | **FIXED** (both halves) — real TURN/DIVE span/remaining |
| **§7** DIVE target not clamped ≥ 0 | **FIXED** |
| **§8** `ESC_STATUS` undecodable by pymavlink | **FIXED** — `ESC_TELEMETRY_1_TO_4`/`_5_TO_8` (11030/11031) emitted alongside |
| **§9** Round-3 notes | **FIXED** |
| **§10** documentation drift | **FIXED** |
| **§11** the v3 architectural ask → `VISION_API.md` | **OUR HALF IS BUILT** (2026-09-07) — the FOV blocker is CLEARED (below); `send_landing_target` ships, default-off because the board still drops msgid 149. Theirs to implement. |

Nine fixed, one open on your decision (§4), one deferred by agreement (§11).

Not on your list, found in our own audit and shipped in the same round: `movement::cancel()`
on the AUTO→other mode edge (a mode change used to leave a move running), and a
freshness + decay ramp on `MANUAL_CONTROL` (`MANUAL_FRESH_MS` / `MANUAL_DECAY_MS`) so a stale
stick stream ages out to neutral instead of holding thrust.

Also still open, and it is **your** finding: `NAMED_VALUE_FLOAT` multiplexing makes LEAK
detection probabilistic. Proposal below.

Also found and fixed **on hardware, not by reading**: a resolved move could emit its terminal
ACK repeatedly (~100 ACCEPTEDs for one DIVE) because the completion path used `s_seq = 0` as an
"already answered" sentinel and sequence 0 is a legal sequence. Now an explicit `s_resolved`
latch. That bug predates your srot branch — your `_ack_budget_s` deadline was masking it.

### 3. `SROT_FW_BEHAVIOUR_REV` is now the coordination signal

`include/config.h` carries `SROT_FW_BEHAVIOUR_REV`, currently **2**. Mirrored on your side as
`srot_protocol.FW_BEHAVIOUR_REV` / `FW_BEHAVIOUR_REV_REQUIRED`.

**It is on the wire, and it is checked at runtime.** The board reports it in
`AUTOPILOT_VERSION.middleware_sw_version` (we have no middleware, so the field was free);
request it with `MAV_CMD_REQUEST_MESSAGE(148)`. `SrotFC.check_behaviour_rev()` reads it at
connect and again inside `arm()`, and **refuses to arm** on a board that reports a revision
below the requirement. Hardware-verified: the board answers **2**.

This is deliberately not left to `test_firmware_behaviour_rev_is_new_enough`. That test reads
this repo's headers off disk and **skips when the firmware repo is not checked out beside the
workspace** — which is the situation on the Jetson, i.e. it skips in the only place the answer
matters. A pytest is a workstation convenience; the wire is the contract.

`0` means "firmware older than 2026-08-01", not "unknown" — that build never populated the
field, so the host fails closed on it. A board that does not answer *at all* warns loudly but
is allowed through, because refusing to arm on a dropped frame is its own hazard. The operator
override is `allow_fw_behaviour_mismatch:=true`, off by default.

This exists because the previous coordination mechanism — `test_srot_protocol_drift.py` grepping
our C++ for `abort()` inside the `Type::STOP` case — **stayed green through the entire fix**. We
ran it. A test that greps another repo's source text is not a version contract: it passes when
the code is refactored, when the call moves, when the comment changes. It cannot tell you what
the firmware *does*.

The double-brake window is the concrete reason this matters. Your `_brake_last_leg` sent an
explicit reverse leg before every `MOVE_STOP` because STOP coasted. STOP now brakes. Old host +
new firmware = **the hull is decelerated twice**. New host + old firmware = **stop coasts**.
Both are real, and neither is detectable by reading source. Check the number.

## The one thing blocking vision

Your split is right and `VISION_API.md` §8's staging is the correct order. One thing blocks it,
and it is not code:

> ⛔ **CLEARED 2026-09-07. This section describes a blocker that no longer
> exists, and it was the named critical path for the whole vision split — so
> a reader who trusts it defers work that is now unblocked.**
>
> Both cameras are calibrated, held-out validated, and the calibrations SHIP
> in `src/mongla_vision/config/calibration/`:
>
> | | fx (at 1280×720) | HFOV air | HFOV water | views |
> |---|---|---|---|---|
> | forward (Fantech) | 851.23 | 73.88° | 53.59° | 23 |
> | downward (global shutter) | 1027.87 | 63.82° | 46.72° | 25 |
>
> `±0.7°`, `calibrateCameraRO`, chosen by k-fold held-out reprojection error,
> AprilCal Max ERE reported. **`camera_info` publishes a real `K` and `D` on
> every frame** — measured off the wire on the vehicle: fx 425.61 at 640×360,
> which is 851.23 correctly rescaled. `mongla_control/bearing.py` does the
> pixel→radian conversion and agrees with an independent computation to
> 0.003°.
>
> Recalibrating is now an AUV command — `ros2 run mongla_vision calibrate` —
> with a guided browser tool, a library of saved calibrations, one-click
> re-apply to a swapped camera, and an in-water mode. See
> [`camera-and-calibration.md`](camera-and-calibration.md).
>
> **The one thing that is still assumed:** the *water* figures are derived
> from the air ones by Snell, not measured. See `BUGS.md` §1a.
>
> Kept below, struck through, because §11 and the migration order both point
> at it.

~~**The camera FOV numbers do not exist anywhere in `mongla_ws`.** No HFOV/VFOV parameter, no
calibration file, no checkerboard script; `K` and `D` are published **empty** on every frame.
The single focal number in the repo, `camera_focal_px: 500.0`, is explicitly commented as a
guess, and it is a guess made for a different purpose.~~

~~`LANDING_TARGET` carries **bearings in radians**. Pixels cannot become radians without a real
FOV. Until someone puts a checkerboard in front of both cameras at 640×480 and writes the two
numbers down, the bearing conversion is unbuildable — and every downstream stage inherits the
error. This is a bench task on your side. It is the critical path.~~

Two more vision points, both from reading your code:

- **Send bearings in the VEHICLE frame, not the camera frame.** Your downward camera rotates
  image-Y into surge and carries a `surge_sign = -1` mount fact. A wrong sign there is
  *positive feedback* — the vehicle accelerates away from the target. Keep that rotation on the
  Jetson, where `vision_thrust_check` already validates it **disarmed**, and the board's axis
  bitmask then means exactly one thing on both cameras.
- **Missing from your open-questions list:** `LANDING_TARGET` needs a **`coasted` bit** and a
  **true gap-age** field. Your `_authority()` branches on `coasted` so it does not
  double-decay a Kalman-coasted box. Without both on the wire, the board decays coasted targets
  roughly 2× too fast and kills the coast inside 0.4 s — well inside the timeout ladder
  (0.10 / 0.40 / 0.80 / 1.00 s). We propose carrying them in the unused `x`/`y`/`z` fields.

**Fire gating stays on the Jetson**, and your own code makes the argument: the stable-frame
counter is a three-valued state machine over *frame identity* (`_FRAME_EPS_S`), and a frozen
detector still emits a fresh-looking value stream. One honest consequence: `in_band` is
computed from the control axes today, so once control moves down, the Jetson must recompute
in-band from the bearings it is already sending. That is cheap — it has the deadband in radians
— and it preserves the frozen-detector guard, which is the whole point.

## Still open — these need your decision, not ours

- **The companion's component id.** We cannot make the GCS-loss failsafe source-specific:
  `JETSON_COMMS.md` tells companions to use `255/190`, and our own LoRa bridge synthesises
  `255/190`. They are indistinguishable on the wire, so a "the companion went away" failsafe
  would also fire when the radio link drops, and vice versa. **A distinct onboard-computer
  compid on your side unblocks it.** One constant for you, and we ship the failsafe.
  We deliberately did *not* ship a fix that looks like one and is not.
- **`arc` and `style_yaw`** are real firmware gaps. Absolute-heading arc; a selectable STYLE
  axis + rate. Not built. Say the word and they land — they are small.
- **LEAK off `NAMED_VALUE_FLOAT`.** Your finding: pymavlink caches one message per msgid, so
  with MV_STATE/LEAK/WTEMP/GAIN multiplexed, LEAK detection is probabilistic. Proposal: move
  LEAK onto `SYS_STATUS` sensor-health bits, where it is a dedicated bit nothing can overwrite.

## ⛔ The standing gate, unchanged

**The board's depth loop has never run closed.** Everything dive-dependent — `set_depth`, the
vision depth axis, `surface` — stays behind the two bench checks in `srot-integration.md`
step 4. A successful in-air `move_forward` is not partial validation of a depth loop, and
nothing in this document changes that.

## Migration order (suggested)

1. Take this PR's fixes; check `FW_BEHAVIOUR_REV` before flying anything (**the double-brake
   window is real in both directions**).
2. ~~Measure both cameras' FOV at 640×480. Nothing vision-shaped moves until this exists.~~ **DONE** — 73.88°/63.82° air, held-out validated, shipping in `config/calibration/` and live on `camera_info`.
3. Run the bench runbook, including the two depth checks. Then, and only then, dives.
4. Stop defaulting `yaw_source` to a duplicate IMU; A/B the board's attitude against
   `mongla_sensors` in the water before deleting anything.
5. Decide the companion compid. Then we ship the source-specific failsafe.

## Related

- [`srot-integration.md`](srot-integration.md) — the verb table and the (now largely historical)
  workaround list
- [`vision-control-split.md`](vision-control-split.md) — the split (its FOV blocker is cleared)
- [`camera-and-calibration.md`](camera-and-calibration.md) — the calibration tool, the library, the in-water mode
- `srot-control-board/`: `AUDIT.md` (R35–R44), `FIRMWARE_CHANGELOG_FOR_DUBURI.md`,
  `VISION_API.md`, `JETSON_COMMS.md`, `PARAMETERS.md`

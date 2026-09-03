# PR E — put the visual servo loop on the side of the cable that can run it

**Repo:** `srot-control-board` · **Depends on:** PR A (`LANDING_TARGET` ingest)
**Status here:** the companion-side half is BUILT, MEASURED and RUNNING, and it
is the thing this proposes to replace. That is deliberate — you get a contract
that has been flown rather than only written.

---

## The measurement that motivates it

We just spent a round on the Pi + AI HAT+ vision stack and found **three
separate bugs of one shape**, none of which raised anything:

| constant | tuned at | what it became | consequence |
|---|---|---|---|
| continuity-lock gate, 0.30 "per tick" | 20 Hz | **2.5x looser** at 50 Hz | a second torpedo hole can steal the aim |
| `align_stable_frames = 3` distinct frames | 20 Hz = 0.10 s | **0.02 s** at 98 Hz | the gate that ARMS THE FIRE measured 20 ms of evidence |
| Kalman coast `max_predict_frames = 30` | 20 Hz = 1.5 s | **0.31 s** at 98 Hz | the coast truncated *below* the control loop's own coast window |

Every one is the same failure: **a constant expressed in TICKS or FRAMES stops
being the physical quantity it was tuned as, the moment the rate underneath it
moves.** All three are fixed on our side (`duburi_ws` @ `srot`), and all three
were reachable only because the servo loop runs on a companion whose rate is
whatever the perception stack happens to do that day — 3-4 Hz on a `.pt`,
20-30 Hz on the Jetson's TensorRT, **55-98 Hz** on the Hailo path, and
**15 Hz on the other camera of the same vehicle**.

**On a 500 Hz task with a fixed tick, those constants are constants.** That is
the argument for the offload, and it is an argument about correctness rather
than about speed.

---

## What we propose moves to the board, in order of value

### 1. The terminal hold (`align(hold=)`) — station-keep on a bearing

Today the companion holds the hull on a pixel target at 50 Hz for the seconds
before a torpedo leaves the tube. During that hold the TARGET is static; what
moves is the hull, pushed by water and its own thrusters.

**A camera is the wrong sensor for that and you already have the right one.**
`config.h:355` polls the BNO's 400 Hz fused output on a 500 Hz task. The
disturbance the hold is fighting is visible to your IMU an order of magnitude
sooner than to a 55 Hz camera, and you are the only side that can act on it in
the same cycle.

Shape: `SROT_VISION` (31001) carries a *bearing to hold*; the board holds it
with the IMU in the loop and refreshes the bearing whenever a new
`LANDING_TARGET` lands. Same keep-alive contract as `DO_MOTOR_TEST` — companion
silence expires the hold — so a stalled Pi cannot pin the hull on a target.

### 2. Propagate the bearing BETWEEN frames

At 55 Hz vision and 500 Hz control you have **nine control cycles per camera
frame**, and today eight of them act on a bearing measured up to 18 ms ago.

Our companion-side answer is a `_freshness` ramp that *decays authority* as the
sample ages — which is the right thing to do when you have no idea what
happened since the frame. **You do.** The hull's own attitude change since the
frame's timestamp says exactly how far the bearing moved: propagate it and the
sample is not stale, it is *corrected*. That converts a decay into a
prediction, and it is the single thing a companion structurally cannot do.

This needs the frame TIMESTAMP on the wire, which `LANDING_TARGET` already
carries (`time_usec`) and PR A's `coasted` + gap-age fields complete.

### 3. The fire gate

`align(fire=, fire_t=)` decides on the companion whether the hull is aligned
enough to fire, from pixel evidence alone — and the table above is what that
evidence was actually worth at high FPS.

**Pixels cannot tell you the hull is still.** A bbox is stationary both when the
vehicle is settled and when it is rotating about the camera's optical axis, and
we have no way to separate those. Your gyro does, in one read.

Ask: gate the shot on *bearing within X* **AND** *|gyro| below Y for T ms*,
with the companion supplying X/Y/T. It is a small addition to machinery you
already have, and it removes the one decision in the whole mission that we make
with the wrong sensor.

---

## What must NOT move, so the boundary is stated rather than negotiated later

- **Detection, class selection, and mission sequencing.** These need a
  26-TOPS accelerator and a filesystem full of models.
- **Pixels -> bearings.** It needs the camera intrinsics (PR A §2: the pinhole
  form, because the linear model is **1.264° wrong at the aim point** on this
  optic) and those belong beside the camera and its calibration file.
- **Anything that must be right when the board is not talking.** The companion
  keeps the supervisory abort.

---

## What we are handing over with it

- The FOV you were blocked on (PR A), measured and held-out validated.
- A companion-side reference implementation of every loop above, running on the
  bench today, with the three rate bugs found and fixed in it — so you inherit
  the corrections rather than rediscovering them at a different tick rate.
- The measured perception rates the contract has to survive: **15-98 Hz, both
  on the same vehicle at the same time.** `VISION_API.md` was written when we
  thought 20. A staleness contract sized for 20 Hz is wrong at both ends of
  that range.

## The honest counter-argument

This is phase-2 work and none of it is on the critical path for a first pool
run: the companion-side loop works and is measured. The reason to do it is that
**three timing bugs of one shape in one round is a pattern, not three
accidents**, and the pattern is a consequence of where the loop lives.

# Payload in simulation — `fire()` without a boat

Until now the payload was the one part of the autonomy stack with **no
simulated path at all**. `PayloadDriver` talks to an ESP32 over a CH340 serial
port, so `duburi.fire()` — and with it `align(fire=…, fire_t=…)`, the mid-hold
shot with its `is_new_frame` gating and `fire_pass` fallback — had never once
executed outside the pool.

`duburi_sim_bridge/payload_sim.py` closes that. It starts with the rest of the
sim (`payload:=false` to disable it).

## Using it

Nothing to configure. `duburi_sim stack` already defaults `payload_port` to the
virtual board, so the manager finds it the way it finds the CH340 on the
vehicle:

```bash
ros2 run duburi_sim_bringup duburi_sim sim
ros2 run duburi_sim_bringup duburi_sim stack --no-vision
#   [PAYLOAD] verified + connected on /tmp/duburi-$USER/payload
```

Then everything works exactly as on the vehicle:

```bash
ros2 run duburi_planner duburi arm
ros2 run duburi_planner duburi fire --fire_channel 3
#   fire -> OK  msg="fire: ch=3 (dropper_1) FIRED"
ros2 topic echo /duburi/sim/payload/fired           # every shot, as an Int32
```

The disarmed interlock still applies -- `fire` before `arm` fails with
"AUV is disarmed", in sim exactly as in the pool.

`payload_port:=auto` restores USB VID/PID scanning if you ever want it.

`duburi.payload_ready()` returns True, and a mission's
`align(target, fwd=…, hold=…, fire=1, fire_t=…)` fires for real.

## Why a PTY, and not a sim-only fire path

**The real driver connects to this, unmodified.** `PayloadDriver.connect(port=…)`
already takes an explicit device path, so a PTY presents a device node that the
existing `pyserial` code opens exactly as it opens the CH340: same `Serial()`
setup, same DTR/RTS suppression, same `VERIFY_BYTE` probe, same single-digit
write, same reconnect-and-retry path.

Nothing in `duburi_control` changes, and that is the entire point — a sim-only
fire path would test sim code rather than the code that flies.

## What a shot actually does

| Channel | Effect |
|---|---|
| 1, 2 | torpedo launched forward from the vehicle's nose |
| 3, 4 | dropper released underneath; it sinks and stays |

The projectile is an ordinary rigid body: buoyant, dragged, and it **collides**.
"Did the torpedo go through the opening" is answered by where it ends up, not
by a log line saying a byte was written.

Verify against a running sim:

```bash
python3 src/duburi_sim_worlds/scripts/payload_check.py
# PASS: real driver -> PTY -> ROS topic -> a body that reaches the board at depth
```

## Four constraints, all found by measurement

**A spawned model cannot be given an initial velocity.** `EntityFactory`
carries a pose and nothing else. The launch is therefore a real force through
`ApplyLinkWrench` (world plugin, in the world template) for a real burn time —
7.6 N for 0.12 s on 228 g, a 4.0 m/s muzzle — after which the round coasts
under physics. Measured muzzle speed 3.1 m/s over the first sample.

**Buoyancy is a whitelist read once at world load.** A model not named in it
gets no buoyancy at all: measured, a round spawned at 1.0 m was on the 2.1 m
floor within half a second. Shots therefore reuse a fixed pool of names,
`payload_shot_0..11`, that `gen_world.py` bakes into every world.
`SHOT_SLOTS` in the node and `PAYLOAD_SHOT_SLOTS` in the generator must agree,
and a test asserts they do.

**The round must clear the hull's collision box.** At a 0.30 m muzzle the
180 mm round spanned 0.21–0.39 m from the vehicle centre while the hull box
reaches 0.229 m, so every shot began with a contact impulse — it dived 0.81 m
in 1.25 s instead of the 0.07 m its buoyancy predicts, and it looked exactly
like a buoyancy failure. The muzzle is now 0.40 m.

**Drag coefficients go on BODY axes, and the round is pitched 90°.** An SDF
cylinder's length runs along its own z, so a torpedo lying along the flight
path is spawned with pitch = π/2 — which puts body +z along the flight and
body +x pointing *down*. Written the obvious way round, the streamlined
coefficient resisted the sink and the broadside coefficient resisted the
flight: the shot travelled 0.14 m in 2 s where it had covered 2.47 m.

## Ballistics — the numbers a shot is timed against

Everything here is derived for the projectile, not copied, because where a
round goes is what a vision pipeline has to time a shot against.

| | torpedo | dropper |
|---|---|---|
| size | 51 mm x 152 mm (rulebook p. 64 max box) | 50 mm ball |
| mass | 325 g, displaces 311 g -> **14 g negative** | 118 g, displaces 65 g |
| muzzle | ~4.5 m/s (12.2 N for 0.12 s) | released, no impulse |
| drag | Cd 0.12 nose-on, 1.0 broadside | Cd 0.47 |
| added mass | 0.1x displacement axial, 1.0x broadside | 0.5x, isotropic |

**Added mass was the stated gap in the first version and is now modelled.** A
body accelerating underwater drags water with it, so it behaves as though
heavier -- and for a cylinder moving broadside that is about its own
displacement, against a tenth of it nose-on. Without it a round decelerates and
turns like something thrown in air, which is exactly the behaviour a pipeline
would learn to time against and then not find in the pool.

Measured, fired level from 0.8 m out: the round covers **2.63 m** and crosses
the board plane still travelling, with the lateral position held to the
millimetre. Drop across a 1.5 m shot is about 17 mm, which is what 14 g of
negative buoyancy gives once added mass is counted.

### Two bugs that made the launcher look feeble

**The burn was measured in WALL time.** A persistent wrench applied for
`0.12 s` of wall clock delivers `0.12 * RTF` seconds of thrust: at the RTF
these courses run, the round left the tube at **1.0 m/s instead of 4.5** and
every shot fell short. It also meant the same code fired differently on a fast
machine and a slow one. The burn now runs on the **sim clock**.

**gz-transport drops a publish before discovery completes.** The launch wrench
was published immediately after the spawn, so the first shots after startup got
**no impulse at all** -- the round simply sank away from the muzzle at 0.2 m/s.
Nothing logs a dropped publish; the shot just looks weak. The node now waits on
`has_connections()`.

Both were invisible in any log and both needed the trajectory sampled off the
pose stream to see -- `gz model -m <name> -p` takes ~2 s per call, which cannot
resolve a half-second flight. A trace that slow also reads a RECYCLED slot as
though it were the new shot, which is how an earlier "it never moved" reading
was produced.

## Fidelity limits — read before trusting a sim shot

The **serial and ROS path is exact** (it is the flight driver). The
**ballistics are approximate**:

- Mass is set from displacement (228 g against 226.2 g displaced, 1.8 g
  negative). Drag is `0.5·ρ·Cd·A` for this cylinder, Cd 1.0 broadside and 0.1
  axial. There is **no added-mass model**, so acceleration off the muzzle is
  optimistic.
- Measured flight: 2.47 m in 2 s with 0.153 m of drop. Over a 1.5 m standoff
  the round drops about 0.09 m. A real slingshot round's numbers are not known
  to us, so treat the drop as *representative, not calibrated* — unlike the
  T200 curve, none of this comes from published measurements.
- **Pass-through IS scorable now.** SDF has no primitive with a hole, so the
  board's collision is tiled in horizontal strips with the circles cut out
  (`_plate_with_holes`) -- 120 boxes for two openings, all primitives. The
  printed face carries the artwork and does not collide. That is what makes
  the rulebook's own distinction measurable.
- The **flight is modelled, not calibrated**. Drag and added-mass coefficients
  are computed from the geometry with textbook Cd values; no published
  ballistics exist for a team-built launcher to fit against. Treat the drop and
  range as representative. Unlike the T200 curve, none of this comes from
  measurements of the real article.


## Scoring a shot

`/duburi/sim/score` grades every fired round, to the handbook (p. 36):

> "Points are awarded for firing torpedoes through any opening. A torpedo must
> pass through the opening for full points. Partial points are awarded if the
> torpedo touches the board without passing through. ... Additional points are
> awarded for firing torpedoes further away from the board. The 'far' distance
> is denoted by the horizontal bars at the bottom of the board."

```
[SCORE] torpedo fired at 1.71 m from the target
[SCORE] TORPEDO THROUGH -- fired from 1.71 m (farther), opening large
```

Each round carries `outcome` (`through` / `miss` / `past_board`, or
`in_bin` / `outside_bin` for a dropper), which `opening` it took, the range
**at the moment of firing**, and the `distance_band` against the two standoffs.

The range is recorded when the trigger is pulled, not at impact: the rule names
the firing position, and the vehicle drifts several centimetres while a round
is in flight -- which is the whole difference on a 1.0 m boundary.

Verified three ways in one run, because "it scored something" is not evidence:

| aim | outcome | band |
|---|---|---|
| large opening, 1.2 m | **through**, opening `large` | far |
| solid board, 1.2 m | **miss** (stopped at the plate) | far |
| large opening, 1.7 m | **through**, opening `large` | **farther** |

The board also carries the two red bars the rulebook describes, so the "far"
line is visible to the operator and to a camera.

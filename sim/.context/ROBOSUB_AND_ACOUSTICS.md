# RoboSub 2026 in this sim, and the acoustics both competitions need

The sim covers **two competitions** now. This is what was added, what is
faithful, and what is our invention.

---

## Two competitions, two pools

`spec/arena.yaml` became `spec/sauvc.yaml` + `spec/robosub.yaml`. A course picks
one with `competition: robosub`; everything else follows.

| | SAUVC | RoboSub 2026 |
|---|---|---|
| Pool | 25 × 16 m, 1.6 m deep | 20 × 12 m, **2.1 m deep** |
| Depth source | rulebook silent; inferred | handbook p. 72, quoted |
| Models | `sauvc_*` | `robosub_*` |
| Textures | `sauvc_textures` | `robosub_textures` |

**Why this needed a refactor rather than a config line.** `resolve_pool` accepted
exactly one preset — the literal string `"sauvc"` — and the depth-spanning props
(the qualification gate, the orange flare) read pool depth at *generation* time
and bake it into `models/`. A pool at a different depth was structurally
impossible, not merely unconfigured. Textures are per-competition for the same
reason: they are sized from the pool, so one shared set stretched over a
differently-shaped pool is wrong with **no error anywhere**.

`prop_competition()` derives a prop's competition from its **name prefix** rather
than a tag on the registry entry. The convention already existed, and deriving
means a prop cannot be registered with a tag that is missing or contradicts its
own name — the failure that once let `target_mat` sit in one table and not the
other.

Adding a third competition is a third spec file. `competitions()` discovers them.

---

## The props

All dimensions are quoted with page numbers in `spec/robosub.yaml`.

| Prop | Handbook | Notes |
|---|---|---|
| `robosub_gate` | 3 × 1.5 m, 610 mm divider (p. 45) | **Asymmetric on purpose** |
| `robosub_slalom` | 0.9 m 1-in PVC, WHITE/RED/WHITE (p. 47) | One SET; place three |
| `robosub_bin_*` | 25 L crate + 305 mm role image (p. 49) | Downward-camera task |
| `robosub_torpedo_*` | 0.6 m board, two openings (p. 51) | Standoffs 0.30 / 0.46 m |
| `robosub_octagon` | 2.7 m diameter, floats (p. 52) | **Surface-anchored** |
| `robosub_resupply_table` | 0.6 m, ½ in PVC (p. 53) | Holds the collectibles |
| `robosub_path_marker` | 1.2 × 0.15 m, ORANGE (p. 54) | gate→slalom, slalom→bins |
| `robosub_pinger` | Benthos ALP-365 (pp. 100–101) | Body only — see below |
| `robosub_item_*` | jars ⌀70 × 50, boxes 84 × 84 × 30 | bolt/plug, pill/bandage |

**Two things are real geometry rather than paint, because scoring depends on it:**

- The **gate's asymmetry** (red-over-black one side, black-over-red the other).
  The vehicle's choice of side *is its role for the whole run*, and every later
  task scores higher for the matching prop. A gate mission must classify which
  half it is looking at, not just find a rectangle.
- The **torpedo board's two openings** are genuine gaps, framed by four slabs
  each, because an SDF primitive cannot express a hole. Scoring distinguishes
  large-then-small, so a torpedo has to actually pass through.

**Thin plates get trivial isotropic inertia.** A physically correct tensor for a
1.2 × 0.15 × 0.006 m path marker violates the triangle inequality once
floating-point rounds it, and Gazebo then refuses **the entire world** — not just
the prop. `sauvc_target_mat` carries the same workaround and the same comment.

### Two geometry bugs that rendering caught and reading did not

1. **The gate's panels were offset along x while sized along y**, so both halves
   stacked in the middle of the gap instead of filling it. Posts, top bar and
   panels now all agree that the gate's *width* runs along y and the vehicle
   passes along x.
2. **The torpedo board came out 0.53 × 0.25 m with its openings side by side.**
   It is a 2 ft square with them stacked. Measured back at 0.60 × 0.56 m.

Neither produced an error. Render a frame of every new prop.

---

## The courses

```bash
ros2 run duburi_sim_bringup duburi_sim sim course:=robosub26_full
ros2 run duburi_sim_bringup duburi_sim sim course:=rs_task_gate      # or
#   rs_task_slalom · rs_task_bins · rs_task_torpedo · rs_task_octagon
```

`robosub26_full` runs all six tasks in handbook order with **both role variants
present**, so the choice made at the gate is live and a mission has to decide
rather than drive at a fixed target.

**Task spacing is ours, and the course files say so.** The handbook gives prop
dimensions and pool depth but no layout — four sections run simultaneously and
the arrangement changes. Distances here are chosen so each task is out of
detection range of the next, which is what makes a full run a search problem.

---

## Acoustics: `hydrophone`

```bash
ros2 run duburi_sim_bridge hydrophone --ros-args \
    -p pinger_x:=4.0 -p pinger_y:=2.0 -p pinger_z:=-1.8 \
    -p freq_khz:=45.0 -p pulse_hz:=2.0
```

Publishes `/duburi/sim/hydrophone/ping` (`Vector3Stamped`: x = bearing°,
y = elevation°, z = SNR dB) and `.../range`.

**What it is, and is not.** Gazebo has no acoustic sensor and no propagation, and
neither does this. It simulates **what a 4-element array reports** — bearing,
elevation, range, SNR, once per ping — by degrading ground truth the way a real
array degrades it. Same bargain as the DVL. Tuning a bearing-homing behaviour
against it transfers; validating a beamformer against it does not, because there
is no waveform here, only its output.

Both competitions need it: SAUVC hides an RJE ULB-362B (45 kHz) in one of four
drums with the drum randomised between attempts, and RoboSub runs **two** Benthos
ALP-365 pingers (25–40 kHz, 0.5–2 Hz) at least 2 kHz apart. `listen_khz` filters,
so a mission that asks for the Deploy pinger stops hearing the Restore one.

### The degradations are the point

| Parameter | Default | Why it exists |
|---|---|---|
| `bearing_noise_deg` + `noise_growth_per_m` | 2.0 + 0.35/m | A distant source is a fuzzy source |
| `dropout_prob` | 0.12 | A mission must tolerate silence |
| `ghost_prob` | 0.06 | **See below** |
| `blind_cone_deg` | 25 | An array is deaf directly beneath itself |
| `max_range_m` | 30 | Beyond this, nothing |

**A ghost is not noise.** Noise scatters around the truth and averages away. A
multipath ghost is a *confident wrong bearing* off a wall, arriving on time and
looking exactly as valid as a real ping — 35–120° off, at plausible SNR. It is
what breaks homing that averages bearings, which is precisely why it is modelled
separately rather than folded into the noise term. A test asserts the ghost
offset stays large; a ghost near the truth would just be noise wearing a hat.

### Verified

Flown around a pinger at a known position while yawing and translating, checked
against `/duburi/sim/ground_truth`:

```
pings heard: 183   ghosts (>30 deg): 13 (7%)
bearing error on non-ghosts: median 5.4 deg, max 22.2 deg, n=170
```

7 % measured against 6 % configured, and the bearing tracks truth. That is the
check to re-run after changing anything here — a sensor that reports a *plausible
but wrong* bearing is the failure this simulator exists to catch.

---

## SAUVC Task 4 is not an underwater-comms problem

Worth stating because it reads like one. *Communication & Localization* is
**bumping three coloured flares in a sequence**, and the sequence is given to the
**team** topside after Navigation. There is no underwater transmission to
emulate, and the flares are found by **vision**, not acoustics. All three flares
and a `task_localization` course already exist.

The genuinely novel acoustic work is the **pinger** above — SAUVC Task 2 and both
RoboSub pinger tasks. What Task 4 still needs is a **sequence-scoring node**:
assert the flares were bumped in the commanded order, which is a contact-and-
scoring problem like `gate_transit_check.py`, not an acoustics one. Not built.

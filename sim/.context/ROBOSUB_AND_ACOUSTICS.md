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

## The props, corrected (2026-08-29)

The first pass was built from the handbook's *prose* and several props came out
wrong. Corrected against the CAD figures:

| Prop | First attempt | Corrected |
|---|---|---|
| Gate | two solid red/black panels filling the frame | a **pass-through PVC frame** — 3.048 m bar, 1.524 m legs in 609.6 mm bands, red divider, two role signs |
| Bins | 4 loose crates on the floor, image lying inside | **one prop**: a raised PVC pipeline with 4 crates hanging off it, 0.335 m square (CleverMade 25 L), role panels standing on posts |
| Torpedo | H-frame, 0.53 × 0.25 m, two square gaps | a 0.6 m **printed board on legs**, four **red circular** openings, all four role images |
| Path markers | decals flat on the floor | two 457 mm segments on **PVC T-stands**, 0.3 m off the bottom |
| Slalom | white/red/white pipes | unchanged — this one was right |

### Role imagery is the real emoji

`gen_pool_texture.make_role_image` renders the **actual glyphs** RoboNation
prints — 🔥 💧 🧭 🔨 🆘 🛟 🚑 🚒 — from `NotoColorEmoji.ttf`.

**That font is a CBDT bitmap with a single 109 px strike**, so
`truetype(..., 109)` is the *only* size that loads; anything else raises
`invalid pixel size`. Render at 109 and resample. Missing font degrades to a
grey panel with a warning rather than failing the asset build.

The droplet is **re-tinted magenta**. Noto's is blue, RoboNation's is magenta,
and Search & Rescue has to be told from Survey & Repair's orange fire —
blue-vs-orange is an *easier* discrimination than the real one, so leaving it
blue would make the sim task easier than the pool task.

### Sign orientation is rulebook-defined, and it is easy to get wrong

`_role_sign` builds a plate whose **thickness runs along local x**, so the
printed faces are ±x and the sign already faces along x at `rpy 0 0 0`. Adding
`yaw 1.5708` turns it **edge-on** — visible as a thin white line, not an image.
Every gate sign was doing exactly that.

| Prop | Faces | Pose |
|---|---|---|
| Gate | the approaching AUV | `rpy 0 0 0` |
| Torpedo | the AUV (it is the aim point) | `rpy 0 0 0` |
| Bins | up-and-back, for the downward camera | pitched ~30° |
| Octagon | **inward**, into the octagon | `yaw = ang + π` |

The octagon's was also wrong: plain `ang` aims each image at the pool wall,
where nothing can read it.

## Ground-truth labels now come from Gazebo

`gt_labels.py` is gone. It projected prop AABBs itself and admitted in its own
docstring that it did "not a full Gazebo visibility check" — a crate behind
another crate still got a full box. Worse, it contained **only `sauvc_*`
entries**, so all 14 RoboSub props resolved to `None` and a RoboSub course
recorded **empty label files**, silently.

Replaced by a `boundingbox_camera` per camera, with
`prop_library.DETECTION_CLASSES` as the single registry: **the index is both
the YOLO class and the Gazebo semantic label**, so there is no second table to
disagree with the first. Three hand-maintained tables deleted.

> **A prop with no label is INVISIBLE to the sensor.** Gazebo emits a box only
> for an entity carrying a `SemanticLabel`, and says nothing about one that has
> none. A test asserts every registered prop has a non-zero label; class 0 is
> reserved for background because gz-sim reports unlabelled entities as 0.

Two things improved beyond the occlusion fix: **runtime spawns are labelled**
(the projector read the course YAML, so `props add` was invisible to it), and
the course YAML is no longer consulted at all.

**Measured cost — the reason this is on by default:** these sensors render on
the same Ogre2 thread as the cameras, the thread that collapsed to 2.83 Hz when
the DVL drew beam visuals. With box sensors on: **12.83 Hz, 5.7 ms jitter**,
against a 12.75 Hz baseline. No measurable cost.

Verified on `robosub26_full`: 447 frames, 447 labels, **446 non-empty**, 3893
boxes across 7 classes — a course that recorded nothing at all before.

## Water is on everywhere now

`water_surface: gerstner` is the **default** in `SCENE_DEFAULTS`; a course opts
out rather than in. Only `task_navigation` had asked for it, which is why every
other course looked like it had no water.

## Course layout, corrected against the official plans (2026-08-29)

The SAUVC props were in the wrong **zones**. The course plan divides the pool
into bands measured from the start wall, and props go in bands, not at points:

| Band | From start wall | x range | Holds |
|---|---|---|---|
| Starting zone | 0 – ~4 m | −12.5 … −8.5 | the 140 cm start square |
| Orange flare | ~4 – ~8 m | −8.5 … −4.5 | the orange flare, **anywhere in it** |
| RGB flares | ~8 – ~16 m | −4.5 … +3.5 | red/yellow/blue, **anywhere in it** |
| Gate line | ~16 m | +3.5 | the gate, anywhere along the line |
| Target zone | ~2 m from far wall | +10.5 | the four drums and the mat |

Two corrections: the orange flare was 1.5 m short of the **gate**, in the wrong
band entirely — it belongs 4–8 m from the *start*, a mid-transit obstacle. And
the three RGB flares were in a neat evenly-spaced row, which is the one
arrangement that makes the task trivial: find one and the other two are a fixed
offset away. They are scattered across their band now, as the plan draws them.

### The SAUVC floor SLOPES

The rulebook's side view gives **1.6 m at the pool centre rising to 1.2 m at
both ends** — a shallow V at a 3.2 % grade. This was modelled as a flat 1.6 m,
which is wrong in three ways that all show up in practice:

* A drum in the target zone sat **0.34 m above the floor** with nothing under it.
* The DVL's bottom-track altitude is constant along a transit when it should change.
* A depth hold that clears the floor mid-pool grounds at the ends.

`floor_depth_at()` and `floor_pitch_at()` give the depth and the local tilt.
Floor-anchored props now sit on the floor **at their own x** and are **pitched
to match it** — anything resting on a slope is tilted by it, and for the 6 ×
2.2 m target mat that is the difference between lying flat and having one edge
buried. Verified: both mat edges clear the floor by exactly `FLOOR_DECAL_Z`.

The floor itself is two tilted slabs meeting at x = 0, not a mesh: the collision
stays a primitive, and a mesh floor for a 25 m pool costs far more to collide
against than two boxes that describe it exactly. A pool with no
`floor_edge_depth` is flat, so RoboSub and every other course are untouched.

The lab's altitude readout follows the slope too — it carried a hardcoded
−1.6 m, which overstated altitude by up to 0.4 m near either wall, and that is
the number an operator reads to decide whether the vehicle is about to ground.
A drift test asserts the JS copy still matches the arena specs.

### Bin images go INSIDE the bins

"**Inside** the bins will be images representing each role" — handbook p. 47,
verbatim. The image lies **flat on the bin floor**, facing up at the downward
camera: you read it looking down into the bin, then drop a marker into that
same bin.

Two wrong versions preceded this. First it sat on **top** of the crate — a lid
over the opening you have to drop through. Then it stood upright on a post
beside the crate: readable, but not what the handbook says, and it puts the
image somewhere a marker never goes.

### The collectibles were positively buoyant

They floated off the resupply table and oscillated against the surface. Not a
physics-engine artefact — the model was wrong: a flat 0.15 kg against 192 cm³
(jar) and 212 cm³ (box) of displacement, so every one of them was lighter than
the water it displaced. Mass is now **computed from volume** at 1150 kg/m³, so
they rest on the table and stay where a manipulator puts them while still being
light enough to lift. Measured: zero drift over 12 s.

The table gained a **rim** as well. A bare plate lets an item slide off the edge
the moment the vehicle disturbs the water, and then the task is unrunnable.

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

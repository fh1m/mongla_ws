# PR P — the geometric allocator, written

**Target:** `srot-control-board` · **Branch:** `duburi-ws/geometric-allocator`
**Status:** ⏳ **CODE READY, NOT SENT** — this session has read-only GitHub access to
`srot-control-board` (branch creation, issue creation and PR creation all return
`403 Resource not accessible by integration`). The patch is in
[`patches/srot-geometric-allocator.patch`](patches/srot-geometric-allocator.patch)
and applies cleanly to their `main` at `f1d3ba9`.

    git -C srot-control-board checkout -b duburi-ws/geometric-allocator main
    git -C srot-control-board am .claude/context/upstream/patches/srot-geometric-allocator.patch
    cd srot-control-board && ./tools/run_native_tests.sh    # 85 checks, no board needed

**Supersedes the ASK half of [PR K](pr-k-the-mixer-is-for-a-different-hull.md) /
their #25.** PR K asked for the mixer to become a function of geometry (its §4.2)
and said we would bring the measured numbers. This is that, implemented, plus a
correction to the numbers PR K itself carried.

---

## 1. ⚠ FIRST: PR K's geometry is wrong, and it is the document they would build from

PR K's table gives the two pair separations as **346.6 mm** and **520.4 mm**, and
states the matrix in a frame where `Fy = heave`, `Fz = surge`, `My = yaw`,
`Mz = roll`.

Both need correcting before anybody implements from it.

**The separations are 350.00 mm and 519.00 mm, exactly.** PR K measured them off
`docs/assets/cad/hull.mgla`, which is vertex-clustered and int16-quantised *for a
web viewer*. `hull_geometry.yaml` retracted those figures the following day in
favour of the CAD's own values, but PR K had already been sent and still says
346.6 / 520.4. The errors are 3.4 mm and 1.4 mm — which is 1 % of the yaw moment
arm, i.e. 1 % of the scarcest authority the hull has.

Re-measured 2026-09-25 from the Onshape GLB export of Part Studio 1, using each
body's exact per-primitive accessor bounds (`tools/hull_stations.py`, new):

```
hull bbox  176.10 x 172.09 x 702.00 mm     fineness 3.986
bore along X: -175.00, +175.00   separation 350.00 mm   <- lateral tunnels
bore along Y: -259.50, +259.50   separation 519.00 mm   <- vertical tunnels
A2212 axial motor at +330.60, z-offset +8.06            <- the only surge
Part 3, a ⌀87.99 mm ring, 2 mm thick, at +351.00        <- its duct, exit side
```

The hull bounding box reproduces our documentation to 0.01 mm, which is why we
trust the rest.

⭐ **And PR K said the axial unit was "absent from the GLB export" and marked
UNKNOWN. It is not absent** — it is at +330.60 with its duct ring at +351.00, and
its **8.06 mm offset from the Z centreline is a real pitch moment**. See §3.

**The frame labels differ from our own shipped constants.** PR K measured in the
glTF export frame (long axis Z); `hull_geometry.yaml` and
`geometric_allocation.py` are in the CAD frame (long axis Y). Each is internally
consistent, and the transform between them is

    cad = (mesh_x, -mesh_z, mesh_y)

so CAD `+Y` (forward) is the export's `-Z`. Nothing in PR K says which frame it is
in. Implemented verbatim, its `Fz = surge` row would put surge on the vertical
tunnels' axis. **The patch uses the CAD frame throughout and states it at the top
of `frames.h`.**

This is our error, not theirs, and it is the exact failure mode our own CLAUDE.md
names — *one truth, two copies is the bug* — arriving across a repo boundary where
no test can reach it.

## 2. What the patch adds

```
FRAME_CLASS = 0   the existing ±1 vectored-8 matrix in mixer.cpp, bit for bit.  DEFAULT.
FRAME_CLASS = 1   mongla-5, allocated from the measured geometry above.
FRAME_CLASS = 2   vectored-8 as geometry (⛔ placeholder arms; measure before use).
ALLOC_PRIO  = 0   uniform scale-down — what mix() does today.  DEFAULT.
ALLOC_PRIO  = 1   attitude met in full, translation takes the headroom left.
```

- `src/control/frames.h` — **geometry only**: position + axis per thruster. Data
  and algebra are separated deliberately, because a number is reviewed against a
  CAD model and a line of code is reviewed against linear algebra. `mixer.cpp`'s
  matrix *was* the geometry, so there was nothing to compare it against — which
  is the whole reason the wrong-hull problem survived this long.
- `src/control/allocator.{h,cpp}` — builds `B[0:3] = axis`, `B[3:6] = r × axis` at
  boot, solves it, and does one N×6 multiply plus two linear passes per tick. **No
  matrix work on the flight loop**, no dynamic allocation, no data-dependent loop
  bounds.
- `src/control/mixer.cpp` — `mix()` delegates when `FRAME_CLASS != 0`.
  `motorAngular()` reads the geometry, so **MOTOR_DETECT stops projecting onto
  three equal ±1s** (see §5).
- `tools/run_native_tests.sh` + `test/native/test_allocator.cpp` — see §6.

⭐ **Their §3 request is honoured, with a better guarantee.** PR K asked them to
keep the per-group saturation scaling. It survives — and not because it was
re-implemented, but because the solution operator is **block-diagonal wherever the
geometry is**: the lateral pair depends only on {sway, yaw}, the vertical pair on
{heave, pitch, a 0.0155 surge trim}, the axial on {surge}. A saturating surge
cannot reach the lateral pair, asserted **bit-identical** in the test.

## 3. What a ±1 matrix costs on this hull, measured

**The six "axes" are not commensurate quantities.** With every output ≤ 1:

| axis | max wrench | limited by |
|---|---|---|
| sway | **2.000** | either lateral tunnel |
| heave | **2.000** | either vertical tunnel |
| surge | **1.000** | the axial unit (there is only one) |
| pitch | **0.519** | the vertical pair's 519 mm arm |
| yaw | **0.350** | the lateral pair's 350 mm arm |
| roll | **0** | ⛔ unactuated |

Sway and yaw differ by **5.714×**, which is *exactly* the reduced matrix's
condition number. A `±1` mixer calls both of them "1.0", so every per-axis gain
tuned against it silently absorbed that factor — and that is a large part of why
the gains could only ever be found empirically, one axis at a time. With each axis
normalised by its own authority, **a demand of 1.0 means the same thing on every
axis**, and a single-axis demand of 1.0 saturates exactly one thruster at exactly
1.0 (asserted for all five).

**It cannot express an off-axis thruster.** The axial unit's 8.06 mm offset is
**0.00806 N·m per newton of surge**. Under the ±1 mixer that moment is simply left
on the vehicle: full surge arrives as an uncorrected pitch-up disturbance for the
attitude loop to absorb — and then for the CoB auto-trim to *learn*, which is how a
real geometric term becomes a mystery trim. The allocator cancels it for **0.0155
of one thruster each** on the vertical pair, and the test asserts net pitch from a
pure surge command is zero.

**It cannot express a dead thruster, and here that matters more than usual.** The
reduced matrix is **5×5, determinant 0.1817, condition number 5.714** — exactly
determined. ⛔ **There is no redundancy at all.** Losing one thruster does not
degrade the solution, it removes an axis. `rebuild()` therefore **refuses** and
keeps the previous solution, because there is no allocation to fall back to; a
caller that reads `false` as "keep flying" is wrong, and the correct response is
to surface. The 8-thruster frame *can* re-solve around a loss, and the test shows
both side by side. **That contrast is an argument for the hardware team**, not
just a code property.

## 4. ⛔ The one thing that can sink the vehicle, stated plainly

**The vertical sign convention in `frames.h` is NOT `mixer.cpp`'s.**

The literal matrix gives every vertical motor a throttle column of `-1`, because
on that vehicle *a positive motor command pushes the hull down*
(`docs/THRUSTER_MAP.md`). Two negatives then make "positive throttle ascends",
which is the convention `depth_control` closes its loop in.

`frames.h` has no such inversion: `+Z` is up and positive output pushes along the
axis, so a positive vertical output ascends directly. **One negation, not two.**

A depth loop wired the wrong way round is positive feedback — measured deeper than
target → command descend → deeper. It does not oscillate, it dives. Before any
armed dive on `FRAME_CLASS != 0`, set `MOT_n_DIRECTION` for the vertical pair and
**confirm it with the Motor Test tab and a hand on the hull**, not by reasoning
about the comment. `depth::preview()` exists for exactly this and can be read
while disarmed. The warning is at the top of `frames.h`, in the file anyone
editing the geometry has open.

## 5. What it does NOT do, on purpose

- **It does not change the vehicle that flies today.** `FRAME_CLASS` defaults to 0
  and class 0 never enters this code: `allocator::begin(0)` returns `false` by
  design. A class whose geometry will not solve falls back to the legacy matrix
  and **says so on the wire as an ERROR** — silently flying a different allocation
  than the operator asked for is the failure this whole PR is about.
- **It does not claim newtons.** Output is a signed fraction of one thruster's
  full output, the same quantity `toDshot()` already consumes. Converting to force
  needs `k_n_per_rpm2`, which exists nowhere in either tree on purpose (our PR O
  asks for the load-cell afternoon that would produce it). The wrench rows are
  force-like and moment-like in consistent units: **exact in direction and ratio,
  silent about magnitude.**
- **It does not fix the saturation feedback their #20 is about.** `allocator::achieved()`
  gives the wrench a set of outputs actually produces — the readback the anti-windup
  needs and the wire does not carry — but nothing consumes it yet. That is #20's
  job and we are not duplicating it.
- **It does not settle the axis signs.** Which end of Y is the bow, and whether the
  axial unit is a tractor or a pusher, are decisions made against the vehicle in
  water. The duct ring at +351 sits just outboard of the motor, where a propeller
  would exhaust — suggestive, not decisive.
- **It is not compiled for the ESP32 here.** We have no `platformio` toolchain in
  this container. The allocator and its tests compile and pass under `g++ -Wall
  -Wextra -Werror`; the `mixer.cpp` / `main.cpp` / `params.cpp` edits are reviewed
  but **not compiled**. Build before flashing.

## 6. The test, and why it is the point

`test/native/test_allocator.cpp` — **85 checks, no board, no toolchain, no water.**
`allocator.cpp` includes nothing but `<math.h>` and `<string.h>` specifically so
this is possible: *a 500 Hz allocator whose only test is a water test is not
tested, it is trusted.*

Every asserted number was derived independently from the CAD stations rather than
recorded from a run of this code — a test that records what the code does cannot
tell you the code is wrong.

**Injection-verified, five ways, each confirmed to fail before this was committed:**

| injection | what failed |
|---|---|
| axial offset `0.00806 → 0` | the pitch moment and both trim outputs |
| lateral station `0.1750 → 0.1732` (PR K's retracted number) | both yaw arms, sway, yaw, and the 5.714 ratio |
| remove the unactuated-axis refusal | roll is no longer reported refused |
| `headroom()` always returns 1.0 | attitude priority stops reserving |
| minimum-norm solve → naive transpose | the redundant frame stops being exact |

Also swept: **6250 hostile demands across both priority modes, worst output
1.000000000**, and a NaN demand reaches no thruster and is reported refused.

---

*Geometry: Onshape GLB export of Part Studio 1, per-primitive accessor bounds,
2026-09-25, via `tools/hull_stations.py`. Firmware read at `mixer.cpp:26-42`,
`config.h:117`, `task_control_loop.cpp:893`, `calibration.cpp:346`.*

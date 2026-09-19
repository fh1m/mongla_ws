# Simulator — pointer

**The simulator lives in this repo at [`sim/`](../../sim/) and its docs live with
it.** This file is a pointer, deliberately thin: folding sim documentation into
`.claude/context/` would create a third packaging story for the same tree, which
is exactly what the sim's own docs warn against.

| Want | Read |
|---|---|
| Operator cold start, terminal by terminal | [`../../README.md`](../../README.md#-simulator--gazebo--ardusub-sitl) · [`../../sim/README.md`](../../sim/README.md) |
| Doc index for the sim | [`../../sim/.context/INDEX.md`](../../sim/.context/INDEX.md) |
| The surface autonomy relies on | [`../../sim/.context/CONTRACT.md`](../../sim/.context/CONTRACT.md) |
| Courses, props, custom models | [`../../sim/.context/WORLD_EDITING.md`](../../sim/.context/WORLD_EDITING.md) |
| Dataset recording | [`../../sim/.context/DATASETS.md`](../../sim/.context/DATASETS.md) |
| Known sim issues | [`../../sim/.context/AUDIT.md`](../../sim/.context/AUDIT.md) |
| Lab HTTP API | [`../../sim/.context/LAB_API.md`](../../sim/.context/LAB_API.md) |

`duburi-sim.md` in this directory is the **legacy** bring-up against the sibling
`~/Ros_workspaces/colcon_ws` tree. Superseded — kept for history only.

## The three things that bite

1. **Build and source autonomy first.** `sim/stack.launch.py` includes
   `duburi_manager`'s and `duburi_vision`'s launch files by share directory. Use
   `./build_sim.sh`, never a bare `colcon build` — `sim/COLCON_IGNORE` makes the
   sim ignore *itself* when it is also the base path.

2. **`flight_controller:=pixhawk` is required on `srot` and inert on `main`.**
   Inert, not an error: `IncludeLaunchDescription.execute()` raises only for
   *missing required* arguments, so extra keys become launch configurations nobody
   reads, with no log line anywhere. A renamed or branch-only launch argument is a
   silent no-op in both directions — which is why
   [`test_sim_contract_drift.py`](../../src/duburi_manager/test/test_sim_contract_drift.py)
   asserts on it rather than trusting the launch to complain.

3. **`mavlink_check` binds UDP 14550 itself.** Run it with the stack DOWN, or it
   competes with the manager for the autonomy link and the resulting mess looks
   like a simulator fault rather than a tooling one.

## What does not transfer from sim to pool

Control behaviour and every `/duburi/move` verb transfer. **Detection thresholds
and vision gains do not** — Gazebo imagery has no backscatter, no turbidity and
perfect optics, so a confidence or gain value tuned in sim is a starting point,
never pool-verified. Heading also differs: sim runs `mavlink_ahrs`, while the real
hull runs `bno085`/`dvl` because its compass is untrusted inside aluminium.

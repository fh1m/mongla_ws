# mongla_runtime — how a Mongla process runs

**[`src/mongla_runtime`](../../../../src/mongla_runtime)**

Every node in the stack needs the same handful of things before it does any work of its
own: an executor that stops cleanly on Ctrl-C, a way to die when the launch that started it
dies, its parameters from `~/.mongla/bringup.yaml` with a misspelt key refused, a restart
when it crashes, and on the manager a realtime priority. This package is those things and
nothing else. It imports nothing from Mongla, so every node package depends on it and none
of them depends on another just for plumbing.

It was split out of `mongla_localization` on 2026-11-20. The helpers had been put there
because it happened to be the one package everything already depended on, which made
vision, the manager and Batighor depend on the estimator to start a node.

## The map

| File | What a node gets from it | Who uses it |
|---|---|---|
| `spin.py` | the executor (`MONGLA_EXECUTOR`) and a Ctrl-C that ends without a traceback | every node |
| `orphan.py` | `die_with_parent`: a node holding a device dies with its launch | vision and localization nodes (the manager guards this itself, because it must disarm first) |
| `params_file.py` | the `nodes:` section of `bringup.yaml`; refuses an unknown or doubly set key | every node |
| `launch_defaults.py` | launch argument defaults from `bringup.yaml`; typed `name:=value` still wins | every launch file |
| `launch_respawn.py` | restart a dead process three times, then say so (measured-bars §69) | bringup, vision and Batighor launches |
| `realtime.py` | SCHED_FIFO 10 and a 0.5 ms switch interval (`MONGLA_REALTIME`, measured-bars §76) | the manager |

## Module map

<!-- modules:begin -->
Generated from each module's docstring by `tools/ci/gen_reference.py`; a module
with no summary line shows as *(no summary)* until it gets one.

| module | what it is |
|---|---|
| `launch_defaults.py` | Bringup settings from a file, so a pool day is a file and not a long command. |
| `launch_respawn.py` | Restart a vehicle process that dies mid-run, three times, then leave it down. |
| `orphan.py` | Every long-running node dies with its parent. |
| `params_file.py` | The node half of `~/.mongla/bringup.yaml`: a typo in `nodes:` stops the node. |
| `realtime.py` | Give a process that streams commands a realtime priority and a short lock. |
| `spin.py` | One place that decides which rclpy executor a Mongla node spins on. |
<!-- modules:end -->

## Tests

The tests that belong to these helpers moved with them: the executor choice, the clean
Ctrl-C, the restart policy and the realtime request. The checks that every node and every
launch file actually USES them stay where the nodes are (`test/` at the top and each
package's own tests).

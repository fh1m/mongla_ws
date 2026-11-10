# The Mongla book

Every chapter of Mongla's documentation, in the order a newcomer should read it. It starts
with why the vehicle is built the way it is, then goes through the machine, how it sees, how it
decides and how to run it, and ends with the record of what we measured and got wrong. New
here? Read [the front page](wiki/Home.md) first; it covers the whole thing in pictures.

Two rules hold throughout. The code beats the docs, and a measurement beats both, so every
threshold the code ships with is in [the ledger](measured-bars.md), with how it was measured.
Each chapter's name here is its own first heading, so this page cannot describe a chapter as
something it is not.

**Read and edit it in [the wiki](https://github.com/fh1m/mongla_ws/wiki).** An edit made there
is committed back to the file here, where the doc tests run ([how](reference/how-the-docs-stay-true.md)).
A new page written in the wiki lands in `wiki/`. Add it to this contents page, or the tests
fail on a page nobody can reach.

The earlier stack (Pixhawk / ArduSub, the Jetson, the simulator) is not documented here. It
lives, whole, at the tag `last-with-pixhawk-and-sim`.

## Part I — Why this exists

1. [The Shift](../../docs/the-shift.md)
2. [Capability Map](../../docs/capability-map.md)
3. [What owning the whole chain lets us do that nobody else can](the-whole-chain.md)
4. [ROADMAP — the one place for where Mongla is headed and what is left](ROADMAP.md)

## Part II — The machine

5. [Vehicle spec — what is actually fitted](platform/vehicle-spec.md)
6. [The srot board / Hengla firmware — architecture, why it is built this way, and where we disagree](platform/srot-architecture.md)
7. [The SROT board, read end to end — and what it means for `mongla_ws`](platform/srot-board-soul.md)
8. [SROT board integration — how the host drives Hengla](platform/srot-integration.md)
9. [Firmware revisions — what each Hengla behaviour revision changed for this host](platform/firmware-revisions.md)
10. [Cross-repo contract — mongla_ws ↔ SROT ↔ Bondor](platform/cross-repo-contract.md)
11. [The Pi↔SROT vision/control split](platform/vision-control-split.md)
12. [System harmony — the four rules, the timing budget, and the settled negatives](platform/system-harmony.md)
13. [Kutum — the compute module](kutum/README.md)
14. [The Pi + AI HAT+ vision box — the measurement that chose the companion](kutum/pi-hailo-vision-box.md)

## Part III — Seeing

15. [Vision Architecture (`mongla_vision`)](perception/vision-architecture.md)
16. [The Hailo-8 pipeline, measured — the CUDA-equivalent reference we never wrote](perception/hailo-vision.md)
17. [The Hailo-8 frontier — what is left in the chip, and what we owe the people who use it next](perception/hailo-frontier.md)
18. [Underwater computer vision, from 55 GB of our own data](perception/underwater-vision.md)
19. [Cameras and calibration — what the field knows, and what we measured](perception/camera-and-calibration.md)
20. [Monochrome vs colour — the camera decision, settled on pixels](perception/camera-colour-decision.md)
21. [Camera latency: the queue, and why the standard fix does not work](perception/camera-latency.md)
22. [Never losing the lock: what was measured, and what it changed](perception/detection-continuity.md)
23. [Never losing the target — the research arsenal](perception/never-lose-the-target.md)
24. [The ladder register — everything built, and whether it actually runs](perception/ladder-register.md)
25. [Downward camera — the axis flip, explained once and for all](perception/downward-camera.md)
26. [Depth Estimation (`mongla_vision/depth`)](perception/depth-estimation.md)
27. [tau = scale ÷ expansion rate — the plan](perception/tau-plan.md)
28. [Two cameras on one vehicle](perception/dual-camera-setup.md)
29. [The vision→control pipeline: what is verified, and what could still bite](perception/pipeline-hardening.md)
30. [Video-File Testing Guide](perception/video-testing.md)
31. [The vision subsystem — what to add, what to fix, what to stop](perception/vision-what-to-do-next.md)

## Part IV — Deciding

32. [Command reference — every verb on `/mongla/move`](missions/command-reference.md)
33. [Mission DSL and client API](missions/client-and-dsl-api.md)
34. [Vision verb results, mid-hold fire & live feedback](missions/vision-results.md)
35. [Precision terminal alignment — hold steady & don't miss the hole](missions/precision-alignment.md)
36. [`mongla.detected()` paradigm — complete reference](missions/detected-paradigm.md)
37. [Mongla Mission Cookbook](missions/mission-cookbook.md)
38. [The verbs the board could give us, and has not yet](missions/verbs-srot-unlocks.md)

## Part V — Running it

39. [The operator runbook — every command, verified](platform/launch-combinations.md)
40. [First pool day — what to verify, in order](platform/first-pool-verification.md)
41. [Pool day](platform/pool-day.md)
42. [The first swim — control only, as a script](platform/first-swim.md)
43. [Round 4a — the identification pool day, as a script](platform/pool-day-round-4a.md)
44. [Environment traps — dependency pitfalls that take down the vision launch](kutum/pi-and-env-traps.md)
45. [Ground-station remote access — drive the Pi without a remote desktop](kutum/remote-access.md)
46. [Operator tooling — Foxglove, rosbag record/replay, scorecards](platform/foxglove-and-bags.md)
47. [ROS 2 conventions — the surface and the coding standards](platform/ros2-conventions.md)

## Part VI — Reference

48. [Command reference — generated from the code](reference/commands.md)
49. [The packages](packages/README.md)
50. [How the docs stay true — the wiki, the files, and the guards](reference/how-the-docs-stay-true.md)

## Part VII — The record

51. [Measured bars — every number the stack ships, and what measured it](measured-bars.md)
52. [Bug register — every defect in mongla_ws, open and fixed](BUGS.md)
53. [Upstream asks — `mongla_ws` → `srot-control-board`](upstream/README.md)
54. [Read coverage of the SROT / Hengla / Bondor repos](upstream/COVERAGE.md)

## Part VIII — Against the world

55. [The verdict — Mongla against the world, in one document](sota/VERDICT.md)
56. [Against the world — how Mongla compares to the best work there is](sota/README.md)
57. [Control, against the world](sota/control.md)
58. [Vision, against the world](sota/vision.md)
59. [Localization, against the world](sota/localization.md)
60. [Planning, the DSL and the mission executive, against the world](sota/planning.md)
61. [The gap ledger — every move, ranked](sota/SOTA-GAPS.md)

## Part IX — The workbench

62. [The workbench — what is being measured, and what it said](workbench/README.md)
63. [Bench — everything provable without water](workbench/BENCH.md)
64. [Pool — one section per session, agenda written before travel](workbench/POOL.md)
65. [Research owed — the questions the sweeps could not answer](workbench/RESEARCH-OWED.md)

## Appendix — Elsewhere

66. [RoboSub Team Intelligence — Mongla Vision Roadmap](scouting/README.md)
67. [Bumblebee — RoboSub 2025 (1st Place)](scouting/bumblebee-2025.md)
68. [Future TODO -- shrink `commands.py` registry](future/future-registry-shrinkage.md)

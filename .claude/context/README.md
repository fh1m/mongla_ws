# The Mongla book

Every document in this folder, in the order a newcomer should read them. Each chapter is
titled by its own first heading, so this contents page cannot describe a document as
something it is not — `test_docs_contract.py` checks the titles, and fails on any document
here that no link from this page reaches. Code beats docs; a measurement beats both — see
[`measured-bars.md`](measured-bars.md).

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
9. [Cross-repo contract — mongla_ws ↔ SROT ↔ Bondor](platform/cross-repo-contract.md)
10. [The Pi↔SROT vision/control split](platform/vision-control-split.md)
11. [The Pi + AI HAT+ vision box — the measurement that chose the companion](platform/pi-hailo-vision-box.md)
12. [System harmony — the four rules, the timing budget, and the settled negatives](platform/system-harmony.md)

## Part III — Seeing

13. [Vision Architecture (`mongla_vision`)](perception/vision-architecture.md)
14. [The Hailo-8 pipeline, measured — the CUDA-equivalent reference we never wrote](perception/hailo-vision.md)
15. [Underwater computer vision, from 55 GB of our own data](perception/underwater-vision.md)
16. [Cameras and calibration — what the field knows, and what we measured](perception/camera-and-calibration.md)
17. [Monochrome vs colour — the camera decision, settled on pixels](perception/camera-colour-decision.md)
18. [Camera latency: the queue, and why the standard fix does not work](perception/camera-latency.md)
19. [Never losing the lock: what was measured, and what it changed](perception/detection-continuity.md)
20. [Never losing the target — the research arsenal](perception/never-lose-the-target.md)
21. [The ladder register — everything built, and whether it actually runs](perception/ladder-register.md)
22. [Downward camera — the axis flip, explained once and for all](perception/downward-camera.md)
23. [Depth Estimation (`mongla_vision/depth`)](perception/depth-estimation.md)
24. [tau = scale ÷ expansion rate — the plan](perception/tau-plan.md)
25. [Two cameras on one vehicle](perception/dual-camera-setup.md)
26. [The vision→control pipeline: what is verified, and what could still bite](perception/pipeline-hardening.md)
27. [Video-File Testing Guide](perception/video-testing.md)
28. [The vision subsystem — what to add, what to fix, what to stop](perception/vision-what-to-do-next.md)

## Part IV — Deciding

29. [Command reference — every verb on `/mongla/move`](missions/command-reference.md)
30. [Mission DSL and client API](missions/client-and-dsl-api.md)
31. [Vision verb results, mid-hold fire & live feedback](missions/vision-results.md)
32. [Precision terminal alignment — hold steady & don't miss the hole](missions/precision-alignment.md)
33. [`mongla.detected()` paradigm — complete reference](missions/detected-paradigm.md)
34. [Mongla Mission Cookbook](missions/mission-cookbook.md)
35. [The verbs the board could give us, and has not yet](missions/verbs-srot-unlocks.md)

## Part V — Running it

36. [The operator runbook — every command, verified](platform/launch-combinations.md)
37. [First pool day — what to verify, in order](platform/first-pool-verification.md)
38. [Pool day](platform/pool-day.md)
39. [Round 4a — the identification pool day, as a script](platform/pool-day-round-4a.md)
40. [Environment traps — dependency pitfalls that take down the vision launch](platform/pi-and-env-traps.md)
41. [Ground-station remote access — drive the Pi without a remote desktop](platform/remote-access.md)
42. [Operator tooling — Foxglove, rosbag record/replay, scorecards](platform/foxglove-and-bags.md)
43. [ROS 2 conventions — the surface and the coding standards](platform/ros2-conventions.md)

## Part VI — Reference

44. [Command reference — generated from the code](reference/commands.md)
45. [The packages](packages/README.md)

## Part VII — The record

46. [Measured bars — every number the stack ships, and what measured it](measured-bars.md)
47. [Mongla / mongla_ws — Unified Bug Register](BUGS.md)
48. [Upstream asks — `mongla_ws` → `srot-control-board`](upstream/README.md)
49. [Read coverage of the SROT / Hengla / Bondor repos](upstream/COVERAGE.md)

## Part VIII — Against the world

50. [The verdict — Mongla against the world, in one document](sota/VERDICT.md)
51. [Against the world — how Mongla compares to the best work there is](sota/README.md)
52. [Control, against the world](sota/control.md)
53. [Vision, against the world](sota/vision.md)
54. [Localization, against the world](sota/localization.md)
55. [Planning, the DSL and the mission executive, against the world](sota/planning.md)
56. [The gap ledger — every move, ranked](sota/SOTA-GAPS.md)

## Part IX — The workbench

57. [The workbench — what is being measured, and what it said](workbench/README.md)
58. [Bench — everything provable without water](workbench/BENCH.md)
59. [Pool — one section per session, agenda written before travel](workbench/POOL.md)
60. [Research owed — the questions the sweeps could not answer](workbench/RESEARCH-OWED.md)

## Appendix — Elsewhere

61. [RoboSub Team Intelligence — Mongla Vision Roadmap](scouting/README.md)
62. [Bumblebee — RoboSub 2025 (1st Place)](scouting/bumblebee-2025.md)
63. [Future TODO -- shrink `commands.py` registry](future/future-registry-shrinkage.md)

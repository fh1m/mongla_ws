# The Mongla book

Every document in this folder, in the order a newcomer should read them. Each chapter is
titled by its own first heading, so this contents page cannot describe a document as
something it is not. Code beats docs; a measurement beats both — see
[`measured-bars.md`](measured-bars.md).

## Part I — Why this exists

1. [The Shift](../../docs/the-shift.md)
2. [Capability Map](../../docs/capability-map.md)
3. [ROADMAP — the one place for where Mongla is headed and what is left](ROADMAP.md)

## Part II — The machine

4. [Vehicle spec — what is actually fitted](platform/vehicle-spec.md)
5. [The srot board / Hengla firmware — architecture, why it is built this way, and where we disagree](platform/srot-architecture.md)
6. [The SROT board, read end to end — and what it means for `mongla_ws](platform/srot-board-soul.md)
7. [SROT control-board integration (branch `srot`)](platform/srot-integration.md)
8. [Cross-repo contract — mongla_ws ↔ SROT ↔ Bondor](platform/cross-repo-contract.md)
9. [The Jetson↔SROT vision/control split](platform/vision-control-split.md)
10. [The Pi + AI HAT+ vision box — measured, built, and ready for srot over USB](platform/pi-hailo-vision-box.md)
11. [System harmony — the four rules, the timing budget, and the settled negatives](platform/system-harmony.md)

## Part III — Seeing

12. [Vision Architecture (`mongla_vision`)](perception/vision-architecture.md)
13. [The Hailo-8 pipeline, measured — the CUDA-equivalent reference we never wrote](perception/hailo-vision.md)
14. [Underwater computer vision, from 55 GB of our own data](perception/underwater-vision.md)
15. [Cameras and calibration — what the field knows, and what we measured](perception/camera-and-calibration.md)
16. [Camera latency: the queue, and why the standard fix does not work](perception/camera-latency.md)
17. [Never losing the lock: what was measured, and what it changed](perception/detection-continuity.md)
18. [Downward camera — the axis flip, explained once and for all](perception/downward-camera.md)
19. [Depth Estimation (`mongla_vision/depth`)](perception/depth-estimation.md)
20. [Two cameras on one vehicle](perception/dual-camera-setup.md)
21. [The vision→control pipeline: what is verified, and what could still bite](perception/pipeline-hardening.md)
22. [Sensors pipeline — `mongla_sensors](perception/sensors-pipeline.md)
23. [Video-File Testing Guide](perception/video-testing.md)

## Part IV — Deciding

24. [Command reference — every verb on `/mongla/move](missions/command-reference.md)
25. [Mission DSL and client API](missions/client-and-dsl-api.md)
26. [Vision verb results, mid-hold fire & live feedback](missions/vision-results.md)
27. [Precision terminal alignment — hold steady & don't miss the hole](missions/precision-alignment.md)
28. [mongla.detected()` paradigm — complete reference](missions/detected-paradigm.md)
29. [Mongla Mission Cookbook](missions/mission-cookbook.md)

## Part V — Running it

30. [Launch Combinations — the "so we never fail" master reference](platform/launch-combinations.md)
31. [Pool day](platform/pool-day.md)
32. [Environment traps — dependency pitfalls that take down the vision launch](platform/pi-and-env-traps.md)
33. [Ground-station remote access — the smooth, drop-proof workflow](platform/remote-access.md)
34. [Operator tooling — Foxglove, rosbag record/replay, scorecards](platform/foxglove-and-bags.md)
35. [ROS2 Conventions — Duburi AUV Codebase](platform/ros2-conventions.md)
36. [Simulator — pointer](platform/mongla-sim.md)

## Part VI — Reference

37. [Command reference — generated from the code](reference/commands.md)
38. [The packages](packages/README.md)

## Part VII — The record

39. [Measured bars — every number the stack ships, and what measured it](measured-bars.md)
40. [Mongla / mongla_ws — Unified Bug Register](BUGS.md)
41. [Upstream asks — `mongla_ws` → `srot-control-board](upstream/README.md)

## Part VIII — Against the world

42. [The verdict — Mongla against the world, in one document](sota/VERDICT.md)
43. [Against the world — how Mongla compares to the best work there is](sota/README.md)
44. [Control, against the world](sota/control.md)
45. [Vision, against the world](sota/vision.md)
46. [Localization, against the world](sota/localization.md)
47. [Planning, the DSL and the mission executive, against the world](sota/planning.md)
48. [The gap ledger — every move, ranked](sota/SOTA-GAPS.md)

## Part IX — The workbench

49. [The workbench — what is being measured, and what it said](workbench/README.md)
50. [Bench — everything provable without water](workbench/BENCH.md)
51. [Pool — one section per session, agenda written before travel](workbench/POOL.md)
52. [Research owed — the questions the sweeps could not answer](workbench/RESEARCH-OWED.md)

## Appendix — History and elsewhere

53. [Legacy: the Pixhawk backend and ArduSub SITL](platform/legacy-pixhawk-and-sitl.md)
54. [RoboSub Team Intelligence — Mongla Vision Roadmap](scouting/README.md)
55. [Future TODO -- shrink `commands.py` registry](future/future-registry-shrinkage.md)

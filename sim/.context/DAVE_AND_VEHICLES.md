# Project DAVE, and other AUV models — assessment

**Verdict: do not adopt DAVE wholesale. Cherry-pick two plugin packages if we
ever want ocean currents. Researched 2026-08-28, not integrated.**

## The blocking fact

| | DAVE ROS 2 needs | we run |
|---|---|---|
| Gazebo | **Harmonic (gz-sim 8)** ✅ | Harmonic |
| ROS | **Jazzy / Ubuntu 24.04 Noble** ❌ | **Humble / 22.04 Jammy** |

The Gazebo half matches exactly. The ROS half does not, and there is no hedge:
no `humble` branch, no Humble Dockerfile (only `jazzy.amd64` / `jazzy.arm64v8`),
only `dave.jazzy.repos`, and zero Humble issue traffic. The installer says
`Requirements: Ubuntu 24.04 LTS Noble` in as many words.

**The nuance:** the lock looks like *packaging*, not code. The three Gazebo
plugin packages link **stock** `gz-sim8 / gz-sensors8 / gz-rendering8 /
gz-transport13 / gz-msgs10 / gz-common5 / gz-plugin2` — plain Harmonic, which we
already have — and the ROS-coupled packages depend only on `rclcpp`,
`sensor_msgs`, `geometry_msgs`, `std_msgs`, `protobuf`, `mavros`, `rclpy`.
Humble + Harmonic is a supported non-default pairing (`ros-humble-ros-gzharmonic`).

So a source build on Humble is *plausible*. It is **unverified and
unsupported** — we would be the first, and the likely friction is C++ standard
(Jazzy defaults to gcc-13 on Noble vs gcc-11 on Jammy) and rosdep key
availability on Jammy. Treat it as a spike with an unknown outcome, not an
afternoon.

## Component by component

| component | verdict | why |
|---|---|---|
| `dave_gz_world_plugins` + `dave_gz_model_plugins` | **the one worth taking** | Ocean currents with a Gauss-Markov process, **tidal oscillation driven from real NOAA data**, depth-stratified currents. Apache-2.0, no CUDA, links stock Harmonic only. Hard to reproduce ourselves. |
| `dave_sensor_models` | useful reference | SDF + meshes for real DVLs — Nortek DVL500/1000, Teledyne Explorer/Workhorse, Sonardyne Syrinx, BlueView P900. Drop-in geometry for our own vehicle SDF. |
| `dave_gz_multibeam_sonar` | separate decision | Real ray-based multibeam sonar, **stock Harmonic, no forked Gazebo** — but **requires NVIDIA CUDA** (`sonar_calculation_cuda.cu`) plus `marine_acoustic_msgs`. Degrades gracefully: without CUDA it configures to a no-op and the rest still builds. |
| `IOES-Lab/asv_wave_sim` | **decline** | Gives an animated wave **surface** (FFT ocean spectra + Ogre shaders) — but **no caustics**, it is **GPL-3.0** (as are its CGAL and FFTW deps, so it propagates), it is tested on **Garden** not Harmonic, and **it is not part of DAVE at all** (absent from `dave.jazzy.repos`; DAVE's own `*_ocean_waves*` worlds load only stock Harmonic systems). |
| `IOES-Lab/gz-sensors-multibeam-sonar` | **dead end** | Default branch is unmodified upstream `gz-sensors`. The only other branch's last commit is literally `"template"`, Aug 2024. Abandoned scaffolding — the maintained sonar is in-tree in `dave`. |
| `IOES-Lab/ardupilot_gazebo` | **nothing to take** | `ahead_by: 0, behind_by: 10` vs `ArduPilot/ardupilot_gazebo`. A stale mirror. DAVE's own installer clones **upstream**, not this fork. We are already ahead of it. |

**Nothing in the DAVE stack provides caustics.** Not `asv_wave_sim` (surface
shader only), not `UnderwaterCamera` (a depth-attenuation post-process). If we
want caustics it is our own shader or a projected animated texture.

## Other AUV vehicle models

Our hull is BlueROV2-Heavy-derived. Alternatives, if a custom-frame model is ever
wanted:

| vehicle | source | format | actuation | licence | usable on Harmonic? |
|---|---|---|---|---|---|
| `rexrov` | DAVE `dave_robot_models` | **SDF** | 8 thrusters, box/work-class frame | Apache-2.0 | **yes — the only ready-to-run non-BlueROV Harmonic-native AUV anywhere** |
| `glider_slocum` | DAVE | SDF | 1 propeller, buoyancy-driven | Apache-2.0 | yes |
| ECA A9 | `uuvsimulator/eca_a9` | URDF/xacro | **1 thruster + 4 control fins** | Apache-2.0 | port needed |
| LAUV | `uuvsimulator/lauv_gazebo` | URDF/xacro | 1 thruster + 4 fins | Apache-2.0 | port needed |
| Desistek SAGA | `uuvsimulator/desistek_saga` | URDF/xacro | 3 thrusters | Apache-2.0 | port needed |
| caldus / caracara / smilodon | `Field-Robotics-Lab/dave` (ROS 1) | URDF/xacro | custom torpedo hulls | Apache-2.0 | Gazebo Classic only |

**The uuv_simulator family is archived** (Feb 2023, ROS 1 + Gazebo Classic) and
its URDFs bind to `uuv_underwater_object_plugin` / `uuv_thruster_plugin` /
`uuv_fin_plugin`. Geometry, meshes, inertia and hydrodynamic *coefficients*
transfer cleanly; the plugin bindings do not — they would be re-expressed as
gz-sim8's built-in `Hydrodynamics` + `Thruster` + `LiftDrag`. For a torpedo AUV
(one thruster, four fins) that is a real but bounded port.

Also worth knowing: **`clydemcqueen/orca4`** (MIT, active) is the cleanest ROS 2
+ ArduSub + Nav2 reference wiring, though BlueROV2-derived so not a frame change.
**Stonefish** (GPL-3.0) has better sonar and caustics but is not Gazebo at all —
its own renderer and scenario format, so a whole-simulator switch.

## Recommendation

Our procedural PBR route gets the visual realism that actually matters for
detection at a fraction of the integration risk. Revisit DAVE only for **ocean
currents** — the one thing it does that we cannot easily reproduce — and take it
as two vendored plugin packages, not as a workspace.

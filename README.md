<p align="center">
  <img src="docs/imgs/mongla-banner.png" alt="Mongla" width="100%"/>
</p>

<h1 align="center">Mongla</h1>

<p align="center">
  <em>Machines that have to work when nobody is watching.</em>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS_2-Humble%20%7C%20Jazzy-blue" alt="ROS 2"/>
  <img src="https://img.shields.io/badge/control_loop-500_Hz-37d6a8" alt="500 Hz"/>
  <img src="https://img.shields.io/badge/vision-Hailo--8-8b7cf6" alt="Hailo-8"/>
  <img src="https://img.shields.io/badge/tests-2757-success" alt="tests"/>
  <img src="https://img.shields.io/badge/licence-MIT-lightgrey" alt="MIT"/>
  <a href="https://fh1m.github.io/mongla_ws/"><img src="https://img.shields.io/badge/field_notes-mongla-0a9396" alt="Docs"/></a>
</p>

<p align="center">
  <a href="https://fh1m.github.io/mongla_ws/the-shift.html"><b>The Shift</b></a> ·
  <a href="https://fh1m.github.io/mongla_ws/capability-map.html"><b>Capability map</b></a> ·
  <a href="#getting-it-running">Run it</a> ·
  <a href="#how-you-ask-it-to-move">Missions</a> ·
  <a href="#the-packages">Packages</a> ·
  <a href="#how-this-project-works">Doctrine</a>
</p>

---

## What this is

Mongla is the autonomy stack for an autonomous underwater vehicle. It runs on a control board
we own down to the firmware, and a Raspberry Pi 5 with a Hailo-8 for vision. It decides what
the machine should do next, and it is built to keep being right about that when there is
nobody in the loop to catch it being wrong.

Underwater, almost everything fights you. Radio does not travel, so there is no GPS and no
telemetry unless you carry it. Water bends light, so the camera you calibrated in air is a
different camera once it is wet. It eats red first, then contrast, then certainty. The floor
throws moving caustics that look exactly like motion. A magnetometer inside an aluminium hull
is a random number generator. And every one of these failures is quiet: nothing errors, the
logs stay green, and the vehicle confidently goes somewhere else.

So this repository is not really a pile of algorithms. It is an argument about **what a
machine has to do to perceive, decide and move in a place that offers it no help** — and the
receipts for every claim it makes.

---

## Three acts

**Act I — the lab years.** Mongla began as the autonomy software for a university AUV
programme, and flew on its vehicles at **RoboSub 2023 (2nd place)** and **RoboSub 2025 (8th
place)**. Four computers, a closed inner loop, and a 20 Hz host loop steering a black box we
could not open. It won trophies and it had a ceiling.

**Act II — owning the stack.** A custom control board (firmware *Hengla*, by Rakibul Islam),
a Pi 5 with dedicated inference silicon, and one USB-C cable between them. A 500 Hz loop we
wrote, on a core nothing else can interrupt. Sensors that had been computed and thrown away —
per-motor RPM, two battery packs, the depth controller's own error — came back, because we
could finally read the source and ask for what was missing. *Read the whole story:*
**[The Shift](https://fh1m.github.io/mongla_ws/the-shift.html)**.

**Act III — open.** In September 2026 I left the university, on principle, and Mongla
continues as an independent project. Nothing here is owed to anyone's roadmap now. What
remains is the part I actually care about: making a machine that perceives honestly and moves
deliberately, and being able to prove it.

*The next act is open — the machine is still being built.*

---

## Why keep going

I build on hope. Not optimism — hope is different: it is the decision to keep working on the
better version of a thing while the current version is still broken.

Pushing a technical limit is the most concrete form of that I know. Every measurement in this
repository that came out worse than expected is a small argument that the world is knowable,
and that the next version can be better than this one. That is worth the hours it takes.

The engineering follows from it. If the point is a better future and not a better demo, then
the work has to be honest in a specific way: **you do not get to claim what you have not
measured.**

---

## How this project works

Five habits, learned the expensive way. Each one exists because its absence cost a run.

**Measure, then ship.** Every shipped threshold lives in
[`measured-bars.md`](.claude/context/measured-bars.md) with the method, the conditions and the
bar it must clear. More than half of that file is *retractions* — numbers we believed, tested
properly, and withdrew. The image-enhancement step everybody adds to underwater vision took
the gate from 30.4 % to **1.2 %** when it was finally measured; it ships disabled, and a test
keeps it disabled.

**A plausible number is not a measurement.** The recurring defect in this codebase was never a
crash. It was a reading that looked reasonable: a phantom depth, a confident zero, 958 ESC
frames that were all exactly `0` with nothing plugged in. Absence is now `NaN`, never `0.0`,
on both sides of the wire.

**Injection-verify every guard.** A test that has never failed against a real defect is
decoration. Break it deliberately, watch it fail, restore it. Several tests here exist because
an earlier version of them passed while the vehicle was broken.

**Refuse loudly.** A verb that reports success while the vehicle does nothing is the failure
that ends a competition run with nobody knowing why. Unsupported verbs are refused before
dispatch, with the reason.

**One truth, never two copies.** Where a constant has to exist in two places, one reads the
other — or a test reads the firmware's own headers and fails when they drift.

**A missing feature is a pull request, not a workaround.** We own the firmware, the ground
station and the board. When the vehicle cannot do something, the honest move is to ask the
right layer to change, with the evidence attached. Nine such asks live in
[`.claude/context/upstream/`](.claude/context/upstream/README.md).

---

## Where it stands, honestly

Nothing on this platform has been in water. The board's depth loop has never run closed; no
mission has been flown; no non-zero thruster value has crossed the wire. The
[capability map](https://fh1m.github.io/mongla_ws/capability-map.html) marks every row
`WATER`, `BENCH`, `BUILT` or `BLOCKED`, and today none of them is `WATER`.

That is not modesty. A map that quietly promotes bench results to flight results is how a team
finds out at the venue.

---

## Getting it running

```bash
./build_mongla.sh
source install/setup.bash
```

```bash
# the vehicle: control, localization, vision on the Hailo
ros2 launch mongla_manager bringup.launch.py vision:=true

# is it healthy enough to arm?
ros2 run mongla_manager bringup_check --srot

# watch everything the board says, with no ROS graph at all
ros2 run mongla_manager connect --watch

# one verb at a time
ros2 run mongla_planner mongla arm
ros2 run mongla_planner mongla set_depth --target -0.5
ros2 run mongla_planner mongla disarm
```

## How you ask it to move

A mission is a Python file with a `run(mongla)` function. The object handed to it is the whole
vehicle.

```python
from mongla_planner.client import TaskAbandoned

def run(mongla, log=None):
    mongla.mission_reset()                     # clear old state, re-zero depth at the surface
    mongla.use_budget(900, reserve_s=45)       # ration the run; the clock starts on arm

    try:
        mongla.arm()
        mongla.set_depth(-0.8)

        with mongla.task('gate', deadline_s=120):
            if mongla.vision.align('gate', yaw=0, lat=0, gain=30, duration=60):
                mongla.vision.move('gate', fwd=None)      # drive through it
    except TaskAbandoned:
        mongla.note('gate', 'ran out of time', success=False)
    finally:
        mongla.surface()
        mongla.disarm()
```

Three ideas hold that up. **Two vision verbs, not nine** — centre on it, or drive to it;
neither raises, both return where the target was, so a mission recovers instead of crashing.
**Effort is not points** — a run is a budget, and each task is asked whether it is still worth
the attempt. **Give up cleanly** — a deadline cancels the goal in flight and hands the mission
its fallback, rather than overrunning into the next task.

Reference: [commands](.claude/context/missions/command-reference.md) ·
[the mission language](.claude/context/missions/client-and-dsl-api.md) ·
[cookbook](.claude/context/missions/mission-cookbook.md)

## The packages

<p align="center">
  <img src="docs/assets/diagrams/package-map.svg" alt="The seven packages and the action between them" width="100%"/>
</p>

| Package | What it is |
|---|---|
| [`mongla_manager`](.claude/context/packages/mongla_manager/README.md) | the one node that talks to the board |
| [`mongla_control`](.claude/context/packages/mongla_control/README.md) | every verb, and the flight-controller boundary |
| [`mongla_vision`](.claude/context/packages/mongla_vision/README.md) | cameras, detection, the visual lock, optical flow, optics |
| [`mongla_localization`](.claude/context/packages/mongla_localization/README.md) | where the vehicle is, without a DVL |
| [`mongla_planner`](.claude/context/packages/mongla_planner/README.md) | the CLI, the mission language, the missions |
| [`mongla_sensors`](.claude/context/packages/mongla_sensors/README.md) | one interface for "which way is north" |
| [`mongla_interfaces`](.claude/context/packages/mongla_interfaces/README.md) | one action, one state topic — the whole surface |

```bash
python3 -m pytest -q src/mongla_control/test      # 865
python3 -m pytest -q src/mongla_vision/test       # 824
python3 -m pytest -q src/mongla_planner/test      # 405
python3 -m pytest -q src/mongla_manager/test      # 457
python3 -m pytest -q src/mongla_localization/test # 286
```

No hardware needed: the board, the cameras and the ROS graph are faked at their real
boundaries.

## The simulator

A full Gazebo pool lives in [`sim/`](sim/) — courses, props, both cameras, ground truth and an
operator lab. It runs ArduSub SITL by design: it is a physics environment, not the vehicle.
Control behaviour transfers; detection thresholds do not, because sim water is too clean.

---

## Credits

Mongla is written by **Muhammad Fahim Faisal**.

**Rakibul Islam** — firmware and hardware lead, co-author of the system — wrote the SROT board
firmware (*Hengla*), the Bondor ground station and the ESC tooling, each in
[his own repositories](https://github.com/RakibulIslam1). Mongla does not vendor them; it
talks to them across a documented protocol, and asks for changes by pull request.

Full authorship and history: [AUTHORS.md](AUTHORS.md).

Standing on [ROS 2](https://docs.ros.org/), [MAVLink](https://mavlink.io/),
[Hailo](https://hailo.ai/), [Ultralytics YOLO](https://github.com/ultralytics/ultralytics),
[supervision](https://github.com/roboflow/supervision) and
[YASMIN](https://github.com/uleroboticsgroup/yasmin).

MIT — see [LICENSE](LICENSE).

<p align="center"><sub>[ SYS: ONLINE ] · the machine is still being built</sub></p>

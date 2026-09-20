# Authors

**Mongla** is written and maintained by:

**Muhammad Fahim Faisal** — author.
Autonomy: perception, localization, control integration, the mission language, the simulator,
and this repository.
<fh1m.dev@gmail.com> · [fh1m.github.io](https://fh1m.github.io/) · [@fh1m](https://github.com/fh1m)

**Rakibul Islam** — firmware and hardware lead, co-author of the system.
The SROT control board firmware (*Hengla*), the Bondor ground station and the ESC tooling —
each in its own repository, under his own authorship:
[srot-control-board](https://github.com/RakibulIslam1/srot-control-board) ·
[srot-ground-station](https://github.com/RakibulIslam1/srot-ground-station) ·
[srot-esc-flasher](https://github.com/RakibulIslam1/srot-esc-flasher)

Those repositories are his work. This one does not vendor them: it talks to them across a
documented wire protocol, and every change we need there is a pull request.

---

## History

Mongla began as the autonomy software for an autonomous underwater vehicle programme at BRAC
University, where the author served as junior member of the AI & Machine Vision team (2024),
AI & Machine Vision sub-team lead (2025), and engineering team lead for the RoboSub 2026
campaign. The software flew on that programme's vehicles at **RoboSub 2023 (2nd place)** and
**RoboSub 2025 (8th place)**.

In September 2026 the author left the university, on principle, and Mongla continues as an
independent project. The vehicles, the team name and the university's materials remain with
the university, and are referred to here only in the past tense, as history. What lives in
this repository is the software and its measurements.

## Third-party work

- Simulator hull meshes derive from [`bluerov2_gz`](https://github.com/clydemcqueen/bluerov2_gz)
  by Clyde McQueen (MIT) — see
  `sim/src/mongla_sim_description/models/mongla_heavy/ATTRIBUTION.md`.
- Built on ROS 2, MAVLink, Hailo's runtime, Ultralytics YOLO, supervision and YASMIN, each
  under its own licence.

Mongla itself is MIT licensed — see [LICENSE](LICENSE).

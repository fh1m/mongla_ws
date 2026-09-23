# SOTA sweep — never lose the target, and keep the frames fast

**The goal, as the project lead stated it:** *"the vision never losing detection in ideal
conditions while keeping frames as fast as possible for control-system responsiveness."*

Two halves that pull against each other. Everything here is judged on both.

Status legend on every claim: **REAL-HARDWARE** (measured on a physical device / in water) ·
**BENCH** (GPU/desktop benchmark, no vehicle) · **SIM/MODEL-ONLY** · **UNVERIFIED** (seen in a
search result, source not fetched — these live in §10 and nowhere else).

Rules from [`../README.md`](../README.md): a citation is a URL that was actually fetched; every
candidate move carries its falsifier; retractions stay.

---

## 0. The numbers this dossier is measured against, reconciled

The brief that commissioned this sweep quoted **98 Hz / 10.2 ms**; `CLAUDE.md` quotes
**80.9 Hz standalone / 53.9 Hz through the graph**. Both are in
[`../../measured-bars.md`](../../measured-bars.md) and they are **not** in conflict — they are
three different things, and using the wrong one is how a move gets mis-ranked:

| figure | what it is | where |
|---|---|---|
| **10.20 ms / 98.01 Hz** | `grr_lowth`, end-to-end **batch 1**, frame → letterbox → chip → NMS decode → boxes, 2026-09-23 | `measured-bars.md` §14.3 |
| **80.9 Hz** | earlier standalone detection-path figure | `measured-bars.md` L59 |
| **53.9 Hz** | **delivered through the ROS graph** | `measured-bars.md` L59 |
| 477.85 FPS | `hailortcli benchmark --hw-only` on stock `yolov8s` | ⛔ **retracted**, §14.1–14.2 — not a rate a vehicle can have |

⭐ **The single largest number on the table is the graph, not the model.** 98 → 53.9 Hz is a
**45 % loss** that no perception method in this dossier comes close to buying back. It is
already written down as finding 1 of `measured-bars.md` §14.3. §8 below is the only section of
this sweep that addresses it, and it is the cheapest item here by a wide margin.

**Geometry this dossier uses** (derived, not guessed): in-water HFOV **46.7° ± 0.7**
(`measured-bars.md` L61, 25 views, `calibrateCameraRO`, held-out validated). Across a 640-px
frame that is **0.073°/px**, i.e. **≈13.7 px per degree**.

| target | 5 m | 8 m | 12 m |
|---|---|---|---|
| 1.5 m gate (span) | 233 px | 146 px | 98 px |
| 0.6 m drum | 94 px | 59 px | 39 px |
| 0.2 m torpedo hole | 31 px | 20 px | 13 px |

Hold these numbers. Q4 (small objects / tiling) is decided by them and by nothing else.

---

## 1. Q1 — detection continuity: what beats detector + tracker + flow + anchor?

_pending_

## 2. Q2 — SAM2 / EfficientTAM / XMem / Cutie: masks through occlusion, on this chip

_pending_

## 3. Q3 — temporal fusion, consecutive-frame detection, test-time augmentation

_pending_

## 4. Q4 — small objects at range: resolution, tiling (SAHI), crops

_pending_

## 5. Q5 — confidence calibration: when to trust a detection underwater

_pending_

## 6. Q6 — genuinely wasted effort

_pending_

## 7. Q7 — geometry and semantics beyond boxes, for localization

_pending_

## 8. The other half of the goal — recovering the 45 % the graph costs

_pending_

## 9. Candidate moves, each with its falsifier

_pending_

## 10. Claims I could NOT verify

_pending_

## 11. Rejected, with the reason

_pending_

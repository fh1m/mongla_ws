# Water lineage

*Generated from [`water.json`](water.json) by `tools/gen_lineage.py`; edit the registry, not this page.*

Every time a vehicle in this project's lineage was in water, and what that does and does not prove about the code here today. Read by tools/gen_lineage.py (which writes lineage.md) and by test/test_water_claims_have_a_scope.py. A status is earned by the evidence listed with it, never inherited: code that descends from a water-tested predecessor has its own status, in docs/capability-map.md.

**Key point:** the vehicles before this one were in water; this platform has not been. A status belongs to the platform and code it was earned on. Code here that descends from a water-tested predecessor carries its own state in [`capability-map.md`](../capability-map.md).

## The platforms

| platform | what it is |
|---|---|
| `duburi-2025` | the university programme's vehicle at RoboSub 2025: Pixhawk and ArduSub, the author's vision stack (not in this repository) |
| `duburi-2026` | the same programme's vehicle at RoboSub 2026: Pixhawk and ArduSub, running the Duburi ROS 2 stack this repository's history begins with |
| `dubomini-srot` | the old Dubomini hull (eight vectored T200s) with the SROT board and Hengla firmware, driven by the firmware team's own tools |
| `mongla-srot-kutum` (current) | the current platform: this stack on the SROT board and the Kutum Pi 5 with a Hailo-8, in the Mongla hull |

## Every time one was in water

| id | when | platform | status | what was observed | what ran | code identity | evidence | still active |
|---|---|---|---|---|---|---|---|---|
| `W-RS25` | 2025 | `duburi-2025` | **HISTORICAL_WATER** | The team scored 4 083 points and placed 8th of 58: gate pass-through, maintain control, coin flip, style, channel at the correct depth. (RoboSub 2025) | The author's vision stack, archived offline. Control and missions were other team members' code. | reported by the author; the code is not in this repository | [`docs/data/robosub.json`](../data/robosub.json): read from RoboNation's published score sheets by tools/robosub_record.py<br>[`docs/imgs/real/manifest.json`](../imgs/real/manifest.json): RoboSub-pool frames from the 2025 archive, scenes robosub-* | no; its successor is mongla_vision, which has its own status |
| `W-MIRPUR25` | 2025-06-29 | `duburi-2025` | **RECORDED_WATER_REPLAY** | Pool runs at the gate and the torpedo board, recorded by the vehicle's camera. The current vision code is measured on these frames, not in water. (Mirpur) | The 2025 vehicle's software; the current code only replays the footage. | footage folder raw_images/Mirpur/sun_june_29 in the 2025 archive | [`docs/imgs/real/manifest.json`](../imgs/real/manifest.json): scenes mirpur-gate and mirpur-torpedo, with the frame each came from<br>[`docs/imgs/real/mirpur-gate.webp`](../imgs/real/mirpur-gate.webp): one of those frames | the footage is a test input for mongla_vision |
| `W-RS26` | 2026-07 | `duburi-2026` | **HISTORICAL_WATER** | The team scored 6 234 points and placed 8th of 58. A pool run the same week sank a downward surge-only align to the floor, fixed in commit 7feb8551. (RoboSub 2026) | The Duburi ROS 2 stack in this repository's history (control, vision and planner over ArduSub), confirmed by the author. | best-known ref 7feb8551 (2026-07-15), the last commit before the competition gap; the exact commit flown is reported, verification pending | [`docs/data/robosub.json`](../data/robosub.json): official score sheets<br>commit `7feb8551`: records a pool observation of this stack<br>Duburi/2026/Cockpit (Jul 13, 2026) (off-repository): BlueOS Cockpit recordings, GMT-7; off-repository, on the author's machine | the ArduSub backend was removed on 2026-11-06 (tag last-with-pixhawk-and-sim); vision, planner and missions descend from it and carry their own status |
| `W-DUBOMINI26` | 2026-08-06 | `dubomini-srot` | **HISTORICAL_WATER** | The SROT board closed its depth loop in DEPTH_HOLD; it bounced at DEPTH_P 3.0 and was tuned in water to P 0.5, I 0.1, D 0.01. A DEPTH_HOLD to AUTO transition crashed the board. Every axis ran reversed until the firmware gained FRAME_REVERSE. (pool test by the firmware team, 2026-08-06 and 2026-08-07) | Hengla firmware and the firmware team's tools. None of this stack ran. | firmware commits 09d0033 and f533bd2 on srot-control-board | [`.claude/context/platform/srot-integration.md`](../../.claude/context/platform/srot-integration.md): our record, section IN-WATER 2026-08-06<br>[srot-control-board#40](https://github.com/RakibulIslam1/srot-control-board/issues/40): the firmware team's in-water source record | the board, and the depth gains it was tuned to; AUTO, which every move verb uses, has not been in water |
| `W-MONGLA` | 2026-11-10 | `mongla-srot-kutum` | **NOT_YET** | Not in water. Everything about this platform is measured on a bench, on recorded footage, or on the live board over a cable. | -- | main | [`docs/capability-map.md`](../capability-map.md): every capability with its own state<br>[`.claude/context/platform/first-pool-verification.md`](../../.claude/context/platform/first-pool-verification.md): what the first water session has to decide | yes |

## The statuses

| status | meaning |
|---|---|
| **HISTORICAL_WATER** | in water, on an earlier vehicle, firmware or stack than the current platform |
| **RECORDED_WATER_REPLAY** | real underwater footage, recorded on an earlier vehicle, replayed through the current code |
| **CURRENT_WATER** | in water on the current platform, with a recorded artifact in this repository |
| **NOT_YET** | the current platform has not been in water |

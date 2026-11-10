# The control-only swim — what the board has to get right

The first run in water uses the board, Kutum and the thrusters, and nothing
else: dive, swim straight, a clockwise half turn, swim back, then a diver turns
the kill switch and every thruster stops. No camera steers it and no odometry
times it. The host mission is `out_and_back`; every move in it is one
SROT_MOVE, so **the board's own behaviour is the whole run.**

This page ranks the open firmware items by whether they can end that run. It
adds no new asks: each line points at the issue or draft that holds the
evidence.

| rank | what goes wrong in the swim | where |
|---|---|---|
| 1 | The depth loop has never run closed, and every SROT_MOVE closes it. Two armed bench checks gate everything below | `platform/srot-integration.md` |
| 2 | **The hull.** The mixer allocates for 8 × T200 vectored; the CAD hull is 5 tunnel/axial thrusters. On the wrong mixer, "forward" is not forward | [#25](https://github.com/RakibulIslam1/srot-control-board/pull/25), PR P [#39](https://github.com/RakibulIslam1/srot-control-board/pull/39), review issue 00 [#46](https://github.com/RakibulIslam1/srot-control-board/issues/46) |
| 3 | A MOTOR_DETECT run on this hull (`FRAME_REVERSE = 1`) leaves every axis inverted, and reports success | PR Q [#34](https://github.com/RakibulIslam1/srot-control-board/issues/34), with review issue 23 posted there |
| 4 | A defaults reset silently flips `FRAME_REVERSE` and the motor directions, and arming is still allowed | review issue 02 [#48](https://github.com/RakibulIslam1/srot-control-board/issues/48) |
| 5 | A turn that never reaches its target times out and is ACKed `ACCEPTED` at 100 %: the half turn can be a quarter turn and the mission swims home on the wrong heading | PR S [#36](https://github.com/RakibulIslam1/srot-control-board/issues/36), [#8](https://github.com/RakibulIslam1/srot-control-board/issues/8) |
| 6 | The kill switch is display-only. With power cut the controllers wind up; turning the knob back resumes the move under the diver's hands. The host now disarms on the edge (B-90), at 2 Hz; the board should do it at 500 Hz | [#12](https://github.com/RakibulIslam1/srot-control-board/pull/12), review issue 11, posted on [#41](https://github.com/RakibulIslam1/srot-control-board/issues/41) |
| 7 | **A relative TURN of exactly 180 deg has no direction.** Its sign is `wrapPi` of +-pi, so `yaw_right(180)` went clockwise 0.498 of the time in a float32 simulation of `movement.cpp` (measured-bars §68); past 180 it turns the other way and 360 completes at once. The host now sends wide turns as legs of at most 120 deg. The board should take the commanded sign as the direction | review issue 14 [#59](https://github.com/RakibulIslam1/srot-control-board/issues/59) |
| 8 | Short legs reverse: the brake impulse ignores leg duration | PR T [#37](https://github.com/RakibulIslam1/srot-control-board/issues/37) |
| 9 | The safety PR: DShot output-path watchdog, IMU-lost handling, the link-age race that can latch a false "companion lost" SURFACE, refusing reboot/param-reset while armed, refusing to arm into a manoeuvre. **Applied to `main` at `f1d3ba9` and built clean for all three environments on 2026-11-07** | PR [#75](https://github.com/RakibulIslam1/srot-control-board/pull/75), opened 2026-11-10 and rebuilt clean for all three environments that day |

Host side, already done: the tether window is a host countdown (the board
refuses moves while disarmed, B-89), the countdown refuses outside tmux, the
kill-switch edge disarms (B-90), and a half turn goes as two 90s.

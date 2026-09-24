# ESC_FLASHING.md recommends Bluejay PWM at 48 kHz (fallback 24 kHz): both put thruster switching noise inside or next to the pinger bands; choose the PWM frequency with acoustics in mind

Part of the 2026-09-24 ecosystem review, round 2 (see ../README.md). Companion-side tracking: fh1m/mongla_ws#45.

## Summary
`docs/ESC_FLASHING.md:99-100` tells the operator to flash Bluejay at **48 kHz**, dropping to **24 kHz** if dead time is ≥ 30 or the thruster stutters. `docs/T200_PROFILE.md:48-50` tells them to reflash at 24 kHz. The flasher's own README (in srot-esc-flasher) flashes at 24 kHz. Thruster PWM switching noise sits at the PWM frequency and its harmonics:
- **24 kHz** is inside the thruster ego-noise band the companion already plans around (mongla ROADMAP H2: 20–25 kHz), and next to the bottom of the RoboSub pinger band (25–40 kHz). Its **2nd harmonic, 48 kHz, is 3 kHz from the SAUVC 45 kHz pinger**.
- **48 kHz** puts the fundamental 3 kHz from the SAUVC pinger.
- Bluejay also builds **96 kHz** for the BB21 (`H`) MCU class, which is above both bands.

No document links the ESC PWM setting to acoustics. No hydrophone is fitted today, so nothing fails now. But the PWM frequency is a setting this project chooses, and it should be chosen deliberately before any acoustic work, not discovered as noise later.

There are also **three different recommendations** for the same setting (48 → 24 here, 24 in T200_PROFILE, 24 in the flasher README), and `ESC_FLASHING.md` exists as two byte-identical copies, one here and one in srot-esc-flasher.

**This is not a blanket "use 96 kHz".** Higher PWM frequency costs duty resolution and, with a large dead time, a larger fraction of each period is lost to blanking. The docs' own dead-time caveat (`ESC_FLASHING.md:175-178`, `T200_PROFILE.md:48-50`) is real, and 96 kHz needs a bench check on a T200 before it is adopted.

## Evidence
At commit `f1d3ba9` (this repo), `c30b843` (srot-esc-flasher), `bird-sanctuary/bluejay@0368d11`.

`docs/ESC_FLASHING.md:99-100`:
```
**PWM choice:** **48 kHz** is the general recommendation (smoother low throttle, quieter). Drop to
**24 kHz** if the detected dead time is ≥30 or the thruster desyncs/stutters. BB2 has plenty of
```
`docs/T200_PROFILE.md:48-50`:
```
6. **Reflash Bluejay at 24 kHz.** Dead time 30 (layout `?-H-30`) wastes duty resolution at 48 kHz;
   Bluejay's guidance is *"the higher your dead time is, the lower your PWM frequency should be"*.
   BR's own ESC runs 24 kHz.
```
srot-esc-flasher `src/esp32_4way/README.md:63` @ `c30b843`:
```
3. For each ESC: select **Bluejay v0.21.0 @ 24 kHz** → **Flash**.
```
`diff` of `docs/ESC_FLASHING.md` here against srot-esc-flasher's copy at `c30b843`: identical.

Bluejay builds 24/48/96 kHz for the `H` (BB21) MCU class (`Makefile:13`, `:111-115`). The `filter-out $(subst L,96,…)` drops 96 kHz only for the `L` class:
```make
PWM_FREQS		= 24 48 96
...
			$(foreach _p, $(filter-out $(subst L,96,$(_m)), $(PWM_FREQS)), \
```
and the resolution cost (`src/Bluejay.asm:141-143`):
```asm
IF PWM_FREQ == PWM_24 or PWM_FREQ == PWM_48 or PWM_FREQ == PWM_96
    ; Number of bits in pwm high byte
    PWM_BITS_H EQU (3 - PWM_CENTERED - PWM_FREQ)
```
With dead time > 0 (`PWM_CENTERED = 1`), that gives 10-bit PWM at 24 kHz, 9-bit at 48 kHz and 8-bit at 96 kHz.

Pinger frequencies: SAUVC "RJE International Pinger Model No. ULB-362B/45 kHz" (quoted in mongla_ws `mongla_planner/missions/competition_config.py:167`). RoboSub 25–40 kHz is from the round-2 perception review's sources and was not re-checked here.

## Failure scenario
1. The team adds a hydrophone pair for the SAUVC 45 kHz drum pinger (a 50-point task), or for RoboSub ordering.
2. The ESCs were flashed at 24 kHz (flasher README) or 48 kHz (this doc). Under thrust, the hydrophone front end sees a strong line at 48 kHz (a harmonic or the fundamental), 3 kHz from the pinger, from eight thrusters mounted a few centimetres away.
3. Either the band-pass filter has to be very narrow (costing ping timing accuracy), or the vehicle has to stop its thrusters to listen, and nobody then knows the noise was a choice.

## Suggested fix
- **Decide the frequency on the bench, with acoustics as one criterion.** For one T200 on the target ESC and layout, flash 24, 48 and 96 kHz. For each, measure (a) the lowest reliable start (the deadband that mongla calls G-12), (b) desync or stutter under a step, (c) ESC temperature at 50 % for 5 min, and (d) if a hydrophone or even a wideband mic is available, the spectrum at 20–50 kHz.
- **If 96 kHz passes (a)–(c), make it the recommendation**, with the reason ("keeps PWM noise out of the 25–45 kHz pinger bands"). If it fails, choose 24 kHz and *document* that the 48 kHz harmonic will need notching at 45 kHz.
- **One copy of the doc.** Keep `ESC_FLASHING.md` in one repo (probably srot-esc-flasher, which owns the procedure) and link to it from the other. Make T200_PROFILE and the flasher README point to the one recommendation instead of restating a frequency.

## How to verify (bench)
The four measurements above, recorded in the doc next to the recommendation. The minimum acceptable result is a table of frequency × {start threshold, desync yes/no, temperature}, from which the choice follows.

## Severity: Medium (planning)
Nothing fails today, because no hydrophone is fitted. But it is a free choice that constrains every future acoustic task, and the docs currently give three different answers.

## Related
- fh1m/mongla_ws#45 (the companion-side tracking issue).
- srot-esc-flasher draft 02 in this batch (the flasher README flashes 24 kHz; the duplicate doc).
- mongla_ws `.claude/context/ROADMAP.md` H2 (hydrophone: ego-noise 20–25 kHz and the 45 kHz SAUVC pinger).

---
_Generated by [Claude Code](https://claude.ai/code)_

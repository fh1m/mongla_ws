# Monochrome vs colour — the camera decision, settled on pixels

**2026-09-23.** The question: *"these are monochrome cameras and SAUVC targets are
coloured balls — can we detect different coloured balls correctly?"*

**The answer is no, and it is not marginal.** But the fix is narrower than
replacing both cameras.

---

## 1. ⛔ The Sonix downward camera is monochrome — measured, not assumed

Captured at 1280×720 MJPG on the vehicle and inspected per-pixel:

```
mean   B = 117.57   G = 117.57   R = 117.57
|R-G| max 0         |B-G| max 0
chroma U mean 128.00 std 0.000   V mean 128.00 std 0.000
saturation mean 0.00   p99 0.0
```

**R == G == B on every pixel; both chroma planes are flat at exactly 128.** The
colour information is not washed out, it is mathematically absent — the UVC
driver is replicating one grey plane into three channels.

## 2. ⛔ And seven of eleven SAUVC classes are named by colour

`models/sauvc_sim.yaml`:

```
2: orange_flare   3: flare_red   4: flare_yellow   5: flare_blue
6: drum_red       7: drum_blue   8: drum_red_pinger
```

`flare_red` / `flare_yellow` / `flare_blue` are **the same object differing only
in colour**. So are `drum_red` / `drum_blue`.

### 2.1 ⭐ The number that settles it

Rec.601 luma — literally what a mono sensor records — with underwater
attenuation applied (red ≈1.0/m, green ≈0.25/m, blue ≈0.05/m):

| | in air | **1 m** | 3 m | 5 m |
|---|---|---|---|---|
| red | 76.2 | **28.0** | 3.8 | 0.5 |
| orange | 158.4 | 92.1 | 42.6 | 24.1 |
| yellow | 225.9 | 144.6 | 74.5 | 43.4 |
| blue | 29.1 | **27.7** | 25.0 | 22.6 |

⛔ **At 1 m depth red reads 28.0 and blue reads 27.7 — a difference of 0.3 out of
255.** That is below sensor noise. `drum_red` and `drum_blue` are the same object.

⭐ **And it is worse than "poor separation": it is NON-MONOTONIC with depth.**
Red and blue collide at 1 m, then separate at 3 m as red goes black. **Any
brightness threshold calibrated at one depth is wrong at another**, which is the
failure mode that cannot be tuned out.

---

## 3. ⭐ But mono is the RIGHT sensor for the downward camera — keep it

`flow_node.py:568` and `distance_estimation_node.py:164` both do
`cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)` **as their first act**. Optical flow
never sees colour.

And a mono sensor is *better* there:

- **no Bayer interpolation** → every pixel is a real measurement, not a
  demosaiced guess, which is exactly what sub-pixel flow wants;
- **~2–3× the light**, because there is no colour filter array — and this is an
  underwater vehicle;
- a **global shutter**, which the flow estimator needs because a rolling shutter
  skews the image while the hull moves.

⛔ **Do not "fix" the downward camera. It is correct.**

---

## 4. ⚠ FIRST, AND FREE: test the forward camera before buying anything

**The forward unit is a Fantech, and its colour status has never been tested** —
it was not connected when the measurement above was taken, and `config.py`
records nothing about colour for either camera.

**Two outcomes, and they point different ways:**

| if the Fantech is… | then |
|---|---|
| **colour** | the split is already right — mono downward, colour forward — and the only forward problem is its **30.18 Hz ceiling** (`measured-bars.md` §17) |
| **mono** | ⛔ the colour classes are undetectable today and a forward replacement is required |

**Run this first.** It is `/tmp/is_mono.py`, thirty seconds, and it decides
whether this is a purchase or a non-problem.

---

## 5. The candidate: OV9782 colour global-shutter binocular (HBVCAM-2436 V11)

MJPG 1280×800 @120 fps, global shutter, colour, USB 2.0, ~11,857 BDT.

### ⭐ What is right about it

- **Colour AND global shutter** — the combination we need for forward, and not
  common at this price.
- **120 fps at 1280×800** — four times the Fantech's 30 Hz ceiling, which is
  *separately* the largest vision bottleneck we measured.
- **Two synchronised sensors** in one purchase.

### ⚠ Three things to get right before ordering

**1. ⛔ Do not buy it for stereo. USB 2.0 will not carry it.**

| stream | bandwidth |
|---|---|
| stereo 2560×800 @120 MJPG | **295 Mbps** |
| USB 2.0 practical ceiling | **~280 Mbps** |
| single eye 1280×800 @120 | 147 Mbps — comfortable |

The listing's "2560×800 120FPS" is above what the bus delivers in practice.
Expect a lower rate or heavier JPEG compression. ⭐ **Buy it as two independent
colour global-shutter cameras**, which is the genuinely useful thing, and treat
any stereo as a bonus to be measured, not assumed.

**2. ⚠ The lens option costs target pixels.** Water divides air FOV by ~1.33:

| lens | in water | 0.2 m hole at 5 m |
|---|---|---|
| **74° — current forward** | 59.0° | **24.9 px** |
| 100° (option 7) | 83.7° | **17.5 px** (−30 %) |
| 125° | 110.6° | 13.3 px |
| 160°/180° | 153.6°/180° | 9.5 / 8.1 px |

⭐ **Prefer the narrowest option offered** for the forward mount. Wide FOV sounds
like more information and is fewer pixels on the thing you must hit.

**3. ⚠ Every calibration is per-lens.** Our measured **46.7° in water** belongs to
one specific lens; `measured-bars.md` already records a live mix-up of which
intrinsics belonged to which camera. A new lens means a new calibration before
any pose or standoff number is trusted.

---

## 6. The recommendation

1. ⭐ **Test the Fantech for colour first.** Free, thirty seconds, and it may
   make the rest unnecessary.
2. **Keep the mono Sonix on the downward mount.** It is the correct sensor for
   flow and replacing it would be a regression.
3. **If the Fantech is mono — or once its 30 Hz ceiling matters, which it
   already does — buy a colour global-shutter unit for the forward mount.** The
   OV9782 module qualifies.
4. **Order the narrowest lens option**, use the two sensors independently rather
   than as a stereo pair, and **re-calibrate** before trusting any geometry.

⚠ **One thing this note does not settle:** whether the detector, retrained on
colour imagery, actually separates the colour classes in real murky water. Red is
**0.5 luma at 5 m** even to a colour sensor's red channel — colour helps, but
attenuation is physics and no sensor recovers light that never arrived. That is
a question for the real footage, which is listed in the session memory.

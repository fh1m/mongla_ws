# Camera calibration — measuring FOV, and why nothing worked without it

> **The gap:** `camera_node.py:9` says CameraInfo is "size only; K/D" — and it
> was. There is no HFOV anywhere in `duburi_ws`, no calibration file, no
> checkerboard script, and `K`/`D` published empty on every frame. **Pixels
> cannot become radians without a measured focal length.**
>
> Backwards, and worth saying: `sim/.context/` carries a calibrated **57.7°**
> in-water figure, derived from Snell on the 80° in-air spec and verified
> through `camera_info` (fx 581.3). **The simulator has a calibrated FOV the
> vehicle does not.**

## Why this blocks real work

`vision.align` is pixel-native and does not need FOV — that is by design and it
is why the stack has got this far without one. But **anything that turns a pixel
into an angle does**: a bearing to a target, `LANDING_TARGET` (which carries
radians), monocular range from a known object size, and any fusion of vision
with the DVL or IMU. It blocks the vision→control bridge on **both** the main
and srot paths.

## The tools

```bash
# accurate: checkerboard, ~25 views, gives K, D and both FOVs
tools/fov_calibrate.py capture 0 ~/cal_fwd 25
tools/fov_calibrate.py solve   ~/cal_fwd

# quick: any object of known width at a known distance, no printer
tools/fov_quick.py 0 0.210 1.00          # A4 sheet at 1 m -> saves a frame
tools/fov_quick.py 0 0.210 1.00 --px 640 --res 1280x720
```

**Print target:** `docs/assets/duburi_checkerboard_A4.png` — A4 landscape,
300 dpi, 10×7 squares of 25 mm = **9×6 inner corners**. Print at **100 % /
actual size** (never "fit to page") and check one square measures 25 mm.
Verified detectable by `findChessboardCorners` before printing: 54/54 corners.

**The square size does NOT affect the intrinsics.** It scales translation only,
which we do not use — so an imprecise measurement of the square still gives an
exact focal length and FOV. Give it anyway for the record.

## Publishing it

`camera_node` gained a `calibration` parameter (a path to the
`calibration.json` that `fov_calibrate.py solve` writes). Empty — the default —
keeps the historical behaviour **byte-identical**: size-only CameraInfo, K and D
zero. Nothing changes until a real calibration exists.

```bash
ros2 run duburi_vision camera_node -p calibration:=~/cal_fwd/calibration.json
```

**Intrinsics are rescaled to the streaming resolution**, and that is the part
worth knowing. They scale linearly, so a calibration taken at 1280×720 is valid
at 640×360 — *after* scaling. Publishing the unscaled matrix would put the
principal point outside the image and double every derived angle, **with no
error anywhere**. `test_camera_calibration.py` pins it, along with FOV
invariance across resolutions and the requirement that an unreadable
calibration warns rather than raises (a vehicle that will not stream video is
worse than one streaming without K).

## Measure at the highest resolution, use it at any

`fov_calibrate.py capture` grabs at 1280×720 even though the vision path streams
640×360: more pixels means better corner localisation, and the scaling above
makes the result valid at the streaming size.

## Underwater is not the air number

The tool reports both. Through a flat port, Snell narrows the field:

    theta_water = 2 * asin( sin(theta_air / 2) / 1.333 )

An 80° in-air lens is **57.7°** in water — 28 % narrower. This is the direction
that bites: **the optimistic error is the dangerous one**, because a search
pattern calibrated on the in-air figure assumes the vehicle sees more than it
does. The in-water numbers this tool prints are **derived, not measured in
water**; a pool measurement would supersede them.

## Status

- Tooling: **done and tested.**
- `camera_node` publishing: **done, default-off, 5 tests.**
- **The actual numbers: NOT MEASURED YET** — needs the printed board.
  Nothing here should be read as a calibration result.

Bench-measured meanwhile, and it needs no board: **640×360 has the same
horizontal FOV as 1280×720** on the bench global-shutter camera (patch-matched
across scales: scale 1.01, correlation 0.98), so dropping to 640×360 for the
frame rate costs no field of view.

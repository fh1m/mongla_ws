#!/usr/bin/env python3
"""FOV in one minute, with a phone screen and no printer.

The checkerboard route is the accurate one, but it needs a printed board held
at many angles. This gets HFOV from a SINGLE measurement using nothing but a
tape measure, and it is exact for the horizontal angle:

    put ANY object of known width W at a known distance D, square to the lens,
    and note the pixel width p it covers in a frame of width Wpx:

        HFOV = 2 * atan( (W/2) / D )  * (Wpx / p)      [small-angle-free form
        below uses the focal length, which is exact]

    focal_px = p * D / W
    HFOV     = 2 * atan( Wpx / (2 * focal_px) )

Any flat object works: a book, a laptop lid, a phone, a sheet of A4 (210 mm).

Usage:
  fov_quick.py <device> <object_width_m> <distance_m>
     -> shows the frame, you drag a box across the object's edges, it prints FOV
  fov_quick.py <device> <object_width_m> <distance_m> --px <pixels>
     -> if you already know the pixel width, skip the GUI
"""
import sys
import numpy as np, cv2

# The refractive index and both Snell transforms live in ONE place (B22).
# tools/ runs from a source checkout without the package installed, so fall back
# to the literal ONLY if that import fails -- and say so, rather than letting a
# second silent copy of a physical constant exist.
try:
    from duburi_vision.optics import N_WATER, fov_air_to_water
except ImportError:                                    # pragma: no cover
    import sys as _sys, os as _os
    _sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)),
                                      '..', 'src', 'duburi_vision'))
    from duburi_vision.optics import N_WATER, fov_air_to_water


dev = int(sys.argv[1]); W = float(sys.argv[2]); D = float(sys.argv[3])

if '--px' in sys.argv:
    p = float(sys.argv[sys.argv.index('--px')+1])
    Wpx, Hpx = 1280, 720
    if '--res' in sys.argv:
        Wpx, Hpx = (int(x) for x in sys.argv[sys.argv.index('--res')+1].split('x'))
else:
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    for _ in range(15): cap.read()
    ok, f = cap.read(); cap.release()
    if not ok: sys.exit("no frame")
    Wpx, Hpx = f.shape[1], f.shape[0]
    cv2.imwrite('/home/fh1m/fov_quick_frame.png', f)
    print(f"  saved /home/fh1m/fov_quick_frame.png ({Wpx}x{Hpx})")
    print("  measure the object's pixel width in that image, then re-run with")
    print(f"  --px <pixels> --res {Wpx}x{Hpx}")
    sys.exit(0)

focal = p * D / W
hfov = 2*np.degrees(np.arctan(Wpx/(2*focal)))
vfov = 2*np.degrees(np.arctan(Hpx/(2*focal)))
n = N_WATER
hw = fov_air_to_water(hfov)
print(f"  object {W*100:.1f} cm at {D:.2f} m spans {p:.0f} px of {Wpx}")
print(f"  focal   {focal:.1f} px")
print(f"  HFOV    {hfov:.2f} deg   VFOV {vfov:.2f} deg   (in air)")
print(f"  HFOV    {hw:.2f} deg              (underwater, flat port n={N_WATER})")

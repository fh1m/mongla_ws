#!/usr/bin/env python3
"""Does a monocular depth network on the Hailo-8 know the floor's SCALE?

Flow refuses without the height above the floor (V-9), and a network that
reads height off one image would remove that input (bead po7.17). The only
use is a metric one: a relative depth map cannot turn pixels into metres.
Hailo Model Zoo v2.20 ships three for the Hailo-8: `scdepthv3`, `fast_depth`
(supervised indoor) and `depth_anything_v2_vits` (zero-shot, relative).

There is no height rig in the room tonight, so absolute accuracy cannot be
measured, but a case with exact truth can. Crop the central `1/z` of a floor
frame and resize it back to full size: a flat floor seen from `1/z` of the
height, exactly (pinhole, a fronto-parallel plane). A network that carries
metric scale must report the median depth over the central region scaled by
`1/z`. A network that normalises each image (affine- or scale-invariant)
reports the same number for both.

Bar, written before the first run (bead po7.17: "height within 10 % of truth
or refused"): over the frames scored, for each zoom z in {1.5, 2.0}, the
median over frames of (reported depth at the zoom) / (reported depth of the
original) must lie within 10 % of 1/z. The network may report depth or
inverse depth (disparity), so the inverse ratio is accepted against z. Both
are printed; a network that fails is refused for height. A pass is not
adoption: the bench height (0.72 m taped) and a water clip with a known
distance still have to agree before anything ships, and the Hailo-8 run time
has to leave both detectors at 95 % of their rate.

Falsifier: `--control` replaces the network by one that scales its output to
the unit median per image, which is what an affine-invariant network does.
It must FAIL the bar; if it passes, the check cannot tell scale from none.

Runs on Kutum (the HEF needs the chip); frames are image files:

    python3 tools/studies/depth_scale_check.py --hef scdepthv3.hef frames/*.png
"""
from __future__ import annotations

import argparse
import sys
import time

import cv2
import numpy as np

ZOOMS = (1.5, 2.0)
TOL = 0.10


def zoom(img, z):
    h, w = img.shape[:2]
    ch, cw = int(round(h / z)), int(round(w / z))
    y, x = (h - ch) // 2, (w - cw) // 2
    return cv2.resize(img[y:y + ch, x:x + cw], (w, h), interpolation=cv2.INTER_LINEAR)


def centre_median(d, frac=0.5):
    h, w = d.shape
    y, x = int(h * (1 - frac) / 2), int(w * (1 - frac) / 2)
    c = d[y:h - y, x:w - x]
    return float(np.median(c[np.isfinite(c)]))


class Net:
    def __init__(self, hef):
        from hailo_platform import FormatType, HailoSchedulingAlgorithm, VDevice
        p = VDevice.create_params()
        p.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
        self._vd = VDevice(p)
        self._m = self._vd.create_infer_model(hef)
        self._m.input().set_format_type(FormatType.UINT8)
        self._m.output().set_format_type(FormatType.FLOAT32)
        self._c = self._m.configure()
        self._c.__enter__()
        self._b = self._c.create_bindings()
        self.hw = tuple(self._m.input().shape[:2])
        self._out = np.empty(self._m.output().shape, np.float32)
        self._b.output().set_buffer(self._out)
        self.ms = []

    def __call__(self, bgr):
        x = np.ascontiguousarray(cv2.cvtColor(cv2.resize(bgr, self.hw[::-1]), cv2.COLOR_BGR2RGB))
        self._b.input().set_buffer(x)
        t = time.perf_counter()
        self._c.run([self._b], 1000)
        self.ms.append(1000 * (time.perf_counter() - t))
        return np.array(self._out, np.float32).reshape(self.hw)

    def close(self):
        self._c.__exit__(None, None, None)


def control(net):
    """What an affine-invariant network does: the same map, scaled to unit median."""
    def f(bgr):
        d = net(bgr)
        return d / max(float(np.median(d)), 1e-9)
    return f


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--hef', required=True)
    ap.add_argument('--control', action='store_true')
    ap.add_argument('frames', nargs='+')
    a = ap.parse_args()
    net = Net(a.hef)
    run = control(net) if a.control else net
    ratios = {z: [] for z in ZOOMS}
    try:
        for path in a.frames:
            img = cv2.imread(path)
            if img is None:
                continue
            base = centre_median(run(img))
            if not np.isfinite(base) or base == 0:
                continue
            for z in ZOOMS:
                ratios[z].append(centre_median(run(zoom(img, z))) / base)
    finally:
        net.close()
    n = len(ratios[ZOOMS[0]])
    if n < 8:
        print(f'only {n} usable frames; need 8')
        return 1
    print(f'{a.hef}{" (CONTROL: unit-median scaling)" if a.control else ""}: {n} frames, '
          f'{np.median(net.ms):.1f} ms a call on this machine')
    ok = True
    for z in ZOOMS:
        r = np.array(ratios[z])
        med, lo, hi = np.median(r), *np.percentile(r, (10, 90))
        as_depth = abs(med - 1 / z) <= TOL / z
        as_disparity = abs(med - z) <= TOL * z
        ok &= as_depth or as_disparity
        print(f'  zoom {z}: reported ratio median {med:.3f} (p10 {lo:.3f}, p90 {hi:.3f}); '
              f'depth needs {1 / z:.3f}, disparity needs {z:.3f}; '
              f'{"within 10 % as depth" if as_depth else "within 10 % as disparity" if as_disparity else "neither"}')
    print('VERDICT ' + ('carries metric scale on this test; the bench and water checks decide'
                        if ok else 'REFUSED for height: the scale is not in the output'))
    return 0


if __name__ == '__main__':
    sys.exit(main())

"""ROOT-SIFT as a CONTROL against the recorded XFeat murky-clip table.

The anchor rung's feature backend was chosen on five of our own clips
(`xfeat_onnx.py`, `measured-bars.md` section 8): reference frame snapped at 40 %
of the clip, matched FRAME-TO-REFERENCE at +1/3/5/8 s, `USAC_MAGSAC`
homography at RANSAC_PX, a pass needing MIN_INLIERS.  XFeat read 4/4 on the
three murky Mirpur clips where ORB read 0/4, 0/4, 3/4.

That is an agreement test until somebody runs the literature's best classical
underwater front-end on the SAME frames.  ROOT-SIFT is that front-end, and it is
NOT a candidate here -- it measures 9.6 Hz on the Pi against a 50 Hz consumer
(section 19).  It is here only to answer: if ROOT-SIFT also reads 4/4, then 4/4
says the clips are easy, not that XFeat is special.

THE FINGERPRINT.  The XFeat column cannot be regenerated on this box (no
`xfeat.onnx` here), so the ROOT-SIFT numbers are compared against a recorded
column.  That only counts if the frames match.  ORB is run FIRST, on the same
extracted frames, and must reproduce its recorded reference keypoint counts
(71 / 7 / 111 / 1205 / 1260) and its recorded pass column (0/4 0/4 3/4 1/4 4/4).
If it does not, the harness is on different pixels and no SIFT number below it
means anything.

Frames are seeked by INDEX, not by CAP_PROP_POS_MSEC, which snaps to keyframes.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import cv2
import numpy as np

RANSAC_PX = 3.0          # anchor.py:57
MIN_INLIERS = 15         # "a homography needs ~15 to be trusted"
OFFSETS_S = (1, 3, 5, 8)
REF_FRACTION = 0.40

ARCHIVE = "/home/fh1m/Work/Projects/Duburi/2025/raw_videos"

# name -> (path, recorded ORB reference keypoint count, recorded ORB passes)
CLIPS = {
    "mirpur_torpedo":   (f"{ARCHIVE}/Mirpur/Sun_June_21/torpedo.mkv",           71, 0),
    "mirpur_torpedo_1": (f"{ARCHIVE}/Mirpur/Sun_June_21/torpedo_1.mkv",          7, 0),
    "mirpur_gate":      (f"{ARCHIVE}/Mirpur/Sun_June_21/gate.mkv",             111, 3),
    "octagon":          (f"{ARCHIVE}/robosub/clips/octagon/octagon_1.mp4",    1205, 1),
    "torpedo_clear":    (f"{ARCHIVE}/robosub/clips/torpedo/torpedo_shark_up_1.mp4", 1260, 4),
}


def extract(path: str, resize: tuple[int, int] | None):
    """Return (ref, [frame @ +1s, +3s, +5s, +8s]) as grey arrays, or None."""
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        return None
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    if n <= 0:
        return None
    idx = [int(n * REF_FRACTION)]
    idx += [idx[0] + int(round(fps * s)) for s in OFFSETS_S]
    out = []
    for i in idx:
        if i >= n:
            out.append(None)
            continue
        cap.set(cv2.CAP_PROP_POS_FRAMES, i)
        ok, frame = cap.read()
        if not ok:
            out.append(None)
            continue
        if resize is not None:
            frame = cv2.resize(frame, resize, interpolation=cv2.INTER_AREA)
        out.append(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    cap.release()
    return out


def load_xfeat(model_path: str, threads: int):
    """The SHIPPED anchor backend, not a reimplementation of it.

    Importing mongla_vision.anchor.xfeat_onnx by path avoids the package
    __init__, which drags in ROS. The point of using the production class is
    that whatever this measures is what the vehicle runs -- a bench copy could
    drift from it and read better.
    """
    import importlib.util
    here = os.path.dirname(os.path.abspath(__file__))
    mod_path = os.path.join(here, "..", "src", "mongla_vision", "mongla_vision",
                            "anchor", "xfeat_onnx.py")
    spec = importlib.util.spec_from_file_location("xfeat_onnx", mod_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod, mod.XFeatONNX(model_path, top_k=1024, threads=threads)


def run_pair_xfeat(mod, net, ref_g, cur_g):
    """Same homography test, XFeat correspondences. Returns (ref kp, inliers).

    XFeat resizes internally to the export's fixed shape, so its inliers are
    counted in MODEL pixels while ORB and SIFT are counted at native. Inlier
    COUNT is the comparable quantity here, not pixel geometry.
    """
    k0, d0 = net.detect(ref_g)
    k1, d1 = net.detect(cur_g)
    if len(k0) < 4 or len(k1) < 4:
        return len(k0), 0
    i0, i1 = net.match(d0, d1)
    if len(i0) < 8:
        return len(k0), 0
    src = k0[i0].astype(np.float32).reshape(-1, 1, 2)
    dst = k1[i1].astype(np.float32).reshape(-1, 1, 2)
    H, mask = cv2.findHomography(src, dst, cv2.USAC_MAGSAC, RANSAC_PX)
    if H is None or mask is None:
        return len(k0), 0
    return len(k0), int(mask.sum())


def root_sift(desc: np.ndarray) -> np.ndarray:
    """L1-normalise then sqrt -- Arandjelovic & Zisserman 2012."""
    desc = desc.astype(np.float32)
    desc /= (desc.sum(axis=1, keepdims=True) + 1e-7)
    return np.sqrt(desc)


def run_pair(det, ref_g, cur_g, norm, rootify):
    k0, d0 = det.detectAndCompute(ref_g, None)
    k1, d1 = det.detectAndCompute(cur_g, None)
    if d0 is None or d1 is None or len(k0) < 4 or len(k1) < 4:
        return len(k0) if k0 else 0, 0
    if rootify:
        d0, d1 = root_sift(d0), root_sift(d1)
    # Mutual nearest neighbour, mirroring the anchor's own matcher
    # (xfeat_onnx.py:262 does exactly this on the similarity matrix).
    bf = cv2.BFMatcher(norm, crossCheck=True)
    m = bf.match(d0, d1)
    if len(m) < 8:
        return len(k0), 0
    src = np.float32([k0[x.queryIdx].pt for x in m]).reshape(-1, 1, 2)
    dst = np.float32([k1[x.trainIdx].pt for x in m]).reshape(-1, 1, 2)
    H, mask = cv2.findHomography(src, dst, cv2.USAC_MAGSAC, RANSAC_PX)
    if H is None or mask is None:
        return len(k0), 0
    return len(k0), int(mask.sum())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--resize", default="none",
                    help="none | WxH, e.g. 640x480 or 320x240")
    ap.add_argument("--nfeatures", type=int, default=4096)
    ap.add_argument("--xfeat", default="",
                    help="path to xfeat_<W>x<H>.onnx; adds the shipped anchor "
                         "backend as a third arm on the SAME frames")
    ap.add_argument("--xfeat-threads", type=int, default=1)
    ap.add_argument("--json", default="")
    args = ap.parse_args()

    resize = None
    if args.resize != "none":
        w, h = args.resize.lower().split("x")
        resize = (int(w), int(h))

    cv2.setNumThreads(1)
    orb = cv2.ORB_create(nfeatures=args.nfeatures)
    sift = cv2.SIFT_create(nfeatures=args.nfeatures)
    xmod = xnet = None
    if args.xfeat:
        xmod, xnet = load_xfeat(args.xfeat, args.xfeat_threads)
        print(f"XFeat arm: {os.path.basename(args.xfeat)} "
              f"{xnet.w}x{xnet.h} top_k={xnet.top_k}")

    print(f"resize={args.resize}  nfeatures={args.nfeatures}  "
          f"RANSAC_PX={RANSAC_PX}  pass>={MIN_INLIERS} inliers")
    print(f"{'clip':<18} {'ORB kp':>7} {'rec':>5} {'ORB':>5} {'rec':>5} "
          f"{'SIFT kp':>8} {'R-SIFT':>7} {'XF kp':>6} {'XFeat':>6}  "
          f"sift_inl / xfeat_inl")
    results = {}
    for name, (path, rec_kp, rec_ok) in CLIPS.items():
        frames = extract(path, resize)
        if frames is None or frames[0] is None:
            print(f"{name:<18} UNREADABLE {path}")
            continue
        ref = frames[0]
        orb_kp = orb_ok = sift_kp = sift_ok = xf_kp = xf_ok = 0
        inl = []
        xinl = []
        for cur in frames[1:]:
            if cur is None:
                continue
            k, i = run_pair(orb, ref, cur, cv2.NORM_HAMMING, False)
            orb_kp = max(orb_kp, k)
            orb_ok += int(i >= MIN_INLIERS)
            k, i = run_pair(sift, ref, cur, cv2.NORM_L2, True)
            sift_kp = max(sift_kp, k)
            sift_ok += int(i >= MIN_INLIERS)
            inl.append(i)
            if xnet is not None:
                k, i = run_pair_xfeat(xmod, xnet, ref, cur)
                xf_kp = max(xf_kp, k)
                xf_ok += int(i >= MIN_INLIERS)
                xinl.append(i)
        print(f"{name:<18} {orb_kp:>7} {rec_kp:>5} {orb_ok:>3}/4 {rec_ok:>3}/4 "
              f"{sift_kp:>8} {sift_ok:>5}/4 {xf_kp:>6} {xf_ok:>4}/4  "
              f"{inl} / {xinl}")
        results[name] = dict(orb_kp=orb_kp, recorded_orb_kp=rec_kp,
                             orb_ok=orb_ok, recorded_orb_ok=rec_ok,
                             sift_kp=sift_kp, sift_ok=sift_ok, sift_inliers=inl,
                             xfeat_kp=xf_kp, xfeat_ok=xf_ok, xfeat_inliers=xinl)

    if args.json:
        with open(args.json, "w") as f:
            json.dump(dict(resize=args.resize, nfeatures=args.nfeatures,
                           results=results), f, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())

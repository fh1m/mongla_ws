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
    ap.add_argument("--json", default="")
    args = ap.parse_args()

    resize = None
    if args.resize != "none":
        w, h = args.resize.lower().split("x")
        resize = (int(w), int(h))

    cv2.setNumThreads(1)
    orb = cv2.ORB_create(nfeatures=args.nfeatures)
    sift = cv2.SIFT_create(nfeatures=args.nfeatures)

    print(f"resize={args.resize}  nfeatures={args.nfeatures}  "
          f"RANSAC_PX={RANSAC_PX}  pass>={MIN_INLIERS} inliers")
    print(f"{'clip':<18} {'ORB kp':>7} {'rec':>5} {'ORB':>5} {'rec':>5} "
          f"{'SIFT kp':>8} {'ROOT-SIFT':>10}  inliers")
    results = {}
    for name, (path, rec_kp, rec_ok) in CLIPS.items():
        frames = extract(path, resize)
        if frames is None or frames[0] is None:
            print(f"{name:<18} UNREADABLE {path}")
            continue
        ref = frames[0]
        orb_kp = orb_ok = sift_kp = sift_ok = 0
        inl = []
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
        print(f"{name:<18} {orb_kp:>7} {rec_kp:>5} {orb_ok:>3}/4 {rec_ok:>3}/4 "
              f"{sift_kp:>8} {sift_ok:>8}/4  {inl}")
        results[name] = dict(orb_kp=orb_kp, recorded_orb_kp=rec_kp,
                             orb_ok=orb_ok, recorded_orb_ok=rec_ok,
                             sift_kp=sift_kp, sift_ok=sift_ok, sift_inliers=inl)

    if args.json:
        with open(args.json, "w") as f:
            json.dump(dict(resize=args.resize, nfeatures=args.nfeatures,
                           results=results), f, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
